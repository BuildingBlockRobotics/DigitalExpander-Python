"""Python driver for the BBR Digital Expander.

The expander is an I2C target that adds four encoder inputs, four
colour/distance sensor ports, four digital outputs and (on the odometry
variant) an IMU and a pose estimator to whatever host you plug it into. It was
built for FTC, but nothing about it is FTC-specific: it speaks plain I2C at
100 or 400 kHz and answers to a 7-bit address in 0x38-0x3B.

The API has two tiers, the same two the FTC and Arduino drivers have. The
EVERYDAY tier is one call per question ("how far?", "do I see red?", "how many
counts?") with no bitmasks and no register knowledge. The ADVANCED tier
underneath exposes the full register map for when you outgrow it.

Anything that can fail raises a :class:`~bbr_digital_expander.errors.BBRError`
naming the failure and, where there is one, the fix. The two documented
sentinels are the same as on the other platforms: a distance with nothing in
range is ``math.inf``, and a colour class with no match is 0.

Protocol contract 1.1. Documentation: https://expander.buildingblockrobotics.com/
"""

from __future__ import annotations

import math
import sys
import time
from typing import List, Optional, Sequence, Tuple, Union

from . import regmap as R
from .errors import (
    BadArgumentError,
    BBRError,
    CommandFailedError,
    CommandTimeoutError,
    ImuFaultError,
    LocalizerNotRunningError,
    NoImuError,
    NotInitializedError,
    ProtocolMismatchError,
    TransportError,
    UnknownVariantError,
    WriteRejectedError,
    WrongDeviceError,
    WrongChannelModeError,
    WrongSensorTypeError,
)
from .transport import Bus, SMBusTransport
from .types import (
    AxisUp,
    ChannelMode,
    ColorClass,
    ColorReading,
    DistanceClass,
    DistanceReading,
    EncoderDirection,
    ImuState,
    LocalizerParams,
    LocalizerState,
    LocalizerStatus,
    OutputConfig,
    OutputMode,
    OutputSource,
    Pose,
    SensorReading,
    SensorType,
    Telemetry,
)

__all__ = ["BBRDigitalExpander"]

_ALL_CHANNELS = (1 << R.NUM_ENCODERS) - 1
_ALL_OUTPUTS = (1 << R.NUM_DOUTS) - 1
_TWO_PI = 2.0 * math.pi


# The protocol is little-endian on the wire regardless of what the host is.
def _le16(d: bytes, off: int) -> int:
    return d[off] | (d[off + 1] << 8)


def _sle16(d: bytes, off: int) -> int:
    v = _le16(d, off)
    return v - 0x10000 if v & 0x8000 else v


def _ule32(d: bytes, off: int) -> int:
    return d[off] | (d[off + 1] << 8) | (d[off + 2] << 16) | (d[off + 3] << 24)


def _sle32(d: bytes, off: int) -> int:
    v = _ule32(d, off)
    return v - 0x100000000 if v & 0x80000000 else v


def _put16(buf: bytearray, off: int, v: int) -> None:
    v &= 0xFFFF
    buf[off] = v & 0xFF
    buf[off + 1] = v >> 8


def _put32(buf: bytearray, off: int, v: int) -> None:
    v &= 0xFFFFFFFF
    buf[off] = v & 0xFF
    buf[off + 1] = (v >> 8) & 0xFF
    buf[off + 2] = (v >> 16) & 0xFF
    buf[off + 3] = (v >> 24) & 0xFF


def _round_i32(v: float) -> int:
    """Round half away from zero, as the other two drivers do — so a value on
    a .5 boundary lands on the same integer on every platform."""
    return int(v - 0.5) if v < 0 else int(v + 0.5)


def _wrap_cdeg(cdeg: int) -> int:
    """Fold a heading in centidegrees into the firmware's -18000..17999."""
    return ((cdeg + 18000) % 36000 + 36000) % 36000 - 18000


def _looks_like_failed_read(d: bytes) -> bool:
    """A block that is entirely 0x00 or entirely 0xFF never came from the
    device: the blocks this guards all contain a timestamp, a CRC or a status
    byte with reserved bits, none of which are ever uniform while the board is
    alive. 0x00 is a transfer that returned nothing, 0xFF a bus released
    mid-read."""
    if len(d) < 2 or d[0] not in (0x00, 0xFF):
        return False
    return all(b == d[0] for b in d)


#: A failed-read streak this long AND this old is a real fault, not a
#: program shutting down mid-transfer. Both must be true: a slow loop can
#: take half a second over a couple of reads, a fast one can rattle off five
#: in a few milliseconds while the bus is being released.
_FAIL_STREAK_MIN_READS = 5
_FAIL_STREAK_MIN_MS = 500


def _crc16_profibus(data: bytes) -> int:
    """PROFIBUS CRC16 (poly 0x1DCF, init 0xFFFF, no reflection, xor-out
    0xFFFF) — the same algorithm the OctoQuad uses, so ported verification
    code works unchanged."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1DCF) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc ^ 0xFFFF


class BBRDigitalExpander:
    """A BBR Digital Expander on an I2C bus.

    :param address: 7-bit address, 0x38-0x3B by the address jumpers.
    :param bus: an adapter number (``1`` on a Raspberry Pi), a device path, or
        any object with ``write``/``read``/``close`` methods — see
        :class:`~bbr_digital_expander.transport.Bus`.
    :param chunk_size: split block reads into transfers of at most this many
        bytes. Only needed for an adapter that cannot do a long transfer;
        never changes what the caller sees, because the register pointer is
        written once and the chunks continue from it, so the whole block
        still comes out of one firmware snapshot.

    Usable as a context manager, which closes the bus on the way out::

        with BBRDigitalExpander() as expander:
            print(expander.heading())
    """

    def __init__(
        self,
        address: int = R.I2C_ADDR_DEFAULT,
        bus: Union[int, str, Bus] = 1,
        *,
        chunk_size: Optional[int] = None,
    ) -> None:
        if not R.I2C_ADDR_MIN <= address <= R.I2C_ADDR_MAX:
            raise BadArgumentError(
                f"address must be 0x{R.I2C_ADDR_MIN:02X}-0x{R.I2C_ADDR_MAX:02X} — "
                "it is set by the address jumpers"
            )
        if chunk_size is not None and chunk_size < 1:
            raise BadArgumentError("chunk_size must be 1 or more")
        self._address = address
        self._bus: Bus = (
            bus if hasattr(bus, "read") and hasattr(bus, "write") else SMBusTransport(bus)
        )
        self._chunk = chunk_size

        self._begun = False
        self._capabilities = 0
        self._protocol_minor = 0
        self._fw_version: Tuple[int, int, int] = (0, 0, 0)
        self._hw_variant = 0
        self._token = 0
        self._command_result = R.OK
        self._imu_verified = False

        self._last_read_fresh = True
        self._fail_streak = 0
        self._fail_streak_start = 0.0
        self._last_good_telemetry: Optional[Telemetry] = None
        self._last_good_imu: Optional[ImuState] = None
        self._last_good_localizer: Optional[LocalizerState] = None

        self._telemetry: Optional[Telemetry] = None
        self._telemetry_at = 0.0
        self._telemetry_max_age_ms = 10
        self._channel_mode_mask: Optional[int] = None

    # ------------------------------------------------------------- lifecycle

    def __enter__(self) -> "BBRDigitalExpander":
        self.begin()
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()

    def close(self) -> None:
        """Release the bus. Reading afterwards is an error."""
        self._begun = False
        self._bus.close()

    def begin(self) -> "BBRDigitalExpander":
        """Verify identity and protocol, and return self.

        Refuses to operate on a wrong ``DEVICE_ID``, an unsupported
        ``PROTOCOL_MAJOR`` or an unrecognised ``HW_VARIANT``: a clear failure
        at setup beats corrupt data an hour later.
        """
        self._begun = False
        self._imu_verified = False
        self._telemetry = None
        self._channel_mode_mask = None

        # DEVICE_ID .. STATUS in one read. A glitched or cut-short transaction
        # reads as all zeros or all 0xFF, both transient, so a malformed
        # identity block is retried before concluding the device is the wrong
        # one. Only a stable, well-formed-but-mismatched answer is a refusal.
        d = b""
        for attempt in range(3):
            try:
                d = self.read_registers(R.REG_DEVICE_ID, 9)
            except TransportError:
                # A cable being plugged in, or a board still booting, answers
                # nothing for a moment. Only the last attempt is fatal.
                if attempt == 2:
                    raise
                time.sleep(0.02)
                continue
            if d[R.REG_DEVICE_ID] == R.DEVICE_ID_VALUE and d[R.REG_PROTOCOL_MAJOR] != 0xFF:
                break
            time.sleep(0.02)
        else:
            raise WrongDeviceError(
                f"not a BBR Digital Expander at 0x{self._address:02X} — DEVICE_ID did "
                "not read back as 0xB2"
            )

        if d[R.REG_PROTOCOL_MAJOR] != R.PROTOCOL_MAJOR_VALUE:
            # Registers may have moved or changed meaning: refuse to operate.
            raise ProtocolMismatchError(
                "firmware speaks a protocol major this driver does not — update the "
                "driver or the firmware"
            )
        self._protocol_minor = d[R.REG_PROTOCOL_MINOR]
        self._fw_version = (
            d[R.REG_FW_VERSION_MAJOR],
            d[R.REG_FW_VERSION_MINOR],
            d[R.REG_FW_VERSION_PATCH],
        )
        self._hw_variant = d[R.REG_HW_VARIANT]
        if self._hw_variant not in (R.VARIANT_BASE, R.VARIANT_ODOMETRY):
            # Guessing here risks driving hardware that is not fitted.
            raise UnknownVariantError(
                "unknown hardware variant — firmware older than the board, or a "
                "mis-assembled board"
            )
        self._capabilities = d[R.REG_CAPABILITIES]
        self._begun = True
        return self

    # -------------------------------------------------------------- identity

    @property
    def address(self) -> int:
        return self._address

    @property
    def capabilities(self) -> int:
        return self._capabilities

    @property
    def protocol_minor(self) -> int:
        return self._protocol_minor

    @property
    def firmware_version(self) -> Tuple[int, int, int]:
        """Firmware major, minor, patch, as read by :meth:`begin`."""
        return self._fw_version

    @property
    def hardware_variant(self) -> int:
        return self._hw_variant

    @property
    def is_odometry_variant(self) -> bool:
        return self._hw_variant == R.VARIANT_ODOMETRY

    def has_capability(self, cap_bit: int) -> bool:
        return bool(self._capabilities & cap_bit)

    @property
    def last_command_result(self) -> int:
        """The firmware's ``ERR_*`` code from the last command that reported one."""
        return self._command_result

    def device_info_lines(self) -> List[str]:
        """Identity, variant, capabilities and STATUS bits, one line each."""
        d = self.read_registers(R.REG_DEVICE_ID, 9)
        caps = [
            (R.CAP_ENCODERS, "encoders"),
            (R.CAP_COLOR, "colour"),
            (R.CAP_IMU, "imu"),
            (R.CAP_DOUT, "outputs"),
        ]
        flags = [
            (R.STATUS_READY, "ready"),
            (R.STATUS_CFG_DIRTY, "unsaved-config"),
            (R.STATUS_FLASH_ERROR, "FLASH-ERROR"),
            (R.STATUS_SENSOR_BUS_ERROR, "SENSOR-BUS-ERROR"),
            (R.STATUS_IMU_PRESENT, "imu-present"),
            (R.STATUS_IMU_CALIBRATED, "imu-calibrated"),
            (R.STATUS_CMD_OVERRUN, "command-overrun"),
            (R.STATUS_CFG_DEFAULTED, "config-defaulted"),
        ]
        cap, status = d[R.REG_CAPABILITIES], d[R.REG_STATUS]
        return [
            f"BBR Digital Expander at 0x{self._address:02X}",
            f"  device id   0x{d[R.REG_DEVICE_ID]:02X}",
            f"  firmware    {d[R.REG_FW_VERSION_MAJOR]}.{d[R.REG_FW_VERSION_MINOR]}."
            f"{d[R.REG_FW_VERSION_PATCH]}",
            f"  protocol    {d[R.REG_PROTOCOL_MAJOR]}.{d[R.REG_PROTOCOL_MINOR]}",
            "  variant     "
            + ("odometry (IMU fitted)" if d[R.REG_HW_VARIANT] == R.VARIANT_ODOMETRY else "base"),
            "  can do      " + " ".join(name for bit, name in caps if cap & bit),
            "  status      " + " ".join(name for bit, name in flags if status & bit),
        ]

    def print_device_info(self, file=None) -> None:
        """Print :meth:`device_info_lines` — the first thing to run on a new board."""
        print("\n".join(self.device_info_lines()), file=file or sys.stdout)

    # ------------------------------------------------------------ validation

    @staticmethod
    def _check_range(text: str, v: int, lo: int, hi: int) -> int:
        if not lo <= v <= hi:
            raise BadArgumentError(text)
        return v

    def _require_begun(self) -> None:
        if not self._begun:
            raise NotInitializedError(
                "begin() has not succeeded — construct the expander and call begin() "
                "before reading from it"
            )

    @staticmethod
    def _check_port(port: int) -> int:
        return BBRDigitalExpander._check_range("sensor port must be 0-3", port, 0,
                                               R.NUM_SENSOR_PORTS - 1)

    @staticmethod
    def _check_channel(channel: int) -> int:
        return BBRDigitalExpander._check_range("encoder channel must be 0-3", channel, 0,
                                               R.NUM_ENCODERS - 1)

    @staticmethod
    def _check_output(output: int) -> int:
        return BBRDigitalExpander._check_range("digital output must be 0-3", output, 0,
                                               R.NUM_DOUTS - 1)

    @staticmethod
    def _check_slot(slot: int) -> int:
        return BBRDigitalExpander._check_range("class slot must be 1-7", slot,
                                               R.CLASS_SLOT_FIRST, R.CLASS_SLOT_LAST)

    # ------------------------------------------------------------- transport

    def read_registers(self, reg: int, length: int) -> bytes:
        """Read ``length`` bytes starting at ``reg``.

        Split into chunks when ``chunk_size`` was given, but the pointer is
        written once so the whole block still comes out of a single firmware
        snapshot.
        """
        if length <= 0:
            return b""
        self._bus.write(self._address, bytes([reg]))
        # The firmware latches a snapshot region when the pointer ENTERS it,
        # and the pointer survives a STOP — so the chunks below simply
        # continue from the pointer written above, with no chance of a 32-bit
        # count tearing across the seam.
        limit = self._chunk or length
        out = bytearray()
        while len(out) < length:
            out += self._bus.read(self._address, min(limit, length - len(out)))
        return bytes(out)

    def write_registers(self, reg: int, data: Sequence[int]) -> None:
        """Write ``data`` starting at ``reg``, as one transaction."""
        self._bus.write(self._address, bytes([reg]) + bytes(data))

    def write_register(self, reg: int, value: int) -> None:
        self.write_registers(reg, [value & 0xFF])

    def read_register(self, reg: int) -> int:
        """One register's value."""
        return self.read_registers(reg, 1)[0]

    def _write_verified(self, reg: int, value: int, name: str) -> None:
        # The bus layer never NAKs: an out-of-range write is silently
        # discarded and the old value kept. Write, read back, compare.
        self.write_register(reg, value)
        if self.read_register(reg) != value:
            raise WriteRejectedError(name)

    # -------------------------------------------------------------- commands

    def run_command(self, opcode: int, arg: int = 0, max_ms: int = 10) -> None:
        """Issue a command the way the protocol requires: one 3-byte block
        write (arg, token, opcode), then poll ``COMMAND_STATUS`` bounded by the
        command's maximum duration.

        Every command carries a token the firmware refuses to honour twice, so
        a transport retry can never execute one twice — a reset cannot
        double-zero.
        """
        self._require_begun()

        # Any command can move counts, classes or modes: drop the caches.
        self._telemetry = None
        self._channel_mode_mask = None

        self._token = (self._token + 1) & 0xFF
        if self._token == R.TOKEN_NONE:
            self._token = 1
        self.write_registers(R.REG_COMMAND_ARG, [arg & 0xFF, self._token, opcode])

        # COMMAND_RESULT and COMMAND_ECHO are written before COMMAND_STATUS
        # leaves BUSY, so a status other than BUSY already has its result
        # beside it — one read is enough, no confirming re-read.
        deadline = time.monotonic() + (max_ms + 50) / 1000.0
        while True:
            out = self.read_registers(R.REG_COMMAND_STATUS, 3)
            if out[0] == R.CMDSTAT_DONE:
                self._command_result = out[1]
                return
            if out[0] == R.CMDSTAT_ERROR:
                self._command_result = out[1]
                raise CommandFailedError(
                    "the expander rejected a command — see the command_result on this "
                    f"error (0x{out[1]:02X})",
                    out[1],
                )
            if time.monotonic() > deadline:
                # Overrunning the documented maximum is a fault a retry will
                # not fix, so this is an error rather than a longer wait.
                raise CommandTimeoutError(
                    "command never finished — the expander may have reset"
                )
            time.sleep(0.002)

    def clear_faults(self) -> None:
        self.run_command(R.CMD_CLEAR_FAULTS, 0, 10)

    def save_config_to_flash(self) -> None:
        """Persist all configuration to flash. Setup-time only (up to 500 ms)."""
        self.run_command(R.CMD_CFG_SAVE_FLASH, 0, R.CMD_CFG_SAVE_FLASH_MAX_MS)

    def load_factory_defaults(self) -> None:
        self.run_command(R.CMD_CFG_LOAD_DEFAULTS, 0, 10)

    def device_status(self) -> int:
        """Device ``STATUS`` bits."""
        return self.read_register(R.REG_STATUS)

    def is_config_dirty(self) -> bool:
        """True when configuration changed in RAM has not been saved to flash."""
        return bool(self.device_status() & R.STATUS_CFG_DIRTY)

    def _auto_save_if_dirty(self) -> None:
        """Save if anything changed, riding out the firmware's one-save-per-second
        rate limit. The setup helpers call this so "it worked yesterday, gone
        today" cannot happen to someone who has never heard of flash wear."""
        if not self.is_config_dirty():
            return
        try:
            self.save_config_to_flash()
        except CommandFailedError as exc:
            if exc.command_result != R.ERR_FLASH_RATE_LIMITED:
                raise
            time.sleep(R.FLASH_SAVE_RATE_LIMIT_MS / 1000.0)
            self.save_config_to_flash()

    # ------------------------------------------------------------- telemetry

    # --------------------------------------------------- failed-read policy

    def is_data_fresh(self) -> bool:
        """True when the most recent snapshot read actually reached the
        device. ``False`` means the readers are serving the last good
        snapshot — expected for a moment while a program is shutting the bus
        down, a diagnostic worth showing otherwise."""
        return self._last_read_fresh

    def _handle_failed_read(self, last_good, what: str):
        """A snapshot read never reached the device: serve the last good
        reading rather than raising mid-loop, but stay fail-loud where it
        matters — raise if the device has NEVER answered, or if the streak has
        outlived any plausible shutdown. Same policy as the FTC driver."""
        self._last_read_fresh = False
        now = time.monotonic()
        if self._fail_streak == 0:
            self._fail_streak_start = now
        self._fail_streak += 1
        if last_good is None:
            raise TransportError(
                f"{what} read did not reach the device and it has never answered — "
                "check that it is wired, powered and at the address you gave"
            )
        if (
            self._fail_streak >= _FAIL_STREAK_MIN_READS
            and (now - self._fail_streak_start) * 1000 >= _FAIL_STREAK_MIN_MS
        ):
            raise BBRError(
                f"{what} reads have been failing for over half a second — not a "
                "normal shutdown; check wiring, cable routing and power"
            )
        return last_good

    def _note_good_read(self) -> None:
        self._last_read_fresh = True
        self._fail_streak = 0

    def read_telemetry(self) -> Telemetry:
        """One snapshot of the whole telemetry block."""
        self._require_begun()
        base = R.SNAP_TELEMETRY_START
        d = self.read_registers(base, 96)
        # The timestamp is nonzero from the first millisecond of boot, so a
        # uniform block can only be a transfer that never happened.
        # A uniform block is a transfer that never happened — usually a bus
        # being torn down, not a fault. Serve the last good snapshot; the
        # streak policy above turns a persistent failure into a loud error.
        if _looks_like_failed_read(d):
            return self._handle_failed_read(self._last_good_telemetry, "Telemetry")

        t = Telemetry()
        for i in range(R.NUM_ENCODERS):
            t.encoder_count[i] = _sle32(d, R.REG_ENC0_COUNT - base + 4 * i)
            t.encoder_velocity[i] = _sle32(d, R.REG_ENC0_VELOCITY - base + 4 * i)
        for i in range(R.NUM_SENSOR_PORTS):
            raw = R.RAW_BASE - base + R.RAW_STRIDE * i
            raw_type = d[R.REG_SENSOR_TYPE - base + i]
            s = SensorReading(
                type=SensorType(raw_type) if raw_type in
                (R.STYPE_EMPTY, R.STYPE_COLOR, R.STYPE_DISTANCE) else SensorType.UNKNOWN,
                status=d[R.REG_SENSOR_STATUS - base + i],
                color_class=d[R.REG_S0_CLASS - base + i],
                confidence=d[R.REG_CLASS_CONFIDENCE - base + i],
            )
            if s.type == SensorType.DISTANCE:
                s.distance = DistanceReading(
                    distance_mm=_le16(d, raw + R.RAW_DISTANCE_MM),
                    signal_rate=_le16(d, raw + R.RAW_SIGNAL_RATE),
                    ambient_rate=_le16(d, raw + R.RAW_AMBIENT_RATE),
                    range_status=d[raw + R.RAW_RANGE_STATUS],
                )
            else:
                s.color = ColorReading(
                    red=_le16(d, raw + R.RAW_RED),
                    green=_le16(d, raw + R.RAW_GREEN),
                    blue=_le16(d, raw + R.RAW_BLUE),
                    ir=_le16(d, raw + R.RAW_IR),
                    proximity=_le16(d, raw + R.RAW_PROXIMITY),
                )
            t.sensor[i] = s
        t.present_mask = d[R.REG_SENSOR_PRESENT_MASK - base]
        t.timestamp_micros = _ule32(d, R.REG_TELEMETRY_TIMESTAMP - base)
        self._note_good_read()
        self._last_good_telemetry = t
        return t

    @property
    def telemetry_max_age_ms(self) -> int:
        """How stale the block behind the everyday getters may be before they
        re-read the device. Default 10 ms; 0 disables caching."""
        return self._telemetry_max_age_ms

    @telemetry_max_age_ms.setter
    def telemetry_max_age_ms(self, ms: int) -> None:
        self._telemetry_max_age_ms = self._check_range(
            "telemetry_max_age_ms must be 0-255", ms, 0, 255
        )

    def invalidate_telemetry(self) -> None:
        """Drop the cached telemetry so the next getter reads the device."""
        self._telemetry = None

    def _cached_telemetry(self) -> Telemetry:
        """The snapshot behind the everyday getters. A loop that asks four
        one-value questions costs one bus transaction, not four, and every
        answer within a pass is self-consistent."""
        now = time.monotonic()
        if (
            self._telemetry is not None
            and (now - self._telemetry_at) * 1000.0 <= self._telemetry_max_age_ms
        ):
            return self._telemetry
        self._telemetry = self.read_telemetry()
        self._telemetry_at = time.monotonic()
        return self._telemetry

    def encoder_pin_state(self) -> int:
        """Live encoder input levels: bit 2n is channel n's A line, bit 2n+1
        its B line. Idle inputs read 1 (pull-ups). A bring-up diagnostic — a
        bit that never toggles while the shaft turns is the dead line. Point
        sample per firmware pass, so poll fast and turn slowly."""
        return self.read_register(R.REG_ENC_PIN_STATE)

    # ==================================================================
    # EVERYDAY TIER — one call per question, no bitmasks, no register map.
    # ==================================================================

    def encoder_count(self, channel: int) -> int:
        """Encoder position of ``channel`` (0-3) in counts.

        On a channel in ``PULSE_WIDTH`` mode this is the pulse width instead;
        use :meth:`pulse_width_us` there so the name tells the truth.
        """
        return self._cached_telemetry().encoder_count[self._check_channel(channel)]

    def encoder_velocity(self, channel: int) -> int:
        """Encoder velocity of ``channel`` (0-3), signed counts per second."""
        return self._cached_telemetry().encoder_velocity[self._check_channel(channel)]

    def pulse_width_us(self, channel: int) -> int:
        """Pulse width on ``channel`` (0-3) in microseconds — the absolute
        position of a PWM encoder, multi-turn accumulated when wrap tracking is
        on. 0 means NO SIGNAL: test for it rather than treating it as a
        position."""
        self._check_channel(channel)
        if not self._channel_mode_mask_cached() & (1 << channel):
            raise WrongChannelModeError(
                "that channel is still in quadrature mode — call "
                "set_channel_mode(ch, ChannelMode.PULSE_WIDTH) first"
            )
        return self._cached_telemetry().encoder_count[channel]

    def reset_encoder(self, channel: int) -> None:
        """Zero one encoder channel (0-3)."""
        self.reset_encoders(1 << self._check_channel(channel))

    def reset_all_encoders(self) -> None:
        """Zero all four encoder channels at once."""
        self.reset_encoders(_ALL_CHANNELS)

    def sensor_connected(self, port: int) -> bool:
        """True while a sensor is detected on ``port`` (0-3)."""
        return self._cached_telemetry().sensor_present(self._check_port(port))

    def sensor_type(self, port: int) -> SensorType:
        """What kind of sensor is on ``port`` (0-3); ``EMPTY`` when none."""
        return self._cached_telemetry().sensor[self._check_port(port)].type

    def color_class(self, port: int) -> int:
        """Which taught colour the sensor on ``port`` (0-3) currently sees: a
        slot number 1-7, or 0 for "nothing I was taught".

        Fails safe — a dark, saturated, stale or unplugged sensor reads 0,
        never a confident wrong answer. Teach colours with :meth:`teach_color`.
        """
        self._check_port(port)
        t = self._cached_telemetry()
        self._reject_wrong_sensor_type(
            port, t, SensorType.DISTANCE,
            "that port has a distance sensor but a colour was asked for — check "
            "which port the sensor is plugged into",
        )
        return t.sensor[port].color_class

    def sees_color(self, port: int, class_slot: int) -> bool:
        """True while the sensor on ``port`` sees the colour taught into
        ``class_slot`` (1-7)."""
        self._check_slot(class_slot)
        return self.color_class(port) == class_slot

    def distance_mm(self, port: int) -> float:
        """Distance measured on ``port`` (0-3), in millimetres, or ``math.inf``
        when nothing is in range — so ``distance_mm(p) < 300`` is always safe
        to write. Raises if that port holds a colour sensor.
        """
        self._check_port(port)
        t = self._cached_telemetry()
        self._reject_wrong_sensor_type(
            port, t, SensorType.COLOR,
            "that port has a colour sensor but a distance was asked for — check "
            "which port the sensor is plugged into",
        )
        if not t.distance_valid(port):
            return math.inf
        return float(t.sensor[port].distance.distance_mm)

    @staticmethod
    def _reject_wrong_sensor_type(
        port: int, t: Telemetry, wrong: SensorType, why: str
    ) -> None:
        """Fail when the port definitely holds the wrong kind of sensor — that
        is a wiring mix-up, not a reading, and silently answering would hide
        it."""
        if t.sensor_present(port) and t.sensor[port].type == wrong:
            raise WrongSensorTypeError(why)

    def teach_color(self, sensor_port: int, class_slot: int) -> None:
        """Teach a colour by example: hold the target in front of the sensor on
        ``sensor_port`` (0-3) and call this — the reading becomes colour number
        ``class_slot`` (1-7).

        Takes up to a few seconds, and saves to flash automatically. Refuses
        rather than storing a class that would never — or always — fire: too
        dark, saturated, or no sensor on the port.
        """
        self._check_port(sensor_port)
        self._check_slot(class_slot)
        self._select_sensor_class(sensor_port, class_slot)
        self.run_command(R.CMD_SENSOR_CAPTURE, 0, R.CMD_SENSOR_CAPTURE_MAX_MS)
        self.run_command(R.CMD_CFG_STORE, 0, 10)
        self._auto_save_if_dirty()

    # The trigger helpers below configure the board to watch a condition and
    # drive a digital output when it is met, with no I2C traffic and no code in
    # your loop. Run them ONCE from a setup script: they save to flash, so the
    # board keeps doing it after a power cycle. Your runtime script then just
    # reads the output pin — see the trigger_setup/trigger_runtime pair.

    def trigger_on_color(self, output: int, sensor_port: int, class_slot: int) -> None:
        """Output ``output`` (0-3) goes high while the sensor on
        ``sensor_port`` sees the colour taught into ``class_slot`` (1-7).
        Saved to flash."""
        self._check_output(output)
        self._check_port(sensor_port)
        self._check_slot(class_slot)
        self._attach_output_to_class(output, sensor_port, class_slot)
        self._auto_save_if_dirty()

    def trigger_when_near(self, output: int, sensor_port: int, max_mm: int) -> None:
        """Output ``output`` (0-3) goes high while the distance sensor on
        ``sensor_port`` sees something within ``max_mm``. Saved to flash."""
        self._check_output(output)
        self._check_port(sensor_port)
        self._check_range("max_mm must be 1 or more", max_mm, 1, R.DISTANCE_INVALID - 1)
        # Slot 7 keeps the range window clear of slots 1-6, so it cannot
        # collide with colours someone taught on the same port.
        self.write_distance_class(
            sensor_port,
            R.CLASS_SLOT_LAST,
            DistanceClass(dist_max=max_mm, min_signal_rate=100),
        )
        self._attach_output_to_class(output, sensor_port, R.CLASS_SLOT_LAST)
        self._auto_save_if_dirty()

    def trigger_when_encoder_past(self, output: int, channel: int, counts: int) -> None:
        """Output ``output`` (0-3) goes high while encoder ``channel`` reads
        ``counts`` or more (signed). Saved to flash."""
        self._check_output(output)
        self._check_channel(channel)
        self.configure_output(
            output,
            OutputConfig(
                source=OutputSource.ENCODER,
                src_index=channel,
                thresh_min=counts,
                thresh_max=0x7FFFFFFF,
            ),
        )
        self._auto_save_if_dirty()

    def trigger_when_facing(self, output: int, heading_deg: float, tolerance_deg: float) -> None:
        """Output ``output`` (0-3) goes high while the robot faces
        ``heading_deg`` give or take ``tolerance_deg`` — so (0, 90, 15) holds
        output 0 high between 75 and 105 degrees. The window may straddle
        +/-180; the board handles that. Needs the odometry variant. Saved to
        flash."""
        self._check_output(output)
        if not math.isfinite(heading_deg):
            raise BadArgumentError("heading_deg must be a real number")
        # Written as a positive test so NaN falls through to the failure.
        if not 0.0 < tolerance_deg <= 180.0:
            raise BadArgumentError("tolerance_deg must be more than 0 and at most 180")
        # A heading trigger on a board with no IMU would never fire.
        self._require_imu()

        c = OutputConfig(source=OutputSource.IMU_HEADING)
        tol_cdeg = _round_i32(tolerance_deg * 100.0)
        if tol_cdeg >= 18000:
            # The whole circle. Written as a non-wrapping full-range window so
            # it reads back as obviously always-true rather than as a wrap
            # window one centidegree wide.
            c.thresh_min, c.thresh_max = R.YAW_MIN_CDEG, R.YAW_MAX_CDEG
        else:
            # Fold into (-360, 360) first: a wild heading would otherwise
            # overflow into a valid-looking window pointing somewhere else.
            centre = _wrap_cdeg(_round_i32(math.fmod(heading_deg, 360.0) * 100.0))
            c.thresh_min = _wrap_cdeg(centre - tol_cdeg)
            c.thresh_max = _wrap_cdeg(centre + tol_cdeg)
        self.configure_output(output, c)
        self._auto_save_if_dirty()

    def disable_output(self, output: int) -> None:
        """Turn one output off entirely, so nothing drives it. Saved to flash."""
        self._check_output(output)
        self.configure_output(output, OutputConfig())  # defaults to DISABLED
        self._auto_save_if_dirty()

    def output_state(self) -> int:
        """Live state of the four digital outputs, bit n = output n."""
        return self.read_register(R.REG_DOUT_STATE)

    def output_latched(self) -> int:
        """Which LATCHED outputs are currently latched, bit n = output n."""
        return self.read_register(R.REG_DOUT_LATCHED)

    def clear_output_latch(self, output: int) -> None:
        """Un-latch one digital output (0-3) configured in LATCHED mode."""
        self.clear_output_latches(1 << self._check_output(output))

    def clear_all_output_latches(self) -> None:
        self.clear_output_latches(_ALL_OUTPUTS)

    def heading(self) -> float:
        """Heading in degrees, [-180, 180), positive = turning left. Zero it
        with :meth:`reset_heading`.

        Odometry variant only: a base board raises rather than reporting a
        plausible heading of 0.00 forever.
        """
        return self.read_imu().yaw_deg

    def reset_heading(self) -> None:
        """Zero yaw at the current orientation."""
        self._require_imu()
        self.run_command(R.CMD_IMU_RESET_HEADING, 0, 10)

    def calibrate_gyro(self) -> bool:
        """Re-estimate gyro bias (about a second). Keep the robot still.

        Returns ``True`` with a fresh bias, or ``False`` if the robot moved:
        the previous bias is kept and the IMU carries on, so a bump never
        stops the program. Call again while still if heading drift matters."""
        self._require_imu()
        return self._run_calibration(R.CMD_CALIBRATE_GYRO, R.CMD_CALIBRATE_GYRO_MAX_MS)

    def _run_calibration(self, opcode: int, max_ms: int) -> bool:
        """A calibration that saw the robot move is not worth an exception:
        the firmware keeps the previous bias (and LOC_RESET still resets the
        pose and starts the localizer), so nothing is less usable than
        before."""
        try:
            self.run_command(opcode, 0, max_ms)
        except CommandFailedError as exc:
            if exc.command_result != R.ERR_IMU_NOT_STATIONARY:
                raise
            return False
        return True

    # ==================================================================
    # ADVANCED TIER — the full register map. Nothing below auto-saves;
    # persist explicitly with save_config_to_flash().
    # ==================================================================

    def reset_encoders(self, channel_mask: int) -> None:
        """Zero the encoder channels whose bits are set; prefer
        :meth:`reset_encoder`."""
        self._check_range("channel mask must be 0-15", channel_mask, 0, _ALL_CHANNELS)
        self.run_command(R.CMD_RESET_ENCODERS, channel_mask, 10)

    def _channel_mode_mask_cached(self) -> int:
        if self._channel_mode_mask is None:
            self._channel_mode_mask = self.read_register(R.REG_CHANNEL_MODE)
        return self._channel_mode_mask

    def set_channel_modes(self, pwm_mask: int) -> None:
        """Set all four channel modes at once; bit n set = channel n PULSE_WIDTH."""
        self._check_range("channel mode mask must be 0-15", pwm_mask, 0, _ALL_CHANNELS)
        self._write_verified(R.REG_CHANNEL_MODE, pwm_mask,
                             "the expander refused the channel mode mask")
        self._channel_mode_mask = pwm_mask
        self._telemetry = None

    def set_channel_mode(self, channel: int, mode: ChannelMode) -> None:
        self._check_channel(channel)
        mask = self.read_register(R.REG_CHANNEL_MODE)
        bit = 1 << channel
        self.set_channel_modes(mask | bit if mode == ChannelMode.PULSE_WIDTH else mask & ~bit)

    def get_channel_mode(self, channel: int) -> ChannelMode:
        self._check_channel(channel)
        mask = self.read_register(R.REG_CHANNEL_MODE)
        self._channel_mode_mask = mask
        return ChannelMode.PULSE_WIDTH if mask & (1 << channel) else ChannelMode.QUADRATURE

    def set_pwm_wrap_mask(self, mask: int) -> None:
        self._check_range("wrap mask must be 0-15", mask, 0, _ALL_CHANNELS)
        self._write_verified(
            R.REG_PWM_WRAP_MASK, mask,
            "the expander refused the wrap mask — are the PWM parameters set for "
            "those channels?",
        )

    def set_pwm_wrap_enabled(self, channel: int, enabled: bool) -> None:
        """Multi-turn wrap tracking for one pulse-width channel; needs PWM params."""
        self._check_channel(channel)
        mask = self.read_register(R.REG_PWM_WRAP_MASK)
        bit = 1 << channel
        self.set_pwm_wrap_mask(mask | bit if enabled else mask & ~bit)

    def set_pwm_channel_params(self, channel: int, min_us: int, max_us: int) -> None:
        """Calibrate one pulse-width channel: the pulse widths at the start and
        end of a revolution (e.g. 1 and 1024 for a REV Through Bore absolute
        output). Preserves the other channels. Persist with
        :meth:`save_config_to_flash`."""
        self._check_channel(channel)
        self._check_range("min_us must be 1 or more", min_us, 1, R.PWM_WIDTH_MAX_US - 1)
        self._check_range("max_us must be greater than min_us", max_us, min_us + 1,
                          R.PWM_WIDTH_MAX_US)
        # Read-modify-write: the window holds all four channels' calibration.
        self.run_command(R.CMD_PWM_PARAMS_LOAD, 0, 10)
        w = bytearray(self.read_registers(R.CFGWIN_BASE, R.CFGWIN_SIZE))
        _put16(w, R.PWMWIN_PWM0_MIN_US - R.CFGWIN_BASE + 4 * channel, min_us)
        _put16(w, R.PWMWIN_PWM0_MAX_US - R.CFGWIN_BASE + 4 * channel, max_us)
        self.write_registers(R.CFGWIN_BASE, w)
        self.run_command(R.CMD_PWM_PARAMS_STORE, 0, 10)

    def get_pwm_channel_params(self, channel: int) -> Tuple[int, int]:
        """The ``(min_us, max_us)`` calibration of one pulse-width channel."""
        self._check_channel(channel)
        self.run_command(R.CMD_PWM_PARAMS_LOAD, 0, 10)
        w = self.read_registers(R.CFGWIN_BASE, R.CFGWIN_SIZE)
        return (
            _le16(w, R.PWMWIN_PWM0_MIN_US - R.CFGWIN_BASE + 4 * channel),
            _le16(w, R.PWMWIN_PWM0_MAX_US - R.CFGWIN_BASE + 4 * channel),
        )

    def set_encoder_direction(self, channel: int, direction: EncoderDirection) -> None:
        """Make ``channel`` (0-3) count UP when the thing it measures moves the
        way you consider forward. ``REVERSE`` flips both the count and the
        velocity.

        This is the fix for an encoder that counts backwards, and it belongs
        here rather than in your script: the odometry localizer reads the
        channels straight off the board, so flipping a sign in your own code
        never reaches it. Push the robot and check — driving forward should
        raise the X pod's count, strafing left should raise the Y pod's.

        Saved to flash automatically, so the direction survives a power cycle.
        Setting the same direction repeatedly costs nothing: the firmware
        compares the payload it would write against what is already stored and
        skips the write. Like the other setup helpers, run this once rather
        than in a loop.
        """
        self._check_channel(channel)
        mask = self.get_encoder_invert_mask()
        bit = 1 << channel
        self.set_encoder_invert_mask(
            mask | bit if direction == EncoderDirection.REVERSE else mask & ~bit
        )
        self._auto_save_if_dirty()

    def get_encoder_direction(self, channel: int) -> EncoderDirection:
        """Which way ``channel`` (0-3) currently counts."""
        self._check_channel(channel)
        mask = self.get_encoder_invert_mask()
        return EncoderDirection.REVERSE if mask & (1 << channel) else EncoderDirection.FORWARD

    def set_encoder_invert_mask(self, mask: int) -> None:
        """All four channels at once as a bitmask: bit n set inverts channel n.
        :meth:`set_encoder_direction` says the same thing one channel at a time
        and reads better; this sets several in one transaction."""
        self._check_range("invert mask must be 0-15", mask, 0, _ALL_CHANNELS)
        self._write_verified(R.REG_ENC_INVERT_MASK, mask,
                             "the expander refused the encoder invert mask")

    def get_encoder_invert_mask(self) -> int:
        return self.read_register(R.REG_ENC_INVERT_MASK)

    def set_imu_axis_up(self, axis_up: AxisUp) -> None:
        """Which robot axis points UP, so yaw is integrated about the true
        vertical when the board is not mounted flat. Flat is ``AxisUp.POS_Z``.
        Re-calibrate the IMU after changing it."""
        self._check_range("axis_up must be an AxisUp value", int(axis_up),
                          R.AXIS_POS_X, R.AXIS_NEG_Z)
        self._write_verified(R.REG_IMU_AXIS_UP, int(axis_up),
                             "the expander refused the IMU axis-up setting")

    def get_imu_axis_up(self) -> AxisUp:
        return AxisUp(self.read_register(R.REG_IMU_AXIS_UP))

    def write_color_class(self, port: int, slot: int, c: ColorClass) -> None:
        """Write a colour class window directly. Does not save to flash."""
        self._check_port(port)
        self._check_slot(slot)
        # Chromaticity is normalized to 0-1000 per channel, so a window outside
        # that can never match and an inverted one can never be entered.
        if (
            max(c.r_max, c.g_max, c.b_max) > R.CHROMA_SCALE
            or c.r_min > c.r_max
            or c.g_min > c.g_max
            or c.b_min > c.b_max
        ):
            raise BadArgumentError(
                "colour window bounds must be 0-1000 with min no greater than max"
            )
        if c.prox_max > R.PROX_MAX_VALUE or c.prox_min > c.prox_max:
            raise BadArgumentError(
                "proximity window must be 0-2047 with min no greater than max"
            )
        self._select_sensor_class(port, slot)
        w = bytearray(R.CFGWIN_SIZE)
        w[R.CFGWIN_ENABLED - R.CFGWIN_BASE] = 1
        w[R.CFGWIN_FLAGS - R.CFGWIN_BASE] = R.CLASSF_PROX_GATE if c.proximity_gate else 0
        for off, value in (
            (R.CFGWIN_R_MIN, c.r_min), (R.CFGWIN_R_MAX, c.r_max),
            (R.CFGWIN_G_MIN, c.g_min), (R.CFGWIN_G_MAX, c.g_max),
            (R.CFGWIN_B_MIN, c.b_min), (R.CFGWIN_B_MAX, c.b_max),
            (R.CFGWIN_PROX_MIN, c.prox_min), (R.CFGWIN_PROX_MAX, c.prox_max),
        ):
            _put16(w, off - R.CFGWIN_BASE, value)
        self.write_registers(R.CFGWIN_BASE, w)
        self.run_command(R.CMD_CFG_STORE, 0, 10)

    def write_distance_class(self, port: int, slot: int, c: DistanceClass) -> None:
        """Write a distance class window directly. Does not save to flash."""
        self._check_port(port)
        self._check_slot(slot)
        self._check_range("dist_max must be at least dist_min and below 65535",
                          c.dist_max, c.dist_min, R.DISTANCE_INVALID - 1)
        self._select_sensor_class(port, slot)
        w = bytearray(R.CFGWIN_SIZE)
        w[R.CFGWIN_ENABLED - R.CFGWIN_BASE] = 1
        w[R.CFGWIN_FLAGS - R.CFGWIN_BASE] = 1  # require a valid RANGE_STATUS
        for off, value in (
            (R.CFGWIN_DIST_MIN, c.dist_min), (R.CFGWIN_DIST_MAX, c.dist_max),
            (R.CFGWIN_HYSTERESIS, c.hysteresis_mm),
            (R.CFGWIN_MIN_SIGNAL_RATE, c.min_signal_rate),
        ):
            _put16(w, off - R.CFGWIN_BASE, value)
        self.write_registers(R.CFGWIN_BASE, w)
        self.run_command(R.CMD_CFG_STORE, 0, 10)

    def _select_sensor_class(self, port: int, slot: int) -> None:
        self._write_verified(R.REG_CFG_SENSOR_SELECT, port,
                             "the expander refused the sensor port selection")
        self._write_verified(R.REG_CFG_CLASS_SELECT, slot,
                             "the expander refused the class slot selection")

    # ----------------------------------------------------- digital outputs

    def configure_output(self, output: int, c: OutputConfig) -> None:
        """Configure a digital output in full. Does not save to flash."""
        self._check_output(output)
        if c.source == OutputSource.SENSOR_CLASS:
            self._check_port(c.src_index)
            self._check_slot(c.class_index)
        elif c.source == OutputSource.ENCODER:
            self._check_channel(c.src_index)
        elif c.source == OutputSource.IMU_HEADING:
            # thresh_min > thresh_max is legal here: that is how the firmware
            # is told the window crosses +/-180. Only the ends are checked.
            for v in (c.thresh_min, c.thresh_max):
                self._check_range("heading window must be within +/-180 degrees", v,
                                  R.YAW_MIN_CDEG, R.YAW_MAX_CDEG)
        self._write_verified(R.REG_DOUT_SELECT, output,
                             "the expander refused the output selection")
        w = bytearray(R.DOUTWIN_SIZE)
        w[R.DOUTWIN_SRC_TYPE - R.DOUTWIN_BASE] = int(c.source)
        w[R.DOUTWIN_SRC_INDEX - R.DOUTWIN_BASE] = c.src_index
        w[R.DOUTWIN_CLASS_INDEX - R.DOUTWIN_BASE] = c.class_index
        w[R.DOUTWIN_MODE - R.DOUTWIN_BASE] = (
            (R.DOUT_MODE_ACTIVE_LOW if c.active_low else 0) | (int(c.mode) << 1)
        )
        w[R.DOUTWIN_DEBOUNCE_ASSERT - R.DOUTWIN_BASE] = c.debounce_assert
        w[R.DOUTWIN_DEBOUNCE_RELEASE - R.DOUTWIN_BASE] = c.debounce_release
        w[R.DOUTWIN_PULSE_MS - R.DOUTWIN_BASE] = c.pulse_ms
        _put32(w, R.DOUTWIN_THRESH_MIN - R.DOUTWIN_BASE, c.thresh_min)
        _put32(w, R.DOUTWIN_THRESH_MAX - R.DOUTWIN_BASE, c.thresh_max)
        self.write_registers(R.DOUTWIN_BASE, w)
        self.run_command(R.CMD_DOUT_STORE, 0, 10)

    def _attach_output_to_class(self, output: int, sensor_port: int, class_slot: int) -> None:
        self.configure_output(
            output,
            OutputConfig(
                source=OutputSource.SENSOR_CLASS,
                src_index=sensor_port,
                class_index=class_slot,
            ),
        )

    def clear_output_latches(self, mask: int) -> None:
        """Un-latch the outputs whose bits are set; prefer
        :meth:`clear_output_latch`."""
        self._check_range("output mask must be 0-15", mask, 0, _ALL_OUTPUTS)
        self.run_command(R.CMD_DOUT_CLEAR_LATCH, mask, 10)

    # ------------------------------------------------------------------ IMU

    def _require_imu(self) -> None:
        """All-zero IMU registers would decode to a heading of exactly 0.00
        that never changes — the worst possible failure, because it looks like
        working software. So every IMU path checks first and raises an error
        naming which of "no IMU on this board" and "IMU fitted but broken" it
        is."""
        self._require_begun()
        if self._imu_verified:
            return

        istat = self.read_register(R.REG_IMU_STATUS)
        if istat == 0xFF:
            # A released bus reads 0xFF, which would even set PRESENT and let
            # garbage through. Never treat it as a fitted IMU.
            raise TransportError(
                "IMU status read as 0xFF — a failed transfer, not a fitted IMU"
            )
        fitted = bool(self._capabilities & R.CAP_IMU)
        responding = bool(istat & R.ISTAT_PRESENT)
        if not fitted and not responding:
            raise NoImuError(
                "this board has no IMU (base variant) — heading and odometry need the "
                "odometry variant"
            )
        if fitted and not responding:
            raise ImuFaultError(
                "IMU fitted but not responding — a hardware fault, not a wrong purchase"
            )
        if not fitted:
            raise ImuFaultError(
                "IMU responding on a board not strapped for one — check the variant straps"
            )
        self._imu_verified = True

    def read_imu(self) -> ImuState:
        """One snapshot of the IMU block. Raises on a board whose IMU cannot be
        trusted rather than reporting zeros."""
        # The full check runs until it passes once; after that the
        # fitted/absent question cannot change, and the status byte inside the
        # snapshot below still guards every read — so heading() in a loop is
        # one transaction.
        self._require_imu()

        base = R.SNAP_IMU_START
        d = self.read_registers(base, 32)
        if _looks_like_failed_read(d):
            return self._handle_failed_read(self._last_good_imu, "IMU")
        s = ImuState(status=d[R.REG_IMU_STATUS - base])
        if not s.status & R.ISTAT_PRESENT:
            self._imu_verified = False  # the IMU vanished after verification
            raise ImuFaultError(
                "the IMU stopped answering mid-run — check power and cable routing"
            )
        s.quat_w = _sle16(d, R.REG_QUAT_W - base) / R.QUAT_SCALE
        s.quat_x = _sle16(d, R.REG_QUAT_X - base) / R.QUAT_SCALE
        s.quat_y = _sle16(d, R.REG_QUAT_Y - base) / R.QUAT_SCALE
        s.quat_z = _sle16(d, R.REG_QUAT_Z - base) / R.QUAT_SCALE
        s.yaw_deg = _sle16(d, R.REG_YAW - base) / R.ANGLE_SCALE_CDEG
        s.pitch_deg = _sle16(d, R.REG_PITCH - base) / R.ANGLE_SCALE_CDEG
        s.roll_deg = _sle16(d, R.REG_ROLL - base) / R.ANGLE_SCALE_CDEG
        s.gyro_dps = [
            _sle16(d, R.REG_GYRO_X - base + 2 * i) / R.GYRO_SCALE_LSB_PER_DPS for i in range(3)
        ]
        s.accel_mps2 = [
            _sle16(d, R.REG_ACCEL_X - base + 2 * i) / R.ACCEL_SCALE_LSB_PER_MPS2
            for i in range(3)
        ]
        s.calib_grade = d[R.REG_IMU_CALIB - base]
        s.timestamp_micros = _ule32(d, R.REG_IMU_TIMESTAMP - base)
        self._note_good_read()
        self._last_good_imu = s
        return s

    # ------------------------------------------------------------ localizer

    def read_localizer_raw(self) -> LocalizerState:
        """Localizer snapshot in any state; check ``status`` yourself."""
        self._require_begun()
        base = R.SNAP_LOCALIZER_START
        crc_off = R.REG_LOC_CRC16 - base

        # I2C acknowledges bytes without verifying them, so a noise-flipped bit
        # would otherwise arrive looking like a perfectly valid pose. Re-read
        # on mismatch, then serve the last good pose rather than a suspect one.
        for attempt in range(1, 4):
            d = self.read_registers(base, 22)
            if _looks_like_failed_read(d):
                return self._handle_failed_read(self._last_good_localizer, "Localizer")
            if _crc16_profibus(d[:20]) == _le16(d, crc_off):
                break
            # Three corrupt reads in a row is a bad moment on the bus, not
            # proof the bus is gone: a burst of noise mid-run would otherwise
            # kill the program outright. Treat it as a failed read and let the
            # streak policy escalate if the corruption actually persists.
            if attempt == 3:
                return self._handle_failed_read(self._last_good_localizer, "Localizer")

        raw_status = d[R.REG_LOC_STATUS - base]
        try:
            status = LocalizerStatus(raw_status)
        except ValueError:
            status = LocalizerStatus.UNKNOWN
        state = LocalizerState(
            status=status,
            flags=d[R.REG_LOC_FLAGS - base],
            vel_x_mm_per_sec=float(_sle16(d, R.REG_LOC_VEL_X - base)),
            vel_y_mm_per_sec=float(_sle16(d, R.REG_LOC_VEL_Y - base)),
            heading_vel_rad_per_sec=_sle16(d, R.REG_LOC_VEL_H - base) / R.LOC_HEADING_VEL_SCALE,
            x_mm=float(_sle16(d, R.REG_LOC_X - base)),
            y_mm=float(_sle16(d, R.REG_LOC_Y - base)),
            heading_rad=_sle16(d, R.REG_LOC_H - base) / R.LOC_HEADING_SCALE,
            timestamp_micros=_ule32(d, R.REG_LOC_TIMESTAMP - base),
        )
        self._note_good_read()
        self._last_good_localizer = state
        return state

    def read_localizer(self) -> LocalizerState:
        """Localizer snapshot, failing unless it is RUNNING."""
        s = self.read_localizer_raw()
        # Same fail-loud contract as the IMU: a localizer stuck at (0, 0, 0)
        # looks exactly like working software, so anything short of RUNNING
        # raises.
        if s.status == LocalizerStatus.RUNNING:
            return s
        if s.status == LocalizerStatus.FAULT_NO_IMU:
            self._require_imu()  # produces the specific base-variant/fault message
            raise ImuFaultError("localizer faulted: the IMU is unavailable")
        if s.status == LocalizerStatus.NOT_READY:
            raise LocalizerNotRunningError(
                "localizer not started — call reset_localizer_and_calibrate_imu()"
            )
        if s.status in (LocalizerStatus.WARMING_UP_IMU, LocalizerStatus.CALIBRATING_IMU):
            raise LocalizerNotRunningError(
                "localizer still starting up — call wait_for_localizer_ready() before "
                "reading a pose"
            )
        raise LocalizerNotRunningError(
            "localizer in an unknown state — firmware newer than this driver?"
        )

    def get_pose(self) -> Pose:
        """Current pose. Fails unless the localizer is RUNNING."""
        return self.read_localizer().pose

    def set_localizer_params(self, p: LocalizerParams) -> None:
        self._check_channel(p.port_x)
        self._check_channel(p.port_y)
        if p.port_x == p.port_y:
            raise BadArgumentError(
                "port_x and port_y are the same channel — the X and Y pods need "
                "different encoder channels"
            )
        # Wire unit is ticks per metre, so a millimetre figure scales by 1000.
        tpm_x = _round_i32(p.ticks_per_mm_x * 1000.0)
        tpm_y = _round_i32(p.ticks_per_mm_y * 1000.0)
        self._check_range("ticks_per_mm_x is out of range — measure it, do not guess",
                          tpm_x, R.LOC_TPM_MIN, R.LOC_TPM_MAX)
        self._check_range("ticks_per_mm_y is out of range — measure it, do not guess",
                          tpm_y, R.LOC_TPM_MIN, R.LOC_TPM_MAX)
        scalar = _round_i32(p.imu_scalar * R.LOC_IMU_SCALAR_SCALE)
        self._check_range("imu_scalar must be between 0.8 and 1.2", scalar,
                          R.LOC_IMU_SCALAR_MIN, R.LOC_IMU_SCALAR_MAX)
        self._check_range("velocity_interval_ms must be 1-255", p.velocity_interval_ms, 1, 255)

        w = bytearray(R.CFGWIN_SIZE)
        _put32(w, R.LOCWIN_TPM_X - R.CFGWIN_BASE, tpm_x)
        _put32(w, R.LOCWIN_TPM_Y - R.CFGWIN_BASE, tpm_y)
        _put16(w, R.LOCWIN_TCP_X - R.CFGWIN_BASE, _round_i32(p.tcp_offset_x_mm))
        _put16(w, R.LOCWIN_TCP_Y - R.CFGWIN_BASE, _round_i32(p.tcp_offset_y_mm))
        _put16(w, R.LOCWIN_IMU_SCALAR - R.CFGWIN_BASE, scalar)
        w[R.LOCWIN_PORT_X - R.CFGWIN_BASE] = p.port_x
        w[R.LOCWIN_PORT_Y - R.CFGWIN_BASE] = p.port_y
        w[R.LOCWIN_VEL_INTERVAL_MS - R.CFGWIN_BASE] = p.velocity_interval_ms
        self.write_registers(R.CFGWIN_BASE, w)
        self.run_command(R.CMD_LOC_PARAMS_STORE, 0, 10)

    def get_localizer_params(self) -> LocalizerParams:
        self.run_command(R.CMD_LOC_PARAMS_LOAD, 0, 10)
        w = self.read_registers(R.CFGWIN_BASE, R.CFGWIN_SIZE)
        return LocalizerParams(
            ticks_per_mm_x=_sle32(w, R.LOCWIN_TPM_X - R.CFGWIN_BASE) / 1000.0,
            ticks_per_mm_y=_sle32(w, R.LOCWIN_TPM_Y - R.CFGWIN_BASE) / 1000.0,
            tcp_offset_x_mm=float(_sle16(w, R.LOCWIN_TCP_X - R.CFGWIN_BASE)),
            tcp_offset_y_mm=float(_sle16(w, R.LOCWIN_TCP_Y - R.CFGWIN_BASE)),
            imu_scalar=_le16(w, R.LOCWIN_IMU_SCALAR - R.CFGWIN_BASE) / R.LOC_IMU_SCALAR_SCALE,
            port_x=w[R.LOCWIN_PORT_X - R.CFGWIN_BASE],
            port_y=w[R.LOCWIN_PORT_Y - R.CFGWIN_BASE],
            velocity_interval_ms=w[R.LOCWIN_VEL_INTERVAL_MS - R.CFGWIN_BASE],
        )

    def reset_localizer_and_calibrate_imu(self) -> bool:
        """Latch stored parameters, zero the pose and calibrate the gyro (about
        a second). Keep the robot completely still. Follow with
        :meth:`wait_for_localizer_ready` before reading a pose.

        Returns ``True`` with a fresh gyro bias, or ``False`` if the robot
        moved during calibration. Either way the pose is reset and the
        localizer starts; ``False`` only means the previous bias is kept.
        Bad localizer parameters still raise."""
        self._require_imu()
        return self._run_calibration(R.CMD_LOC_RESET, R.CMD_LOC_RESET_MAX_MS)

    def wait_for_localizer_ready(self, timeout_ms: int = 5000) -> None:
        """Block until the localizer reaches RUNNING; keep the robot still.
        Raises on timeout, or on a state waiting cannot fix."""
        deadline = time.monotonic() + timeout_ms / 1000.0
        while True:
            s = self.read_localizer_raw()
            if s.status == LocalizerStatus.RUNNING:
                return
            if s.status == LocalizerStatus.FAULT_NO_IMU:
                self._require_imu()
                raise ImuFaultError("localizer faulted: the IMU is unavailable")
            if s.status == LocalizerStatus.NOT_READY:
                raise LocalizerNotRunningError(
                    "localizer not started — call reset_localizer_and_calibrate_imu() first"
                )
            if time.monotonic() > deadline:
                raise CommandTimeoutError(
                    "the localizer did not become ready in time — was the robot moving?"
                )
            time.sleep(0.05)  # warming up or calibrating: keep waiting

    def set_pose(self, x_mm: float, y_mm: float, heading_rad: float) -> None:
        """Teleport the reported pose — the relocalization primitive after a
        vision fix or a wall touch. The heading is normalized for you."""
        self._check_range("pose X is outside +/-32767 mm", _round_i32(x_mm), -32768, 32767)
        self._check_range("pose Y is outside +/-32767 mm", _round_i32(y_mm), -32768, 32767)
        # Normalize into [-pi, pi): fmod lands in (-2pi, 2pi), then fold the
        # ends so exactly +pi from caller code is accepted as -pi.
        h = math.fmod(heading_rad, _TWO_PI)
        if h >= math.pi:
            h -= _TWO_PI
        if h < -math.pi:
            h += _TWO_PI

        w = bytearray(6)
        _put16(w, R.LOCWIN_POSE_X - R.CFGWIN_BASE, _round_i32(x_mm))
        _put16(w, R.LOCWIN_POSE_Y - R.CFGWIN_BASE, _round_i32(y_mm))
        _put16(w, R.LOCWIN_POSE_H - R.CFGWIN_BASE, _round_i32(h * R.LOC_HEADING_SCALE))
        # Written immediately before the command on purpose: the config window
        # is shared between overlays and LOC_SET_POSE reads whatever is there
        # now.
        self.write_registers(R.CFGWIN_BASE, w)
        self.run_command(R.CMD_LOC_SET_POSE, 0, 10)
