"""How the driver reaches the bus.

The expander is an ordinary I2C target, so the transport is deliberately
tiny: write some bytes, read some bytes. Anything implementing :class:`Bus`
will do — the default is Linux ``/dev/i2c-*`` through ``smbus2``, which is
what a Raspberry Pi has, and the test suite passes a fake in its place.

Reads go through ``i2c_rdwr`` raw messages rather than the SMBus block-read
calls on purpose: SMBus block transfers are capped at 32 bytes, and the
telemetry block is 96. A raw read has no such cap, so one snapshot is one
transaction.
"""

from __future__ import annotations

from typing import Protocol, Union

from .errors import TransportError

__all__ = ["Bus", "SMBusTransport"]


class Bus(Protocol):
    """The whole interface a transport has to provide."""

    def write(self, address: int, data: bytes) -> None:
        """Write ``data`` to the 7-bit ``address``, as one transaction."""

    def read(self, address: int, length: int) -> bytes:
        """Read ``length`` bytes from the 7-bit ``address``, as one transaction."""

    def close(self) -> None:
        """Release the underlying device, if this transport owns one."""


class SMBusTransport:
    """``smbus2`` over a Linux I2C character device.

    :param bus: an adapter number (``1`` for a Raspberry Pi's header pins), a
        device path (``"/dev/i2c-1"``), or an already-open ``SMBus`` object,
        which is then not closed by this class.
    """

    def __init__(self, bus: Union[int, str, "object"] = 1) -> None:
        try:
            from smbus2 import SMBus, i2c_msg
        except ImportError as exc:  # pragma: no cover - environment-dependent
            raise TransportError(
                "smbus2 is not installed — run 'pip install smbus2', and on a "
                "Raspberry Pi enable I2C with 'sudo raspi-config' first"
            ) from exc

        self._i2c_msg = i2c_msg
        # An SMBus handed in belongs to the caller; one opened here does not.
        self._external = not isinstance(bus, (int, str))
        if self._external:
            self._bus = bus
        else:
            try:
                self._bus = SMBus(bus)
            except OSError as exc:
                raise TransportError(
                    f"could not open I2C bus {bus!r} — is I2C enabled, and is this "
                    f"user in the 'i2c' group? ({exc})"
                ) from exc

    def write(self, address: int, data: bytes) -> None:
        msg = self._i2c_msg.write(address, data)
        try:
            self._bus.i2c_rdwr(msg)
        except OSError as exc:
            raise TransportError(
                "the expander did not acknowledge a write — check wiring, power "
                f"and the address jumpers ({exc})"
            ) from exc

    def read(self, address: int, length: int) -> bytes:
        msg = self._i2c_msg.read(address, length)
        try:
            self._bus.i2c_rdwr(msg)
        except OSError as exc:
            raise TransportError(
                "the expander did not answer a read — check wiring, power and the "
                f"address jumpers ({exc})"
            ) from exc
        data = bytes(bytearray(list(msg)))
        if len(data) != length:
            raise TransportError(
                "short I2C read — the expander stopped answering mid-block"
            )
        return data

    def close(self) -> None:
        if not self._external:
            self._bus.close()
