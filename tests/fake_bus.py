"""A fake expander on a fake bus, enough of one to test the driver against.

It is a flat 256-byte register file plus the parts of the protocol the driver
actually depends on: a pointer that survives a STOP, commands that complete,
and a CRC over the localizer block. It is not a firmware simulator — where the
real board would validate a value, this one usually accepts it.
"""

from __future__ import annotations

from typing import List

from bbr_digital_expander import regmap as R
from bbr_digital_expander.expander import _crc16_profibus


class FakeExpanderBus:
    """Bus-shaped object holding one fake expander."""

    def __init__(self, address: int = R.I2C_ADDR_DEFAULT, variant: int = R.VARIANT_ODOMETRY):
        self.address = address
        self.regs = bytearray(256)
        self.pointer = 0
        self.closed = False
        self.commands: List[tuple] = []  #: (opcode, arg, token), in order
        self.writes: List[tuple] = []  #: (reg, bytes), in order
        self.command_errors: dict = {}  #: opcode -> ERR_* code to fail it with

        self.regs[R.REG_DEVICE_ID] = R.DEVICE_ID_VALUE
        self.regs[R.REG_FW_VERSION_MAJOR] = 1
        self.regs[R.REG_FW_VERSION_MINOR] = 2
        self.regs[R.REG_FW_VERSION_PATCH] = 3
        self.regs[R.REG_PROTOCOL_MAJOR] = R.PROTOCOL_MAJOR_VALUE
        self.regs[R.REG_PROTOCOL_MINOR] = R.PROTOCOL_MINOR_VALUE
        self.regs[R.REG_HW_VARIANT] = variant
        self.regs[R.REG_CAPABILITIES] = R.CAP_ENCODERS | R.CAP_COLOR | R.CAP_DOUT | (
            R.CAP_IMU if variant == R.VARIANT_ODOMETRY else 0
        )
        self.regs[R.REG_STATUS] = R.STATUS_READY | (
            R.STATUS_IMU_PRESENT if variant == R.VARIANT_ODOMETRY else 0
        )
        if variant == R.VARIANT_ODOMETRY:
            self.regs[R.REG_IMU_STATUS] = R.ISTAT_PRESENT | R.ISTAT_FUSION_VALID
        # Nonzero from the first millisecond of boot, like the real thing.
        self.set_u32(R.REG_TELEMETRY_TIMESTAMP, 1234)
        self.set_u32(R.REG_IMU_TIMESTAMP, 1234)
        self.set_u32(R.REG_LOC_TIMESTAMP, 1234)
        self.restamp_localizer_crc()

    # ------------------------------------------------------------- helpers

    def set_u16(self, reg: int, value: int) -> None:
        self.regs[reg] = value & 0xFF
        self.regs[reg + 1] = (value >> 8) & 0xFF

    def set_i16(self, reg: int, value: int) -> None:
        self.set_u16(reg, value & 0xFFFF)

    def set_u32(self, reg: int, value: int) -> None:
        for i in range(4):
            self.regs[reg + i] = (value >> (8 * i)) & 0xFF

    def set_i32(self, reg: int, value: int) -> None:
        self.set_u32(reg, value & 0xFFFFFFFF)

    def restamp_localizer_crc(self) -> None:
        base = R.SNAP_LOCALIZER_START
        self.set_u16(R.REG_LOC_CRC16, _crc16_profibus(bytes(self.regs[base:base + 20])))

    # ----------------------------------------------------------- bus shape

    def write(self, address: int, data: bytes) -> None:
        assert address == self.address, "write to the wrong address"
        assert not self.closed, "write after close"
        self.pointer = data[0]
        if len(data) == 1:
            return
        payload = data[1:]
        self.writes.append((self.pointer, bytes(payload)))
        for i, b in enumerate(payload):
            self.regs[(self.pointer + i) & 0xFF] = b
        if self.pointer <= R.REG_COMMAND < self.pointer + len(payload):
            self._run_command()

    def read(self, address: int, length: int) -> bytes:
        assert address == self.address, "read from the wrong address"
        assert not self.closed, "read after close"
        out = bytes(self.regs[self.pointer:self.pointer + length])
        self.pointer = (self.pointer + length) & 0xFF
        return out

    def close(self) -> None:
        self.closed = True

    # ------------------------------------------------------------ commands

    def _run_command(self) -> None:
        opcode = self.regs[R.REG_COMMAND]
        arg = self.regs[R.REG_COMMAND_ARG]
        self.commands.append((opcode, arg, self.regs[R.REG_COMMAND_TOKEN]))
        self.regs[R.REG_COMMAND_ECHO] = opcode
        if opcode in self.command_errors:
            self.regs[R.REG_COMMAND_RESULT] = self.command_errors[opcode]
            self.regs[R.REG_COMMAND_STATUS] = R.CMDSTAT_ERROR
            return
        self.regs[R.REG_COMMAND_RESULT] = R.OK
        self.regs[R.REG_COMMAND_STATUS] = R.CMDSTAT_DONE
        if opcode == R.CMD_CFG_SAVE_FLASH:
            self.regs[R.REG_STATUS] &= ~R.STATUS_CFG_DIRTY & 0xFF
        elif opcode in (R.CMD_CFG_STORE, R.CMD_DOUT_STORE, R.CMD_LOC_PARAMS_STORE,
                        R.CMD_PWM_PARAMS_STORE):
            self.regs[R.REG_STATUS] |= R.STATUS_CFG_DIRTY
        elif opcode == R.CMD_RESET_ENCODERS:
            for ch in range(R.NUM_ENCODERS):
                if arg & (1 << ch):
                    self.set_i32(R.REG_ENC0_COUNT + 4 * ch, 0)
