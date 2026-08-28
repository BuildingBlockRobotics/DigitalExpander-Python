"""Failure statuses and the exceptions that carry them.

The Arduino driver returns ``false`` and leaves the reason in
``lastStatus()``; the FTC driver throws. Python throws, like the FTC one — a
call that silently returned ``None`` in a loop would read as working software.
The status codes and the sentences beside them are the same on all three, so a
problem someone hits on one platform is searchable from any of the others.
"""

from enum import IntEnum

__all__ = [
    "Status",
    "BBRError",
    "NotInitializedError",
    "TransportError",
    "WrongDeviceError",
    "ProtocolMismatchError",
    "UnknownVariantError",
    "BadArgumentError",
    "WriteRejectedError",
    "CommandFailedError",
    "CommandTimeoutError",
    "NoImuError",
    "ImuFaultError",
    "WrongSensorTypeError",
    "WrongChannelModeError",
    "CrcMismatchError",
    "LocalizerNotRunningError",
]


class Status(IntEnum):
    """Why the last call failed. ``OK`` means it did not."""

    OK = 0
    NOT_INITIALIZED = 1  #: begin() has not run, or it failed
    TRANSPORT = 2  #: the device did not answer, or answered short
    WRONG_DEVICE = 3  #: DEVICE_ID is not a BBR Digital Expander
    PROTOCOL_MISMATCH = 4  #: PROTOCOL_MAJOR is not one this driver speaks
    UNKNOWN_VARIANT = 5  #: HW_VARIANT is not a build this driver knows
    BAD_ARGUMENT = 6  #: a port, channel, slot or value was out of range
    WRITE_REJECTED = 7  #: the device kept its old value (out of range?)
    COMMAND_FAILED = 8  #: the firmware ran the command and reported an error
    COMMAND_TIMEOUT = 9  #: the command never left BUSY
    NO_IMU = 10  #: base variant: there is no IMU on this board
    IMU_FAULT = 11  #: an IMU is fitted (or strapped for) but not answering
    WRONG_SENSOR_TYPE = 12  #: that port holds the other kind of sensor
    WRONG_CHANNEL_MODE = 13  #: quadrature call on a pulse-width channel, or vice versa
    CRC_MISMATCH = 14  #: the localizer block failed CRC16 three times
    LOCALIZER_NOT_RUNNING = 15  #: pose read before the localizer was ready


class BBRError(Exception):
    """Base class for every failure this driver reports.

    ``except BBRError`` catches the lot; the subclasses below let you catch
    one kind. ``status`` is the machine-readable form of the same thing.
    """

    status = Status.OK

    def __init__(self, message: str) -> None:
        super().__init__(message)
        self.message = message


class NotInitializedError(BBRError):
    status = Status.NOT_INITIALIZED


class TransportError(BBRError):
    status = Status.TRANSPORT


class WrongDeviceError(BBRError):
    status = Status.WRONG_DEVICE


class ProtocolMismatchError(BBRError):
    status = Status.PROTOCOL_MISMATCH


class UnknownVariantError(BBRError):
    status = Status.UNKNOWN_VARIANT


class BadArgumentError(BBRError, ValueError):
    """Also a ``ValueError``: a port or slot out of range is a caller bug."""

    status = Status.BAD_ARGUMENT


class WriteRejectedError(BBRError):
    status = Status.WRITE_REJECTED


class CommandFailedError(BBRError):
    """The firmware ran the command and refused it.

    ``command_result`` is the firmware's ``ERR_*`` code — the specific
    reason, where the message is the general one.
    """

    status = Status.COMMAND_FAILED

    def __init__(self, message: str, command_result: int = 0) -> None:
        super().__init__(message)
        self.command_result = command_result


class CommandTimeoutError(BBRError):
    status = Status.COMMAND_TIMEOUT


class NoImuError(BBRError):
    status = Status.NO_IMU


class ImuFaultError(BBRError):
    status = Status.IMU_FAULT


class WrongSensorTypeError(BBRError):
    status = Status.WRONG_SENSOR_TYPE


class WrongChannelModeError(BBRError):
    status = Status.WRONG_CHANNEL_MODE


class CrcMismatchError(BBRError):
    status = Status.CRC_MISMATCH


class LocalizerNotRunningError(BBRError):
    status = Status.LOCALIZER_NOT_RUNNING
