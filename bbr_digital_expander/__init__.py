"""BBR Digital Expander — Python driver.

Four encoder inputs, four colour/distance sensor ports, four condition-driven
digital outputs, an IMU and an onboard odometry localizer, over one I2C
connection. Built for FTC, but the board is an ordinary I2C target — this
package drives it from a Raspberry Pi, or from anything else running Linux
with an I2C bus.

    from bbr_digital_expander import BBRDigitalExpander

    with BBRDigitalExpander() as expander:
        expander.print_device_info()
        print(expander.encoder_count(0), expander.distance_mm(1))

Documentation: https://expander.buildingblockrobotics.com/
"""

from . import regmap
from .errors import (
    BBRError,
    BadArgumentError,
    CommandFailedError,
    CommandTimeoutError,
    CrcMismatchError,
    ImuFaultError,
    LocalizerNotRunningError,
    NoImuError,
    NotInitializedError,
    ProtocolMismatchError,
    Status,
    TransportError,
    UnknownVariantError,
    WriteRejectedError,
    WrongChannelModeError,
    WrongDeviceError,
    WrongSensorTypeError,
)
from .expander import BBRDigitalExpander
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

__version__ = "1.0.0"

__all__ = [
    "BBRDigitalExpander",
    "Bus",
    "SMBusTransport",
    "regmap",
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
    "SensorType",
    "ChannelMode",
    "EncoderDirection",
    "OutputSource",
    "OutputMode",
    "AxisUp",
    "LocalizerStatus",
    "ColorReading",
    "DistanceReading",
    "SensorReading",
    "Telemetry",
    "ImuState",
    "Pose",
    "LocalizerState",
    "LocalizerParams",
    "ColorClass",
    "DistanceClass",
    "OutputConfig",
    "__version__",
]
