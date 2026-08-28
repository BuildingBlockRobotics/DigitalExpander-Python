"""The values the driver hands back, and the ones you hand it.

Everything here is a plain dataclass or an ``IntEnum`` — no bit fiddling
required at the call site, and every field carries the unit in its name.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Optional

from . import regmap as R

__all__ = [
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
]


class SensorType(IntEnum):
    EMPTY = R.STYPE_EMPTY
    COLOR = R.STYPE_COLOR
    DISTANCE = R.STYPE_DISTANCE
    UNKNOWN = R.STYPE_UNKNOWN


class ChannelMode(IntEnum):
    QUADRATURE = 0
    PULSE_WIDTH = 1


class EncoderDirection(IntEnum):
    """Which way an encoder channel counts."""

    FORWARD = 0
    REVERSE = 1


class OutputSource(IntEnum):
    """What drives a digital output."""

    DISABLED = R.DOUT_SRC_DISABLED
    SENSOR_CLASS = R.DOUT_SRC_SENSOR_CLASS
    ENCODER = R.DOUT_SRC_ENCODER
    IMU_HEADING = R.DOUT_SRC_IMU_HEADING


class OutputMode(IntEnum):
    """How a digital output behaves once its source matches."""

    LEVEL = R.DOUT_MODE_LEVEL
    LATCHED = R.DOUT_MODE_LATCHED
    PULSE = R.DOUT_MODE_PULSE


class AxisUp(IntEnum):
    """Which robot axis points up. Flat on the chassis is ``POS_Z``."""

    POS_X = R.AXIS_POS_X
    NEG_X = R.AXIS_NEG_X
    POS_Y = R.AXIS_POS_Y
    NEG_Y = R.AXIS_NEG_Y
    POS_Z = R.AXIS_POS_Z
    NEG_Z = R.AXIS_NEG_Z


class LocalizerStatus(IntEnum):
    NOT_READY = R.LOC_NOT_READY
    WARMING_UP_IMU = R.LOC_WARMING_UP_IMU
    CALIBRATING_IMU = R.LOC_CALIBRATING_IMU
    RUNNING = R.LOC_RUNNING
    FAULT_NO_IMU = R.LOC_FAULT_NO_IMU
    UNKNOWN = 0xFF


@dataclass
class ColorReading:
    """Rescaled 16-bit channels from a colour sensor."""

    red: int = 0
    green: int = 0
    blue: int = 0
    ir: int = 0
    proximity: int = 0


@dataclass
class DistanceReading:
    distance_mm: int = 0  #: ``DISTANCE_INVALID`` (0xFFFF) when out of range
    signal_rate: int = 0
    ambient_rate: int = 0
    range_status: int = 0


@dataclass
class SensorReading:
    """One sensor port's reading.

    Exactly one of ``color`` and ``distance`` is populated, chosen by
    ``type`` — the firmware overlays them on the same registers, so the other
    one would be the same bytes reinterpreted, not a second measurement.
    """

    type: SensorType = SensorType.EMPTY
    status: int = 0  #: ``SSTAT_*`` bits
    color_class: int = 0  #: taught colour 1-7 currently seen, 0 = no match
    confidence: int = 0  #: 0-255
    color: Optional[ColorReading] = None
    distance: Optional[DistanceReading] = None


@dataclass
class Telemetry:
    """One snapshot of the whole telemetry block.

    Every value in it was sampled by the same firmware pass, so they are
    mutually consistent by construction — you can compare an encoder count
    against a colour class without wondering whether they are from the same
    instant.

    On a channel in ``PULSE_WIDTH`` mode, ``encoder_count`` is the pulse width
    in microseconds (0 means NO SIGNAL) and ``encoder_velocity`` is us/s.
    """

    encoder_count: List[int] = field(default_factory=lambda: [0] * R.NUM_ENCODERS)
    encoder_velocity: List[int] = field(default_factory=lambda: [0] * R.NUM_ENCODERS)
    sensor: List[SensorReading] = field(
        default_factory=lambda: [SensorReading() for _ in range(R.NUM_SENSOR_PORTS)]
    )
    present_mask: int = 0
    timestamp_micros: int = 0  #: firmware clock, shared with the IMU block

    def sensor_present(self, port: int) -> bool:
        return 0 <= port < R.NUM_SENSOR_PORTS and bool(self.present_mask & (1 << port))

    def distance_valid(self, port: int) -> bool:
        if not 0 <= port < R.NUM_SENSOR_PORTS:
            return False
        s = self.sensor[port]
        return (
            s.type == SensorType.DISTANCE
            and s.distance is not None
            and s.distance.distance_mm != R.DISTANCE_INVALID
        )


@dataclass
class ImuState:
    """One snapshot of the IMU block. Angles in degrees, rates in deg/s."""

    quat_w: float = 0.0
    quat_x: float = 0.0
    quat_y: float = 0.0
    quat_z: float = 0.0
    yaw_deg: float = 0.0  #: robot frame, CCW-positive, [-180, 180)
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    gyro_dps: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  #: x y z
    accel_mps2: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  #: gravity removed
    calib_grade: int = 0  #: 0-3
    status: int = 0  #: ``ISTAT_*`` bits
    timestamp_micros: int = 0

    @property
    def gyro_saturated(self) -> bool:
        """True while the turn rate exceeds what the gyro can measure — usually
        a hard collision. The heading is suspect from that moment on."""
        return bool(self.status & R.ISTAT_SATURATED)

    @property
    def bias_valid(self) -> bool:
        """True once the robot has been still long enough to trust the bias."""
        return bool(self.status & R.ISTAT_BIAS_VALID)

    @property
    def fusion_valid(self) -> bool:
        """True while yaw/pitch/roll/quat are usable."""
        return bool(self.status & R.ISTAT_FUSION_VALID)


@dataclass
class Pose:
    """Robot pose: millimetres and radians, +X forward, +Y left, CCW positive."""

    x_mm: float = 0.0
    y_mm: float = 0.0
    heading_rad: float = 0.0

    @property
    def heading_deg(self) -> float:
        return math.degrees(self.heading_rad)


@dataclass
class LocalizerState:
    """One snapshot of the localizer block."""

    status: LocalizerStatus = LocalizerStatus.NOT_READY
    flags: int = 0  #: ``LOCF_*`` bits
    x_mm: float = 0.0
    y_mm: float = 0.0
    heading_rad: float = 0.0
    vel_x_mm_per_sec: float = 0.0  #: world frame
    vel_y_mm_per_sec: float = 0.0  #: world frame
    heading_vel_rad_per_sec: float = 0.0
    timestamp_micros: int = 0

    @property
    def pose(self) -> Pose:
        return Pose(self.x_mm, self.y_mm, self.heading_rad)

    @property
    def gyro_ever_saturated(self) -> bool:
        """True if the gyro has clipped at any point since the last localizer
        reset (latched) — relocalize, the integrated pose has drifted."""
        return bool(self.flags & R.LOCF_GYRO_SATURATED_EVER)

    @property
    def pose_clipped(self) -> bool:
        """True while the pose sits outside +/-32.7 m and reads clamped."""
        return bool(self.flags & R.LOCF_POSE_CLIPPED)

    @property
    def port_conflict(self) -> bool:
        """True if a pod channel's invert mask, mode or PWM parameters changed
        while the localizer was running. One tick delta was swallowed rather
        than integrated as a jump, so the pose is missing it; latched. Set
        direction and mode at setup, before starting the localizer."""
        return bool(self.flags & R.LOCF_PORT_CONFLICT)


@dataclass
class LocalizerParams:
    """Localizer setup. Stored values take effect on the next localizer reset."""

    #: Pod resolution, encoder ticks per millimetre of travel. Measure it:
    #: push the robot a couple of metres along one axis and divide.
    ticks_per_mm_x: float = 0.0
    ticks_per_mm_y: float = 0.0
    #: Tracking-point offset from the robot centre, mm, robot frame.
    tcp_offset_x_mm: float = 0.0
    tcp_offset_y_mm: float = 0.0
    #: Gyro scale correction from the spin-ten-turns calibration, near 1.0.
    imu_scalar: float = 1.0
    #: Encoder channels the X and Y pods are wired to, 0-3, and different.
    port_x: int = 0
    port_y: int = 1
    #: Velocity averaging window, ms.
    velocity_interval_ms: int = R.DEFAULT_LOC_VEL_INTERVAL_MS


@dataclass
class ColorClass:
    """A colour class window, in normalized chromaticity (0-1000 per channel)."""

    r_min: int = 0
    r_max: int = 0
    g_min: int = 0
    g_max: int = 0
    b_min: int = 0
    b_max: int = 0
    proximity_gate: bool = False
    prox_min: int = 0
    prox_max: int = R.PROX_MAX_VALUE


@dataclass
class DistanceClass:
    """A distance class window in millimetres, inclusive bounds."""

    dist_min: int = 0
    dist_max: int = 0
    hysteresis_mm: int = 10
    min_signal_rate: int = 0


@dataclass
class OutputConfig:
    """Full digital-output configuration; see the triggers guide."""

    source: OutputSource = OutputSource.DISABLED
    src_index: int = 0  #: sensor port or encoder channel
    class_index: int = 0  #: class slot, for SENSOR_CLASS sources
    active_low: bool = False
    mode: OutputMode = OutputMode.LEVEL
    debounce_assert: int = R.DEFAULT_DEBOUNCE_ASSERT
    debounce_release: int = R.DEFAULT_DEBOUNCE_RELEASE
    pulse_ms: int = 0
    thresh_min: int = 0  #: encoder counts, or yaw in centidegrees
    thresh_max: int = 0
