"""Unit tests for the Python driver, against the fake bus.

These check the wire format and the refusals — the parts a port can get
subtly wrong (endianness, sign, scale factors, window arithmetic) and the
parts that exist so a fault is loud rather than plausible. They cannot check
anything about the real board; that is what the examples on real hardware are
for.
"""

from __future__ import annotations

import math
import unittest

from bbr_digital_expander import (
    BBRDigitalExpander,
    BadArgumentError,
    BBRError,
    ChannelMode,
    DistanceClass,
    EncoderDirection,
    LocalizerNotRunningError,
    LocalizerParams,
    NoImuError,
    ProtocolMismatchError,
    SensorType,
    TransportError,
    WrongChannelModeError,
    WrongDeviceError,
    WrongSensorTypeError,
)
from bbr_digital_expander import regmap as R
from tests.fake_bus import FakeExpanderBus


def begun(variant=R.VARIANT_ODOMETRY):
    bus = FakeExpanderBus(variant=variant)
    return BBRDigitalExpander(bus=bus).begin(), bus


class TestIdentity(unittest.TestCase):
    def test_begin_reads_identity(self):
        exp, _ = begun()
        self.assertEqual(exp.firmware_version, (1, 2, 3))
        self.assertEqual(exp.protocol_minor, R.PROTOCOL_MINOR_VALUE)
        self.assertTrue(exp.is_odometry_variant)

    def test_wrong_device_id_refuses(self):
        bus = FakeExpanderBus()
        bus.regs[R.REG_DEVICE_ID] = 0x11
        with self.assertRaises(WrongDeviceError):
            BBRDigitalExpander(bus=bus).begin()

    def test_wrong_protocol_major_refuses(self):
        bus = FakeExpanderBus()
        bus.regs[R.REG_PROTOCOL_MAJOR] = R.PROTOCOL_MAJOR_VALUE + 1
        with self.assertRaises(ProtocolMismatchError):
            BBRDigitalExpander(bus=bus).begin()

    def test_address_outside_the_jumper_range_refuses(self):
        with self.assertRaises(BadArgumentError):
            BBRDigitalExpander(address=0x40, bus=FakeExpanderBus())

    def test_calls_before_begin_refuse(self):
        exp = BBRDigitalExpander(bus=FakeExpanderBus())
        with self.assertRaises(Exception):
            exp.read_telemetry()

    def test_context_manager_closes_the_bus(self):
        bus = FakeExpanderBus()
        with BBRDigitalExpander(bus=bus):
            pass
        self.assertTrue(bus.closed)


class TestTelemetry(unittest.TestCase):
    def test_encoder_counts_are_signed_little_endian(self):
        exp, bus = begun()
        bus.set_i32(R.REG_ENC0_COUNT, -1234567)
        bus.set_i32(R.REG_ENC1_COUNT, 42)
        bus.set_i32(R.REG_ENC0_VELOCITY, -80)
        self.assertEqual(exp.encoder_count(0), -1234567)
        self.assertEqual(exp.encoder_count(1), 42)
        self.assertEqual(exp.encoder_velocity(0), -80)

    def test_one_transaction_answers_several_questions(self):
        exp, bus = begun()
        bus.set_i32(R.REG_ENC0_COUNT, 7)
        exp.encoder_count(0)
        reads = len(bus.writes)
        for ch in range(4):
            exp.encoder_count(ch)
        # The cache means the extra getters cost no further pointer writes.
        self.assertEqual(len(bus.writes), reads)

    def test_colour_port_decodes_channels_and_class(self):
        exp, bus = begun()
        bus.regs[R.REG_SENSOR_TYPE + 2] = R.STYPE_COLOR
        bus.regs[R.REG_SENSOR_PRESENT_MASK] = 0b0100
        bus.regs[R.REG_S0_CLASS + 2] = 3
        raw = R.RAW_BASE + R.RAW_STRIDE * 2
        bus.set_u16(raw + R.RAW_RED, 600)
        bus.set_u16(raw + R.RAW_PROXIMITY, 1500)
        t = exp.read_telemetry()
        self.assertEqual(t.sensor[2].type, SensorType.COLOR)
        self.assertEqual(t.sensor[2].color.red, 600)
        self.assertEqual(t.sensor[2].color.proximity, 1500)
        self.assertTrue(t.sensor_present(2))
        self.assertEqual(exp.color_class(2), 3)
        self.assertTrue(exp.sees_color(2, 3))
        self.assertFalse(exp.sees_color(2, 4))

    def test_distance_port_reads_mm_and_infinity_out_of_range(self):
        exp, bus = begun()
        bus.regs[R.REG_SENSOR_TYPE + 1] = R.STYPE_DISTANCE
        bus.regs[R.REG_SENSOR_PRESENT_MASK] = 0b0010
        raw = R.RAW_BASE + R.RAW_STRIDE * 1
        bus.set_u16(raw + R.RAW_DISTANCE_MM, 250)
        self.assertEqual(exp.distance_mm(1), 250.0)

        bus.set_u16(raw + R.RAW_DISTANCE_MM, R.DISTANCE_INVALID)
        exp.invalidate_telemetry()
        self.assertEqual(exp.distance_mm(1), math.inf)
        # The sentinel is infinity precisely so this comparison is safe.
        self.assertFalse(exp.distance_mm(1) < 300)

    def test_asking_a_colour_port_for_a_distance_refuses(self):
        exp, bus = begun()
        bus.regs[R.REG_SENSOR_TYPE + 0] = R.STYPE_COLOR
        bus.regs[R.REG_SENSOR_PRESENT_MASK] = 0b0001
        with self.assertRaises(WrongSensorTypeError):
            exp.distance_mm(0)

    def test_asking_a_distance_port_for_a_colour_refuses(self):
        exp, bus = begun()
        bus.regs[R.REG_SENSOR_TYPE + 0] = R.STYPE_DISTANCE
        bus.regs[R.REG_SENSOR_PRESENT_MASK] = 0b0001
        with self.assertRaises(WrongSensorTypeError):
            exp.color_class(0)

    def test_uniform_block_is_a_failed_transfer_not_data(self):
        exp, bus = begun()
        for i in range(R.SNAP_TELEMETRY_START, R.SNAP_TELEMETRY_START + 96):
            bus.regs[i] = 0xFF
        with self.assertRaises(Exception):
            exp.read_telemetry()

    def test_port_out_of_range_refuses(self):
        exp, _ = begun()
        for call in (exp.distance_mm, exp.color_class, exp.sensor_connected):
            with self.assertRaises(BadArgumentError):
                call(4)


class TestCommands(unittest.TestCase):
    def test_reset_encoder_sends_a_mask_and_a_fresh_token(self):
        exp, bus = begun()
        bus.set_i32(R.REG_ENC0_COUNT, 500)
        exp.reset_encoder(2)
        exp.reset_all_encoders()
        opcodes = [c[0] for c in bus.commands]
        args = [c[1] for c in bus.commands]
        tokens = [c[2] for c in bus.commands]
        self.assertEqual(opcodes, [R.CMD_RESET_ENCODERS, R.CMD_RESET_ENCODERS])
        self.assertEqual(args, [0b0100, 0b1111])
        self.assertEqual(len(set(tokens)), 2)  # never the same token twice
        self.assertNotIn(R.TOKEN_NONE, tokens)
        self.assertEqual(exp.encoder_count(0), 0)

    def test_teach_colour_captures_stores_and_saves(self):
        exp, bus = begun()
        exp.teach_color(1, 2)
        self.assertEqual(bus.regs[R.REG_CFG_SENSOR_SELECT], 1)
        self.assertEqual(bus.regs[R.REG_CFG_CLASS_SELECT], 2)
        self.assertEqual(
            [c[0] for c in bus.commands],
            [R.CMD_SENSOR_CAPTURE, R.CMD_CFG_STORE, R.CMD_CFG_SAVE_FLASH],
        )
        self.assertFalse(exp.is_config_dirty())


class TestEncoderSetup(unittest.TestCase):
    def test_direction_flips_one_bit_and_saves(self):
        exp, bus = begun()
        exp.set_encoder_direction(1, EncoderDirection.REVERSE)
        self.assertEqual(bus.regs[R.REG_ENC_INVERT_MASK], 0b0010)
        self.assertEqual(exp.get_encoder_direction(1), EncoderDirection.REVERSE)
        exp.set_encoder_direction(1, EncoderDirection.FORWARD)
        self.assertEqual(bus.regs[R.REG_ENC_INVERT_MASK], 0)

    def test_pulse_width_on_a_quadrature_channel_refuses(self):
        exp, _ = begun()
        with self.assertRaises(WrongChannelModeError):
            exp.pulse_width_us(0)

    def test_pulse_width_reads_the_count_register_in_pwm_mode(self):
        exp, bus = begun()
        exp.set_channel_mode(3, ChannelMode.PULSE_WIDTH)
        bus.set_i32(R.REG_ENC0_COUNT + 4 * 3, 1024)
        self.assertEqual(bus.regs[R.REG_CHANNEL_MODE], 0b1000)
        self.assertEqual(exp.pulse_width_us(3), 1024)

    def test_pwm_params_preserve_the_other_channels(self):
        exp, bus = begun()
        exp.set_pwm_channel_params(0, 1, 1024)
        exp.set_pwm_channel_params(2, 5, 2000)
        w = bus.regs[R.CFGWIN_BASE:R.CFGWIN_BASE + R.CFGWIN_SIZE]
        self.assertEqual(w[0] | (w[1] << 8), 1)
        self.assertEqual(w[2] | (w[3] << 8), 1024)
        self.assertEqual(w[8] | (w[9] << 8), 5)
        self.assertEqual(exp.get_pwm_channel_params(2), (5, 2000))

    def test_backwards_pwm_window_refuses(self):
        exp, _ = begun()
        with self.assertRaises(BadArgumentError):
            exp.set_pwm_channel_params(0, 900, 900)


class TestTriggers(unittest.TestCase):
    def _window(self, bus):
        base = R.DOUTWIN_BASE
        d = bus.regs
        lo = int.from_bytes(d[R.DOUTWIN_THRESH_MIN:R.DOUTWIN_THRESH_MIN + 4], "little", signed=True)
        hi = int.from_bytes(d[R.DOUTWIN_THRESH_MAX:R.DOUTWIN_THRESH_MAX + 4], "little", signed=True)
        return d[base], lo, hi

    def test_colour_trigger_attaches_the_class_to_the_output(self):
        exp, bus = begun()
        exp.trigger_on_color(0, 1, 3)
        self.assertEqual(bus.regs[R.REG_DOUT_SELECT], 0)
        self.assertEqual(bus.regs[R.DOUTWIN_SRC_TYPE], R.DOUT_SRC_SENSOR_CLASS)
        self.assertEqual(bus.regs[R.DOUTWIN_SRC_INDEX], 1)
        self.assertEqual(bus.regs[R.DOUTWIN_CLASS_INDEX], 3)

    def test_proximity_trigger_uses_the_last_slot(self):
        exp, bus = begun()
        exp.trigger_when_near(2, 1, 300)
        self.assertEqual(bus.regs[R.REG_CFG_CLASS_SELECT], R.CLASS_SLOT_LAST)
        self.assertEqual(bus.regs[R.DOUTWIN_CLASS_INDEX], R.CLASS_SLOT_LAST)

    def test_heading_window_wraps_across_180(self):
        exp, bus = begun()
        exp.trigger_when_facing(0, 175.0, 10.0)
        src, lo, hi = self._window(bus)
        self.assertEqual(src, R.DOUT_SRC_IMU_HEADING)
        self.assertEqual(lo, 16500)
        self.assertEqual(hi, -17500)  # folded past +180, as the firmware expects

    def test_a_full_circle_reads_as_always_true(self):
        exp, bus = begun()
        exp.trigger_when_facing(0, 0.0, 180.0)
        _, lo, hi = self._window(bus)
        self.assertEqual((lo, hi), (R.YAW_MIN_CDEG, R.YAW_MAX_CDEG))

    def test_bad_heading_arguments_refuse(self):
        exp, _ = begun()
        for heading, tol in ((float("nan"), 10.0), (0.0, 0.0), (0.0, 200.0)):
            with self.assertRaises(BadArgumentError):
                exp.trigger_when_facing(0, heading, tol)

    def test_encoder_trigger_is_open_ended_above_the_threshold(self):
        exp, bus = begun()
        exp.trigger_when_encoder_past(1, 2, -500)
        src, lo, hi = self._window(bus)
        self.assertEqual(src, R.DOUT_SRC_ENCODER)
        self.assertEqual(lo, -500)
        self.assertEqual(hi, 0x7FFFFFFF)

    def test_distance_class_window_is_written_in_millimetres(self):
        exp, bus = begun()
        exp.write_distance_class(0, 1, DistanceClass(dist_min=50, dist_max=400,
                                                     hysteresis_mm=15, min_signal_rate=100))
        w = bus.regs
        self.assertEqual(w[R.CFGWIN_ENABLED], 1)
        self.assertEqual(w[R.CFGWIN_DIST_MIN] | (w[R.CFGWIN_DIST_MIN + 1] << 8), 50)
        self.assertEqual(w[R.CFGWIN_DIST_MAX] | (w[R.CFGWIN_DIST_MAX + 1] << 8), 400)


class TestImu(unittest.TestCase):
    def test_heading_decodes_centidegrees(self):
        exp, bus = begun()
        bus.set_i16(R.REG_YAW, -4512)
        self.assertAlmostEqual(exp.heading(), -45.12, places=4)

    def test_gyro_and_accel_scales(self):
        exp, bus = begun()
        bus.set_i16(R.REG_GYRO_Z, 16 * 90)
        bus.set_i16(R.REG_ACCEL_X, 100 * -3)
        s = exp.read_imu()
        self.assertAlmostEqual(s.gyro_dps[2], 90.0)
        self.assertAlmostEqual(s.accel_mps2[0], -3.0)
        self.assertTrue(s.fusion_valid)

    def test_a_base_board_refuses_rather_than_reporting_zero(self):
        exp, _ = begun(variant=R.VARIANT_BASE)
        with self.assertRaises(NoImuError):
            exp.heading()


class TestLocalizer(unittest.TestCase):
    def _running(self):
        exp, bus = begun()
        bus.regs[R.REG_LOC_STATUS] = R.LOC_RUNNING
        bus.set_i16(R.REG_LOC_X, 1500)
        bus.set_i16(R.REG_LOC_Y, -250)
        bus.set_i16(R.REG_LOC_H, int(round(math.pi / 2 * R.LOC_HEADING_SCALE)))
        bus.restamp_localizer_crc()
        return exp, bus

    def test_pose_decodes_mm_and_radians(self):
        exp, _ = self._running()
        p = exp.get_pose()
        self.assertEqual((p.x_mm, p.y_mm), (1500.0, -250.0))
        self.assertAlmostEqual(p.heading_deg, 90.0, places=2)

    def test_a_corrupt_block_serves_the_last_pose_not_the_suspect_one(self):
        exp, bus = self._running()
        good = exp.get_pose()
        bus.set_i16(R.REG_LOC_X, 999)  # CRC now stale
        served = exp.get_pose()
        self.assertEqual((served.x_mm, served.y_mm), (good.x_mm, good.y_mm))
        self.assertFalse(exp.is_data_fresh())

    def test_a_corrupt_block_raises_if_no_pose_was_ever_read(self):
        exp, bus = self._running()
        bus.set_i16(R.REG_LOC_X, 999)  # CRC stale before any good read
        with self.assertRaises(TransportError):
            exp.get_pose()

    def test_a_corruption_streak_that_outlives_a_shutdown_raises(self):
        exp, bus = self._running()
        exp.get_pose()
        bus.set_i16(R.REG_LOC_X, 999)
        for _ in range(4):
            exp.get_pose()  # stale but tolerated
        # Backdate the streak: five reads is only a fault once it has also
        # lasted longer than any plausible teardown.
        exp._fail_streak_start -= 1.0
        with self.assertRaises(BBRError):
            exp.get_pose()

    def test_a_good_read_clears_the_streak(self):
        exp, bus = self._running()
        exp.get_pose()
        bus.set_i16(R.REG_LOC_X, 999)
        exp.get_pose()
        self.assertFalse(exp.is_data_fresh())
        bus.restamp_localizer_crc()  # bus recovers
        self.assertEqual(exp.get_pose().x_mm, 999.0)
        self.assertTrue(exp.is_data_fresh())

    def test_pose_before_the_localizer_runs_refuses(self):
        exp, bus = begun()
        bus.regs[R.REG_LOC_STATUS] = R.LOC_NOT_READY
        bus.restamp_localizer_crc()
        with self.assertRaises(LocalizerNotRunningError):
            exp.get_pose()

    def test_params_round_trip_through_the_window(self):
        exp, _ = self._running()
        p = LocalizerParams(ticks_per_mm_x=13.26, ticks_per_mm_y=13.26,
                            tcp_offset_x_mm=-40, tcp_offset_y_mm=15,
                            imu_scalar=1.005, port_x=0, port_y=2)
        exp.set_localizer_params(p)
        back = exp.get_localizer_params()
        self.assertAlmostEqual(back.ticks_per_mm_x, 13.26, places=3)
        self.assertAlmostEqual(back.tcp_offset_x_mm, -40.0)
        self.assertAlmostEqual(back.imu_scalar, 1.005, places=4)
        self.assertEqual((back.port_x, back.port_y), (0, 2))

    def test_pods_on_the_same_channel_refuse(self):
        exp, _ = self._running()
        with self.assertRaises(BadArgumentError):
            exp.set_localizer_params(LocalizerParams(ticks_per_mm_x=13.0, ticks_per_mm_y=13.0,
                                                     port_x=1, port_y=1))

    def test_unmeasured_ticks_per_mm_refuses(self):
        exp, _ = self._running()
        with self.assertRaises(BadArgumentError):
            exp.set_localizer_params(LocalizerParams())  # zeros: nobody measured

    def test_set_pose_normalizes_the_heading(self):
        exp, bus = self._running()
        exp.set_pose(100, -200, math.pi * 3)  # 540 degrees
        h = int.from_bytes(bus.regs[R.LOCWIN_POSE_H:R.LOCWIN_POSE_H + 2], "little", signed=True)
        self.assertAlmostEqual(h / R.LOC_HEADING_SCALE, -math.pi, places=3)


if __name__ == "__main__":
    unittest.main()
