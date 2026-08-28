#!/usr/bin/env python3
"""localizer — where is the robot on the field?

Two dead-wheel odometry pods and the onboard IMU, fused on the board into a
pose you can just read. Odometry variant only.

Setup is once per robot: tell it how many encoder ticks make a millimetre
(measure it — push the robot two metres and divide), where the tracking point
sits relative to the robot centre, and which channels the pods are on.
"""

import sys
import time

from bbr_digital_expander import (
    BBRDigitalExpander,
    BBRError,
    EncoderDirection,
    LocalizerParams,
)


def start_localizer(expander: BBRDigitalExpander) -> None:
    print("Hold the robot completely still...")
    # Latches the stored parameters, zeroes the pose and calibrates the gyro.
    expander.reset_localizer_and_calibrate_imu()
    expander.wait_for_localizer_ready(5000)
    print("Localizer running.")


def main() -> int:
    try:
        with BBRDigitalExpander(bus=1) as expander:
            expander.set_localizer_params(
                LocalizerParams(
                    ticks_per_mm_x=19.894,  # MEASURE these — a guess here is a wrong pose
                    ticks_per_mm_y=19.894,
                    tcp_offset_x_mm=0.0,
                    tcp_offset_y_mm=0.0,
                    imu_scalar=1.0,  # from the spin-ten-turns calibration
                    # WHICH ENCODER CHANNELS THE PODS ARE ON. Any two of 0-3,
                    # as long as they differ and both are set — the localizer
                    # refuses to start otherwise. Both must stay in QUADRATURE
                    # mode: a channel switched to PULSE_WIDTH measures a pulse
                    # width rather than counting ticks and cannot drive the pose.
                    port_x=0,  # pod that rolls when the robot drives forward/back
                    port_y=1,  # pod that rolls when the robot strafes left/right
                )
            )
            expander.save_config_to_flash()  # parameters do NOT save themselves

            # DIRECTION. The localizer needs forward and left travel to BOTH
            # count positive. It reads the channels itself, so flipping a sign
            # in this script never reaches it — fix it here instead.
            #
            # Push the robot to find out which way each pod counts: driving
            # forward should raise the X count, strafing left should raise Y.
            # If one goes the wrong way, change its FORWARD to REVERSE.
            # These two DO save themselves, so the direction survives a reboot.
            expander.set_encoder_direction(0, EncoderDirection.FORWARD)  # X pod
            expander.set_encoder_direction(1, EncoderDirection.FORWARD)  # Y pod

            start_localizer(expander)

            while True:
                s = expander.read_localizer()
                line = (f"x={s.x_mm:.0f}mm y={s.y_mm:.0f}mm "
                        f"heading={s.pose.heading_deg:.1f}deg")
                # Latched since the last reset: the pose has drifted by an
                # unknown amount and needs a fresh fix, not a smaller loop time.
                if s.gyro_ever_saturated:
                    line += "  [COLLISION — relocalize]"
                if s.port_conflict:
                    line += "  [pod channel claimed elsewhere]"
                if s.pose_clipped:
                    line += "  [pose out of range]"
                print(line)
                time.sleep(0.05)
    except KeyboardInterrupt:
        return 0
    except BBRError as exc:
        print(f"Expander error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
