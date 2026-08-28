#!/usr/bin/env python3
"""heading_imu — which way is the robot facing?

Degrees, [-180, 180), positive turning left. The fusion runs on the board, so
this is a register read rather than a filter you have to keep fed.

Odometry variant only. A base board raises NoImuError rather than reporting a
plausible heading of 0.00 forever — an IMU that reads a steady zero is the
worst kind of failure, because it looks like working software.
"""

import sys
import time

from bbr_digital_expander import BBRDigitalExpander, BBRError, NoImuError


def main() -> int:
    try:
        with BBRDigitalExpander(bus=1) as expander:
            print("Hold the robot still — calibrating the gyro...")
            expander.calibrate_gyro()
            expander.reset_heading()

            while True:
                s = expander.read_imu()
                line = (
                    f"yaw {s.yaw_deg:7.2f}  pitch {s.pitch_deg:7.2f}  "
                    f"roll {s.roll_deg:7.2f}  calib {s.calib_grade}/3"
                )
                # Latched conditions worth acting on rather than printing past.
                if s.gyro_saturated:
                    line += "  [GYRO CLIPPED — heading suspect]"
                if not s.bias_valid:
                    line += "  [bias not settled yet]"
                print(line)
                time.sleep(0.1)
    except KeyboardInterrupt:
        return 0
    except NoImuError as exc:
        print(f"{exc}", file=sys.stderr)
        return 1
    except BBRError as exc:
        print(f"Expander error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
