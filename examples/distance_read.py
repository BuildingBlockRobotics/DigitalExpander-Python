#!/usr/bin/env python3
"""distance_read — how far away is it?

Millimetres, or math.inf when nothing is in range. The infinity is deliberate:
`distance_mm(port) < 300` is simply False when there is nothing there, which is
what you meant, so you never have to remember to test a sentinel before
comparing.
"""

import math
import sys
import time

from bbr_digital_expander import BBRDigitalExpander, BBRError

SENSOR_PORT = 1


def main() -> int:
    try:
        with BBRDigitalExpander(bus=1) as expander:
            while True:
                mm = expander.distance_mm(SENSOR_PORT)
                if math.isinf(mm):
                    print("nothing in range")
                else:
                    print(f"{mm:.0f} mm")
                if mm < 300:
                    print("  close enough to stop")
                time.sleep(0.1)
    except KeyboardInterrupt:
        return 0
    except BBRError as exc:
        # A colour sensor on this port lands here rather than returning a
        # number: that is a wiring mix-up, not a reading.
        print(f"Expander error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
