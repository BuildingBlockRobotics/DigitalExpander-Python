#!/usr/bin/env python3
"""color_read — which taught colour is the sensor looking at?

The board classifies for you. You get a slot number 1-7, or 0 for "nothing I
was taught" — no thresholds, no RGB arithmetic, and no chance of a confident
wrong answer from a dark or saturated reading, which both report 0.

Teach the colours first with color_teach.py.
"""

import sys
import time

from bbr_digital_expander import BBRDigitalExpander, BBRError

SENSOR_PORT = 0


def main() -> int:
    try:
        with BBRDigitalExpander(bus=1) as expander:
            if not expander.sensor_connected(SENSOR_PORT):
                print(f"No sensor on port {SENSOR_PORT} — plug one in.", file=sys.stderr)
                return 1
            while True:
                slot = expander.color_class(SENSOR_PORT)
                print(f"colour {slot}" if slot else "nothing I was taught")
                # Or ask about one colour directly:
                if expander.sees_color(SENSOR_PORT, 1):
                    print("  that is colour 1")
                time.sleep(0.2)
    except KeyboardInterrupt:
        return 0
    except BBRError as exc:
        print(f"Expander error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
