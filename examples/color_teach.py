#!/usr/bin/env python3
"""color_teach — teach the board a colour by showing it one.

You never write RGB thresholds by hand. Hold the thing you care about in front
of the sensor, type a digit, and the board stores what it saw as that colour
number. From then on it answers "do I see colour 3?" by itself, and the answer
survives a power cycle because teaching saves to flash.

Run this once, on the field, under the lighting you will actually compete in.
Then use color_read.py (or a trigger) at runtime.
"""

import sys

from bbr_digital_expander import BBRDigitalExpander, BBRError

SENSOR_PORT = 0


def main() -> int:
    try:
        with BBRDigitalExpander(bus=1) as expander:
            if not expander.sensor_connected(SENSOR_PORT):
                print(f"No sensor on port {SENSOR_PORT} — plug one in.", file=sys.stderr)
                return 1

            print("Hold a colour in front of the sensor, then type 1-7 to teach it")
            print("as that colour number. 0 shows what is seen now, Ctrl-C quits.")
            while True:
                answer = input("> ").strip()
                if answer not in [str(n) for n in range(8)]:
                    continue
                slot = int(answer)
                if slot == 0:
                    print(f"  currently seeing colour {expander.color_class(SENSOR_PORT)}")
                    continue

                print(f"Teaching colour {slot} — hold still...")
                try:
                    # Takes a few seconds, and saves to flash on success. It
                    # refuses rather than storing a class that would never fire
                    # (too dark) or always fire (saturated) — an error here is
                    # the board protecting you from a colour that would have
                    # failed silently at the worst moment.
                    expander.teach_color(SENSOR_PORT, slot)
                    print("  stored and saved.")
                except BBRError as exc:
                    print(f"  refused: {exc}")
    except (KeyboardInterrupt, EOFError):
        return 0
    except BBRError as exc:
        print(f"Expander error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
