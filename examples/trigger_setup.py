#!/usr/bin/env python3
"""trigger_setup — teach the board to watch a condition for you. RUN ONCE.

This is the whole point of the product. The board can watch a distance, a
taught colour, an encoder threshold or a heading, and drive one of its four
digital outputs when the condition is met — with no I2C traffic and no code in
your loop. The reaction time is the board's, not your script's, which matters
more on a Raspberry Pi than on a microcontroller: Linux can take your process
away for tens of milliseconds whenever it likes, and the board does not care.

The configuration is saved to flash, so it survives a power cycle. Run this
once, then run trigger_runtime.py and never talk to the board again.
"""

import sys

from bbr_digital_expander import BBRDigitalExpander, BBRError


def report(what, action) -> None:
    try:
        action()
        print(f"  ok      {what}")
    except BBRError as exc:
        print(f"  FAILED  {what} — {exc}")


def main() -> int:
    try:
        expander = BBRDigitalExpander(bus=1).begin()
    except BBRError as exc:
        print(f"Expander not found: {exc}", file=sys.stderr)
        return 1

    try:
        print("Configuring triggers (this writes to flash)...")

        # Output 0 goes high while the distance sensor on port 0 sees
        # something within 200 mm. The board debounces it and applies
        # hysteresis so the output does not chatter at the edge of the window.
        report("output 0 <- within 200 mm of port 0",
               lambda: expander.trigger_when_near(0, 0, 200))

        # Output 1 goes high while the colour sensor on port 1 sees colour 1.
        # Teach colour 1 first with color_teach.py.
        report("output 1 <- port 1 sees colour 1",
               lambda: expander.trigger_on_color(1, 1, 1))

        # Output 2 goes high once encoder channel 0 passes 5000 counts.
        report("output 2 <- encoder 0 past 5000 counts",
               lambda: expander.trigger_when_encoder_past(2, 0, 5000))

        # Output 3 goes high while the robot faces 90 degrees +/- 15.
        # Odometry variant only — this one is expected to fail on a base board.
        report("output 3 <- facing 90 +/- 15 degrees",
               lambda: expander.trigger_when_facing(3, 90.0, 15.0))
    finally:
        expander.close()

    print("\nDone. Run trigger_runtime.py — the board does the rest.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
