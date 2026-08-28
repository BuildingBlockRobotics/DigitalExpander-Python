#!/usr/bin/env python3
"""trigger_runtime — the other half of trigger_setup: no I2C at all.

After trigger_setup.py has run once, the expander watches its condition on its
own and drives a digital output pin. Your script reads that pin like any other
GPIO input. There is no driver call in this loop, no bus transaction, and
nothing to go wrong at a bad moment — the same as reading a limit switch.

Wire expander digital output 0 to a Raspberry Pi GPIO (BCM 17 below) and share
a ground. The expander's outputs are 3.3 V, which is what the Pi's pins want.
"""

import sys
import time

TRIGGER_PIN = 17  # BCM numbering


def main() -> int:
    try:
        from gpiozero import DigitalInputDevice
    except ImportError:
        print("This example needs gpiozero: pip install gpiozero", file=sys.stderr)
        return 1

    trigger = DigitalInputDevice(TRIGGER_PIN)
    try:
        while True:
            if trigger.value:
                print("target in range")
            time.sleep(0.05)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
