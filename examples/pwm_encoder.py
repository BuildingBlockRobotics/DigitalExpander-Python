#!/usr/bin/env python3
"""pwm_encoder — absolute encoders that report position as a pulse width.

A REV Through Bore encoder's absolute output is a PWM signal whose pulse width
is the shaft angle. Put a channel into PULSE_WIDTH mode and the same count
register reports microseconds instead of counts — absolute, correct the
instant you power on, with no homing move.

A pulse width of 0 means NO SIGNAL. Test for it; do not treat it as "the shaft
is at zero", which is exactly what it is not.
"""

import sys
import time

from bbr_digital_expander import BBRDigitalExpander, BBRError, ChannelMode

CHANNEL = 3


def main() -> int:
    try:
        with BBRDigitalExpander(bus=1) as expander:
            expander.set_channel_mode(CHANNEL, ChannelMode.PULSE_WIDTH)
            # The pulse widths at the start and end of one revolution. For a
            # REV Through Bore absolute output that is 1 us to 1024 us.
            # Measure yours.
            expander.set_pwm_channel_params(CHANNEL, 1, 1024)
            # With the range known, the board can count revolutions through
            # the wrap, so a multi-turn mechanism reports a continuously
            # growing position.
            expander.set_pwm_wrap_enabled(CHANNEL, True)
            expander.save_config_to_flash()

            while True:
                us = expander.pulse_width_us(CHANNEL)
                print("NO SIGNAL — check the encoder cable" if us == 0 else f"{us} us")
                time.sleep(0.1)
    except KeyboardInterrupt:
        return 0
    except BBRError as exc:
        print(f"Expander error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
