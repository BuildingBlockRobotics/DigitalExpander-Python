#!/usr/bin/env python3
"""latched_output — catching an event too brief for your loop to see.

A line of tape passing under a sensor at speed may be visible for 3 ms. A loop
that polls every 20 ms will miss it most times, and the times it does not are
the ones that make the bug look intermittent. Python on a general-purpose OS
makes that worse, not better: the scheduler can take your loop away for tens
of milliseconds at a time.

A LATCHED output stays high once the condition has been true even for an
instant, until you explicitly clear it. So the question stops being "is it
true right now?" and becomes "has it been true since I last looked?", which is
the question you actually wanted to ask.
"""

import sys
import time

from bbr_digital_expander import (
    BBRDigitalExpander,
    BBRError,
    OutputConfig,
    OutputMode,
    OutputSource,
)

OUTPUT_INDEX = 0
SENSOR_PORT = 0
COLOR_SLOT = 1


def main() -> int:
    try:
        with BBRDigitalExpander(bus=1) as expander:
            # The advanced tier: full control, and it does NOT save to flash
            # for you.
            expander.configure_output(
                OUTPUT_INDEX,
                OutputConfig(
                    source=OutputSource.SENSOR_CLASS,
                    src_index=SENSOR_PORT,
                    class_index=COLOR_SLOT,
                    mode=OutputMode.LATCHED,
                    debounce_assert=1,  # latch on the first sample that matches
                ),
            )
            expander.save_config_to_flash()

            print("Watching. Ctrl-C to stop.")
            while True:
                if expander.output_latched() & (1 << OUTPUT_INDEX):
                    print("the colour went past since the last clear")
                    expander.clear_output_latch(OUTPUT_INDEX)
                time.sleep(0.2)
    except KeyboardInterrupt:
        return 0
    except BBRError as exc:
        print(f"Expander error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
