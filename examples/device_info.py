#!/usr/bin/env python3
"""device_info — is it there, and what is it?

The first thing to run on a new board, and the first thing to run when
something has stopped working. Nothing here configures anything: it reads the
identity registers and prints them.

If this fails, no other example can work, so fix it here — check the wiring,
the 3.3 V supply, the bus pull-ups and the address jumpers.
"""

import sys

from bbr_digital_expander import BBRDigitalExpander, BBRError


def main() -> int:
    try:
        with BBRDigitalExpander(bus=1) as expander:
            expander.print_device_info()
    except BBRError as exc:
        print(f"Expander not found: {exc}", file=sys.stderr)
        print("\nTry 'i2cdetect -y 1' — the board should appear at 0x38-0x3b.",
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
