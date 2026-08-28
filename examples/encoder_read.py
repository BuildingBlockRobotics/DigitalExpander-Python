#!/usr/bin/env python3
"""encoder_read — four quadrature encoders, counted on the board.

The board does the counting, so nothing here is timing-sensitive: no
interrupts, no missed edges when Python is busy elsewhere, and no penalty for
reading slowly. Every value printed on one line came from the same firmware
snapshot.

Press Ctrl-C to stop; 'r' then Enter is not read here — see color_teach for
an interactive example.
"""

import sys
import time

from bbr_digital_expander import BBRDigitalExpander, BBRError


def main() -> int:
    try:
        with BBRDigitalExpander(bus=1) as expander:
            expander.reset_all_encoders()
            while True:
                t = expander.read_telemetry()
                print(
                    "  ".join(
                        f"ch{ch}: {t.encoder_count[ch]:>9} counts "
                        f"({t.encoder_velocity[ch]:>7}/s)"
                        for ch in range(4)
                    )
                )
                # A bit that never toggles while you turn that shaft is the
                # dead wire: bit 2n is channel n's A line, bit 2n+1 its B.
                print(f"    pins 0b{expander.encoder_pin_state():08b}")
                time.sleep(0.2)
    except KeyboardInterrupt:
        return 0
    except BBRError as exc:
        print(f"Expander error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
