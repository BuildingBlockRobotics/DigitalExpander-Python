#!/usr/bin/env python3
"""sensor_dump — everything the board knows, in one bus transaction.

The bring-up diagnostic. Every value printed in one block came from a single
firmware snapshot, so if two of them disagree, that disagreement is real and
not an artifact of reading them at different times.
"""

import sys
import time

from bbr_digital_expander import BBRDigitalExpander, BBRError, SensorType
from bbr_digital_expander import regmap as R

STATUS_BITS = [
    (R.SSTAT_VALID, "valid"),
    (R.SSTAT_STALE, "STALE"),
    (R.SSTAT_QUALITY_FAIL, "QUALITY-FAIL"),
    (R.SSTAT_OVERFLOW, "SATURATED"),
    (R.SSTAT_BUS_ERROR, "BUS-ERROR"),
    (R.SSTAT_INSUFFICIENT_SIGNAL, "WEAK-SIGNAL"),
]


def describe(port: int, t) -> str:
    if not t.sensor_present(port):
        return "empty"
    s = t.sensor[port]
    flags = " ".join(name for bit, name in STATUS_BITS if s.status & bit)
    if s.type == SensorType.DISTANCE:
        d = s.distance
        reading = f"{d.distance_mm} mm" if t.distance_valid(port) else "out of range"
        return f"distance {reading}, signal {d.signal_rate}  {flags}"
    c = s.color
    return (
        f"colour class {s.color_class} conf {s.confidence}  "
        f"r={c.red} g={c.green} b={c.blue} ir={c.ir} prox={c.proximity}  {flags}"
    )


def main() -> int:
    try:
        with BBRDigitalExpander(bus=1) as expander:
            expander.print_device_info()
            while True:
                t = expander.read_telemetry()
                print(f"\n--- t={t.timestamp_micros} us")
                for ch in range(4):
                    print(f"  encoder {ch}: {t.encoder_count[ch]} counts, "
                          f"{t.encoder_velocity[ch]} counts/s")
                # A line that never toggles while you turn the shaft is the
                # dead wire.
                print(f"  encoder pins: 0b{expander.encoder_pin_state():08b}")
                for port in range(4):
                    print(f"  port {port}: {describe(port, t)}")
                print(f"  digital outputs: 0b{expander.output_state():04b}  "
                      f"latched 0b{expander.output_latched():04b}")
                time.sleep(0.5)
    except KeyboardInterrupt:
        return 0
    except BBRError as exc:
        print(f"Expander error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
