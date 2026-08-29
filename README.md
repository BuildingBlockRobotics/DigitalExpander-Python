# BBR Digital Expander — Python driver

A Python driver for the **BBR Digital Expander**: a board that adds four
quadrature (or pulse-width) encoder inputs, four colour/distance sensor ports,
four digital outputs and — on the odometry variant — an IMU and a full pose
estimator, all through a single I2C connection.

**Documentation: <https://expander.buildingblockrobotics.com/>**

The expander was designed for FTC, but it is an ordinary I2C target: 100 or
400 kHz, one 7-bit address in `0x38`–`0x3B`, no vendor runtime, no special
host. This package drives it from a Raspberry Pi, and from anything else
running Linux with an I2C bus — a Jetson, a Pi Zero, an x86 box with a USB
adapter that presents `/dev/i2c-*`.

The board can also watch a condition for you — a taught colour, a distance, an
encoder threshold, a heading — and drive a digital output when it is met, with
**no I2C traffic and no code in your loop**. That matters more here than on a
microcontroller: Linux can take your process away for tens of milliseconds
whenever it likes, and the board does not care.

## Wiring, before anything else

A REV Control Hub does two things for this board that a Raspberry Pi does not:

- **Fit bus pull-ups.** The Expander carries none on the host-facing bus. A
  Pi's SDA1/SCL1 pins do have 1.8 kΩ pull-ups to 3.3 V fitted on the board, so
  on a Pi you usually need nothing extra — but they are the *only* pull-ups in
  the system, and a long cable may still want a few hundred picofarads' worth
  of help at 400 kHz.
- **Supply 3.3 V.** Roughly 55 mA; there is no regulator on the expander. Pin
  1 of the Pi header is 3.3 V and is fine for that. **Do not use pin 2 or 4
  (5 V)** — every I/O pin on the expander is 3.3 V and none of them are 5 V
  tolerant.

| Raspberry Pi header | Expander |
|---|---|
| pin 1 — 3V3 | 3.3 V |
| pin 3 — GPIO 2 (SDA1) | SDA |
| pin 5 — GPIO 3 (SCL1) | SCL |
| pin 6 — GND | GND |

## Install

Enable I2C once, and make sure your user can reach it:

```sh
sudo raspi-config nonint do_i2c 0     # Interface Options -> I2C
sudo usermod -aG i2c "$USER"          # log out and back in
```

Then:

```sh
pip install bbr-digital-expander
```

The package is not on PyPI yet, so until it is, install it from GitHub:

```sh
pip install git+https://github.com/BuildingBlockRobotics/DigitalExpander-Python
```

Check the board is there before writing any code:

```sh
i2cdetect -y 1        # the expander shows up at 0x38-0x3b
```

### If reads come back corrupt

You are unlikely to hit this — the driver has been run against real hardware
on a Pi 4 and a Pi 5 at the stock bus rate, with no `config.txt` changes at
all — but it is worth knowing why the failure looks the way it does if you do.

The expander is an RP2040 in I2C target mode, so it stretches the clock while
its firmware services a transfer, and the Pi's hardware I2C controller has a
long-standing bug handling clock stretching. The symptom would be occasional
garbage rather than a clean failure — which the driver turns into a loud
error, because the identity, telemetry, IMU and localizer blocks are all
checked for it.

Two fixes, in order of preference:

- **Slow the bus down.** In `/boot/firmware/config.txt`:
  `dtparam=i2c_arm_baudrate=50000`. Reboot.
- **Use software I2C**, which has no such bug, by adding
  `dtoverlay=i2c-gpio,i2c_gpio_sda=23,i2c_gpio_scl=24` and passing that
  adapter's number as `bus=`.

## Hello, expander

```python
from bbr_digital_expander import BBRDigitalExpander

with BBRDigitalExpander(bus=1) as expander:
    expander.print_device_info()

    print(expander.encoder_count(0), "counts")
    print(expander.distance_mm(1), "mm")
    if expander.sees_color(2, 1):
        print("port 2 sees colour 1")
```

`begin()` runs on the way into the `with` block and refuses to operate on a
wrong `DEVICE_ID`, an unsupported protocol major or an unrecognised hardware
variant — a clear failure at setup beats corrupt data an hour later.

## Two tiers

The **everyday tier** is one call per question, with no bitmasks and no
register knowledge: `encoder_count(0)`, `distance_mm(1)`, `sees_color(2, 1)`,
`heading()`, `teach_color(0, 3)`, `trigger_when_near(0, 1, 300)`.

The **advanced tier** underneath is the full register map: `read_telemetry()`,
`read_imu()`, `configure_output()`, `set_localizer_params()`, and raw
`read_registers()` / `write_registers()` when you want to go straight at it.

The same two tiers, the same helper names and the same refusals exist in the
FTC (Java) and Arduino (C++) drivers, so learning one teaches you the others.

## Failure is an exception

Anything that can fail raises a subclass of `BBRError`, carrying a sentence
that names the failure and, where there is one, the fix:

```python
from bbr_digital_expander import BBRError, NoImuError

try:
    print(expander.heading())
except NoImuError:
    print("base variant — no IMU fitted")
except BBRError as exc:
    print(f"expander: {exc}")
```

A snapshot read that never reached the device is the one exception to
"failure is an exception". Telemetry, IMU and localizer reads serve the last
good snapshot instead of raising — a bus being torn down mid-transfer, or a
burst of noise that corrupts three localizer blocks in a row, should not kill
a running program. `is_data_fresh()` reports whether the last read was real.
Fail-loud survives in two places: a device that has never answered raises
`TransportError`, and a streak that keeps failing for over half a second
(5+ reads) raises `BBRError`. The FTC driver behaves identically.

There are exactly two sentinels rather than exceptions, both deliberate and
both shared with the other drivers:

- `distance_mm(port)` is `math.inf` when nothing is in range, so
  `distance_mm(port) < 300` is always safe to write.
- `color_class(port)` is `0` for "nothing I was taught" — a dark, saturated,
  stale or unplugged sensor reads 0, never a confident wrong answer.

## Examples

Every one runs as-is on a Pi: `python3 examples/device_info.py`.

| Example | What it shows |
|---|---|
| `device_info.py` | Is it there, and what is it? Run this first. |
| `sensor_dump.py` | Everything the board knows, in one bus transaction. |
| `encoder_read.py` | Four encoders counted on the board, plus the pin-state diagnostic. |
| `pwm_encoder.py` | Absolute encoders that report position as a pulse width. |
| `color_read.py` | Which taught colour the sensor is looking at. |
| `color_teach.py` | Teach a colour by showing it one. Run once, on the field. |
| `distance_read.py` | Millimetres, and why "nothing in range" is infinity. |
| `heading_imu.py` | Heading, and what to do when the gyro clips. |
| `latched_output.py` | Catching an event too brief for your loop to see. |
| `trigger_setup.py` | Teach the board to watch a condition for you. Run once. |
| `trigger_runtime.py` | The other half: a GPIO read, no I2C at all. |
| `localizer.py` | Fused dead-wheel odometry — where the robot is on the field. |

## Tests

```sh
python3 -m unittest discover -s tests -t .
```

They run against a fake bus, so they need no hardware. They check the wire
format and the refusals — the parts a driver gets subtly wrong (endianness,
sign, scale factors, window arithmetic) and the parts that exist so a fault is
loud rather than plausible.

## Licence

MIT. See `LICENSE`.
