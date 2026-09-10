Pressure sensor: AHT20 + BMP280 module  
Screen: Lcd12864 12864-06D (GMG12864-06d) or nokia 3310 screen  
GPS: GYNEO6-MV2 (big antenna) or GPS + BDS ATGM336H (substitute for NEO-M8N or NEO-M6N)  
IMU: LSM6DS3 or GY-BMI160  

## Wiring for the current firmware

`Vario/Vario.ino` targets the **Arduino MKR Zero**. Only the barometer, the IMU
and the buzzer are needed to test it — the screen, the GPS and SD logging are
not used yet.

Everything runs at **3.3 V**. Do not connect these sensors to 5 V.

### Barometer and IMU — one shared I2C bus

| MKR Zero | AHT20 + BMP280 | LSM6DS3 |
| --- | --- | --- |
| VCC (3.3 V) | VCC | VCC |
| GND | GND | GND |
| D11 (SDA) | SDA | SDA |
| D12 (SCL) | SCL | SCL |

Both modules hang off the same two pins, in parallel. The firmware probes the
BMP280 at 0x76 then 0x77, and the LSM6DS3 at 0x6A then 0x6B, so either address
strap works either way round; the AHT20 at 0x38 is simply ignored. Most
breakouts carry their own pull-ups and two sets in parallel is still fine at
400 kHz.

The IMU can be mounted at **any fixed angle** — the attitude is seeded from
gravity at power-up — but it must be rigidly fixed to the same body as the
barometer, and held still while it calibrates.

### Buzzer

```
D5 ──[ 100 Ω ]── buzzer ── GND
```

Use a passive buzzer or a piezo. A magnetic buzzer needs a transistor; the pin
cannot drive it. At 3.3 V a piezo is noticeably quieter than it was on the 5 V
Nano, so for real flying drive it from a small MOSFET on the 5 V rail.

### Pins left alone for the planned additions

| Pins | Reserved for |
| --- | --- |
| D8 / D9 / D10 | SPI MOSI / SCK / MISO — the GMG12864-06d screen |
| D13 / D14 | Serial1 RX / TX — the GPS (D13 is RX, so it goes to the GPS **TX**) |
| onboard SD | SPI1 on internal pins 26/27/29 plus `SDCARD_SS_PIN`; uses no header pins |
| D0–D4, D6, D7, A0–A6 | free — screen CS/DC/RST, buttons |

`LED_BUILTIN` on the MKR Zero is pin 32 and is internal, so D6 really is free.

### Board support and libraries

- Board: **Arduino SAMD Boards** (`arduino:samd`), then select *Arduino MKR Zero*
- **Adafruit BMP280 Library**
- **Adafruit LSM6DS**
- **Adafruit BusIO** and **Adafruit Unified Sensor** (dependencies of the two above)

NewTone is no longer used: it writes ATmega timer registers directly and cannot
build for the SAMD21. The audio uses the SAMD core's own `tone()` / `noTone()`.

### What the startup sounds mean

| Sound | Meaning |
| --- | --- |
| 800 Hz then 1200 Hz | ready, filter seeded |
| one short 1000 Hz chirp | hold still, gyro calibration starting (2 s) |
| two short 400 Hz beeps | movement detected, calibration retrying |
| two long 300 Hz beeps | IMU not found — running barometer-only, still flyable |
| three 1200 Hz chirps, repeating forever | barometer not found, nothing works without it |

### Bench test

Set `DEBUG_SERIAL` to `1` in `Vario/Vario.ino` and open the Serial Plotter at
115200 baud to watch `alt`, `vel` and `bias`. Sitting still, `vel` should settle
near zero and stay silent; lifting the board a metre should give a short burst
of beeps whose pitch rises with how fast you lift it, and lowering it quickly
should trip the sink tone.
