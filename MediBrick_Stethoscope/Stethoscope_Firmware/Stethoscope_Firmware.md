# MediBrick Stethoscope and Blood Pressure Firmware (v1.3)

Firmware for the ESP32-S3 Feather (Thing Plus C also supported) with the ES8388 audio codec, MPRLS pressure sensor, SSD1306 OLED, MAX17048 battery gauge, a button and Bluetooth serial.
Design decisions, the command map and the hardware test tables are in [Design.md](Design.md).

## Installation

Open `Stethoscope_Firmware/Stethoscope_Firmware.ino` in the Arduino IDE (or use `arduino-cli`) and flash it to the ESP32-S3.

The libraries can be installed with the scripts in the top folder of this repository ([Unix Shell](/install_libraries.sh), [Windows Power Shell](/install_libraries.ps1)). They install:

- [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO)
- [Adafruit GFX Library](https://github.com/adafruit/Adafruit-GFX-Library)
- [Adafruit MAX1704X](https://github.com/adafruit/Adafruit_MAX1704X)
- [Adafruit MPRLS Library](https://github.com/adafruit/Adafruit_MPRLS)
- [Adafruit SSD1306](https://github.com/adafruit/Adafruit_SSD1306)
- [arduino-audio-driver](https://github.com/pschatzmann/arduino-audio-driver) (pschatzmann, ES8388 codec driver)
- [UUtzinger BLESerial](https://github.com/uutzinger/Arduino_BLESerial), [RingBuffer](https://github.com/uutzinger/Arduino_RingBuffer) and [logger](https://github.com/uutzinger/logger)

The audio is handled with the ESP-IDF `i2s_std` driver, `arduino-audio-tools` is not needed.

Board: Adafruit Feather ESP32-S3 (`esp32:esp32:adafruit_feather_esp32s3`) or SparkFun Thing Plus C (`esp32:esp32:esp32thing_plus_c`). Check that the pin map at the top of the sketch matches your wiring.
Recommended board options: "USB Mode: Hardware CDC and JTAG" and "USB CDC On Boot: Enabled". With this setting the firmware can tell whether a USB host is plugged in, which the low battery shutdown uses.

Compile check:

```sh
arduino-cli compile --fqbn esp32:esp32:adafruit_feather_esp32s3 Stethoscope_Firmware
```

## Features

- Stethoscope audio from the electret microphones (Line 1), a jack microphone (Line 2), or simulated tones and noise, to the line out or the headphone jack.
- Blood pressure readings in mmHg, shown on the OLED.
- Audio data (left, right) and pressure over USB serial and/or Bluetooth serial as text lines.
- Audio chain on core 1: high-pass and low-pass filter, software gain, optional subtraction of a background microphone (R - g*L), selectable sample rate, click-free start/stop with fades.
- Settings are saved to flash (NVS) and restored at boot.
- Button: short press switches the OLED screen, long press (2 s, then release) goes to deep sleep, a press wakes up. The button connects the pin to GND.
- Auto shutoff after 4.5 hours.
- Low battery warning below 10 %. Shutdown (deep sleep) at 5 % or 3.4 V or less, for 20 s, when no USB host is plugged in and the battery is not charging.
- Bluetooth stall watchdog: if the BLE sender is stuck (subscribed, buffer at least half full, nothing sent for 5 s) BLE restarts and the client has to connect again.

## Usage

Connect the MediBrick to a computer over USB, open a serial terminal (Serial Monitor of the Arduino IDE, or any terminal; 500000 baud applies to UART boards only, native USB ignores the baud rate) and type `?` for the help screen. Commands are lines: type the command and press Enter. Commands also work over Bluetooth, and the answer goes back on the link the command came from.

By default the board runs in mode 3 (audio and pressure). The pressure is zeroed when the pressure mode starts, so keep the cuff open to air at that moment.  Turn data reporting on and off with `z`; the `s` command shows the current state of all settings.

### Data output

Text lines, one per output sample, each ending with a single newline. Each signal keeps its column within a mode; a line may have fewer numbers than another (the receiving program must accept that).

| Mode | Audio only | With a new pressure reading | Pressure only (`m1`) |
|---|---|---|---|
| `N0` | `left,right` | `left,right,pressure` | `pressure` |
| `N1` | `result` | `result,pressure` | `pressure` |
| `N2` | `left,right` | `left,right,pressure` | `pressure` |

- `left` and `right` are the filtered channels. With `N1` only the subtracted result is sent (`R - Ng*L + Nb`, clipped to 16 bit, what both ears hear), which halves the data rate. With `N2` `left` is the filtered background (not subtracted) and `right` is the result.
- The pressure (mmHg, one decimal) is added to the next line after a reading, about 10 times per second.
- `La<n>` sends every n-th audio frame and `Lp<n>` every n-th pressure value, to limit the data rate. Raise `La` when a link drops lines (the `i` command shows the dropped lines and kB/s per link).

### Commands

Numbers are typed directly behind the command letters, for example `Fh200`, `Gs2`, `R16`.

**General**

| Command | Function |
|---|---|
| `?` | Help screen |
| `s` | Show current settings |
| `i` | Show system info and limits (versions, heap, audio timing, link rates and dropped lines, battery) |
| `x` | Reboot |

**Data and start/stop**

| Command | Function |
|---|---|
| `z` | Toggle data reporting on/off |
| `zs` / `zb` / `za` | Data reporting off on USB / BLE / both |
| `zS` / `zB` / `zA` | Data reporting on on USB / BLE / both |
| `.` | Toggle start/stop of the measurement |
| `>` / `<` | Start / stop the measurement |
| `A0` / `A1` | Autostart at boot off / on |

**Mode**

| Command | Function |
|---|---|
| `m0` | Audio and pressure off (battery, display, button and commands stay on) |
| `m1` | Pressure only |
| `m2` | Audio only |
| `m3` | Both |

**Persistence**

| Command | Function |
|---|---|
| `Ps` | Save settings to flash |
| `Pl` | Load settings from flash |
| `Pd` | Print the saved settings |
| `Pc` | Clear the saved settings |
| `Pr` | Reset to factory defaults |

**Input and output line**

| Command | Function |
|---|---|
| `a1` | Input Line 1 (electret microphones with amplifier) |
| `a2` | Input Line 2 (jack microphone) |
| `a3` | Simulated tones, left and right |
| `a4` | Simulated tones plus white noise |
| `o1` | Output Line 1 (line out) |
| `o2` | Output Line 2 (headphone) |

**Filter, gain and sample rate** (engineering)

| Command | Function |
|---|---|
| `FH<Hz>` | High-pass on, set cutoff |
| `Fh` / `Fh<Hz>` | High-pass off / set cutoff only |
| `Fh+` / `Fh-` | Step the high-pass cutoff |
| `FL<Hz>` | Low-pass on, set cutoff |
| `Fl` / `Fl<Hz>` | Low-pass off / set cutoff only |
| `Fl+` / `Fl-` | Step the low-pass cutoff |
| `Gi<0-8>`, `Gi+`, `Gi-` | Hardware input gain (3 dB steps) |
| `Go<0-100>`, `Go+`, `Go-` | Hardware output (headphone/line) volume |
| `Gs<x>`, `Gs+`, `Gs-` | Software gain |
| `R<8\|16\|32\|48>` | Sample rate in kHz (filters and tones follow) |

**Background subtraction** (engineering)

Result = R - Ng*L + Nb, with L the background microphone (left) and R the stethoscope microphone (right).

| Command | Function |
|---|---|
| `N0` | Subtraction off |
| `N1` | Result in both ears |
| `N2` | Left original (filtered), result in the right ear |
| `Ng<x>` | Gain on the left (background) channel |
| `Nb<x>` | Bias in counts (acts with `N1` and `N2`) |

**Simulation** (inputs `a3` and `a4`)

| Command | Function |
|---|---|
| `Tl<Hz>[,<amp>]` | Left tone |
| `Tr<Hz>[,<amp>]` | Right tone |
| `Tn<amp>` | White noise on both channels, amplitude 0 to 32767 counts |

**Pressure** (engineering)

mmHg = (reading - zero) / Qc. The reading is low-pass filtered (first order) and arrives 10 times per second.

| Command | Function |
|---|---|
| `QF` / `Qf` | Pressure low-pass on / off |
| `QF<Hz>` | Set the cutoff (0.1 to 4 Hz) and switch the low-pass on |
| `Qf<Hz>` | Set the cutoff only |
| `Qc<x>` | Calibration factor, 0.5 to 2 |
| `Qz` | Zero now (cuff open to air) |

**Links and log**

| Command | Function |
|---|---|
| `La<n>` | Audio output decimation (send every n-th frame) |
| `Lp<n>` | Pressure output decimation |
| `l<0-5>` | Log level: 0 debug ... 5 none |
| `Ls<baud>` | Baud rate (UART boards only) |

**Bluetooth serial** (off by default in this firmware)

| Command | Function |
|---|---|
| `Lb` / `LB` | Stop / start BLE now. `Ps` saves it as the boot setting |
| `Ln<name>` | BLE name; `Ps`, then `Lb` and `LB` to apply |
| `Lm0` | Fast: 7.5 ms link, +9 dBm |
| `Lm1` | LowPower: 60-120 ms, -6 dBm |
| `Lm2` | LongRange: coded PHY, +9 dBm |
| `Lm3` | Balanced: 15-30 ms, 0 dBm (default) |
| `Lt<dBm>` | BLE transmit power, -24 to +9 dBm |
| `Lta` | Default power of the BLE mode |
| `Lbs` | Status of the BLE library (printed on USB) |

Notes:

- The BLE radio bursts can be heard or seen in the microphone signal. Record audio with BLE off (`Lb`).
- The mode restarts BLE, so the Bluetooth client has to connect again.
- Rough BLE limits: In the hardware tests Balanced carried about 10 kB/s and Fast up to about 90 kB/s. Use `La` to stay below the limit.
