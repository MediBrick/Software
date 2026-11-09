# MediBrick ECG and Bio-Impedance Firmware (MAX30001G)

Firmware for an ESP32-S3 (compiled for the Adafruit Feather ESP32-S3) with the MAX30001G ECG and bio-impedance front end over SPI, a SSD1306 OLED, a MAX17048 battery gauge, a button and Bluetooth serial.
It is an interactive program: all settings are changed with text commands over USB or Bluetooth and can be saved to flash.

## Installation

Open `MAX30001G_Firmware/MAX30001G_Firmware.ino` in the Arduino IDE (or use `arduino-cli`) and flash it to the ESP32-S3.

The libraries can be installed with the scripts in the top folder of this repository ([Unix Shell](/install_libraries.sh), [Windows Power Shell](/install_libraries.ps1)). They install:

- [UUtzinger MAX30001G](https://github.com/uutzinger/Arduino_MAX30001G) (the chip driver)
- [UUtzinger BLESerial](https://github.com/uutzinger/Arduino_BLESerial), [RingBuffer](https://github.com/uutzinger/Arduino_RingBuffer) and [logger](https://github.com/uutzinger/logger)
- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino)
- [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO), [GFX Library](https://github.com/adafruit/Adafruit-GFX-Library), [SSD1306](https://github.com/adafruit/Adafruit_SSD1306) and [MAX1704X](https://github.com/adafruit/Adafruit_MAX1704X)

Board: Adafruit Feather ESP32-S3 (`esp32:esp32:adafruit_feather_esp32s3`). The pins are defined at the top of the sketch: chip select 6, interrupts INT1B 12 and INT2B 13, button 9.
Recommended board options: "USB Mode: Hardware CDC and JTAG" and "USB CDC On Boot: Enabled". With this setting the firmware can tell whether a USB host is plugged in, which the low battery shutdown uses.

Compile check:

```sh
arduino-cli compile --fqbn esp32:esp32:adafruit_feather_esp32s3 MAX30001G_Firmware
```

## Features

- ECG (2 or 3 leads), bio-impedance (BIOZ) at one frequency, both together, internal and external calibration modes, and impedance spectroscopy (frequency sweep).
- Optional R-to-R detection with heart rate, and a 50/60 Hz notch filter for the ECG.
- Text data output on USB serial and/or Bluetooth serial (on by default, mode Fast, name `BLESerialDevice`).
- Settings are saved to flash (NVS) and restored at boot.
- Button: short press switches the OLED screen, long press (2 s, then release) goes to deep sleep, a press wakes up. **The button connects the pin to 3.3 V** (this board uses an active-high button, unlike the other MediBricks).
- Low battery: below 10 % the display shows "LOW" and a warning is logged. At 5 % or 3.4 V or less for 20 s, with no USB host plugged in and the battery not charging, the board goes into deep sleep.
- Bluetooth watchdog: if the BLE sender is stuck (a client is subscribed, the buffer is at least half full and nothing was sent for 5 s) BLE restarts and the client has to connect again.

## Usage

Connect the MediBrick over USB, open a serial terminal (Serial Monitor of the Arduino IDE or any terminal, 2000000 baud on boards with a UART bridge, native USB ignores the baud rate) and type `?` for the help screen.
Commands are lines: type the command and press Enter. The same commands work over Bluetooth, and the answer goes back on the link the command came from.

Typical session:

1. `m1` (ECG) or `m2` (BIOZ) or `m3` (both) selects the mode.
2. Change settings if needed (`s` shows the current ones), for example `Eg2`.
3. `>` starts the measurement, `z` toggles the data output, `<` stops.
4. `Ps` saves the settings so they are used after a reboot.

### Data output

Text lines, one per display time point:

- `ECG [mV]: v v v ...` all ECG samples since the last BIOZ interrupt, followed by the R-to-R and heart rate values (`RR [ms]: ...`, `HR [bpm]: ...`) when R-to-R detection is on.
- `BIOZ [Ohm]: v v v ...` the BIOZ samples.
- In combined mode both are on the same line. The calibration and scan modes use their own labels.

### Commands

Numbers follow the command letters directly, for example `Bf8000`. Defaults are in the help screen (`?`).

**General**

| Command | Function |
|---|---|
| `?` | Help screen |
| `s` | Show current settings |
| `a` | Apply the current settings again (re-setup of the chip) |
| `h` | Health check |
| `i` | Device info |
| `r` / `t` / `p` | Print all registers / status registers / config registers |
| `x` | Reboot |

**Data and start/stop**

| Command | Function |
|---|---|
| `z` | Toggle data reporting on/off |
| `zs` / `zb` / `za` | Data reporting off on USB / BLE / both |
| `zS` / `zB` / `zA` | Data reporting on on USB / BLE / both |
| `.` | Toggle start/stop |
| `>` / `<` | Start / stop the measurement |
| `c` | Reset the sample counter |
| `f` | FIFO reset |

**Mode** (stops the previous measurement)

| Command | Function |
|---|---|
| `m1` | ECG |
| `m2` | BIOZ |
| `m3` | ECG + BIOZ |
| `m4` / `m5` | ECG / BIOZ signal calibration (internal test signal) |
| `m6` | BIOZ internal impedance calibration |
| `m7` | BIOZ external impedance calibration |
| `m8` | BIOZ impedance spectroscopy |

**Persistence**

| Command | Function |
|---|---|
| `Ps` | Save settings to flash |
| `Pl` | Load settings from flash |
| `Pd` | Print the saved settings |
| `Pc` | Clear the saved settings |
| `(` / `)` | Save / restore a snapshot of the settings in RAM |

**ECG settings**

| Command | Function |
|---|---|
| `Es<0-2>` | Speed |
| `Eg<0-3>` | Gain |
| `El<0-3,255>` | Digital low-pass filter (255 = default) |
| `Eh<0-1,255>` | Digital high-pass filter |
| `Ee<2\|3>` | Number of leads |
| `Er<0\|1>` | R-to-R detection off / on |
| `En<0\|50\|60>` | Notch filter off / 50 Hz / 60 Hz |
| `Eq<1-100>` | Notch filter Q |

**BIOZ settings**

| Command | Function |
|---|---|
| `Bs<0-1>` | Speed |
| `Bg<0-3>` | Gain |
| `Ba<0-7>` | Analog high-pass |
| `Bd<0-3>` / `Bh<0-3>` | Digital low-pass / high-pass |
| `Bf<Hz>` | Excitation frequency |
| `Bc<nA>` | Excitation current |
| `Bp<deg>` | Phase |
| `Bl<0\|1>` | Lead bias off / on |
| `Bo<0\|1>` | Lead-off detection off / on |
| `Bw<2\|4>` | 2 or 4 wire measurement |

**BIOZ scan (impedance spectroscopy) and calibration settings**

| Command | Function |
|---|---|
| `Sa<1-8>` | Averages |
| `Sf<0\|1>` | Fast mode |
| `Sr<0\|1>` | Full range |
| `Si<0\|1>` | Source: 0 external, 1 internal |
| `Sp<0\|1>` | Phase range: 0 full |
| `Sh<0-2>` | High-pass mode: 0 dynamic, 1 fixed, 2 bypass |
| `Sv<0-7>` / `Sx<255,0-7>` | Fixed / internal high-pass setting |
| `St<1-64>` / `Sc<1-64>` | Settling time / current settling time |
| `Sm<nA>` | Maximum current |
| `So<0\|1>` | Autorange |
| `Sy<0\|1>` | Human-safe current limit |
| `Kp` / `Kr` | Print / reset the scan calibration |
| `Kg<ppm>` / `Ke<0\|1>` | Set the global correction / enable the correction |
| `Cr<Ohm>` | Internal calibration resistor |
| `Cm<0-3>` / `Cf<0-4>` | Calibration modulation / modulation frequency |
| `Ce<0\|1>` / `Cb<0\|1>` | ECG / BIOZ signal mode for calibration |

**Log level and special**

| Command | Function |
|---|---|
| `l0` ... `l5` | Log level: 0 debug, 1 info (default), 2 warnings, 3 errors, 4 critical, 5 none |
| `w` | Chip software reset |
| `y` | Chip synchronize |
| `k` | Clear the latched status flags |

**Bluetooth serial** (on by default)

| Command | Function |
|---|---|
| `Lb` / `LB` | Stop / start BLE now. `Ps` saves it as the boot setting |
| `Ln<name>` | BLE name; `Ps`, then `Lb` and `LB` to apply |
| `Lm0` | Fast: 7.5 ms link, +9 dBm (default) |
| `Lm1` | LowPower: 60-120 ms, -6 dBm |
| `Lm2` | LongRange: coded PHY, +9 dBm |
| `Lm3` | Balanced: 15-30 ms, 0 dBm |
| `Lt<dBm>` | BLE transmit power, -24 to +9 dBm |
| `Lta` | Default power of the BLE mode |
| `Lbs` | Status of the BLE library (printed on USB) |

Changing the mode or `Lta` restarts BLE, so the Bluetooth client has to connect again.
