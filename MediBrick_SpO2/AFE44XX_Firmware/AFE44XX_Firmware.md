# MediBrick Pulse Oximeter Firmware (AFE4490 / AFE4400)

Firmware for an ESP32 (compiled for the Adafruit Feather ESP32-S3) with the AFE4490 or AFE4400 pulse oximeter front end over SPI, a SSD1306 OLED, a MAX17048 battery gauge, a button and Bluetooth serial.
It is an interactive program: the settings of the receiver, the LED transmitter and the timing are changed with text commands over USB or Bluetooth and can be saved to flash.

## Installation

Open `AFE44XX_Firmware/AFE44XX_Firmware.ino` in the Arduino IDE (or use `arduino-cli`) and flash it to the ESP32-S3.

The libraries can be installed with the scripts in the top folder of this repository ([Unix Shell](/install_libraries.sh), [Windows Power Shell](/install_libraries.ps1)). They install:

- [UUtzinger AFE44XX](https://github.com/uutzinger/Arduino_AFE44XX) (the chip driver)
- [UUtzinger BLESerial](https://github.com/uutzinger/Arduino_BLESerial), [RingBuffer](https://github.com/uutzinger/Arduino_RingBuffer) and [logger](https://github.com/uutzinger/logger)
- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino)
- [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO), [GFX Library](https://github.com/adafruit/Adafruit-GFX-Library), [SSD1306](https://github.com/adafruit/Adafruit_SSD1306) and [MAX1704X](https://github.com/adafruit/Adafruit_MAX1704X)

The pins are defined at the top of the sketch: reset 14, data ready (DRDY) 12, chip select 15, photodiode alarm 32, LED alarm 26, diagnostics end 27, power down 25, button 4. Check that they match your wiring.
Recommended board options: "USB Mode: Hardware CDC and JTAG" and "USB CDC On Boot: Enabled". With this setting the firmware can tell whether a USB host is plugged in, which the low battery shutdown uses.

Compile check:

```sh
arduino-cli compile --fqbn esp32:esp32:adafruit_feather_esp32s3 AFE44XX_Firmware
```

## Features

- Red and infrared photoplethysmography with ambient light correction, read at the data ready interrupt of the chip.
- Settings for the receiver (stage 2 gain, feedback capacitor and resistor, filter, ambient current), the LED transmitter (reference voltage, current range, LED power), the timing, averaging and the alarm pins.
- Text data output on USB serial (500000 baud on boards with a UART bridge, native USB ignores it) and/or Bluetooth serial (on by default, mode Fast, name `BLESerialDevice`).
- Settings are saved to flash (NVS) and restored at boot.
- Button: short press switches the OLED screen, long press (2 s, then release) goes to deep sleep, a press wakes up. The button connects the pin to GND.
- Low battery: below 10 % the status screen shows "LOW BATTERY" and a warning is logged. At 5 % or 3.4 V or less for 20 s, with no USB host plugged in and the battery not charging, the board goes into deep sleep.
- Bluetooth watchdog: if the BLE sender is stuck (a client is subscribed, the buffer is at least half full and nothing was sent for 5 s) BLE restarts and the client has to connect again.

## Usage

Connect the MediBrick over USB, open a serial terminal and type `?` for the help screen.
Commands are lines: type the command and press Enter. The same commands work over Bluetooth, and the answer goes back on the link the command came from.

Typical session:

1. `s` shows the current settings.
2. Change settings if needed, for example `LP1200` for the LED 1 power.
3. `>` starts the measurement, `z` toggles the data output, `<` stops.
4. `Ps` saves the settings so they are used after a reboot.

### Data output

One text line per sample, with tabs between the values:

```
IR:    <ir>	Red:    <red>	IR_a:    <ir ambient>	Red_a:    <red ambient>	S/s: <samples per second>
```

`IR` and `Red` are the values after the ambient light correction, `IR_a` and `Red_a` are the ambient light values, `S/s` is the measured sample rate.

### Commands

Numbers follow the command letters directly, for example `Ma8`. The help screen (`?`) lists the ranges.

**General**

| Command | Function |
|---|---|
| `?` | Help screen |
| `s` | Show current settings |
| `a` | Apply the current settings again (re-setup of the chip) |
| `h` | Health check |
| `i` | Device info |
| `r` / `p` | Print all registers / timing registers |
| `t` | Print the status (diagnostics) |
| `x` | Reboot |

**Data and start/stop**

| Command | Function |
|---|---|
| `z` | Toggle data reporting on/off |
| `zs` / `zb` / `za` | Data reporting off on USB / BLE / both |
| `zS` / `zB` / `zA` | Data reporting on on USB / BLE / both |
| `zz` | Print one sample |
| `.` | Toggle start/stop |
| `>` / `<` | Start / stop the measurement |
| `c` | Reset the sample counter |

**Persistence**

| Command | Function |
|---|---|
| `Ps` | Save settings to flash |
| `Pl` | Load settings from flash |
| `Pd` | Print the saved settings |
| `Pc` | Clear the saved settings |
| `(` / `)` | Save / restore a snapshot of the settings in RAM |

**Acquisition**

| Command | Function |
|---|---|
| `Ma<1-16>` | Averages |
| `Mt<0.1-100>` | Timing factor |
| `Mp<0-7>` | Alarm pins |

**Receiver**

| Command | Function |
|---|---|
| `Rg1<-1..4>` / `Rg2<-1..4>` | Stage 2 gain LED 1 / LED 2 |
| `RC1<pF>` / `RC2<pF>` | Feedback capacitor LED 1 / LED 2 |
| `RR1<kOhm>` / `RR2<kOhm>` | Feedback resistor LED 1 / LED 2 |
| `RFC<0\|1>` | Filter edge: 0 = 500 Hz, 1 = 1000 Hz |
| `RFB<0-9>` | Ambient background current in uA |

**Transmitter**

| Command | Function |
|---|---|
| `TV<0-3>` | Reference voltage |
| `TC<0-3>` | Current range |
| `LP1<0-255>` / `LP2<0-255>` | LED 1 / LED 2 power |

**Hardware options and power**

| Command | Function |
|---|---|
| `Xt1` ... `Xt4` | Toggle: same gain for LED 1 and 2, tri state, H bridge, bypass ADC |
| `Xt5` / `Xt6` / `Xt7` | Toggle the power of the receiver / transmitter / whole chip |

**Log level and special**

| Command | Function |
|---|---|
| `l0` ... `l5` | Log level: 0 debug, 1 info (default), 2 warnings, 3 errors, 4 critical, 5 none |
| `w` | Chip software reset |

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
