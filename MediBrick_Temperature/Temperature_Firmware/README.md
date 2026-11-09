# MediBrick Thermistor Board Firmware

Firmware for the MediBrick 3 channel thermistor board ([Thermistor_Brick](https://github.com/MediBrick/Thermistor_Brick)).
It reads up to three thermistors, each in a Wheatstone bridge, with the microcontroller's ADC and streams the result as text over USB serial.
The instrument is configured with single line commands from a serial terminal. Settings can be stored in the microcontroller's non volatile memory.

Sketch: [Temperature_Firmware.ino](Temperature_Firmware.ino)

## Device information

### Supported boards

The board is selected automatically from the board you compile for.

| | ESP32 (SparkFun Thing Plus C, WROOM) | ESP32-S3 (Adafruit Feather ESP32-S3) |
|---|---|---|
| Thermistors | 3 | 2, 3 only with I2C turned off |
| ADC channels | 6 | 4, 6 only with I2C turned off |
| Default ADC channels | 6 | 4 |
| Total ADC sampling rate (all channels together) | 20 kHz to 2 MHz | 611 Hz to 83.3 kHz |
| Bytes per conversion result in a DMA frame | 2 | 4 |
| Largest DMA frame | 4092 bytes = 2046 samples | 4092 bytes = 1023 samples |
| Bits per conversion | 12 | 12 |
| USB | USB-UART bridge, the baud rate is the real line rate | native USB, the baud rate is ignored |

On the ESP32-S3 the third thermistor uses GPIO4, which is the I2C clock line (SCL) of the STEMMA QT connector. It can only be used when I2C is turned off (`w0`).

Each thermistor needs two ADC channels, the A and B side of its Wheatstone bridge. The firmware reports the voltage difference B-A.
ADC channel numbers are not pin numbers, the pin assignment is described in [Thermistor_Brick/assembly.md](https://github.com/MediBrick/Thermistor_Brick/blob/main/assembly.md).

### Output

One line per output sample, values of the thermistors separated by commas. Data goes to USB serial and/or BLE serial (see `z`).
Commands are accepted on both links, the answer goes back to the link the command came from.

* Temperature mode (`m2`, default): temperature in degrees Celsius with one decimal, e.g. `36.4, 35.9, 22.1`.
  The calibration is only valid between 0 and 50 C, outside this range or if the calculation fails the value is `nan`.
* Resistance mode (`m1`): resistance of each thermistor in Ohm, calculated from the bridge voltage, supply voltage and bridge resistors, e.g. `5986, 6120, 10031`.
  Useful to check the bridge setup: with a known resistor in place of the thermistor the output should match its value.
* Voltage mode (`m0`): voltage difference of each bridge in mV, e.g. `-12, 8, 301`.

### Battery, display and button

With I2C on (`w1`) the firmware looks for a MAX17048 battery gauge and a 128x32 SSD1306 OLED (address 0x3C) on the STEMMA QT / QWIIC connector. Both are optional. With I2C off (`w0`) they are switched off because the I2C pins are ADC inputs.

* Screen 1 (default): battery level, charge rate in %/hr and a charge/discharge indicator.
* Screen 2: run state, unit and the latest value of every thermistor. While measuring it refreshes twice per second, a refresh takes about 12 ms of I2C time.
* The button connects to GND. Short press switches the screen, a long press (2 s, acted on at release) puts the board into deep sleep, pressing it again wakes the board (it reboots).
  Button pin: GPIO27 on the SparkFun Thing Plus C (A10), GPIO14 on the Adafruit Feather ESP32-S3 (A4). Both are RTC pins that can wake from deep sleep.
* `s` shows the battery level and charge rate.

### Limitations

The ADC runs continuously and every output sample is the average of `bin size` ADC samples ("binning"). The limits that follow from this are:

* **ADC rate.** `channels x output rate x bin size` is the total ADC sampling rate. It must be within the range of the chip (table above).
  A high output rate therefore needs a small bin size, a low output rate a large one. Averaging more samples reduces noise.
* **Frame size.** `channels x bin size` must fit in one DMA frame. This limits the bin size to 1023 / channels on the ESP32-S3 and 2046 / channels on the ESP32.
* **Data link.** The output rate is also limited by how many bytes per second the link can carry. This depends on the line length, which is about 8 characters per thermistor plus 2.
  Conservative defaults are used for the sustainable rate of each link and can be changed (`Lu`, `Lr`):

  | Link | ESP32-S3 | ESP32 |
  |---|---|---|
  | USB | 200 kBytes/s | 40 kBytes/s (80% of the UART baud rate) |
  | BLE | 30 kBytes/s | 25 kBytes/s |

  These are about a quarter of what was measured on an ESP32-S3 streaming test program (USB 870 kBytes/s, BLE 129 kBytes/s). The ESP32 BLE value is an estimate, it was not measured.
  The limit of every enabled data output applies (`z`). If both USB and BLE are on, the lower of the two is used.
  If a link can not keep up (no client, slow connection) its buffer fills and complete lines are dropped. Dropped lines are counted per link and shown by `i`.
* **ADC accuracy.** The ESP32 ADC is not very linear. Calibration values stored in the chip are applied, and averaging reduces noise.
* Changing channels, output rate, bin size or attenuation restarts the ADC. Samples in progress are discarded.

With the factory settings the largest output rate is about 11,000 Hz on the ESP32-S3 (4 channels) and about 1,500 Hz on the ESP32 (6 channels), the lowest about 1 Hz and 10 Hz.
The program computes the exact numbers for your configuration, use `i` to see them.

## Setting up the instrument

Connect with a serial terminal at 500000 baud (the baud rate is ignored on the ESP32-S3). Commands are lines ending with Enter. Send `?` for the command menu.
The sequence below goes from the measurements on the board to the finished configuration. Do it once for each board and store the result (step 7).

Output keeps streaming while you type. Send `<` to stop it while you configure and `>` to start it again (`.` toggles).
Messages and answers to commands are written directly to the link and can appear in the middle of a data line while streaming.
Stop the stream while you configure, or set `l5` to silence the log messages.

### 1. Bridge supply voltage

Measure the supply voltage of the bridge with a multimeter, it is about 3.3 V, and enter it in mV:

```
v3320
```

If you did not measure it, the nominal 3.3 V (`v3300`) can be used. An error here directly scales into a temperature error.

### 2. Bridge resistors

The bridge resistors are nominally 10 kOhm. For an accurate temperature measure the three resistors of each thermistor with a multimeter at the test points on the PCB (the default values were measured on the board before it was assembled):

| Resistor | Test points |
|---|---|
| R1 | TP1 - TP3 |
| R2 | TP1 - TP2 |
| R3 | TP2 - TP3 |

R3 is the one that matters most for the result, measure it carefully (see [assembly.md](https://github.com/MediBrick/Thermistor_Brick/blob/main/assembly.md)).
Enter the values in Ohm, one command for each thermistor (1 to 3):

```
r1,9960,9970,9960
r2,9950,9990,9970
r3,10000,9980,10030
```

The format is `r<thermistor>,<R1>,<R2>,<R3>`. If you did not measure them, use 10000 for all three.

### 3. Steinhart-Hart coefficients

The resistance of the thermistor is converted to temperature with the Steinhart-Hart equation

```
1/T = A + B ln(R) + C ln(R)^3
```

with R in Ohm and T in Kelvin. The coefficients A, B and C come from the manufacturer's temperature / resistance table in the thermistor datasheet.
They are calculated with the spreadsheet [Steinhart-Hart inverse fit.ods](https://github.com/MediBrick/Thermistor_Brick/tree/main) in the Thermistor_Brick repository:
enter the temperature and resistance pairs from the datasheet and the spreadsheet fits A, B and C.
Fit the table over the range you will measure (the calibration of this firmware is meant for 0 to 50 C).

Enter the three numbers, plain or in exponent notation:

```
h0.001111285538,0.0002371215953,7.520676806e-8
```

The same coefficients are used for all thermistors, all of them must be the same type. The values are stored with single precision (about 7 digits).

### 4. ADC attenuation

The attenuation sets the input range of the ADC. Both sides of the bridge are measured against ground, so the range must cover the voltage at the bridge nodes, which is about half the supply (about 1.65 V) when the bridge is balanced.

| Command | Range | Accurate range of the ADC |
|---|---|---|
| `g0` | 0 to 1.1 V | 100 to 950 mV |
| `g1` | 0 to 1.5 V | 100 to 1250 mV |
| `g2` | 0 to 2.2 V | 150 to 1750 mV |
| `g3` (default) | 0 to 3.9 V | 150 to 2450 mV |

Use `g3` unless you have a reason not to. `g0` and `g1` are too small for a 3.3 V bridge. `g2` can work but the bridge nodes are close to the end of its accurate range.

### 5. Number of channels and I2C

Each thermistor uses two ADC channels: `c2` for one, `c4` for two, `c6` for three thermistors.
On the ESP32-S3, `c6` requires I2C to be turned off first (`w0`), because the third thermistor uses the I2C clock pin.

```
w0
c6
```

### 6. Output rate and bin size

Select the output rate in samples per second and thermistor, then the bin size (the number of ADC samples averaged for each output sample):

```
f50
n100
```

The total ADC rate `channels x output rate x bin size` must be inside the range of the chip. The program tells you if it is not and shows the valid ranges, for example:

```
Not changed: bin size is too large for this output rate and channel count
  Output rate 10..1538 Hz at 6 channels (USB limit: 1538 lines/s)
  Bin size    67..341 at 50 Hz output rate
```

* Changing the output rate (`f`), the channels (`c`) or the links (`u`, `l`, `b`) moves the bin size into the valid range automatically if it no longer fits. A message tells you the new bin size.
* `n` is checked strictly. A bin size that is not valid for the current output rate is refused.
* A larger bin size averages more samples and gives less noise. Select the largest bin size that your output rate allows. That means: select the output rate first, then `n` as large as accepted.

### 7. Data output

USB output is on by default. To receive the data over BLE use `zB` (BLE in addition to USB) or `zA` (USB and BLE). `zs` then switches USB off, which leaves BLE only.
The board advertises as `MediBrickThermistor` and works with any BLE serial (Nordic UART) client, e.g. the SerialUI program.
Data is only sent while a client is subscribed. Commands can be sent over BLE as well.

### 8. Check and store

Send `s` to show all settings and `i` to show the system information and the limits for the current settings. Check that the numbers are what you intended, then store them:

```
Ps
```

The stored settings are loaded when the board starts. Without `Ps` all changes are lost at reboot.

## Command reference

| Command | Description |
|---|---|
| `?` | help screen |
| `s` | show current settings |
| `i` | show system information and limits for the current settings |
| `.` | toggle start / stop |
| `>` / `<` | start / stop |
| `x` | reboot |
| `m0` / `m1` / `m2` | output voltage difference in mV / thermistor resistance in Ohm / temperature in C |
| `c2`, `c4`, `c6` | number of ADC channels (2 per thermistor) |
| `w0` / `w1` | I2C off / on (6 channels on the ESP32-S3 need I2C off) |
| `f<Hz>` | output rate per thermistor |
| `n<bin>` | ADC samples averaged per output sample |
| `g<0-3>` | ADC attenuation |
| `v<mV>` | bridge supply voltage |
| `r<th>,<R1>,<R2>,<R3>` | bridge resistors of thermistor 1 to 3 in Ohm |
| `h<A>,<B>,<C>` | Steinhart-Hart coefficients |
| `z` | toggle continuous data display on / off |
| `zs` / `zb` / `za` | switch off the data output to USB / BLE / both |
| `zS` / `zB` / `zA` | switch on the data output to USB / BLE / both (and turn the data display on) |
| `Lu<bytes/s>` | sustainable USB data rate, limits the output rate if USB output is on |
| `Lr<bytes/s>` | sustainable BLE data rate, limits the output rate if BLE output is on |
| `Lb` / `LB` | stop / start BLE now (`Ps` saves it as the boot setting; BLE is on by default) |
| `Ln<name>` | BLE name; `Ps`, then `Lb` and `LB` to apply it |
| `Lm<0-3>` | BLE mode: 0 Fast (default), 1 LowPower, 2 LongRange, 3 Balanced; BLE restarts |
| `Lt<dBm>` / `Lta` | BLE transmit power -24 to +9 dBm / default power of the mode |
| `Lbs` | status of the BLE library (printed on USB) |
| `l<0-5>` | log level: 0 debug, 1 info, 2 warning, 3 error, 4 critical, 5 none (debug messages also need `DEBUG` defined when compiling) |
| `Ps` | save settings to non volatile memory |
| `Pl` | load saved settings |
| `Pd` | print saved settings |
| `Pc` | clear saved settings, the factory defaults apply after the next reboot |
| `Pr` | load the factory defaults into the running settings now (use `Ps` to save them) |

## Factory defaults

| Setting | Default |
|---|---|
| Output | temperature (`m2`) |
| ADC channels | 6 (ESP32), 4 (ESP32-S3) |
| I2C | on |
| Output rate | 50 Hz |
| Bin size | 168 |
| Attenuation | 3 (0 to 3.9 V) |
| Bridge supply | 3320 mV |
| Bridge resistors | values measured on the prototype board, see `DEFAULT_R` in the sketch |
| Steinhart-Hart | A = 0.001111285538, B = 0.0002371215953, C = 7.520676806e-8 |
| Data output | USB only |
| USB limit | 200000 bytes/s (ESP32-S3), 40000 bytes/s (ESP32) |
| BLE limit | 30000 bytes/s (ESP32-S3), 25000 bytes/s (ESP32) |
| Log level | 1 (info) |

The defaults are in the sketch (`DEFAULT_*`, `SYS`) and are used when nothing is stored.

## Building

Arduino IDE with the Espressif ESP32 board package. Select your board (SparkFun ESP32 Thing Plus C or Adafruit Feather ESP32-S3).
Libraries: AudioTools (arduino-audio-tools), NimBLE-Arduino, UUtzinger_logger, UUtzinger_BLESerial (BLE serial and `Linereader.h`) and UUtzinger_RingBuffer. They can be installed with `install_libraries.sh` / `install_libraries.ps1` in the Software folder.
