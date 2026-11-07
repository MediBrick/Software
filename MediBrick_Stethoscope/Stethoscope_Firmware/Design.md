## Software Updates

Read help screen for MAX30001G Firmware
Read help screen for Thermistor Firmware
Design new help screen for Stehoscope similar to these two

Need section for persistent setting
Need mode switch for pressure audio, both
Need output selection for USB, Bluetooth or both
Need output switch to output either to Line 1 or Line 2
Need start stop measurement
Need input selection for Line 1 or Line 2 or simulated two tones for left and right channel and simulated tones plus white noise generator.
Need Engineering section to
  select line1 or line 2 as input 
  set high and low filter frequency for audio processing
  step up down high and low pass filter frequency
  adjust hardware input gain
  adjust hardware output gain
  adjust software gain on processing
  enable/disable L-R subtraction, set gain and bias on subtracting channel

Need Engineering Links and Logs
  output decimation for audio
  output decimation for pressure
  log level selection
  BLE name
  baudrate

Add battery monitor, OLED display
Add sleep button
Add status screen for mode, pressure, running, minimize stalling due to screen refresh
Add bluetooth serial in/out, similar to Thermistor firmware but adjust update mechanisms in main loop as necessary 
Add option for lowpass filter on pressure readings

## Implementation of Software changes

Base: `Stethoscope_Firmware.ino` (1174 lines, single file, single-character commands, blocking audio loop).
Pattern to follow: `Temperature_Firmware.ino` (settings struct + NVS, `LineReader` command lines, `RingBuffer` per link, USB + BLE, short/long button, battery + OLED) and the help screen layout of `MAX30001G_Firmware.ino`.
Libraries already used by those sketches: `UUtzinger_BLESerial`, `UUtzinger_RingBuffer`, `UUtzinger_logger`, `Linereader.h`, `Adafruit_MAX1704X`.

### What exists today
- Modes pressure / stethoscope / both (`p s b`), volumes, SW gain, HP/LP IIR, R-L subtraction with scale, plot decimation, `j/J` save/load, OLED + battery, button = sleep, 4.5 h auto shutoff.
- Audio loop blocks in `i2s_read(..., portMAX_DELAY)`, so OLED, pressure, serial commands and (later) BLE all wait on audio.
- `AUDIO_ADC_IN` / `AUDIO_DAC_OUT` are compile-time. Codec is configured once in `initES8388()`.
- Single-character commands conflict with the Thermistor convention (`.` `<` `>` mean start/stop there).

### Decisions I will assume unless you say otherwise
1. **Command syntax** moves to Thermistor style: one line per command, letter + optional argument (`Fh40`; upper case = on, lower case = off for filters), `LineReader` for USB and BLE. Old single characters are mapped in Step 1 (table below).
2. **Start/stop** (`. > <`) = mute the codec and stop streaming data (I2S/codec stay configured, data buffers cleared). A full audio stop is a mode change: `m0` (new) switches audio and pressure off, `m1` is pressure only. In mode 0 the battery monitor, display, button and commands stay active. A persistent `autostart` flag decides whether it runs after boot (default on, as today).
3. **Simulated inputs** are generated in software and replace the I2S samples *before* the processing chain, so filters, gains, subtraction, outputs and data streams all see them. Own phase-accumulator sine and xorshift white noise (cheap, no AudioTools dependency). Left and right tones have separate frequency and amplitude.
4. **Audio runs in its own FreeRTOS task** (core 1): `i2s_read` (short timeout) -> source select -> process -> `i2s_write` -> push decimated lines into ring buffers. The main loop does commands, BLE/USB drain, pressure, battery, OLED. Only the main task touches `Wire` (ES8388 register writes, OLED, MPRLS, MAX17048), so no I2C mutex is needed. Settings are shared through a settings struct plus a "filters dirty" flag.
5. **Decimation** of audio output = the user's low-pass filter followed by plain drop-N (output every Nth sample of the processed audio). The data stream is exactly what is heard, and the low-pass is the anti-alias filter, so for clean decimation set it to about 0.4x the output rate or lower (the low-pass also affects the headphone audio). A boxcar average was considered and rejected: it is only a weak low-pass (about -4 dB at the output Nyquist) and makes the data differ from the audio. `s` warns when the low-pass is off or above 0.5x the output rate while decimating. A separate data-path low-pass, set automatically from `La`, is an option for later if logging at a much lower rate than listening is needed. Pressure output decimation = output every Nth reading.
6. **Output data format** stays text CSV (Serial Plotter compatible): `left,right` or `left,right,pressure`; pressure-only: `pressure`.
7. **Link rates are not throttled.** 16 kHz x 2 channels as text is about 190 kB/s; the Thermistor limits (200 kB/s USB, 30 kB/s BLE) are conservative, about 1/4 of what the links can do. Data lines are written as fast as the links accept them, with no rate limit and no automatic decimation. If a link's ring buffer is full, whole lines are dropped and counted per link; the counters are shown by `s` and `i`. The user picks `La`/`Lp` to fit the link.

### Command map (draft help screen)

```
================================================================================
| MediBrick Stethoscope and Blood Pressure Board                               |
================================================================================
| GENERAL COMMANDS                       | DATA COMMANDS                       |
|----------------------------------------|-------------------------------------|
| ?: help screen                         | z: toggle data reporting on/off     |
| s: show current settings               | z[s|b|a]: USB, BLE, both, off       |
| i: show system info and limits         | z[S|B|A]: USB, BLE, both, on        |
| x: reboot                              |                                     |
| Button: short = screen, long = sleep   |                                     |
|========================================|=====================================|
| PERSISTENCE                            | START/STOP                          |
|----------------------------------------|-------------------------------------|
| Ps: save NVS settings                  | .: toggle start/stop                |
| Pl: load NVS settings                  | >: start measurement                |
| Pd: print NVS settings                 | <: stop measurement                 |
| Pc: clear NVS settings                 | A<0|1>: autostart at boot           |
| Pr: reset to factory defaults          |                                     |
|========================================|=====================================|
| MODE AND ROUTING                       | INPUT SOURCE                        |
|----------------------------------------|-------------------------------------|
| m0: audio and pressure off             | a1: Line 1 (electret + amplifier)   |
| m1: pressure only                      | a2: Line 2 (jack microphone)        |
| m2: audio only                         | a3: simulated tones L and R         |
| m3: both                               | a4: simulated tones + white noise   |
| o1: audio out to Line 1 (line out)     |                                     |
| o2: audio out to Line 2 (headphone)    |                                     |
|========================================|=====================================|
| ENGINEERING: AUDIO FILTER AND GAIN     | ENGINEERING: L-R SUBTRACTION        |
|----------------------------------------|-------------------------------------|
| FH[Hz]: high-pass ON (+ set Hz)        | N<0|1>: subtraction off/on          |
| Fh: OFF, Fh<Hz>: set Hz only           | Ng<x>: gain on subtracted channel   |
| Fh+ / Fh-: step high-pass up/down      | Nb<x>: bias (counts) to result      |
| FL[Hz]: low-pass ON (+ set Hz)         |   result = R - Ng*L + Nb            |
| Fl: OFF, Fl<Hz>: set Hz only           |   output L = R = result (N on)      |
| Fl+ / Fl-: step low-pass up/down       |                                     |
| Gi<0-8> / Gi+ / Gi-: HW input gain     | SIMULATION (a3, a4)                 |
| Go<0-100> / Go+ / Go-: HW output gain  | Tl<Hz>,<amp>: left tone             |
| Gs<x> / Gs+ / Gs-: software gain       | Tr<Hz>,<amp>: right tone            |
| R<8|16|32|48>: sample rate in kHz      | Tn<amp>: white noise amplitude      |
|========================================|=====================================|
| ENGINEERING: PRESSURE                  | ENGINEERING: LINKS AND LOG          |
|----------------------------------------|-------------------------------------|
| Qe<0|1>: pressure low-pass off/on      | La<n>: audio output decimation      |
| Qf<Hz>: pressure low-pass cutoff       | Lp<n>: pressure output decimation   |
| Qz: re-zero pressure (cuff open)       | l<0-5>: log level 0 debug .. 5 none |
|                                        | Ln<name>: BLE name (needs Ps, x)    |
|                                        | Lb<0|1>: BLE stop/start now (Ps: boot)|
|                                        | Ls<baud>: serial baud (needs Ps, x) |
================================================================================
```

Old command -> new command: `p/s/b` -> `m1/m2/m3`; `< >` volume -> `Go-/Go+`; `, .` input volume -> `Gi-/Gi+`; `+ -` -> `Gs+/Gs-`; `L/l H/h` -> `Fl+/Fl-`, `Fh+/Fh-`; `G/g K/k` -> `FH1/FH0`, `FL1/FL0`; `N/n` -> `N1/N0`; `C/c` -> `Ng`; `d` -> `La`; `t` plot -> `z`; `j/J` -> `Ps/Pl`; `v` -> `s`. Open question: `s` was "stethoscope only"; it becomes "show settings" as in the other two firmwares.

### Step 1 - Settings, persistence, command parser, help (no behaviour change) - DONE, compiles, not yet tested on hardware
- Add `Settings` struct with all items above; `setDefaults()`, `readSettings()`, `saveSettings()`, range validation (NVS keys <= 15 characters, namespace stays `mb-stetho`).
- Replace `handleSerialCommands()` with `LineReader<128>` + `processCommand(char*, CommandSource)`; port every existing function to the new commands; add `P*`, `s`, `i`, `x`, `l`, `Ln`, `Ls`.
- Logging through `logger.h` (`LOGln`, `LOGI`...) with `l<0-5>`; baud/BLE name applied at boot.
- Write `helpMenu()`, `printSettings()`, `printSystemInfo()` using the layout above. Update the file header comment.
- Test: every old function reachable under the new command, settings survive reboot, `Pr` restores defaults.

### Step 2 - Start/stop and non-blocking main loop - DONE, compiles, not yet tested on hardware
- Add `measurement_running`, `startMeasurement()`, `stopMeasurement()`, commands `. > <`, `A`, `m1..m3` (mode change = stop, switch, start).
- Move audio into a task; main loop becomes: commands, button, battery/OLED, pressure, drain. Use `i2s_read` with a short timeout in the task; remove `portMAX_DELAY` from the main loop.
- Mute the codec on stop and before sleep.
- Test: OLED redraws and serial commands stay responsive while audio runs; no `i2s_write incomplete`; audio has no dropouts during OLED refresh.

### Step 3 - Links: USB + BLE, ring buffers, decimation - DONE, compiles, not yet tested on hardware
- Add `BLESerial` (task pump mode) with `settings.ble_name`, command input from BLE, one `RingBuffer` per link, `queueLine()`, `drainUsb()`, `drainBle()` as in Thermistor. Lines dropped are counted and shown by `i`.
- `z` link selection, `La`/`Lp` decimation. No rate limits (assumption 7); per-link dropped-line counters shown by `s` and `i`. Note: the code from this step still uses a block average for audio decimation; it is replaced by drop-N in Step 6 (assumption 5).
- Check how Thermistor routes `LOGln` replies to BLE and do the same here.
- Test: plot on USB, plot on BLE, both; check the dropped-line counters at several decimation values to find what each link sustains.

### Step 4 - Input source and output selection - DONE, compiles, not yet tested on hardware
- Input `a1..a4` (the number is the Line number for a1/a2): `a1/a2` select the ES8388 ADC input mux (ADCCONTROL2, reg 0x0A) at runtime through a small `es8388Write()` over `Wire` (verify values against `es8388.c` around line 348 and the datasheet), `a3/a4` switch the audio task to the generators. Mux and generator state changes happen with output muted to avoid clicks.
- Output `o1/o2`: write only the selected output pair into DACPOWER (reg 0x04): **0x30 = LOUT1+ROUT1 (Line 1, line out jack J1), 0x0C = LOUT2+ROUT2 (Line 2, headphone jack J3)**. These bits (5/4/3/2 = LOUT1/ROUT1/LOUT2/ROUT2) were found on the board: the first version used the values of the arduino-audio-driver enum (LOUT1 = 0x04 ...), which select the other pair, so `o2` played on J1. The PCB netlist confirms J1 <- C24/C23 (1 uF) <- LOUT1/ROUT1 and J3 <- C25/C26 (47 uF) <- LOUT2/ROUT2. The other pair is powered down so it does not load the jack. The existing `Go` volume needs no change: the driver's volume "hack 1" writes the OUT1 and OUT2 volume registers together, only the powered pair is audible.
- The codec is only written when the selection changed, with the DAC muted and a 20 ms settle before unmuting if the audio is running.
- Simulated sources: tones are rotating phasors (4 multiplications and 2 additions per sample and tone, radius reset once per block), noise is a xorshift32 generator; the same noise sample is added to both channels so L-R subtraction can be tested. Amplitudes are peak counts. The samples replace the codec samples before the processing chain.
- Generators: left/right tone frequency and amplitude, white noise amplitude (`Tl<Hz>[,<amp>]`, `Tr<Hz>[,<amp>]`, `Tn<amp>`), defaults 100 Hz / 250 Hz at 3277 counts (-20 dBFS) and noise 328 counts (-40 dBFS).
- Replace the compile-time `AUDIO_ADC_IN` / `AUDIO_DAC_OUT` and the OLED `#if` strings.
- Test: tones visible in the plot on `a3`, noise added on `a4`; switching a1 <-> a2 with a mic on each; output audible on the selected jack only.

### Hardware bring-up test (after Step 4, before Step 5)
Run on the board in this order. Run it first on `Stethoscope_Firmware_legacy_i2s` (Steps 1 to 4 on the legacy I2S driver), then repeat on `Stethoscope_Firmware` (Step 5, i2s_std) and compare. Status is shown per test: PASS, OPEN, PENDING (needs a code change first).

**Preparation**
- Arduino IDE settings (Adafruit Feather ESP32-S3): USB CDC On Boot Enabled, USB Mode Hardware CDC and JTAG, Arduino and Events run on Core 1, partition Default (3 MB APP), Erase All Flash disabled, Core Debug Level None (set to Error only to see I2S driver errors), CPU 240 MHz. The board entry must match the variant (8 MB flash = "No PSRAM" version).
- Wiring per Assembly.md: the ESP32 transmits on GPIO35 (pad DIN) and receives on GPIO37 (pad DOUT); J5 and J6 are the two electret microphones (Line 1), the headphone jack J3 is Line 2, the line out jack J1 is Line 1.
- Serial terminal at 500000 baud, one command per line. Start with `Pr` (defaults: input gain 0 dB, filters off, software gain 1.0, subtraction off, tones 100 Hz left and 250 Hz right) and check with `s`. Keep the output of `i` of every run.

**Tests**

| # | Test | Expected | Status |
|---|---|---|---|
| T1 | Boot: log shows "ES8388 configured", "Audio subsystem ready"; `i` lists the sensors; OLED shows the mode and battery. | no init errors; codec answers at I2C address 0x10 | PASS (legacy) |
| T2 | Sample rate: run `i` twice about 10 s apart. | blocks grow by 62.5 per second, `max gap` about 16000 us, `processing max` well below 16000 us, read/write errors 0/0, free heap constant | PASS (legacy, gap 16007 us, processing max 200 us) |
| T3 | Transmit path: `m2`, `a3`, `o2`, headphones on J3; then `o1` on J1 with an amplifier; `a4` adds noise. A low impedance headphone on J1 loses the bass (1 uF into 32 Ohm is a 5 kHz high-pass). | 100 Hz in the left ear, 250 Hz in the right ear; sound only on the selected jack; noise centred (same noise in both channels) | PASS (legacy) |
| T4 | Receive path and input level: `Pr`, `Gi0`, `a1`, `z`; make a loud sound next to a microphone; `i` shows the input peak per channel in % of full scale and the frames at full scale since the last `i`. Raise with `Gi+` (3 dB per step). | peaks of loud sounds 30 to 50 % of full scale, no frames at full scale; heart sounds are much quieter and need more gain | PARTIAL: a tap on the capsule at 0 dB reaches about 32000 (full scale) and `Gi+` increases the level, so the path reaches full scale and the gain works (a tap is the worst case, not a calibration). OPEN: the 3 dB steps with a steady tone (phone near the microphone, peaks in `i` at each `Gi`) and the gain for real signals (speech at 30 cm, chest piece), see T15. The old default 75 (+18 dB) saturated, the default is now 0 dB |
| T5 | Channel order and crosstalk: tap only one microphone. | only one column reacts. Prediction from the KiCad netlist (the amplifier numbering is crossed): **J5 (amplifier 1) -> C22 -> codec RIN1 = `right` column; J6 (amplifier 2) -> C21 -> codec LIN1 = `left` column**. Connection chosen: the stethoscope microphone (channel 2) on **J5 = right**, the background microphone (channel 1) on **J6 = left**, as the subtraction expects | PASS for the order: the first number of the data line (`left`) reacts to J6, as predicted by the netlist (J6 -> LIN1), so J5 is the second number (`right`). Crosstalk accepted: both microphones sit in one housing that conducts sound, so each hears some of the other's signal (acoustic and mechanical, not electrical). Consequence for T14: the background microphone also hears part of the stethoscope signal, so the subtraction removes some of it; `Ng` below 1 trades background reduction against signal loss |
| T6 | Line 2 microphone: `a2` with a microphone on the headphone jack J3. | signal in both columns (left and right inputs are wired together) | OPEN |
| T7 | Gains and filters: `Gi+/Gi-`, `Gs`, `FH20`, `Fl2000`, `Go`. | signal size changes in 3 dB steps; high-pass removes DC offset in the plot; headphone volume follows `Go` | PASS (legacy, simulated tones): `FH200` both levels match, `FH80`+`FL300` 2270 / 1770 (expected 2277 / 1767), `FL150` left 2225 matches; the right channel value of 1650 was a reading error (the plot program autoscales and the value had to be divided by the axis scale factor of 2); there is no channel anomaly, and on the i2s_std build `FL150` gives the calculated values on both channels. `Gs2` doubles, `Gs+/Gs-` work, `FH5000` and the high-pass above low-pass are rejected, `Go-/Go+` change the volume, `Ps`, `x`, `s`, `Pr` work. Not yet tested: `Gi` with a microphone steady sound |
| T8 | Start/stop and modes: `.` several times, `m0` to `m3`, `a`/`o` switching. | no loud click, no hang, `i` error counters stay 0 | PARTIAL: start/stop and mode switching work without hang; small clicks on start (`.` on, `>`), when switching `a1`/`a2`/`a3` and on `o1`/`o2`; none on stop and on `a3` <-> `a4` (declick is planned in Step 6). OPEN: autostart `A0`/`A1` after a reboot, `m1`/`m3` need the pressure sensor |
| T9 | Links: `zA` with USB and a BLE client, dropped-line counters in `i` at `La1`, `La4`, `La8`; commands over BLE; `Lb0` stops BLE now, `Lb1` starts it, `Ln<name>` then `Lb0`, `Lb1` applies a new name. | no dropped lines at the chosen `La`; commands answer on the link they came from; BLE restarts without reboot | PARTIAL: BLE connects and works. With `zA` and `La1` (190 kB/s) the audio stayed clean (gap 16023 us, errors 0/0, processing max 5421 us because 256 text lines are formatted per block); USB dropped 1901 lines in total (the plot program slows down at `La1`, `La4` and `La8`, a limit of the viewer), BLE (Balanced) dropped 846364 lines at `La1`. `i` now shows the kB/s sent per link and the dropped lines since the last `i`. BLE rates measured at the laptop terminal (a3 tones, 11 bytes per line, so the generated rate is 5.5 kB/s at `La32`, 11 at `La16`, 22 at `La8`, 44 at `La4`, 88 at `La2`, 176 at `La1`): Balanced `La32` 5.5, `La16` about 10.5, and 10.5 kB/s at every smaller `La`, so the Balanced ceiling is about 10.5 kB/s (use `La32` or higher, `La16` is at the limit). Fast (`Lm0`): `La32` 11, `La16` 22, `La8` 45, `La4` 90 kB/s, `La2` lost the connection. The Fast numbers are twice the generated rate: OPEN, check the received text for lines that arrive twice (possible duplicate sending in `drainBle` if `writeNonBlocking` returns less than it queued) or a counting effect of the terminal. To see the firmware side rate while streaming over BLE keep USB quiet with `zs` (USB data off, BLE data on) and send `i` twice, 10 s apart (the rates are averaged since the previous `i`). Firmware side `i` (Fast, `La16`): 11.5 kB/s sent, 0 dropped, buffer empty; `La8`: 23.3 kB/s; the laptop terminal shows twice that in Fast mode (duplicate lines? check with the `awk` repeat count of a capture). After overloading at `La4` and `La2` the BLE sender got stuck: connected and subscribed, 16379 of 16384 bytes buffered, 0.0 kB/s sent, all lines dropped (`writeReady()` of the library stays false). Added: BLE stall watchdog (data on, subscribed, buffer at least half full and nothing sent for 5 s: log a warning and restart BLE, the client connects again; restarts are counted in `i`) and `Lbs` (prints the status dump of the BLE library to USB, with the TX buffer `locked` and `pending` flags). OPEN: heap after 10 `Lb0`/`Lb1` cycles, restart of BLE and a new name (145804 bytes after several restarts, 146368 before: possible small leak) |
| T10 | Pressure: `m1` and `m3`. | about 0 mmHg open to air, rises with the cuff | DEFERRED: the pressure sensor is not attached yet (also the `m1`/`m3` checks of T8, T16 pressure column and T19 in mode 3) |
| T11 | Heat: touch or measure the regulator (AP2112) and the charger after 10 minutes with audio and BLE running on USB. | stable, no continuous rise (measured 50 C case on USB, fine) | PASS |
| T12 | I2C bus: after adding RC low-pass filters to SDA and SCL, the log showed `Error: ->p_wire->endTransmission: 4` (twice, during `Tl`/`Tr` and a mode change). Run many commands (`Tl`, `Tr`, `Gi+`, `Go+`, `a`/`o`, `m0` to `m3`) and look at the I2C line of `i`. | clock 100000 Hz; failed ES8388 write tries since boot stays 0 (or very few and always repeated successfully); no error lines in the log | PASS (2026-11-07, legacy build with the I2C retries, 100 kHz clock): the command series for volume, mute, input and output selection and mode changes produced no errors in the log |
| T13 | Persistence: `Ps`, change settings, `Pl` (load), `Pd` (print saved), `Pc` (clear) then reboot. | after `Pc` and a reboot the defaults are active: BLE off, BLE mode Balanced, input gain 0 dB, filters off | PASS (2026-11-07): changed settings (`Gi3`, `Go60`, `FH30`, `FL1500`, `La8`, `Tl120,2000`, `Lm0`, `Lt-6`, `Ln`) were shown by `s`, saved with `Ps`, printed by `Pd`, loaded by `Pl`, survived a reboot; `Pr` changed only the running settings (the saved copy came back after `x`); `Pc` cleared them and the next boot used the defaults; `Lb1`, `Ps`, `x` started BLE at boot (Balanced, -6/0 dBm). The aliasing warning appeared correctly (low-pass 1500 Hz above half of the 2000 Hz output rate) |
| T14 | Subtraction with real microphones: stethoscope microphone on J5 (right), background microphone on J6 (left); `N1`, vary `Ng`; make a sound near only J6, then near only J5. | common background noise is reduced by `Ng` near the matching gain; sound near J6 only is subtracted, near J5 remains | PARTIAL (2026-11-07): with real microphones `N1` removes the background noise and also the BLE bursts, which are common-mode on both channels. With `Ng1` the noise is not perceivable (the two microphone paths are well matched); `FL2000` removes more noise (the hiss left after the subtraction is broadband and uncorrelated between the microphones), with `FL1000` no noise is heard. Candidate working settings for the defaults review in Step 9: `FH20`, `FL1000`, `Ng1`, `N1`. OPEN: BLE connected and streaming with `N1` (can be tested with a background sound). DEFERRED until the chest piece is available: loss of the stethoscope signal (`N0` against `N1`) (housing crosstalk, plus the left microphone's own noise is added to the result) |
| T15 | Real recording: chest piece on the chest, `Gi` chosen from T4, compare no filter with `FH20` and `FL1000`. | heart sounds visible, no frames at full scale, baseline drift removed by the high-pass | DEFERRED: no chest piece yet; after the new microphone cables (coax or twisted pair, capsule capacitors) and the pressure sensor are installed |
| T16 | Data format: the plot program accepts `left,right`; in mode 3 the pressure appears as a third column (about 10 values per second); mode 1 prints only pressure. | correct number of columns, no mixed lines | OPEN |
| T17 | USB behaviour: close and reopen the terminal while `z` is on, unplug and replug the cable, `Ls` baud change with `Ps` and `x`. | no stall of audio, data resumes, commands still work | PASS (2026-11-07): closing and reopening the terminal (disconnect and reconnect) works. Unplugging the USB cable resets the board because it loses its power (to be checked in T18: a connected battery should keep it running). The baud rate (`Ls`) has no effect on the Feather ESP32-S3 (native USB CDC has no baud rate), it only matters on UART bridge boards such as the Thing Plus C; help and `s` now say so |
| T18 | Battery and sleep: run on battery only; compare the battery percentage with the voltage; button puts the board to sleep and wakes it. | plausible percentage; after wake the settings and audio are restored | OPEN |
| T19 | Soak: mode 3, USB streaming, 30 to 60 minutes, many mode changes. | free heap stable, `max gap` near 16 ms, errors 0, temperature levels off | OPEN |
| T20 | Pressure scale: compare `m1` readings with a reference gauge at two or three pressures (for example 50, 100, 150 mmHg). | readings within the tolerance of the sensor; the divisor 0.9762 in the code verified or corrected | DEFERRED: the pressure sensor is not attached yet (also the `m1`/`m3` checks of T8, T16 pressure column and T19 in mode 3) |

**T7 details (simulated tones `a3`, `z` with a plot, `La4`; start with `Pr`).** Default tones: left 100 Hz and right 250 Hz, both peak 3277. At `La4` the data rate is 4000 values per second, an exact multiple of 100 Hz and 250 Hz, so the sampled phases repeat and the plot peak can sit up to 0.3 % (100 Hz) and 1.9 % (250 Hz) below the true amplitude. For amplitude measurements use `Tl103` and `Tr257` instead. Expected peaks (calculated with the discrete filters of the code and the four-sample average of the data line; the true amplitude is 3277):

| Setting | 103 Hz (left) | 257 Hz (right) | 100 Hz | 250 Hz |
|---|---|---|---|---|
| no filter, `Gs1` | 3274 | 3256 | 3274 | 3257 |
| `FH200` | 1441 | 2470 | 1407 | 2445 |
| `FL150` (two stages) | 2225 | 828 | 2267 | 863 |
| `FH80` + `FL300` | 2277 | 1767 | 2265 | 1804 |

- Baseline measured (legacy build): channel 1 (left, 100 Hz) about 3277, channel 2 (right, 250 Hz) 3220; consistent with the sampling effect above. PASS.
- `Gs2`: both peaks double (about 6550); `Gs+/Gs-` change by 3 dB (x1.414); clipping at 32767 above about `Gs9`.
- `Fh` switches the high-pass off, `Fh100` sets the cutoff without changing on/off; `Fl` switches the low-pass off. Valid ranges 1 to 2000 Hz (high-pass) and 100 to 7200 Hz (low-pass).
- `FH5000` is rejected with the range; with both filters on, `FH300` then `FL200` is rejected (high-pass must stay below low-pass).
- `Go-/Go+`: audible steps of 3 dB; `Go0` is -45 dB. Keep the headphone volume low when raising it.
- `Gi` has no effect on simulated tones; check it with a microphone (`a1`, `Gi0`, steady sound): each `Gi+` raises the input peak in `i` by about 3 dB.
- `Ps`, `x`, `s` shows the saved values; `Pr` resets them.

**Noise investigation: bursts every ~155 ms (microphone test, `a1`, 0 dB)**

| # | Test | Result |
|---|---|---|
| N1 | Compare `Lb1` and `Lb0` (BLE stopped), same plot. | PASS: with BLE on there are bursts of about +-200 counts, identical on both channels, every ~620 data lines (155 ms at `La4`); with `Lb0` they are gone. Also audible: microphone playback (`a1`) in earbuds crackled with an irregular repeating pattern, gone after `Lb0`, `Ps`, `x` (BLE advertising was running; the input never clipped and the audio timing was clean). Decision: BLE is **off by default** (`Pr` and a fresh board start with BLE off; `Lb1` starts it, `Ps` keeps it on after a reboot). The BLE library advertises every 100 to 150 ms in `Mode::Fast` (+0 to 10 ms random delay), which matches. |
| N2 | Change `La` (1, 4, 8) with BLE on. | OPEN: the spacing must stay constant in milliseconds (confirms BLE, not a sample count effect). |
| N3 | BLE TX power (`ble.setPower(dBm, PWR_ADV / PWR_CONN)`), for example +9 and -12 dBm. | OPEN: `Lt<dBm>` is implemented (try `Lt9` and `Lt-12`). If the burst size follows the power it is RF picked up by the microphones or preamplifier; if not it is a supply or ground transient. |
| N4 | Move the Feather antenna away from the microphone wires, or cover the wires with grounded foil; run on battery instead of USB. | OPEN: shows RF pickup through the wires versus supply coupling. |
| N5 | Small C0G capacitor across each electret capsule (10 pF to 100 pF, at most 1 nF so the 16 kHz band is not affected), shortest leads at the capsule; optional ferrite bead (about 600 Ohm at 100 MHz, 0603 or 0805) in series with mic+. | OPEN: helps if it is RF rectified in the electret transistor or the preamplifier input. The 500 pF in the op-amp feedback only sets the 16 kHz low-pass and does not help against RF at the input. |
| N6 | Supply: 100 to 470 uF plus 0.1 uF across 3.3 V / GND at the PCB; feed the analog 3.3 V pad (J2 pad 10) through a few Ohm plus bulk capacitor or a separate LDO (check the codec minimum analog supply). | OPEN: helps if it is a supply or ground transient. |
| N7 | BLE client connected, idle and then streaming (`zB`). | OPEN: connection events (7.5 to 10 ms in `Mode::Fast`, about 100 Hz) would put bursts in the heart sound band. If not fixed in hardware, stream audio over USB only. Note (T14): the BLE bursts cancel in the subtraction R - L, so repeat this test with `N1` on: BLE connected and streaming may be usable for recording if the microphones are matched. |

**BLE modes of the library (`ble.begin(Mode, ...)`, currently `Mode::Fast`)**

| Mode | Advertising interval | Connection interval, latency | PHY | TX power advertising / connection |
|---|---|---|---|---|
| Fast | 100 to 150 ms | 7.5 to 10 ms, 0 | 2M | +9 / +9 dBm (maximum) |
| Balanced | 0.5 to 1 s | 15 to 30 ms, 2 | 1M | -6 / 0 dBm |
| LowPower | 1 to 2 s | 60 to 120 ms, 8 | 1M | -9 / -6 dBm |
| LongRange | 0.5 to 1 s | 30 to 60 ms, 2 | Coded | +9 / +9 dBm |

Fast is the noisiest choice: the highest power and the most events. A connected client makes the radio fire at every connection interval even when no data is sent (about 100 per second in Fast, 30 to 60 per second in Balanced, 8 to 16 per second in LowPower), and streaming adds longer transmissions. Implemented: BLE mode `Lm<0-3>` (0 Fast, 1 LowPower, 2 LongRange, 3 Balanced; default **Balanced**; BLE restarts when the mode changes while it runs) and BLE transmit power `Lt<dBm>` (-24 to +9 dBm for advertising, scanning and connection, applied immediately; `Lta` = default power of the mode), both saved with `Ps`; `i` shows the mode and the actual advertising and connection power. The library's automatic link adjustment changes the PHY (2M or 1M by signal strength), pacing and connection interval, not the transmit power, which is set in `begin()` and by `setPower()`. Use: use BLE for commands and low rate data, for pressure-only streaming (`m1`, no microphone in use) or with the hardware measures below; stream audio over USB. Default mode to be chosen in Step 9 (Balanced is the suggested compromise).

**Findings and corrections from the first tests (2026-11-06)**
- I2S data pin roles: the ESP32 transmits on GPIO35 and receives on GPIO37 (KiCad netlist and Assembly.md); the firmware had them reversed.
- `RATE_16K` of the codec driver is an enum index (2), not 16000 Hz: the legacy I2S setup ran at 2441 Hz (gap 104865 us). The firmware now uses `AUDIO_SAMPLE_RATE_HZ` for the I2S clock.
- ES8388 DACPOWER output enable bits: 0x30 = LOUT1+ROUT1 (Line 1, J1), 0x0C = LOUT2+ROUT2 (Line 2, headphone J3); the driver library names are the other way round.
- The default input gain was +18 dB, which saturated the microphones; the default is now 0 dB and `i` shows the input level.
- I2C: sporadic `endTransmission: 4` errors on codec writes after adding RC filters to SDA and SCL. Changes: the I2C clock is explicit and 100 kHz for all devices (including the OLED, which used 400 kHz during transfers); the codec volume registers are written only when the value changes (before, every setting command wrote 7 registers); codec writes are tried up to 3 times (`codecWrite`, also for mute, and for the input select and output enable registers); a failed register write is repeated at the next change; failed tries are counted and shown by `i`.
- BLE advertising causes the periodic bursts; BLE can be stopped and started at runtime with `Lb0` and `Lb1`.

### Step 5 - Move from the legacy I2S driver to i2s_std (before the audio chain) - DONE, compiles, not yet tested on hardware
- **Implemented:** `initI2SDriver()` creates a TX and an RX channel on port 0 (master, full duplex, Philips 16 bit stereo, MCLK = 256*fs, 8 DMA buffers of 128 frames, zeros sent when nothing is queued) with `i2s_new_channel` and `i2s_channel_init_std_mode` on both; the channels are created stopped. The audio task enables/disables them (`i2s_channel_enable` TX then RX, disable RX then TX); BCLK and WS stop when disabled, MCLK keeps running (checked in the IDF header). Reads and writes use `i2s_channel_read/write` with 100 ms timeouts. `shutdownAudio()` uses `i2s_del_channel`. No legacy `driver/i2s.h` call is left and the deprecation warnings are gone.
- **First hardware result (2026-11-07, i2s_std build):** boot without I2S errors, 62.5 blocks per second, longest gap 16019 us (legacy 16007 to 16028 us), read/write errors 0/0, processing max 618 us, heap stable (145488 bytes), I2C clock 100000 Hz with 0 failed tries. The start-up transient on the left input (peak 16000 in the first interval after boot) is also present, so it is not caused by the driver. Regression (2026-11-07, i2s_std build): tones in the correct ears, filter peaks as calculated (`FH200`, `FL150` on both channels), start/stop and mode changes with a slight click (as before), the I2C command series without errors, persistence and BLE work. PASS. OPEN: microphone order on this build and autostart `A0`/`A1`.
- **Differences to the legacy version handled in code:** there is no `i2s_zero_dma_buffer`, so audio left in the transmit buffers when stopping may be sent again after a restart; the unmute after a start is delayed by the DMA depth (64 ms at 16 kHz) while the DAC is muted. A read that returns an incomplete frame (timeout) keeps the leftover bytes and completes them with the next read, so left and right stay paired. `use_apll` has no equivalent on the S3 (no APLL). The legacy version is kept for comparison in `Stethoscope_Firmware_legacy_i2s/` (delete it when no longer needed).
- `driver/i2s.h` is deprecated (compile warning "This set of I2S APIs has been deprecated" plus deprecated `dma_buf_count`/`dma_buf_len`). Arduino ESP32 core 3.3.12 uses ESP-IDF 5.5.5, where the legacy driver still works. Espressif states the deprecated APIs "will no longer be supported in the future"; no removal date is given in the installed headers. Decided order: test Steps 1 to 4 on the board with the legacy driver first (known-good baseline), then do this step, then the audio chain (Step 6), so the sample-rate command `R` and the rewritten audio task are written once against the final API.
- Keep `AudioBoard` for the codec (not `I2SCodecStream`, which is only `AudioBoard` + `I2SStream` glue and does not fit the audio task: its reads wait forever by default and stop means tearing the driver down). Replace only the I2S layer, either with `driver/i2s_std.h` directly (channel enable/disable replaces `i2s_start`/`i2s_stop`, `i2s_channel_reconfig_std_clock` replaces `i2s_set_clk` for the `R` command) or with AudioTools' `I2SStream`.
- Touches `initI2SDriver()`, `shutdownAudio()`, the start/stop handshake and the read/write calls in the audio task. Keep MCLK = 256 * fs and codec slave mode. The data pin constants `MY_I2S_DATA_TX` (35) and `MY_I2S_DATA_RX` (37) stay the same.
- Robustness in the audio task: process only complete frames. After a read that returns a partial block (timeout), discard the odd samples so left and right can not swap until the next restart.
- **Test checklist (compare with the legacy driver version, keep it as a git branch or tag):**
  1. Channel order: feed a tone into one microphone only and note whether it appears in the `left` or `right` data column; also check which ear plays the `a3` left tone. The result must be the same before and after, and the same after several reboots.
  2. Noise floor: RMS of the data stream with the input shorted (`z` on, `La` 1), at every sample rate, before and after.
  3. Clock accuracy: check the real sample rate at every rate (for example a `Tl` tone at 1000 Hz on `a3` measured by counting zero crossings in a recording); the divider on the ESP32-S3 may not hit the nominal rate exactly.
  4. Clicks or bursts on start/stop (`.`), on `m` mode changes, and on `a`/`o` switching; silence after a restart.
  5. `i`: `audio_gap_us_max` (about one block, 16 ms at 16 kHz, well below the DMA depth), `audio_proc_us_max`, read/write error counters stay 0.
  6. No "driver conflict" abort at boot (no other library may use the legacy driver).
- The clock reconfiguration is exercised by the `R` command in Step 6; all four sample rates are tested there.

### Step 6 - Engineering audio chain (6a decimation and clipping, 6c subtraction: DONE and tested; 6b sample rates: DONE, compiles, not yet tested on hardware; 6d declick: DONE, compiles, not yet tested on hardware)
- **Subtraction modes (decided 2026-11-07):** `N0` off, `N1` the result in both ears, `N2` the left (background, filtered) channel unchanged in the left ear and the result in the right ear. Result = right - `Ng` * left + `Nb` (bias in counts). The data line always carries the filtered left channel and the right channel or the result. Saved with `Ps` (keys `ncMode`, `ncScale`, `ncBias`; the old key `ncEn` is ignored).
- **First hardware result (2026-11-07):** subtraction `N0`/`N1`/`N2` works as expected. With `La16` (1000 Hz output) `Tr700` aliases to 300 Hz (expected: above Nyquist; the low-pass at or below about 400 Hz removes it). The oscillating peak amplitude of the 103 Hz and 257 Hz tones in the plot is a sampling effect of the 1000 Hz data rate (9.7 and 3.9 samples per cycle, beat with the nearest multiple of the data rate), not a signal fault; use `La4` or `La1` to read amplitudes. `Nb` works as designed: `Nb4000` lifts the second channel (the result) by 4000 and `Nb0` removes it (`Nb` acts only with `N1`/`N2`).
- **Clipping and rate (2026-11-07):** with `Gs16` the default tones clip; `i` counted 253528 of about 480000 output samples in 15 s (53 %, the calculated share for 52400 count tones with 16 bit limits is about 57 %). At `La1` USB sent 162.5 kB/s (the full 16000 lines per second) with 0 lines dropped; processing max 5041 us (formatting 256 text lines per block dominates; optional: replace `snprintf` by a small integer to text routine).
- **Implemented in 6d (declick):** section 6 of the audio task is a gain ramp of the output (`AUDIO_FADE_MS` = 20 ms) towards a target set by the main loop (`audio_fade_target`, 0 or 1; the level is reported back in `audio_fade_level`). `audioQuiet(true)` fades out, waits until the output is silent and the faded audio has left the DMA buffers; `audioQuiet(false, settleMs)` holds the output at zero for `settleMs` (`AUDIO_SETTLE_MS` = 200 ms after a start or a change of the codec input or output, 0 for software only changes) and then fades in. During the hold and while the target is 0 no data lines are sent, so the start-up transient of the codec input does not appear in the data (it is still counted by the input level of `i`). Used by: start/stop (`.`, `>`, `<`, mode changes), the input source (`a`), the output line (`o`), the subtraction mode (`N`), the sample rate (`R`), and before sleep and reboot. The codec is unmuted while the output is silent. The path (hold, ramp, nothing) is chosen once per block, a multiplication per sample is only done while fading. Open: gain changes (`Gs`, `Gi`, `Go`) and filter on/off steps are still instant; if `o1`/`o2` still clicks (output driver power-up) keep both output pairs powered and select the line with the output volume registers.
- **Implemented in 6b:** `R8`, `R16`, `R32`, `R48` set the sample rate (`settings.fs_khz`, key `fsKhz`, default 16). The change stops the audio (muted), sets the clock of both I2S channels with `i2s_channel_reconfig_std_clock` (MCLK = 256 * fs), restarts if it was running, recomputes the filter coefficients and the tone generators, and moves the cutoffs and tone frequencies into the range of the new rate (high-pass 1 Hz to fs/8, low-pass fs/160 to 0.45 fs, tones to 0.45 fs). The ES8388 setup does not depend on the rate (single speed, ratio 256). Block time: 32 ms at 8 kHz, 16 ms at 16 kHz, 8 ms at 32 kHz, 5.33 ms at 48 kHz (256 frames); DMA depth 128, 64, 32 and 21 ms.
- **Implemented in 6a/6c:** the audio task is rewritten with documented sections; the data line takes every `La`-th filtered frame (drop-N, no averaging); the results are rounded to integers once and used for the data line and the DAC; bypass (everything off) skips sections 2, 3 and 5; output samples that had to be clipped are counted and shown by `i`; `s` warns when `La` > 1 and the low-pass is off or above half the output rate.
- **Declick (from the T8 test, 2026-11-07):** small clicks were heard on start (`.` on, `>`) but not on stop, when switching `a1`/`a2`/`a3`, but not `a3` <-> `a4`, and on `o1`/`o2`. Plan: (1) a software fade of about 20 ms in the audio task (gain ramp towards a target set by the main loop, one multiply per sample only while the gain is not 1, path chosen once per block); the main loop fades out, waits for silence, changes the codec (input select, output enable, mute) and fades in; used for start/stop, `a`, `o` and mode changes. (2) Hardware unmute with the ES8388 soft ramp (DACCONTROL3 bit pattern; the library's unmute writes 0x00 which appears to switch the soft ramp off while its mute writes 0x3C; check against the datasheet) and keep the I2S clocks running long enough for a ramp to finish before stopping. (3) If `o1`/`o2` still clicks (output driver power-up bias step), keep both output pairs powered and select the line with the output volume registers (the unused pair at the minimum, -45 dB). Input changes from a codec line to a simulated tone are discontinuities of the signal and need the fade. Start-up transient (T13, 2026-11-07): the first `i` after a boot (3 blocks into the run) showed a left input peak of 24028 (73 % of full scale) and 221 on the right, 380 and 350 eight seconds later. The codec input (or the coupling capacitor) settles for some time after the clocks start. Check whether it recurs on every `.` stop/start. The fade-in after a start or a line change should therefore wait for the input to settle (about 100 to 200 ms, to be measured) and the data output should skip those first blocks.
- **Code requirements (from the user):** the processing chain is documented well (a comment block that states the signal flow, formulas, units, and the order of gain, filter, subtraction and clipping). No function that is used only once: single-use code stays inline in the audio task, and functions are created only when called from more than one place. Code is optimised for the microcontroller: per-block work (coefficients, gain factors, branch selection) is computed once per block and not per sample, no divisions, `powf`/`expf` or `String` in the per-sample loop, float math with `f` constants, tight loops with minimal branching (select the processing path once per block), no heap allocation, and the work per block is measured with `audio_proc_us_max`.
- `Fh/Fl` syntax: upper case turns the filter ON, lower case turns it OFF. `FH2000` = high-pass on and set to 2000 Hz; `FH` = on at the current Hz; `Fh` = off. `Fh2000` = set the cutoff to 2000 Hz without enabling (the on/off state is left as it was); `Fh+`/`Fh-` step the cutoff. Same for `FL`/`Fl`. Out-of-range values are rejected with the valid range printed.
- Valid cutoff range follows the sample rate fs: high-pass 1 Hz to fs/8 (2000 Hz at 16 kHz), low-pass fs/160 to 0.45*fs (100 to 7200 Hz at 16 kHz), and high-pass must stay below low-pass. Coefficients use the exact form `alpha = exp(-2*pi*fc/fs)` so cutoffs near Nyquist are correct, and are recomputed from the audio task when the dirty flag is set. Changing `R` re-validates both cutoffs and clamps them if needed.
- Sample rate `R8, R16, R32, R48` = 8000, 16000, 32000, 48000 Hz (nominal; the S3 clock divider is fractional and may be a few ppm off), default 16 kHz, saved with `Ps`. Applying it stops audio, reconfigures the I2S clock with `i2s_channel_reconfig_std_clock` from Step 5 (MCLK = 256*fs, codec stays in slave mode), recomputes filters, simulation phase increments and decimated output rates, and restarts. Larger fs means shorter DMA buffering (64 ms at 16 kHz, 21 ms at 48 kHz) and more data (48 kHz stereo text is about 570 kB/s before decimation), so use `La`. The rates were chosen to fit the project, not to match other hardware: 8, 16, 32 double step by step, 48 kHz is the top of the codec's single-speed range (MCLK = 256 * fs = 12.288 MHz). The 11.025, 22.05 and 44.1 kHz rates were dropped.
- `Gi` (index 0 to 8 = 0 to 24 dB, 3 dB per step), `Go` (0 to 100, 1.5 dB per register step), `Gs` (software gain, widen to 0.1 to 16).
- `N`, `Ng`, `Nb`: result = R - Ng*L + Nb, i.e. stethoscope (right, J5, channel 2) minus the scaled background (left, J6, channel 1); this matches the current code and the way the microphones are connected, no setting for the channel is needed. With subtraction on, the DAC gets the result on both channels (today the DAC gets L on the left and the corrected R on the right, so the noise reference is heard in one ear). The data stream keeps two columns: processed L (background) and result. With subtraction off, L and R pass through unchanged.
- `i` (system info and limits) lists: hardware input gain options (index 0..8 = 0, 3, ... 24 dB, x1 to x256), hardware output gain (reg 0..33, -45 to +4.5 dB in 1.5 dB steps, setting 0..100), the valid `Fh`/`Fl` range for the current sample rate, the available sample rates, decimation choices and resulting output rates, plus the dropped-line counters.
- Clip protection and a clipping counter shown in `s`.
- Audio data decimation (assumption 5): the data line takes every `La`-th sample of the processed audio (after high-pass, gain, low-pass and subtraction), no accumulators. Remove the block average added in Step 3. The `s` warning also fires when the low-pass is off and `La` > 1.
- Test: sine through `a3` at known frequency, check -3 dB point of each filter and the gain steps.

### Step 7 - Pressure - DONE, compiles, not yet tested (the pressure sensor is not attached)
- **Unit and formula:** the sensor `MPRLS0300YG00001B` is a 0 to 300 mmHg gauge sensor with transfer function B (2.5 % to 22.5 % of 2^24 counts); with the constructor settings of the sketch the library returns mmHg (the conversion factor is 1). `mmHg = (reading - zero) / calibration factor`, shown as 0 if negative. The zero is taken when the pressure mode starts (cuff open to air) and by `Qz`; it is not saved.
- **Commands (implemented):** `Qe<0|1>` low-pass off/on, `Qf<Hz>` cutoff 0.1 to 4 Hz (readings come 10 times per second), `Qc<x>` calibration factor 0.5 to 2 (default 0.9762, the factor that was fixed in the code before), `Qz` zero now (takes half a second, the cuff must be open to air), `Lp<n>` output only every n-th reading. `Qe`, `Qf`, `Qc` and `Lp` are saved with `Ps` (keys `pLpEn`, `pLpFc`, `pCal`, `pDecim`) and shown by `s`; `i` shows the pressure, the zero and the last sensor reading.
- **Low-pass:** first order, `y += alpha * (x - y)` with `alpha = 1 - exp(-2 * pi * fc * dt)`, `dt` the real time since the last reading; it starts again with the first reading after a start, a mode change, `Qz` or switching it off.
- **Tests when the sensor is attached:** zero at open air (about 0 mmHg), `Qz` after pressing, squeeze or inflate the cuff, a step of pressure with `Qe1` and `Qf1` (time constant 1/(2 pi fc) = 0.16 s, 63 % of the step after 0.16 s) against `Qe0`, `Qc1` against `Qc0.9762` (readings differ by 2.4 %), `Lp10` (one value per second), the pressure column in mode 3, `Ps`, `x`, `s` (settings kept) and the check against a reference gauge (T20).

### Step 7 (earlier plan, kept for reference)
- 1st-order IIR low-pass on the pressure reading: `y += alpha*(x-y)`, alpha from the real reading interval (`MPRLS_INTERVAL_MS`), `Qe`, `Qf` (0.1 to 4 Hz). Reset the filter on start and on `Qz`.
- `Qz` re-zero command (today zero is only taken when the sensor is initialized).
- Pressure output decimation `Lp`.
- Test: step pressure change, compare filter response with the expected time constant.

### Step 8 - OLED, button, battery - DONE, compiles, not yet tested on hardware
- **Implemented:** (1) Button as in the Thermistor firmware: debounced (50 ms); a short press switches between the status screen and the battery screen; holding it for 2 s shows "Release button to go to sleep" and releasing it goes to deep sleep; a release that belongs to the wake-up press is ignored. (2) Status screen (128 x 32, text size 1): row 1 mode (Audio, Pressure, Both, Off, No sensors), RUN or STOP and the battery percent at the right; row 2 the audio route (`L1>L2 16kHz N1`: input, output line, sample rate, subtraction mode) or the pressure low-pass state; rows 3 and 4 either the data links in use and the Bluetooth state (off, advertising, connected, streaming) with `LOW BATTERY` replacing the Bluetooth row below 10 %, or, while the pressure is measured, the pressure in large digits. (3) Battery screen as in the Thermistor firmware: charge and discharge boxes, rate in %/hr, battery icon, percent and voltage. (4) Anti-stall: the screens build their text first and only write the display (about 40 ms of I2C time at 100 kHz) when something changed; the loop calls them at most every 200 ms; the audio is a separate task and not affected. (5) One `enterDeepSleep(reason)` for the button and the 4.5 h auto shutoff without the old blocking 5 s delay: message on the display, audio faded out and shut down (codec powered down), BLE stopped, display off, wake on the button. (6) The battery gauge is read every 5 s (percent, voltage, charge rate); a low battery warning (below 10 %) is shown on the status screen and logged; `i` shows the battery.
- **Low battery shutdown (2026-11-07):** below 10 % the status screen shows `LOW BATTERY` and the log warns. At or below 5 % or 3.4 V (the 3.3 V regulator needs about 3.5 V under load, a brown-out of the rail must be avoided) for 20 s, without a USB host plugged in (`Serial.isPlugged()`, only available with the Hardware CDC mode of the S3) and while the battery is not charging (positive charge rate of the MAX17048), the board goes to deep sleep (`enterDeepSleep("Battery empty")`). A power-only charger has no USB host, so charging is used as the second sign of external power; a voltage below 2.5 V is a gauge without a battery and is ignored. Thresholds are `#define`s (`BATTERY_SHUTDOWN_PERCENT`, `BATTERY_SHUTDOWN_VOLT`, `BATTERY_SHUTDOWN_HOLD_MS`) and are listed in the limits of `i`. Test: set `BATTERY_SHUTDOWN_PERCENT` to 100 temporarily: on battery without USB it must shut down after 20 s with the log message; with the USB cable to a computer or while charging it must not.
- **Tests:** short press changes the screens, long press (2 s) shows the message and sleeps on release, wake with the button (settings and audio come back), the status screen follows `m`, `a`, `o`, `N`, `R`, `.` and `z`; `i` shows the battery; run `La1` with `z` and press the button repeatedly to see that the audio timing in `i` (gap, errors) does not change while the display refreshes; battery operation: unplug USB with a battery connected (the board reset earlier when the cable was unplugged, T18). Note: in deep sleep the 3.3 V rail of the board stays on, the amplifier and the microphone bias keep drawing current (a few mA), so the battery is slowly emptied in sleep.
- Button as in Thermistor: short press cycles screens (status, battery), long press = deep sleep. Debounce, ignore release after wake. Remove the blocking 5 s `delay()` in `enterDeepSleep()` (mute codec and stop audio first).
- Status screen: mode, running/stopped, input source, output line, pressure (size 2 when pressure is active), link icons (U/B), battery %. Battery screen: percent, charge rate, voltage.
- Anti-stall: redraw only when a value changed and no more often than every 200 ms; no redraw from inside the audio task; compare strings before `display.display()`.
- Keep 4.5 h auto shutoff, optionally a low-battery warning.
- Test: audio stays continuous while the OLED refreshes at its maximum rate.

### Future task - Review the BLESerial library (found during T9)
- **Stuck sender after an overload:** `writeReady()` is `!txLocked && isSubscribed()`; the lock is only released when the library's TX buffer drains below the low water mark, which needs the TX pump to get its status callback. After the `La4`/`La2` overload (and a lost connection) the pump stayed stuck with a full buffer while the link still reported connected and subscribed. Check the state machine (`txPending`, `Recovering`, `Discarding`), whether TX state and the lock are reset on a new connection or disconnect, and add a timeout for `Pending`. Use `Lbs` to see `locked`/`pending` when it happens. The sketch now has a watchdog as a safety net.
- **CPU and cores:** the TX pump task ("BLETxPump", priority 1) and the RSSI task (priority 2) are pinned to **core 1**; the NimBLE host task runs on core 0. On core 1 also run our audio task (priority 5, 11 % of the core at `La16`, about 34 % at `La1`) and the Arduino loop (priority 1) which never sleeps while measuring (it only calls `yield()`), so the pump shares the rest of the core with the busy loop. Tests: replace `yield()` in the loop by `delay(1)`; consider pinning the BLE tasks to core 0 (library change); measure the CPU load per task and the stack high water marks.
- **Throughput in Fast mode:** the received rate is twice the sent rate (see T9); find out whether lines are sent twice.
- **Overload behaviour:** a client must not be dropped (La2 lost the connection) when the offered rate is above what the link carries; let lines be dropped instead and keep the connection.

### Step 9 - Review default values
- Go through every default in one table (`setDefaults()` and the `#define`s) and check each against the hardware and the use case: sample rate, boot mode, autostart, HW input gain (Line 1 amplifier has gain 2, Line 2 is a jack mic), HW output gain (headphone safety at boot), software gain, filter cutoffs and on/off state (heart sounds about 20 to 400 Hz, lung sounds up to about 2 kHz), subtraction gain/bias, simulation tone frequencies/amplitudes/noise level, decimation, pressure low-pass, log level, BLE name, baud rate, auto shutoff time.
- Decide per default: keep, change, or make persistent. Update `Pr` (reset to factory defaults) and the help/`i` texts to match.
- Test: `Pr` then reboot gives a usable, safe (not loud) state on both outputs with each input source.

### Step 10 - Verification
- Compile for the Feather ESP32-S3; check free heap and loop time with audio + pressure + BLE + USB all on.
- Soak test: both mode, BLE + USB, 30 min, no dropped audio blocks.
- Update README / header comment and `APP_VERSION`.


### Decisions on the earlier open questions
1. Subtraction is always L - R within the same input; there is no subtraction between lines. On Line 2 the left and right inputs are hardwired to the same path, so subtraction there gives close to zero (expected, not a bug).
2. Start/stop only mutes and stops the data; a full audio stop is mode `m0` (audio and pressure off, battery monitor, display, button and commands stay on) or `m1` (pressure only).
3. The widened filter limits (high-pass 1 Hz to fs/8, low-pass fs/160 to 0.45*fs) are accepted.

## Hardware Description

### ES8388

#### Line 1:
Elektret microphone and to external audio amplifier output

Design:
- Elektret microphone
- Mic+ to 2.2k to V3.3
- Mic - to GND
- Mic + to 4.7uF to (A) (give highpass of about 3.4Hz)
- (A) with 1M to V bias 
- VBias to Amp in positive
- (A) with 10k to amplifier input negative
- Amplifier feedback with 20k // 500pF (16kHz lowpass)
- Gain is -2

Amp output with 1uF to Line 1; LIN1 or RIN1 

LOUT1, ROUT1 to 1 uF to stereo audio jack. With 10kOhm amplifier load this is about 16Hz.

#### Line 2:
Designed for headphone output. 

LIN2 & RIN2 connected together then 1uF to pin 1 of 4 pin audio jack

ROUT2, LOUT2 with 47uF to stereo out & mic audio jack

With 32 Ohm load and 47uF this is 106Hz highpass.

#### Checked against the schematic (Sound.kicad_sch, Rev 4, 2026-06-01)
- Two identical microphone amplifiers on one TLV9062 dual op-amp (electret connectors J5 and J6): 2.2k bias from A_3.3V, 4.7 uF coupling, 1M to Vmid, 10k input resistor, 20k // 500 pF feedback ("LP 16 kHz, Gain -2"), Vmid from a 47k/47k divider with 1 uF and 0.1 uF. This matches the Line 1 description above. The two amplifier outputs go through 1 uF to the Line 1 inputs, so left and right of Line 1 are two separate microphones.
- Line 2 headphone jack has two 47 uF coupling capacitors; the line out jack has 1 uF capacitors. This matches the description above.
- I2S lines (MCLK, SCLK, LRCK, DSDIN, ASDOUT) have 33 Ohm series resistors; SDA/SCL have 100 Ohm series resistors and 2.0k pull-ups. The CE pin of the ES8388 appears to be pulled low through 10k, consistent with the I2C address 0x10 used in the code (not certain at the resolution of the image).
- **I2S data pin roles (resolved from the KiCad netlist, Sound.kicad_pcb):** J2 pad 5 ("DOUT") -> R11 -> codec ASDOUT (codec data out), pad 6 ("LRCLK") -> R10 -> LRCK, pad 7 ("DIN") -> R9 -> DSDIN (codec data in), pad 8 ("SLCK") -> R7 -> SCLK, pad 9 ("MCLK") -> R6 -> MCLK. The pad names are from the codec's point of view. Assembly.md connects pad DIN to Feather MOSI / GPIO35 and pad DOUT to MISO / GPIO37, so **the ESP32 transmits on GPIO35 and receives on GPIO37**. The assembly instructions are correct and need no change. The firmware constants were reversed before (transmit on 37, receive on 35, with the comment "Assembled with switched in/out"); they are now `MY_I2S_DATA_TX` = 35 and `MY_I2S_DATA_RX` = 37. A prototype wired the old way must be rewired to the assembly table, or the two constants swapped. The KiCad netlist also shows IC1 pins 21 and 22 (RIN2, LIN2) on one net, as described for Line 2.

### MPRLS0300YG:
SDA, SCL
EOC, RST