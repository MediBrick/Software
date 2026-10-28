/******************************************************************************************************/
// AFE44XX Interactive Program
//
// Allows user to experiment with settings of the pulse oximeter AFE at runtime through a
// serial terminal (USB or BLE) command-line interface.
//
// This program demonstrates:
//   - Startup and configuration of the AFE44XX
//   - Register inspection and modification
//   - Health checks and diagnostics
//   - Real-time acquisition of background subtracted IR and RED readings
//   - Saving and restoring settings (NVS preferences and RAM snapshot)
//   - Output of data and command responses over USB serial and/or BLE serial
//
// The AFE functions are organized into the following groups:
//   - Photodiode receiver settings
//     - Transimpedance amplifier feedback resistor and capacitor
//     - Second amplifier stage gain
//   - LED transmitter settings
//     - Infrared and red LED power settings
//   - Timing
//     - Program timing settings from the AFE4490 datasheet
//       Modify default settings by a linear factor but maintaining ADC reset pulse, sampling offset and conversion offset
//     - Averaging of readings
//   - All other hardware options
//
// Command Interface:
//   Type '?' for help menu listing all commands
//   Single-letter commands control the program
//   Commands can be followed by numbers to set parameter values
//
// Urs Utzinger, 2024, 2026
/******************************************************************************************************/

#include <Arduino.h>
#include <Preferences.h>
#include <stdlib.h>
#include "afe44xx.h"
#include "logger.h"
#include "RingBuffer.h"
#include <BLESerial.h>
#include <Linereader.h>

/******************************************************************************************************/
// Constants
/******************************************************************************************************/

// Default Log Level (can be changed at runtime)
// LOG_LEVEL_DEBUG, LOG_LEVEL_INFO, LOG_LEVEL_WARN,
// LOG_LEVEL_ERROR, LOG_LEVEL_CRITICAL, LOG_LEVEL_NONE
#define LOG_LEVEL LOG_LEVEL_INFO

// Serial Communication
//////////////////////////////////////////////////////////////////////////////////////////////
constexpr unsigned long BAUDRATE         = 500'000UL;  // Up to 2,000,000 on ESP32, however more than 500 kBaud might be unreliable
const size_t   SERIAL_OUTPUT_BUFFER_SIZE = 4096U;
const size_t   SERIAL_LINE_BUFFER_SIZE   = 256U;
const size_t   SERIAL_OUTPUT_CHUNK_BYTES = 64U;
const uint32_t LOOP_INTERVAL_US          = 1000U;

// BLE Serial
//////////////////////////////////////////////////////////////////////////////////////////////
constexpr bool useTaskPump               = true;       // set to 'false' to use polling mode

// Hardware Pin Connection
//////////////////////////////////////////////////////////////////////////////////////////////
// Check assembly instructions for https://github.com/uutzinger/BioMedicalSensorBoard/blob/main/SPO2_Board/assembly.md
// DRDY, CS, PWDN connection are required for this software to run.
// If you do not want to use data ready interrupt, you can poll the data directly from the registers.
//
#define RESET_PIN    14  // active low, hardware reset, use if SPI not longer works with the sensor
#define DRDY_PIN     12  // data ready interrupt from AFE
#define CS_PIN       15  // chip select, active low, needed for SPI, you can hardwire this pin or activate when you have multipole SPI devices
#define PDALARM_PIN  32  // Photodiode Alarm pin, this can also be used to show activity of the AFE measurement process
#define LEDALARM_PIN 26  // LED Alarm pin, this can also be used to show activity of the AFE measurement process
#define DIAGEND_PIN  27  // This can be used to wait until diagnosis is finished, but it is usually not necessary
#define PWDN_PIN     25  // power down pin, active low, you can hardwire for always on and turn off LED, Photodiode and other hardware with software

/******************************************************************************************************/
// Global Configuration Variables
/******************************************************************************************************/

// AFE
//////////////////////////////////////////////////////////////////////////////////////////////
// these are needed for interactive user programming during runtime
/* Diagnostics */
uint32_t afe44xx_diag_status;         // holds results of the diagnosis function
/* Report data */
bool    data_reporting       = false; // continuously report IR and red measurements
const uint8_t DATA_OUT_SERIAL = 0x01U;
const uint8_t DATA_OUT_BLE    = 0x02U;
const uint8_t DATA_OUT_ALL    = DATA_OUT_SERIAL | DATA_OUT_BLE;
uint8_t data_output_mask      = DATA_OUT_SERIAL;  // where continuous data is sent
/* Receiver Stage  */
bool    afe44xx_sameGain     = true;  // same gain and feddback ressistor/capacitor for LED1 and LED2
int8_t  afe44xx_stage2Gain1  = 0;     // stage 2 gain for LED1: -1 disable, 0=1, 1=1.5, 2=2, 2=3, 4=4
int16_t afe44xx_CF1          = 5;     // feedback capacitor on Photodiode amplifier for LED1: 5pF, 5..275pF
int16_t afe44xx_RF1          = 500;   // Feedback resistor on Photodiode amplifier for LED1: 500kOhm, -1..1000kOhm, -1 = 0kOhm
uint8_t afe44xx_Fc           = 0;     // Low pass filter edge frequency 0=500Hz or 1=1000Hz
uint8_t afe44xx_CAmbient     = 0;     // Photodiode ambient current 0..9 micro Amp, you can measure by blocking the PD with black tape, depends on temperature
int8_t  afe44xx_stage2Gain2  = 0;     // stage 2 gain for LED 2, -1 disable, 0=1, 1=1.5, 2=2, 2=3, 4=4, its better to increase power of LED than to increase this gain
int16_t afe44xx_CF2          = 5;     // Feedback capacitor: 5pF 0..275 pF, same as LED1
int16_t afe44xx_RF2          = 500;   // Feedback resistor: 500kOhm -1..1000kOhm, same as LED1
/* Transmitter Stage */
uint8_t afe44xx_txRefVoltage = 0;     // Reference votlage for LEDs 1=0.5V, 0,2=0.75V, 3=1V, higher Voltage results in more current which increases brightness of LED
uint8_t afe44xx_range        = 0;     // Current range for LEDs 0=hig, 1=low, 2=high, 3=off // low is 50, 75, 100 mA, high 100, 150, 200 mA depending on voltage reference
uint8_t afe44xx_led1Power    = 24;    // LED1 power 0..255, this should be adjusted for each measurement subject so that readings are larger than 1,048,576 (half of max reading (2^21-1))
uint8_t afe44xx_led2Power    = 24;    // LED2 power 0..255
/* Digitial IO mode */
bool    afe44xx_tristate     = false; // Output logic tri state should be enabled if more than one SPI device is attached to the SPI bus, when the device is inactive to reduce power consumption
bool    afe44xx_hbridge      = true;  // use h-bridge for LED driver, this should be the default default
uint8_t afe44xx_alarmpins    = 0b101; // Photodiode and LED alarm pins can indicate activity, e.g. show that LED is sampled or LED is converted, see options below
/* 
Alarmpins
  PD Alarm pin                LED Alarm pin
  0 Sample LED2 pulse         Sample LED1 pulse
  1 LED2 LED pulse            LED1 LED pulse
  2 Sample LED2 ambient pulse Sample LED1 ambient pulse
  3 LED2 convert              LED1 convert
  4 LED2 ambient convert      LED1 ambient convert
  5 None                      None
  6 None                      None
  7 None                      None 
*/
float   afe44xx_factor       = 1.0; // Timing factor e.g. 0.5 or 8, 1 default, the default factor results in 500 samples per second.
/* 
Timing
  With a factor of 1.0 the default timing for 8000 clock cycles occurs. 
  At 4Mhz clock rate this will result in 500 measurements per second,
  on both red and ir channel including backgroudn subtraction.
  If averaging is set other than 1, the data rate will be reduced by the numer of averages.
  The default timing is taken from data sheet example. ADC reset pulse length is not changed by factor.
  If factor is <1 you will receive more samples per second, if its larger you will receive less samples per second.
  */
/* Hardware configuration*/
bool    afe44xx_bypassADC    = false; // use internal ADC, the default
bool    afe44xx_powerRX      = true;  // power the photodiode circuit, it will not work otherwise
bool    afe44xx_powerTX      = true;  // power the LED driver, it will not work otherwise
bool    afe44xx_powerAFE     = true;  // power the device, it will not work otherwise
uint8_t afe44xx_numAverage   = 4;     // number of averages caculated for LED1,2 and ambient measurements, this occurs inside the AFE
bool    afe44xx_running      = false; // keep track if system is running
bool    afe44xx_shortDiag    = true;  // short diagnostic 8ms or long diagnostics 16ms
bool    afe44xx_useXTAL      = true;  // use external crystal, otherwise provide clock signal, usual design is with crystal

// Persistence
//////////////////////////////////////////////////////////////////////////////////////////////
// The AFE44XX registers are volatile and are lost at power down or reset. The sketch globals above
// are the source of truth: they are written to the registers by setup() and by applySettings().
// - NVS preferences (Ps, Pl, Pd, Pc) keep these globals across reboot, they are loaded at boot.
// - RAM snapshot ( "(" and ")" ) keeps a copy of these globals until reboot.
const uint16_t AFE_PREF_SCHEMA_VERSION = 1U;
const char     AFE_PREF_NAMESPACE[]    = "afe44xx";

struct AfeSettings {
  bool    sameGain;
  int8_t  stage2Gain1, stage2Gain2;
  int16_t CF1, CF2, RF1, RF2;
  uint8_t Fc, CAmbient;
  uint8_t txRefVoltage, range, led1Power, led2Power;
  bool    tristate, hbridge, bypassADC, powerRX, powerTX, powerAFE, shortDiag, useXTAL;
  uint8_t alarmpins, numAverage;
  float   factor;
};
AfeSettings afe44xx_snapshot;
bool        afe44xx_snapshot_valid = false;

// Program Globals
//////////////////////////////////////////////////////////////////////////////////////////////
uint32_t      currentTime;                                                  // populated at beginning of main loop
uint32_t      lastTime;                                                     // to compute sampling frequency
float         samplingRate;                                                 // computed in main loop
uint32_t      sample_count = 0UL;                                           // number of samples read since start or reset
char          tmpStr[256];                                                  // general string buffer
const char    waitmsg[] = {"Waiting for serial terminal (5 seconds, press Enter to skip)"};
extern int    currentLogLevel;                                              // log level when using the logger.cpp provided logging functions

// Data retrieved from the AFE
//////////////////////////////////////////////////////////////////////////////////////////////
int32_t irRawValue = 0;      // raw value for IR LED data
int32_t redRawValue = 0;     // raw value for Red LED data
int32_t irAmbientValue = 0;  // Background reading for IR LED
int32_t redAmbientValue = 0; // Background reading for Red LED
int32_t irValue = 0;         // IR data, background subtracted by AFE
int32_t redValue = 0;        // Red data, background subtracted by AFE

// Serial buffer
//////////////////////////////////////////////////////////////////////////////////////////////
//  output buffers for chunking
//  input buffers for command line input
RingBuffer<char, SERIAL_OUTPUT_BUFFER_SIZE> serial_output_buff;
RingBuffer<char, SERIAL_OUTPUT_BUFFER_SIZE> ble_output_buff;
char          serial_input_buff[128];                // buffer for incoming serial data
// Temporary buffer for continuous data line reporting,
//   drains to serial and BLE output buffers
char          serial_line_buff[SERIAL_LINE_BUFFER_SIZE];
size_t        serial_line_len = 0U;

enum class CommandSource : uint8_t {
  UsbSerial,
  Ble
};

CommandSource active_command_source = CommandSource::UsbSerial;

// BLE Serial
//////////////////////////////////////////////////////////////////////////////////////////////
BLESerial     ble;
char          ble_input_buff[128];

// Line Reader for command input
//////////////////////////////////////////////////////////////////////////////////////////////
LineReader<128>         ble_lr;
LineReader<128>         serial_lr;

// Instantiate the device driver
//////////////////////////////////////////////////////////////////////////////////////////////
AFE44XX afe44xx(CS_PIN, PWDN_PIN, DRDY_PIN);

// Function declarations
//////////////////////////////////////////////////////////////////////////////////////////////
void processCommand(const char* input_buff, CommandSource source);
void handleGeneralCommand(char cmd);
void handleParameterCommand(const char* command);
void serialTrigger(const char* mess, int timeout);
void applySettings();
void helpMenu();
void showSettings();
void printPreferencesProfile();
bool savePreferencesProfile();
bool loadPreferencesProfile(bool report);
bool clearPreferencesProfile();
void captureSettings(AfeSettings &c);
void restoreSettings(const AfeSettings &c);
bool serialBufferWriteChar(char c);
bool serialBufferWrite(const char* text);
bool serialBufferWriteLine();
void serialOutUpdate(size_t max_bytes = SERIAL_OUTPUT_CHUNK_BYTES);

/******************************************************************************************************/
// Setup
/******************************************************************************************************/

void setup() {

  // Initialize globals
  // ------------------
  logSetLevel(LOG_LEVEL);
  afe44xx_diag_status = 0;

  // Serial
  // ------
  Serial.begin(BAUDRATE);       // setup serial
  Serial.setTimeout(1000);      // default is 1000
  serialTrigger(waitmsg, 5000); // give user time to open serial terminal

  LOGI("Starting");

  // Restore saved settings from NVS, if any
  loadPreferencesProfile(false);

  // BLE Serial
  // ----------
  ble.setPumpMode(useTaskPump ? BLESerial::PumpMode::Task : BLESerial::PumpMode::Polling);

  if (!ble.begin(
        BLESerial::Mode::Fast,        /*Fast, LowPower, LongRange, Balanced*/
        "BLESerialDevice",            /*Name*/
        BLESerial::Security::None     /*None, JustWorks, PasskeyDisplay*/
       )
     )  {
    LOGE("BLESerial begin() failed");
    while (true) delay(1000);
  }

  ble.setLogLevel(LOG_LEVEL);

  // AFE
  // ------
  afe44xx.begin();              // setup AFE: power cycle, SPI.being, clear control register

  // Additional wire connections
  pinMode(RESET_PIN,   OUTPUT); // Reset pin, active low
  digitalWrite(RESET_PIN,HIGH); // Reset pin high, high-low transition creates reset
  pinMode(PDALARM_PIN, INPUT);  // Photodiode Alarm, active high
  pinMode(LEDALARM_PIN,INPUT);  // LED Alarm, active high
  pinMode(DIAGEND_PIN, INPUT);  // Diag End, active high
  LOGI("AFE: Pins for RESET, PD&LED ALARM, DIAGEND set");

  // Hard reset (do a software reset instead)
  // LOGD("AFE: hard reset")
  // digitalWrite(RESET_PIN,LOW);
  // delay(100);
  // digitalWrite(RESET_PIN,HIGH);
  // delay(500);

  // Initialize the AFE to default values (instead do all individual steps as outline below)
  // afe44xx.initialze();

  // 0) Reset the AFE
  afe44xx.reset();                                          // software reset

  // 1) Configure Timer
  afe44xx.setTimers(afe44xx_factor);                        // set timing

  afe44xx.clearCNTRL0();                                    // clear CONTROL0 register

  // 2) Configure AFE
  afe44xx.bypassADC(afe44xx_bypassADC);                     // use internal/external ADC
  afe44xx.setOscillator(afe44xx_useXTAL);                   // use crystal or clock signal
  afe44xx.setShortDiagnostic(afe44xx_shortDiag);            // short or long diagnostic sequence
  afe44xx.setTristate(afe44xx_tristate);                    // tri state or push pull output
  afe44xx.setHbridge(afe44xx_hbridge);                      // h bridge or push pull output
  afe44xx.setAlarmpins(afe44xx_alarmpins);                  // monitor activity on PD and LED alarm pins  
  afe44xx.setPowerAFE(afe44xx_powerAFE);                    // power up/down
  afe44xx.setPowerRX(afe44xx_powerRX);                      // power up/down
  afe44xx.setPowerTX(afe44xx_powerTX);                      // power up/down

  // 3) Configure Receiver
  afe44xx.setSameGain(afe44xx_sameGain);                    // same or separate settings for LED1 and LED2
  afe44xx.setFirstStageFeedbackCapacitor(afe44xx_CF1,1);    // e.g. 5pF, LED1
  afe44xx.setFirstStageFeedbackCapacitor(afe44xx_CF2,2);    // e.g. 5pF,LED2
  afe44xx.setFirstStageFeedbackResistor(afe44xx_RF1,1);     // e.g. 500kOhm LED1
  afe44xx.setFirstStageFeedbackResistor(afe44xx_RF2,2);     // e.g. 500kOhm LED1
  afe44xx.setFirstStageCutOffFrequency(afe44xx_Fc );        // 500 or 100 Hz low pass
  afe44xx.setSecondStageGain(afe44xx_stage2Gain1, true, 1); // gain enabled, on LED1
  afe44xx.setSecondStageGain(afe44xx_stage2Gain2, true, 2); // gain enabled on LED2
  afe44xx.setCancellationCurrent(afe44xx_CAmbient);         // e.g. 0 microAmp

  // 4) Configure Transmitter
  afe44xx.setLEDdriver(afe44xx_txRefVoltage,afe44xx_range); // e.g. 0.75V, 150mA
  afe44xx.setLEDpower(afe44xx_led1Power, 1);                // e.g. 0..255 on both LEDs
  afe44xx.setLEDpower(afe44xx_led2Power, 2);                // e.g. 0..255 on both LEDs

  // 5) Start the continous measurement process
  afe44xx_running = afe44xx.start(afe44xx_numAverage);      // start system and engage averaging

  // Diagnostics
  // -----------
  LOGD("AFE: Register Dump");
  afe44xx.dumpRegisters();                                  // Display all readable registers
  /* Run Diagnostics and Print */  
  LOGD("AFE: Starting Diagnostics");
  afe44xx.readDiag(afe44xx_diag_status);                    // Run diagnostics
  afe44xx.printDiag(afe44xx_diag_status);                   // Display in readable format
  if (afe44xx.selfCheck()) { LOGD("AFE: Self Check Passed"); } else { LOGD("AFE: Self Check Failed") ; } // Run self check, system should be running
  LOGD("AFE: Read Timers");
  afe44xx.readTimers();
  // Read and display the status of the diagnostic pins
  bool pdAlarmStatus  = digitalRead(PDALARM_PIN);           // check pins
  bool ledAlarmStatus = digitalRead(LEDALARM_PIN);
  bool diagEndStatus  = digitalRead(DIAGEND_PIN);
  LOGI("AFE: Alarm Pins:");
  LOGI("  PD:        %s",  pdAlarmStatus ? "HIGH" : "LOW");
  LOGI("  LED:       %s", ledAlarmStatus ? "HIGH" : "LOW");
  LOGI("  Diag Ends: %s",  diagEndStatus ? "HIGH" : "LOW");

  LOGI("Setup complete, AFE started");

  lastTime = micros();
  samplingRate = 500.*afe44xx_factor/float(afe44xx_numAverage);

  LOGln("");
  LOGln("AFE44XX Interactive Program Starting");
  LOGln("======================================");
  LOGln("Ready for commands. Type '?' for help.");
  LOGln("");
}

/******************************************************************************************************/
// Main Loop
/******************************************************************************************************/

void loop() {

  currentTime = micros();                                   // keep track of loop time

  // Obtain new data with ISR
  // =======================================================
  if (afe44xx.afe44xx_data_ready) {
    afe44xx.getData(irRawValue, redRawValue, irAmbientValue, redAmbientValue, irValue, redValue);
    sample_count++;
    samplingRate = 0.9*samplingRate + 100000./float(currentTime-lastTime); // low pass filtered sampling rate
    lastTime = currentTime;
    if (data_reporting) {
      snprintf(tmpStr, sizeof(tmpStr), "IR: %9ld\tRed: %9ld\tIR_a: %9ld\tRed_a: %9ld\tS/s: %.1f",
               (long)irValue, (long)redValue, (long)irAmbientValue, (long)redAmbientValue, samplingRate);
      serialBufferWrite(tmpStr);
      serialBufferWriteLine();
    }
  }

  // BLE Serial Update
  // =======================================================
  if (!useTaskPump) {
    ble.update(); // polling pump
  }

  // Command Receiver (String)
  // =======================================================
  if (ble_lr.poll(ble, ble_input_buff, sizeof(ble_input_buff))) {
    logSetOutput(ble);
    processCommand(ble_input_buff, CommandSource::Ble);
  }

  if (serial_lr.poll(Serial, serial_input_buff, sizeof(serial_input_buff))) {
    logSetOutput(Serial);
    processCommand(serial_input_buff, CommandSource::UsbSerial);
  }

  // Send data in chunks over serial to avoid blocking
  // =======================================================
  serialOutUpdate();

  // Small delay, to maintain loop timing and prevent overwhelming the system
  // ===========
  const uint32_t elapsed_us = micros() - currentTime;
  if (elapsed_us < LOOP_INTERVAL_US) {
    delayMicroseconds(static_cast<unsigned int>(LOOP_INTERVAL_US - elapsed_us));
  }
}

/******************************************************************************************************/
// Support functions
//
// These functions are used to allow runtime configuration of the AFE44XX through user input.
// Single letters are used for general commands.
// Upper case commands indicate the command category and are followed by the function letter and optional number.
// Lower case commands with a number are z (data output) and l (log level).
/******************************************************************************************************/

// Boot helper - prints message and waits until timeout or user input
void serialTrigger(const char* mess, int timeout) {
  uint32_t startTime = millis();
  LOGln("");
  LOGln("%s", mess);
  while (!Serial.available() && ((millis() - startTime) < timeout)) {
    delay(500);
  }
  // Clear the serial input buffer
  while (Serial.available()) { Serial.read(); }
}

// Serial Output (Ring Buffer)
//////////////////////////////////////////////////////////////////////////////////////////////
// These are used to queue data for serial output, so that the main loop can
// continue processing data without blocking on Serial.write().

bool serialBufferWriteChar(char c) {
  if (serial_line_len >= sizeof(serial_line_buff)) {
    return false;
  }
  serial_line_buff[serial_line_len++] = c;
  return true;
}

bool serialBufferWrite(const char* text) {
  if (text == nullptr) {
    return true;
  }
  bool ok = true;
  while (*text != '\0') {
    ok = serialBufferWriteChar(*text++) && ok;
  }
  return ok;
}

bool serialBufferWriteLine() {
  bool ok = serialBufferWriteChar('\r');
  ok = serialBufferWriteChar('\n') && ok;
  if ((data_output_mask & DATA_OUT_SERIAL) != 0U) {
    ok = (serial_output_buff.push(serial_line_buff, serial_line_len) == serial_line_len) && ok;
  }
  // Copy data to the BLE output buffer only when a client is subscribed.
  // drainBleDataOutput() handles writeReady() flow control.
  if ((data_output_mask & DATA_OUT_BLE) != 0U && ble.isSubscribed()) {
    ok = (ble_output_buff.push(serial_line_buff, serial_line_len) == serial_line_len) && ok;
  }
  serial_line_len = 0U;
  return ok;
}

size_t drainSerialDataOutput(size_t max_bytes) {
  size_t sent = 0U;
  while (sent < max_bytes && serial_output_buff.available() > 0) {
    if (Serial.availableForWrite() <= 0) {
      break;
    }

    char c = '\0';
    if (serial_output_buff.peek(c) != 1U) {
      break;
    }
    if (Serial.write(static_cast<uint8_t>(c)) != 1U) {
      break;
    }
    serial_output_buff.consume();
    sent++;
  }
  return sent;
}

size_t drainBleDataOutput(size_t max_bytes) {
  if (!ble.isSubscribed()) {
    ble_output_buff.clear();
    return 0U;
  }
  if (!ble.writeReady()) {
    return 0U;
  }

  size_t sent = 0U;
  while (sent < max_bytes && ble_output_buff.available() > 0) {
    if (!ble.writeReady()) {
      break;
    }

    char c = '\0';
    if (ble_output_buff.peek(c) != 1U) {
      break;
    }
    if (ble.write(static_cast<uint8_t>(c)) != 1U) {
      break;
    }
    ble_output_buff.consume();
    sent++;
  }
  return sent;
}

// Write buffered data to enabled outputs in chunks.
void serialOutUpdate(size_t max_bytes) {
  if ((data_output_mask & DATA_OUT_SERIAL) != 0U) {
    drainSerialDataOutput(max_bytes);
  } else if (serial_output_buff.available() > 0) {
    serial_output_buff.clear();
  }

  if ((data_output_mask & DATA_OUT_BLE) != 0U) {
    drainBleDataOutput(max_bytes);
  } else if (ble_output_buff.available() > 0) {
    ble_output_buff.clear();
  }
}

void printDataOutputSettings() {
  LOG("Continuous data display: ");
  LOG("%s", data_reporting ? "ON" : "OFF");
  LOG(" | USB: ");
  LOG("%s", (data_output_mask & DATA_OUT_SERIAL) ? "ON" : "OFF");
  LOG(" | BLE: ");
  LOGln("%s", (data_output_mask & DATA_OUT_BLE) ? "ON" : "OFF");
}

// Persistence functions
//////////////////////////////////////////////////////////////////////////////////////////////

// Copy current settings globals into a struct
void captureSettings(AfeSettings &c) {
  c.sameGain = afe44xx_sameGain;
  c.stage2Gain1 = afe44xx_stage2Gain1; c.stage2Gain2 = afe44xx_stage2Gain2;
  c.CF1 = afe44xx_CF1; c.CF2 = afe44xx_CF2; c.RF1 = afe44xx_RF1; c.RF2 = afe44xx_RF2;
  c.Fc = afe44xx_Fc; c.CAmbient = afe44xx_CAmbient;
  c.txRefVoltage = afe44xx_txRefVoltage; c.range = afe44xx_range;
  c.led1Power = afe44xx_led1Power; c.led2Power = afe44xx_led2Power;
  c.tristate = afe44xx_tristate; c.hbridge = afe44xx_hbridge; c.bypassADC = afe44xx_bypassADC;
  c.powerRX = afe44xx_powerRX; c.powerTX = afe44xx_powerTX; c.powerAFE = afe44xx_powerAFE;
  c.shortDiag = afe44xx_shortDiag; c.useXTAL = afe44xx_useXTAL;
  c.alarmpins = afe44xx_alarmpins; c.numAverage = afe44xx_numAverage; c.factor = afe44xx_factor;
}

// Copy a struct into the settings globals (does not touch the AFE)
void restoreSettings(const AfeSettings &c) {
  afe44xx_sameGain = c.sameGain;
  afe44xx_stage2Gain1 = c.stage2Gain1; afe44xx_stage2Gain2 = c.stage2Gain2;
  afe44xx_CF1 = c.CF1; afe44xx_CF2 = c.CF2; afe44xx_RF1 = c.RF1; afe44xx_RF2 = c.RF2;
  afe44xx_Fc = c.Fc; afe44xx_CAmbient = c.CAmbient;
  afe44xx_txRefVoltage = c.txRefVoltage; afe44xx_range = c.range;
  afe44xx_led1Power = c.led1Power; afe44xx_led2Power = c.led2Power;
  afe44xx_tristate = c.tristate; afe44xx_hbridge = c.hbridge; afe44xx_bypassADC = c.bypassADC;
  afe44xx_powerRX = c.powerRX; afe44xx_powerTX = c.powerTX; afe44xx_powerAFE = c.powerAFE;
  afe44xx_shortDiag = c.shortDiag; afe44xx_useXTAL = c.useXTAL;
  afe44xx_alarmpins = c.alarmpins; afe44xx_numAverage = c.numAverage; afe44xx_factor = c.factor;
}

void printPreferencesProfile() {
  Preferences prefs;
  if (!prefs.begin(AFE_PREF_NAMESPACE, true)) { LOGln("Preferences open failed"); return; }
  const uint16_t schema = prefs.getUShort("schema", 0U);
  prefs.end();
  LOGln("==Saved NVS Settings============================================================");
  snprintf(tmpStr, sizeof(tmpStr), "Namespace: %s  Schema: %u (expected %u)", AFE_PREF_NAMESPACE, schema, AFE_PREF_SCHEMA_VERSION); LOGln("%s", tmpStr);
  if (schema != AFE_PREF_SCHEMA_VERSION) {
    LOGln("No compatible saved settings found");
  } else {
    LOGln("Saved settings exist, use Pl to load them, s to show active settings");
  }
  LOGln("================================================================================");
}

bool savePreferencesProfile() {
  Preferences prefs;
  if (!prefs.begin(AFE_PREF_NAMESPACE, false)) { LOGln("Preferences save failed: namespace open failed"); return false; }
  AfeSettings c; captureSettings(c);
  prefs.putUShort("schema", AFE_PREF_SCHEMA_VERSION);
  prefs.putBool("sameGain", c.sameGain);
  prefs.putChar("s2g1", c.stage2Gain1); prefs.putChar("s2g2", c.stage2Gain2);
  prefs.putShort("cf1", c.CF1); prefs.putShort("cf2", c.CF2);
  prefs.putShort("rf1", c.RF1); prefs.putShort("rf2", c.RF2);
  prefs.putUChar("fc", c.Fc); prefs.putUChar("camb", c.CAmbient);
  prefs.putUChar("txref", c.txRefVoltage); prefs.putUChar("range", c.range);
  prefs.putUChar("led1", c.led1Power); prefs.putUChar("led2", c.led2Power);
  prefs.putBool("tristate", c.tristate); prefs.putBool("hbridge", c.hbridge); prefs.putBool("bypass", c.bypassADC);
  prefs.putBool("pwrrx", c.powerRX); prefs.putBool("pwrtx", c.powerTX); prefs.putBool("pwrafe", c.powerAFE);
  prefs.putBool("shortdiag", c.shortDiag); prefs.putBool("xtal", c.useXTAL);
  prefs.putUChar("alarm", c.alarmpins); prefs.putUChar("navg", c.numAverage);
  prefs.putFloat("factor", c.factor);
  prefs.end();
  LOGln("Settings saved to NVS");
  return true;
}

// Load settings from NVS into globals; values are range checked, invalid ones keep the current value.
bool loadPreferencesProfile(bool report) {
  Preferences prefs;
  if (!prefs.begin(AFE_PREF_NAMESPACE, true)) {
    if (report) { LOGln("Preferences load failed: namespace open failed"); }
    return false;
  }
  if (prefs.getUShort("schema", 0U) != AFE_PREF_SCHEMA_VERSION) {
    prefs.end();
    if (report) { LOGln("No compatible saved settings found"); }
    return false;
  }
  AfeSettings c; captureSettings(c);  // defaults = current values
  c.sameGain    = prefs.getBool("sameGain", c.sameGain);
  c.stage2Gain1 = prefs.getChar("s2g1", c.stage2Gain1); if (c.stage2Gain1 < -1 || c.stage2Gain1 > 4) c.stage2Gain1 = afe44xx_stage2Gain1;
  c.stage2Gain2 = prefs.getChar("s2g2", c.stage2Gain2); if (c.stage2Gain2 < -1 || c.stage2Gain2 > 4) c.stage2Gain2 = afe44xx_stage2Gain2;
  c.CF1 = prefs.getShort("cf1", c.CF1); if (c.CF1 < 0 || c.CF1 > 275)   c.CF1 = afe44xx_CF1;
  c.CF2 = prefs.getShort("cf2", c.CF2); if (c.CF2 < 0 || c.CF2 > 275)   c.CF2 = afe44xx_CF2;
  c.RF1 = prefs.getShort("rf1", c.RF1); if (c.RF1 < -1 || c.RF1 > 1000) c.RF1 = afe44xx_RF1;
  c.RF2 = prefs.getShort("rf2", c.RF2); if (c.RF2 < -1 || c.RF2 > 1000) c.RF2 = afe44xx_RF2;
  c.Fc       = prefs.getUChar("fc", c.Fc);        if (c.Fc > 1)        c.Fc = afe44xx_Fc;
  c.CAmbient = prefs.getUChar("camb", c.CAmbient); if (c.CAmbient > 10) c.CAmbient = afe44xx_CAmbient;
  c.txRefVoltage = prefs.getUChar("txref", c.txRefVoltage); if (c.txRefVoltage > 3) c.txRefVoltage = afe44xx_txRefVoltage;
  c.range        = prefs.getUChar("range", c.range);        if (c.range > 3)        c.range = afe44xx_range;
  c.led1Power = prefs.getUChar("led1", c.led1Power);
  c.led2Power = prefs.getUChar("led2", c.led2Power);
  c.tristate  = prefs.getBool("tristate", c.tristate);
  c.hbridge   = prefs.getBool("hbridge", c.hbridge);
  c.bypassADC = prefs.getBool("bypass", c.bypassADC);
  c.powerRX   = prefs.getBool("pwrrx", c.powerRX);
  c.powerTX   = prefs.getBool("pwrtx", c.powerTX);
  c.powerAFE  = prefs.getBool("pwrafe", c.powerAFE);
  c.shortDiag = prefs.getBool("shortdiag", c.shortDiag);
  c.useXTAL   = prefs.getBool("xtal", c.useXTAL);
  c.alarmpins = prefs.getUChar("alarm", c.alarmpins); if (c.alarmpins > 7) c.alarmpins = afe44xx_alarmpins;
  c.numAverage = prefs.getUChar("navg", c.numAverage); if (c.numAverage < 1 || c.numAverage > 16) c.numAverage = afe44xx_numAverage;
  c.factor = prefs.getFloat("factor", c.factor); if (!(c.factor >= 0.1f && c.factor <= 100.0f)) c.factor = afe44xx_factor;
  prefs.end();
  restoreSettings(c);
  if (report) { LOGln("Settings loaded from NVS"); }
  return true;
}

bool clearPreferencesProfile() {
  Preferences prefs;
  if (!prefs.begin(AFE_PREF_NAMESPACE, false)) { LOGln("Preferences clear failed: namespace open failed"); return false; }
  const bool ok = prefs.clear();
  prefs.end();
  LOGln("%s", ok ? "Saved NVS settings cleared" : "Preferences clear failed");
  return ok;
}

// Write all settings to the AFE registers, restarts acquisition if it was running
void applySettings() {
  bool wasRunning = afe44xx_running;
  if ((bool)afe44xx_running == true) { afe44xx_running=afe44xx.stop(); }
  afe44xx.setTimers(afe44xx_factor);                        // default timing
  afe44xx.clearCNTRL0();                                    // clear CONTROL0 register
  afe44xx.bypassADC(afe44xx_bypassADC);                     // use internal/external ADC
  afe44xx.setOscillator(afe44xx_useXTAL);                   // use external crystal or clock signal
  afe44xx.setShortDiagnostic(afe44xx_shortDiag);            // short or long diagnostic sequence
  afe44xx.setTristate(afe44xx_tristate);                    // tri state or push pull output
  afe44xx.setHbridge(afe44xx_hbridge);                      // h bridge or push pull output
  afe44xx.setAlarmpins(afe44xx_alarmpins);                  // monitor activity on PD and LED alarm pins  
  afe44xx.setPowerAFE(afe44xx_powerAFE);                    // power up/down
  afe44xx.setPowerRX(afe44xx_powerRX);                      // power up/down
  afe44xx.setPowerTX(afe44xx_powerTX);                      // power up/down
  afe44xx.setSameGain(afe44xx_sameGain);                    // same or separate settings for LED1 and LED2
  afe44xx.setFirstStageFeedbackCapacitor(afe44xx_CF1,1);    // e.g. 5pF, LED1
  afe44xx.setFirstStageFeedbackCapacitor(afe44xx_CF2,2);    // e.g. 5pF,LED2
  afe44xx.setFirstStageFeedbackResistor(afe44xx_RF1,1);     // e.g. 500kOhm LED1
  afe44xx.setFirstStageFeedbackResistor(afe44xx_RF2,2);     // e.g. 500kOhm LED1
  afe44xx.setFirstStageCutOffFrequency(afe44xx_Fc );        // 500 or 100 Hz low pass
  afe44xx.setSecondStageGain(afe44xx_stage2Gain1, true, 1); // gain enabled, on LED1
  afe44xx.setSecondStageGain(afe44xx_stage2Gain2, true, 2); // gain enabled on LED2
  afe44xx.setCancellationCurrent(afe44xx_CAmbient);         // e.g. 0 microAmp
  afe44xx.setLEDdriver(afe44xx_txRefVoltage,afe44xx_range); // e.g. 0.75V, 150mA
  afe44xx.setLEDpower(afe44xx_led1Power, 1);                // e.g. 24 of 255 on both LEDs
  afe44xx.setLEDpower(afe44xx_led2Power, 2);                // e.g. 24 of 255 on both LEDs
  if (wasRunning == true) { afe44xx_running=afe44xx.start(afe44xx_numAverage); } // number of averages and start of system
}

// USER INTERFACE
//////////////////////////////////////////////////////////////////////////////////////////////

// Help Menu
void helpMenu() {
  LOGln("================================================================================");
  LOGln("| AFE4490 Pulse Oximeter Analog Front End Program                              |");
  LOGln("| 2026 Urs Utzinger                                                            |");
  LOGln("================================================================================");
  LOGln("| GENERAL COMMANDS                       | DATA COMMANDS                       |");
  LOGln("|----------------------------------------|-------------------------------------|");
  LOGln("| ?: help screen                         | z: toggle data reporting on/off     |");
  LOGln("| s: show current settings               | z[s|b|a]: USB, BLE, both, off       |");
  LOGln("| a: apply current settings (re-setup)   |                                     |");
  LOGln("| h: run health check                    | z[S|B|A]: USB, BLE, both, on        |");
  LOGln("| i: print device info                   | zz: print one sample                |");
  LOGln("| r: print all registers                 | c: reset sample counter             |");
  LOGln("| t: print status (diagnostics)          |                                     |");
  LOGln("| p: print timing registers              |                                     |");
  LOGln("| x: reboot                              |                                     |");
  LOGln("================================================================================");
  LOGln("| PERSISTENCE                            | RAM SNAPSHOT                        |");
  LOGln("|----------------------------------------|-------------------------------------|");
  LOGln("| Ps: save NVS preferences               | (: save settings snapshot           |");
  LOGln("| Pl: load NVS preferences               | ): restore settings snapshot        |");
  LOGln("| Pd: print NVS preferences              |                                     |");
  LOGln("| Pc: clear NVS preferences              |                                     |");
  LOGln("|========================================|=====================================|");
  LOGln("| ACQUISITION SETTINGS                   | START/STOP                          |");
  LOGln("|----------------------------------------|-------------------------------------|");
  LOGln("| Ma<n>: averages  (1-16)     Ma4        | .: toggle start/stop                |");
  LOGln("| Mt<f>: timing factor (0.1-100) Mt1     | >: start measurement                |");
  LOGln("| Mp<n>: alarm pins (0-7)       Mp5      | <: stop measurement                 |");
  LOGln("|========================================|=====================================|");
  LOGln("| RECEIVER SETTINGS                      | TRANSMITTER SETTINGS                |");
  LOGln("|----------------------------------------|-------------------------------------|");
  LOGln("| Rg1: stage2 gain LED1 (-1..4) Rg1-1    | TV<n>: ref voltage (0-3)     TV0    |");
  LOGln("| Rg2: stage2 gain LED2 (-1..4) Rg23     | TC<n>: current range (0-3)   TC0    |");
  LOGln("| RC1: feedback cap LED1 (pF)   RC15     | LP1<n>: LED1 power (0-255) LP1255   |");
  LOGln("| RC2: feedback cap LED2 (pF)   RC25     | LP2<n>: LED2 power (0-255) LP2255   |");
  LOGln("| RR1: feedback res LED1 (kOhm) RR1200   |                                     |");
  LOGln("| RR2: feedback res LED2 (kOhm) RR2200   |                                     |");
  LOGln("| RFC<n>: filter edge (0=500,1=1000 Hz)  |                                     |");
  LOGln("| RFB<n>: ambient current (0-9 uA) RFB1  |                                     |");
  LOGln("|========================================|=====================================|");
  LOGln("| HARDWARE OPTIONS                       | POWER                               |");
  LOGln("|----------------------------------------|-------------------------------------|");
  LOGln("| Xt1: same gain LED1&2 on/off           | Xt5: RX power on/off                |");
  LOGln("| Xt2: tri state on/off                  | Xt6: TX power on/off                |");
  LOGln("| Xt3: h bridge on/off                   | Xt7: AFE power on/off               |");
  LOGln("| Xt4: bypass ADC on/off                 |                                     |");
  LOGln("|========================================|=====================================|");
  LOGln("| LOG LEVEL                              | SPECIAL                             |");
  LOGln("|----------------------------------------|-------------------------------------|");
  LOGln("| l0: debug (verbose)                    | w: AFE software reset               |");
  LOGln("| l1: info (default)                     |                                     |");
  LOGln("| l2: warnings                           |                                     |");
  LOGln("| l3: errors only                        |                                     |");
  LOGln("| l4: critical only                      |                                     |");
  LOGln("| l5: none (silent)                      |                                     |");
  LOGln("================================================================================");
  LOGln("");
  LOGln("Examples:");
  LOGln("  Rg13     - Set stage 2 gain of LED1 to 3 (Rg2<n> for LED2)");
  LOGln("  LP1128   - Set LED1 power to 128");
  LOGln("  Mt0.5    - Set timing factor to 0.5 (1000 samples per second)");
  LOGln("  .        - Start/stop measurement");
  LOGln("  z        - Toggle continuous data display");
  LOGln("");
}

// Gain label for second stage gain index
const char* stage2GainLabel(int8_t gain) {
  switch (gain) {
    case -1: return "disabled";
    case 0:  return "1";
    case 1:  return "1.5";
    case 2:  return "2";
    case 3:  return "3";
    case 4:  return "4";
    default: return "?";
  }
}

// Show Current Settings
void showSettings() {
  LOGln("================================================================================");
  LOGln("Current Settings:");
  LOGln("================================================================================");
  LOGln("--General--");
  LOGln("Running: %s | Data Display: %s", afe44xx_running ? "YES" : "NO", data_reporting ? "ON" : "OFF");
  LOGln("Data Output: USB %s | BLE %s", (data_output_mask & DATA_OUT_SERIAL) ? "ON" : "OFF", (data_output_mask & DATA_OUT_BLE) ? "ON" : "OFF");
  LOGln("Log Level: %d", logGetLevel());
  LOGln("Timing factor: %.2f", afe44xx_factor);
  LOGln("Number of averages: %u", afe44xx_numAverage);
  LOGln("Sampling rate: %.1f S/s | Samples: %lu", samplingRate, static_cast<unsigned long>(sample_count));
  LOGln("--Receiver--");
  LOGln("Same Gain: %s", afe44xx_sameGain ? "On" : "Off");
  LOGln("Stage 2 Gain LED1: %s", stage2GainLabel(afe44xx_stage2Gain1));
  LOGln("Stage 2 Gain LED2: %s", stage2GainLabel(afe44xx_stage2Gain2));
  LOGln("LED1 Cf: %d pF | Rf: %d kOhm", afe44xx_CF1, afe44xx_RF1);
  LOGln("LED2 Cf: %d pF | Rf: %d kOhm", afe44xx_CF2, afe44xx_RF2);
  LOGln("Lowpass edge frequency: %s Hz", (afe44xx_Fc == 0) ? "500" : "1000");
  LOGln("Ambient background current: %d uA", afe44xx_CAmbient);
  LOGln("--Transmitter--");
  switch (afe44xx_txRefVoltage) {
    case 0:  LOGln("TX ref Voltage: %.2f V", 0.75); break;
    case 1:  LOGln("TX ref Voltage: %.2f V", 0.5);  break;
    case 2:  LOGln("TX ref Voltage: %.2f V", 0.75); break;
    case 3:  LOGln("TX ref Voltage: %.2f V", 1.0);  break;
    default: LOGln("TX ref Voltage: unknown");      break;
  }
  if (afe44xx_range == 3) {
    LOGln("Current range: Off");
  } else {
    // low range (1) is half of the high range (0, 2)
    static const int highRange[4] = {150, 100, 150, 200};  // mA for ref voltage 0, 1, 2, 3
    const int high = highRange[afe44xx_txRefVoltage & 3];
    LOGln("Current range: %d mA", (afe44xx_range == 1) ? high / 2 : high);
  }
  LOGln("LED1 power: %u", afe44xx_led1Power);
  LOGln("LED2 power: %u", afe44xx_led2Power);
  LOGln("--I/O--");
  LOGln("Tri State: %s | H Bridge: %s | Bypass ADC: %s", afe44xx_tristate ? "On" : "Off", afe44xx_hbridge ? "On" : "Off", afe44xx_bypassADC ? "On" : "Off");
  LOGln("Power RX: %s TX: %s AFE: %s", afe44xx_powerRX ? "On" : "Off", afe44xx_powerTX ? "On" : "Off", afe44xx_powerAFE ? "On" : "Off");
  LOGln("Alarm Pins: %u", afe44xx_alarmpins);
  LOGln("--Diagnostics--");
  afe44xx.readDiag(afe44xx_diag_status);
  afe44xx.printDiag(afe44xx_diag_status);
  LOGln("================================================================================");
}

/******************************************************************************************************/
// Command Handlers
/******************************************************************************************************/

void processCommand(const char* input_buff, CommandSource source) {
  if (input_buff == nullptr) return;

  while (*input_buff == ' ' || *input_buff == '\t') {
    input_buff++;
  }

  char command_buff[128];
  size_t len = strnlen(input_buff, sizeof(command_buff) - 1U);
  memcpy(command_buff, input_buff, len);
  command_buff[len] = '\0';
  while (len > 0U && (command_buff[len - 1U] == ' ' || command_buff[len - 1U] == '\t')) {
    command_buff[--len] = '\0';
  }

  if (len == 0U) return;  // Empty command

  active_command_source = source;

  // Single-character commands
  if (len == 1U) {
    handleGeneralCommand(command_buff[0]);
  }
  // Parameter commands
  else {
    handleParameterCommand(command_buff);
  }
}

void handleGeneralCommand(char cmd) {
  switch (cmd) {
    case '?':  // Help
      helpMenu();
      break;

    case 's':  // Show settings
      showSettings();
      break;

    case 'a':  // Apply settings
      applySettings();
      LOGln("Settings applied");
      break;

    case 'h':  // Health check
      {
        const bool wasRunning = afe44xx_running;
        if (wasRunning) { afe44xx_running = afe44xx.stop(); }
        afe44xx.readDiag(afe44xx_diag_status);
        afe44xx.printDiag(afe44xx_diag_status);
        const bool ok = afe44xx.selfCheck();
        if (wasRunning) { afe44xx_running = afe44xx.start(afe44xx_numAverage); }
        LOGln("Self check: %s", ok ? "PASS" : "FAIL");
      }
      break;

    case 'i':  // Print device info
      LOGln("==Device Info==");
      LOGln("");
      LOGln("AFE--");
      LOGln("  Running: %s", afe44xx_running ? "YES" : "NO");
      LOGln("  PD alarm pin: %s", digitalRead(PDALARM_PIN) ? "HIGH" : "LOW");
      LOGln("  LED alarm pin: %s", digitalRead(LEDALARM_PIN) ? "HIGH" : "LOW");
      LOGln("  Diag end pin: %s", digitalRead(DIAGEND_PIN) ? "HIGH" : "LOW");

      LOGln("");
      LOGln("Routing--");
      LOG("  Current command source/response destination: ");
      LOGln("%s", active_command_source == CommandSource::Ble ? "BLE" : "USB Serial");
      printDataOutputSettings();

      LOGln("");
      LOGln("USB Serial--");
      LOG("  Connected: ");
      LOGln("%s", static_cast<bool>(Serial) ? "YES" : "NO");
      LOG("  Available for write: ");
      LOGln("%d", Serial.availableForWrite());

      LOGln("");
      LOGln("BLE--");
      LOG("  Connected: ");
      LOGln("%s", ble.isConnected() ? "YES" : "NO");
      LOG("  Subscribed: ");
      LOGln("%s", ble.isSubscribed() ? "YES" : "NO");
      LOG("  Pump mode: ");
      LOGln("%s", ble.getPumpMode() == BLESerial::PumpMode::Task ? "Task" : "Polling");
      LOG("  TX buffer free: ");
      LOGln("%u", static_cast<unsigned>(ble.getTxFree()));
      if (active_command_source == CommandSource::Ble) {
        ble.printStats(ble);
      } else {
        ble.printStats(Serial);
      }
      break;

    case 'r':  // Print all registers
      afe44xx.dumpRegisters();
      break;

    case 't':  // Print status (diagnostics)
      {
        const bool wasRunning = afe44xx_running;
        if (wasRunning) { afe44xx_running = afe44xx.stop(); }
        afe44xx.readDiag(afe44xx_diag_status);
        afe44xx.printDiag(afe44xx_diag_status);
        if (wasRunning) { afe44xx_running = afe44xx.start(afe44xx_numAverage); }
      }
      break;

    case 'p':  // Print timing registers
      LOGln("==Timer Settings================================================================");
      afe44xx.readTimers();
      LOGln("================================================================================");
      break;

    case '(':  // Save settings snapshot
      captureSettings(afe44xx_snapshot);
      afe44xx_snapshot_valid = true;
      LOGln("Settings saved to snapshot");
      break;

    case ')':  // Restore settings snapshot
      if (!afe44xx_snapshot_valid) {
        LOGln("No settings snapshot available");
      } else {
        restoreSettings(afe44xx_snapshot);
        applySettings();
        LOGln("Settings restored from snapshot");
      }
      break;

    case 'w':  // Software reset
      LOGln("Performing software reset...");
      afe44xx.reset();
      afe44xx_running = false;
      LOGln("Reset complete. Use 'a' to apply settings and '>' to start.");
      break;

    case 'x':  // Reboot
      LOGln("Rebooting...");
      #ifdef ARDUINO_ARCH_ESP32
        ESP.restart();
      #else
        LOGln("Reboot not supported on this platform");
      #endif
      break;

    case 'c':  // Reset sample counter
      sample_count = 0UL;
      LOGln("Sample counter reset");
      break;

    case 'z':  // Toggle data display
      data_reporting = !data_reporting;
      if (data_reporting && data_output_mask == 0U) {
        data_output_mask = DATA_OUT_ALL;
      }
      printDataOutputSettings();
      break;

    case '.':  // Toggle start/stop
      if (afe44xx_running) {
        afe44xx_running = afe44xx.stop();
        LOGln("Stopped data acquisition");
      } else {
        afe44xx_running = afe44xx.start(afe44xx_numAverage);
        LOGln("Started data acquisition with average of %u", afe44xx_numAverage);
      }
      break;

    case '>':  // Start
      afe44xx_running = afe44xx.start(afe44xx_numAverage);
      LOGln("Started data acquisition with average of %u", afe44xx_numAverage);
      break;

    case '<':  // Stop
      if (afe44xx_running) {
        afe44xx_running = afe44xx.stop();
        LOGln("Stopped data acquisition");
      } else {
        LOGln("Not running");
      }
      break;

    default:
      LOGln("Unknown command");
      break;
  }
}

// Receiver (R), transmitter (T) and LED power (L) commands.
// Commands set tmpStr to a response message which is printed at the end.
void handleReceiverTransmitterCommand(const char* command) {
  long tmpI;                            // int
  int commandLen = strlen(command);     // length of command

  if (commandLen < 2) {
    snprintf(tmpStr, sizeof(tmpStr), "Command too short: %s chars. Check help with ?", command); LOGln("%s", tmpStr);
    return;
  }

  switch (command[0]) {

    // Check if first character is 'R' for Receiver Stage
    case 'R': {
      if (commandLen > 3) { // we have number
        tmpI = strtol(&command[3], NULL, 10); // string is R | X | LEADnum |value
      } else if (commandLen == 3) {
        tmpI = 0;
      } else {
        snprintf(tmpStr, sizeof(tmpStr), "Command too short: %s chars. Check help with ?", command); LOGln("%s", tmpStr);
        return;
      }
      switch (command[1]){
        case 'g': { // set gain
          if ( (tmpI < -1) || (tmpI>4)) {
            snprintf(tmpStr, sizeof(tmpStr), "Stage 2 gain outside of range [-1..4]");
          } else {
            if (command[2] == '1') {
              afe44xx_stage2Gain1 = tmpI;
              if (afe44xx.setSecondStageGain(afe44xx_stage2Gain1, true, 1) == 0) {
                snprintf(tmpStr, sizeof(tmpStr), "Stage 2 gain LED1 set to: %d", tmpI);
              } else {
                snprintf(tmpStr, sizeof(tmpStr), "Error: Stage 2 gain LED1 could not be set to: %d", tmpI);              
              }
            } else if (command[2] == '2') {
              afe44xx_stage2Gain2 = tmpI;
              if (afe44xx.setSecondStageGain(afe44xx_stage2Gain2, true, 2) == 2) {
                snprintf(tmpStr, sizeof(tmpStr), "Stage 2 gain LED2 set to: %d", tmpI);
              } else {
                snprintf(tmpStr, sizeof(tmpStr), "Error: Stage 2 gain LED2 could not be set to: %d", tmpI);              
              
              }
            } else {
              strcpy(tmpStr, "Invalid LED number for command 'Rg'");
            }
          }
          break;
        }
        case 'C': { // feedback capacitor
          if ( (tmpI < 0) || (tmpI>275)) {
            snprintf(tmpStr, sizeof(tmpStr), "Feedback capacitor outside of range [0..275]");
          } else {
            if (command[2] == '1') {
              afe44xx_CF1 = tmpI;
              if (afe44xx.setFirstStageFeedbackCapacitor(afe44xx_CF1,1) == 0) {
                snprintf(tmpStr, sizeof(tmpStr), "Feedback capacitor LED1 set to: %d", tmpI);
              } else {
                snprintf(tmpStr, sizeof(tmpStr), "Error: Feedback capacitor LED1 could not be set to: %d", tmpI); 
              }
            } else if (command[2] == '2') {
              afe44xx_CF2 = tmpI; 
              if (afe44xx.setFirstStageFeedbackCapacitor(afe44xx_CF2,2) == 0) {
                snprintf(tmpStr, sizeof(tmpStr), "Feedback capacitor LED2 set to: %d", tmpI);
              } else {
                snprintf(tmpStr, sizeof(tmpStr), "Error: Feedback capacitor LED2 could not be set to: %d", tmpI);
              } 
            } else {
              strcpy(tmpStr, "Invalid LED number for command 'C'");
            }
          }
          break;
        }
        case 'R': { // feedback resistor
          if ( (tmpI < -1) || (tmpI>1000)) {
            snprintf(tmpStr, sizeof(tmpStr), "Feedback resistor ouside of range [-1..1000]");
          } else {
            if (command[2] == '1') {
                afe44xx_RF1 = tmpI;
                if (afe44xx.setFirstStageFeedbackResistor(afe44xx_RF1,1)==0) {
                  snprintf(tmpStr, sizeof(tmpStr), "Feedback resistor LED1 set to: %d", tmpI);
                } else {
                  snprintf(tmpStr, sizeof(tmpStr), "Error: Feedback resistor LED1 could not be set to: %d", tmpI);
                }
            } else if (command[2] == '2') {
                afe44xx_RF2 = tmpI;
                if (afe44xx.setFirstStageFeedbackResistor(afe44xx_RF2,2)==0) {
                  snprintf(tmpStr, sizeof(tmpStr), "Feedback resistor LED2 set to: %d", tmpI);
                } else {
                  snprintf(tmpStr, sizeof(tmpStr), "Error: Feedback resistor LED2 could not be set to: %d", tmpI);
                } 
            } else {
              strcpy(tmpStr, "Invalid LED number for command 'R'");
            }
          }
          break;
        }
        case 'F': { 
          if (command[2] == 'C') { // edge frequency of low pass filter
            if ( (tmpI >= 0) && (tmpI<=1)) {
              afe44xx_Fc = tmpI;
              if (afe44xx.setFirstStageCutOffFrequency(afe44xx_Fc) == 0) {
                snprintf(tmpStr, sizeof(tmpStr), "Edge frequency set to: %d", tmpI);
              } else {
                snprintf(tmpStr, sizeof(tmpStr), "Error: Edge frequency could not be set to: %d", tmpI);
              }
            } else {
              snprintf(tmpStr, sizeof(tmpStr), "Edge frequency ouside of range [0,1]");
            }
          }
          else if (command[2] == 'B') { // ambient bias current for photodiode 
            if ( (tmpI >= 0) && (tmpI<=10)) {
              afe44xx_CAmbient=tmpI;
              if (afe44xx.setCancellationCurrent(afe44xx_CAmbient) == 0) {
                snprintf(tmpStr, sizeof(tmpStr), "Ambient background current set to: %d", tmpI);
              } else {
                snprintf(tmpStr, sizeof(tmpStr), "Error: Ambient background could not be set to: %d", tmpI);
              }
            } else {
              snprintf(tmpStr, sizeof(tmpStr), "Ambient background current outside of range");
            }
          }
          break;
        }
        default: {
          strcpy(tmpStr, "Invalid sub-command for 'R'");
          break;
        }
      }
      break; 
    }

    // Check if first character is 'T' for Transmitter Stage
    case 'T': {
      if (commandLen > 2) {
        tmpI = strtol(&command[2], NULL, 10);
      } else if (commandLen == 2) {
        tmpI = 0;
      } else {
        snprintf(tmpStr, sizeof(tmpStr), "Command too short: %s chars. Check help with ?", command); LOGln("%s", tmpStr);
        return;
      }
      switch (command[1]){
        case 'V': { // reference voltage
          if ( (tmpI >= 0) && (tmpI<=3)) {
            afe44xx_txRefVoltage =  tmpI; 
            if (afe44xx.setLEDdriver(afe44xx_txRefVoltage,afe44xx_range) == 0) {
              snprintf(tmpStr, sizeof(tmpStr), "Reference voltage set to: %d", tmpI);
            } else {
              snprintf(tmpStr, sizeof(tmpStr), "Error: Reference voltage could not be set to: %d", tmpI);
            }
          } else {
            snprintf(tmpStr, sizeof(tmpStr), "Reference voltage outside of range [0..3]");
          }
          break;
        }
        case 'C':{ // current range
          if ( (tmpI >= 0) && (tmpI<=3)) {
            afe44xx_range = tmpI;
            if (afe44xx.setLEDdriver(afe44xx_txRefVoltage,afe44xx_range) == 0) {
              snprintf(tmpStr, sizeof(tmpStr), "Current range set to: %d", tmpI);
            } else {
              snprintf(tmpStr, sizeof(tmpStr), "Error: Current range could not be set to: %d", tmpI);
            }
          } else {
            snprintf(tmpStr, sizeof(tmpStr), "Current range outside of range [0..3]");
          }
          break;
        }
        default: {
          strcpy(tmpStr, "Invalid sub-command for 'T'");
          break;
        }
      }
      break;
    }

    // Check if first character is 'L' for LED power
    case 'L': {
      if (commandLen > 3) {
        tmpI = strtol(&command[3], NULL, 10);
      } else if (commandLen == 3) {
        tmpI = 0;
      } else {
        snprintf(tmpStr, sizeof(tmpStr), "Command too short: %s. Check help with ?", command); LOGln("%s", tmpStr);
        return;
      }
      if (command[1] == 'P') {
        if ( (tmpI < 0) || (tmpI>255)) {
          snprintf(tmpStr, sizeof(tmpStr), "LED power range outside of range [0..255]");
        } else {
          switch (command[2]) {
            case '1': { // LED1
              afe44xx_led1Power =  tmpI;
              afe44xx.setLEDpower(afe44xx_led1Power, 1);
              snprintf(tmpStr, sizeof(tmpStr), "LED1 power set to: %d", tmpI);
              break;
            }
            case '2': { // LED2
              afe44xx_led2Power =  tmpI;
              afe44xx.setLEDpower(afe44xx_led2Power, 2);
              snprintf(tmpStr, sizeof(tmpStr), "LED2 power set to: %d", tmpI);
              break;
            }
            default: {
              strcpy(tmpStr, "Invalid LED number");
              break;
            }
          }
        }
      } else {
        strcpy(tmpStr, "Invalid sub-command for 'L'");
      }
      break;
    }

    default: {
      snprintf(tmpStr, sizeof(tmpStr), "Unknown command: %s", command);
      break;
    }

  }
  LOGln("%s", tmpStr);
}

void handleParameterCommand(const char* command) {
  const size_t command_len = strlen(command);
  const char cmd1 = command[0];
  const char cmd2 = (command_len > 1U) ? command[1] : '\0';
  const char* numeric_text = (cmd1 == 'l') ? &command[1] : &command[2];
  char* numeric_end = nullptr;
  const long value = strtol(numeric_text, &numeric_end, 10);
  const bool numeric_valid =
    numeric_text[0] != '\0' && numeric_end != numeric_text && *numeric_end == '\0';

  // Data reporting output commands
  if (cmd1 == 'z') {
    if (command_len != 2U) {
      LOGln("Data reporting commands must be exactly two characters");
      return;
    }
    switch (cmd2) {
      case 's':
        data_output_mask &= static_cast<uint8_t>(~DATA_OUT_SERIAL);
        break;

      case 'b':
        data_output_mask &= static_cast<uint8_t>(~DATA_OUT_BLE);
        break;

      case 'a':
        data_output_mask = 0U;
        break;

      case 'S':
        data_output_mask |= DATA_OUT_SERIAL;
        data_reporting = true;
        break;

      case 'B':
        data_output_mask |= DATA_OUT_BLE;
        data_reporting = true;
        break;

      case 'A':
        data_output_mask = DATA_OUT_ALL;
        data_reporting = true;
        break;

      case 'z':  // print one sample
        afe44xx.getData(irRawValue, redRawValue, irAmbientValue, redAmbientValue, irValue, redValue);
        LOGln("IR: %ld\tRed: %ld\tIR_r: %ld\tRed_r: %ld\tIR_a: %ld\tRed_a: %ld",
              (long)irValue, (long)redValue, (long)irRawValue, (long)redRawValue, (long)irAmbientValue, (long)redAmbientValue);
        return;

      default:
        LOGln("Unknown data reporting command");
        return;
    }
    if (data_output_mask == 0U) {
      data_reporting = false;
    }
    printDataOutputSettings();
    return;
  }

  // Log level commands
  if (cmd1 == 'l') {
    if (!numeric_valid || value < LOG_LEVEL_DEBUG || value > LOG_LEVEL_NONE) {
      LOGln("Log level must be 0-5 (debug, info, warn, error, critical, none)");
      return;
    }
    logSetLevel(static_cast<int>(value));
    LOG("Log level set to: ");
    LOGln("%ld", value);
    return;
  }

  // Non-volatile preferences commands. These store the sketch settings,
  // not raw AFE register snapshots.
  if (cmd1 == 'P') {
    if (command_len != 2U) {
      LOGln("Preferences commands must be exactly two characters");
      return;
    }
    switch (cmd2) {
      case 's':  // Save preferences
        savePreferencesProfile();
        break;

      case 'l':  // Load preferences
        if (loadPreferencesProfile(true)) { applySettings(); }
        break;

      case 'c':  // Clear preferences
        clearPreferencesProfile();
        break;

      case 'd':  // Display active preference profile
        printPreferencesProfile();
        break;

      default:
        LOGln("Unknown preferences command");
        break;
    }
    return;
  }

  // Acquisition settings
  if (cmd1 == 'M') {
    switch (cmd2) {
      case 'a': {  // number of averages
        if (!numeric_valid || value < 1 || value > 16) {
          LOGln("Number of averages out of range [1..16]");
          return;
        }
        afe44xx_numAverage = static_cast<uint8_t>(value);
        if (afe44xx_running) {
          afe44xx_running = afe44xx.stop();
          delay(10);
          afe44xx_running = afe44xx.start(afe44xx_numAverage);
          LOGln("Re-started data acquisition with average of %u", afe44xx_numAverage);
        } else {
          LOGln("System not running, number of averages set to: %u", afe44xx_numAverage);
        }
        return;
      }
      case 't': {  // timing factor
        const float factor = static_cast<float>(atof(numeric_text));
        if (numeric_text[0] == '\0' || factor < 0.1f || factor > 100.0f) {
          LOGln("Timing factor out of valid range [0.1..100.0]");
          return;
        }
        afe44xx_factor = factor;
        if (afe44xx_running) {
          afe44xx_running = afe44xx.stop();
          afe44xx.setTimers(afe44xx_factor);
          afe44xx_running = afe44xx.start(afe44xx_numAverage);
          LOGln("Re-started data acquisition with timing factor of %.2f and averaging of %u", afe44xx_factor, afe44xx_numAverage);
        } else {
          afe44xx.setTimers(afe44xx_factor);
          LOGln("System not running, timing factor set to: %.2f", afe44xx_factor);
        }
        return;
      }
      case 'p': {  // alarm pins
        if (!numeric_valid || value < 0 || value > 7) {
          LOGln("Alarm pins out of valid range [0..7]");
          return;
        }
        afe44xx_alarmpins = static_cast<uint8_t>(value);
        if (afe44xx.setAlarmpins(afe44xx_alarmpins) == 0) {
          LOGln("Alarm pins: %u", afe44xx_alarmpins);
        } else {
          LOGln("Error: Alarm pins could not be set to: %u", afe44xx_alarmpins);
        }
        return;
      }
      default:
        LOGln("Unknown acquisition command");
        return;
    }
  }

  // Hardware toggles
  if (cmd1 == 'X') {
    if (cmd2 != 't' || !numeric_valid || value < 1 || value > 7) {
      LOGln("Toggle command must be Xt1-Xt7");
      return;
    }
    switch (value) {
      case 1:
        afe44xx_sameGain = !afe44xx_sameGain;
        if (afe44xx.setSameGain(afe44xx_sameGain) == 0) {
          LOGln("Same gain: %s", afe44xx_sameGain ? "On" : "Off");
        } else {
          LOGln("Error: Same gain could not be set to: %s", afe44xx_sameGain ? "On" : "Off");
        }
        break;
      case 2:
        afe44xx_tristate = !afe44xx_tristate;
        afe44xx.setTristate(afe44xx_tristate);
        LOGln("Tri State: %s", afe44xx_tristate ? "On" : "Off");
        break;
      case 3:
        afe44xx_hbridge = !afe44xx_hbridge;
        afe44xx.setHbridge(afe44xx_hbridge);
        LOGln("H bridge: %s", afe44xx_hbridge ? "On" : "Off");
        break;
      case 4:
        afe44xx_bypassADC = !afe44xx_bypassADC;
        afe44xx.bypassADC(afe44xx_bypassADC);
        LOGln("Bypass ADC: %s", afe44xx_bypassADC ? "On" : "Off");
        break;
      case 5:
        afe44xx_powerRX = !afe44xx_powerRX;
        afe44xx.setPowerRX(afe44xx_powerRX);
        LOGln("Power RX: %s", afe44xx_powerRX ? "On" : "Off");
        break;
      case 6:
        afe44xx_powerTX = !afe44xx_powerTX;
        afe44xx.setPowerTX(afe44xx_powerTX);
        LOGln("Power TX: %s", afe44xx_powerTX ? "On" : "Off");
        break;
      case 7:
        afe44xx_powerAFE = !afe44xx_powerAFE;
        afe44xx.setPowerAFE(afe44xx_powerAFE);
        LOGln("Power AFE: %s", afe44xx_powerAFE ? "On" : "Off");
        break;
    }
    return;
  }

  // Receiver, transmitter and LED power
  if (cmd1 == 'R' || cmd1 == 'T' || cmd1 == 'L') {
    handleReceiverTransmitterCommand(command);
    return;
  }

  LOGln("Unknown command");
}

