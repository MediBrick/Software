/******************************************************************************************************/
// MAX30001G Interactive Program
//
// Allows user to experiment with ECG and BIOZ settings for the MAX30001G AFE at runtime through a
// serial terminal command-line interface.
//
// This program demonstrates:
//   - Initialization and configuration of MAX30001G AFE
//   - Switching between operation modes (ECG, BIOZ, combined, calibration, scanning)
//   - Register inspection and modification
//   - Health checks and diagnostics
//   - Real-time data acquisition and display
//   - Testing all library features interactively
//
// Operation Modes:
//   - ECG:         Single-channel ECG acquisition (2-lead or 3-lead)
//   - BIOZ:        Bio-impedance  measurement
//   - ECG+BIOZ:    Simultaneous ECG and BIOZ
//   - ECG Cal:     ECG signal calibration (internal)
//   - BIOZ Cal:    BIOZ signal calibration (internal)
//   - BIOZ Int Cal: Internal impedance calibration with known resistor
//   - BIOZ Ext Cal: External impedance calibration for custom resistors
//   - BIOZ Scan:   Impedance spectroscopy frequency sweep
//
// Command Interface:
//   Type '?' for help menu listing all commands
//   Single-letter commands control operation modes and settings
//   Commands can be followed by numbers to set parameter values
//
// Urs Utzinger, 2026
// OpenAI Codex assisted development
/******************************************************************************************************/

#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include "max30001g.h"
#include "logger.h"
#include "RingBuffer.h"
#include <BLESerial.h>
#include <Linereader.h>
#include <StreamString.h>

#include <Adafruit_MAX1704X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ESP32 RTC and sleep control (guarded for non-ESP32 IntelliSense)
#if defined(ARDUINO_ARCH_ESP32)
  #include "driver/rtc_io.h"
  #include "esp_sleep.h"
  #include "driver/gpio.h"
#else
  // Minimal stubs to satisfy IntelliSense when not using ESP32 toolchain
  typedef int gpio_num_t;
  static inline void rtc_gpio_init(int) {}
  static inline bool rtc_gpio_is_valid_gpio(int) { return false; }
  static inline void rtc_gpio_set_direction(int, int) {}
  static inline void rtc_gpio_pullup_dis(int) {}
  static inline void rtc_gpio_pulldown_en(int) {}
  static inline void gpio_hold_en(int) {}
  static inline void gpio_hold_dis(int) {}
  static inline void gpio_deep_sleep_hold_en() {}
  static inline void gpio_deep_sleep_hold_dis() {}
  static inline void esp_sleep_enable_ext0_wakeup(int, int) {}
  static inline void esp_deep_sleep_start() {}
  typedef enum {
    ESP_SLEEP_WAKEUP_UNDEFINED = 0,
    ESP_SLEEP_WAKEUP_EXT0,
    ESP_SLEEP_WAKEUP_EXT1,
    ESP_SLEEP_WAKEUP_TIMER,
    ESP_SLEEP_WAKEUP_TOUCHPAD,
    ESP_SLEEP_WAKEUP_ULP
  } esp_sleep_wakeup_cause_t;
  static inline esp_sleep_wakeup_cause_t esp_sleep_get_wakeup_cause() { return ESP_SLEEP_WAKEUP_UNDEFINED; }
  #ifndef GPIO_NUM_9
  #define GPIO_NUM_9 9
  #endif
#endif

/******************************************************************************************************/
// Constants
/******************************************************************************************************/

// Default Log Level (can be changed at runtime)
// LOG_LEVEL_DEBUG, LOG_LEVEL_INFO, LOG_LEVEL_WARN,
// LOG_LEVEL_ERROR, LOG_LEVEL_CRITICAL, LOG_LEVEL_NONE
#define LOG_LEVEL LOG_LEVEL_INFO

// LED
//////////////////////////////////////////////////////////////////////////////////////////////
constexpr uint8_t  LED_PIN               = LED_BUILTIN;
constexpr uint32_t BLINK_INTERVAL_ON_US  = 200'000U;
constexpr uint32_t BLINK_INTERVAL_OFF_US = 800'000U;

// Button
//////////////////////////////////////////////////////////////////////////////////////////////
constexpr int      BUTTON_PIN            = 9;
constexpr gpio_num_t BUTTON_PIN_GPIO     = GPIO_NUM_9;
constexpr uint32_t BUTTON_DEBOUNCE_US    = 50'000U;
constexpr uint32_t BUTTON_LONG_PRESS_US  = 2'000'000U;

// Battery Indicator
//////////////////////////////////////////////////////////////////////////////////////////////
constexpr uint32_t BATTERY_UPDATE_INTERVAL_US = 2'000'000U;

// Serial Communication
//////////////////////////////////////////////////////////////////////////////////////////////
constexpr unsigned long BAUDRATE         = 2'000'000UL;
const size_t   SERIAL_OUTPUT_BUFFER_SIZE = 4096U;
const size_t   SERIAL_OUTPUT_CHUNK_BYTES = 64U;
const uint32_t LOOP_INTERVAL_US          = 1000U;

// BLE Serial
//////////////////////////////////////////////////////////////////////////////////////////////
constexpr bool useTaskPump               = true;       // set to 'false' to use polling mode

// Startup, FIFO, and timeout policy
//////////////////////////////////////////////////////////////////////////////////////////////
const uint32_t PLL_STARTUP_SETTLE_MS            = 500U;
const uint32_t PLL_STATUS_CLEAR_MS              = 10U;
const uint32_t SCAN_TIMEOUT_MS                  = 180000U;
const uint32_t ECG_DATA_TIMEOUT_MIN_MS          = 100U;
const uint32_t BIOZ_DATA_TIMEOUT_MIN_MS         = 250U;
const uint8_t  ECG_FIFO_INTERRUPT_THRESHOLD     = 32U;
const uint8_t  BIOZ_FIFO_INTERRUPT_THRESHOLD    = 8U;
const uint8_t  ECG_CAL_FIFO_INTERRUPT_THRESHOLD = 8U;
const uint8_t  BIOZ_IMPEDANCE_FIFO_THRESHOLD    = 1U;
const uint8_t  NO_FIFO_THRESHOLD_OVERRIDE       = 255U;
const uint8_t  ECG_FILTER_AUTO                  = 255U;
const float    ECG_NOTCH_Q_DEFAULT              = 20.0f; // increase for narrower band

// Hardware Pin Connections
//////////////////////////////////////////////////////////////////////////////////////////////
// Adjust these to match your hardware configuration
const uint8_t  AFE_CS_PIN                       = 6;    // SPI chip select
const int      AFE_INT1_PIN                     = 12;   // INTB interrupt pin
const int      AFE_INT2_PIN                     = 13;   // INT2B interrupt pin

// Display
//////////////////////////////////////////////////////////////////////////////////////////////
constexpr uint32_t DISPLAY_BATTERY_UPDATE_INTERVAL_US = 1'000'000U;
constexpr uint32_t DISPLAY_DATA_UPDATE_INTERVAL_US    = 100'000U;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // 32 or 64 OLED display height, in pixels
#define OLED_RESET -1 // Not used
#define SCREEN_ADDRESS 0x3C

#define BAT_X 0
#define BAT_Y 16
#define BAT_W 64
#define BAT_H 16
#define BAT_FULL (BAT_W-9) // Define your constant width here
#define BAT_THICK 10 // Define your constant width here
#define CHRG_X 1
#define CHRG_Y 1

#define A1_X 1
#define A1_Y 1
#define A_W 126
#define A_H 14
#define A2_X 1
#define A2_Y 2 + A_H

const unsigned char BATTERY_BITMAP[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 00000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00000000
  0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0,  // 00111111, 11111111, 11111111, 11111111, 01111111, 11111111, 11111111, 11110000
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,  // 01000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00010000
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18,  // 01000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00011000
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18,  // 01000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00011000
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E,  // 01000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00011110
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1A,  // 01000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00011010
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1A,  // 01000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00011010
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1A,  // 01000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00011010
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1A,  // 01000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00011010
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E,  // 01000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00011110
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18,  // 01000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00011000
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18,  // 01000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00011000
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,  // 01000000, 00000000, 00000000, 00010000, 00000000, 00000000, 00000000, 00010000
  0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0,  // 00111111, 11111111, 11111111, 11111111, 11111111, 11111111, 11111111, 11110000
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00   // 00000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00000000, 00000000
};

/******************************************************************************************************/
// Operation Modes
/******************************************************************************************************/
enum OperationMode {
  MODE_IDLE = 0,
  MODE_ECG,
  MODE_BIOZ,
  MODE_ECG_BIOZ,
  MODE_ECG_CAL,
  MODE_BIOZ_CAL,
  MODE_BIOZ_INTERNAL_CAL,
  MODE_BIOZ_EXTERNAL_CAL,
  MODE_BIOZ_SCAN
};

/******************************************************************************************************/
// Global Configuration Variables
/******************************************************************************************************/

// AFE
//////////////////////////////////////////////////////////////////////////////////////////////

OperationMode current_mode    = MODE_IDLE;
bool          measurement_running = false;
bool          data_reporting  = false;  // Continuous data reporting
const uint8_t DATA_OUT_SERIAL = 0x01U;
const uint8_t DATA_OUT_BLE    = 0x02U;
const uint8_t DATA_OUT_ALL    = DATA_OUT_SERIAL | DATA_OUT_BLE;
uint8_t data_output_mask      = DATA_OUT_SERIAL;

// ECG Configuration
uint8_t ecg_speed      = 1;                   // 0=~125sps, 1=~256sps, 2=~512sps
uint8_t ecg_gain       = 2;                   // 0=20V/V, 1=40V/V, 2=80V/V, 3=160V/V
bool    ecg_threeleads = true;                // true=3-lead, false=2-lead
bool    ecg_rtor       = true;                // Enable R-to-R detection
uint8_t ecg_lpf        = ECG_FILTER_AUTO;     // 0=bypass, 1=40Hz, 2=100Hz, 3=150Hz, 255=mode default
uint8_t ecg_hpf        = ECG_FILTER_AUTO;     // 0=bypass, 1=0.5Hz, 255=mode default
uint8_t ecg_notch      = 0U;                  // 0=off, 50=50Hz notch, 60=60Hz notch
float   ecg_notch_q    = ECG_NOTCH_Q_DEFAULT; // notch Q factor, higher=narrower notch

// BIOZ Configuration
uint8_t  bioz_speed = 0;       // 0=low-rate (~25-32sps), 1=high-rate (~50-64sps)
uint8_t  bioz_gain = 1;        // 0=10V/V, 1=20V/V, 2=40V/V, 3=80V/V
uint8_t  bioz_ahpf = 1;        // Analog HPF: 0=60Hz, 1=150Hz, 2=500Hz, 3=1kHz, 4=2kHz, 5=4kHz, 6/7=bypass
uint8_t  bioz_dlpf = 1;        // Digital LPF: 0=bypass, 1=~4Hz, 2=~8Hz, 3=~16Hz
uint8_t  bioz_dhpf = 0;        // Digital HPF: 0=bypass, 1=0.05Hz, 2/3=0.5Hz
uint32_t bioz_frequency = 8000;   // Modulation frequency in Hz
uint32_t bioz_current = 8000;     // Drive current in nA
float    bioz_phase = 0.0f;          // Demodulation phase offset in degrees
bool     bioz_leadbias = true;        // Enable lead bias
bool     bioz_leadsoffdetect = false; // Enable lead-off detection
bool     bioz_fourleads = false;      // 4-wire vs 2-wire BIOZ

// BIOZ Scan Configuration
uint8_t scan_avg = 8;         // Number of averages per frequency point (1-8)
bool    scan_fast = false;       // Fast scan mode (60sps vs 30sps)
bool    scan_fullrange = false;  // Include lowest frequencies
bool    scan_internal = false;   // false=external scan, true=internal resistor scan
BIOZScanPhaseRange scan_phase_range = BIOZ_SCAN_PHASE_FULL; // FULL or REDUCED
BIOZScanAHPFMode scan_ahpf_mode = BIOZ_SCAN_AHPF_DYNAMIC; // dynamic, fixed, or bypass
uint8_t scan_fixed_ahpf = 1U; // fixed scan AHPF selector: 1=150Hz
uint8_t scan_internal_bist_ahpf = 255U; // 255=dynamic table, 0..7=fixed internal-BIST AHPF
uint8_t scan_settle_samples = 24U; // Samples to discard after phase/frequency/filter changes
uint8_t scan_current_change_settle_samples = 24U; // Samples to discard after current changes
int32_t scan_max_current_nA = 96000; // Hard scan current ceiling
bool    scan_autorange_enable = true; // Allow scan current optimizer
bool    scan_human_safe = false; // Preset: external 4-wire fixed-current scan at <=8 uA

// BIOZ Calibration
uint32_t cal_resistance = 1000;  // Internal calibration resistor value (Ohms)
uint8_t cal_modulation = 0;      // Calibration modulation mode
uint8_t cal_mod_freq = 3;        // Calibration modulation frequency
bool ecg_cal_unipolar = true;    // ECG VCAL mode: true=unipolar, false=bipolar
bool bioz_cal_unipolar = false;  // BIOZ VCAL mode: true=unipolar, false=bipolar

// BIOZ signal calibration uses internal VCAL voltage, not drive current.
// Keep this mode raw because Ohm conversion divides by BIOZ current magnitude.
const bool BIOZ_CAL_ROUTE_ECG = false;
const bool BIOZ_CAL_ROUTE_BIOZ = true;
const bool BIOZ_CAL_0P5_MV = true;    // false=0.25mV, true=0.5mV
const uint8_t BIOZ_CAL_FCAL_1HZ = 0b100;
const uint8_t BIOZ_CAL_DUTY_PERCENT = 50;
const uint8_t BIOZ_CAL_AHPF_BYPASS = 6;
const uint8_t BIOZ_CAL_DLPF_BYPASS = 0;
const uint8_t BIOZ_CAL_DHPF_BYPASS = 0;

// External BIOZ scan calibration. Source defaults are conservative; optional
// ESP32 Preferences can persist calibration and scan defaults across reboot.
const float BIOZ_SCAN_CAL_DEFAULT_GLOBAL_K = 1.25065f;
const uint32_t BIOZ_SCAN_CAL_DEFAULT_GLOBAL_K_PPM = 1250650UL;
const uint16_t BIOZ_PREF_SCHEMA_VERSION = 1U;
const uint16_t BIOZ_SCAN_CAL_VERSION = 1U;
const uint16_t BIOZ_RELIABILITY_MAP_VERSION = 1U;
const char BIOZ_PREF_NAMESPACE[] = "max30001g";
bool bioz_scan_cal_enabled = false;
float bioz_scan_cal_global_k = BIOZ_SCAN_CAL_DEFAULT_GLOBAL_K;

// Program Globals
//////////////////////////////////////////////////////////////////////////////////////////////
struct ModeRuntimePolicy;

// Main Loop Timing
uint32_t      current_time;
uint32_t      last_time                   = 0;
uint32_t      sample_count                = 0;
uint32_t      last_sample_count           = 0;
uint32_t      last_blink_us               = 0;       // last LED blink time (microseconds)

uint32_t last_button_change_us = 0;
uint32_t button_pressed_us = 0;
uint32_t last_display_us = 0;
uint32_t last_battery_us = 0;
uint32_t display_update_interval_us = DISPLAY_BATTERY_UPDATE_INTERVAL_US;
bool display_update_requested = false;

//
float         sampling_rate               = 0.0f;    // measured sampling rate (Hz) for current mode
uint8_t       serial_index                = 0;       // index of the current character in the serial input buffer
bool          serial_command_complete     = false;   // true when a complete command line has been received
uint32_t      last_ecg_data_ms            = 0;       // last ECG data received time (milliseconds)
uint32_t      last_bioz_data_ms           = 0;       // last BIOZ data received time (milliseconds)
uint32_t      mode_start_ms               = 0;       // time when the current mode was started (milliseconds)
uint8_t       active_ecg_fifo_threshold   = ECG_FIFO_INTERRUPT_THRESHOLD;
uint8_t       active_bioz_fifo_threshold  = BIOZ_FIFO_INTERRUPT_THRESHOLD;
bool          ecg_timeout_reported        = false;
bool          bioz_timeout_reported       = false;
bool          scan_timeout_reported       = false;
int           saved_measurement_log_level = LOG_LEVEL; // saved log level before entering measurement mode
bool          measurement_log_level_saved = false;    // true if log level was saved before entering measurement mode
const uint8_t MAX_PENDING_RTOR            = 8U;       // maximum number of pending R-to-R intervals to store for reporting
float         pending_rtor_ms[MAX_PENDING_RTOR];      // array to store pending R-to-R intervals in milliseconds
uint8_t       pending_rtor_count          = 0U;       // number of pending R-to-R intervals stored in the array
uint32_t      rtor_seen_count             = 0UL;      // total number of R-to-R intervals seen since program start
uint32_t      rtor_printed_count          = 0UL;      // total number of R-to-R intervals printed to serial since program start
uint32_t      rtor_dropped_count          = 0UL;      // total number of R-to-R intervals dropped due to buffer overflow since program start
const char    waitmsg[] = {"Waiting for serial terminal (5 seconds, press Enter to skip)"};
extern int    currentLogLevel;

// LED
//////////////////////////////////////////////////////////////////////////////////////////////
bool          led_state                   = LOW;     // current LED state

// Button
//////////////////////////////////////////////////////////////////////////////////////////////
volatile bool button_edge_pending = false;
int           button_raw_state = LOW;
int           button_state = LOW;

// Interrupt function for different states of OLED when button is pressed
void IRAM_ATTR handleButtonPress() {
  button_edge_pending = true;
}

// Serial buffer
//////////////////////////////////////////////////////////////////////////////////////////////
//  output buffer for chunking
//  input buffer for command line input
RingBuffer<char, SERIAL_OUTPUT_BUFFER_SIZE> serial_output_buff;
RingBuffer<char, SERIAL_OUTPUT_BUFFER_SIZE> ble_output_buff;
char          serial_input_buff[128];                // buffer for incoming serial data
// Temporary buffer for continous data line reporting,
//   drains to serial and BLE output buffers
char          serial_line_buff[SERIAL_OUTPUT_BUFFER_SIZE];
size_t        serial_line_len = 0U;

enum class CommandSource : uint8_t {
  UsbSerial,
  Ble
};

void processCommand(const char* input_buff, CommandSource source);

// uint32_t serial_command_count = 0U;
// uint32_t ble_command_count = 0U;
// size_t serial_command_length = 0U;
// size_t ble_command_length = 0U;
// char serial_last_command[128] = "";
// char ble_last_command[128] = "";
CommandSource active_command_source = CommandSource::UsbSerial;

// Instantiate the MAX30001G driver
//////////////////////////////////////////////////////////////////////////////////////////////
MAX30001G afe(AFE_CS_PIN, AFE_INT1_PIN, AFE_INT2_PIN);

// Notchfilter
//////////////////////////////////////////////////////////////////////////////////////////////
struct ECGNotchFilterState {
  bool enabled;
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;
  float z1;
  float z2;
  float center_hz;
  float sample_rate_hz;
};

ECGNotchFilterState ecg_notch_filter = {false, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// BLE Serial
//////////////////////////////////////////////////////////////////////////////////////////////
BLESerial     ble;
char          ble_input_buff[128];

// Line Reader for command input
//////////////////////////////////////////////////////////////////////////////////////////////
LineReader<128>         ble_lr;
LineReader<128>         serial_lr;

// Battery Indicator
//////////////////////////////////////////////////////////////////////////////////////////////
Adafruit_MAX17048 lipo;
bool battery_available = false;
float charge_rate_percent_per_hour = NAN;
float cell_percent = NAN;

// Display
//////////////////////////////////////////////////////////////////////////////////////////////
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool display_available = false;

// Define different states for OLED
enum DisplayMode {
  DISPLAY_BATTERY,
  DISPLAY_DATA
};

DisplayMode display_mode = DISPLAY_BATTERY;

/******************************************************************************************************/
// Helper Functions
/******************************************************************************************************/

void dataToSerialBuff();
void serialOutUpdate(size_t max_bytes = SERIAL_OUTPUT_CHUNK_BYTES);
void serialOutFlush(uint32_t timeout_ms = 500U);
void printDataOutputSettings();
void queueMeasurementState();
void requestStartMeasurement();
void toggleMeasurement();
bool serialBufferWriteChar(char c);
bool serialBufferWrite(const char* text);
bool serialBufferWriteFloat(float value, uint8_t decimals);
bool serialBufferWriteUInt(uint32_t value);
bool serialBufferWriteLine();
void printHzOrBypass(float hz, uint8_t decimals = 2U);
void printBiozScanCalibrationSettings();
void printPreferencesProfile();
bool savePreferencesProfile();
bool loadPreferencesProfile(bool report);
bool clearPreferencesProfile();
void applyHumanSafeScanPreset();
uint32_t biozScanCalibrationKppm();
uint8_t biozScanReliabilityScore(float frequency_hz, float corrected_magnitude_ohm, float phase_deg);
void configureEcgNotchFilter();
float applyEcgNotchFilter(float sample);
const char* ecgNotchLabel();

// ESP
//////////////////////////////////////////////////////////////////////////////////////////////

void printWakeupReason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : LOGln("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : LOGln("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : LOGln("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : LOGln("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : LOGln("Wakeup caused by ULP program"); break;
    default : LOGln("Wakeup was not caused by deep sleep: %d", static_cast<int>(wakeup_reason)); break;
  }
}

// AFE
//////////////////////////////////////////////////////////////////////////////////////////////

struct ModeRuntimePolicy {
  bool expect_ecg;
  bool expect_bioz;
  bool bioz_raw;
  bool expect_scan;
  bool warmup_update_before_settle;
  bool synch_after_post_reset;
  uint8_t ecg_fifo_threshold;
  uint8_t bioz_fifo_threshold;
  const char* ecg_label;
  const char* bioz_label;
};

ModeRuntimePolicy getModeRuntimePolicy(OperationMode mode) {
  switch (mode) {
    case MODE_ECG:
      return {true, false, false, false, false, false,
              ECG_FIFO_INTERRUPT_THRESHOLD, BIOZ_FIFO_INTERRUPT_THRESHOLD,
              "ECG [mV]: ", nullptr};
    case MODE_BIOZ:
      return {false, true, false, false, false, false,
              ECG_FIFO_INTERRUPT_THRESHOLD, BIOZ_FIFO_INTERRUPT_THRESHOLD,
              nullptr, "BIOZ [Ohm]: "};
    case MODE_ECG_BIOZ:
      return {true, true, false, false, false, false,
              ECG_FIFO_INTERRUPT_THRESHOLD, BIOZ_FIFO_INTERRUPT_THRESHOLD,
              "ECG [mV]: ", "BIOZ [Ohm]: "};
    case MODE_ECG_CAL:
      return {true, false, false, false, false, false,
              ECG_CAL_FIFO_INTERRUPT_THRESHOLD, BIOZ_FIFO_INTERRUPT_THRESHOLD,
              "ECG_Cal_[mV]: ", nullptr};
    case MODE_BIOZ_CAL:
      return {false, true, true, false, false, false,
              ECG_FIFO_INTERRUPT_THRESHOLD, BIOZ_FIFO_INTERRUPT_THRESHOLD,
              nullptr, "BIOZ_Cal_[raw]: "};
    case MODE_BIOZ_INTERNAL_CAL:
      return {false, true, false, false, false, true,
              ECG_FIFO_INTERRUPT_THRESHOLD, BIOZ_IMPEDANCE_FIFO_THRESHOLD,
              nullptr, "BIOZ [Ohm]: "};
    case MODE_BIOZ_EXTERNAL_CAL:
      return {false, true, false, false, false, true,
              ECG_FIFO_INTERRUPT_THRESHOLD, BIOZ_IMPEDANCE_FIFO_THRESHOLD,
              nullptr, "BIOZ [Ohm]: "};
    case MODE_BIOZ_SCAN:
      return {false, false, false, true, true, false,
              NO_FIFO_THRESHOLD_OVERRIDE, NO_FIFO_THRESHOLD_OVERRIDE,
              nullptr, nullptr};
    case MODE_IDLE:
    default:
      return {false, false, false, false, false, false,
              NO_FIFO_THRESHOLD_OVERRIDE, NO_FIFO_THRESHOLD_OVERRIDE,
              nullptr, nullptr};
  }
}

void applyModeFifoThresholds(const ModeRuntimePolicy& policy) {
  if (policy.ecg_fifo_threshold == NO_FIFO_THRESHOLD_OVERRIDE ||
      policy.bioz_fifo_threshold == NO_FIFO_THRESHOLD_OVERRIDE) {
    active_ecg_fifo_threshold = ECG_FIFO_INTERRUPT_THRESHOLD;
    active_bioz_fifo_threshold = BIOZ_FIFO_INTERRUPT_THRESHOLD;
    return;
  }

  const auto adaptiveEcgFifoThreshold = [](uint8_t speed_selector) -> uint8_t {
    switch (speed_selector) {
      case 0U: return 32U; // ~128 sps, about 250 ms FIFO window
      case 1U: return 16U; // ~256 sps, about 62 ms FIFO window
      case 2U:
      default: return 8U;  // ~512 sps, about 16 ms FIFO window
    }
  };

  active_ecg_fifo_threshold = policy.expect_ecg ? adaptiveEcgFifoThreshold(ecg_speed) : policy.ecg_fifo_threshold;
  active_bioz_fifo_threshold = policy.bioz_fifo_threshold;
  afe.setFIFOInterruptThreshold(active_ecg_fifo_threshold, active_bioz_fifo_threshold);
}

uint32_t computeDataTimeoutMs(float sampling_rate_sps, uint8_t fifo_threshold, uint8_t multiplier, uint32_t minimum_ms) {
  if (sampling_rate_sps <= 0.0f || fifo_threshold == 0U) {
    return minimum_ms;
  }

  const float threshold_period_ms = (1000.0f * static_cast<float>(fifo_threshold)) / sampling_rate_sps;
  const uint32_t computed_ms = static_cast<uint32_t>(threshold_period_ms * static_cast<float>(multiplier));
  return (computed_ms < minimum_ms) ? minimum_ms : computed_ms;
}

void reportErrorStatus(const char* context) {
  afe.readStatusRegisters();
  const uint32_t status_word = status.all & 0x00FFFFFFUL;
  LOGE("%s STATUS=0x%06lX | ECG FIFO=%s overflow=%s | BIOZ FIFO=%s overflow=%s | RTOR=%s | PLL=%s",
       context,
       static_cast<unsigned long>(status_word),
       status.bit.eint ? "ready" : "idle",
       status.bit.eovf ? "yes" : "no",
       status.bit.bint ? "ready" : "idle",
       status.bit.bovf ? "yes" : "no",
       status.bit.rrint ? "ready" : "idle",
       status.bit.pllint ? "unlocked" : "locked");
}

const char* ecgNotchLabel() {
  switch (ecg_notch) {
    case 50U: return "50 Hz";
    case 60U: return "60 Hz";
    default:  return "OFF";
  }
}

void configureEcgNotchFilter() {
  ecg_notch_filter.enabled = false;
  ecg_notch_filter.center_hz = 0.0f;
  ecg_notch_filter.sample_rate_hz = ECG_samplingRate;
  ecg_notch_filter.b0 = 1.0f;
  ecg_notch_filter.b1 = 0.0f;
  ecg_notch_filter.b2 = 0.0f;
  ecg_notch_filter.a1 = 0.0f;
  ecg_notch_filter.a2 = 0.0f;
  // reset Ecg Notch Filter State
  ecg_notch_filter.z1 = 0.0f;
  ecg_notch_filter.z2 = 0.0f;


  if ((ecg_notch != 50U && ecg_notch != 60U) || ECG_samplingRate <= 0.0f) {
    return;
  }

  const float f0 = static_cast<float>(ecg_notch);
  const float nyquist = 0.5f * ECG_samplingRate;
  if (f0 >= nyquist) {
    LOGW("ECG notch %.1f Hz disabled because ECG sampling rate %.2f sps is too low.", f0, ECG_samplingRate);
    return;
  }

  const float q = (ecg_notch_q > 0.0f) ? ecg_notch_q : ECG_NOTCH_Q_DEFAULT;
  const float w0 = 2.0f * PI * f0 / ECG_samplingRate;
  const float alpha = sinf(w0) / (2.0f * q);
  const float cos_w0 = cosf(w0);
  const float a0 = 1.0f + alpha;

  ecg_notch_filter.b0 = 1.0f / a0;
  ecg_notch_filter.b1 = (-2.0f * cos_w0) / a0;
  ecg_notch_filter.b2 = 1.0f / a0;
  ecg_notch_filter.a1 = (-2.0f * cos_w0) / a0;
  ecg_notch_filter.a2 = (1.0f - alpha) / a0;
  ecg_notch_filter.center_hz = f0;
  ecg_notch_filter.enabled = true;
}

float applyEcgNotchFilter(float sample) {
  if (!ecg_notch_filter.enabled) {
    return sample;
  }

  const float y = (ecg_notch_filter.b0 * sample) + ecg_notch_filter.z1;
  ecg_notch_filter.z1 = (ecg_notch_filter.b1 * sample) - (ecg_notch_filter.a1 * y) + ecg_notch_filter.z2;
  ecg_notch_filter.z2 = (ecg_notch_filter.b2 * sample) - (ecg_notch_filter.a2 * y);
  return y;
}

uint8_t biozScanReliabilityScore(float frequency_hz, float corrected_magnitude_ohm, float phase_deg) {
  if (scan_internal) {
    return 0U;
  }
  if (corrected_magnitude_ohm <= 0.0f || frequency_hz <= 0.0f) {
    return 0U;
  }

  // Provisional external-scan validity score from fixture analysis.
  // 3=validated region,
  // 2=near boundary,
  // 1=questionable,
  // 0=outside validated region.
  //
  // Base table:
  //           Frequency (Hz)
  //  Mag      1024 2048 4096 8192 >=18204
  //       <20 1    1    1    1    1
  //   20- 10k 3    3    3    3    3
  //  10k- 33k 3    3    3    3    1
  //  33k- 50k 3    3    3    1    1
  //  50k-100k 2    2    1    1    1
  // 100k-150k 1    0    0    0    0

  uint8_t score = 0U;
  if (corrected_magnitude_ohm < 20.0f) {
    score = 1U;
  } else if (corrected_magnitude_ohm <= 10000.0f) {
    score = 3U;
  } else if (corrected_magnitude_ohm <= 33000.0f) {
    score = (frequency_hz <= 8192.0f) ? 3U : 1U;
  } else if (corrected_magnitude_ohm <= 50000.0f) {
    score = (frequency_hz <= 4096.0f) ? 3U : 1U;
  } else if (corrected_magnitude_ohm <= 100000.0f) {
    score = (frequency_hz <= 2048.0f) ? 2U : 1U;
  } else if (corrected_magnitude_ohm <= 150000.0f) {
    score = (frequency_hz <= 1024.0f) ? 1U : 0U;
  } else {
    score = 0U;
  }

  if (score == 0U) {
    return score;
  }

  const float abs_phase = fabsf(phase_deg);

  // Verification run 20260717_133127 showed that high-frequency resistor stress
  // cases can collapse into a lower measured magnitude bin and look valid if
  // only magnitude/frequency are scored. Downgrade only moderate/high measured
  // impedances; low-ohm RC loads and capacitive loads can legitimately have
  // large phase and should not be rejected by this rule.
  if (corrected_magnitude_ohm >= 5000.0f) {
    if (frequency_hz >= 81920.0f && abs_phase >= 25.0f) {
      if (score > 1U) {
        score = 1U;
      }
    } else if (frequency_hz >= 40960.0f && abs_phase >= 20.0f) {
      if (score > 2U) {
        score = 2U;
      }
    }
  }

  return score;
}

void startMeasurement() {
  applySettings();
  LOGI("Measurement starting; runtime log level limited to ERROR during acquisition");

  // enter Measurement LogMode
  if (!measurement_log_level_saved) {
    saved_measurement_log_level = currentLogLevel;
    measurement_log_level_saved = true;
  }
  currentLogLevel = LOG_LEVEL_ERROR;

  afe.start();
  measurement_running = true;
  sample_count = 0;
  last_sample_count = 0;
  last_time = millis();

  const ModeRuntimePolicy policy = getModeRuntimePolicy(current_mode);
  if (policy.warmup_update_before_settle) {
    afe.update(current_mode == MODE_BIOZ_CAL);
  }

  // PLL startup and warm-up
  delay(PLL_STARTUP_SETTLE_MS);
  afe.readStatusRegisters(); // Clear startup PLLINT if it latched before PLL settled.
  delay(PLL_STATUS_CLEAR_MS);
  afe.FIFOReset(); // Discard samples collected during PLL warm-up.
  if (policy.synch_after_post_reset) {
    afe.synch();
  }

  // clear DataBuffers
  ECG_data.clear();
  BIOZ_data.clear();
  RTOR_data.clear();
  BIOZ_spectrum.clear();
  pending_rtor_count = 0U;

  // reset Runtime Monitoring State
  const uint32_t now = millis();
  last_ecg_data_ms = now;
  last_bioz_data_ms = now;
  mode_start_ms = now;
  ecg_timeout_reported = false;
  bioz_timeout_reported = false;
  scan_timeout_reported = false;

  // reset Ecg Notch Filter State
  ecg_notch_filter.z1 = 0.0f;
  ecg_notch_filter.z2 = 0.0f;

  // report Pll Status "Startup active"
  afe.readStatusRegisters();
  const uint32_t status_word = status.all & 0x00FFFFFFUL;
  const bool pll_unlocked = ((status_word & MAX30001_STATUS_PLLINT) != 0U);
  if (pll_unlocked) {
    LOGW("%s PLL: UNLOCKED (STATUS=0x%06lX)", "Startup active", static_cast<unsigned long>(status_word));
    LOGW("PLL remains unlocked after startup; check the MAX30001G external clock/oscillator.");
  } else {
    LOGI("%s PLL: OK (STATUS=0x%06lX)", "Startup active", static_cast<unsigned long>(status_word));
  }
  queueMeasurementState();
}

void stopMeasurement() {
  afe.stop();
  measurement_running = false;
  serialOutFlush();

  // restore Measurement LogMode
  if (measurement_log_level_saved) {
    currentLogLevel = saved_measurement_log_level;
    measurement_log_level_saved = false;
  }

  ecg_timeout_reported = false;
  bioz_timeout_reported = false;
  scan_timeout_reported = false;
  LOGI("Measurement stopped; runtime log level restored");
  queueMeasurementState();
}

void queueMeasurementState() {
  serialBufferWrite("Measurement state: ");
  serialBufferWrite(measurement_running ? "RUNNING" : "STOPPED");
  serialBufferWriteLine();
}

void requestStartMeasurement() {
  if (current_mode == MODE_IDLE) {
    LOGln("Cannot start - set a mode first (m1-m8)");
    return;
  }
  if (measurement_running) {
    LOGln("Already running");
    return;
  }
  startMeasurement();
}

void toggleMeasurement() {
  if (measurement_running) {
    stopMeasurement();
    return;
  }

  requestStartMeasurement();
}

void printDataOutputSettings() {
  LOG("Continuous data display: ");
  LOG("%s", data_reporting ? "ON" : "OFF");
  LOG(" | USB: ");
  LOG("%s", (data_output_mask & DATA_OUT_SERIAL) ? "ON" : "OFF");
  LOG(" | BLE: ");
  LOGln("%s", (data_output_mask & DATA_OUT_BLE) ? "ON" : "OFF");
}

void checkDataTimeouts() {
  if (!measurement_running) {
    return;
  }

  const ModeRuntimePolicy policy = getModeRuntimePolicy(current_mode);
  const uint32_t now = millis();

  if (policy.expect_scan && !scan_timeout_reported && ((now - mode_start_ms) >= SCAN_TIMEOUT_MS)) {
    LOGE("BIOZ scan timed out before a spectrum was available.");
    reportErrorStatus("BIOZ scan timeout");
    scan_timeout_reported = true;
    stopMeasurement();
    return;
  }

  const uint32_t bioz_timeout_ms = computeDataTimeoutMs(BIOZ_samplingRate, active_bioz_fifo_threshold, 4U, BIOZ_DATA_TIMEOUT_MIN_MS);
  uint32_t ecg_timeout_ms = computeDataTimeoutMs(ECG_samplingRate, active_ecg_fifo_threshold, 3U, ECG_DATA_TIMEOUT_MIN_MS);
  if (policy.expect_ecg && policy.expect_bioz) {
    ecg_timeout_ms = (bioz_timeout_ms > ecg_timeout_ms) ? bioz_timeout_ms : ecg_timeout_ms;
  }

  const bool ecg_timed_out = policy.expect_ecg && !ecg_timeout_reported &&
                             ((now - last_ecg_data_ms) >= ecg_timeout_ms);
  const bool bioz_timed_out = policy.expect_bioz && !bioz_timeout_reported &&
                              ((now - last_bioz_data_ms) >= bioz_timeout_ms);

  if (!ecg_timed_out && !bioz_timed_out) {
    return;
  }

  if (ecg_timed_out) {
    LOGW("No ECG data after %lu ms; attempting direct FIFO read.",
         static_cast<unsigned long>(ecg_timeout_ms));
    afe.readECG_FIFO(false);
  }

  if (bioz_timed_out) {
    LOGW("No %s data after %lu ms; attempting direct FIFO read.",
         policy.bioz_raw ? "raw BIOZ" : "BIOZ",
         static_cast<unsigned long>(bioz_timeout_ms));
    afe.readBIOZ_FIFO(policy.bioz_raw);
  }

  dataToSerialBuff();

  bool report_failure = false;
  if (ecg_timed_out && ((millis() - last_ecg_data_ms) >= ecg_timeout_ms)) {
    LOGE("No ECG data available after interrupt and direct FIFO reads.");
    ecg_timeout_reported = true;
    report_failure = true;
  }

  if (bioz_timed_out && ((millis() - last_bioz_data_ms) >= bioz_timeout_ms)) {
    LOGE("No %s data available after interrupt and direct FIFO reads.",
         policy.bioz_raw ? "raw BIOZ" : "BIOZ");
    bioz_timeout_reported = true;
    report_failure = true;
  }

  if (report_failure) {
    reportErrorStatus("Acquisition failure");
  }
}

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

// Print Mode Name
const char* getModeName(OperationMode mode) {
  switch (mode) {
    case MODE_IDLE:               return "IDLE";
    case MODE_ECG:                return "ECG";
    case MODE_BIOZ:               return "BIOZ";
    case MODE_ECG_BIOZ:           return "ECG+BIOZ";
    case MODE_ECG_CAL:            return "ECG Calibration";
    case MODE_BIOZ_CAL:           return "BIOZ Calibration";
    case MODE_BIOZ_INTERNAL_CAL:  return "BIOZ Internal Cal";
    case MODE_BIOZ_EXTERNAL_CAL:  return "BIOZ External Cal";
    case MODE_BIOZ_SCAN:          return "BIOZ Scan";
    default:                      return "UNKNOWN";
  }
}

const char* getDisplayModeName(OperationMode mode) {
  switch (mode) {
    case MODE_ECG:                return "ECG";
    case MODE_BIOZ:               return "BIOZ";
    case MODE_ECG_BIOZ:           return "ECG&BIOZ";
    case MODE_BIOZ_SCAN:          return "BIOZ Spectroscopy";
    case MODE_ECG_CAL:            return "ECG Calibration";
    case MODE_BIOZ_CAL:           return "BIOZ Calibration";
    case MODE_BIOZ_INTERNAL_CAL:  return "BIOZ Internal Cal";
    case MODE_BIOZ_EXTERNAL_CAL:  return "BIOZ External Cal";
    case MODE_IDLE:               return "Idle";
    default:                      return "Unknown";
  }
}

const char* onOffLabel(bool enabled) {
  return enabled ? "ON" : "OFF";
}

const char* calSignalModeLabel(bool unipolar) {
  return unipolar ? "unipolar" : "bipolar";
}

const char* scanPhaseRangeLabel(BIOZScanPhaseRange range) {
  return (range == BIOZ_SCAN_PHASE_REDUCED) ? "reduced(0/45/90/135 deg)" : "full";
}

const char* scanAhpfModeLabel(BIOZScanAHPFMode mode) {
  switch (mode) {
    case BIOZ_SCAN_AHPF_DYNAMIC: return "dynamic";
    case BIOZ_SCAN_AHPF_FIXED: return "fixed";
    case BIOZ_SCAN_AHPF_BYPASS: return "bypass";
    default: return "dynamic";
  }
}

float requestedEcgSamplingRateSps(uint8_t selector) {
  switch (selector) {
    case 0: return 125.0f;
    case 1: return 256.0f;
    case 2: return 512.0f;
    default: return 0.0f;
  }
}

float requestedEcgLpfHz(uint8_t selector) {
  switch (selector) {
    case 1: return 40.0f;
    case 2: return 100.0f;
    case 3: return 150.0f;
    default: return 0.0f;
  }
}

float requestedEcgHpfHz(uint8_t selector) {
  return (selector == 1U) ? 0.5f : 0.0f;
}

uint8_t defaultEcgLpfSelector(OperationMode mode, uint8_t speed) {
  (void)mode;
  switch (speed) {
    case 0U: return 1U;
    case 1U: return 2U;
    case 2U:
    default: return 3U;
  }
}

uint8_t defaultEcgHpfSelector(OperationMode mode) {
  (void)mode;
  return 0U;
}

uint8_t resolvedEcgLpfSelector(OperationMode mode) {
  return (ecg_lpf == ECG_FILTER_AUTO) ? defaultEcgLpfSelector(mode, ecg_speed) : ecg_lpf;
}

uint8_t resolvedEcgHpfSelector(OperationMode mode) {
  return (ecg_hpf == ECG_FILTER_AUTO) ? defaultEcgHpfSelector(mode) : ecg_hpf;
}

void printRequestedEcgFilterSummary(OperationMode mode) {
  LOG("  Requested HPF: ");
  if (ecg_hpf == ECG_FILTER_AUTO) {
    LOG("auto (");
    printHzOrBypass(requestedEcgHpfHz(defaultEcgHpfSelector(mode)), 2U);
    LOG(")");
  } else {
    printHzOrBypass(requestedEcgHpfHz(ecg_hpf), 2U);
  }

  LOG(" | Requested LPF: ");
  if (ecg_lpf == ECG_FILTER_AUTO) {
    LOG("auto (");
    printHzOrBypass(requestedEcgLpfHz(defaultEcgLpfSelector(mode, ecg_speed)), 0U);
    LOG(")");
  } else {
    printHzOrBypass(requestedEcgLpfHz(ecg_lpf), 0U);
  }
  LOG(" | Requested Notch: ");
  LOG("%s", ecgNotchLabel());
  LOG(" (Q=");
  LOG("%.1f", ecg_notch_q);
  LOG(")");
  LOGln("");
}

float requestedBiozSamplingRateSps(uint8_t selector) {
  return (selector == 0U) ? 32.0f : 64.0f;
}

float requestedBiozAhpfHz(uint8_t selector) {
  switch (selector) {
    case 0: return 60.0f;
    case 1: return 150.0f;
    case 2: return 500.0f;
    case 3: return 1000.0f;
    case 4: return 2000.0f;
    case 5: return 4000.0f;
    default: return 0.0f;
  }
}

float requestedBiozDlpfHz(uint8_t selector) {
  switch (selector) {
    case 1: return 4.0f;
    case 2: return 8.0f;
    case 3: return 16.0f;
    default: return 0.0f;
  }
}

float requestedBiozDhpfHz(uint8_t selector) {
  switch (selector) {
    case 1: return 0.05f;
    case 2: return 0.5f;
    default: return 0.0f;
  }
}

float requestedCalibrationModulationFrequencyHz(uint8_t selector) {
  switch (selector) {
    case 1: return 0.0625f;
    case 2: return 0.25f;
    case 3: return 1.0f;
    case 4: return 4.0f;
    default: return 4.0f;
  }
}

float requestedCalibrationModulationOhm(uint32_t resistance, uint8_t modulation) {
  if (modulation == 0U || resistance > 5000U) {
    return 0.0f;
  }

  const uint16_t rnom_thresholds[] = {2500U, 1667U, 1250U, 1000U, 833U, 714U, 625U, 0U};
  const float rmod_table[8][4] = {
    {0.0f, 247.5f, 980.6f, 2960.7f},
    {0.0f,  61.9f, 245.2f,  740.4f},
    {0.0f,  27.5f, 109.0f,  329.1f},
    {0.0f,   0.0f,  61.3f,  185.1f},
    {0.0f,   0.0f,  39.2f,  118.5f},
    {0.0f,   0.0f,  27.2f,   82.3f},
    {0.0f,   0.0f,  20.0f,   60.5f},
    {0.0f,   0.0f,  15.3f,   46.3f}
  };

  uint8_t row = 7U;
  for (uint8_t i = 0U; i < 8U; ++i) {
    if (resistance > rnom_thresholds[i]) {
      row = i;
      break;
    }
  }

  if (modulation > 3U) {
    modulation = 3U;
  }
  return rmod_table[row][modulation];
}

const char* scanAhpfOverrideLabel(uint8_t selector) {
  switch (selector) {
    case 255U: return "dynamic";
    case 0U: return "60 Hz";
    case 1U: return "150 Hz";
    case 2U: return "500 Hz";
    case 3U: return "1 kHz";
    case 4U: return "2 kHz";
    case 5U: return "4 kHz";
    case 6U:
    case 7U: return "bypass";
    default: return "unknown";
  }
}

// AFE Status Reporting
//////////////////////////////////////////////////////////////////////////////////////////////

void printHzOrBypass(float hz, uint8_t decimals) {
  if (hz <= 0.0f) {
    LOG("bypass");
  } else {
    LOG("%.*f", decimals, hz);
    LOG(" Hz");
  }
}

void printAppliedBiozSummary() {
  LOG("  Applied BIOZ: ");
  LOG("%.1f", BIOZ_samplingRate);
  LOG(" sps, ");
  LOG("%d", BIOZ_gain);
  LOG(" V/V, AHPF ");
  printHzOrBypass(BIOZ_ahpf, 0U);
  LOG(", DLPF ");
  printHzOrBypass(BIOZ_dlpf, 2U);
  LOG(", DHPF ");
  printHzOrBypass(BIOZ_dhpf, 2U);
  LOG(", ");
  LOG("%.1f", BIOZ_frequency);
  LOG(" Hz, ");
  LOG("%d", BIOZ_cgmag);
  LOG(" nA, ");
  LOG("%.2f", BIOZ_phase);
  LOGln(" deg");
}

void printAppliedBiozSignalCalibrationSummary() {
  LOG("  Applied BIOZ Cal: ");
  LOG("%.1f", BIOZ_samplingRate);
  LOG(" sps, ");
  LOG("%d", BIOZ_gain);
  LOG(" V/V, VCAL ");
  LOG("%s", calSignalModeLabel(bioz_cal_unipolar));
  LOG(", ");
  LOG("%s", BIOZ_CAL_0P5_MV ? "0.5 mV" : "0.25 mV");
  LOG(", ");
  LOG("%.4f", CAL_fcal);
  LOG(" Hz, ");
  LOG("%s", BIOZ_CAL_DUTY_PERCENT == 50 ? "50% duty" : "custom duty");
  LOG(", AHPF ");
  printHzOrBypass(BIOZ_ahpf, 0U);
  LOG(", DLPF ");
  printHzOrBypass(BIOZ_dlpf, 2U);
  LOG(", DHPF ");
  printHzOrBypass(BIOZ_dhpf, 2U);
  LOGln(", current generator OFF");
}

void printAppliedEcgSignalCalibrationSummary() {
  LOG("  Applied ECG Cal: ");
  LOG("%.1f", ECG_samplingRate);
  LOG(" sps, ");
  LOG("%d", ECG_gain);
  LOG(" V/V, VCAL ");
  LOG("%s", calSignalModeLabel(ecg_cal_unipolar));
  LOG(", 0.5 mV, ");
  LOG("%.4f", CAL_fcal);
  LOG(" Hz, 50%% duty, DHPF ");
  printHzOrBypass(ECG_hpf, 2U);
  LOG(", DLPF ");
  printHzOrBypass(ECG_lpf, 0U);
  LOGln("");
}

void printAppliedEcgSummary() {
  LOG("  Applied ECG: ");
  LOG("%.1f", ECG_samplingRate);
  LOG(" sps, ");
  LOG("%d", ECG_gain);
  LOG(" V/V, HPF ");
  printHzOrBypass(ECG_hpf, 2U);
  LOG(", LPF ");
  printHzOrBypass(ECG_lpf, 0U);
  LOG(", Notch ");
  LOG("%s", ecgNotchLabel());
  LOG(" (Q=");
  LOG("%.1f", ecg_notch_q);
  LOG(")");
  LOGln("");
}

void printBiozScanCalibrationSettings() {
  LOG("  External Scan Correction: ");
  LOG("%s", bioz_scan_cal_enabled ? "ON" : "OFF");
  LOG(" | Model: global median magnitude scale | K=");
  LOG("%.5f", bioz_scan_cal_global_k);
  LOGln(" | Phase correction: OFF");
}

void applyHumanSafeScanPreset() {
  scan_human_safe = true;
  scan_internal = false;
  scan_phase_range = BIOZ_SCAN_PHASE_REDUCED;
  scan_ahpf_mode = BIOZ_SCAN_AHPF_FIXED;
  scan_fixed_ahpf = 1U;
  scan_fast = true;
  scan_fullrange = false;
  scan_avg = 8U;
  scan_settle_samples = 48U;
  scan_current_change_settle_samples = 64U;
  scan_max_current_nA = 8000;
  scan_autorange_enable = false;
  bioz_current = 8000;
  bioz_fourleads = true;
}

uint32_t biozScanCalibrationKppm() {
  if (bioz_scan_cal_global_k <= 0.0f) {
    return BIOZ_SCAN_CAL_DEFAULT_GLOBAL_K_PPM;
  }
  const float ppm = bioz_scan_cal_global_k * 1000000.0f;
  if (ppm < 500000.0f) {
    return 500000UL;
  }
  if (ppm > 2000000.0f) {
    return 2000000UL;
  }
  return static_cast<uint32_t>(ppm + 0.5f);
}

void printPreferencesProfile() {
  LOGln("  Preferences profile:");
  LOG("    Namespace: ");
  LOGln("%s", BIOZ_PREF_NAMESPACE);
  LOG("    Schema: ");
  LOGln("%u", BIOZ_PREF_SCHEMA_VERSION);
  LOG("    Calibration version: ");
  LOGln("%u", BIOZ_SCAN_CAL_VERSION);
  LOG("    Reliability map version: ");
  LOGln("%u", BIOZ_RELIABILITY_MAP_VERSION);
  LOG("    Correction enabled: ");
  LOGln("%s", bioz_scan_cal_enabled ? "YES" : "NO");
  LOG("    Global K ppm: ");
  LOG("%lu", static_cast<unsigned long>(biozScanCalibrationKppm()));
  LOG(" (");
  LOG("%.6f", bioz_scan_cal_global_k);
  LOGln(")");
  LOG("    Scan: avg=");
  LOG("%u", scan_avg);
  LOG(", fast=");
  LOG("%d", scan_fast ? 1 : 0);
  LOG(", full=");
  LOG("%d", scan_fullrange ? 1 : 0);
  LOG(", internal=");
  LOG("%d", scan_internal ? 1 : 0);
  LOG(", phase=");
  LOG("%d", (scan_phase_range == BIOZ_SCAN_PHASE_REDUCED) ? 1 : 0);
  LOG(", ahpf-mode=");
  LOG("%s", scanAhpfModeLabel(scan_ahpf_mode));
  LOG(", fixed-ahpf=");
  LOG("%s", scanAhpfOverrideLabel(scan_fixed_ahpf));
  LOG(", int-ahpf=");
  LOG("%u", scan_internal_bist_ahpf);
  LOG(", settle=");
  LOG("%u", scan_settle_samples);
  LOG(", current-settle=");
  LOG("%u", scan_current_change_settle_samples);
  LOG(", max-current=");
  LOG("%ld", static_cast<long>(scan_max_current_nA));
  LOG(", autorange=");
  LOG("%d", scan_autorange_enable ? 1 : 0);
  LOG(", human-safe=");
  LOGln("%d", scan_human_safe ? 1 : 0);
  LOG("    BIOZ default: current=");
  LOG("%lu", static_cast<unsigned long>(bioz_current));
  LOG(" nA, wires=");
  LOGln("%s", bioz_fourleads ? "4" : "2");
}

// AFE Preferences Profile Save/Load
//////////////////////////////////////////////////////////////////////////////////////////////

bool savePreferencesProfile() {
  Preferences prefs;
  if (!prefs.begin(BIOZ_PREF_NAMESPACE, false)) {
    LOGW("Preferences save failed: namespace open failed");
    return false;
  }

  prefs.putUShort("schema", BIOZ_PREF_SCHEMA_VERSION);
  prefs.putUShort("cal_ver", BIOZ_SCAN_CAL_VERSION);
  prefs.putUShort("rel_ver", BIOZ_RELIABILITY_MAP_VERSION);
  prefs.putBool("cal_en", bioz_scan_cal_enabled);
  prefs.putUInt("cal_k_ppm", biozScanCalibrationKppm());
  prefs.putUChar("scan_avg", scan_avg);
  prefs.putBool("scan_fast", scan_fast);
  prefs.putBool("scan_full", scan_fullrange);
  prefs.putBool("scan_int", scan_internal);
  prefs.putUChar("scan_phase", (scan_phase_range == BIOZ_SCAN_PHASE_REDUCED) ? 1U : 0U);
  prefs.putUChar("scan_ahpf_mode", static_cast<uint8_t>(scan_ahpf_mode));
  prefs.putUChar("scan_fixed_ahpf", scan_fixed_ahpf);
  prefs.putUChar("scan_ahpf", scan_internal_bist_ahpf);
  prefs.putUChar("scan_settle", scan_settle_samples);
  prefs.putUChar("scan_csettle", scan_current_change_settle_samples);
  prefs.putUInt("scan_max_cur", static_cast<uint32_t>(scan_max_current_nA));
  prefs.putBool("scan_auto", scan_autorange_enable);
  prefs.putBool("scan_hsafe", scan_human_safe);
  prefs.putUInt("bioz_current", bioz_current);
  prefs.putBool("bioz_4wire", bioz_fourleads);
  prefs.end();
  LOGln("Preferences profile saved");
  return true;
}

bool loadPreferencesProfile(bool report) {
  Preferences prefs;
  if (!prefs.begin(BIOZ_PREF_NAMESPACE, true)) {
    if (report) {
      LOGW("Preferences load failed: namespace open failed");
    }
    return false;
  }

  const uint16_t schema = prefs.getUShort("schema", 0U);
  if (schema != BIOZ_PREF_SCHEMA_VERSION) {
    prefs.end();
    if (report) {
      LOGW("No compatible preferences profile found");
    }
    return false;
  }

  const uint32_t k_ppm = prefs.getUInt("cal_k_ppm", BIOZ_SCAN_CAL_DEFAULT_GLOBAL_K_PPM);
  if (k_ppm >= 500000UL && k_ppm <= 2000000UL) {
    bioz_scan_cal_global_k = static_cast<float>(k_ppm) / 1000000.0f;
  }
  bioz_scan_cal_enabled = prefs.getBool("cal_en", false);

  scan_avg = prefs.getUChar("scan_avg", scan_avg);
  if (scan_avg < 1U) { scan_avg = 1U; }
  if (scan_avg > 8U) { scan_avg = 8U; }

  scan_fast = prefs.getBool("scan_fast", scan_fast);
  scan_fullrange = prefs.getBool("scan_full", scan_fullrange);
  scan_internal = prefs.getBool("scan_int", scan_internal);
  scan_phase_range = (prefs.getUChar("scan_phase", (scan_phase_range == BIOZ_SCAN_PHASE_REDUCED) ? 1U : 0U) == 1U)
                       ? BIOZ_SCAN_PHASE_REDUCED
                       : BIOZ_SCAN_PHASE_FULL;

  const uint8_t ahpf_mode = prefs.getUChar("scan_ahpf_mode", static_cast<uint8_t>(scan_ahpf_mode));
  scan_ahpf_mode = (ahpf_mode <= static_cast<uint8_t>(BIOZ_SCAN_AHPF_BYPASS))
                     ? static_cast<BIOZScanAHPFMode>(ahpf_mode)
                     : BIOZ_SCAN_AHPF_DYNAMIC;
  scan_fixed_ahpf = prefs.getUChar("scan_fixed_ahpf", scan_fixed_ahpf);
  if (scan_fixed_ahpf > 7U) {
    scan_fixed_ahpf = 1U;
  }

  scan_internal_bist_ahpf = prefs.getUChar("scan_ahpf", scan_internal_bist_ahpf);
  if ((scan_internal_bist_ahpf > 7U) && (scan_internal_bist_ahpf != 255U)) {
    scan_internal_bist_ahpf = 255U;
  }

  scan_settle_samples = prefs.getUChar("scan_settle", scan_settle_samples);
  if (scan_settle_samples < 1U) { scan_settle_samples = 1U; }
  if (scan_settle_samples > 64U) { scan_settle_samples = 64U; }

  scan_current_change_settle_samples = prefs.getUChar("scan_csettle", scan_current_change_settle_samples);
  if (scan_current_change_settle_samples < scan_settle_samples) {
    scan_current_change_settle_samples = scan_settle_samples;
  }
  if (scan_current_change_settle_samples > 64U) {
    scan_current_change_settle_samples = 64U;
  }

  scan_max_current_nA = static_cast<int32_t>(prefs.getUInt("scan_max_cur", static_cast<uint32_t>(scan_max_current_nA)));
  if (scan_max_current_nA < 55) { scan_max_current_nA = 55; }
  if (scan_max_current_nA > 96000) { scan_max_current_nA = 96000; }
  scan_autorange_enable = prefs.getBool("scan_auto", scan_autorange_enable);
  scan_human_safe = prefs.getBool("scan_hsafe", scan_human_safe);

  bioz_current = prefs.getUInt("bioz_current", bioz_current);
  if (bioz_current > 96000UL) {
    bioz_current = 96000UL;
  }
  bioz_fourleads = prefs.getBool("bioz_4wire", bioz_fourleads);
  if (scan_human_safe) {
    applyHumanSafeScanPreset();
  } else if (bioz_current > static_cast<uint32_t>(scan_max_current_nA)) {
    bioz_current = static_cast<uint32_t>(scan_max_current_nA);
  }

  prefs.end();
  if (report) {
    LOGln("Preferences profile loaded");
    printPreferencesProfile();
  }
  return true;
}

bool clearPreferencesProfile() {
  Preferences prefs;
  if (!prefs.begin(BIOZ_PREF_NAMESPACE, false)) {
    LOGW("Preferences clear failed: namespace open failed");
    return false;
  }
  const bool ok = prefs.clear();
  prefs.end();
  LOGln("%s", ok ? "Preferences profile cleared" : "Preferences clear failed");
  return ok;
}

// Apply current settings to AFE (setup based on mode)
void applySettings() {
  const ModeRuntimePolicy policy = getModeRuntimePolicy(current_mode);

  if (measurement_running) {
    LOGln("Stopping measurement before applying new settings...");
    stopMeasurement();
    delay(100);
  }

  LOGI("Applying settings for mode: %s", getModeName(current_mode));

  switch (current_mode) {
    case MODE_ECG:
      afe.setupECG(ecg_speed, ecg_gain, ecg_threeleads);
      afe.setECGfilter(resolvedEcgLpfSelector(current_mode), resolvedEcgHpfSelector(current_mode));
      configureEcgNotchFilter();
      applyModeFifoThresholds(policy);
      if (!ecg_rtor) {
        afe.setDefaultNoRtoR();
        RTOR_data.clear();
      }
      LOGln("ECG mode configured");
      printAppliedEcgSummary();
      break;

    case MODE_BIOZ:
      afe.setupBIOZ(bioz_speed, bioz_gain, bioz_ahpf, bioz_dlpf, bioz_dhpf,
                    bioz_frequency, bioz_current, bioz_phase,
                    bioz_leadbias, bioz_leadsoffdetect, bioz_fourleads);
      applyModeFifoThresholds(policy);
      LOGln("BIOZ mode configured");
      printAppliedBiozSummary();
      break;

    case MODE_ECG_BIOZ:
      afe.setupECGandBIOZ(ecg_speed, ecg_gain, ecg_threeleads,
                          bioz_speed, bioz_gain, bioz_dlpf, bioz_dhpf,
                          bioz_frequency, bioz_current, bioz_phase,
                          bioz_leadbias, bioz_leadsoffdetect, bioz_fourleads);
      afe.setECGfilter(resolvedEcgLpfSelector(current_mode), resolvedEcgHpfSelector(current_mode));
      configureEcgNotchFilter();
      applyModeFifoThresholds(policy);
      if (!ecg_rtor) {
        afe.setDefaultNoRtoR();
        RTOR_data.clear();
      }
      LOGln("ECG+BIOZ mode configured");
      printAppliedEcgSummary();
      printAppliedBiozSummary();
      break;

    case MODE_ECG_CAL:
      afe.setupECGSignalCalibration(ecg_speed, ecg_gain);
      afe.setECGfilter(resolvedEcgLpfSelector(current_mode), resolvedEcgHpfSelector(current_mode));
      configureEcgNotchFilter();
      afe.setTestSignal(true, false, ecg_cal_unipolar, true, 0b100, 50);
      applyModeFifoThresholds(policy);
      LOGln("ECG calibration mode configured");
      printAppliedEcgSignalCalibrationSummary();
      break;

    case MODE_BIOZ_CAL:
      afe.setupBIOZSignalCalibration(bioz_speed, bioz_gain);
      applyModeFifoThresholds(policy);
      afe.setTestSignal(BIOZ_CAL_ROUTE_ECG, BIOZ_CAL_ROUTE_BIOZ, bioz_cal_unipolar,
                        BIOZ_CAL_0P5_MV, BIOZ_CAL_FCAL_1HZ, BIOZ_CAL_DUTY_PERCENT);
      afe.setBIOZfilter(BIOZ_CAL_AHPF_BYPASS, BIOZ_CAL_DLPF_BYPASS, BIOZ_CAL_DHPF_BYPASS);
      LOG("BIOZ calibration mode configured: raw ");
      LOG("%s", calSignalModeLabel(bioz_cal_unipolar));
      LOGln(" 0.5mV VCAL at ~1Hz, filters bypassed");
      printAppliedBiozSignalCalibrationSummary();
      break;

    case MODE_BIOZ_INTERNAL_CAL:
      afe.setupBIOZImpedanceCalibration(bioz_speed, bioz_gain, bioz_ahpf, bioz_dlpf, bioz_dhpf,
                                         bioz_frequency, bioz_current, bioz_phase,
                                         cal_resistance, cal_modulation, cal_mod_freq);
      applyModeFifoThresholds(policy);
      LOGln("BIOZ internal calibration mode configured");
      printAppliedBiozSummary();
      LOG("  Applied BIST: RNOM ");
      LOG("%.1f", BIOZ_test_rnom);
      LOG(" Ohm, RMOD ");
      LOG("%.1f", BIOZ_test_rmod);
      LOG(" Ohm, RMOD freq ");
      LOG("%.4f", BIOZ_test_frequency);
      LOGln(" Hz");
      break;

    case MODE_BIOZ_EXTERNAL_CAL:
      afe.setupBIOZExternalImpedanceCalibration(bioz_frequency, bioz_phase);
      applyModeFifoThresholds(policy);
      LOGln("BIOZ external calibration mode configured");
      printAppliedBiozSummary();
      break;

    case MODE_BIOZ_SCAN:
      {
        BIOZScanConfig config;
        config.avg = scan_avg;
        config.fast = scan_fast;
        config.fourleads = bioz_fourleads;
        config.freq_end_index = scan_fullrange ? (MAX30001_BIOZ_NUM_FREQUENCIES - 1U) : 7U;
        config.phase_range = scan_phase_range;
        config.ahpf_mode = scan_ahpf_mode;
        config.fixed_ahpf = scan_fixed_ahpf;
        config.use_internal_resistor = scan_internal;
        config.internal_resistor_ohm = static_cast<uint16_t>(cal_resistance);
        config.internal_bist_ahpf = scan_internal_bist_ahpf;
        config.initial_current_nA = bioz_current;
        config.max_current_nA = scan_max_current_nA;
        config.autorange_enable = scan_autorange_enable;
        config.settle_samples = scan_settle_samples;
        config.current_change_settle_samples = scan_current_change_settle_samples;
        afe.setupBIOZScan(config);
        LOGI("BIOZ scan mode configured (%s)", scan_internal ? "internal resistor" : "external electrodes");
        LOGI("Requested scan start current: %ld nA | max current: %ld nA | autorange: %s | phase range: %s | AHPF mode: %s | fixed AHPF: %s | internal AHPF: %s | settle: %u samples | current-change settle: %u samples",
             static_cast<long>(config.initial_current_nA),
             static_cast<long>(config.max_current_nA),
             config.autorange_enable ? "ON" : "OFF",
             scanPhaseRangeLabel(config.phase_range),
             scanAhpfModeLabel(config.ahpf_mode),
             scanAhpfOverrideLabel(config.fixed_ahpf),
             scanAhpfOverrideLabel(config.internal_bist_ahpf),
             config.settle_samples,
             config.current_change_settle_samples);
      }
      break;

    case MODE_IDLE:
      afe.enableAFE(false, false, false);
      LOGln("IDLE mode - AFE disabled");
      break;

    default:
      LOGln("Unknown mode");
      break;
  }
}

// USER INTERFACE
//////////////////////////////////////////////////////////////////////////////////////////////

// Help Menu
void helpMenu() {
  LOGln("================================================================================");
  LOGln("| MAX30001G ECG and Bio-Impedance Program                                      |");
  LOGln("| 2026 Urs Utzinger & GPT                                                      |");
  LOGln("================================================================================");
  LOGln("| GENERAL COMMANDS                       | DATA COMMANDS                       |");
  LOGln("|----------------------------------------|-------------------------------------|");
  LOGln("| ?: help screen                         | z: toggle data reporting on/off     |");
  LOGln("| s: show current settings               | z[s|b|a]: USB, BLE, both, off       |");
  LOGln("| a: apply current settings (re-setup)   |                                     |");
  LOGln("| h: run health check                    | z[S|B|A]: USB, BLE, both, on        |");
  LOGln("| i: print device info                   | c: reset sample counter             |");
  LOGln("| r: print all registers                 | f: FIFO reset                       |");
  LOGln("| t: print status registers              |                                     |");
  LOGln("| p: print config registers              |                                     |");
  LOGln("| x: reboot                              |                                     |");
  LOGln("================================================================================");
  LOGln("| PERSISTENCE                            | RAM SNAPSHOT                        |");
  LOGln("|----------------------------------------|-------------------------------------|");
  LOGln("| Ps: save NVS preferences               | (: save volatile register snapshot  |");
  LOGln("| Pl: load NVS preferences               | ): restore volatile register snap   |");
  LOGln("| Pd: print NVS preferences              |                                     |");
  LOGln("| Pc: clear NVS preferences              |                                     |");
  LOGln("|========================================|=====================================|");
  LOGln("| OPERATION MODES (auto-stop previous)   | START/STOP                          |");
  LOGln("|----------------------------------------|-------------------------------------|");
  LOGln("| m1: ECG mode                           | .: toggle start/stop                |");
  LOGln("| m2: BIOZ mode                          | >: start measurement                |");
  LOGln("| m3: ECG + BIOZ mode                    | <: stop measurement                 |");
  LOGln("| m4: ECG signal calibration             |                                     |");
  LOGln("| m5: BIOZ signal calibration            |                                     |");
  LOGln("| m6: BIOZ internal impedance            |                                     |");
  LOGln("| m7: BIOZ external impedance            |                                     |");
  LOGln("| m8: BIOZ impedance spectroscopy        |                                     |");
  LOGln("|========================================|=====================================|");
  LOGln("| ECG SETTINGS                           | BIOZ SETTINGS                       |");
  LOGln("|----------------------------------------|-------------------------------------|");
  LOGln("| Es<n>: speed      (0-2)     Es1        | Bs<n>: speed      (0-1)     Bs0     |");
  LOGln("| Eg<n>: gain       (0-3)     Eg2        | Bg<n>: gain       (0-3)     Bg1     |");
  LOGln("| El<n>: dig LPF    (0-3,255) El255      | Ba<n>: analog HPF (0-7)     Ba1     |");
  LOGln("| Eh<n>: dig HPF    (0-1,255) Eh255      | Bd<n>: digital LPF(0-3)     Bd1     |");
  LOGln("| Ee<n>: leads      (2 or 3)  Ee3        | Bh<n>: digital HPF(0-3)     Bh0     |");
  LOGln("| Er<n>: R-to-R     (0=off,1) Er1        |                                     |");
  LOGln("| En<n>: notch  (0=off,50,60) En0        | Bf<n>: frequency Hz         Bf8000  |");
  LOGln("| Eq<n>: notch Q    (1-100)   Eq20       | Bc<n>: current nA           Bc8000  |");
  LOGln("|                                        | Bp<n>: phase deg            Bp0     |");
  LOGln("|                                        | Bl<n>: lead bias  (0=off,1) Bl1     |");
  LOGln("|                                        | Bo<n>: lead-off   (0=off,1) Bo0     |");
  LOGln("|                                        | Bw<n>: wires      (2 or 4)  Bw2     |");
  LOGln("|========================================|=====================================|");
  LOGln("| BIOZ SCAN SETTINGS                     | INTERNAL CALIBRATION SETTINGS       |");
  LOGln("|----------------------------------------|-------------------------------------|");
  LOGln("| Sa<n>: averages   (1-8)     Sa8        | Cr<n>: internal resistor    Cr1000  |");
  LOGln("| Sf<n>: fast mode  (0=off,1) Sf0        | Cm<n>: cal modulation(0-3)  Cm0     |");
  LOGln("| Sr<n>: full range (0=off,1) Sr0        | Cf<n>: mod frequency(0-4)   Cf3     |");
  LOGln("| Si<n>: source     (0=ext,1=int) Si0    | Ce<n>: ECG sig mode(0/1)    Ce1     |");
  LOGln("| Sp<n>: phase rng  (0=full,1) Sp0       | Cb<n>: BIOZ sig mode(0/1)   Cb0     |");
  LOGln("| Sh<n>: AHPF mode  (0=dyn,1=fix,2=byp)  |-------------------------------------|");
  LOGln("| Sv<n>: fixed AHPF (0-7)     Sv1        |                                     |");
  LOGln("| Sx<n>: int AHPF   (255,0-7) Sx255      |                                     |");
  LOGln("| St<n>: settle     (1-64)    St24       | Sm<n>: max current nA       Sm96000 |");
  LOGln("| Sc<n>: cur settle (1-64)    Sc24       | So<n>: autorange  (0=off,1) So1     |");
  LOGln("|                                        | Sy<n>: human-safe (0/1)     Sy0     |");
  LOGln("|----------------------------------------|-------------------------------------|");
  LOGln("| Kp:    print scan calibration          | Kr:  reset scan calibration default |");
  LOGln("| Kg<n>: set global K ppm (1250650)      | Ke<n>: enable correction (0/1)      |");
  LOGln("|========================================|=====================================|");
  LOGln("| LOG LEVEL                              | SPECIAL                             |");
  LOGln("|----------------------------------------|-------------------------------------|");
  LOGln("| l0: debug (verbose)                    | w: AFE software reset               |");
  LOGln("| l1: info (default)                     | y: AFE synchronize                  |");
  LOGln("| l2: warnings                           | k: AFE clear latched status flags   |");
  LOGln("| l3: errors only                        |                                     |");
  LOGln("| l4: critical only                      |                                     |");
  LOGln("| l5: none (silent)                      |                                     |");
  LOGln("================================================================================");
  LOGln("");
  LOGln("Examples:");
  LOGln("  m1       - Switch to ECG mode");
  LOGln("  Eg3      - Set ECG gain to 160 V/V (level 3)");
  LOGln("  Bf40000  - Set BIOZ frequency to 40 kHz");
  LOGln("  .        - Start/stop measurement");
  LOGln("  z        - Toggle continuous data display");
  LOGln("");
}

// Show Current Settings
void showSettings() {
  LOGln("================================================================================");
  LOGln("Current Settings:");
  LOGln("================================================================================");
  LOG("Mode: ");
  LOG("%s", getModeName(current_mode));
  LOG(" | Running: ");
  LOG("%s", measurement_running ? "YES" : "NO");
  LOG(" | Data Display: ");
  LOGln("%s", data_reporting ? "ON" : "OFF");
  LOG("Data Output: USB ");
  LOG("%s", (data_output_mask & DATA_OUT_SERIAL) ? "ON" : "OFF");
  LOG(" | BLE ");
  LOGln("%s", (data_output_mask & DATA_OUT_BLE) ? "ON" : "OFF");
  LOG("FIFO Thresholds: ECG ");
  LOG("%u", active_ecg_fifo_threshold);
  LOG(" samples | BIOZ ");
  LOG("%u", active_bioz_fifo_threshold);
  LOGln(" samples");
  LOGln("--------------------------------------------------------------------------------");
  LOGln("ECG Settings:");
  LOG("  Requested Speed: ~");
  LOG("%.0f", requestedEcgSamplingRateSps(ecg_speed));
  LOG(" sps (selector ");
  LOG("%u", ecg_speed);
  LOGln(")");
  LOG("  Requested Gain: ");
  LOG("%d", 20 << ecg_gain);
  LOG(" V/V (selector ");
  LOG("%u", ecg_gain);
  LOGln(")");
  printRequestedEcgFilterSummary(current_mode);
  LOG("  Leads: ");
  LOGln("%s", ecg_threeleads ? "3-lead" : "2-lead");
  LOG("  R-to-R: ");
  LOGln("%s", onOffLabel(ecg_rtor));
  if (current_mode == MODE_ECG_CAL) {
    LOGln("  Note: ECG signal calibration uses internal VCAL; ECG digital HPF/LPF remain active.");
    printAppliedEcgSignalCalibrationSummary();
  } else if (current_mode == MODE_ECG || current_mode == MODE_ECG_BIOZ) {
    printAppliedEcgSummary();
  }
  LOGln("--------------------------------------------------------------------------------");
  LOGln("BIOZ Settings:");
  LOG("  Requested Speed: ~");
  LOG("%.0f", requestedBiozSamplingRateSps(bioz_speed));
  LOG(" sps (selector ");
  LOG("%u", bioz_speed);
  LOGln(")");
  LOG("  Requested Gain: ");
  LOG("%d", 10 * (1 << bioz_gain));
  LOG(" V/V (selector ");
  LOG("%u", bioz_gain);
  LOGln(")");
  LOG("  Requested AHPF: ");
  printHzOrBypass(requestedBiozAhpfHz(bioz_ahpf), 0U);
  LOG(" (selector ");
  LOG("%u", bioz_ahpf);
  LOG(") | Requested DLPF: ");
  printHzOrBypass(requestedBiozDlpfHz(bioz_dlpf), 0U);
  LOG(" (selector ");
  LOG("%u", bioz_dlpf);
  LOG(") | Requested DHPF: ");
  printHzOrBypass(requestedBiozDhpfHz(bioz_dhpf), 2U);
  LOG(" (selector ");
  LOG("%u", bioz_dhpf);
  LOGln(")");
  LOG("  Requested Frequency: ");
  LOG("%lu", static_cast<unsigned long>(bioz_frequency));
  LOG(" Hz | Requested Current: ");
  LOG("%lu", static_cast<unsigned long>(bioz_current));
  LOG(" nA | Requested Phase: ");
  LOG("%.1f", bioz_phase);
  LOGln(" deg");
  LOG("  Lead Bias: ");
  LOG("%s", onOffLabel(bioz_leadbias));
  LOG(" | Lead-Off Detect: ");
  LOG("%s", onOffLabel(bioz_leadsoffdetect));
  LOG(" | Wires: ");
  LOGln("%s", bioz_fourleads ? "4-wire" : "2-wire");
  if (current_mode == MODE_BIOZ_CAL) {
    LOGln("  Note: signal calibration overrides normal BIOZ drive frequency/current/phase and forces bypass filters.");
    printAppliedBiozSignalCalibrationSummary();
  } else if (current_mode == MODE_BIOZ || current_mode == MODE_ECG_BIOZ ||
             current_mode == MODE_BIOZ_INTERNAL_CAL || current_mode == MODE_BIOZ_EXTERNAL_CAL) {
    printAppliedBiozSummary();
  }
  LOGln("--------------------------------------------------------------------------------");
  LOGln("Scan Settings:");
  LOG("  Averages: ");
  LOG("%u", scan_avg);
  LOG(" | Fast Mode: ");
  LOG("%s", scan_fast ? "ON" : "OFF");
  LOG(" | Full Range: ");
  LOG("%s", scan_fullrange ? "ON" : "OFF");
  LOG(" | Source: ");
  LOGln("%s", scan_internal ? "Internal Resistor" : "External");
  LOG("  Phase Range: ");
  LOG("%s", scanPhaseRangeLabel(scan_phase_range));
  LOG(" | AHPF Mode: ");
  LOG("%s", scanAhpfModeLabel(scan_ahpf_mode));
  LOG(" | Fixed AHPF: ");
  LOG("%s", scanAhpfOverrideLabel(scan_fixed_ahpf));
  LOG(" | Internal BIST AHPF: ");
  LOG("%s", scanAhpfOverrideLabel(scan_internal_bist_ahpf));
  LOG(" | Settle: ");
  LOG("%u", scan_settle_samples);
  LOG(" | Current Settle: ");
  LOGln("%u", scan_current_change_settle_samples);
  LOG("  Max Current: ");
  LOG("%ld", static_cast<long>(scan_max_current_nA));
  LOG(" nA | Autorange: ");
  LOG("%s", scan_autorange_enable ? "ON" : "OFF");
  LOG(" | Human-Safe Preset: ");
  LOGln("%s", scan_human_safe ? "ON" : "OFF");
  printBiozScanCalibrationSettings();
  LOGln("--------------------------------------------------------------------------------");
  LOGln("Calibration Settings:");
  LOG("  Requested Resistor: ");
  LOG("%lu", static_cast<unsigned long>(cal_resistance));
  LOG(" Ohm | Requested Modulation: ");
  if (cal_modulation == 0U) {
    LOG("DC only");
  } else {
    LOG("%.1f", requestedCalibrationModulationOhm(cal_resistance, cal_modulation));
    LOG(" Ohm step");
  }
  LOG(" (selector ");
  LOG("%u", cal_modulation);
  LOG(") | Requested Mod Frequency: ");
  LOG("%.4f", requestedCalibrationModulationFrequencyHz(cal_mod_freq));
  LOG(" Hz (selector ");
  LOG("%u", cal_mod_freq);
  LOGln(")");
  LOG("  ECG Sig Mode: ");
  LOG("%s", calSignalModeLabel(ecg_cal_unipolar));
  LOG(" | BIOZ Sig Mode: ");
  LOGln("%s", calSignalModeLabel(bioz_cal_unipolar));
  if (current_mode == MODE_BIOZ_INTERNAL_CAL) {
    LOG("  Applied BIST: RNOM ");
    LOG("%.1f", BIOZ_test_rnom);
    LOG(" Ohm, RMOD ");
    LOG("%.1f", BIOZ_test_rmod);
    LOG(" Ohm, RMOD freq ");
    LOG("%.4f", BIOZ_test_frequency);
    LOGln(" Hz");
  }
  LOGln("================================================================================");
}

/******************************************************************************************************/
// Command Handlers
/******************************************************************************************************/

void handleGeneralCommand(char cmd) {
  switch (cmd) {
    case '?':  // Help
      helpMenu();
      break;

    case 's':  // Show settings
      showSettings();
      break;

    case 'h':  // Health check
      {
        LOGln("Running health check...");
        HealthCheckResult result = afe.healthCheck();
        const bool info_ok = (result.info_reg != 0U);
        const bool status_ok = !result.fault_present;
        LOG("SPI OK: ");
        LOGln("%s", result.spi_ok ? "YES" : "NO");
        LOG("INFO OK: ");
        LOGln("%s", info_ok ? "YES" : "NO");
        LOG("STATUS OK: ");
        LOGln("%s", status_ok ? "YES" : "NO");
        LOG("INFO REG: 0x");
        LOGln("%02X", result.info_reg);
        LOG("STATUS REG: 0x");
        LOGln("%06lX", static_cast<unsigned long>(result.status_reg));
        LOG("PLL Unlocked: ");
        LOGln("%s", result.pll_unlocked ? "YES" : "NO");
        LOG("Fault Present: ");
        LOGln("%s", result.fault_present ? "YES" : "NO");
        LOG("Overall: ");
        LOGln("%s", result.spi_ok && info_ok && status_ok ? "PASS" : "FAIL");
      }
      break;

    case 'i':  // Print device info
      LOGln("==Device Info==");

      LOGln("");
      LOGln("AFE--");

      afe.readInfo();
      afe.printInfo();

      if (battery_available) {
        cell_percent = lipo.cellPercent();
        charge_rate_percent_per_hour = lipo.chargeRate();
        LOGln("");
        LOGln("Battery--");
        LOGln("  Charge state: %.2f%%", cell_percent);
        LOGln("  Charge rate: %.2f%%/hour", charge_rate_percent_per_hour);
      } else {
        LOGln("");
        LOGln("Battery: not detected");
      }

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
      // LOG("  Commands processed: ");
      // LOGln("%lu", static_cast<unsigned long>(serial_command_count));
      // LOG("  Most recent command (%u bytes): ", static_cast<unsigned>(serial_command_length));
      // LOGln("%s", serial_last_command[0] != '\0' ? serial_last_command : "<none>");

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
      // LOG("  Commands processed: ");
      // LOGln("%lu", static_cast<unsigned long>(ble_command_count));
      // LOG("  Most recent command (%u bytes): ", static_cast<unsigned>(ble_command_length));
      // LOGln("%s", ble_last_command[0] != '\0' ? ble_last_command : "<none>");
      if (active_command_source == CommandSource::Ble) {
        ble.printStats(ble);
      } else {
        ble.printStats(Serial);
      }

      break;

    case 'r':  // Print all registers
      afe.readAllRegisters();
      afe.printAllRegisters();
      break;

    case 't':  // Print status
      afe.readStatusRegisters();
      afe.printStatus();
      break;

    case 'p':  // Print config registers
      afe.printConfig();
      break;

    case '(':  // Save config
      afe.saveConfig();
      LOGln("Configuration saved to snapshot");
      break;

    case ')':  // Restore config
      afe.restoreConfig();
      LOGln("Configuration restored from snapshot");
      break;

    case 'w':  // Software reset
      LOGln("Performing software reset...");
      afe.swReset();
      delay(500);
      LOGln("Reset complete. Reinitialize before use.");
      measurement_running = false;
      current_mode = MODE_IDLE;
      break;

    case 'x':  // Reboot
      LOGln("Rebooting...");
      #ifdef ARDUINO_ARCH_ESP32
          ESP.restart();
      #elif defined(ARDUINO_ARCH_ESP8266)
          ESP.restart();
      #else
        LOGln("Reboot not supported on this platform");
      #endif
      break;

    case 'y':  // Synchronize
      LOGln("Synchronizing...");
      afe.synch();
      LOGln("Sync complete");
      break;

    case 'k':  // Clear latched status flags
      afe.clearLatchedStatusFlags();
      LOGln("Latched status flags cleared");
      break;

    case 'f':  // FIFO reset
      afe.FIFOReset();
      LOGln("FIFO reset");
      break;

    case 'c':  // Reset sample counter
      sample_count = 0;
      last_sample_count = 0;
      last_time = millis();
      LOGln("Sample counter reset");
      break;

    case 'z':  // Toggle data display
      data_reporting = !data_reporting;
      if (data_reporting && data_output_mask == 0U) {
        data_output_mask = DATA_OUT_ALL;
      }
      printDataOutputSettings();
      break;

    case 'a':  // Apply settings
      applySettings();
      break;

    case '.':  // Toggle start/stop
      toggleMeasurement();
      break;

    case '>':  // Start
      requestStartMeasurement();
      break;

    case '<':  // Stop
      if (measurement_running) {
        stopMeasurement();
      } else {
        LOGln("Not running");
      }
      break;

    default:
      LOGln("Unknown command");
      break;
  }
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
    if (measurement_running && measurement_log_level_saved) {
      saved_measurement_log_level = static_cast<int>(value);
      LOG("Log level saved for after acquisition: ");
    } else {
      logSetLevel(static_cast<int>(value));
      LOG("Log level set to: ");
    }
    LOGln("%ld", value);
    return;
  }

  // External BIOZ scan calibration commands
  if (cmd1 == 'K') {
    switch (cmd2) {
      case 'p':  // Print scan calibration
        if (command_len != 2U) {
          LOGln("Kp does not take an argument");
          break;
        }
        printBiozScanCalibrationSettings();
        LOGln("  Reliability: 3=validated, 2=boundary, 1=questionable, 0=outside");
        break;

      case 'e':  // Enable/disable correction
        if (!numeric_valid) {
          LOGln("Ke requires an integer argument");
          break;
        }
        if (value == 0 || value == 1) {
          bioz_scan_cal_enabled = (value == 1);
          LOG("BIOZ scan correction: ");
          LOGln(bioz_scan_cal_enabled ? "ON" : "OFF");
        } else {
          LOGln("BIOZ scan correction enable must be 0 or 1");
        }
        break;

      case 'g':  // Global K in ppm, e.g. Kg1250650 -> 1.25065
        if (!numeric_valid) {
          LOGln("Kg requires an integer argument");
          break;
        }
        if (value >= 500000 && value <= 2000000) {
          bioz_scan_cal_global_k = static_cast<float>(value) / 1000000.0f;
          LOG("BIOZ scan global K set to: ");
          LOGln("%.6f", bioz_scan_cal_global_k);
        } else {
          LOGln("BIOZ scan global K ppm must be 500000-2000000");
        }
        break;

      case 'r':  // Reset runtime calibration defaults
        if (command_len != 2U) {
          LOGln("Kr does not take an argument");
          break;
        }
        bioz_scan_cal_enabled = false;
        bioz_scan_cal_global_k = BIOZ_SCAN_CAL_DEFAULT_GLOBAL_K;
        LOGln("BIOZ scan calibration reset to runtime defaults");
        printBiozScanCalibrationSettings();
        break;

      default:
        LOGln("Unknown BIOZ scan calibration command");
        break;
    }
    return;
  }

  // Non-volatile preferences commands. These store semantic sketch settings,
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
        loadPreferencesProfile(true);
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

  // Mode commands
  if (cmd1 == 'm') {
    if (command_len != 2U) {
      LOGln("Mode command must be m1-m8");
      return;
    }
    if (measurement_running) {
      stopMeasurement();
      LOGln("Stopped previous measurement");
    }

    switch (cmd2) {
      case '1': current_mode = MODE_ECG; break;
      case '2': current_mode = MODE_BIOZ; break;
      case '3': current_mode = MODE_ECG_BIOZ; break;
      case '4': current_mode = MODE_ECG_CAL; break;
      case '5': current_mode = MODE_BIOZ_CAL; break;
      case '6': current_mode = MODE_BIOZ_INTERNAL_CAL; break;
      case '7': current_mode = MODE_BIOZ_EXTERNAL_CAL; break;
      case '8': current_mode = MODE_BIOZ_SCAN; break;
      default:
        LOGln("Invalid mode number (1-8)");
        return;
    }
    LOG("Mode changed to: ");
    LOGln("%s", getModeName(current_mode));
    applySettings();
    return;
  }

  // ECG commands
  if (cmd1 == 'E') {
    if (strchr("sglhernq", cmd2) != nullptr && !numeric_valid) {
      LOGln("ECG setting requires an integer argument");
      return;
    }
    switch (cmd2) {
      case 's':  // ECG speed
        if (value >= 0 && value <= 2) {
          ecg_speed = value;
          LOG("ECG speed requested: ~");
          LOG("%.0f", requestedEcgSamplingRateSps(ecg_speed));
          LOG(" sps (selector ");
          LOG("%u", ecg_speed);
          LOGln(")");
        } else {
          LOGln("ECG speed must be 0-2");
        }
        break;

      case 'g':  // ECG gain
        if (value >= 0 && value <= 3) {
          ecg_gain = value;
          LOG("ECG gain requested: ");
          LOG("%d", 20 << ecg_gain);
          LOG(" V/V (selector ");
          LOG("%u", ecg_gain);
          LOGln(")");
        } else {
          LOGln("ECG gain must be 0-3");
        }
        break;

      case 'l':  // ECG digital LPF
        if ((value >= 0 && value <= 3) || value == 255) {
          ecg_lpf = static_cast<uint8_t>(value);
          LOG("ECG digital LPF requested: ");
          if (ecg_lpf == ECG_FILTER_AUTO) {
            LOG("auto (");
            printHzOrBypass(requestedEcgLpfHz(defaultEcgLpfSelector(current_mode, ecg_speed)), 0U);
            LOGln(")");
          } else {
            printHzOrBypass(requestedEcgLpfHz(ecg_lpf), 0U);
            LOGln("");
          }
        } else {
          LOGln("ECG digital LPF must be 0-3 or 255 (auto)");
        }
        break;

      case 'h':  // ECG digital HPF
        if ((value >= 0 && value <= 1) || value == 255) {
          ecg_hpf = static_cast<uint8_t>(value);
          LOG("ECG digital HPF requested: ");
          if (ecg_hpf == ECG_FILTER_AUTO) {
            LOG("auto (");
            printHzOrBypass(requestedEcgHpfHz(defaultEcgHpfSelector(current_mode)), 2U);
            LOGln(")");
          } else {
            printHzOrBypass(requestedEcgHpfHz(ecg_hpf), 2U);
            LOGln("");
          }
        } else {
          LOGln("ECG digital HPF must be 0-1 or 255 (auto)");
        }
        break;

      case 'e':  // ECG leads
        if (value == 2 || value == 3) {
          ecg_threeleads = (value == 3);
          LOG("ECG leads requested: ");
          LOGln("%s", ecg_threeleads ? "3-lead" : "2-lead");
        } else {
          LOGln("ECG leads must be 2 or 3");
        }
        break;

      case 'r':  // ECG R-to-R
        if (value == 0 || value == 1) {
          ecg_rtor = (value == 1);
          LOG("ECG R-to-R requested: ");
          LOGln("%s", onOffLabel(ecg_rtor));
        } else {
          LOGln("ECG R-to-R must be 0 or 1");
        }
        break;

      case 'n':  // ECG notch
        if (value == 0 || value == 50 || value == 60) {
          ecg_notch = static_cast<uint8_t>(value);
          LOG("ECG notch requested: ");
          LOGln(ecgNotchLabel());
        } else {
          LOGln("ECG notch must be 0, 50, or 60");
        }
        break;

      case 'q':  // ECG notch Q
        if (value >= 1 && value <= 100) {
          ecg_notch_q = static_cast<float>(value);
          LOG("ECG notch Q requested: ");
          LOGln("%.1f", ecg_notch_q);
        } else {
          LOGln("ECG notch Q must be 1-100");
        }
        break;

      default:
        LOGln("Unknown ECG command");
        break;
    }
    return;
  }

  // BIOZ commands
  if (cmd1 == 'B') {
    if (strchr("sgadhfcplow", cmd2) != nullptr && !numeric_valid) {
      LOGln("BIOZ setting requires an integer argument");
      return;
    }
    switch (cmd2) {
      case 's':  // BIOZ speed
        if (value >= 0 && value <= 1) {
          bioz_speed = value;
          LOG("BIOZ speed requested: ~");
          LOG("%.0f", requestedBiozSamplingRateSps(bioz_speed));
          LOG(" sps (selector ");
          LOG("%u", bioz_speed);
          LOGln(")");
        } else {
          LOGln("BIOZ speed must be 0-1");
        }
        break;

      case 'g':  // BIOZ gain
        if (value >= 0 && value <= 3) {
          bioz_gain = value;
          LOG("BIOZ gain requested: ");
          LOG("%d", 10 * (1 << bioz_gain));
          LOG(" V/V (selector ");
          LOG("%u", bioz_gain);
          LOGln(")");
        } else {
          LOGln("BIOZ gain must be 0-3");
        }
        break;

      case 'a':  // BIOZ analog HPF
        if (current_mode == MODE_BIOZ_CAL) {
          LOGln("BIOZ signal calibration forces AHPF bypass; command ignored in m5.");
          break;
        }
        if (value >= 0 && value <= 7) {
          bioz_ahpf = value;
          LOG("BIOZ analog HPF requested: ");
          printHzOrBypass(requestedBiozAhpfHz(bioz_ahpf), 0U);
          LOG(" (selector ");
          LOG("%u", bioz_ahpf);
          LOGln(")");
        } else {
          LOGln("BIOZ analog HPF must be 0-7");
        }
        break;

      case 'd':  // BIOZ digital LPF
        if (current_mode == MODE_BIOZ_CAL) {
          LOGln("BIOZ signal calibration forces DLPF bypass; command ignored in m5.");
          break;
        }
        if (value >= 0 && value <= 3) {
          bioz_dlpf = value;
          LOG("BIOZ digital LPF requested: ");
          printHzOrBypass(requestedBiozDlpfHz(bioz_dlpf), 0U);
          LOG(" (selector ");
          LOG("%u", bioz_dlpf);
          LOGln(")");
        } else {
          LOGln("BIOZ digital LPF must be 0-3");
        }
        break;

      case 'h':  // BIOZ digital HPF
        if (current_mode == MODE_BIOZ_CAL) {
          LOGln("BIOZ signal calibration forces DHPF bypass; command ignored in m5.");
          break;
        }
        if (value >= 0 && value <= 3) {
          bioz_dhpf = value;
          LOG("BIOZ digital HPF requested: ");
          printHzOrBypass(requestedBiozDhpfHz(bioz_dhpf), 2U);
          LOG(" (selector ");
          LOG("%u", bioz_dhpf);
          LOGln(")");
        } else {
          LOGln("BIOZ digital HPF must be 0-3");
        }
        break;

      case 'f':  // BIOZ frequency
        if (value >= 125 && value <= 128000) {
          bioz_frequency = value;
          LOG("BIOZ frequency requested: ");
          LOG("%ld", value);
          LOGln(" Hz");
        } else {
          LOGln("BIOZ frequency must be 125-128000 Hz");
        }
        break;

      case 'c':  // BIOZ current
        if (value >= 0 && value <= 96000) {
          bioz_current = value;
          LOG("BIOZ current requested: ");
          LOG("%ld", value);
          LOGln(" nA");
        } else {
          LOGln("BIOZ current must be 0-96000 nA");
        }
        break;

      case 'p':  // BIOZ phase
        bioz_phase = value;
        LOG("BIOZ phase requested: ");
        LOG("%ld", value);
        LOGln(" deg");
        break;

      case 'l':  // BIOZ lead bias
        if (value == 0 || value == 1) {
          bioz_leadbias = (value == 1);
          LOG("BIOZ lead bias requested: ");
          LOGln("%s", onOffLabel(bioz_leadbias));
        } else {
          LOGln("BIOZ lead bias must be 0 or 1");
        }
        break;

      case 'o':  // BIOZ lead-off detect
        if (value == 0 || value == 1) {
          bioz_leadsoffdetect = (value == 1);
          LOG("BIOZ lead-off detect requested: ");
          LOGln("%s", onOffLabel(bioz_leadsoffdetect));
        } else {
          LOGln("BIOZ lead-off must be 0 or 1");
        }
        break;

      case 'w':  // BIOZ wires
        if (value == 2 || value == 4) {
          bioz_fourleads = (value == 4);
          LOG("BIOZ wiring requested: ");
          LOGln("%s", bioz_fourleads ? "4-wire" : "2-wire");
        } else {
          LOGln("BIOZ wires must be 2 or 4");
        }
        break;

      default:
        LOGln("Unknown BIOZ command");
        break;
    }
    return;
  }

  // Scan commands
  if (cmd1 == 'S') {
    if (strchr("afriphvxtcmoy", cmd2) != nullptr && !numeric_valid) {
      LOGln("Scan setting requires an integer argument");
      return;
    }
    switch (cmd2) {
      case 'a':  // Scan averages
        if (value >= 1 && value <= 8) {
          scan_avg = value;
          LOG("Scan averages requested: ");
          LOGln("%ld", value);
        } else {
          LOGln("Scan averages must be 1-8");
        }
        break;

      case 'f':  // Scan fast mode
        if (value == 0 || value == 1) {
          scan_fast = (value == 1);
          LOG("Scan speed requested: ");
          LOGln("%s", scan_fast ? "~64 sps BIOZ" : "~32 sps BIOZ");
        } else {
          LOGln("Scan fast mode must be 0 or 1");
        }
        break;

      case 'r':  // Scan full range
        if (value == 0 || value == 1) {
          scan_fullrange = (value == 1);
          LOG("Scan range requested: ");
          LOGln("%s", scan_fullrange ? "128 kHz down to 125 Hz" : "128 kHz down to 1 kHz");
        } else {
          LOGln("Scan full range must be 0 or 1");
        }
        break;

      case 'i':  // Scan internal resistor
        if (value == 0 || value == 1) {
          scan_internal = (value == 1);
          LOG("Scan source requested: ");
          LOGln("%s", scan_internal ? "INTERNAL RESISTOR" : "EXTERNAL ELECTRODES");
        } else {
          LOGln("Scan source must be 0 or 1");
        }
        break;

      case 'p':  // Scan phase range
        if (value == 0 || value == 1) {
          scan_phase_range = (value == 1) ? BIOZ_SCAN_PHASE_REDUCED : BIOZ_SCAN_PHASE_FULL;
          LOG("Scan phase range requested: ");
          LOGln("%s", scanPhaseRangeLabel(scan_phase_range));
        } else {
          LOGln("Scan phase range must be 0 (full) or 1 (reduced)");
        }
        break;

      case 'h':  // Scan AHPF mode
        if (value >= 0 && value <= 2) {
          scan_ahpf_mode = static_cast<BIOZScanAHPFMode>(value);
          if ((scan_ahpf_mode != BIOZ_SCAN_AHPF_FIXED) || (scan_fixed_ahpf != 1U)) {
            scan_human_safe = false;
          }
          LOG("Scan AHPF mode requested: ");
          LOGln("%s", scanAhpfModeLabel(scan_ahpf_mode));
        } else {
          LOGln("Scan AHPF mode must be 0 (dynamic), 1 (fixed), or 2 (bypass)");
        }
        break;

      case 'v':  // Scan fixed AHPF selector
        if (value >= 0 && value <= 7) {
          scan_fixed_ahpf = static_cast<uint8_t>(value);
          if ((scan_ahpf_mode != BIOZ_SCAN_AHPF_FIXED) || (scan_fixed_ahpf != 1U)) {
            scan_human_safe = false;
          }
          LOG("Scan fixed AHPF requested: ");
          LOGln("%s", scanAhpfOverrideLabel(scan_fixed_ahpf));
        } else {
          LOGln("Scan fixed AHPF must be 0-7");
        }
        break;

      case 'x':  // Scan internal BIST AHPF override
        if ((value >= 0 && value <= 7) || value == 255) {
          scan_internal_bist_ahpf = static_cast<uint8_t>(value);
          LOG("Scan internal AHPF requested: ");
          LOGln("%s", scanAhpfOverrideLabel(scan_internal_bist_ahpf));
        } else {
          LOGln("Scan internal AHPF must be 255 (dynamic) or 0-7");
        }
        break;

      case 't':  // Scan settle samples
        if (value >= 1 && value <= 64) {
          scan_settle_samples = static_cast<uint8_t>(value);
          if (scan_current_change_settle_samples < scan_settle_samples) {
            scan_current_change_settle_samples = scan_settle_samples;
          }
          LOG("Scan settle requested: ");
          LOG("%u", scan_settle_samples);
          LOGln(" samples");
        } else {
          LOGln("Scan settle must be 1-64 samples");
        }
        break;

      case 'c':  // Scan current-change settle samples
        if (value >= 1 && value <= 64) {
          scan_current_change_settle_samples = static_cast<uint8_t>(value);
          if (scan_current_change_settle_samples < scan_settle_samples) {
            scan_current_change_settle_samples = scan_settle_samples;
          }
          LOG("Scan current-change settle requested: ");
          LOG("%u", scan_current_change_settle_samples);
          LOGln(" samples");
        } else {
          LOGln("Scan current-change settle must be 1-64 samples");
        }
        break;

      case 'm':  // Scan maximum current
        if (value >= 55 && value <= 96000) {
          scan_max_current_nA = value;
          if (bioz_current > static_cast<uint32_t>(scan_max_current_nA)) {
            bioz_current = static_cast<uint32_t>(scan_max_current_nA);
          }
          if (scan_max_current_nA != 8000 || scan_autorange_enable) {
            scan_human_safe = false;
          }
          LOG("Scan max current requested: ");
          LOG("%ld", static_cast<long>(scan_max_current_nA));
          LOGln(" nA");
        } else {
          LOGln("Scan max current must be 55-96000 nA");
        }
        break;

      case 'o':  // Scan autorange enable
        if (value == 0 || value == 1) {
          scan_autorange_enable = (value == 1);
          if (scan_autorange_enable || scan_max_current_nA != 8000) {
            scan_human_safe = false;
          }
          LOG("Scan autorange requested: ");
          LOGln("%s", scan_autorange_enable ? "ON" : "OFF");
        } else {
          LOGln("Scan autorange must be 0 or 1");
        }
        break;

      case 'y':  // Human-safe scan preset
        if (value == 0 || value == 1) {
          if (value == 1) {
            applyHumanSafeScanPreset();
            LOGln("Human-safe BIOZ scan preset ON: external, 4-wire, reduced phase, fast, fixed 150 Hz AHPF, 8 uA max, autorange OFF");
          } else {
            scan_human_safe = false;
            scan_max_current_nA = 96000;
            scan_autorange_enable = true;
            LOGln("Human-safe BIOZ scan preset OFF: scan max current 96000 nA, autorange ON");
          }
        } else {
          LOGln("Human-safe scan preset must be 0 or 1");
        }
        break;

      default:
        LOGln("Unknown Scan command");
        break;
    }
    return;
  }

  // Calibration commands
  if (cmd1 == 'C') {
    if (strchr("rmfeb", cmd2) != nullptr && !numeric_valid) {
      LOGln("Calibration setting requires an integer argument");
      return;
    }
    switch (cmd2) {
      case 'r':  // Calibration resistor
        if (value > 0 && value <= 100000) {
          cal_resistance = value;
          LOG("Calibration resistor requested: ");
          LOG("%ld", value);
          LOGln(" Ohm");
        } else {
          LOGln("Calibration resistor must be 1-100000 Ohm");
        }
        break;

      case 'm':  // Calibration modulation
        if (value >= 0 && value <= 3) {
          cal_modulation = value;
          LOG("Calibration modulation requested: ");
          if (cal_modulation == 0U) {
            LOGln("DC only");
          } else {
            LOG("%.1f", requestedCalibrationModulationOhm(cal_resistance, cal_modulation));
            LOG(" Ohm step (selector ");
            LOG("%u", cal_modulation);
            LOGln(")");
          }
        } else {
          LOGln("Calibration modulation must be 0-3");
        }
        break;

      case 'f':  // Calibration modulation frequency
        if (value >= 0 && value <= 4) {
          cal_mod_freq = value;
          LOG("Calibration modulation frequency requested: ");
          LOG("%.4f", requestedCalibrationModulationFrequencyHz(cal_mod_freq));
          LOG(" Hz (selector ");
          LOG("%u", cal_mod_freq);
          LOGln(")");
        } else {
          LOGln("Calibration mod frequency must be 0-4");
        }
        break;

      case 'e':  // ECG signal calibration mode
        if (value == 0 || value == 1) {
          ecg_cal_unipolar = (value == 1);
          LOG("ECG signal calibration mode requested: ");
          LOGln("%s", calSignalModeLabel(ecg_cal_unipolar));
        } else {
          LOGln("ECG signal calibration mode must be 0 (bipolar) or 1 (unipolar)");
        }
        break;

      case 'b':  // BIOZ signal calibration mode
        if (value == 0 || value == 1) {
          bioz_cal_unipolar = (value == 1);
          LOG("BIOZ signal calibration mode requested: ");
          LOGln("%s", calSignalModeLabel(bioz_cal_unipolar));
        } else {
          LOGln("BIOZ signal calibration mode must be 0 (bipolar) or 1 (unipolar)");
        }
        break;

      default:
        LOGln("Unknown Calibration command");
        break;
    }
    return;
  }

  LOGln("Unknown command");
}

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
  // if (source == CommandSource::Ble) {
    // ++ble_command_count;
    // ble_command_length = len;
    // me`mcpy(ble_last_command, command_buff, len + 1U);
  // } else {
    // ++serial_command_count;
    // serial_command_length = len;
    // memcpy(serial_last_command, command_buff, len + 1U);
  // }

  // Single-character commands
  if (len == 1U) {
    handleGeneralCommand(command_buff[0]);
  }
  // Parameter commands
  else {
    handleParameterCommand(command_buff);
  }
}

// Display Data
//////////////////////////////////////////////////////////////////////////////////////////////

void printSpectrum(const ImpedanceSpectrum& spectrum) {
  if (bioz_scan_cal_enabled && !scan_internal) {
    LOGln("frequency_hz,magnitude_ohm,phase_deg,reliability");
  } else {
    LOGln("frequency_hz,magnitude_ohm,phase_deg");
  }
  for (uint8_t i = 0; i < MAX30001_BIOZ_NUM_FREQUENCIES; ++i) {
    if (spectrum.frequency[i] <= 0.0f) {
      continue;
    }

    const float output_magnitude = (bioz_scan_cal_enabled && !scan_internal && bioz_scan_cal_global_k > 0.0f)
      ? spectrum.magnitude[i] * bioz_scan_cal_global_k
      : spectrum.magnitude[i];
    LOG("%.1f", spectrum.frequency[i]);
    LOG(",");
    LOG("%.3f", output_magnitude);
    LOG(",");
    LOG("%.3f", spectrum.phase[i]);
    if (bioz_scan_cal_enabled && !scan_internal) {
      LOG(",");
      LOG("%u", biozScanReliabilityScore(spectrum.frequency[i], output_magnitude, spectrum.phase[i]));
    }
    LOGln("");
  }
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

bool serialBufferWriteFloat(float value, uint8_t decimals) {
  char text[24];
  dtostrf(value, 0, decimals, text);
  return serialBufferWrite(text);
}

bool serialBufferWriteUInt(uint32_t value) {
  char text[12];
  ultoa(value, text, 10);
  return serialBufferWrite(text);
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

void drainRtorToPending(bool keep_for_ecg_line) {
  float value = 0.0f;
  while (RTOR_data.available() > 0) {
    RTOR_data.pop(value);
    rtor_seen_count++;
    if (keep_for_ecg_line) {
      if (pending_rtor_count < MAX_PENDING_RTOR) {
        pending_rtor_ms[pending_rtor_count++] = value;
      } else {
        rtor_dropped_count++;
      }
    }
  }
}

bool appendPendingRtorToSerialLine() {
  if (pending_rtor_count == 0U) {
    return false;
  }

  bool wrote = false;
  serialBufferWrite("RR [ms]: ");
  for (uint8_t i = 0U; i < pending_rtor_count; i++) {
    serialBufferWriteFloat(pending_rtor_ms[i], 1U);
    serialBufferWriteChar(' ');
  }

  serialBufferWrite("HR [bpm]: ");
  for (uint8_t i = 0U; i < pending_rtor_count; i++) {
    if (pending_rtor_ms[i] > 0.0f) {
      serialBufferWriteFloat(60000.0f / pending_rtor_ms[i], 1U);
      serialBufferWriteChar(' ');
    }
  }

  rtor_printed_count += pending_rtor_count;
  pending_rtor_count = 0U;
  wrote = true;
  return wrote;
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

void serialOutFlush(uint32_t timeout_ms) {
  const uint32_t start_ms = millis();
  while ((serial_output_buff.available() > 0 || ble_output_buff.available() > 0) &&
         ((millis() - start_ms) < timeout_ms)) {
    serialOutUpdate(SERIAL_OUTPUT_CHUNK_BYTES);
    delay(1);
  }
}

// Write all available data to serial output buffer
void dataToSerialBuff() {
  float value = 0.0f;
  bool printed_data = false;

  const ModeRuntimePolicy policy = getModeRuntimePolicy(current_mode);

  if (!data_reporting) { //---------------------------------

    // No reporting, just pop the data to clear the buffers

    // ECG
    while (ECG_data.available() > 0) {
      ECG_data.pop(value);
      applyEcgNotchFilter(value);
      last_ecg_data_ms = millis();
      ecg_timeout_reported = false;
      sample_count++;
    }

    // BIOZ
    if (!policy.expect_scan) {
      while (BIOZ_data.available() > 0) {
        BIOZ_data.pop(value);
        last_bioz_data_ms = millis();
        bioz_timeout_reported = false;
        sample_count++;
      }
    }

    // RtoR
    if (RTOR_data.available() > 0) {
      drainRtorToPending(false);
    }


  } else { // -------------------------------

    // In combined ECG+BIOZ mode the two streams arrive at different rates
    // (ECG is 4-16x faster than BIOZ). Gate the combined print on the slowest
    // expected stream so all data lands on one line per BIOZ interrupt cycle.
    // RTOR is slower still and is held pending until the next ECG-containing
    // line. It is not used as a hard gate because waiting for a beat can
    // overflow the ECG software buffer.
    // Single-channel modes are unaffected (their only expected stream triggers).
    // BIOZ scan mode owns BIOZ_data internally until it publishes BIOZ_spectrum.

    const bool ecg_ready  = policy.expect_ecg  && (ECG_data.available()  > 0);
    const bool bioz_ready = policy.expect_bioz  && (BIOZ_data.available() > 0) && !policy.expect_scan;

    if (RTOR_data.available() > 0) {
      drainRtorToPending(policy.expect_ecg && ecg_rtor);
    }

    const bool trigger_print =
      (policy.expect_ecg && policy.expect_bioz) ? bioz_ready           // combined: wait for slower BIOZ
      : policy.expect_ecg                        ? ecg_ready            // ECG-only
      : policy.expect_bioz                       ? bioz_ready           // BIOZ-only
                                                 : false;               // scan / idle

    if (trigger_print) {

      // ECG Data (all accumulated samples since last BIOZ interrupt)

      if (ecg_ready) {
        serialBufferWrite(policy.ecg_label != nullptr ? policy.ecg_label : "ECG [mV]: ");
        while (ECG_data.available() > 0) {
          ECG_data.pop(value);
          const float ecg_output = applyEcgNotchFilter(value);
          serialBufferWriteFloat(ecg_output, 3U);
          serialBufferWriteChar(' ');
          last_ecg_data_ms = millis();
          ecg_timeout_reported = false;
          sample_count++;
        }
        printed_data = true;
      }

      // BIOZ Data (all accumulated samples since last BIOZ interrupt)

      if (bioz_ready) {
        serialBufferWrite(policy.bioz_label != nullptr ? policy.bioz_label : "BIOZ [Ohm]: ");
        while (BIOZ_data.available() > 0) {
          BIOZ_data.pop(value);
          if (policy.bioz_raw) {
            serialBufferWriteFloat(value, 0U);
          } else {
            serialBufferWriteFloat(value, 2U);
          }
          serialBufferWriteChar(' ');
          last_bioz_data_ms = millis();
          bioz_timeout_reported = false;
          sample_count++;
        }
        printed_data = true;
      }

      // RTOR/HR Data. Append only to ECG-containing lines so one serial line
      // remains one display timepoint for host-side plotting.
      if (policy.expect_ecg && ecg_ready) {
        if (appendPendingRtorToSerialLine()) {
          printed_data = true;
        }
      } else if (!policy.expect_ecg || !ecg_rtor) {
        if (pending_rtor_count > 0U) {
          rtor_dropped_count += pending_rtor_count;
          pending_rtor_count = 0U;
        }
        printed_data = true;
      }
    }

    // // Periodic sampling rate display (every 5 seconds)
    //   current_time = millis();
    //   if (current_time - last_time >= 5000) {
    //     uint32_t sample_delta = sample_count - last_sample_count;
    //     uint32_t time_delta = current_time - last_time;
    //     sampling_rate = (sample_delta * 1000.0f) / time_delta;

    //     // if (data_reporting) {
    //     //   LOG("Sampling rate [1/s]: ");
    //     //   LOG(sampling_rate, 1);
    //     //   LOG(" Samples: ");
    //     //   LOG(sample_count);
    //     // }

    //     last_sample_count = sample_count;
    //     last_time = current_time;
    //   }

    if (printed_data) { serialBufferWriteLine(); }

  }

  while (BIOZ_spectrum.available() > 0) {
    ImpedanceSpectrum spectrum;
    if (BIOZ_spectrum.pop(spectrum) != 1U) {
      break;
    }
    LOGI("BIOZ scan complete");
    printSpectrum(spectrum);
    stopMeasurement();
  }
}

// Display
//////////////////////////////////////////////////////////////////////////////////////////////

void drawBatteryIndicator(int x, int y, float percent) {
  display.drawBitmap(x, y, BATTERY_BITMAP, BAT_W, BAT_H, SSD1306_WHITE);
  float pct = isfinite(percent) ? percent : 0.0f;
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  int width = (int)roundf((pct / 100.0f) * BAT_FULL);
  display.fillRect(x + 3, y + 3, width, BAT_THICK, SSD1306_WHITE); // Draw the filled rectangle
}

void drawChargingIndicator(int x, int y, int width, int height, float percent_per_hour,
                           float max_percent = 100.0f) {
  // Draw the outline rectangles with one pixel separation
  display.drawRect(x, y, width, height, SSD1306_WHITE); // Left rectangle
  display.drawRect(x + width + 1, y, width, height, SSD1306_WHITE); // Right rectangle

  // Calculate the width of the inner bar, scaled to -max_percent .. +max_percent.
  const float rate = isfinite(percent_per_hour) ? percent_per_hour : 0.0f;
  const float scale_max = (max_percent > 0.0f) ? max_percent : 100.0f;
  int innerWidth = (int)roundf((width - 2) * (fabsf(rate) / scale_max));
  if (innerWidth < 0) innerWidth = 0;
  if (innerWidth > (width - 2)) innerWidth = (width - 2);

  // Show at least a 1-pixel bar for any non-zero rate so small values
  // (e.g. idle discharge / trickle charge) are still visible.
  if (innerWidth == 0 && fabsf(rate) > 0.05f) {
    innerWidth = 1;
  }

  // Draw the inner filled rectangle based on charge rate
  if (rate < 0.0f) {
    // Discharging (left rectangle)
    display.fillRect(x + width - innerWidth, y + 2, innerWidth, height - 4, SSD1306_WHITE);
  } else {
    // Charging (right rectangle)
    display.fillRect(x + width + 2, y + 2, innerWidth, height - 4, SSD1306_WHITE);
  }
}

void drawMeasurementModeDisplay(OperationMode mode, bool running) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(running ? "Measuring" : "Selected");

  const char* mode_name = getDisplayModeName(mode);
  display.setCursor(0, 14);
  if (mode == MODE_BIOZ_SCAN ||
      mode == MODE_ECG_CAL ||
      mode == MODE_BIOZ_CAL ||
      mode == MODE_BIOZ_INTERNAL_CAL ||
      mode == MODE_BIOZ_EXTERNAL_CAL) {
    display.setTextSize(1);
  } else {
    display.setTextSize(2);
  }
  display.print(mode_name);
  display.setTextSize(1);
}

void enterDeepSleep() {
  LOGln("Long press detected; entering deep sleep...");

  // Resetting after end() clears calibration, lead-detection, and channel-enable
  // bits. With EN_ECG, EN_BIOZ, and EN_ULP_LON clear, the MAX30001G enters
  // its shutdown state.
  afe.end();
  afe.swReset();
  measurement_running = false;

  if (display_available) {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
  }

  if (battery_available) {
    lipo.enableSleep(true);
    lipo.sleep(true);
  }

  Serial.flush();
  ble.flush();
  ble.end();
  detachInterrupt(digitalPinToInterrupt(BUTTON_PIN));

  digitalWrite(LED_PIN, LOW);
  led_state = false;

  #ifdef NEOPIXEL_POWER
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, LOW);
  #endif

  #if defined(ARDUINO_ARCH_ESP32)
    // Configure the released button as an active-high deep-sleep wake source.
    rtc_gpio_init(BUTTON_PIN_GPIO);
    rtc_gpio_set_direction(BUTTON_PIN_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_dis(BUTTON_PIN_GPIO);
    rtc_gpio_pulldown_en(BUTTON_PIN_GPIO);
    esp_sleep_enable_ext0_wakeup(BUTTON_PIN_GPIO, 1);

    if (rtc_gpio_is_valid_gpio((gpio_num_t)LED_PIN)) {
      gpio_hold_dis((gpio_num_t)LED_PIN);
      gpio_hold_en((gpio_num_t)LED_PIN);
      gpio_deep_sleep_hold_en();
    }
  #endif

  #ifdef I2C_POWER
    digitalWrite(I2C_POWER, LOW);
  #endif

  delay(20);
  #if defined(ARDUINO_ARCH_ESP32)
    esp_deep_sleep_start();
  #else
    LOGE("Deep sleep is only implemented for ESP32 targets");
  #endif
}

/******************************************************************************************************/
// Setup
/******************************************************************************************************/

void setup() {

  #if defined(ARDUINO_ARCH_ESP32)
    // Release any previous deep-sleep holds so we can drive pins normally
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis((gpio_num_t)LED_PIN);
  #endif

  currentLogLevel = LOG_LEVEL;

  // Serial Setup
  Serial.begin(BAUDRATE);
  Serial.setTimeout(1000);
  serialTrigger(waitmsg, 5000);

  printWakeupReason();

  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLDOWN);
  button_raw_state = digitalRead(BUTTON_PIN);
  button_state = button_raw_state;
  last_button_change_us = micros();
  if (button_state == HIGH) {
    button_pressed_us = last_button_change_us;
  }
  #ifdef I2C_POWER
    pinMode(I2C_POWER, OUTPUT);
    digitalWrite(I2C_POWER, HIGH); // Turn on power to QWIIC connector
  #endif
  delay(10);
  Wire.begin();

  // Initialize PSRAM (optional check)
  #ifdef ARDUINO_ARCH_ESP32
    if (psramInit()) {
      LOG("PSRAM: total=");
      LOG("%lu", static_cast<unsigned long>(ESP.getPsramSize()));
      LOG(" free=");
      LOGln("%lu", static_cast<unsigned long>(ESP.getFreePsram()));
    } else {
      LOGln("No PSRAM available.");
    }
  #endif

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, CHANGE);

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

  // LIPO Battery
  // -------------
  battery_available = lipo.begin();
  if (battery_available) {
    cell_percent = lipo.cellPercent();
    charge_rate_percent_per_hour = lipo.chargeRate();
    LOGI("MAX17048 initialized");
  } else {
    LOGW("MAX17048 not detected; battery monitoring disabled");
  }

  // OLED Display
  // -------------
   // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  Wire.beginTransmission(SCREEN_ADDRESS);
  const bool display_detected = (Wire.endTransmission() == 0U);
  display_available = display_detected &&
                      display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS, true, false);
  if (display_available) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("Display Starting...");
    display.display();
  } else {
    LOGW("SSD1306 not detected or buffer allocation failed; display disabled");
  }

  // AFE Initialization
  afe.begin();
  LOGI("MAX30001G initialized");

  // Read device info
  afe.readInfo();
  afe.printInfo();

  // Health check
  LOGI("Running health check...");
  HealthCheckResult result = afe.healthCheck();
  const bool info_ok = (result.info_reg != 0U);
  const bool status_ok = !result.fault_present;
  if (result.spi_ok && info_ok && status_ok) {
    LOGI("Health check: PASS");
  } else {
    LOGE("Health check: FAIL");
  }

  loadPreferencesProfile(false);

  LOGln("");
  LOGln("MAX30001G Interactive Program Starting");
  LOGln("======================================");
  LOGln("Ready for commands. Type '?' for help.");
  LOGln("");
}

/******************************************************************************************************/
// Main Loop
/******************************************************************************************************/

void loop() {
  current_time = micros();

  // Battery Update
  // =======================================================
  if (battery_available &&
      (current_time - last_battery_us) >= BATTERY_UPDATE_INTERVAL_US) {
    last_battery_us = current_time;
    const float updated_charge_rate = lipo.chargeRate();
    const float updated_cell_percent = lipo.cellPercent();
    if (isfinite(updated_charge_rate)) {
      charge_rate_percent_per_hour = updated_charge_rate;
    }
    if (isfinite(updated_cell_percent)) {
      cell_percent = updated_cell_percent;
    }
  }

  // Button Update
  // =======================================================
  if (button_edge_pending) {
    noInterrupts();
    button_edge_pending = false;
    interrupts();

    const int reading = digitalRead(BUTTON_PIN);
    if (reading != button_raw_state) {
      button_raw_state = reading;
      last_button_change_us = current_time;
    }
  }

  if ((button_raw_state != button_state) &&
      ((current_time - last_button_change_us) >= BUTTON_DEBOUNCE_US)) {
    button_state = button_raw_state;

    if (button_state == HIGH) {
      button_pressed_us = current_time;
      digitalWrite(LED_PIN, HIGH);
      led_state = true;
    } else {
      const uint32_t press_duration_us = current_time - button_pressed_us;
      digitalWrite(LED_PIN, LOW);
      led_state = false;

      if (press_duration_us >= BUTTON_LONG_PRESS_US) {
        enterDeepSleep();
      } else {
        LOGI("Short press; toggling display state");
        display_mode = (display_mode == DISPLAY_BATTERY) ? DISPLAY_DATA : DISPLAY_BATTERY;
        display_update_interval_us = (display_mode == DISPLAY_DATA)
                                       ? DISPLAY_DATA_UPDATE_INTERVAL_US
                                       : DISPLAY_BATTERY_UPDATE_INTERVAL_US;
        display_update_requested = true;
      }
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

  // Update AFE and display data if running
  if (measurement_running) {
    afe.update(current_mode == MODE_BIOZ_CAL);
    dataToSerialBuff();
    checkDataTimeouts();
  }

  // DisplayUpdate
  // =======================================================
  // Full-frame SSD1306 updates are slow, so automatic refresh is disabled
  // during AFE measurement. Button presses still request a one-shot refresh.
  if (display_available) {
    const bool timed_display_update = !measurement_running &&
                                      ((current_time - last_display_us) >= display_update_interval_us);
    if (display_update_requested || timed_display_update) {
      last_display_us = current_time;
      display_update_requested = false;
      if (display_mode == DISPLAY_DATA) {
        // Measurement mode display
        drawMeasurementModeDisplay(current_mode, measurement_running);
        display.display();
      } else {
        // Battery status display
        display.clearDisplay();
        drawBatteryIndicator(BAT_X, BAT_Y, cell_percent);
        drawChargingIndicator(CHRG_X, CHRG_Y, BAT_W / 2, BAT_H - 2,
                              charge_rate_percent_per_hour, 10.0f);
        display.setCursor(BAT_X + BAT_W + 4, CHRG_Y + 3);
        if (isfinite(charge_rate_percent_per_hour)) {
          display.print(charge_rate_percent_per_hour, 1);
          display.print("%/hr");
        } else {
          display.print("rate n/a");
        }
        display.setCursor(BAT_X + BAT_W + 4, BAT_Y + 4);
        if (isfinite(cell_percent)) {
          display.print(cell_percent, 0);
          display.print("%");
        } else {
          display.print("bat n/a");
        }
        display.display();
      }
    }
  } else {
    display_update_requested = false;
  }

  // Send data in chunks over serial to avoid blocking
  // =======================================================
  serialOutUpdate();

  // Blink LED
  // =========
  if (button_state == LOW) {
    const uint32_t interval_us = led_state ? BLINK_INTERVAL_ON_US : BLINK_INTERVAL_OFF_US;
    if ((current_time - last_blink_us) >= interval_us) {
      led_state = !led_state;
      digitalWrite(LED_PIN, led_state ? HIGH : LOW);
      last_blink_us = current_time;
    }
  }

  // Small delay, to maintain loop timing and prevent overwhelming the system
  // ===========
  const uint32_t elapsed_us = micros() - current_time;
  if (elapsed_us < LOOP_INTERVAL_US) {
    delayMicroseconds(static_cast<unsigned int>(LOOP_INTERVAL_US - elapsed_us));
  }
}
