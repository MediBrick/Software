/**
 * For the MediBrick 3 Channel Thermistor Board
 * @author Urs Utzinger
 * @copyright MIT 
 * Supported boards (selected automatically from the board you compile for):
 *   - ESP32 (SparkFun Thing Plus C, WROOM): ADC up to 2 MHz, 3 thermistors
 *   - ESP32-S3 (Adafruit Feather ESP32-S3):  ADC up to 83 kHz, 2 thermistors, 3 if I2C is turned off
 *
 * OpenAI Codex and Anthropic Claude assisted development
 **/

/* ============================================================================================================================ */
// Includes and library imports
/* ============================================================================================================================ */

#include "Arduino.h"
#include <Preferences.h>
#include <RingBuffer.h>
#include <BLESerial.h>
// AudioTools has its own (different) RingBuffer in namespace audio_tools. Do not import the namespace globally,
// otherwise RingBuffer is ambiguous. The AudioTools names we use are imported individually below.
#define USE_AUDIOTOOLS_NS false
#include "AudioTools.h"
#undef LOGD
#undef LOGI
#undef LOGW
#undef LOGE
#undef LOG_LEVEL
#undef USE_AUDIO_LOGGING
#undef LOGGER_H   // BLESerial.h already included logger.h, include it again to get our LOG macros back
#include "logger.h"
#define LOG_LEVEL LOG_LEVEL_INFO
#include <Linereader.h>

// AudioTools names used in this sketch
using audio_tools::AnalogAudioStream;
using audio_tools::Bin;
using audio_tools::ChannelBinDiff;
using audio_tools::ConverterStream;
using audio_tools::AudioLogger;
using audio_tools::RX_MODE;

/* ============================================================================================================================ */
// Constants and types
/* ============================================================================================================================ */

#define BAUDRATE 500000                        // baud rate of the USB serial port (ignored by native USB)
#define BLE_NAME "MediBrickThermistor"         // default BLE device name
#define BLE_NAME_MAX 24                        // longest BLE device name the Ln command accepts

// Data outputs, bit mask in the settings
#define DATA_OUT_USB  0x01                     // send data over USB serial
#define DATA_OUT_BLE  0x02                     // send data over BLE serial
#define DATA_OUT_ALL  (DATA_OUT_USB | DATA_OUT_BLE)  // both

#define OUT_BUFFER_SIZE 8192                   // bytes per output link (power of 2), holds the lines of several ADC reads
#define OUT_CHUNK_BYTES  128                   // bytes moved from an output buffer to a link in one step

// ADC attenuation 3.9V range (11 dB or 12 dB depending on ESP32 core version)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define M_ADC_ATTENUATION ADC_ATTEN_DB_12
#else
  #define M_ADC_ATTENUATION ADC_ATTEN_DB_11
#endif
#define M_AVERAGE     true                     // Enable averaging, otherwise we sum
#define M_BIT_DEPTH     16                     // bit width of the ADC data after conversion, do not change
#define M_BYTES_PER_SAMPLE sizeof(int16_t)     // bytes per converted sample
#define M_READ_BYTES  2048                     // preferred number of ADC bytes per read, more output samples per read for small bin sizes
#define M_LINE_CHARS_PER_VALUE 8               // ", -1234" or ", 25.3", used to estimate the serial load

// Output modes
#define MODE_VOLTAGE      0                    // voltage difference of the bridge in mV
#define MODE_RESISTANCE   1                    // thermistor resistance in Ohm
#define MODE_TEMPERATURE  2                    // temperature in degrees Celsius

// Where a command came from, the answer goes back there
// (needs to be before the first function, the Arduino build generates function prototypes there)
enum class CommandSource : uint8_t { Usb, Ble };  // command received over USB serial or BLE serial

/* ============================================================================================================================ */
// System information (fixed properties of the microcontroller)
/* ============================================================================================================================ */

// Fixed properties of the microcontroller, selected by the board you compile for.
// The continuous ADC (AudioTools AnalogAudioStream) only supports ADC unit 1.
// One read from the ADC is one DMA frame, the driver limits a frame to 4092 bytes. 
// Each conversion result takes SOC_ADC_DIGI_RESULT_BYTES in the frame (ESP32: 2, ESP32-S3: 4).
// The total sampling rate (all ADC channels together) is limited by the chip.
// ADC channel numbers are NOT the same as pins, they depend on the chip, see Thermistor_Brick/assembly.md.
// Each thermistor needs two ADC channels (A and B side of the Wheatstone bridge), the board reports the voltage difference B-A.
struct SystemInfo {
  const char*  name;                   // board name
  uint8_t      adc_result_bytes;       // bytes per conversion result inside a DMA frame
  uint8_t      adc_bit_width;          // bits per conversion
  uint32_t     max_frame_bytes;        // largest DMA frame (one read)
  uint32_t     max_frame_samples;      // max_frame_bytes / adc_result_bytes
  uint32_t     min_adc_rate;           // Hz, total over all ADC channels
  uint32_t     max_adc_rate;           // Hz, total over all ADC channels
  uint8_t      max_adc_channels;       // 2 per thermistor
  uint8_t      max_adc_channels_i2c;   // ADC channels available while I2C is on (pins shared with SDA/SCL otherwise)
  uint8_t      default_adc_channels;   // ADC channels used when nothing is saved
  bool         usb_native;             // true: native USB (baud rate is ignored), false: USB-UART bridge (baud rate is the real line rate)
  uint32_t     usb_default_bps;        // conservative default for the sustainable USB payload in bytes/s while running the application
  uint32_t     ble_default_bps;        // conservative default for the sustainable BLE payload in bytes/s while running the application
  adc_channel_t adc_map[6];            // ADC channel of: Thermistor 1 A, 1 B, 2 A, 2 B, 3 A, 3 B
};

// Link throughput defaults (bytes/s of text payload)
// Measured with SerialUI test programs on an ESP32-S3 that did little else than streaming:
//   native USB: ~870 kBytes/s, BLE: ~129 kBytes/s
// The temperature application also runs the ADC and formats text, the BLE rate depends strongly on the central
// (phones/OS negotiate shorter MTU and longer connection intervals), therefore about 1/4 of the measurement is used.
//   ESP32-S3 USB:  native USB CDC, baud rate is ignored, 870 -> 200 kBytes/s
//   ESP32 USB:     USB-UART bridge (CP2102N on the Thing Plus C), the baud rate is the real line rate,
//                  10 bits per byte, 80% usable. Raise BAUDRATE to increase this.
//   BLE:           ESP32-S3 129 -> 30 kBytes/s. For the ESP32 no measurement exists, it has the same radio and
//                  a slower CPU, so use slightly less: 25 kBytes/s.
// Both defaults can be changed at runtime and saved (Lu, Lb commands).
#define SYS_MAX_FRAME_BYTES 4092               // largest ADC DMA frame of the driver in bytes
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  // Adafruit Feather ESP32-S3: only 5 ADC1 pins are free (D5, D6, A5/GPIO8, D9, D10),
  // GPIO3 and GPIO4 are ADC1 too but are used by I2C (STEMMA QT).
  // Therefore two thermistors are available, a third one needs I2C to be turned off:
  // wire it to ADC1_CH7 (A5) and ADC1_CH3 (GPIO4, SCL).
  static const SystemInfo SYS = {    // properties of this board
    "ESP32-S3", SOC_ADC_DIGI_RESULT_BYTES, 12, SYS_MAX_FRAME_BYTES, SYS_MAX_FRAME_BYTES / SOC_ADC_DIGI_RESULT_BYTES,
    SOC_ADC_SAMPLE_FREQ_THRES_LOW, SOC_ADC_SAMPLE_FREQ_THRES_HIGH,
    6, 4, 4,
    true, 200000, 30000,
    { ADC_CHANNEL_8,   // GPIO9  (D9)   Thermistor 1 A
      ADC_CHANNEL_9,   // GPIO10 (D10)  Thermistor 1 B
      ADC_CHANNEL_4,   // GPIO5  (D5)   Thermistor 2 A
      ADC_CHANNEL_5,   // GPIO6  (D6)   Thermistor 2 B
      ADC_CHANNEL_7,   // GPIO8  (A5)   Thermistor 3 A
      ADC_CHANNEL_3 }  // GPIO4  (SCL)  Thermistor 3 B
  };
#else
  // Sparkfun ESP32 Thing Plus C (ESP32 WROOM)
  static const SystemInfo SYS = {    // properties of this board
    "ESP32", SOC_ADC_DIGI_RESULT_BYTES, 12, SYS_MAX_FRAME_BYTES, SYS_MAX_FRAME_BYTES / SOC_ADC_DIGI_RESULT_BYTES,
    SOC_ADC_SAMPLE_FREQ_THRES_LOW, SOC_ADC_SAMPLE_FREQ_THRES_HIGH,
    6, 6, 6,
    false, (BAUDRATE / 10) * 8 / 10, 25000,
    { ADC_CHANNEL_5,   //    33         Thermistor 1 A
      ADC_CHANNEL_4,   //    32         Thermistor 1 B
      ADC_CHANNEL_6,   // A2 34         Thermistor 2 A
      ADC_CHANNEL_3,   // A3 39         Thermistor 2 B
      ADC_CHANNEL_0,   // A4 36         Thermistor 3 A
      ADC_CHANNEL_7 }  // A5 35         Thermistor 3 B
  };
#endif

/* ============================================================================================================================ */
// Settings: factory defaults and storage layout
// Everything here can be changed over the serial port and stored in non volatile memory (see helpMenu()).
// The DEFAULT_ values are the factory defaults, used when nothing has been saved.
/* ============================================================================================================================ */

#define DEFAULT_MODE        MODE_TEMPERATURE   // output temperature
#define DEFAULT_HALF_BRIDGE false              // full bridge: two ADC channels are subtracted, one value per thermistor
#define DEFAULT_I2C         true               // I2C bus in use
#define DEFAULT_BIN_SIZE    168                // ADC samples averaged per output sample
#define DEFAULT_OUT_RATE    50                 // output samples per thermistor per second
#define DEFAULT_ATTEN       3                  // index into ATTEN_TABLE: 0..3.9V
#define DEFAULT_DATA_OUT    DATA_OUT_USB       // data goes to USB only
#define DEFAULT_LOG_LEVEL   LOG_LEVEL_INFO     // log level

// These are the values measured on the board before it got assembled
// If you did not measure them, you can use R=10k and measure V_IN
// You still should measure TP2-TP3 resistance for R3
#define DEFAULT_V_IN        3320               // bridge supply voltage in mV

// Bridge resistors per thermistor {R1, R2, R3}
static const int32_t DEFAULT_R[3][3] = {       // in Ohm
  {  9960, 9970,  9960 },   // Thermistor 1  (R3 alternative 9940)
  {  9950, 9990,  9970 },   // Thermistor 2  (R3 alternative 9900)
  { 10000, 9980, 10030 },   // Thermistor 3  (R3 alternative 10000)
};

// From thermistor manufacturer datasheet
// See python program calculating the coefficients from manufacturer provide temperature, resistance curve
#define DEFAULT_A_COEFF 0.001111285538f        // Steinhart-Hart coefficient A
#define DEFAULT_B_COEFF 0.0002371215953f       // Steinhart-Hart coefficient B
#define DEFAULT_C_COEFF 0.00000007520676806f   // Steinhart-Hart coefficient C

#define PREF_NAMESPACE      "thermistor"       // non volatile storage namespace
#define PREF_SCHEMA_VERSION 2                  // layout version of the stored settings, stored data of another version is ignored

// | attenuation     | range   | accurate range |
// | ------------    | --------| -------------- |
// | ADC_ATTEN_DB_0  | 0..1.1V | 100-950mV      |
// | ADC_ATTEN_DB_2_5| 0..1.5V | 100-1250mV     |
// | ADC_ATTEN_DB_6  | 0..2.2V | 150-1750mV     |
// | ADC_ATTEN_DB_12 | 0..3.9V | 150-2450mV     | * depends on resistors

struct Settings {                // all settings that can be changed by the user
  uint8_t  mode;             // MODE_VOLTAGE, MODE_RESISTANCE or MODE_TEMPERATURE
  uint8_t  channels;         // ADC channels: 2, 4 or 6 (= 1, 2 or 3 thermistors)
  bool     half_bridge;      // false: channel pairs are subtracted (one value per thermistor), true: every ADC channel is output as voltage
  bool     i2c;              // true: I2C bus is in use, its pins are not available as ADC inputs
  uint16_t out_rate;         // output samples per thermistor per second
  uint16_t bin_size;         // ADC samples averaged per output sample
  uint32_t usb_bps;          // sustainable USB payload in bytes/s, limits the output rate
  uint32_t ble_bps;          // sustainable BLE payload in bytes/s, limits the output rate when BLE output is on
  uint8_t  data_out;         // DATA_OUT_USB and/or DATA_OUT_BLE, the limit of every enabled link applies
  uint8_t  log_level;        // LOG_LEVEL_DEBUG .. LOG_LEVEL_NONE
  char     ble_name[BLE_NAME_MAX + 1];   // BLE device name, applied at boot
  uint32_t serial_baud;      // baud rate of the USB serial port, applied at boot (ignored by native USB)
  uint8_t  atten;            // index into ATTEN_TABLE
  int32_t  v_in;             // bridge supply in mV
  int32_t  r[3][3];          // bridge resistors per thermistor
  float    sh[3];            // Steinhart-Hart A, B, C
};

struct AttenEntry { uint8_t value; const char* range; };   // ADC attenuation setting and the input range it gives
static const AttenEntry ATTEN_TABLE[4] = {                 // selectable attenuations, index is the g command
  { ADC_ATTEN_DB_0,   "0..1.1V"  },
  { ADC_ATTEN_DB_2_5, "0..1.5V"  },
  { ADC_ATTEN_DB_6,   "0..2.2V"  },
  { M_ADC_ATTENUATION, "0..3.9V" },   // 11/12 dB depending on the ESP32 core version
};

/* ============================================================================================================================ */
// Global variables and objects
/* ============================================================================================================================ */

// State
Settings settings;                          // current settings
bool     measurement_running = false;       // ADC running and data being printed
bool     adc_running = false;               // ADC driver is started
bool     data_reporting = true;             // data lines are sent to the enabled links, otherwise the ADC data is discarded

// Buffer geometry, depends on settings, recalculated by applyAdc()
size_t   buffer_factor = 1;                 // output samples per read
size_t   adc_samples   = 0;                 // total ADC samples (not bytes) per read
size_t   buffer_bytes  = 0;                 // bytes requested per read

// ADC and conversion
AnalogAudioStream        analog_in;         // On board analog to digital converter
ChannelBinDiff           bindiffer;         // Binning each channel by average length, setup see applyAdc()
Bin                      binner;            // Binning each channel without difference (half bridge), setup see applyAdc()
// Pipe the analog input to the binnning and differ converter (full bridge) or to the binning converter (half bridge)
ConverterStream<int16_t> converted_stream_full(analog_in, bindiffer);
ConverterStream<int16_t> converted_stream_half(analog_in, binner);
ConverterStream<int16_t>& convertedStream() { return settings.half_bridge ? converted_stream_half : converted_stream_full; }
// converted samples of one read, largest possible read
int16_t process_buffer[SYS_MAX_FRAME_BYTES / SOC_ADC_DIGI_RESULT_BYTES]; 
char    out_text_buffer[64];                // one output line while it is formatted

// Command input
BLESerial       ble;                        // BLE serial interface
char            serial_input_buff[128];     // command received over USB serial
char            ble_input_buff[128];        // command received over BLE serial
LineReader<128> serial_lr;                  // collects USB serial characters to a command line
LineReader<128> ble_lr;                     // collects BLE serial characters to a command line
const char      waitmsg[] = {"Waiting for serial terminal (5 seconds, press Enter to skip)"};

// Data output
RingBuffer<char, OUT_BUFFER_SIZE> usb_out_buff; // output lines waiting for the USB link
RingBuffer<char, OUT_BUFFER_SIZE> ble_out_buff; // output lines waiting for the BLE link
uint32_t usb_lines_dropped = 0;             // lines lost because the USB buffer was full
uint32_t ble_lines_dropped = 0;             // lines lost because the BLE buffer was full

/* ============================================================================================================================ */
// Limits derived from system info and settings
/* ============================================================================================================================ */

// The output rate is limited by the data link (USB, and BLE if enabled, the lower one counts) and by the ADC.
// For a selected output rate the bin size then has to be chosen so that
//   channels * out_rate * bin_size is within min_adc_rate..max_adc_rate    (ADC rate)
//   channels * bin_size fits in one DMA frame                              (frame size)
// A high output rate therefore needs a small bin size, a low output rate a large one.

// Characters of one output line, estimated
// Values per output line: one per thermistor in full bridge mode, one per ADC channel in half bridge mode
uint32_t valuesPerLine(const Settings& s) { return s.half_bridge ? s.channels : s.channels / 2; }
uint32_t serialLineChars(const Settings& s) { return valuesPerLine(s) * M_LINE_CHARS_PER_VALUE + 2; }

// Name of the link that limits the output rate
const char* linkName(const Settings& s) {
  const bool usb_on = s.data_out & DATA_OUT_USB, ble_on = s.data_out & DATA_OUT_BLE;
  if (usb_on && ble_on) return (s.ble_bps < s.usb_bps) ? "BLE" : "USB";
  if (ble_on) return "BLE";
  if (usb_on) return "USB";
  return "no output enabled, no";
}

// Lines per second the enabled links can carry
uint32_t maxLinesPerSecond(const Settings& s) {
  uint32_t bps = 0xFFFFFFFFUL;                                      // no link enabled: no limit
  if ((s.data_out & DATA_OUT_USB) && s.usb_bps < bps) bps = s.usb_bps;   // by_serial_USB
  if ((s.data_out & DATA_OUT_BLE) && s.ble_bps < bps) bps = s.ble_bps;   // by_serial_BLE
  return bps / serialLineChars(s);
}

// Largest output rate for the selected channels and links: the ADC at bin size 1, or the data link
uint32_t maxOutRate(const Settings& s) {
  return min(SYS.max_adc_rate / s.channels, maxLinesPerSecond(s));
}
// Smallest output rate for the selected channels: the ADC must reach its minimum rate with the largest bin that fits a frame
uint32_t minOutRate(uint8_t channels) {
  uint32_t denom = channels * (SYS.max_frame_samples / channels);
  return max<uint32_t>(1, (SYS.min_adc_rate + denom - 1) / denom);
}
// Largest bin size for the selected channels and output rate
uint32_t maxBinSize(uint8_t channels, uint32_t out_rate) {
  uint32_t by_frame = SYS.max_frame_samples / channels;
  uint32_t by_rate  = SYS.max_adc_rate / (channels * out_rate);
  return min(by_frame, by_rate);
}
// Smallest bin size for the selected channels and output rate
uint32_t minBinSize(uint8_t channels, uint32_t out_rate) {
  uint32_t denom = channels * out_rate;
  return max<uint32_t>(1, (SYS.min_adc_rate + denom - 1) / denom);
}

// Checks everything except the bin size. Returns nullptr if fine, otherwise the reason.
const char* rateError(const Settings& s) {
  if (s.channels != 2 && s.channels != 4 && s.channels != 6) return "channels must be 2, 4 or 6";
  if (s.channels > SYS.max_adc_channels)                     return "this board does not have that many ADC channels";
  if (s.i2c && s.channels > SYS.max_adc_channels_i2c)        return "this many channels need I2C turned off (w0)";
  if (s.atten > 3)                                           return "attenuation must be 0..3";
  if (s.mode > MODE_TEMPERATURE)                             return "output mode must be 0..2";
  if (s.data_out > DATA_OUT_ALL)                             return "data output must be 0..3";
  if (s.out_rate > maxOutRate(s))                            return "output rate is too high (ADC, USB or BLE limit)";
  if (s.out_rate < minOutRate(s.channels))                   return "output rate is too low for the ADC of this board";
  return nullptr;
}

// Checks everything. Returns nullptr if the settings can run, otherwise the reason they can not.
const char* configError(const Settings& s) {
  const char* err = rateError(s);
  if (err) return err;
  if (s.bin_size > maxBinSize(s.channels, s.out_rate))       return "bin size is too large for this output rate and channel count";
  if (s.bin_size < minBinSize(s.channels, s.out_rate))       return "bin size is too small for this output rate and channel count";
  return nullptr;
}

// Moves the bin size into the allowed range for the selected channels and output rate. Call only if rateError() is clean.
void fitBinSize(Settings& s) {
  s.bin_size = constrain(s.bin_size, minBinSize(s.channels, s.out_rate), maxBinSize(s.channels, s.out_rate));
}

void printLimits(const Settings& s) {
  LOGln("  Output rate %lu..%lu Hz at %u channels (%s limit: %lu lines/s)",
        (unsigned long)minOutRate(s.channels), (unsigned long)maxOutRate(s), s.channels,
        linkName(s), (unsigned long)maxLinesPerSecond(s));
  if (rateError(s)) return;   // no bin size range without a valid rate
  LOGln("  Bin size    %lu..%lu at %u Hz output rate", 
        (unsigned long)minBinSize(s.channels, s.out_rate), (unsigned long)maxBinSize(s.channels, s.out_rate), s.out_rate);
}

/* ============================================================================================================================ */
// Settings handling
/* ============================================================================================================================ */

void setDefaults(Settings& s) {
  s.mode     = DEFAULT_MODE;
  s.channels = SYS.default_adc_channels;
  s.half_bridge = DEFAULT_HALF_BRIDGE;
  s.i2c      = DEFAULT_I2C;
  s.out_rate = DEFAULT_OUT_RATE;
  s.bin_size = DEFAULT_BIN_SIZE;
  s.usb_bps  = SYS.usb_default_bps;
  s.ble_bps  = SYS.ble_default_bps;
  s.data_out = DEFAULT_DATA_OUT;
  s.log_level = DEFAULT_LOG_LEVEL;
  strlcpy(s.ble_name, BLE_NAME, sizeof(s.ble_name));
  s.serial_baud = BAUDRATE;
  s.atten    = DEFAULT_ATTEN;
  s.v_in     = DEFAULT_V_IN;
  for (int ch = 0; ch < 3; ch++) for (int i = 0; i < 3; i++) s.r[ch][i] = DEFAULT_R[ch][i];
  s.sh[0] = DEFAULT_A_COEFF; s.sh[1] = DEFAULT_B_COEFF; s.sh[2] = DEFAULT_C_COEFF;
}

// Try new settings. When stopped they take effect at the next start, otherwise the ADC is restarted.
// With fit_bin the bin size is moved into the allowed range if the new rate, channels or link limits need it,
// otherwise the bin size is checked as given. On failure the previous settings stay in place.
bool applySettings(Settings candidate, bool fit_bin = true) {
  const char* err = fit_bin ? rateError(candidate) : configError(candidate);
  if (!err && fit_bin) {
    const uint16_t wanted = candidate.bin_size;
    fitBinSize(candidate);
    if (candidate.bin_size != wanted) LOGln("Bin size changed from %u to %u to fit the output rate", wanted, candidate.bin_size);
    err = configError(candidate);
  }
  if (err) {
    LOGln("Not changed: %s", err);
    printLimits(candidate);
    return false;
  }
  const Settings previous = settings;
  settings = candidate;
  const bool adc_changed = previous.channels != candidate.channels || previous.half_bridge != candidate.half_bridge || previous.out_rate != candidate.out_rate ||
                           previous.bin_size != candidate.bin_size || previous.atten != candidate.atten;
  if (measurement_running && adc_changed && !startMeasurement()) {
    LOGE("ADC restart failed, restoring previous settings");
    settings = previous;
    startMeasurement();
    return false;
  }
  return true;
}

// Non volatile storage
// --------------------
bool saveSettings() {
  Preferences prefs;
  if (!prefs.begin(PREF_NAMESPACE, false)) { LOGW("Preferences save failed: namespace open failed"); return false; }
  prefs.putUShort("schema", PREF_SCHEMA_VERSION);
  prefs.putUChar("mode", settings.mode);
  prefs.putUChar("chan", settings.channels);
  prefs.putBool("hb", settings.half_bridge);
  prefs.putBool("i2c", settings.i2c);
  prefs.putUShort("rate", settings.out_rate);
  prefs.putUShort("bin", settings.bin_size);
  prefs.putUChar("atten", settings.atten);
  prefs.putUInt("usb_bps", settings.usb_bps);
  prefs.putUInt("ble_bps", settings.ble_bps);
  prefs.putUChar("dout", settings.data_out);
  prefs.putUChar("loglvl", settings.log_level);
  prefs.putString("ble_name", settings.ble_name);
  prefs.putUInt("baud", settings.serial_baud);
  prefs.putInt("vin", settings.v_in);
  char key[8];
  for (int ch = 0; ch < 3; ch++) for (int i = 0; i < 3; i++) {
    snprintf(key, sizeof(key), "r%d%d", ch + 1, i + 1);
    prefs.putInt(key, settings.r[ch][i]);
  }
  prefs.putFloat("sh_a", settings.sh[0]);
  prefs.putFloat("sh_b", settings.sh[1]);
  prefs.putFloat("sh_c", settings.sh[2]);
  prefs.end();
  return true;
}

// Reads the stored settings into s (s should hold defaults), returns false if nothing valid is stored
bool readSettings(Settings& s) {
  Preferences prefs;
  if (!prefs.begin(PREF_NAMESPACE, true)) return false;   // namespace does not exist yet
  if (prefs.getUShort("schema", 0) != PREF_SCHEMA_VERSION) { prefs.end(); return false; }
  s.mode     = prefs.getUChar("mode", s.mode);
  s.channels = prefs.getUChar("chan", s.channels);
  s.half_bridge = prefs.getBool("hb", s.half_bridge);
  s.i2c      = prefs.getBool("i2c", s.i2c);
  s.out_rate = prefs.getUShort("rate", s.out_rate);
  s.bin_size = prefs.getUShort("bin", s.bin_size);
  s.atten    = prefs.getUChar("atten", s.atten);
  s.usb_bps  = prefs.getUInt("usb_bps", s.usb_bps);
  s.ble_bps  = prefs.getUInt("ble_bps", s.ble_bps);
  s.data_out = prefs.getUChar("dout", s.data_out);
  s.log_level = prefs.getUChar("loglvl", s.log_level);
  if (prefs.isKey("ble_name")) prefs.getString("ble_name", s.ble_name, sizeof(s.ble_name));
  s.serial_baud = prefs.getUInt("baud", s.serial_baud);
  if (s.serial_baud < 9600 || s.serial_baud > 2000000) s.serial_baud = BAUDRATE;
  s.v_in     = prefs.getInt("vin", s.v_in);
  char key[8];
  for (int ch = 0; ch < 3; ch++) for (int i = 0; i < 3; i++) {
    snprintf(key, sizeof(key), "r%d%d", ch + 1, i + 1);
    s.r[ch][i] = prefs.getInt(key, s.r[ch][i]);
  }
  s.sh[0] = prefs.getFloat("sh_a", s.sh[0]);
  s.sh[1] = prefs.getFloat("sh_b", s.sh[1]);
  s.sh[2] = prefs.getFloat("sh_c", s.sh[2]);
  prefs.end();
  if (!rateError(s)) fitBinSize(s);
  if (configError(s)) {   // stored acquisition setup not usable on this board, use the defaults for it
    Settings d; setDefaults(d);
    s.channels = d.channels; s.half_bridge = d.half_bridge; s.i2c = d.i2c; s.out_rate = d.out_rate; s.bin_size = d.bin_size; s.atten = d.atten;
    s.usb_bps = d.usb_bps; s.ble_bps = d.ble_bps; s.data_out = d.data_out;
  }
  return true;
}

bool clearSettings() {
  Preferences prefs;
  if (!prefs.begin(PREF_NAMESPACE, false)) return false;
  prefs.clear();
  prefs.end();
  return true;
}

/* ============================================================================================================================ */
// Logging
/* ============================================================================================================================ */

// Logger level: ours and the one of the AudioTools library (which has no "none" and "critical")
void applyLogLevel(int level) {
  logSetLevel(level);
  AudioLogger::LogLevel audio_level = AudioLogger::Error;
  if      (level <= LOG_LEVEL_DEBUG) audio_level = AudioLogger::Debug;
  else if (level == LOG_LEVEL_INFO)  audio_level = AudioLogger::Info;
  else if (level == LOG_LEVEL_WARN)  audio_level = AudioLogger::Warning;
  AudioLogger::instance().setLogLevel(audio_level);
}

/* ============================================================================================================================ */
// ADC configuration and start / stop
/* ============================================================================================================================ */

// (Re)start the continuous ADC with the current settings.
// Channels, rate, bin size and attenuation are only read by the driver at begin(), so changing them at runtime means end() followed by begin().
bool applyAdc() {
  if (configError(settings)) return false;

  if (adc_running) { analog_in.end(); adc_running = false; }

  const uint32_t ch  = settings.channels;
  const uint32_t bin = settings.bin_size;

  // Several output samples per read, but one read (= one DMA frame) must stay below the driver limit
  buffer_factor = M_READ_BYTES / (ch * bin * M_BYTES_PER_SAMPLE);
  const size_t frame_limit = SYS.max_frame_samples / (ch * bin);
  if (buffer_factor > frame_limit) buffer_factor = frame_limit;
  if (buffer_factor < 1) buffer_factor = 1;
  adc_samples  = ch * bin * buffer_factor;
  buffer_bytes = adc_samples * M_BYTES_PER_SAMPLE;

  auto adcConfig = analog_in.defaultConfig(RX_MODE);
  adcConfig.sample_rate = settings.out_rate * bin;      // sample rate per channel before decimating
  adcConfig.channels = ch;                              // 2 ADC channels per thermistor
  adcConfig.adc_bit_width = SYS.adc_bit_width;          // Supports only 12
  adcConfig.buffer_size = adc_samples;                  // in total number of samples
  adcConfig.adc_calibration_active = true;              // use and apply the calibration settings of ADC stored in ESP32
  adcConfig.is_auto_center_read = false;                // do not engage auto-centering of signal (would subtract average of signal)
  adcConfig.adc_attenuation = ATTEN_TABLE[settings.atten].value; // set analog input range
  adcConfig.timeout = 0;                                // reads return what is available, the main loop keeps the output links moving
  for (uint32_t i = 0; i < ch; i++) adcConfig.adc_channels[i] = SYS.adc_map[i];

  if (!analog_in.begin(adcConfig)) return false;
  adc_running = true;

  bindiffer.setChannels(ch);
  bindiffer.setBits(M_BIT_DEPTH);
  bindiffer.setBinSize(bin);                            // also clears the partial bin
  bindiffer.setAverage(M_AVERAGE);
  binner.setChannels(ch);
  binner.setBits(M_BIT_DEPTH);
  binner.setBinSize(bin);
  binner.setAverage(M_AVERAGE);
  return true;
}

void stopMeasurement() {
  if (adc_running) { analog_in.end(); adc_running = false; }
  measurement_running = false;
  usb_out_buff.clear();
  ble_out_buff.clear();
}

bool startMeasurement() {
  measurement_running = applyAdc();
  return measurement_running;
}

/* ============================================================================================================================ */
// Data output
// Output lines are formatted once and copied into one ring buffer per link, so a slow or absent link
// (e.g. no BLE client) can not stall the other one. The main loop moves the buffered bytes to the links
// without blocking. If a link can not keep up its buffer fills and complete lines are dropped (and counted).
/* ============================================================================================================================ */

// Place one complete line (including CR LF) in the buffers of the enabled links
void queueLine(const char* line, size_t len) {
  if (settings.data_out & DATA_OUT_USB) {
    if (usb_out_buff.push(line, len) != len) usb_lines_dropped++;       // push is all or nothing
  }
  if ((settings.data_out & DATA_OUT_BLE) && ble.isSubscribed()) {
    if (ble_out_buff.push(line, len) != len) ble_lines_dropped++;
  }
}

// Move buffered bytes to the links, as much as they accept right now
void drainUsb() {
  if (!(settings.data_out & DATA_OUT_USB)) { usb_out_buff.clear(); return; }
  size_t room = Serial.availableForWrite();
  char chunk[OUT_CHUNK_BYTES];
  while (room > 0 && usb_out_buff.available() > 0) {
    size_t m = usb_out_buff.peek(chunk, min(room, sizeof(chunk)));
    size_t w = Serial.write((const uint8_t*)chunk, m);
    usb_out_buff.consume(w);
    room -= w;
    if (w < m) break;
  }
}

void drainBle() {
  if (!(settings.data_out & DATA_OUT_BLE) || !ble.isSubscribed()) { ble_out_buff.clear(); return; }
  if (!ble.writeReady()) return;
  size_t room = ble.writeAvailable();
  char chunk[OUT_CHUNK_BYTES];
  while (room > 0 && ble_out_buff.available() > 0) {
    size_t m = ble_out_buff.peek(chunk, min(room, sizeof(chunk)));
    size_t w = ble.writeNonBlocking((const uint8_t*)chunk, m);
    ble_out_buff.consume(w);
    room -= w;
    if (w < m) break;
  }
}

/* ============================================================================================================================ */
// Information display (system info, settings, help)
/* ============================================================================================================================ */

const char* modeName(uint8_t mode) {
  switch (mode) {
    case MODE_VOLTAGE:     return "voltage difference in mV";
    case MODE_RESISTANCE:  return "thermistor resistance in Ohm";
    default:               return "temperature in C";
  }
}

void printSystemInfo() {
  LOGln("System: %s", SYS.name);
  LOGln("  Bits per conversion       : %u", SYS.adc_bit_width);
  LOGln("  Bytes per conversion result: %u", SYS.adc_result_bytes);
  LOGln("  Max frame                 : %lu bytes = %lu samples", (unsigned long)SYS.max_frame_bytes, (unsigned long)SYS.max_frame_samples);
  LOGln("  Total ADC rate            : %lu..%lu Hz", (unsigned long)SYS.min_adc_rate, (unsigned long)SYS.max_adc_rate);
  LOGln("  ADC channels              : max %u, max %u with I2C on", SYS.max_adc_channels, SYS.max_adc_channels_i2c);
  LOGln("  USB                       : %s, default %lu bytes/s", SYS.usb_native ? "native, baud rate ignored" : "UART bridge", (unsigned long)SYS.usb_default_bps);
  LOGln("  BLE                       : default %lu bytes/s", (unsigned long)SYS.ble_default_bps);
  LOGln("  Text line                 : about %lu chars at the selected channels", (unsigned long)serialLineChars(settings));
  LOGln("Limits for the current settings:");
  printLimits(settings);
  LOGln("Outputs:");
  LOGln("  USB: %s, %u of %u bytes buffered, %lu lines dropped", (settings.data_out & DATA_OUT_USB) ? "on" : "off",
        (unsigned)usb_out_buff.available(), (unsigned)usb_out_buff.capacity(), (unsigned long)usb_lines_dropped);
  LOGln("  BLE: %s, %s, %s, %u of %u bytes buffered, %lu lines dropped", (settings.data_out & DATA_OUT_BLE) ? "on" : "off",
        ble.isConnected() ? "connected" : "not connected", ble.isSubscribed() ? "subscribed" : "not subscribed",
        (unsigned)ble_out_buff.available(), (unsigned)ble_out_buff.capacity(), (unsigned long)ble_lines_dropped);
}

void printSettings(const Settings& s) {
  LOGln("  Bridge           : %s (b)", s.half_bridge ? "half bridge, every ADC channel as voltage in mV" : "full bridge, channel pairs subtracted");
  LOGln("  Output           : %s (m)%s", modeName(s.mode), s.half_bridge ? ", not used in half bridge mode" : "");
  LOGln("  ADC channels     : %u = %u thermistors (c), I2C %s (w)", s.channels, s.channels / 2, s.i2c ? "on" : "off");
  LOGln("  Output rate      : %u Hz per thermistor (f)", s.out_rate);
  LOGln("  Bin size         : %u samples averaged (n), ADC rate %lu Hz", s.bin_size, (unsigned long)s.channels * s.out_rate * s.bin_size);
  LOGln("  Attenuation      : %u = %s (g)", s.atten, ATTEN_TABLE[s.atten].range);
  LOGln("  Data output      : USB %s, BLE %s (z)", (s.data_out & DATA_OUT_USB) ? "on" : "off", (s.data_out & DATA_OUT_BLE) ? "on" : "off");
  LOGln("  USB limit        : %lu bytes/s (Lu)", (unsigned long)s.usb_bps);
  LOGln("  BLE limit        : %lu bytes/s (Lb)", (unsigned long)s.ble_bps);
  LOGln("  Serial baud rate : %lu (Ls, used after reboot)", (unsigned long)s.serial_baud);
  LOGln("  BLE name         : %s (Ln, used after reboot)", s.ble_name);
  LOGln("  Log level        : %u (l) 0 debug, 1 info, 2 warning, 3 error, 4 critical, 5 none", s.log_level);
  LOGln("  Vin              : %ld mV (v)", (long)s.v_in);
  for (int ch = 0; ch < s.channels / 2; ch++) {
    LOGln("  Thermistor %d R1-3: %ld, %ld, %ld Ohm (r)", ch + 1, (long)s.r[ch][0], (long)s.r[ch][1], (long)s.r[ch][2]);
  }
  LOGln("  Steinhart-Hart   : A=%.10e B=%.10e C=%.10e (h)", s.sh[0], s.sh[1], s.sh[2]);
}

void printDataOutputSettings() {
  LOGln("Continuous data display: %s | USB: %s | BLE: %s", data_reporting ? "ON" : "OFF",
        (settings.data_out & DATA_OUT_USB) ? "ON" : "OFF", (settings.data_out & DATA_OUT_BLE) ? "ON" : "OFF");
}

void printSettingsHeader() {
  LOGln("================================================================================");
  LOGln("Current Settings:");
  LOGln("================================================================================");
  LOGln("Board: %s | Running: %s", SYS.name, measurement_running ? "YES" : "NO");
  printDataOutputSettings();
  LOGln("--------------------------------------------------------------------------------");
  printSettings(settings);
  LOGln("================================================================================");
}

void helpMenu() {
  LOGln("================================================================================");
  LOGln("| MediBrick 3 Channel Thermistor Board                                         |");
  LOGln("================================================================================");
  LOGln("| GENERAL COMMANDS                       | DATA COMMANDS                       |");
  LOGln("|----------------------------------------|-------------------------------------|");
  LOGln("| ?: help screen                         | z: toggle data reporting on/off     |");
  LOGln("| s: show current settings               | z[s|b|a]: USB, BLE, both, off       |");
  LOGln("| i: show system info and limits         | z[S|B|A]: USB, BLE, both, on        |");
  LOGln("| x: reboot                              |                                     |");
  LOGln("|========================================|=====================================|");
  LOGln("| PERSISTENCE                            | START/STOP                          |");
  LOGln("|----------------------------------------|-------------------------------------|");
  LOGln("| Ps: save NVS settings                  | .: toggle start/stop                |");
  LOGln("| Pl: load NVS settings                  | >: start measurement                |");
  LOGln("| Pd: print NVS settings                 | <: stop measurement                 |");
  LOGln("| Pc: clear NVS settings                 |                                     |");
  LOGln("| Pr: reset to factory defaults          |                                     |");
  LOGln("|========================================|=====================================|");
  LOGln("| MEASUREMENT (EXPERIMENT)               | CALIBRATION                         |");
  LOGln("|----------------------------------------|-------------------------------------|");
  LOGln("| m<0-2>: 0 mV, 1 Ohm, 2 temp C          | v<mV>: bridge supply voltage        |");
  LOGln("| f<Hz>: output rate per thermistor      | r<th>,<R1>,<R2>,<R3>: resistors     |");
  LOGln("| n<bin>: no. of samples averaged        | h<A>,<B>,<C>: Steinhart-Hart coeffs |");
  LOGln("| b<0|1>: 0 full bridge, 1 half bridge   |                                     |");
  LOGln("|========================================|=====================================|");
  LOGln("| ENGINEERING: ADC                       | ENGINEERING: LINKS AND LOG          |");
  LOGln("|----------------------------------------|-------------------------------------|");
  LOGln("| c<2|4|6>: ADC channels (2 per resistor)| Lu<B/s>: USB data rate limit        |");
  LOGln("| w<0|1>: I2C off/on (S3 6 ch: off)      | Lb<B/s>: BLE data rate limit        |");
  LOGln("| g<0-3>: range 0 to 1.1, 1.5, 2.2, 3.9 V| l<0-5>: log level 0 debug .. 5 none |");
  LOGln("|                                        | Ln<name>: BLE name (needs Ps, x)    |");
  LOGln("|                                        | Ls<baud>: serial baud (needs Ps, x) |");
  LOGln("================================================================================");
  LOGln("");
  LOGln("Examples:");
  LOGln("  m2       - Output temperature");
  LOGln("  n100     - Average 100 ADC samples for each output sample");
  LOGln("  f50      - 50 output samples per second");
  LOGln("  .        - Start/stop measurement");
  LOGln("  z        - Toggle continuous data display");
  LOGln("  r1,9960,9970,9960   - Bridge resistors of thermistor 1");
  LOGln("  h1.1e-3,2.37e-4,7.5e-8 - Steinhart-Hart coefficients");
  LOGln("c, w, g, f and n restart the ADC and are checked against the limits shown by i. f, c, z, Lu, Lb move the bin size into range if needed.");
  LOGln("Changes are lost on reboot unless saved with Ps.");
  LOGln("");
}

// Parse up to n numbers separated by commas/spaces, returns the number parsed
int parseNumbers(const char* p, double* out, int n) {
  int count = 0;
  while (count < n) {
    while (*p == ' ' || *p == ',' || *p == '\t') p++;
    if (*p == '\0') break;
    char* end;
    double v = strtod(p, &end);
    if (end == p) return -1;   // not a number
    out[count++] = v;
    p = end;
  }
  while (*p == ' ' || *p == '\t') p++;
  return (*p == '\0') ? count : -1;   // trailing garbage is an error
}

void handleGeneralCommand(char c) {
  switch (c) {
    case '?': helpMenu(); break;
    case 's': printSettingsHeader(); break;
    case 'i': printSystemInfo(); break;
    case 'z': {   // toggle data display
      data_reporting = !data_reporting;
      if (data_reporting && settings.data_out == 0) {   // no link selected: use all, or USB only if the BLE rate limit does not allow it
        Settings next = settings;
        next.data_out = DATA_OUT_ALL;
        if (!applySettings(next)) { next.data_out = DATA_OUT_USB; applySettings(next); }
      }
      printDataOutputSettings();
      break;
    }
    case '.':
      if (measurement_running) { stopMeasurement(); LOGln("Stopped"); }
      else                     { LOGln(startMeasurement() ? "Started" : "Start failed, ADC configuration error"); }
      break;
    case '>':
      if (measurement_running) LOGln("Already running");
      else                     LOGln(startMeasurement() ? "Started" : "Start failed, ADC configuration error");
      break;
    case '<':
      if (measurement_running) { stopMeasurement(); LOGln("Stopped"); }
      else                     LOGln("Already stopped");
      break;
    case 'x': LOGln("Rebooting"); Serial.flush(); ESP.restart(); break;
    default:  LOGln("Unknown command '%c', ? for help", c); break;
  }
}

void handleParameterCommand(const char* cmd) {
  const char* arg = cmd + 1;
  double v[4];
  Settings next = settings;   // candidate for settings that need checking against the limits

  switch (cmd[0]) {
    case 'm': {
      if (parseNumbers(arg, v, 1) != 1 || v[0] < MODE_VOLTAGE || v[0] > MODE_TEMPERATURE) { LOGln("Usage: m0 (voltage in mV), m1 (resistance in Ohm) or m2 (temperature in C)"); return; }
      settings.mode = (uint8_t)v[0];
      LOGln("Output: %s", modeName(settings.mode));
      if (settings.half_bridge) LOGln("Half bridge mode (b1) always outputs ADC channel voltages in mV");
      break;
    }
    case 'b': {
      if (parseNumbers(arg, v, 1) != 1 || (v[0] != 0 && v[0] != 1)) { LOGln("Usage: b0 (full bridge, channel pairs subtracted) or b1 (half bridge, every ADC channel as voltage)"); return; }
      next.half_bridge = v[0] == 1;
      if (applySettings(next)) { LOGln("%s bridge, %lu values per line", settings.half_bridge ? "Half" : "Full", (unsigned long)valuesPerLine(settings)); printLimits(settings); }
      break;
    }
    case 'c': {
      if (parseNumbers(arg, v, 1) != 1 || (v[0] != 2 && v[0] != 4 && v[0] != 6)) { LOGln("Usage: c2, c4 or c6 (ADC channels, 2 per thermistor)"); return; }
      next.channels = (uint8_t)v[0];
      if (applySettings(next)) { LOGln("ADC channels %u (%u thermistors)", settings.channels, settings.channels / 2); printLimits(settings); }
      break;
    }
    case 'w': {
      if (parseNumbers(arg, v, 1) != 1 || (v[0] != 0 && v[0] != 1)) { LOGln("Usage: w0 (I2C off) or w1 (I2C on)"); return; }
      next.i2c = (v[0] == 1);
      if (applySettings(next)) LOGln("I2C %s", settings.i2c ? "on" : "off");
      break;
    }
    case 'f': {
      if (parseNumbers(arg, v, 1) != 1 || v[0] < 1 || v[0] > 65535) { LOGln("Usage: f<Hz>"); return; }
      next.out_rate = (uint16_t)v[0];
      if (applySettings(next)) { LOGln("Output rate %u Hz, bin size %u, ADC rate %lu Hz", settings.out_rate, settings.bin_size, (unsigned long)settings.channels * settings.out_rate * settings.bin_size); printLimits(settings); }
      break;
    }
    case 'n': {
      if (parseNumbers(arg, v, 1) != 1 || v[0] < 1 || v[0] > 65535) { LOGln("Usage: n<bin>"); return; }
      next.bin_size = (uint16_t)v[0];
      if (applySettings(next, false)) LOGln("Bin size %u, ADC rate %lu Hz", settings.bin_size, (unsigned long)settings.channels * settings.out_rate * settings.bin_size);
      break;
    }
    case 'z': {   // data output links
      uint8_t mask = settings.data_out;
      bool report = data_reporting;
      if (cmd[2] != '\0') { LOGln("Data reporting commands must be exactly two characters"); return; }
      switch (cmd[1]) {
        case 's': mask &= (uint8_t)~DATA_OUT_USB; break;                 // USB off
        case 'b': mask &= (uint8_t)~DATA_OUT_BLE; break;                 // BLE off
        case 'a': mask = 0; break;                                       // all off
        case 'S': mask |= DATA_OUT_USB; report = true; break;            // USB on
        case 'B': mask |= DATA_OUT_BLE; report = true; break;            // BLE on
        case 'A': mask = DATA_OUT_ALL;  report = true; break;            // both on
        default:  LOGln("Unknown data reporting command, z[s|b|a] switch off, z[S|B|A] switch on USB, BLE, all"); return;
      }
      if (mask == 0) report = false;
      next.data_out = mask;
      if (applySettings(next)) { data_reporting = report; printDataOutputSettings(); printLimits(settings); }
      break;
    }
    case 'L': {
      if (cmd[1] == 's') {   // serial baud rate, applied at next boot
        if (parseNumbers(cmd + 2, v, 1) != 1 || v[0] < 9600 || v[0] > 2000000) { LOGln("Usage: Ls<baud>, 9600..2000000"); return; }
        settings.serial_baud = (uint32_t)v[0];
        LOGln("Serial baud rate %lu, save with Ps and reboot with x to apply, then reconnect the terminal at the new rate", (unsigned long)settings.serial_baud);
        break;
      }
      if (cmd[1] == 'n') {   // BLE name, kept as typed (case and inner spaces), applied at next boot
        const char* name = cmd + 2;
        while (*name == ' ' || *name == '\t') name++;
        const size_t n = strlen(name);
        bool ok = n > 0 && n <= BLE_NAME_MAX;
        for (size_t i = 0; ok && i < n; i++) ok = (name[i] >= 0x20 && name[i] < 0x7F);
        if (!ok) { LOGln("Usage: Ln<name>, 1..%d printable ASCII characters", BLE_NAME_MAX); return; }
        strlcpy(settings.ble_name, name, sizeof(settings.ble_name));
        LOGln("BLE name '%s', save with Ps and reboot with x to apply", settings.ble_name);
        break;
      }
      const bool usb = (cmd[1] == 'u');
      if ((!usb && cmd[1] != 'b') || parseNumbers(cmd + 2, v, 1) != 1 || v[0] < 500 || v[0] > 4000000) { LOGln("Usage: Lu<bytes/s> or Lb<bytes/s>, 500..4000000"); return; }
      if (usb) next.usb_bps = (uint32_t)v[0]; else next.ble_bps = (uint32_t)v[0];
      if (applySettings(next)) { LOGln("%s limit %lu bytes/s", usb ? "USB" : "BLE", (unsigned long)(usb ? settings.usb_bps : settings.ble_bps)); printLimits(settings); }
      break;
    }
    case 'l': {
      if (parseNumbers(arg, v, 1) != 1 || v[0] < LOG_LEVEL_DEBUG || v[0] > LOG_LEVEL_NONE) { LOGln("Usage: l<0-5>  0 debug, 1 info, 2 warning, 3 error, 4 critical, 5 none"); return; }
      settings.log_level = (uint8_t)v[0];
      applyLogLevel(settings.log_level);
      switch (settings.log_level) {
        case 0:
          LOGln("Log level DEBUG: %u", settings.log_level);
          break;
        case 2:
          LOGln("Log level WARNING: %u", settings.log_level);
          break;
        case 3:
          LOGln("Log level ERRORS: %u", settings.log_level);
          break;
        case 4:
          LOGln("Log level CRITICAL: %u", settings.log_level);
          break;
        case 5:
          LOGln("Log level NONE: %u", settings.log_level);
          break;
        default:   // 1
          LOGln("Log level INFO: %u", settings.log_level);
          break;
      }
      break;
    }
    case 'g': {
      if (parseNumbers(arg, v, 1) != 1 || v[0] < 0 || v[0] > 3) { LOGln("Usage: g<0-3>  0:1.1V 1:1.5V 2:2.2V 3:3.9V"); return; }
      next.atten = (uint8_t)v[0];
      if (applySettings(next)) LOGln("Attenuation %u (%s)", settings.atten, ATTEN_TABLE[settings.atten].range);
      break;
    }
    case 'v': {
      if (parseNumbers(arg, v, 1) != 1 || v[0] < 1000 || v[0] > 5000) { LOGln("Usage: v<mV>, 1000..5000"); return; }
      settings.v_in = (int32_t)v[0];
      LOGln("Vin %ld mV", (long)settings.v_in);
      break;
    }
    case 'r': {
      if (parseNumbers(arg, v, 4) != 4 || v[0] < 1 || v[0] > 3 || v[1] < 1 || v[2] < 1 || v[3] < 1) {
        LOGln("Usage: r<thermistor 1..3>,<R1>,<R2>,<R3> resistors in Ohm"); return;
      }
      const int ch = (int)v[0] - 1;
      settings.r[ch][0] = (int32_t)v[1]; settings.r[ch][1] = (int32_t)v[2]; settings.r[ch][2] = (int32_t)v[3];
      LOGln("Thermistor %d R1-3: %ld, %ld, %ld Ohm", ch + 1, (long)settings.r[ch][0], (long)settings.r[ch][1], (long)settings.r[ch][2]);
      break;
    }
    case 'h': {
      if (parseNumbers(arg, v, 3) != 3 || v[0] == 0 || v[1] == 0 || v[2] == 0) { LOGln("Usage: h<A>,<B>,<C>, e.g. h1.1e-3,2.37e-4,7.5e-8"); return; }
      settings.sh[0] = (float)v[0]; settings.sh[1] = (float)v[1]; settings.sh[2] = (float)v[2];
      LOGln("Steinhart-Hart A=%.10e B=%.10e C=%.10e", settings.sh[0], settings.sh[1], settings.sh[2]);
      break;
    }
    case 'P': {
      if (cmd[1] == 's' && cmd[2] == '\0') {
        LOGln(saveSettings() ? "Settings saved" : "Saving settings failed");
      } else if (cmd[1] == 'l' && cmd[2] == '\0') {
        Settings loaded = settings;
        if (!readSettings(loaded)) { LOGln("No saved settings found"); return; }
        if (!applySettings(loaded, false)) return;
        applyLogLevel(settings.log_level);
        LOGln("Settings loaded");
        printSettings(settings);
      } else if (cmd[1] == 'd' && cmd[2] == '\0') {
        Settings stored; setDefaults(stored);
        if (!readSettings(stored)) { LOGln("No saved settings found"); return; }
        LOGln("Saved settings:");
        printSettings(stored);
      } else if (cmd[1] == 'c' && cmd[2] == '\0') {
        LOGln(clearSettings() ? "Saved settings cleared, defaults apply after reboot" : "Clearing settings failed");
      } else if (cmd[1] == 'r' && cmd[2] == '\0') {
        Settings defaults; setDefaults(defaults);
        if (!applySettings(defaults, false)) return;
        applyLogLevel(settings.log_level);
        LOGln("Factory defaults loaded, use Ps to save them");
        printSettings(settings);
      } else {
        LOGln("Usage: Ps (save), Pl (load), Pd (print saved), Pc (clear saved), Pr (reset to factory defaults)");
      }
      break;
    }
    default:
      LOGln("Unknown command '%s', ? for help", cmd);
      break;
  }
}

void processCommand(const char* input, CommandSource source) {
  while (*input == ' ' || *input == '\t') input++;
  char cmd[128];
  size_t len = strnlen(input, sizeof(cmd) - 1);
  memcpy(cmd, input, len);
  cmd[len] = '\0';
  while (len > 0 && (cmd[len - 1] == ' ' || cmd[len - 1] == '\t')) cmd[--len] = '\0';
  if (len == 0) return;
  logSetOutput(source == CommandSource::Ble ? (Print&)ble : (Print&)Serial);   // answer on the link the command came from
  if (len == 1) handleGeneralCommand(cmd[0]);
  else          handleParameterCommand(cmd);
}

// Boot helper - prints message and waits until timeout or user input
void serialTrigger(const char* mess, int timeout) {
  uint32_t startTime = millis();
  LOGln("");
  LOGln("%s", mess);
  while (!Serial.available() && ((millis() - startTime) < timeout)) {
    delay(100);
  }
  // Clear the serial input buffer
  while (Serial.available()) { Serial.read(); }
}

/* ============================================================================================================================ */
// Setup
/* ============================================================================================================================ */

// Serial.setTxBufferSize() exists for UART and hardware CDC, not for TinyUSB CDC (ESP32-S3 with USB mode TinyUSB)
#if !(defined(CONFIG_IDF_TARGET_ESP32S3) && defined(ARDUINO_USB_MODE) && (ARDUINO_USB_MODE == 0) && ARDUINO_USB_CDC_ON_BOOT)
  #define SERIAL_HAS_TX_BUFFER_SIZE 1
#endif

void setup() {

  // Serial Interface ------------
#ifdef SERIAL_HAS_TX_BUFFER_SIZE
  Serial.setTxBufferSize(4096);   // must be before begin(), more room for the data between two passes of the main loop
#endif
  Serial.begin(BAUDRATE);
  serialTrigger(waitmsg, 5000); // Wait for Serial to be ready

  // Logging level ---------------
  AudioLogger::instance().begin(Serial, AudioLogger::Warning); // Debug, Warning, Info, Error
  logSetOutput(Serial);

  // Settings -------------------
  setDefaults(settings);
  const bool loaded = readSettings(settings);
  applyLogLevel(settings.log_level);
  if (settings.serial_baud != BAUDRATE) Serial.updateBaudRate(settings.serial_baud);   // ignored by native USB
  if (loaded) LOGI("Loaded saved settings");
  else        LOGI("No saved settings, using defaults");

  // BLE Serial -----------------
  // Task pump: a background task moves the data between the BLE stack and the BLESerial buffers
  ble.setPumpMode(BLESerial::PumpMode::Task);
  if (!ble.begin(BLESerial::Mode::Fast, settings.ble_name, BLESerial::Security::None)) {
    LOGE("BLESerial begin() failed");   // USB keeps working
  }

  // ADC ------------------------
  // Calibrated readings from ESP32 ADC are in units of milli Volts
  LOGI("Configuring ADC for %s: %u thermistors, ADC rate %lu Hz", SYS.name, settings.channels / 2, (unsigned long)settings.channels * settings.out_rate * settings.bin_size);
  if (startMeasurement()) {
    LOGI("ADC configured");
  } else {
    LOGE("ADC configuration failed");
    while(1) {delay(50);}
  }
  LOGI("Send ? for the command menu");
}

/* ============================================================================================================================ */
// Main Loop
/* ============================================================================================================================ */

void loop() {

  // Commands -------------------
  if (ble_lr.poll(ble, ble_input_buff, sizeof(ble_input_buff))) {
    processCommand(ble_input_buff, CommandSource::Ble);
  }
  if (serial_lr.poll(Serial, serial_input_buff, sizeof(serial_input_buff))) {
    processCommand(serial_input_buff, CommandSource::Usb);
  }

  // Data -----------------------
  if (measurement_running) {
    // Read what the conversion stream has ready (the ADC read does not wait), this can be anything from nothing to several output samples
    size_t bytes_read = convertedStream().readBytes((uint8_t*) process_buffer, buffer_bytes);
    if (bytes_read > 0 && data_reporting) {   // when data reporting is off the data is read and discarded
      const int num_thermistors = valuesPerLine(settings);   // half bridge: one value per ADC channel, always voltage
      const size_t lines = bytes_read / M_BYTES_PER_SAMPLE / num_thermistors;
      int16_t* sample_buffer = (int16_t*) process_buffer;

      for (size_t i = 0; i < lines; i++) {
        size_t n = 0;
        for (int ch = 0; ch < num_thermistors; ch++) {
          const char* sep = (ch > 0) ? ", " : "";
          if (settings.half_bridge || settings.mode == MODE_VOLTAGE) {
            n += snprintf(out_text_buffer + n, sizeof(out_text_buffer) - n, "%s%d", sep, sample_buffer[ch]);
          } else {
            int32_t T_R = calcres(sample_buffer[ch], settings.v_in, settings.r[ch][0], settings.r[ch][1], settings.r[ch][2]); // resistance in Ohm
            if (settings.mode == MODE_RESISTANCE) {
              n += snprintf(out_text_buffer + n, sizeof(out_text_buffer) - n, "%s%ld", sep, (long)T_R);
            } else {
              int16_t T = calctemp(T_R, settings.sh[0], settings.sh[1], settings.sh[2]);   // Temperature in centi degrees
              // Calibration is valid between 0 and 50C
              float T_f = (T < 0 || T > 5000) ? NAN : float(T)/100.;
              n += snprintf(out_text_buffer + n, sizeof(out_text_buffer) - n, "%s%5.2f", sep, T_f);
            }
          }
        }
        n += snprintf(out_text_buffer + n, sizeof(out_text_buffer) - n, "\r\n");
        queueLine(out_text_buffer, n);
        sample_buffer += num_thermistors;
      }
    }
  }

  // Output ---------------------
  drainUsb();
  drainBle();
  if (!measurement_running) delay(1);   // idle, give other tasks time
  else                      yield();
}

/* ============================================================================================================================ */
// Thermistor related functions
/* ============================================================================================================================ */

// Calculating the actual resistance of the thermistor based on the measurements
// [Wikipedia Wheatstone bridge formula](https://en.wikipedia.org/wiki/Wheatstone_bridge):
// $ R_{\text{Thermistor}} = \frac{R_3 \left( V_{\text{in}} R_2 - V_{\text{diff}} (R_1 + R_2) \right)}{V_{\text{in}} R_1 + V_{\text{diff}} (R_1 + R_2)} $
int32_t calcres(
  int16_t Vdiff, // Voltage difference between two measurement points in Wheatstone bridge in mV
  int32_t Vin,   // Voltage from power supply for Wheatstone bridge input in mV
  int32_t R1,    // Resistor 1 in Wheatstone bridge in Ohms
  int32_t R2,    // Resistor 2 in Wheatstone bridge in Ohms
  int32_t R3     // Resistor 3 in Wheatstone bridge in Ohms
) {
  // Implemented with integer math: We need to use uint64_t because resistors are 10,000 Ohm and Vin is 3,300 mV resulting in numbers larger than int32_t (32,000)
  int64_t numerator = int64_t(R3) * ( (int64_t(Vin) * R2) - (int64_t(Vdiff) * (R1 + R2)) );
  int64_t denominator = (int64_t(Vin) * R1) + (int64_t(Vdiff) * (R1 + R2));
  if (denominator == 0) return 0;
  int32_t R4 = int32_t(numerator / denominator);
  return int32_t(R4);
}

// Calculate the temperature based on measured component values and manufacturer data: https://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation
// This is optimized to use minimal number of floating point operations and to avoid integer overflow
int16_t calctemp(
  int32_t R,     // Resistance in Ohm
  float A,       // Steinhart-Hart Coefficient A
  float B,       // Steinhart-Hart Coefficient B
  float C        // Steinhart-Hart Coefficient C
) {
  if (R <= 0) return -1; // invalid resistance, reported as NAN by the caller
  // Steinhart-Hart Equation
  float lnR = log(float(R));
  //LOGD("R4: %d, %f", Rr, lnR);
  float Tinv = (A + (B * lnR) + (C * (lnR * lnR * lnR)));
  //LOGD("Tinv %f", Tinv );
  float T = 1.0 / Tinv;
  T -= 273.15; // Convert Kelvin to Centigrade
  //LOGD("T %f", T);
  //LOGD("T %d", int16_t(T*100.0));  
  return int16_t(T * 100.0); // Convert temperature to centidegrees Celsius
}
