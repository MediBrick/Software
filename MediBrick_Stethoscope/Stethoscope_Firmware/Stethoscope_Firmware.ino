/*
  MediBrick Stethoscope + Blood Pressure
  ESP32-S3 Feather

  Commands are typed as lines (Enter), ? shows the help screen.
  Design and implementation steps: see Design.md.

  Default boot mode: BOTH

  I2S uses the ESP-IDF 5 standard mode driver (driver/i2s_std.h), the legacy driver/i2s.h is not used any more.

  Hardware Configuration of pre Amplifier.
  These are the options for components for the PCB microphone amplifier.
  R4,R8 and C5,C8
  Gain 17.5x: 82k   || 100pF v1 saturated
  Gain 12x:   56k   || 150pF
  Gain 10x:   47k   || 180pF v3 **
  Gain  2x:    8.2k || 1nF   v2 +/-6000 counts
  Amplifier designed as ~20kHz lowpass

*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPRLS.h>
#include "driver/rtc_io.h"
#include "driver/i2s_std.h"
#include "AudioBoard.h"
#include <Adafruit_MAX1704X.h>
#include <Preferences.h>
#include <math.h>
#include <BLESerial.h>
#include "logger.h"
#include <Linereader.h>
#include <RingBuffer.h>

#define APP_VERSION 1.3
#define BAUDRATE 500000                        // baud rate of the USB serial port (ignored by native USB)
#define BLE_NAME "MediBrickStethoscope"        // default BLE device name
#define BLE_NAME_MAX 24                        // longest BLE device name the Ln command accepts
#define BLE_MODE_DEFAULT 3                     // 0 Fast, 1 LowPower, 2 LongRange, 3 Balanced (same order as the library's Mode enum)
#define BLE_PWR_AUTO 127                       // BLE transmit power: the default of the BLE mode
#define BLE_PWR_MIN (-24)                      // dBm
#define BLE_PWR_MAX 9                          // dBm
#define BLE_STALL_MS 5000                      // BLE output buffer half full and nothing sent for this long: the BLE sender is stuck
#define PREF_NAMESPACE "mb-stetho"
#define OUT_BUFFER_SIZE 16384                  // bytes of data lines waiting for one link (power of 2)
#define OUT_CHUNK_BYTES   128                  // bytes moved from the output buffer to a link in one step

// Data outputs, bit mask in the settings
#define DATA_OUT_USB  0x01                     // send data over USB serial
#define DATA_OUT_BLE  0x02                     // send data over BLE serial
#define DATA_OUT_ALL  (DATA_OUT_USB | DATA_OUT_BLE)
#define AUDIO_TASK_STACK 8192                  // bytes
#define AUDIO_TASK_PRIORITY 5                  // above the Arduino loop task (1), the task mostly waits for the I2S DMA
#define AUDIO_TASK_CORE     1
char line[128];                                // data output line (pressure only mode)

// ===================== APP MODES =====================
#define APP_MODE_PRESSURE                  1
#define APP_MODE_STETHOSCOPE               2
#define APP_MODE_BOTH                      3
#define APP_MODE_OFF                       0   // audio and pressure off, battery, display, button and commands stay on
#define APP_MODE_UNAVAILABLE              -1   // requested subsystem could not be started

volatile int appMode = APP_MODE_BOTH;   // active mode (can differ from settings.mode if a subsystem is missing)
bool measurement_running = false;       // start / stop, audio and pressure only run while true

// ===================== PIN MAP =====================
static const int MY_I2C_SDA    =         SDA;
static const int MY_I2C_SCL    =         SCL;
static const int MY_I2CSPEED   =      100000; // Clock rate of the I2C bus for all devices (codec, OLED, pressure sensor, battery gauge), kept low because of the filters on SDA and SCL

static const int MY_ES8388ADDR =        0x10; // Address of ES8388 I2C port

static const gpio_num_t MY_BUTTON = GPIO_NUM_12; // Feather 12, Thing+ 9 active LOW

static const int MY_MPRLS_EOC  =          10; // Feather D10
static const int MY_MPRLS_RST  =          11; // Feather D11

static const int MY_I2S_MCLK   =          14; // Feather 14,   A4, Thing+ 13 Master Clock
static const int MY_I2S_BCLK   =          36; // Feather 36,  SCK, Thing+ 18 Bit Clock
static const int MY_I2S_LRCLK  =           8; // Feather  8,   A5, Thing+ 35 Left/Right Clock / Word Select
// I2S data pins as on the PCB (Assembly.md, KiCad netlist), named from the ESP32 point of view:
// J2 pad "DIN" (7) is the codec DSDIN input, it must be wired to the Feather MOSI pin, the ESP32 transmits on it.
// J2 pad "DOUT" (5) is the codec ASDOUT output, it must be wired to the Feather MISO pin, the ESP32 receives on it.
static const int MY_I2S_DATA_TX =         35; // Feather 35, MOSI, Thing+ 23: ESP32 data out -> pad DIN  -> ES8388 DSDIN (DAC, playback)
static const int MY_I2S_DATA_RX =         37; // Feather 37, MISO, Thing+ 19: ESP32 data in  <- pad DOUT <- ES8388 ASDOUT (ADC, microphone)

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

// ===================== OLED =====================
#define OLED_SCREEN_WIDTH                128
#define OLED_SCREEN_HEIGHT                32
#define OLED_RESET                        -1
#define OLED_ADDR                       0x3C
#define OLED_INTERVAL_MS                 200
#define BATTERY_INTERVAL_MS             5000
#define BATTERY_LOW_PERCENT             10.0f       // warning on the status screen below this level
#define BATTERY_SHUTDOWN_PERCENT         5.0f       // shut down at or below this level or voltage ...
#define BATTERY_SHUTDOWN_VOLT            3.4f       // (the 3.3 V regulator needs about 3.5 V under load)
#define BATTERY_SHUTDOWN_HOLD_MS       20000UL      // ... when it lasted this long without USB host and without charging
#define BUTTON_DEBOUNCE_US            50000UL
#define BUTTON_LONG_PRESS_US        2000000UL       // hold the button this long and release it to go to sleep
#define AUTO_SHUTOFF_MS            16200000UL       // 4.5 hours
#define BAT_X 0                                     // battery screen: battery icon and charge boxes
#define BAT_Y 16
#define BAT_W 64
#define BAT_H 16
#define BAT_FULL (BAT_W - 9)
#define BAT_THICK 10
#define CHRG_X 1
#define CHRG_Y 1

Adafruit_SSD1306 display(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, &Wire, OLED_RESET, MY_I2CSPEED, MY_I2CSPEED);   // clock during and after a display transfer

// ===================== PRESSURE SENSOR =====================
// 1000 to 6.25 ms
#define MPRLS_INTERVAL_MS     100

Adafruit_MPRLS mpr(
  MY_MPRLS_RST, // RESET PIN
  MY_MPRLS_EOC, // EOC PIN
  0.0f,         // PSI_min
  300.0f,       // PSI_max
  2.5f,         // OUTPUT_min
  22.5f,        // OUTPUT_max
  1.0f          // PSI_to_HPA
);

volatile bool  newPressure         =   false;
float         pressureZero         =    0.0f;   // sensor reading with the cuff open to air, taken at the start and by Qz
float         pressureRaw          =    0.0f;   // last sensor reading in mmHg before zero and calibration
float         pressureFiltered     =    0.0f;   // state of the pressure low-pass
bool          pressureFilterValid  =   false;   // the low-pass state holds a reading (it is started with the first reading)
uint16_t      pressureDecimCount   =       0;   // readings since the last pressure output value
bool          pressureInitialized  =   false;
unsigned long awakeStartMs         =       0;


// ===================== ES8388 / I2S =====================
#define I2S_PORT                   I2S_NUM_0
// Sample rate: 8, 16, 32 or 48 kHz, set with the R command (settings.fs_khz). The codec runs in single speed mode with
// MCLK = 256 * fs, so a rate change needs no codec register change, only the I2S clock.
#define FS_KHZ_DEFAULT                    16
#define AUDIO_DMA_BUF_LEN                128
#define AUDIO_DMA_BUF_COUNT                8
#define AUDIO_BLOCK_FRAMES               256
// CHANNELS2, CHANNELS9, CHANNELS16
#define AUDIO_NUM_CHANNELS         CHANNELS2
// BIT LENGTH_16BITS, BIT_LENGTH_18BITS
// BIT_LENGTH_20BITS, BIT_LENGTH_24BITS, BIT_LENGTH_32BITS
#define AUDIO_BIT_DEPTH    BIT_LENGTH_16BITS
// Input line (Line 1 / Line 2) and output line (Line 1 / Line 2) are selected at runtime (a, o commands)
#define ES8388_ADCCONTROL2_REG          0x0A   // ADC input select: 0x00 LIN1/RIN1, 0x50 LIN2/RIN2
#define ES8388_DACPOWER_REG             0x04   // DAC and output power, bits 5/4/3/2 = LOUT1/ROUT1/LOUT2/ROUT2 (found on the board: the
                                                   // arduino-audio-driver enum names these the other way round)
#define ES8388_ADC_SEL_LINE1            0x00
#define ES8388_ADC_SEL_LINE2            0x50
#define ES8388_DAC_PWR_LINE1            0x30   // LOUT1 + ROUT1: line out jack J1
#define ES8388_DAC_PWR_LINE2            0x0C   // LOUT2 + ROUT2: headphone jack J3

// Output: regValue = vol/3  -> 34 discrete levels (reg 0..33), 1.5 dB per step
// Vol step of 3 = 1 register step = 1.5 dB
#define AUDIO_OUTPUT_VOLUME               80
#define AUDIO_OUTPUT_VOLUME_STEP           6
// 0 to 100, internally it will map to 9 discrete levels
// Vol:   0,   13,  25,   38,   50,   63,   75,   88, 100
// dB:    0,    3,   6,    9,   12,   15,   18,   21,  24
// Power: 1,    2,   4,    8,   16,   32,   64,  128, 256  (power-of-2 per 3 dB step)
// Stepping is index-based (1 step = 3 dB) via stepInputVolume()
#define AUDIO_INPUT_VOLUME                 0   // 0 dB: start low, raise with Gi while watching the input level in the i command

// I2S_DATA_BIT_WIDTH_16BIT, _24BIT, _32BIT
#define AUDIO_I2S_DATA_BITS                 I2S_DATA_BIT_WIDTH_16BIT
// Audio in the DMA buffers: AUDIO_DMA_BUF_COUNT buffers of AUDIO_DMA_BUF_LEN frames. Time they can hold in us:
#define AUDIO_DMA_DEPTH_US(fs) ((uint32_t)(1000000.0f * AUDIO_DMA_BUF_COUNT * AUDIO_DMA_BUF_LEN / (fs)))   // fs in Hz

// ===================== DSP SETTINGS =====================
// Valid cutoffs and tone frequencies follow the sample rate fs (in Hz, settings.fs_khz * 1000)
#define HP_CUTOFF_MIN_HZ                   1.0f              // high-pass: 1 Hz .. fs/8
#define HP_CUTOFF_MAX_HZ(fs)     ((fs) / 8.0f)
#define LP_CUTOFF_MIN_HZ(fs)     ((fs) / 160.0f)             // low-pass: fs/160 .. 0.45 fs
#define LP_CUTOFF_MAX_HZ(fs)     ((fs) * 0.45f)
#define HP_CUTOFF_DEFAULT_HZ              20.0f
#define LP_CUTOFF_DEFAULT_HZ            2000.0f
#define FILTER_STEP_FACTOR                 1.25f             // Fh+/Fh-/Fl+/Fl- multiply/divide the cutoff by this
#define SW_GAIN_MIN                        0.1f
#define SW_GAIN_MAX                       16.0f
#define SW_GAIN_DEFAULT                    1.0f
#define SW_GAIN_STEP_FACTOR                1.4142f           // Gs+/Gs- = 3 dB
#define NOISE_LEFT_SCALE_MIN               0.0f
#define NOISE_LEFT_SCALE_MAX               4.0f
#define NOISE_LEFT_SCALE_DEFAULT           1.0f
#define NC_BIAS_MAX                    32767.0f   // subtraction bias range in counts
#define P_LP_FC_MIN                        0.1f   // pressure low-pass cutoff range in Hz (readings come at 1000 / MPRLS_INTERVAL_MS per second)
#define P_LP_FC_MAX                        4.0f
#define P_LP_FC_DEFAULT                    2.0f
#define P_CAL_MIN                          0.5f   // pressure calibration factor: mmHg = (reading - zero) / factor
#define P_CAL_MAX                          2.0f
#define P_CAL_DEFAULT                      0.9762f
#define P_DECIM_MAX                      255
#define AUDIO_FADE_MS                     20     // fade out / fade in time of the output around a change (clicks)
#define AUDIO_SETTLE_MS                  200     // output held at zero after a start or a change of the codec input (the input settles)
#define AUDIO_DECIMATION_MAX             255
#define SIM_TONE_L_HZ_DEFAULT            100.0f   // simulated input: left tone
#define SIM_TONE_R_HZ_DEFAULT            250.0f   // simulated input: right tone
#define SIM_TONE_HZ_MIN                    1.0f
#define SIM_TONE_HZ_MAX(fs)     ((fs) * 0.45f)
#define SIM_AMP_MAX                    32767     // peak amplitude in counts
#define SIM_TONE_AMP_DEFAULT            3277     // -20 dBFS
#define SIM_NOISE_AMP_DEFAULT            328     // -40 dBFS peak
#define IN_SRC_LINE1                       1     // a1: the input source number is the number of the a command
#define IN_SRC_LINE2                       2     // a2
#define IN_SRC_TONES                       3     // a3: simulated left and right tone
#define IN_SRC_TONES_NOISE                 4     // a4: simulated tones plus white noise
#define AUDIO_DECIMATION_DEFAULT           4

// Routing as last written to the codec (0xFF: unknown, written at the next applyRouting)
uint8_t codec_adc_sel   = 0xFF;                 // ADCCONTROL2
uint8_t codec_dac_power = 0xFF;                 // DACPOWER

// I2S channels (standard mode, full duplex on one port), created in initI2SDriver()
i2s_chan_handle_t i2s_tx = nullptr;             // ESP32 -> codec DSDIN (playback)
i2s_chan_handle_t i2s_rx = nullptr;             // codec ASDOUT -> ESP32 (microphones)
uint8_t           i2s_fs_khz = 0;               // sample rate the I2S clock is running at (kHz)

// Audio task (reads I2S, processes, writes I2S, queues data lines) and its handshake with the main loop
TaskHandle_t      audio_task_handle  = nullptr;
volatile bool     audio_run_req      = false;   // main loop -> task: run (true) or idle (false)
volatile bool     audio_running      = false;   // task -> main loop: I2S is running
volatile bool     audio_task_exit    = false;   // main loop -> task: delete yourself
volatile uint32_t audio_blocks       = 0;       // processed blocks
volatile uint32_t audio_read_errors  = 0;
volatile uint32_t audio_write_errors = 0;
volatile uint32_t audio_proc_us_max  = 0;       // longest processing time of a block
volatile int32_t  audio_peak_l       = 0;       // largest left input sample since the last i command (0..32768)
volatile int32_t  audio_peak_r       = 0;       // largest right input sample since the last i command
volatile uint32_t audio_clip_in      = 0;       // input frames with a sample at full scale since the last i command
volatile float    audio_fade_target  = 0.0f;     // main loop -> task: output gain to fade to (0 silent, 1 full)
volatile float    audio_fade_level   = 0.0f;     // task -> main loop: output gain now
volatile bool     audio_resettle     = false;    // main loop -> task: hold the output at zero for audio_settle_ms, then fade in
volatile uint16_t audio_settle_ms    = 0;
uint8_t           applied_in_src     = 0xFF;     // input source and subtraction mode the output was last faded for (0xFF: unknown)
uint8_t           applied_nc_mode    = 0xFF;
volatile uint32_t audio_clip_out     = 0;       // output samples limited to 16 bit (clipped) since the last i command
volatile uint32_t audio_gap_us_max   = 0;       // longest time between two blocks, the DMA buffer holds AUDIO_DMA_BUF_LEN * AUDIO_DMA_BUF_COUNT frames
volatile uint32_t usb_lines_dropped  = 0;       // data lines lost because the USB buffer was full
volatile uint32_t ble_lines_dropped  = 0;       // data lines lost because the BLE buffer was full
uint32_t          usb_bytes_sent     = 0;       // bytes written to the USB link since boot (main loop only)
uint32_t          ble_bytes_sent     = 0;       // bytes written to the BLE link since boot (main loop only)
uint32_t          ble_stall_restarts = 0;       // BLE restarted by the stall watchdog since boot
volatile bool     ble_subscribed     = false;   // BLE client is listening, updated by the main loop for the audio task
RingBuffer<char, OUT_BUFFER_SIZE> usb_out_buff; // data lines waiting for the USB link
RingBuffer<char, OUT_BUFFER_SIZE> ble_out_buff; // data lines waiting for the BLE link
bool audioInitialized    =             false;
bool codecPinsConfigured =             false;

float hpAlpha            =                 0.0f;
float lpAlpha            =                 0.0f;

struct ChannelFilterState {
  float hpXPrev;
  float hpYPrev;
  float lp1State;
  float lp2State;
};

ChannelFilterState leftFilterState  = {0.0f, 0.0f, 0.0f, 0.0f};
ChannelFilterState rightFilterState = {0.0f, 0.0f, 0.0f, 0.0f};

Adafruit_MAX17048 maxlipo;
bool batteryGaugeInitialized =           false;

audio_driver::DriverPins codecPins;
AudioBoard               audioBoard(AudioDriverES8388, codecPins);
// ===================== TIMING =====================
unsigned long lastPressureMs =               0;
unsigned long lastOledMs     =               0;
unsigned long lastBatteryMs  =               0;

// ===================== DISPLAY STATE =====================
bool  updateOLED    =                    false;
float pressure_mmHg =                        0.0f;
float batteryPercent =                      -1.0f;
float batteryVoltage =                       NAN;
float batteryChargeRate =                    NAN;       // percent per hour, positive while charging

// Display and button
enum ScreenMode { SCREEN_STATUS, SCREEN_BATTERY };
ScreenMode screenMode = SCREEN_STATUS;              // short button press switches
uint8_t   displayContent = 0;                       // what is on the display: 0 a message or unknown, 1 status screen, 2 battery screen
int       button_raw_state = HIGH;                  // HIGH: released, LOW: pressed
int       button_state = HIGH;                      // debounced
bool      button_ignore_release = false;            // button was still held at boot (wake up), ignore that release
bool      button_long_shown = false;                // the "release to sleep" message is on the display
uint32_t  last_button_change_us = 0;
uint32_t  button_pressed_us = 0;

// ===================== SETTINGS =====================
// Everything the user can change and save with Ps
struct Settings {
  uint8_t  mode;                    // APP_MODE_OFF / PRESSURE / STETHOSCOPE / BOTH, applied at boot
  bool     autostart;               // start measuring at boot
  uint8_t  fs_khz;                  // audio sample rate in kHz: 8, 16, 32 or 48
  uint8_t  data_out;                // DATA_OUT_USB and/or DATA_OUT_BLE: links that receive the data
  int      out_vol;                 // hardware output volume 0..100
  int      in_vol;                  // hardware input volume 0..100 (9 levels, 3 dB each)
  float    sw_gain;                 // software gain
  bool     hp_en;                   // high-pass on
  bool     lp_en;                   // low-pass on
  float    hp_fc;                   // high-pass cutoff in Hz
  float    lp_fc;                   // low-pass cutoff in Hz
  uint8_t  nc_mode;                 // subtraction: 0 off, 1 result in both ears, 2 left original and right result
  float    nc_gain;                 // gain on the subtracted (left, background) channel
  float    nc_bias;                 // offset in counts added to the result
  uint16_t audio_decim;             // audio data output decimation
  bool     p_lp_en;                 // pressure low-pass on
  float    p_lp_fc;                 // pressure low-pass cutoff in Hz
  float    p_cal;                   // pressure calibration factor, mmHg = (reading - zero) / p_cal
  uint16_t p_decim;                 // pressure output decimation (every p_decim-th reading is output)
  uint8_t  in_src;                  // IN_SRC_LINE1 / LINE2 / TONES / TONES_NOISE
  uint8_t  out_line;                // 1: audio out on Line 1 (LOUT1/ROUT1), 2: Line 2 (LOUT2/ROUT2, headphone)
  float    tone_l_hz;               // simulated left tone frequency
  float    tone_r_hz;               // simulated right tone frequency
  uint16_t tone_l_amp;              // simulated left tone peak amplitude in counts
  uint16_t tone_r_amp;              // simulated right tone peak amplitude in counts
  uint16_t noise_amp;               // simulated white noise peak amplitude in counts (added to both channels)
  uint8_t  log_level;               // LOG_LEVEL_DEBUG .. LOG_LEVEL_NONE
  char     ble_name[BLE_NAME_MAX + 1]; // BLE device name, applied at boot
  bool     ble_en;                  // BLE started at boot (advertising and the BLE serial link)
  uint8_t  ble_mode;                // BLE mode: 0 Fast, 1 LowPower, 2 LongRange, 3 Balanced
  int8_t   ble_pwr;                 // BLE transmit power in dBm (advertising, scanning and connection), BLE_PWR_AUTO = default of the mode
  uint32_t serial_baud;             // baud rate of the USB serial port, applied at boot (ignored by native USB)
};

Settings settings;
volatile bool data_reporting = false;   // continuous data output (not saved)

enum class CommandSource : uint8_t { Usb, Ble };   // command received over USB serial or BLE serial
BLESerial       ble;                // BLE serial interface
bool            ble_on = false;     // BLE was started (settings.ble_en and begin() succeeded)
LineReader<128> serial_lr;          // collects USB serial characters to a command line
LineReader<128> ble_lr;             // collects BLE serial characters to a command line
char            serial_input_buff[128];
char            ble_input_buff[128];

// ES8388 writes over I2C. The bus sometimes reports an error (Wire endTransmission 4), so a write is tried up to 3 times.
// The values last written are remembered, a write is only done when the value changes.
int      codec_out_vol    = -1;                  // output volume as last written to the codec (-1: unknown)
int      codec_in_vol     = -1;                  // input volume as last written to the codec
uint32_t codec_i2c_errors = 0;                   // failed tries since boot, shown by the i command
enum { CODEC_MUTE, CODEC_VOLUME, CODEC_IN_VOLUME };

bool codecWrite(int what, int value) {
  for (int attempt = 0; attempt < 3; attempt++) {
    const bool ok = (what == CODEC_MUTE)   ? audioBoard.setMute(value != 0) :
                    (what == CODEC_VOLUME) ? audioBoard.setVolume(value)    : audioBoard.setInputVolume(value);
    if (ok) return true;
    codec_i2c_errors = codec_i2c_errors + 1;
    delay(2);
  }
  LOGW("ES8388 write failed 3 times (item %d, value %d)", what, value);
  return false;
}

// ===================== HELPERS =====================
float clampf(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

const char *inputName(int src) {
  return (src == IN_SRC_LINE1) ? "L1" : (src == IN_SRC_LINE2) ? "L2" : (src == IN_SRC_TONES) ? "SIM" : "SIM+N";
}

const char *modeName(int mode) {
  return (mode == APP_MODE_PRESSURE)    ? "Pressure"    :
         (mode == APP_MODE_STETHOSCOPE) ? "Stethoscope" :
         (mode == APP_MODE_BOTH)        ? "Both"        :
         (mode == APP_MODE_OFF)         ? "Off"         : "Unavailable";
}

// Parse up to n numbers separated by commas/spaces, returns the number parsed, -1 on garbage
int parseNumbers(const char *p, double *out, int n) {
  int count = 0;
  while (count < n) {
    while (*p == ' ' || *p == ',' || *p == '\t') p++;
    if (*p == '\0') break;
    char *end;
    double v = strtod(p, &end);
    if (end == p) return -1;
    out[count++] = v;
    p = end;
  }
  while (*p == ' ' || *p == '\t') p++;
  return (*p == '\0') ? count : -1;
}

// Vol values that correspond exactly to each of the 9 discrete gain indices.
// Using these avoids asymmetric jumps when stepping up vs. down.
static const int inputVolLevels[9] = {0, 13, 25, 38, 50, 63, 75, 88, 100};

int inputGainIndex(int volSetting) {
  int vol = (int)clampf((float)volSetting, 0.0f, 100.0f);
  return (vol * 8) / 100;
}

void getInputGainInfo(int volSetting, int &idx, int &db, int &factor) {
  static const int dbTable[9] = {0, 3, 6, 9, 12, 15, 18, 21, 24};
  static const int factorTable[9] = {1, 2, 4, 8, 16, 32, 64, 128, 256};

  idx = inputGainIndex(volSetting);
  db = dbTable[idx];
  factor = factorTable[idx];
}

void getOutputGainInfo(int volSetting, int &regValue, float &db, float &factor) {
  int vol = (int)clampf((float)volSetting, 0.0f, 100.0f);

  // arduino-audio-driver default for ES8388 is volume hack 1:
  // register value = volume/3 for analog output volume regs.
  regValue = vol / 3;

  // ES8388 DAC Control 24/25/26/27: -45 dB + 1.5 dB per step.
  db = -45.0f + (1.5f * (float)regValue);
  factor = powf(10.0f, db / 20.0f);
}

// ===================== SETTINGS HANDLING =====================
void setDefaults(Settings &s) {
  s.mode        = APP_MODE_BOTH;
  s.autostart   = true;
  s.fs_khz      = FS_KHZ_DEFAULT;
  s.data_out    = DATA_OUT_USB;
  s.out_vol     = AUDIO_OUTPUT_VOLUME;
  s.in_vol      = AUDIO_INPUT_VOLUME;
  s.sw_gain     = SW_GAIN_DEFAULT;
  s.hp_en       = false;
  s.lp_en       = false;
  s.hp_fc       = HP_CUTOFF_DEFAULT_HZ;
  s.lp_fc       = LP_CUTOFF_DEFAULT_HZ;
  s.nc_mode     = 0;
  s.nc_gain     = NOISE_LEFT_SCALE_DEFAULT;
  s.nc_bias     = 0.0f;
  s.audio_decim = AUDIO_DECIMATION_DEFAULT;
  s.p_lp_en     = false;
  s.p_lp_fc     = P_LP_FC_DEFAULT;
  s.p_cal       = P_CAL_DEFAULT;
  s.p_decim     = 1;
  s.in_src      = IN_SRC_LINE1;
  s.out_line    = 2;
  s.tone_l_hz   = SIM_TONE_L_HZ_DEFAULT;
  s.tone_r_hz   = SIM_TONE_R_HZ_DEFAULT;
  s.tone_l_amp  = SIM_TONE_AMP_DEFAULT;
  s.tone_r_amp  = SIM_TONE_AMP_DEFAULT;
  s.noise_amp   = SIM_NOISE_AMP_DEFAULT;
  s.log_level   = LOG_LEVEL_INFO;
  strlcpy(s.ble_name, BLE_NAME, sizeof(s.ble_name));
  s.ble_en      = false;                   // BLE off: its radio bursts are audible in the microphone signal, start it with Lb1
  s.ble_mode    = BLE_MODE_DEFAULT;
  s.ble_pwr     = BLE_PWR_AUTO;
  s.serial_baud = BAUDRATE;
}

// Returns nullptr if the settings are usable, otherwise the reason they are not
const char *settingsError(const Settings &s) {
  if (s.mode > APP_MODE_BOTH)                                  return "mode must be 0, 1, 2 or 3";
  if (s.out_vol < 0 || s.out_vol > 100)                        return "output gain must be 0..100";
  if (s.in_vol  < 0 || s.in_vol  > 100)                        return "input gain must be 0..100";
  if (s.sw_gain < SW_GAIN_MIN || s.sw_gain > SW_GAIN_MAX)      return "software gain is out of range";
  if (s.fs_khz != 8 && s.fs_khz != 16 && s.fs_khz != 32 && s.fs_khz != 48) return "sample rate must be 8, 16, 32 or 48 kHz";
  const float fs = s.fs_khz * 1000.0f;
  if (s.hp_fc < HP_CUTOFF_MIN_HZ || s.hp_fc > HP_CUTOFF_MAX_HZ(fs)) return "high-pass cutoff is out of range for this sample rate";
  if (s.lp_fc < LP_CUTOFF_MIN_HZ(fs) || s.lp_fc > LP_CUTOFF_MAX_HZ(fs)) return "low-pass cutoff is out of range for this sample rate";
  if (s.hp_en && s.lp_en && s.hp_fc >= s.lp_fc)                return "high-pass cutoff must be below low-pass cutoff";
  if (s.nc_mode > 2)                                           return "subtraction mode must be 0, 1 or 2";
  if (s.nc_gain < NOISE_LEFT_SCALE_MIN || s.nc_gain > NOISE_LEFT_SCALE_MAX) return "subtraction gain is out of range";
  if (s.nc_bias < -NC_BIAS_MAX || s.nc_bias > NC_BIAS_MAX)     return "subtraction bias is out of range";
  if (s.audio_decim < 1 || s.audio_decim > AUDIO_DECIMATION_MAX) return "decimation is out of range";
  if (s.p_lp_fc < P_LP_FC_MIN || s.p_lp_fc > P_LP_FC_MAX)      return "pressure low-pass cutoff is out of range";
  if (s.p_cal < P_CAL_MIN || s.p_cal > P_CAL_MAX)              return "pressure calibration factor is out of range";
  if (s.p_decim < 1 || s.p_decim > P_DECIM_MAX)                return "pressure decimation is out of range";
  if (s.ble_mode > 3)                                          return "BLE mode must be 0..3";
  if (s.ble_pwr != BLE_PWR_AUTO && (s.ble_pwr < BLE_PWR_MIN || s.ble_pwr > BLE_PWR_MAX)) return "BLE power is out of range";
  if (s.in_src < IN_SRC_LINE1 || s.in_src > IN_SRC_TONES_NOISE) return "input source must be 1..4";
  if (s.out_line < 1 || s.out_line > 2)                        return "output line must be 1 or 2";
  if (s.tone_l_hz < SIM_TONE_HZ_MIN || s.tone_l_hz > SIM_TONE_HZ_MAX(fs)) return "left tone frequency is out of range for this sample rate";
  if (s.tone_r_hz < SIM_TONE_HZ_MIN || s.tone_r_hz > SIM_TONE_HZ_MAX(fs)) return "right tone frequency is out of range for this sample rate";
  if (s.tone_l_amp > SIM_AMP_MAX || s.tone_r_amp > SIM_AMP_MAX || s.noise_amp > SIM_AMP_MAX) return "amplitude must be 0..32767";
  return nullptr;
}

// Moves stored values into their valid ranges (NVS content can be old or damaged)
void sanitizeSettings(Settings &s) {
  Settings d; setDefaults(d);
  if (s.mode > APP_MODE_BOTH) s.mode = d.mode;
  s.out_vol     = (int)clampf((float)s.out_vol, 0.0f, 100.0f);
  s.in_vol      = (int)clampf((float)s.in_vol,  0.0f, 100.0f);
  s.sw_gain     = clampf(s.sw_gain,  SW_GAIN_MIN, SW_GAIN_MAX);
  if (s.fs_khz != 8 && s.fs_khz != 16 && s.fs_khz != 32 && s.fs_khz != 48) s.fs_khz = d.fs_khz;
  const float fs = s.fs_khz * 1000.0f;
  s.hp_fc       = clampf(s.hp_fc,    HP_CUTOFF_MIN_HZ, HP_CUTOFF_MAX_HZ(fs));
  s.lp_fc       = clampf(s.lp_fc,    LP_CUTOFF_MIN_HZ(fs), LP_CUTOFF_MAX_HZ(fs));
  s.nc_gain     = clampf(s.nc_gain,  NOISE_LEFT_SCALE_MIN, NOISE_LEFT_SCALE_MAX);
  s.nc_bias     = clampf(s.nc_bias,  -NC_BIAS_MAX, NC_BIAS_MAX);
  if (s.nc_mode > 2) s.nc_mode = d.nc_mode;
  if (s.audio_decim < 1 || s.audio_decim > AUDIO_DECIMATION_MAX) s.audio_decim = d.audio_decim;
  s.data_out   &= DATA_OUT_ALL;
  s.p_lp_fc     = clampf(s.p_lp_fc, P_LP_FC_MIN, P_LP_FC_MAX);
  s.p_cal       = clampf(s.p_cal, P_CAL_MIN, P_CAL_MAX);
  if (s.p_decim < 1 || s.p_decim > P_DECIM_MAX) s.p_decim = d.p_decim;
  if (s.ble_mode > 3) s.ble_mode = d.ble_mode;
  if (s.ble_pwr != BLE_PWR_AUTO && (s.ble_pwr < BLE_PWR_MIN || s.ble_pwr > BLE_PWR_MAX)) s.ble_pwr = d.ble_pwr;
  if (s.in_src < IN_SRC_LINE1 || s.in_src > IN_SRC_TONES_NOISE) s.in_src = d.in_src;
  if (s.out_line < 1 || s.out_line > 2) s.out_line = d.out_line;
  s.tone_l_hz   = clampf(s.tone_l_hz, SIM_TONE_HZ_MIN, SIM_TONE_HZ_MAX(fs));
  s.tone_r_hz   = clampf(s.tone_r_hz, SIM_TONE_HZ_MIN, SIM_TONE_HZ_MAX(fs));
  if (s.tone_l_amp > SIM_AMP_MAX) s.tone_l_amp = d.tone_l_amp;
  if (s.tone_r_amp > SIM_AMP_MAX) s.tone_r_amp = d.tone_r_amp;
  if (s.noise_amp  > SIM_AMP_MAX) s.noise_amp  = d.noise_amp;
  if (s.log_level > LOG_LEVEL_NONE) s.log_level = d.log_level;
  if (s.serial_baud < 9600 || s.serial_baud > 2000000) s.serial_baud = d.serial_baud;
  if (s.hp_en && s.lp_en && s.hp_fc >= s.lp_fc) { s.hp_fc = d.hp_fc; s.lp_fc = d.lp_fc; }
}

bool validBleName(const char *n) {
  size_t len = strlen(n);
  if (len == 0 || len > BLE_NAME_MAX) return false;
  for (size_t i = 0; i < len; i++) if (n[i] < 32 || n[i] > 126) return false;
  return true;
}

bool saveSettings(const Settings &s) {
  Preferences prefs;
  if (!prefs.begin(PREF_NAMESPACE, false)) return false;
  prefs.putUChar ("mode",      s.mode);
  prefs.putBool  ("autoStart", s.autostart);
  prefs.putUChar ("fsKhz",     s.fs_khz);
  prefs.putUChar ("dataOut",   s.data_out);
  prefs.putUChar ("inSrc",     s.in_src);
  prefs.putUChar ("outLine",   s.out_line);
  prefs.putFloat ("tlHz",      s.tone_l_hz);
  prefs.putFloat ("trHz",      s.tone_r_hz);
  prefs.putUShort("tlAmp",     s.tone_l_amp);
  prefs.putUShort("trAmp",     s.tone_r_amp);
  prefs.putUShort("nsAmp",     s.noise_amp);
  prefs.putInt   ("outVol",    s.out_vol);
  prefs.putInt   ("inVol",     s.in_vol);
  prefs.putFloat ("swGain",    s.sw_gain);
  prefs.putBool  ("hpEn",      s.hp_en);
  prefs.putBool  ("lpEn",      s.lp_en);
  prefs.putFloat ("hpFc",      s.hp_fc);
  prefs.putFloat ("lpFc",      s.lp_fc);
  prefs.putUChar ("ncMode",    s.nc_mode);
  prefs.putFloat ("ncScale",   s.nc_gain);
  prefs.putFloat ("ncBias",    s.nc_bias);
  prefs.putInt   ("plotDecim", s.audio_decim);
  prefs.putUChar ("logLvl",    s.log_level);
  prefs.putString("bleName",   s.ble_name);
  prefs.putBool  ("bleEn",     s.ble_en);
  prefs.putBool  ("pLpEn",     s.p_lp_en);
  prefs.putFloat ("pLpFc",     s.p_lp_fc);
  prefs.putFloat ("pCal",      s.p_cal);
  prefs.putUShort("pDecim",    s.p_decim);
  prefs.putUChar ("bleMode",   s.ble_mode);
  prefs.putChar  ("blePwr",    s.ble_pwr);
  prefs.putUInt  ("baud",      s.serial_baud);
  prefs.end();
  return true;
}

// Reads saved values on top of s. Returns false if nothing is saved.
bool readSettings(Settings &s) {
  Preferences prefs;
  if (!prefs.begin(PREF_NAMESPACE, true)) return false;
  if (!prefs.isKey("outVol")) { prefs.end(); return false; }
  s.mode        = prefs.getUChar ("mode",      s.mode);
  s.autostart   = prefs.getBool  ("autoStart", s.autostart);
  s.fs_khz      = prefs.getUChar ("fsKhz",     s.fs_khz);
  s.data_out    = prefs.getUChar ("dataOut",   s.data_out);
  s.in_src      = prefs.getUChar ("inSrc",     s.in_src);
  s.out_line    = prefs.getUChar ("outLine",   s.out_line);
  s.tone_l_hz   = prefs.getFloat ("tlHz",      s.tone_l_hz);
  s.tone_r_hz   = prefs.getFloat ("trHz",      s.tone_r_hz);
  s.tone_l_amp  = prefs.getUShort("tlAmp",     s.tone_l_amp);
  s.tone_r_amp  = prefs.getUShort("trAmp",     s.tone_r_amp);
  s.noise_amp   = prefs.getUShort("nsAmp",     s.noise_amp);
  s.out_vol     = prefs.getInt   ("outVol",    s.out_vol);
  s.in_vol      = prefs.getInt   ("inVol",     s.in_vol);
  s.sw_gain     = prefs.getFloat ("swGain",    s.sw_gain);
  s.hp_en       = prefs.getBool  ("hpEn",      s.hp_en);
  s.lp_en       = prefs.getBool  ("lpEn",      s.lp_en);
  s.hp_fc       = prefs.getFloat ("hpFc",      s.hp_fc);
  s.lp_fc       = prefs.getFloat ("lpFc",      s.lp_fc);
  s.nc_mode     = prefs.getUChar ("ncMode",    s.nc_mode);
  s.nc_gain     = prefs.getFloat ("ncScale",   s.nc_gain);
  s.nc_bias     = prefs.getFloat ("ncBias",    s.nc_bias);
  s.audio_decim = (uint16_t)prefs.getInt("plotDecim", s.audio_decim);
  s.log_level   = prefs.getUChar ("logLvl",    s.log_level);
  s.serial_baud = prefs.getUInt  ("baud",      s.serial_baud);
  s.ble_en      = prefs.getBool  ("bleEn",     s.ble_en);
  s.p_lp_en     = prefs.getBool  ("pLpEn",     s.p_lp_en);
  s.p_lp_fc     = prefs.getFloat ("pLpFc",     s.p_lp_fc);
  s.p_cal       = prefs.getFloat ("pCal",      s.p_cal);
  s.p_decim     = prefs.getUShort("pDecim",    s.p_decim);
  s.ble_mode    = prefs.getUChar ("bleMode",   s.ble_mode);
  s.ble_pwr     = prefs.getChar  ("blePwr",    s.ble_pwr);
  String name   = prefs.getString("bleName",   s.ble_name);
  prefs.end();
  if (validBleName(name.c_str())) strlcpy(s.ble_name, name.c_str(), sizeof(s.ble_name));
  sanitizeSettings(s);
  return true;
}

bool clearSettings() {
  Preferences prefs;
  if (!prefs.begin(PREF_NAMESPACE, false)) return false;
  prefs.clear();
  prefs.end();
  return true;
}

// ===================== AUDIO FILTERING =====================
// The audio filters. They are used by the audio task (section 2 of the processing chain) on every sample of
// the left and the right channel. Each stage can be switched on and off (FH/Fh, FL/Fl) and the cutoff
// frequencies are set with the Fh and Fl commands.
//
// What a filter does: a high-pass filter lets high frequencies through and blocks low frequencies, a low-pass
// filter does the opposite. The cutoff frequency fc is where the signal is reduced to 71 % (-3 dB).
// Both filters here are "IIR" filters: the new output sample is calculated from the new input sample and from
// the previous samples, so the filter needs a small memory per channel (ChannelFilterState). fs is the
// sample rate (set with R, 16000 samples per second by default).
//
//   Stage 1 High-pass   y[n] = a * ( y[n-1] + x[n] - x[n-1] )          a = exp(-2 * pi * fc / fs)
//           x = input, y = output, [n] = this sample, [n-1] = the previous sample.
//           A constant (DC) input gives x[n] - x[n-1] = 0, so the output decays to 0. This removes the DC offset
//           of the microphone amplifier and slow rumble (handling noise). Reduces by 6 dB per octave below fc.
//
//   Stage 2 Gain        y = x * software gain                          (Gs command)
//
//   Stage 3 Low-pass    s1 += b * ( x  - s1 )                          b = 1 - exp(-2 * pi * fc / fs)
//                       s2 += b * ( s1 - s2 )                          output = s2
//           Each line moves a stored value s a fraction b of the way towards its input, which smooths the signal
//           (a running average with a decaying memory). Two of these stages in a row make a steeper low-pass:
//           each stage is -3 dB at fc, together -6 dB at fc and 12 dB per octave above. Removes hiss and
//           high frequency noise above what is needed (heart sounds are mostly below 400 Hz).
//
// The coefficients a and b only depend on fc and fs, so they are calculated once when the cutoff changes
// (updateFilterCoefficients) and not for every sample. The exponential function is slow, the loop below
// only uses multiplications and additions.

void updateFilterCoefficients() {
  const float fs = settings.fs_khz * 1000.0f;
  hpAlpha = expf(-2.0f * PI * settings.hp_fc / fs);
  lpAlpha = 1.0f - expf(-2.0f * PI * settings.lp_fc / fs);
}

float applyAudioFilters(float inputSample, ChannelFilterState &state, const Settings &cfg, float hpA, float lpA) {
  float x = inputSample;

  if (cfg.hp_en) {
    float y = hpA * (state.hpYPrev + x - state.hpXPrev);
    state.hpXPrev = x;
    state.hpYPrev = y;
    x = y;
  }

  x *= cfg.sw_gain;

  if (cfg.lp_en) {
    state.lp1State += lpA * (x - state.lp1State);
    state.lp2State += lpA * (state.lp1State - state.lp2State);
    x = state.lp2State;
  }

  return x;
}

void resetAudioFilterState() {
  leftFilterState = {0.0f, 0.0f, 0.0f, 0.0f};
  rightFilterState = {0.0f, 0.0f, 0.0f, 0.0f};
}

// ===================== DATA OUTPUT =====================
// Data lines are formatted once and copied into one ring buffer per link, so a slow or absent link (e.g. no BLE client)
// can not stall the other one or the audio. The main loop moves the buffered bytes to the links without blocking.
// Nothing is throttled: if a link can not keep up its buffer fills and complete lines are dropped (and counted).
void queueLine(const char *text, size_t len) {
  const uint8_t links = settings.data_out;
  if ((links & DATA_OUT_USB) && usb_out_buff.push(text, len) != len) usb_lines_dropped = usb_lines_dropped + 1;   // push is all or nothing
  if ((links & DATA_OUT_BLE) && ble_subscribed && ble_out_buff.push(text, len) != len) ble_lines_dropped = ble_lines_dropped + 1;
}

void drainUsb() {
  if (!data_reporting || !(settings.data_out & DATA_OUT_USB)) { usb_out_buff.clear(); return; }
  size_t room = Serial.availableForWrite();
  char chunk[OUT_CHUNK_BYTES];
  while (room > 0 && usb_out_buff.available() > 0) {
    size_t m = usb_out_buff.peek(chunk, min(room, sizeof(chunk)));
    size_t w = Serial.write((const uint8_t *)chunk, m);
    usb_out_buff.consume(w);
    usb_bytes_sent += w;
    room -= w;
    if (w < m) break;
  }
}

void drainBle() {
  if (!data_reporting || !(settings.data_out & DATA_OUT_BLE) || !ble_subscribed) { ble_out_buff.clear(); return; }
  if (!ble.writeReady()) return;
  size_t room = ble.writeAvailable();
  char chunk[OUT_CHUNK_BYTES];
  while (room > 0 && ble_out_buff.available() > 0) {
    size_t m = ble_out_buff.peek(chunk, min(room, sizeof(chunk)));
    size_t w = ble.writeNonBlocking((const uint8_t *)chunk, m);
    ble_out_buff.consume(w);
    ble_bytes_sent += w;
    room -= w;
    if (w < m) break;
  }
}

// Starts the BLE serial link: advertising and the command and data service. ble_on tells if it runs.
// A background task moves the data between the BLE stack and the BLESerial buffers.
void bleStart() {
  if (ble_on) return;
  ble.setPumpMode(BLESerial::PumpMode::Task);
  static const BLESerial::Mode modes[4] = {BLESerial::Mode::Fast, BLESerial::Mode::LowPower, BLESerial::Mode::LongRange, BLESerial::Mode::Balanced};
  ble_on = ble.begin(modes[settings.ble_mode], settings.ble_name, BLESerial::Security::None);
  if (!ble_on) { LOGE("BLESerial begin() failed"); return; }   // USB keeps working
  if (settings.ble_pwr != BLE_PWR_AUTO) ble.setPower(settings.ble_pwr, PWR_ALL);   // instead of the default power of the mode
}

const char *bleModeName(int mode) {
  static const char *const names[4] = {"Fast", "LowPower", "LongRange", "Balanced"};
  return names[mode & 3];
}

// Stops the BLE serial link completely: a connected client is disconnected and the radio is switched off.
void bleStop() {
  if (!ble_on) return;
  ble_subscribed = false;                          // the audio task stops queueing data for BLE
  ble.flush();                                     // let a reply to a command received over BLE go out
  ble.end();
  ble_on = false;
  ble_out_buff.clear();
  ble_lr.reset();
}

// ===================== AUDIO TASK =====================
// Runs on its own core 1 task so that commands, display, battery and pressure can not stall the audio.
// The task owns the I2S driver while it runs. The main loop only sets audio_run_req / audio_task_exit and
// waits for audio_running, and it is the only one that talks to the codec and the OLED over I2C.
//
// HOW THE AUDIO IS PROCESSED
// The codec delivers the sound as numbers: fs samples per second (8, 16, 32 or 48 kHz, R command, 16 kHz by default),
// 16 bit (-32768 .. +32767), left and right channel alternating (L, R, L, R, ...). The task works on blocks of
// AUDIO_BLOCK_FRAMES = 256 frames (16 ms at 16 kHz) at a time. For every block it does the following, in this order:
//
//   codec ADC --> [1 SIMULATION] --> [2 FILTERS AND GAIN] --> [3 SUBTRACTION] --+--> [5 CLIP] --> codec DAC
//                  (only a3, a4)     high-pass, gain, low-pass    R - g * L + b  |    to 16 bit    (headphone)
//                                                                               +--> [4 DATA LINE] --> USB / BLE
//
//   1  SIMULATION    replaces the microphone samples by calculated tones and noise (input a3 and a4)
//   2  FILTERS+GAIN  high-pass, software gain, low-pass, separately for left and right channel
//   3  SUBTRACTION   optional (N1, N2): removes what the left microphone (background noise) hears from the right
//                    (stethoscope) microphone: result = right - Ng * left + Nb
//   4  DATA LINE     every audio_decim-th frame is sent as text "left,right" to USB and/or BLE (z command)
//   5  CLIP + PLAY   limits the result to 16 bit and gives it to the DAC, this is what you hear
//   6  FADE          ramps the output down and up around a start, a stop or a change, so it does not click
//
// The settings are copied at the start of each block, so a command never changes them in the middle of a block.
void audioTask(void *) {
  static int16_t buf[AUDIO_BLOCK_FRAMES * 2];
  int decimCounter = 0;                           // frames since the last data line (section 4)
  uint16_t lastDecim = 1;
  uint8_t lastFsKhz = 0;                          // sample rate the tone coefficients were calculated for
  float fadeGain = 0.0f;                          // gain applied to the output (section 6)
  int32_t settleLeft = 0;                         // frames the output is still held at zero (section 6)
  uint32_t lastReadUs = 0;
  size_t carry = 0;                               // bytes of an incomplete frame left over from the previous read
  char out[48];

  // Memory of the tone generators (see section 1 below)
  float lc = 1.0f, ls = 0.0f, lx = 0.0f, ly = 0.0f;   // left tone:  c = cos(w), s = sin(w), current point (x, y) of the rotation
  float rc = 1.0f, rs = 0.0f, rx = 0.0f, ry = 0.0f;   // right tone: same for the right channel
  float lastLHz = 0.0f, lastRHz = 0.0f;
  uint16_t lastLAmp = 0xFFFF, lastRAmp = 0xFFFF;
  uint32_t rng = 2463534242UL;                        // state of the noise generator

  while (!audio_task_exit) {
    if (!audio_run_req) {                         // idle
      if (audio_running) {                        // stop: BCLK and WS stop, MCLK keeps running
        i2s_channel_disable(i2s_rx);
        i2s_channel_disable(i2s_tx);
        audio_running = false;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    if (!audio_running) {                         // start: the transmit buffers send zeros until the first block is written
      if (i2s_channel_enable(i2s_tx) != ESP_OK || i2s_channel_enable(i2s_rx) != ESP_OK) {
        i2s_channel_disable(i2s_rx);              // only the one that was enabled accepts this, the other call fails harmlessly
        i2s_channel_disable(i2s_tx);
        audio_read_errors = audio_read_errors + 1;
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }
      resetAudioFilterState();
      decimCounter = 0;
      lastReadUs = 0;
      carry = 0;
      fadeGain = 0.0f;                            // every start is silent until the main loop asks for the fade in
      settleLeft = 0;
      audio_fade_level = 0.0f;
      audio_running = true;
    }

    // Read behind the bytes of an incomplete frame (a read that timed out can end inside a frame). Left and right
    // samples must stay paired, so the leftover bytes are kept and completed by the next read.
    size_t bytesRead = 0;
    esp_err_t r = i2s_channel_read(i2s_rx, ((uint8_t *)buf) + carry, sizeof(buf) - carry, &bytesRead, 100);
    bytesRead += carry;
    const size_t wholeBytes = bytesRead & ~(size_t)3;           // complete frames: 4 bytes = one left and one right sample
    carry = bytesRead - wholeBytes;
    if (wholeBytes == 0) { if (r != ESP_OK) audio_read_errors = audio_read_errors + 1; continue; }
    bytesRead = wholeBytes;

    const uint32_t t0 = micros();
    if (lastReadUs != 0 && (t0 - lastReadUs) > audio_gap_us_max) audio_gap_us_max = t0 - lastReadUs;
    lastReadUs = t0;

    const Settings cfg = settings;                // settings and coefficients as they are for this block
    const float hpA = hpAlpha, lpA = lpAlpha;
    const bool bypass = cfg.nc_mode == 0 && !cfg.hp_en && !cfg.lp_en && fabsf(cfg.sw_gain - 1.0f) < 0.0001f;   // nothing to calculate: sections 2, 3 and 5 are skipped
    const bool bothEars = (cfg.nc_mode == 1);     // N1: the result is played in both ears
    const float fsHz = cfg.fs_khz * 1000.0f;
    const float fadeTarget = audio_fade_target;
    if (audio_resettle) {                         // hold the output at zero while the input settles, then fade in
      audio_resettle = false;
      fadeGain = 0.0f;
      settleLeft = (int32_t)(audio_settle_ms * fsHz / 1000.0f);
    }
    const bool report = data_reporting && settleLeft <= 0 && fadeTarget > 0.0f;   // no data while the output is silent (start, change)
    const int sampleCount = bytesRead / sizeof(int16_t);
    if (!report || cfg.audio_decim != lastDecim) { decimCounter = 0; lastDecim = cfg.audio_decim; }

    // ==================================================================================================================
    // SECTION 1: SIMULATED INPUT (only when the input source is a3 or a4)
    //
    // This replaces the samples that came from the microphones by calculated signals. It is used to test the
    // filters, the data output and the subtraction without a microphone.
    //
    // Tones: a sine wave is calculated without a sine function for every sample. A point (x, y) is rotated around
    // the origin by a small angle w for every sample, w = 2 * pi * f / fs (f = tone frequency, fs = sample rate).
    // The height of the point, y, is the sine wave, its distance from the origin is the amplitude:
    //
    //      x_new = c * x - s * y          c = cos(w), s = sin(w), calculated once when the frequency is set
    //      y_new = s * x + c * y          y is the output sample
    //
    // That is 4 multiplications and 2 additions per sample. Rounding errors slowly change the distance from the
    // origin, so once per block the point is scaled back to the amplitude.
    //
    // Noise (a4 only): the xorshift generator turns a 32 bit number into the next number of a long pseudo random
    // sequence with 3 shifts and 3 exclusive-or operations. The number is scaled to +-noise amplitude (white
    // noise: all frequencies equally strong). The same noise value is added to the left and the right channel,
    // like background noise that reaches both microphones, so the subtraction (section 3) can remove it.
    //
    // Left and right tone are written into buf as 16 bit samples, the same format the codec delivers.
    // ==================================================================================================================
    if (cfg.in_src >= IN_SRC_TONES) {
      const bool fsChanged = (cfg.fs_khz != lastFsKhz);                 // the angle per sample depends on the sample rate
      lastFsKhz = cfg.fs_khz;
      if (fsChanged || cfg.tone_l_hz != lastLHz || cfg.tone_l_amp != lastLAmp) {      // new left tone: coefficients and start state
        lastLHz = cfg.tone_l_hz; lastLAmp = cfg.tone_l_amp;
        lc = cosf(2.0f * PI * lastLHz / fsHz); ls = sinf(2.0f * PI * lastLHz / fsHz);
        lx = lastLAmp; ly = 0.0f;
      } else if (lastLAmp != 0) {                                         // keep the radius at the amplitude
        const float k = lastLAmp / sqrtf(lx * lx + ly * ly); lx *= k; ly *= k;
      }
      if (fsChanged || cfg.tone_r_hz != lastRHz || cfg.tone_r_amp != lastRAmp) {
        lastRHz = cfg.tone_r_hz; lastRAmp = cfg.tone_r_amp;
        rc = cosf(2.0f * PI * lastRHz / fsHz); rs = sinf(2.0f * PI * lastRHz / fsHz);
        rx = lastRAmp; ry = 0.0f;
      } else if (lastRAmp != 0) {
        const float k = lastRAmp / sqrtf(rx * rx + ry * ry); rx *= k; ry *= k;
      }
      const float noiseScale = (cfg.in_src == IN_SRC_TONES_NOISE) ? cfg.noise_amp * (1.0f / 2147483648.0f) : 0.0f;
      for (int i = 0; i + 1 < sampleCount; i += 2) {
        const float lxn = lc * lx - ls * ly; ly = ls * lx + lc * ly; lx = lxn;
        const float rxn = rc * rx - rs * ry; ry = rs * rx + rc * ry; rx = rxn;
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        const float noise = (int32_t)rng * noiseScale;
        buf[i]     = (int16_t)fminf(fmaxf(ly + noise, -32768.0f), 32767.0f);
        buf[i + 1] = (int16_t)fminf(fmaxf(ry + noise, -32768.0f), 32767.0f);
      }
    }

    // One pass over the block: sections 2 to 5 are done one frame (one left and one right sample) at a time.
    int32_t peakL = 0, peakR = 0;                 // input level of this block, shown by the i command to set the input gain (Gi)
    uint32_t clippedFrames = 0;                   // input frames at full scale
    uint32_t clippedOut = 0;                      // output samples that had to be limited to 16 bit
    for (int i = 0; i + 1 < sampleCount; i += 2) {

      // Input level meter: largest absolute sample and number of frames with a sample at full scale (+-32767), measured on
      // the input before the filters. A signal at full scale is clipped by the codec and can not be repaired by software.
      const int32_t absL = abs((int32_t)buf[i]), absR = abs((int32_t)buf[i + 1]);
      if (absL > peakL) peakL = absL;
      if (absR > peakR) peakR = absR;
      if (absL >= 32767 || absR >= 32767) clippedFrames = clippedFrames + 1;

      // ================================================================================================================
      // SECTION 2: FILTERS AND GAIN  (high-pass -> software gain -> low-pass, for the left and the right channel)
      // SECTION 3: SUBTRACTION       (result = right - Ng * left + Nb)
      //
      // Section 2 is done by applyAudioFilters(), explained at "AUDIO FILTERING" above. Each channel has its own
      // filter memory. If the high-pass, the low-pass and the subtraction are off and the gain is 1.0 (bypass), the
      // samples are passed on unchanged and nothing is calculated.
      //
      // Section 3, the subtraction: the left microphone hears the background (room noise), the right microphone
      // hears the body sound plus the same background. Subtracting the filtered left signal, multiplied by a gain
      // that matches the two microphones (Ng), from the filtered right signal leaves mainly the body sound. Nb adds a
      // constant offset (in counts) to the result. Whatever reaches both microphones equally, like the radio bursts of
      // the Bluetooth, is removed as well.
      //
      // The results are rounded to whole numbers once: leftI is the filtered left (background) channel, rightI is the
      // filtered right channel, or the result of the subtraction if it is on. They are used for the data line and for
      // the DAC.
      // ================================================================================================================
      int32_t leftI, rightI;
      if (bypass) {
        leftI  = buf[i];
        rightI = buf[i + 1];
      } else {
        const float leftY = applyAudioFilters((float)buf[i],     leftFilterState,  cfg, hpA, lpA);
        float rightY      = applyAudioFilters((float)buf[i + 1], rightFilterState, cfg, hpA, lpA);
        if (cfg.nc_mode != 0) rightY = rightY - cfg.nc_gain * leftY + cfg.nc_bias;
        leftI  = lrintf(leftY);
        rightI = lrintf(rightY);
      }

      // ================================================================================================================
      // SECTION 4: DATA LINE (text output for plotting and recording, only while data reporting is on, command z)
      //
      // The audio has 16000 frames per second, more than a USB or Bluetooth link can carry as text. Only every
      // audio_decim-th frame (La command) is sent: the filtered audio is simply sampled at a lower rate. The low-pass
      // filter is the anti-alias filter; to avoid aliasing set it to about 0.4 times the output rate or lower.
      // The line has the form "left,right" and in mode 3 (both) a third number, the pressure in mmHg. The line is
      // put in the output buffer of each selected link; the main loop sends it, the audio never waits for the link.
      // ================================================================================================================
      if (report && ++decimCounter >= cfg.audio_decim) {
        decimCounter = 0;
        int n;
        if (appMode == APP_MODE_BOTH && newPressure) {
          n = snprintf(out, sizeof(out), "%ld,%ld,%.1f\r\n", (long)leftI, (long)rightI, pressure_mmHg);
          newPressure = false;
        } else {
          n = snprintf(out, sizeof(out), "%ld,%ld\r\n", (long)leftI, (long)rightI);
        }
        queueLine(out, n);
      }

      // ================================================================================================================
      // SECTION 5: CLIP AND PLAY BACK
      // Filters, gain and subtraction can make a value larger than a 16 bit number can hold. Such values are
      // limited (clipped) to -32768 .. +32767, otherwise they would wrap around and sound like a loud crack. The
      // number of limited samples is shown by the i command.
      // What is played: subtraction off (N0): left and right as they are; N1: the result in both ears; N2: the left
      // (background) channel in the left ear and the result in the right ear.
      // The samples replace the input samples in buf and are written to the codec DAC after the loop. In bypass the
      // input samples are already in buf.
      // ================================================================================================================
      if (!bypass) {
        int32_t outL = bothEars ? rightI : leftI;
        int32_t outR = rightI;
        if      (outL >  32767) { outL =  32767; clippedOut = clippedOut + 1; }
        else if (outL < -32768) { outL = -32768; clippedOut = clippedOut + 1; }
        if      (outR >  32767) { outR =  32767; clippedOut = clippedOut + 1; }
        else if (outR < -32768) { outR = -32768; clippedOut = clippedOut + 1; }
        buf[i]     = (int16_t)outL;
        buf[i + 1] = (int16_t)outR;
      }
    }
    if (peakL > audio_peak_l) audio_peak_l = peakL;
    if (peakR > audio_peak_r) audio_peak_r = peakR;
    audio_clip_in = audio_clip_in + clippedFrames;
    audio_clip_out = audio_clip_out + clippedOut;

    // ==================================================================================================================
    // SECTION 6: FADE (no clicks)
    // A sudden change of the signal, like switching the input, starting the audio or unmuting the codec, is heard as a
    // click. The main loop therefore asks for a fade (audio_fade_target = 0 or 1): the gain of the output moves
    // linearly to the target in AUDIO_FADE_MS, the main loop waits until the output is silent before it changes the
    // codec, and afterwards the output is held at zero for audio_settle_ms (the codec input needs some time to settle
    // after a start or a change of the input line) before it fades in. The data line is not sent during the hold, the
    // settling input would show up in it.
    // The three cases are chosen once per block: hold (silence), ramp (multiply by the changing gain, only while
    // fading), or nothing (gain 1, the samples stay as they are).
    // ==================================================================================================================
    if (settleLeft > 0) {
      memset(buf, 0, bytesRead);
      settleLeft -= sampleCount / 2;
      fadeGain = 0.0f;
    } else if (fadeGain != fadeTarget) {
      const float step = (fadeTarget > fadeGain) ? 1000.0f / (AUDIO_FADE_MS * fsHz) : -1000.0f / (AUDIO_FADE_MS * fsHz);
      for (int i = 0; i + 1 < sampleCount; i += 2) {
        fadeGain += step;
        if (step > 0.0f ? fadeGain > fadeTarget : fadeGain < fadeTarget) fadeGain = fadeTarget;
        buf[i]     = (int16_t)lrintf(buf[i]     * fadeGain);
        buf[i + 1] = (int16_t)lrintf(buf[i + 1] * fadeGain);
      }
    } else if (fadeGain == 0.0f) {
      memset(buf, 0, bytesRead);
    }
    audio_fade_level = fadeGain;
    const uint32_t procUs = micros() - t0;
    if (procUs > audio_proc_us_max) audio_proc_us_max = procUs;

    // Write the processed audio data back to the I2S interface.
    size_t txOffset = 0;
    while (txOffset < bytesRead) {
      size_t chunkWritten = 0;
      esp_err_t w = i2s_channel_write(i2s_tx, ((const uint8_t *)buf) + txOffset, bytesRead - txOffset, &chunkWritten, 100);
      if (w != ESP_OK || chunkWritten == 0) { audio_write_errors = audio_write_errors + 1; break; }
      txOffset += chunkWritten;
    }
    if (carry != 0) memmove(buf, ((uint8_t *)buf) + bytesRead, carry);   // keep the incomplete frame for the next read
    audio_blocks = audio_blocks + 1;
  }

  audio_running = false;
  audio_task_handle = nullptr;
  vTaskDelete(nullptr);
}

// ===================== PRINTING =====================
void applyLogLevel(int level) {
  logSetLevel(level);
}

void printCodecVolumeStatus(const Settings &s) {
  int inIdx, inDb, inFactor;
  getInputGainInfo(s.in_vol, inIdx, inDb, inFactor);

  int outReg;
  float outDb, outFactor;
  getOutputGainInfo(s.out_vol, outReg, outDb, outFactor);

  LOGln("  HW input gain    : idx %d = %+d dB, x%d (Gi, setting %d)", inIdx, inDb, inFactor, s.in_vol);
  LOGln("  HW output gain   : setting %d = reg %d, %+.1f dB, x%.3f (Go)", s.out_vol, outReg, outDb, outFactor);
}

void printSettings(const Settings &s) {
  LOGln("  Mode at boot     : %s (m)", modeName(s.mode));
  LOGln("  Autostart        : %s (A)", s.autostart ? "ON" : "OFF");
  LOGln("  Input source     : %s (a)", s.in_src == IN_SRC_LINE1 ? "Line 1" : s.in_src == IN_SRC_LINE2 ? "Line 2" :
                                       s.in_src == IN_SRC_TONES ? "simulated tones" : "simulated tones + noise");
  LOGln("  Output line      : Line %u (o)", s.out_line);
  LOGln("  Simulation       : left %.1f Hz amp %u, right %.1f Hz amp %u, noise amp %u (Tl, Tr, Tn)", s.tone_l_hz, s.tone_l_amp, s.tone_r_hz, s.tone_r_amp, s.noise_amp);
  LOGln("  Data links       : USB %s, BLE %s (z)", (s.data_out & DATA_OUT_USB) ? "on" : "off", (s.data_out & DATA_OUT_BLE) ? "on" : "off");
  printCodecVolumeStatus(s);
  LOGln("  Software gain    : %.2f (Gs)", s.sw_gain);
  LOGln("  High-pass filter : %s @ %.1f Hz (FH/Fh)", s.hp_en ? "ON " : "OFF", s.hp_fc);
  LOGln("  Low-pass filter  : %s @ %.1f Hz (FL/Fl)", s.lp_en ? "ON " : "OFF", s.lp_fc);
  LOGln("  Subtraction R-L  : %s, left gain %.2f, bias %.0f (N, Ng, Nb)",
        s.nc_mode == 0 ? "OFF" : s.nc_mode == 1 ? "ON, result in both ears" : "ON, left original and right result", s.nc_gain, s.nc_bias);
  LOGln("  Pressure         : low-pass %s @ %.1f Hz (Qe, Qf), calibration factor %.4f (Qc), output every %u-th reading (Lp)",
        s.p_lp_en ? "ON " : "OFF", s.p_lp_fc, s.p_cal, s.p_decim);
  LOGln("  Sample rate      : %u kHz (R)", s.fs_khz);
  LOGln("  Audio decimation : %u -> %.0f Hz output (La)", s.audio_decim, s.fs_khz * 1000.0f / s.audio_decim);
  if (s.audio_decim > 1 && (!s.lp_en || s.lp_fc > 0.5f * s.fs_khz * 1000.0f / s.audio_decim))
    LOGln("  Warning          : the data output keeps every %u-th frame, the low-pass is off or above half the output rate (0.4 is better), it will alias", s.audio_decim);
  LOGln("  Log level        : %u (l)", s.log_level);
  LOGln("  BLE              : %s at boot (Lb), name %s (Ln), mode %s (Lm), power %s (Lt)", s.ble_en ? "ON" : "OFF", s.ble_name, bleModeName(s.ble_mode),
        s.ble_pwr == BLE_PWR_AUTO ? "default of the mode" : (String(s.ble_pwr) + " dBm").c_str());
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  LOGln("  Serial baud rate : %lu (Ls, no effect: native USB has no baud rate)", (unsigned long)s.serial_baud);
#else
  LOGln("  Serial baud rate : %lu (Ls, used after reboot)", (unsigned long)s.serial_baud);
#endif
}

void printAllSettings() {
  LOGln("================================================================================");
  LOGln("Current Settings:");
  LOGln("================================================================================");
  LOGln("Active mode: %s | Measurement: %s | Audio: %s | Pressure: %s | Data output: %s", modeName(appMode),
        measurement_running ? "RUNNING" : "STOPPED", audioInitialized ? "ready" : "off", pressureInitialized ? "ready" : "off", data_reporting ? "ON" : "OFF");
  if (batteryGaugeInitialized && batteryPercent >= 0.0f) LOGln("Battery: %.0f %%", batteryPercent);
  LOGln("--------------------------------------------------------------------------------");
  printSettings(settings);
  LOGln("================================================================================");
}

void printSystemInfo() {
  LOGln("================================================================================");
  LOGln("System Info:");
  LOGln("================================================================================");
  LOGln("Stethoscope & BP Brick v%s, %s, uptime %lu s, free heap %lu bytes",
        STR(APP_VERSION), ESP.getChipModel(), millis() / 1000UL, (unsigned long)ESP.getFreeHeap());
  const float fsNow = settings.fs_khz * 1000.0f;
  LOGln("Audio clock       : sample rate setting %u kHz, I2S clock running at %u kHz", settings.fs_khz, i2s_fs_khz);
  LOGln("Audio task        : %s, %lu blocks, read/write errors %lu/%lu",
        audio_running ? "running" : "idle", (unsigned long)audio_blocks, (unsigned long)audio_read_errors, (unsigned long)audio_write_errors);
  LOGln("                    block time: processing max %lu us, max gap between blocks %lu us (DMA holds %d us)",
        (unsigned long)audio_proc_us_max, (unsigned long)audio_gap_us_max,
        (int)AUDIO_DMA_DEPTH_US(fsNow));
  LOGln("Input level       : left peak %ld (%.0f %% of full scale), right peak %ld (%.0f %%), %lu frames at full scale since the last i",
        (long)audio_peak_l, 100.0f * audio_peak_l / 32768.0f, (long)audio_peak_r, 100.0f * audio_peak_r / 32768.0f, (unsigned long)audio_clip_in);
  LOGln("Output clipping   : %lu samples limited to 16 bit since the last i", (unsigned long)audio_clip_out);
  LOGln("Data output       : %s, %.0f lines/s per link (La %u)", data_reporting ? "ON" : "OFF", fsNow / settings.audio_decim, settings.audio_decim);
  // Link rates and dropped lines since the previous i command (the totals since boot are in brackets)
  static uint32_t prevMs = 0, prevUsbBytes = 0, prevBleBytes = 0, prevUsbDrop = 0, prevBleDrop = 0;
  const uint32_t nowMs = millis();
  const float seconds = (prevMs != 0) ? (nowMs - prevMs) / 1000.0f : 0.0f;   // no rate at the first i
  const float usbKBs  = (seconds > 0.0f) ? (usb_bytes_sent - prevUsbBytes) / seconds / 1000.0f : 0.0f;
  const float bleKBs  = (seconds > 0.0f) ? (ble_bytes_sent - prevBleBytes) / seconds / 1000.0f : 0.0f;
  LOGln("  USB             : %s, %u of %u bytes buffered, sending %.1f kB/s, %lu lines dropped since the last i (%lu in total)", (settings.data_out & DATA_OUT_USB) ? "on" : "off",
        (unsigned)usb_out_buff.available(), (unsigned)usb_out_buff.capacity(), usbKBs, (unsigned long)(usb_lines_dropped - prevUsbDrop), (unsigned long)usb_lines_dropped);
  if (ble_on) LOGln("  BLE radio       : mode %s, transmit power advertising %d dBm, connection %d dBm", bleModeName(settings.ble_mode),
                    (int)NimBLEDevice::getPower(PWR_ADV), (int)NimBLEDevice::getPower(PWR_CONN));
  LOGln("  BLE             : radio %s, data link %s, %s, %u of %u bytes buffered, sending %.1f kB/s, %lu lines dropped since the last i (%lu in total), %lu stall restarts, name %s",
        !ble_on ? "STOPPED" : ble.isConnected() ? "connected" : "ON (advertising)", (settings.data_out & DATA_OUT_BLE) ? "on" : "off",
        ble_subscribed ? "subscribed" : "not subscribed",
        (unsigned)ble_out_buff.available(), (unsigned)ble_out_buff.capacity(), bleKBs, (unsigned long)(ble_lines_dropped - prevBleDrop), (unsigned long)ble_lines_dropped, (unsigned long)ble_stall_restarts, settings.ble_name);
  prevMs = nowMs;
  prevUsbBytes = usb_bytes_sent;
  prevBleBytes = ble_bytes_sent;
  prevUsbDrop = usb_lines_dropped;
  prevBleDrop = ble_lines_dropped;
  LOGln("I2C               : clock %lu Hz, failed ES8388 write tries since boot %lu", (unsigned long)Wire.getClock(), (unsigned long)codec_i2c_errors);
  if (pressureInitialized) LOGln("Pressure          : %.1f mmHg, zero %.2f, last sensor reading %.2f, low-pass %s", pressure_mmHg, pressureZero, pressureRaw, settings.p_lp_en ? "ON" : "OFF");
  if (batteryGaugeInitialized) LOGln("Battery           : %.0f %%, %.2f V, %+.1f %%/hr", batteryPercent, batteryVoltage, batteryChargeRate);
  LOGln("Sensors           : MPRLS %s, ES8388/I2S %s, battery gauge %s, OLED yes",
        pressureInitialized ? "ready" : "not started", audioInitialized ? "ready" : "not started",
        batteryGaugeInitialized ? "yes" : "no");
  LOGln("================================================================================");
  LOGln("System Limits:");
  LOGln("================================================================================");
  LOGln("Audio sample rate : %.0f Hz, 16 bit, stereo (R8, R16, R32, R48 kHz), blocks of %d frames = %.1f ms", fsNow, AUDIO_BLOCK_FRAMES, 1000.0f * AUDIO_BLOCK_FRAMES / fsNow);
  LOGln("Input source      : a1 Line 1, a2 Line 2, a3 simulated tones, a4 simulated tones + white noise");
  LOGln("HW input gain     : Gi<0-8>, 3 dB per step:");
  LOGln("                    idx 0..8 = 0, 3, 6, 9, 12, 15, 18, 21, 24 dB = x1 .. x256");
  LOGln("Output line       : o1 Line 1 (LOUT1/ROUT1), o2 Line 2 (LOUT2/ROUT2, headphone)");
  LOGln("HW output gain    : Go<0-100>, setting/3 = reg 0..33, 1.5 dB per step:");
  LOGln("                    -45.0 dB (reg 0) .. +4.5 dB (reg 33), Go+/Go- step %d", AUDIO_OUTPUT_VOLUME_STEP);
  LOGln("Software gain     : Gs<%.1f-%.1f>, Gs+/Gs- step 3 dB", SW_GAIN_MIN, SW_GAIN_MAX);
  LOGln("High-pass cutoff  : Fh<%.0f-%.0f Hz> at %.0f Hz sample rate", HP_CUTOFF_MIN_HZ, HP_CUTOFF_MAX_HZ(fsNow), fsNow);
  LOGln("Low-pass cutoff   : Fl<%.0f-%.0f Hz>, high-pass must be below low-pass when both are on", LP_CUTOFF_MIN_HZ(fsNow), LP_CUTOFF_MAX_HZ(fsNow));
  LOGln("                    Fh+/Fh-/Fl+/Fl- change the cutoff by a factor %.2f", FILTER_STEP_FACTOR);
  LOGln("Subtraction gain  : Ng<%.1f-%.1f>", NOISE_LEFT_SCALE_MIN, NOISE_LEFT_SCALE_MAX);
  LOGln("Audio decimation  : La<1-%d>, output rate = sample rate / La", AUDIO_DECIMATION_MAX);
  LOGln("Simulation        : Tl/Tr<%.0f-%.0f Hz>[,<amp>], Tn<amp>, amplitudes 0..%d peak counts", SIM_TONE_HZ_MIN, SIM_TONE_HZ_MAX(fsNow), SIM_AMP_MAX);
  LOGln("Battery           : low warning below %.0f %%, shutdown at %.0f %% or %.1f V after %lu s without USB host and without charging",
        BATTERY_LOW_PERCENT, BATTERY_SHUTDOWN_PERCENT, BATTERY_SHUTDOWN_VOLT, BATTERY_SHUTDOWN_HOLD_MS / 1000UL);
  LOGln("Pressure          : %d ms between readings (%.1f per second), Qf<%.1f-%.1f Hz>, Qc<%.1f-%.1f>, Lp<1-%d>", MPRLS_INTERVAL_MS,
        1000.0f / MPRLS_INTERVAL_MS, P_LP_FC_MIN, P_LP_FC_MAX, P_CAL_MIN, P_CAL_MAX, P_DECIM_MAX);
  audio_clip_out = 0;
  audio_peak_l = 0;                                // a new measurement interval starts with every i
  audio_peak_r = 0;
  audio_clip_in = 0;
  LOGln("================================================================================");
}

void helpMenu() {
  LOGln("================================================================================");
  LOGln("| MediBrick Stethoscope and Blood Pressure Board               v%s            |", STR(APP_VERSION));
  LOGln("================================================================================");
  LOGln("| GENERAL COMMANDS                       | DATA AND START/STOP                 |");
  LOGln("|----------------------------------------|-------------------------------------|");
  LOGln("| ?: help screen                         | z: toggle data reporting on/off     |");
  LOGln("| s: show current settings               | z[s|b|a]: USB, BLE, both, off       |");
  LOGln("| i: show system info and limits         | z[S|B|A]: USB, BLE, both, on        |");
  LOGln("| x: reboot                              | .: toggle start/stop                |");
  LOGln("| Button: short = screen, long = sleep   | >: start measurement                |");
  LOGln("|                                        | <: stop measurement                 |");
  LOGln("|                                        | A<0|1>: autostart at boot           |");
  LOGln("|========================================|=====================================|");
  LOGln("| PERSISTENCE                            | MODE                                |");
  LOGln("|----------------------------------------|-------------------------------------|");
  LOGln("| Ps: save NVS settings                  | m0: audio and pressure off          |");
  LOGln("| Pl: load NVS settings                  | m1: pressure only                   |");
  LOGln("| Pd: print NVS settings                 | m2: audio only                      |");
  LOGln("| Pc: clear NVS settings                 | m3: both                            |");
  LOGln("| Pr: reset to factory defaults          |                                     |");
  LOGln("|========================================|=====================================|");
  LOGln("| INPUT SOURCE                           | OUTPUT LINE                         |");
  LOGln("|----------------------------------------|-------------------------------------|");
  LOGln("| a1: Line 1 (electret + amplifier)      | o1: Line 1 (line out)               |");
  LOGln("| a2: Line 2 (jack microphone)           | o2: Line 2 (headphone)              |");
  LOGln("| a3: simulated tones L and R            |                                     |");
  LOGln("| a4: simulated tones + white noise      |                                     |");
  LOGln("|========================================|=====================================|");
  LOGln("| ENGINEERING: AUDIO FILTER AND GAIN     | ENGINEERING: L-R SUBTRACTION        |");
  LOGln("|----------------------------------------|-------------------------------------|");
  LOGln("| FH[Hz]: high-pass ON (+ set Hz)        | N<0|1|2>: subtraction mode          |");
  LOGln("| Fh: OFF, Fh<Hz>: set Hz only           |   1 result in both ears             |");
  LOGln("| Fh+ / Fh-: step high-pass up/down      |   2 left original, right result     |");
  LOGln("| FL[Hz]: low-pass ON (+ set Hz)         | Ng<x>: gain on left (background)    |");
  LOGln("| Fl: OFF, Fl<Hz>: set Hz only           | Nb<x>: bias in counts               |");
  LOGln("| Fl+ / Fl-: step low-pass up/down       |   result = R - Ng*L + Nb            |");
  LOGln("| Gi<0-8> / Gi+ / Gi-: HW input gain     | SIMULATION (a3, a4)                 |");
  LOGln("| Go<0-100> / Go+ / Go-: HW output gain  | Tl<Hz>[,<amp>]: left tone           |");
  LOGln("| Gs<x> / Gs+ / Gs-: software gain       | Tr<Hz>[,<amp>]: right tone          |");
  LOGln("| R<8|16|32|48>: sample rate in kHz      | Tn<amp>: white noise, both channels |");
  LOGln("|   (filters and tones follow R)         |   amp = peak counts, 0..32767       |");
  LOGln("|========================================|=====================================|");
  LOGln("| ENGINEERING: PRESSURE                  | mmHg = (reading - zero) / Qc        |");
  LOGln("|----------------------------------------|-------------------------------------|");
  LOGln("| Qe<0|1>: low-pass off/on               | low-pass: first order, readings     |");
  LOGln("| Qf<Hz>: low-pass cutoff, 0.1 to 4 Hz   | come 10 times per second            |");
  LOGln("| Qc<x>: calibration factor, 0.5 to 2    | zero is taken when the pressure     |");
  LOGln("| Qz: zero now, cuff open to air         | mode starts and by Qz               |");
  LOGln("|========================================|=====================================|");
  LOGln("| ENGINEERING: LINKS AND LOG                                                   |");
  LOGln("|------------------------------------------------------------------------------|");
  LOGln("| La<n>: audio output decimation         | l<0-5>: log level 0 debug .. 5 none |");
  LOGln("| Lp<n>: pressure output decimation      | Ls<baud>: baud, UART boards only    |");
  LOGln("================================================================================");
  LOGln("| ENGINEERING: BLUETOOTH SERIAL (off by default)                               |");
  LOGln("|------------------------------------------------------------------------------|");
  LOGln("| Lb<0|1>: BLE stop/start (Ps: boot)     | Lm<0-3>: BLE mode (BLE restarts)    |");
  LOGln("| Ln<name>: BLE name (Ps, Lb0, Lb1)      |   0 Fast      7.5 ms link  +9 dBm   |");
  LOGln("| Lt<dBm>: BLE power, -24 to +9 dBm      |   1 LowPower  60-120 ms    -6 dBm   |");
  LOGln("|   e.g. -12 -9 -6 -3 0 3 6 9            |   2 LongRange coded PHY    +9 dBm   |");
  LOGln("| Lta: default power of the mode         |   3 Balanced  15-30 ms      0 dBm   |");
  LOGln("| Lbs: BLE library status (to USB)       | Power shown is the link power;      |");
  LOGln("| Default: off, mode 3, power auto       | advertising is lower in 1 and 3.    |");
  LOGln("| BLE bursts are heard in the mics:      | Fast and LongRange use the most.    |");
  LOGln("| record audio with BLE off (Lb0).       | A stuck BLE sender restarts BLE.    |");
  LOGln("================================================================================");
  LOGln("");
  LOGln("Examples:");
  LOGln("  m2        - Audio only                   m0    - Audio and pressure off");
  LOGln("  FH30      - High-pass on at 30 Hz        Fh500 - set 500 Hz, on/off unchanged");
  LOGln("  Fl        - Low-pass off                 Gi+   - 3 dB more input gain");
  LOGln("  z         - Toggle continuous data display   .  - Start/stop");
  LOGln("Changes are lost on reboot unless saved with Ps.");
  LOGln("");
}

bool initBatteryGauge() {
  if (!maxlipo.begin()) {
    LOGE("MAX17048 init failed");
    return false;
  }
  LOGI("MAX17048 battery gauge ready");
  return true;
}

// Draw battery % in the top-right corner.
void drawBatteryPercent() {
  if (!batteryGaugeInitialized || batteryPercent < 0.0f) return;
  char buf[8];
  snprintf(buf, sizeof(buf), "%d%%", (int)(batteryPercent + 0.5f));
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(OLED_SCREEN_WIDTH - w, 0);
  display.print(buf);
}

// Display a message on the OLED screen.
// Not used during runtime measurement updates.
void oledMessage(const String &line1,
                 const String &line2 = "",
                 const String &line3 = "",
                 bool showBattery = true) {
  displayContent = 0;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  if (line2.length()) display.println(line2);
  if (line3.length()) display.println(line3);
  if (showBattery) drawBatteryPercent();
  display.display();
}

// ===================== SCREENS, BUTTON, SLEEP =====================
// A screen refresh takes about 40 ms of I2C time (100 kHz), the main loop waits for it (the audio does not, it is a
// separate task). To keep the loop free the screens first build their text, compare it with what is on the display and
// only write the display when something changed. The loop calls them at most every OLED_INTERVAL_MS.

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

void drawBatteryIndicator(int x, int y, float percent) {
  display.drawBitmap(x, y, BATTERY_BITMAP, BAT_W, BAT_H, SSD1306_WHITE);
  float pct = isfinite(percent) ? percent : 0.0f;
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  int width = (int)roundf((pct / 100.0f) * BAT_FULL);
  display.fillRect(x + 3, y + 3, width, BAT_THICK, SSD1306_WHITE);
}

// Two boxes, discharge fills the left one, charge the right one, scaled to +-max_percent per hour
void drawChargingIndicator(int x, int y, int width, int height, float percent_per_hour, float max_percent) {
  display.drawRect(x, y, width, height, SSD1306_WHITE);
  display.drawRect(x + width + 1, y, width, height, SSD1306_WHITE);
  const float rate = isfinite(percent_per_hour) ? percent_per_hour : 0.0f;
  int inner = (int)roundf((width - 2) * (fabsf(rate) / max_percent));
  if (inner > width - 2) inner = width - 2;
  if (inner == 0 && fabsf(rate) > 0.05f) inner = 1;   // small rates stay visible
  if (rate < 0.0f) display.fillRect(x + width - inner, y + 2, inner, height - 4, SSD1306_WHITE);
  else             display.fillRect(x + width + 2, y + 2, inner, height - 4, SSD1306_WHITE);
}

// Battery screen: charge boxes (discharge left, charge right), rate, battery icon, percent and voltage
void drawBatteryScreen() {
  static int lastPct = -2, lastVolt = -2, lastRate = -9999;
  const int pct  = (batteryPercent >= 0.0f) ? (int)(batteryPercent + 0.5f) : -1;
  const int volt = isfinite(batteryVoltage) ? (int)(batteryVoltage * 100.0f + 0.5f) : -1;
  const int rate = isfinite(batteryChargeRate) ? (int)roundf(batteryChargeRate * 10.0f) : -9999;
  if (displayContent == 2 && pct == lastPct && volt == lastVolt && rate == lastRate) return;   // nothing new to show
  lastPct = pct; lastVolt = volt; lastRate = rate;
  displayContent = 2;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  drawBatteryIndicator(BAT_X, BAT_Y, batteryPercent);
  drawChargingIndicator(CHRG_X, CHRG_Y, BAT_W / 2, BAT_H - 2, batteryChargeRate, 10.0f);
  display.setCursor(BAT_X + BAT_W + 4, CHRG_Y + 3);
  if (rate != -9999) { display.print(batteryChargeRate, 1); display.print("%/hr"); }
  else               { display.print("rate n/a"); }
  display.setCursor(BAT_X + BAT_W + 4, BAT_Y + 4);
  if (pct >= 0) { display.print(pct); display.print("% "); }
  if (volt >= 0) { display.print(batteryVoltage, 2); display.print("V"); }
  if (pct < 0 && volt < 0) display.print("bat n/a");
  display.display();
}

// Status screen (128 x 32, text size 1 is 21 characters wide):
//   mode, run state and battery percent in the first row, the audio route in the second row, then either the data and
//   Bluetooth state (rows 3 and 4) or, while the pressure is measured, the pressure in large digits (rows 3 and 4).
struct StatusFrame { char head[24]; char line1[24]; char line2[24]; char line3[24]; char big[16]; char pct[8]; };

void drawStatusScreen() {
  static StatusFrame last = {};
  StatusFrame f = {};
  const char *modeText = (appMode == APP_MODE_STETHOSCOPE) ? "Audio" : (appMode == APP_MODE_PRESSURE) ? "Pressure" :
                         (appMode == APP_MODE_BOTH) ? "Both" : (appMode == APP_MODE_OFF) ? "Off" : "No sensors";
  const bool active = (appMode != APP_MODE_OFF && appMode != APP_MODE_UNAVAILABLE);
  snprintf(f.head, sizeof(f.head), "%s%s", modeText, !active ? "" : measurement_running ? " RUN" : " STOP");
  if (batteryGaugeInitialized && batteryPercent >= 0.0f) snprintf(f.pct, sizeof(f.pct), "%d%%", (int)(batteryPercent + 0.5f));

  if (appMode == APP_MODE_STETHOSCOPE || appMode == APP_MODE_BOTH)
    snprintf(f.line1, sizeof(f.line1), "%s>L%u %ukHz N%u", inputName(settings.in_src), settings.out_line, settings.fs_khz, settings.nc_mode);
  else if (appMode == APP_MODE_PRESSURE) snprintf(f.line1, sizeof(f.line1), "Low-pass %s", settings.p_lp_en ? "on" : "off");
  else if (appMode == APP_MODE_OFF)      snprintf(f.line1, sizeof(f.line1), "Audio, pressure off");
  else                                   snprintf(f.line1, sizeof(f.line1), "Check sensors");

  if (measurement_running && pressureInitialized && (appMode == APP_MODE_PRESSURE || appMode == APP_MODE_BOTH)) {
    snprintf(f.big, sizeof(f.big), "%.1f mmHg", pressure_mmHg);
  } else {
    const bool dataUsb = data_reporting && (settings.data_out & DATA_OUT_USB), dataBle = data_reporting && ble_on && (settings.data_out & DATA_OUT_BLE);
    snprintf(f.line2, sizeof(f.line2), "Data %s%s%s", (dataUsb || dataBle) ? "" : "off", dataUsb ? "USB " : "", dataBle ? "BLE" : "");
    if (batteryGaugeInitialized && batteryPercent >= 0.0f && batteryPercent < BATTERY_LOW_PERCENT) snprintf(f.line3, sizeof(f.line3), "LOW BATTERY");
    else snprintf(f.line3, sizeof(f.line3), "BLE %s", !ble_on ? "off" : ble_subscribed ? "streaming" : ble.isConnected() ? "connected" : "advertising");
  }

  if (displayContent == 1 && memcmp(&f, &last, sizeof(f)) == 0) return;      // nothing new to show
  last = f;
  displayContent = 1;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(f.head);
  if (f.pct[0]) {
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(f.pct, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(OLED_SCREEN_WIDTH - w, 0);
    display.print(f.pct);
  }
  display.setCursor(0, 8);
  display.print(f.line1);
  if (f.big[0]) {
    display.setTextSize(2);
    display.setCursor(0, 16);
    display.print(f.big);
  } else {
    display.setCursor(0, 16);
    display.print(f.line2);
    display.setCursor(0, 24);
    display.print(f.line3);
  }
  display.display();
}

void waitForButtonRelease() {
  while (digitalRead(MY_BUTTON) == LOW) {
    delay(10);
  }
  delay(50);
}

// Deep sleep, the button wakes the board (it restarts like after a reset). The audio is faded out and shut down first
// (no pop, the codec is powered down), BLE stopped, the display switched off.
void enterDeepSleep(const char *reason) {
  LOGln("%s, entering deep sleep, press the button to wake up", reason);
  oledMessage("Sleeping...", reason, "Press to wake", false);
  shutdownAudio();
  bleStop();
  Serial.flush();
  display.clearDisplay();
  display.display();
  display.ssd1306_command(SSD1306_DISPLAYOFF);

  waitForButtonRelease();

  rtc_gpio_init(MY_BUTTON);
  rtc_gpio_set_direction(MY_BUTTON, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en(MY_BUTTON);
  rtc_gpio_pulldown_dis(MY_BUTTON);

  esp_sleep_enable_ext0_wakeup(MY_BUTTON, 0);
  delay(20);
  esp_deep_sleep_start();
}

// Debounced button: a short press switches the screen (status, battery), holding it for BUTTON_LONG_PRESS_US shows
// "Release button" and releasing it goes to deep sleep.
void serviceButton(uint32_t nowUs) {
  const int reading = digitalRead(MY_BUTTON);
  if (reading != button_raw_state) { button_raw_state = reading; last_button_change_us = nowUs; }
  if (button_raw_state != button_state && (nowUs - last_button_change_us) >= BUTTON_DEBOUNCE_US) {
    button_state = button_raw_state;
    if (button_state == LOW) {
      button_pressed_us = nowUs;
      button_long_shown = false;
    } else if (button_ignore_release) {
      button_ignore_release = false;
    } else if (nowUs - button_pressed_us >= BUTTON_LONG_PRESS_US) {
      enterDeepSleep("Button");
    } else {
      screenMode = (screenMode == SCREEN_STATUS) ? SCREEN_BATTERY : SCREEN_STATUS;
      updateOLED = true;
      lastOledMs = 0;
    }
  }
  if (button_state == LOW && !button_ignore_release && !button_long_shown && (nowUs - button_pressed_us) >= BUTTON_LONG_PRESS_US) {
    oledMessage("Release button", "to go to sleep", "", false);
    button_long_shown = true;
  }
}

void printWakeReason() {
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  const char *reason = (cause == ESP_SLEEP_WAKEUP_EXT0) ? "EXT0 (button)" : "power-on/reset/other";
  LOGI("Wake reason: %s", reason);
}

bool initOLED() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    LOGE("OLED init failed");
    return false;
  }

  display.setRotation(2);
  display.clearDisplay();
  display.display();
  return true;
}

bool initPressure() {
  if (!mpr.begin()) {
    LOGE("MPRLS init failed");
    return false;
  }
  return true;
}

float measurePressureZero(int samples = 25) {
  float sum = 0.0f;
  int count = 0;

  for (int i = 0; i < samples; i++) {
    float p = mpr.readPressure();
    if (!isnan(p)) {
      sum += p;
      count++;
    }
    delay(20);
  }

  if (count == 0) return 0.0f;
  return sum / count;
}

bool initES8388() {
  if (!codecPinsConfigured) {
    codecPins.addI2C(
      audio_driver::PinFunction::CODEC,
      MY_I2C_SCL,
      MY_I2C_SDA,
      MY_ES8388ADDR,
      MY_I2CSPEED,
      Wire,
      false
    );

    codecPins.addI2S(
      audio_driver::PinFunction::CODEC, // Codec
      MY_I2S_MCLK,                      // Master Clock
      MY_I2S_BCLK,                      // Bit Clock
      MY_I2S_LRCLK,                     // Left/Right Clock
      MY_I2S_DATA_TX,                   // Data out of the ESP32 (to codec DSDIN)
      MY_I2S_DATA_RX                    // Data in of the ESP32 (from codec ASDOUT)
    );
    codecPinsConfigured = true;
  }

  audio_driver::CodecConfig cfg;
  cfg.output_device = DAC_OUTPUT_LINE2;   // the lines are set by applyRouting() after init
  cfg.input_device  = ADC_INPUT_LINE1;
  cfg.i2s.bits      = AUDIO_BIT_DEPTH;
  cfg.i2s.rate      = (settings.fs_khz == 8) ? RATE_8K : (settings.fs_khz == 32) ? RATE_32K : (settings.fs_khz == 48) ? RATE_48K : RATE_16K;   // enum of the codec driver, not Hz; the ES8388 setup does not depend on it
  cfg.i2s.channels  = AUDIO_NUM_CHANNELS;
  cfg.i2s.fmt       = I2S_NORMAL;
  cfg.i2s.mode      = MODE_SLAVE;

  if (!audioBoard.begin(cfg)) {
    LOGE("ES8388 init failed");
    return false;
  }

  codec_out_vol = codec_in_vol = -1;
  if (codecWrite(CODEC_VOLUME, settings.out_vol))    codec_out_vol = settings.out_vol;
  if (codecWrite(CODEC_IN_VOLUME, settings.in_vol)) codec_in_vol  = settings.in_vol;

  LOGI("ES8388 configured");
  return true;
}

// Creates the I2S channels: master, full duplex, Philips (I2S) format, 16 bit stereo, MCLK = 256 * fs, the ESP32 generates
// MCLK, BCLK and WS and the codec follows (slave). The channels are created stopped; the audio task enables them.
bool initI2SDriver() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num        = AUDIO_DMA_BUF_COUNT;
  chan_cfg.dma_frame_num       = AUDIO_DMA_BUF_LEN;
  chan_cfg.auto_clear_after_cb = true;             // send zeros when there is no data to send

  esp_err_t err = i2s_new_channel(&chan_cfg, &i2s_tx, &i2s_rx);
  if (err != ESP_OK) {
    LOGE("i2s_new_channel failed: %d", err);
    return false;
  }

  i2s_std_config_t std_cfg = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)settings.fs_khz * 1000),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(AUDIO_I2S_DATA_BITS, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = (gpio_num_t)MY_I2S_MCLK,
      .bclk = (gpio_num_t)MY_I2S_BCLK,
      .ws   = (gpio_num_t)MY_I2S_LRCLK,
      .dout = (gpio_num_t)MY_I2S_DATA_TX,
      .din  = (gpio_num_t)MY_I2S_DATA_RX,
      .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
    },
  };
  // Same configuration on both channels of one port: the driver makes them a full duplex pair sharing the clocks
  err = i2s_channel_init_std_mode(i2s_tx, &std_cfg);
  if (err == ESP_OK) err = i2s_channel_init_std_mode(i2s_rx, &std_cfg);
  if (err != ESP_OK) {
    LOGE("i2s_channel_init_std_mode failed: %d", err);
    i2s_del_channel(i2s_tx);
    i2s_del_channel(i2s_rx);
    i2s_tx = i2s_rx = nullptr;
    return false;
  }

  i2s_fs_khz = settings.fs_khz;
  return true;
}

// Selects the codec input line and output line from the settings and applies the input source and the subtraction mode.
// Only acts when something changed. The output is faded out before and faded in after the change; if the codec input
// or output was changed it is held at zero while the input settles.
void applyRouting() {
  if (!audioInitialized) return;
  const uint8_t adcSel   = (settings.in_src == IN_SRC_LINE2) ? ES8388_ADC_SEL_LINE2 : ES8388_ADC_SEL_LINE1;
  const uint8_t dacPower = (settings.out_line == 1)         ? ES8388_DAC_PWR_LINE1 : ES8388_DAC_PWR_LINE2;
  const bool codecChange  = (adcSel != codec_adc_sel || dacPower != codec_dac_power);
  const bool sourceChange = (settings.in_src != applied_in_src || settings.nc_mode != applied_nc_mode);
  if (!codecChange && !sourceChange) return;
  const bool running = audio_running;              // the codec is unmuted while the audio runs
  audioQuiet(true, 0);
  if (codecChange) {
    if (running) codecWrite(CODEC_MUTE, 1);
    const uint8_t regs[2][2] = {{ES8388_ADCCONTROL2_REG, adcSel}, {ES8388_DACPOWER_REG, dacPower}};
    uint8_t *const written[2] = {&codec_adc_sel, &codec_dac_power};   // what is in the codec, only updated after a successful write
    for (int r = 0; r < 2; r++) {
      if (*written[r] == regs[r][1]) continue;
      int err = -1;
      for (int attempt = 0; attempt < 3 && err != 0; attempt++) {
        Wire.beginTransmission(MY_ES8388ADDR);
        Wire.write(regs[r][0]);
        Wire.write(regs[r][1]);
        err = Wire.endTransmission();
        if (err != 0) { codec_i2c_errors = codec_i2c_errors + 1; delay(2); }
      }
      if (err == 0) *written[r] = regs[r][1];
      else          LOGE("ES8388 register 0x%02X write failed (endTransmission %d), tried again at the next change", regs[r][0], err);
    }
    if (running) codecWrite(CODEC_MUTE, 0);
  }
  applied_in_src  = settings.in_src;
  applied_nc_mode = settings.nc_mode;
  audioQuiet(false, codecChange ? AUDIO_SETTLE_MS : 0);
}

bool ensurePressureReady() {
  if (pressureInitialized) return true;

  oledMessage("Pressure mode", "Initializing...");
  if (!initPressure()) {
    oledMessage("Pressure init", "FAILED");
    return false;
  }

  oledMessage("Zeroing...", "Open cuff to air");
  delay(500);
  pressureZero = measurePressureZero();
  pressureInitialized = true;

  LOGI("Pressure subsystem ready");
  return true;
}

bool ensureAudioReady() {
  if (audioInitialized) return true;

  oledMessage("Stethoscope", "Config codec...");
  if (!initES8388()) {
    oledMessage("Codec init", "FAILED");
    return false;
  }

  oledMessage("Stethoscope", "Config I2S...");
  if (!initI2SDriver()) {
    audioBoard.end();
    oledMessage("I2S init", "FAILED");
    return false;
  }

  // Stopped and muted until the run state says otherwise, the task owns the I2S channels from here on
  codecWrite(CODEC_MUTE, 1);
  audio_running   = false;
  audio_run_req   = false;
  audio_task_exit = false;
  if (xTaskCreatePinnedToCore(audioTask, "audio", AUDIO_TASK_STACK, nullptr, AUDIO_TASK_PRIORITY, &audio_task_handle, AUDIO_TASK_CORE) != pdPASS) {
    LOGE("Audio task creation failed");
    i2s_del_channel(i2s_tx);
    i2s_del_channel(i2s_rx);
    i2s_tx = i2s_rx = nullptr;
    audioBoard.end();
    oledMessage("Audio task", "FAILED");
    return false;
  }

  audioInitialized = true;
  codec_adc_sel = codec_dac_power = 0xFF;
  applied_in_src = applied_nc_mode = 0xFF;
  applyRouting();                                  // codec is muted here
  LOGI("Audio subsystem ready");
  return true;
}

// Waits until the audio task reports the requested state
bool waitAudioRunning(bool want, uint32_t timeoutMs) {
  const uint32_t t = millis();
  while (audio_running != want) {
    if (millis() - t > timeoutMs) return false;
    delay(1);
  }
  return true;
}

// Fades the output out and waits until it is silent (quiet = true), or holds it silent for settleMs and then fades it in
// (quiet = false). Used around everything that would click: start, stop, change of input or output. Does nothing if the
// audio is not running. The output is silent for the codec changes in between.
void audioQuiet(bool quiet, uint16_t settleMs) {
  if (!audio_running) return;
  if (quiet) {
    audio_fade_target = 0.0f;
    const uint32_t t = millis();
    while (audio_fade_level > 0.0f && millis() - t < 200) delay(1);
    delay(AUDIO_DMA_DEPTH_US(i2s_fs_khz * 1000.0f) / 1000 + 5);   // the faded audio has to leave the transmit buffers
  } else {
    audio_settle_ms = settleMs;
    audio_resettle = true;
    audio_fade_target = 1.0f;
  }
}

// Starts or stops the audio data flow (the codec stays configured). Stopping: fade out, mute, stop the I2S channels.
// Starting: the task starts silent, the codec is unmuted while the output is silent, then the output fades in after the
// settling time.
void audioSetRunning(bool run) {
  if (!audioInitialized) return;
  if (run) {
    audio_fade_target = 0.0f;
    audio_run_req = true;
    if (!waitAudioRunning(true, 500)) LOGE("Audio task did not start");
    codecWrite(CODEC_MUTE, 0);
    audioQuiet(false, AUDIO_SETTLE_MS);
  } else {
    audioQuiet(true, 0);
    codecWrite(CODEC_MUTE, 1);
    audio_run_req = false;
    if (!waitAudioRunning(false, 500)) LOGE("Audio task did not stop");
  }
}

// Audio and pressure only run while measuring and while the mode includes them
void applyRunState() {
  const bool audioMode = (appMode == APP_MODE_STETHOSCOPE || appMode == APP_MODE_BOTH);
  audioSetRunning(measurement_running && audioMode);
}

void shutdownAudio() {
  if (!audioInitialized) return;

  audioSetRunning(false);
  audio_task_exit = true;                          // the task deletes itself
  for (int i = 0; i < 100 && audio_task_handle != nullptr; i++) delay(5);
  if (audio_task_handle != nullptr) {              // did not leave in time
    LOGE("Audio task did not exit");
    vTaskDelete(audio_task_handle);
    audio_task_handle = nullptr;
  }
  audio_task_exit = false;
  audio_running = false;
  if (i2s_del_channel(i2s_tx) != ESP_OK || i2s_del_channel(i2s_rx) != ESP_OK) {
    LOGE("i2s_del_channel failed");
  }
  i2s_tx = i2s_rx = nullptr;
  audioBoard.end();
  audioInitialized = false;
  resetAudioFilterState();
  LOGI("Audio subsystem stopped");
}

void setAppMode(int newMode) {
  bool pressureReady = pressureInitialized;
  bool audioReady = audioInitialized;

  if (newMode == APP_MODE_PRESSURE || newMode == APP_MODE_BOTH) {
    pressureReady = ensurePressureReady();
  }

  if (newMode == APP_MODE_STETHOSCOPE || newMode == APP_MODE_BOTH) {
    audioReady = ensureAudioReady();
  }

  int resolvedMode = APP_MODE_UNAVAILABLE;
  if (newMode == APP_MODE_OFF) {
    resolvedMode = APP_MODE_OFF;
  } else if (newMode == APP_MODE_BOTH) {
    if (pressureReady && audioReady) resolvedMode = APP_MODE_BOTH;
    else if (pressureReady) resolvedMode = APP_MODE_PRESSURE;
    else if (audioReady) resolvedMode = APP_MODE_STETHOSCOPE;
  } else if (newMode == APP_MODE_PRESSURE) {
    if (pressureReady) {
      resolvedMode = APP_MODE_PRESSURE;
    } else {
      audioReady = audioReady || ensureAudioReady();
      if (audioReady) resolvedMode = APP_MODE_STETHOSCOPE;
    }
  } else if (newMode == APP_MODE_STETHOSCOPE) {
    if (audioReady) {
      resolvedMode = APP_MODE_STETHOSCOPE;
    } else {
      pressureReady = pressureReady || ensurePressureReady();
      if (pressureReady) resolvedMode = APP_MODE_PRESSURE;
    }
  }

  if (resolvedMode != APP_MODE_STETHOSCOPE && resolvedMode != APP_MODE_BOTH) {
    shutdownAudio();
  }

  appMode = resolvedMode;
  usb_out_buff.clear();
  newPressure = false;
  applyRunState();
  lastOledMs = 0;
  lastPressureMs = 0;
  lastBatteryMs = 0;
  updateOLED = true;

  if (resolvedMode != newMode) {
    LOGW("Requested mode unavailable; using available subsystem");
  }
  LOGln("Mode active: %s", modeName(appMode));

}

// ===================== COMMANDS =====================

// Push the current settings to the DSP and the codec
void applySettings() {
  updateFilterCoefficients();
  if (audioInitialized && settings.fs_khz != i2s_fs_khz) {
    // New sample rate: stop the audio (muted, the I2S channels are disabled by the audio task), set the I2S clock of
    // both channels (MCLK = 256 * fs, BCLK and WS follow), restart if it was running. The codec follows the clocks.
    const bool wasRunning = audio_run_req;
    audioSetRunning(false);
    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)settings.fs_khz * 1000);
    if (i2s_channel_reconfig_std_clock(i2s_tx, &clk) == ESP_OK && i2s_channel_reconfig_std_clock(i2s_rx, &clk) == ESP_OK) {
      i2s_fs_khz = settings.fs_khz;
      audio_gap_us_max = 0;                          // the block time is different now
      audio_proc_us_max = 0;
    } else {
      LOGE("I2S clock change to %u kHz failed", settings.fs_khz);
    }
    if (wasRunning) audioSetRunning(true);
  }
  if (audioInitialized) {
    if (settings.out_vol != codec_out_vol && codecWrite(CODEC_VOLUME, settings.out_vol))    codec_out_vol = settings.out_vol;
    if (settings.in_vol  != codec_in_vol  && codecWrite(CODEC_IN_VOLUME, settings.in_vol)) codec_in_vol  = settings.in_vol;
    applyRouting();
  }
}

// Checks and activates new settings, returns false (and keeps the old ones) if they are not valid
bool commitSettings(const Settings &next) {
  const char *err = settingsError(next);
  if (err) {
    LOGln("Rejected: %s", err);
    return false;
  }
  settings = next;
  applySettings();
  return true;
}

bool startMeasurement() {
  if (appMode == APP_MODE_OFF || appMode == APP_MODE_UNAVAILABLE) return false;
  measurement_running = true;
  lastPressureMs = 0;
  applyRunState();
  updateOLED = true;
  return true;
}

void stopMeasurement() {
  measurement_running = false;
  applyRunState();
  usb_out_buff.clear();
  newPressure = false;
  updateOLED = true;
}

void handleGeneralCommand(char c) {
  switch (c) {
    case '.':
      if (measurement_running) { stopMeasurement(); LOGln("Stopped"); }
      else                     { LOGln(startMeasurement() ? "Started" : "Nothing to start in mode %s, use m1, m2 or m3", modeName(appMode)); }
      break;
    case '>':
      if (measurement_running) LOGln("Already running");
      else                     LOGln(startMeasurement() ? "Started" : "Nothing to start in mode %s, use m1, m2 or m3", modeName(appMode));
      break;
    case '<':
      if (!measurement_running) LOGln("Already stopped");
      else                      { stopMeasurement(); LOGln("Stopped"); }
      break;
    case '?': helpMenu(); break;
    case 's': printAllSettings(); break;
    case 'i': printSystemInfo(); break;
    case 'z':
      data_reporting = !data_reporting;
      if (data_reporting && settings.data_out == 0) settings.data_out = DATA_OUT_ALL;   // no link selected: use all
      LOGln("Data reporting: %s, USB %s, BLE %s", data_reporting ? "ON" : "OFF",
            (settings.data_out & DATA_OUT_USB) ? "on" : "off", (settings.data_out & DATA_OUT_BLE) ? "on" : "off");
      break;
    case 'x':
      LOGln("Rebooting");
      if (audioInitialized) { audioQuiet(true, 0); codecWrite(CODEC_MUTE, 1); }
      delay(100);
      ESP.restart();
      break;
    default:
      LOGln("Unknown command '%c', ? for help", c);
      break;
  }
}

// Cutoff step: multiply/divide by a factor, at least 1 Hz, kept inside the valid range
float stepCutoff(float fc, int direction, float minHz, float maxHz) {
  float n = (direction > 0) ? fmaxf(fc * FILTER_STEP_FACTOR, fc + 1.0f)
                            : fminf(fc / FILTER_STEP_FACTOR, fc - 1.0f);
  return clampf(roundf(n), minHz, maxHz);
}

// F: FH/Fh high-pass, FL/Fl low-pass. Upper case: on, lower case: off.
// No number: switch. Number: upper case sets the cutoff and switches on, lower case sets the cutoff only. + / - step the cutoff.
void handleFilterCommand(const char *cmd) {
  const char kind = cmd[1];
  const bool hp = (kind == 'H' || kind == 'h');
  const bool lp = (kind == 'L' || kind == 'l');
  if (!hp && !lp) { LOGln("Usage: FH/Fh high-pass, FL/Fl low-pass, see ? for details"); return; }
  const bool upper = (kind == 'H' || kind == 'L');
  const char *arg = cmd + 2;
  Settings next = settings;
  bool &en = hp ? next.hp_en : next.lp_en;
  float &fc = hp ? next.hp_fc : next.lp_fc;
  const float fs = settings.fs_khz * 1000.0f;
  const float minHz = hp ? HP_CUTOFF_MIN_HZ : LP_CUTOFF_MIN_HZ(fs);
  const float maxHz = hp ? HP_CUTOFF_MAX_HZ(fs) : LP_CUTOFF_MAX_HZ(fs);

  if (*arg == '\0') {
    en = upper;
  } else if ((arg[0] == '+' || arg[0] == '-') && arg[1] == '\0') {
    fc = stepCutoff(fc, arg[0] == '+' ? +1 : -1, minHz, maxHz);
  } else {
    double v[1];
    if (parseNumbers(arg, v, 1) != 1) { LOGln("Usage: %s[Hz], %s+, %s-", hp ? "FH/Fh" : "FL/Fl", hp ? "Fh" : "Fl", hp ? "Fh" : "Fl"); return; }
    if (v[0] < minHz || v[0] > maxHz) { LOGln("Rejected: %s cutoff must be %.0f..%.0f Hz", hp ? "high-pass" : "low-pass", minHz, maxHz); return; }
    fc = (float)v[0];
    if (upper) en = true;
  }
  if (!commitSettings(next)) return;
  if (hp) LOGln("High-pass filter = %s @ %.1f Hz", settings.hp_en ? "ON" : "OFF", settings.hp_fc);
  else    LOGln("Low-pass filter = %s @ %.1f Hz",  settings.lp_en ? "ON" : "OFF", settings.lp_fc);
}

// G: Gi hardware input gain, Go hardware output gain, Gs software gain
void handleGainCommand(const char *cmd) {
  const char kind = cmd[1];
  const char *arg = cmd + 2;
  const bool step = ((arg[0] == '+' || arg[0] == '-') && arg[1] == '\0');
  const int dir = (arg[0] == '+') ? +1 : -1;
  Settings next = settings;
  double v[1];

  if (kind == 'i') {
    if (step) {
      int idx = constrain(inputGainIndex(next.in_vol) + dir, 0, 8);
      next.in_vol = inputVolLevels[idx];
    } else if (parseNumbers(arg, v, 1) == 1 && v[0] >= 0 && v[0] <= 8 && v[0] == floor(v[0])) {
      next.in_vol = inputVolLevels[(int)v[0]];
    } else { LOGln("Usage: Gi<0-8>, Gi+, Gi-"); return; }
  } else if (kind == 'o') {
    if (step) {
      next.out_vol = (int)clampf((float)(next.out_vol + dir * AUDIO_OUTPUT_VOLUME_STEP), 0.0f, 100.0f);
    } else if (parseNumbers(arg, v, 1) == 1 && v[0] >= 0 && v[0] <= 100) {
      next.out_vol = (int)lrint(v[0]);
    } else { LOGln("Usage: Go<0-100>, Go+, Go-"); return; }
  } else if (kind == 's') {
    if (step) {
      next.sw_gain = clampf(dir > 0 ? next.sw_gain * SW_GAIN_STEP_FACTOR : next.sw_gain / SW_GAIN_STEP_FACTOR, SW_GAIN_MIN, SW_GAIN_MAX);
    } else if (parseNumbers(arg, v, 1) == 1) {
      next.sw_gain = (float)v[0];
    } else { LOGln("Usage: Gs<%.1f-%.1f>, Gs+, Gs-", SW_GAIN_MIN, SW_GAIN_MAX); return; }
  } else {
    LOGln("Usage: Gi (HW input gain), Go (HW output gain), Gs (software gain)");
    return;
  }
  if (!commitSettings(next)) return;
  if (kind == 's') LOGln("Software gain = %.2f", settings.sw_gain);
  else             printCodecVolumeStatus(settings);
}

// N<0|1|2> subtraction mode, Ng<x> gain on the subtracted (left, background) channel, Nb<x> bias in counts
void handleSubtractCommand(const char *cmd) {
  Settings next = settings;
  double v[1];
  if (cmd[1] == 'g') {
    if (parseNumbers(cmd + 2, v, 1) != 1) { LOGln("Usage: Ng<%.1f-%.1f>", NOISE_LEFT_SCALE_MIN, NOISE_LEFT_SCALE_MAX); return; }
    next.nc_gain = (float)v[0];
  } else if (cmd[1] == 'b') {
    if (parseNumbers(cmd + 2, v, 1) != 1) { LOGln("Usage: Nb<%.0f..%.0f> counts", -NC_BIAS_MAX, NC_BIAS_MAX); return; }
    next.nc_bias = (float)v[0];
  } else if (parseNumbers(cmd + 1, v, 1) == 1 && (v[0] == 0 || v[0] == 1 || v[0] == 2)) {
    next.nc_mode = (uint8_t)v[0];
  } else { LOGln("Usage: N0 off, N1 result in both ears, N2 left original and right result; Ng<x> gain; Nb<x> bias"); return; }
  if (commitSettings(next)) LOGln("Subtraction R-L = %s, left gain %.2f, bias %.0f",
                                  settings.nc_mode == 0 ? "OFF" : settings.nc_mode == 1 ? "ON, result in both ears" : "ON, left original and right result",
                                  settings.nc_gain, settings.nc_bias);
}

// L: La audio decimation, Ln BLE name, Ls serial baud rate
void handleLinkCommand(const char *cmd) {
  Settings next = settings;
  double v[1];
  if (cmd[1] == 'a') {
    if (parseNumbers(cmd + 2, v, 1) != 1 || v[0] < 1 || v[0] > AUDIO_DECIMATION_MAX || v[0] != floor(v[0])) {
      LOGln("Usage: La<1-%d>", AUDIO_DECIMATION_MAX); return;
    }
    next.audio_decim = (uint16_t)v[0];
    if (commitSettings(next)) LOGln("Audio decimation %u, output rate %.0f lines/s per link", settings.audio_decim, settings.fs_khz * 1000.0f / settings.audio_decim);
  } else if (cmd[1] == 's') {
    if (parseNumbers(cmd + 2, v, 1) != 1 || v[0] < 9600 || v[0] > 2000000) { LOGln("Usage: Ls<baud>, 9600..2000000"); return; }
    settings.serial_baud = (uint32_t)v[0];
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
    LOGln("Serial baud rate %lu stored, but native USB has no baud rate, it only matters on boards with a UART bridge", (unsigned long)settings.serial_baud);
#else
    LOGln("Serial baud rate %lu, save with Ps and reboot with x to apply, then reconnect the terminal at the new rate", (unsigned long)settings.serial_baud);
#endif
  } else if (cmd[1] == 'b' && cmd[2] == 's' && cmd[3] == '\0') {   // Lbs: status of the BLE library (printed on USB)
    if (ble_on) ble.printStats(Serial);
    else        LOGln("BLE is stopped");
  } else if (cmd[1] == 'b') {
    if (parseNumbers(cmd + 2, v, 1) != 1 || (v[0] != 0 && v[0] != 1)) { LOGln("Usage: Lb0 or Lb1, BLE off/on at boot"); return; }
    settings.ble_en = (v[0] == 1);
    if (settings.ble_en) {
      bleStart();
      LOGln(ble_on ? "BLE started as '%s', save with Ps to start it at boot" : "BLE start failed", settings.ble_name);
    } else {
      LOGln("BLE stopped, save with Ps to keep it off after a reboot");   // before the stop: this may be the reply to a BLE command
      bleStop();
    }
  } else if (cmd[1] == 'p') {   // Lp: pressure output decimation
    if (parseNumbers(cmd + 2, v, 1) != 1 || v[0] < 1 || v[0] > P_DECIM_MAX || v[0] != floor(v[0])) { LOGln("Usage: Lp<1-%d>", P_DECIM_MAX); return; }
    next.p_decim = (uint16_t)v[0];
    if (commitSettings(next)) LOGln("Pressure output every %u-th reading, %.1f values per second", settings.p_decim, 1000.0f / MPRLS_INTERVAL_MS / settings.p_decim);
  } else if (cmd[1] == 'm') {   // BLE mode, BLE is restarted if it runs
    if (parseNumbers(cmd + 2, v, 1) != 1 || v[0] < 0 || v[0] > 3 || v[0] != floor(v[0])) { LOGln("Usage: Lm0 Fast, Lm1 LowPower, Lm2 LongRange, Lm3 Balanced"); return; }
    settings.ble_mode = (uint8_t)v[0];
    LOGln("BLE mode = %s, %s", bleModeName(settings.ble_mode), ble_on ? "BLE restarts" : "save with Ps");   // before the restart: this may be the reply to a BLE command
    if (ble_on) { bleStop(); bleStart(); }
  } else if (cmd[1] == 't') {   // BLE transmit power
    const bool autoPwr = (cmd[2] == 'a' && cmd[3] == '\0');
    if (!autoPwr && (parseNumbers(cmd + 2, v, 1) != 1 || v[0] < BLE_PWR_MIN || v[0] > BLE_PWR_MAX || v[0] != floor(v[0]))) {
      LOGln("Usage: Lt<%d..%d dBm>, Lta = default of the BLE mode", BLE_PWR_MIN, BLE_PWR_MAX); return;
    }
    settings.ble_pwr = autoPwr ? (int8_t)BLE_PWR_AUTO : (int8_t)v[0];
    if (autoPwr) LOGln("BLE power = default of the mode, %s", ble_on ? "BLE restarts" : "save with Ps");
    else         LOGln("BLE power = %d dBm, %s", settings.ble_pwr, ble_on ? "applied now" : "save with Ps");
    if (ble_on && autoPwr) { bleStop(); bleStart(); }
    else if (ble_on)       ble.setPower(settings.ble_pwr, PWR_ALL);
  } else if (cmd[1] == 'n') {
    const char *name = cmd + 2;
    while (*name == ' ') name++;
    if (!validBleName(name)) { LOGln("Usage: Ln<name>, 1..%d printable ASCII characters", BLE_NAME_MAX); return; }
    strlcpy(settings.ble_name, name, sizeof(settings.ble_name));
    LOGln("BLE name '%s', save with Ps; Lb0 then Lb1 applies it now", settings.ble_name);
  } else {
    LOGln("Usage: La<n> audio decimation, Lb<0|1> BLE off/on, Lm<0-3> BLE mode, Lt<dBm> BLE power, Ln<name> BLE name, Ls<baud> serial baud rate");
  }
}

// Q: pressure. Qe<0|1> low-pass off/on, Qf<Hz> low-pass cutoff, Qc<x> calibration factor, Qz zero now (cuff open to air)
void handlePressureCommand(const char *cmd) {
  Settings next = settings;
  double v[1];
  if (cmd[1] == 'z' && cmd[2] == '\0') {
    if (!pressureInitialized) { LOGln("Pressure sensor not started, use m1 or m3"); return; }
    LOGln("Zeroing, keep the cuff open to air");
    pressureZero = measurePressureZero();          // takes half a second
    pressureFilterValid = false;                   // the low-pass starts again with the next reading
    LOGln("Pressure zero = %.2f", pressureZero);
    return;
  } else if (cmd[1] == 'e') {
    if (parseNumbers(cmd + 2, v, 1) != 1 || (v[0] != 0 && v[0] != 1)) { LOGln("Usage: Qe0 or Qe1, pressure low-pass off/on"); return; }
    next.p_lp_en = (v[0] == 1);
  } else if (cmd[1] == 'f') {
    if (parseNumbers(cmd + 2, v, 1) != 1) { LOGln("Usage: Qf<%.1f-%.1f Hz>", P_LP_FC_MIN, P_LP_FC_MAX); return; }
    next.p_lp_fc = (float)v[0];
  } else if (cmd[1] == 'c') {
    if (parseNumbers(cmd + 2, v, 1) != 1) { LOGln("Usage: Qc<%.1f-%.1f>, mmHg = (reading - zero) / Qc", P_CAL_MIN, P_CAL_MAX); return; }
    next.p_cal = (float)v[0];
  } else { LOGln("Usage: Qe<0|1> low-pass, Qf<Hz> cutoff, Qc<x> calibration factor, Qz zero now"); return; }
  if (commitSettings(next)) LOGln("Pressure low-pass %s @ %.1f Hz, calibration factor %.4f, save with Ps", settings.p_lp_en ? "ON" : "OFF", settings.p_lp_fc, settings.p_cal);
}

// a<0-3> input source, o<1|2> output line, Tl / Tr / Tn simulated tones and noise
void handleRoutingCommand(const char *cmd) {
  Settings next = settings;
  double v[2];
  if (cmd[0] == 'a') {
    if (parseNumbers(cmd + 1, v, 1) != 1 || v[0] < IN_SRC_LINE1 || v[0] > IN_SRC_TONES_NOISE || v[0] != floor(v[0])) { LOGln("Usage: a1 Line 1, a2 Line 2, a3 simulated tones, a4 simulated tones + noise"); return; }
    next.in_src = (uint8_t)v[0];
    if (commitSettings(next)) LOGln("Input source = %s", settings.in_src == IN_SRC_LINE1 ? "Line 1" : settings.in_src == IN_SRC_LINE2 ? "Line 2" :
                                                      settings.in_src == IN_SRC_TONES ? "simulated tones" : "simulated tones + white noise");
  } else if (cmd[0] == 'o') {
    if (parseNumbers(cmd + 1, v, 1) != 1 || (v[0] != 1 && v[0] != 2)) { LOGln("Usage: o1 Line 1 (line out), o2 Line 2 (headphone)"); return; }
    next.out_line = (uint8_t)v[0];
    if (commitSettings(next)) LOGln("Output line = Line %u", settings.out_line);
  } else if (cmd[1] == 'n') {   // Tn<amp>
    if (parseNumbers(cmd + 2, v, 1) != 1 || v[0] < 0 || v[0] > SIM_AMP_MAX) { LOGln("Usage: Tn<0-%d> peak counts", SIM_AMP_MAX); return; }
    next.noise_amp = (uint16_t)lrint(v[0]);
    if (commitSettings(next)) LOGln("Simulated noise amplitude = %u", settings.noise_amp);
  } else if (cmd[1] == 'l' || cmd[1] == 'r') {   // Tl / Tr <Hz>[,<amp>]
    const int n = parseNumbers(cmd + 2, v, 2);
    const float toneMax = SIM_TONE_HZ_MAX(settings.fs_khz * 1000.0f);
    if (n < 1 || v[0] < SIM_TONE_HZ_MIN || v[0] > toneMax || (n == 2 && (v[1] < 0 || v[1] > SIM_AMP_MAX))) {
      LOGln("Usage: Tl or Tr <%.0f-%.0f Hz>[,<0-%d peak counts>]", SIM_TONE_HZ_MIN, toneMax, SIM_AMP_MAX); return;
    }
    if (cmd[1] == 'l') { next.tone_l_hz = (float)v[0]; if (n == 2) next.tone_l_amp = (uint16_t)lrint(v[1]); }
    else               { next.tone_r_hz = (float)v[0]; if (n == 2) next.tone_r_amp = (uint16_t)lrint(v[1]); }
    if (commitSettings(next)) LOGln("Simulated tones: left %.1f Hz amp %u, right %.1f Hz amp %u", settings.tone_l_hz, settings.tone_l_amp, settings.tone_r_hz, settings.tone_r_amp);
  } else {
    LOGln("Usage: Tl<Hz>[,<amp>], Tr<Hz>[,<amp>], Tn<amp>");
  }
}

// P: persistence
void handlePersistenceCommand(const char *cmd) {
  if (cmd[2] != '\0') { LOGln("Usage: Ps (save), Pl (load), Pd (print saved), Pc (clear saved), Pr (reset to factory defaults)"); return; }
  switch (cmd[1]) {
    case 's':
      LOGln(saveSettings(settings) ? "Settings saved" : "Saving settings failed");
      break;
    case 'l': {
      Settings next = settings;
      if (!readSettings(next)) { LOGln("No saved settings"); break; }
      if (commitSettings(next)) { LOGln("Settings loaded"); printSettings(settings); }
      break;
    }
    case 'd': {
      Settings saved; setDefaults(saved);
      if (!readSettings(saved)) { LOGln("No saved settings"); break; }
      LOGln("Saved settings:");
      printSettings(saved);
      break;
    }
    case 'c':
      LOGln(clearSettings() ? "Saved settings cleared" : "Clearing settings failed");
      break;
    case 'r': {
      Settings d; setDefaults(d);
      d.log_level = settings.log_level;   // keep the current log level and link identity until reboot
      if (commitSettings(d)) { applyLogLevel(settings.log_level); LOGln("Factory defaults active, save with Ps"); printSettings(settings); }
      break;
    }
    default:
      LOGln("Usage: Ps (save), Pl (load), Pd (print saved), Pc (clear saved), Pr (reset to factory defaults)");
      break;
  }
}

void handleParameterCommand(const char *cmd) {
  double v[1];
  switch (cmd[0]) {
    case 'm':
      if (parseNumbers(cmd + 1, v, 1) != 1 || (v[0] != 0 && v[0] != 1 && v[0] != 2 && v[0] != 3)) { LOGln("Usage: m0 off, m1 pressure, m2 audio, m3 both"); return; }
      settings.mode = (uint8_t)v[0];
      measurement_running = (settings.mode != APP_MODE_OFF);   // a mode change starts measuring, m0 stops
      setAppMode(settings.mode);
      break;
    case 'A':
      if (parseNumbers(cmd + 1, v, 1) != 1 || (v[0] != 0 && v[0] != 1)) { LOGln("Usage: A0 or A1, start measuring at boot off/on"); return; }
      settings.autostart = (v[0] == 1);
      LOGln("Autostart = %s, save with Ps", settings.autostart ? "ON" : "OFF");
      break;
    case 'z': {   // data output links: lower case switches a link off, upper case switches it on and starts the data output
      uint8_t mask = settings.data_out;
      bool report = data_reporting;
      if (cmd[2] != '\0') { LOGln("Data reporting commands must be exactly two characters"); return; }
      switch (cmd[1]) {
        case 's': mask &= (uint8_t)~DATA_OUT_USB; break;
        case 'b': mask &= (uint8_t)~DATA_OUT_BLE; break;
        case 'a': mask = 0; break;
        case 'S': mask |= DATA_OUT_USB; report = true; break;
        case 'B': mask |= DATA_OUT_BLE; report = true; break;
        case 'A': mask = DATA_OUT_ALL;  report = true; break;
        default:  LOGln("Unknown data reporting command, z[s|b|a] switch off, z[S|B|A] switch on USB, BLE, all"); return;
      }
      if (mask == 0) report = false;
      settings.data_out = mask;
      data_reporting = report;
      LOGln("Data reporting: %s, USB %s, BLE %s", data_reporting ? "ON" : "OFF",
            (mask & DATA_OUT_USB) ? "on" : "off", (mask & DATA_OUT_BLE) ? "on" : "off");
      break;
    }
    case 'R': {   // sample rate in kHz; cutoffs and tones are moved into the range of the new rate
      if (parseNumbers(cmd + 1, v, 1) != 1 || (v[0] != 8 && v[0] != 16 && v[0] != 32 && v[0] != 48)) { LOGln("Usage: R8, R16, R32 or R48 (sample rate in kHz)"); return; }
      Settings next = settings;
      next.fs_khz = (uint8_t)v[0];
      const float fs = next.fs_khz * 1000.0f;
      next.hp_fc     = clampf(next.hp_fc,     HP_CUTOFF_MIN_HZ,    HP_CUTOFF_MAX_HZ(fs));
      next.lp_fc     = clampf(next.lp_fc,     LP_CUTOFF_MIN_HZ(fs), LP_CUTOFF_MAX_HZ(fs));
      next.tone_l_hz = clampf(next.tone_l_hz, SIM_TONE_HZ_MIN,     SIM_TONE_HZ_MAX(fs));
      next.tone_r_hz = clampf(next.tone_r_hz, SIM_TONE_HZ_MIN,     SIM_TONE_HZ_MAX(fs));
      if (commitSettings(next))
        LOGln("Sample rate = %u kHz: high-pass %.1f Hz, low-pass %.1f Hz, data output %.0f lines/s per link (La %u), save with Ps",
              settings.fs_khz, settings.hp_fc, settings.lp_fc, fs / settings.audio_decim, settings.audio_decim);
      break;
    }
    case 'F': handleFilterCommand(cmd);      break;
    case 'G': handleGainCommand(cmd);        break;
    case 'N': handleSubtractCommand(cmd);    break;
    case 'a':
    case 'o':
    case 'T': handleRoutingCommand(cmd);     break;
    case 'Q': handlePressureCommand(cmd);    break;
    case 'L': handleLinkCommand(cmd);        break;
    case 'P': handlePersistenceCommand(cmd); break;
    case 'l': {
      if (parseNumbers(cmd + 1, v, 1) != 1 || v[0] < LOG_LEVEL_DEBUG || v[0] > LOG_LEVEL_NONE || v[0] != floor(v[0])) {
        LOGln("Usage: l<0-5>  0 debug, 1 info, 2 warning, 3 error, 4 critical, 5 none"); return;
      }
      settings.log_level = (uint8_t)v[0];
      applyLogLevel(settings.log_level);
      LOGln("Log level %u", settings.log_level);
      break;
    }
    default:
      LOGln("Unknown command '%s', ? for help", cmd);
      break;
  }
}

void processCommand(const char *input, CommandSource source) {
  while (*input == ' ' || *input == '\t') input++;
  char cmd[128];
  size_t len = strnlen(input, sizeof(cmd) - 1);
  memcpy(cmd, input, len);
  cmd[len] = '\0';
  while (len > 0 && (cmd[len - 1] == ' ' || cmd[len - 1] == '\t')) cmd[--len] = '\0';
  if (len == 0) return;
  logSetOutput(source == CommandSource::Ble ? (Print &)ble : (Print &)Serial);   // answer on the link the command came from
  if (len == 1) handleGeneralCommand(cmd[0]);
  else          handleParameterCommand(cmd);
  logSetOutput(Serial);
}


// Serial.setTxBufferSize() exists for UART and hardware CDC, not for TinyUSB CDC (ESP32-S3 with USB mode TinyUSB)
#if !(defined(CONFIG_IDF_TARGET_ESP32S3) && defined(ARDUINO_USB_MODE) && (ARDUINO_USB_MODE == 0) && ARDUINO_USB_CDC_ON_BOOT)
  #define SERIAL_HAS_TX_BUFFER_SIZE 1
#endif

void setup() {
#ifdef SERIAL_HAS_TX_BUFFER_SIZE
  Serial.setTxBufferSize(4096);   // must be before begin(), more room for the data between two passes of the main loop
#endif
  Serial.begin(BAUDRATE);
  delay(500);
  logSetOutput(Serial);

  // Settings -------------------
  setDefaults(settings);
  const bool loaded = readSettings(settings);
  applyLogLevel(settings.log_level);
#if !defined(ARDUINO_USB_CDC_ON_BOOT) || !ARDUINO_USB_CDC_ON_BOOT   // native USB CDC has no baud rate
  if (settings.serial_baud != BAUDRATE) Serial.updateBaudRate(settings.serial_baud);
#endif
  printWakeReason();
  if (loaded) LOGI("Loaded saved settings");
  else        LOGI("No saved settings, using defaults");
  updateFilterCoefficients();

  pinMode(MY_BUTTON, INPUT_PULLUP);
  button_raw_state = button_state = digitalRead(MY_BUTTON);
  button_ignore_release = (button_state == LOW);      // still held from the wake up
  last_button_change_us = micros();
  Wire.begin(MY_I2C_SDA, MY_I2C_SCL);
  Wire.setClock(MY_I2CSPEED);

  if (!initOLED()) {
    while (true) delay(100);
  }

  batteryGaugeInitialized = initBatteryGauge();

  // BLE Serial: a background task moves the data between the BLE stack and the BLESerial buffers
  // BLE can be switched off (Lb0): no advertising, no radio activity next to the microphone preamplifier.
  if (settings.ble_en) bleStart();
  else                 LOGI("BLE is off (Lb1 starts it)");

  helpMenu();
  LOGI("Booting in configured default mode");
  measurement_running = settings.autostart && settings.mode != APP_MODE_OFF;
  setAppMode(settings.mode);
  awakeStartMs = millis();
}

void loop() {
  serviceButton(micros());
  ble_subscribed = ble_on && ble.isSubscribed();
  if (ble_on && ble_lr.poll(ble, ble_input_buff, sizeof(ble_input_buff))) {
    processCommand(ble_input_buff, CommandSource::Ble);
  }
  if (serial_lr.poll(Serial, serial_input_buff, sizeof(serial_input_buff))) {
    processCommand(serial_input_buff, CommandSource::Usb);
  }

  unsigned long currentTime = millis();

  // ------------------------------------

  if (currentTime - awakeStartMs >= AUTO_SHUTOFF_MS) {
     enterDeepSleep("Auto shutoff");
  }

  // ------------------------------------
  // Queue pressure data if data reporting is enabled (in both mode the audio task adds it to its lines)
  if (data_reporting && measurement_running && pressureInitialized && appMode == APP_MODE_PRESSURE && newPressure) {
    newPressure = false;
    int n = snprintf(line, sizeof(line), "%.1f\r\n", pressure_mmHg);
    queueLine(line, n);
  } // end plot pressure

  // ------------------------------------
  // Read pressure data
  if (
      measurement_running &&
      (appMode == APP_MODE_PRESSURE || appMode == APP_MODE_BOTH) &&
      pressureInitialized &&
      currentTime - lastPressureMs >= MPRLS_INTERVAL_MS
     )
  {

    // The time since the last reading is used for the low-pass, the loop does not run at exactly the reading interval.
    // lastPressureMs is 0 after a start or a mode change: the low-pass starts again with the first reading.
    const float dt = (lastPressureMs != 0) ? (currentTime - lastPressureMs) * 0.001f : MPRLS_INTERVAL_MS * 0.001f;
    if (lastPressureMs == 0) pressureFilterValid = false;
    lastPressureMs = currentTime;

    pressureRaw = mpr.readPressure();                 // mmHg for this sensor (see the constructor settings)
    if (isnan(pressureRaw)) {
      oledMessage("Pressure error", "Read failed");
      return;
    }

    // mmHg = (reading - zero) / calibration factor, then the low-pass (first order: y += alpha * (x - y),
    // alpha = 1 - exp(-2 * pi * fc * dt)), then values below 0 are shown as 0
    float p = (pressureRaw - pressureZero) / settings.p_cal;
    if (settings.p_lp_en) {
      if (!pressureFilterValid) { pressureFiltered = p; pressureFilterValid = true; }
      else                      pressureFiltered += (1.0f - expf(-2.0f * PI * settings.p_lp_fc * dt)) * (p - pressureFiltered);
      p = pressureFiltered;
    } else {
      pressureFilterValid = false;
    }
    pressure_mmHg = (p < 0.0f) ? 0.0f : p;

    if (++pressureDecimCount >= settings.p_decim) {   // every p_decim-th reading is output
      pressureDecimCount = 0;
      newPressure = true;
    }
    updateOLED = true;
  } // end pressure reading

  // ------------------------------------
  // Read battery level
  if (batteryGaugeInitialized && currentTime - lastBatteryMs >= BATTERY_INTERVAL_MS) {
    lastBatteryMs = currentTime;
    const float pct = maxlipo.cellPercent(), volt = maxlipo.cellVoltage(), rate = maxlipo.chargeRate();
    if (!isnan(pct))     batteryPercent = clampf(pct, 0.0f, 100.0f);
    if (isfinite(volt))  batteryVoltage = volt;
    if (isfinite(rate))  batteryChargeRate = rate;
    updateOLED = true;                              // the screens only write the display when a value changed
    static bool lowLogged = false;
    const bool low = (batteryPercent >= 0.0f && batteryPercent < BATTERY_LOW_PERCENT);
    if (low && !lowLogged) LOGW("Low battery: %.0f %%", batteryPercent);
    lowLogged = low;

    // Battery empty: shut down to protect the battery and to avoid a brown-out of the 3.3 V rail. Not while a USB host is
    // plugged in (Serial.isPlugged(), a power-only charger has no host) and not while the battery charges. The condition
    // has to hold for BATTERY_SHUTDOWN_HOLD_MS. A voltage below 2.5 V is a gauge without battery and is ignored.
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT && defined(ARDUINO_USB_MODE) && ARDUINO_USB_MODE == 1
    const bool usbHost = Serial.isPlugged();
#else
    const bool usbHost = false;                     // can not tell with this USB mode, only the charge rate is used
#endif
    const bool charging = isfinite(batteryChargeRate) && batteryChargeRate > 0.0f;
    const bool empty = (batteryPercent >= 0.0f && batteryPercent <= BATTERY_SHUTDOWN_PERCENT) ||
                       (isfinite(batteryVoltage) && batteryVoltage > 2.5f && batteryVoltage <= BATTERY_SHUTDOWN_VOLT);
    static uint32_t emptySinceMs = 0;
    if (!empty || usbHost || charging) {
      emptySinceMs = 0;
    } else if (emptySinceMs == 0) {
      emptySinceMs = currentTime | 1UL;             // |1: never 0, 0 means "not empty"
      LOGW("Battery empty (%.0f %%, %.2f V), shutting down in %lu s unless USB or a charger is connected", batteryPercent, batteryVoltage, BATTERY_SHUTDOWN_HOLD_MS / 1000UL);
    } else if (currentTime - emptySinceMs >= BATTERY_SHUTDOWN_HOLD_MS) {
      enterDeepSleep("Battery empty");
    }
  } // end battery read

  // ------------------------------------
  // Update the OLED display
  if (currentTime - lastOledMs >= OLED_INTERVAL_MS) {
    lastOledMs = currentTime;
    if (updateOLED) {
      updateOLED = false;
      if (screenMode == SCREEN_BATTERY) drawBatteryScreen();
      else                              drawStatusScreen();
    }
  } // end OLED update

  // ------------------------------------
  // Output
  drainUsb();
  drainBle();

  // BLE watchdog: data is on, a client is subscribed, the BLE output buffer is at least half full and no byte went out for
  // BLE_STALL_MS. The BLE sender is stuck (e.g. after an overload), restart BLE; the client has to connect again.
  static uint32_t bleProgressMs = 0, bleLastBytes = 0;
  if (!data_reporting || !ble_subscribed || ble_bytes_sent != bleLastBytes || ble_out_buff.available() < OUT_BUFFER_SIZE / 2) {
    bleLastBytes = ble_bytes_sent;
    bleProgressMs = currentTime;
  } else if (currentTime - bleProgressMs > BLE_STALL_MS) {
    LOGW("BLE transmit stalled, restarting BLE (connect the client again)");
    bleStop();
    bleStart();
    ble_stall_restarts = ble_stall_restarts + 1;
    bleLastBytes = ble_bytes_sent;
    bleProgressMs = currentTime;
  }
  if (!measurement_running || appMode == APP_MODE_OFF) delay(1);   // idle, give other tasks time
  else                                                 yield();

} // end loop()
