/*
  MediBrick Stethoscope + Blood Pressure
  ESP32-S3 Feather

  Runtime Serial Commands:
    p = Pressure only
    s = Stethoscope only
    b = Both (pressure on OLED + audio active)
    ? = Help
    < = output volume down
    > = output volume up
    , = input volume down
    . = input volume up
    + = software gain up
    - = software gain down
    L/l = low-pass cutoff up/down
    H/h = high-pass cutoff up/down
    G/g = high-pass ON/OFF
    K/k = low-pass ON/OFF
    N/n = noise cancel ON/OFF
    C/c = noise left scale up/down
    d = cycle plot decimation (1/2/4/8)
    j = save settings to flash
    J = load settings from flash
    v = print volumes

  Default boot mode: BOTH

  Future releases should migrate of driver/i2s.h

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
#include "driver/i2s.h"
#include "AudioBoard.h"
#include <Adafruit_MAX1704X.h>
#include <Preferences.h>
#include <math.h>

#define APP_VERSION 1.0
#define BAUDRATE 500000
char line[128];

// ===================== APP MODES =====================
#define APP_MODE_PRESSURE                  1
#define APP_MODE_STETHOSCOPE               2
#define APP_MODE_BOTH                      3
#define APP_MODE_NONE                      0

int appMode = APP_MODE_BOTH;   // default autonomous mode

// ===================== PIN MAP =====================
static const int MY_I2C_SDA    =         SDA;
static const int MY_I2C_SCL    =         SCL;
static const int MY_I2CSPEED   =      400000; // Clock Rate

static const int MY_ES8388ADDR =        0x10; // Address of ES8388 I2C port

static const gpio_num_t MY_BUTTON = GPIO_NUM_12; // Feather 12, Thing+ 9 active LOW

static const int MY_MPRLS_EOC  =          10; // Feather D10
static const int MY_MPRLS_RST  =          11; // Feather D11

static const int MY_I2S_MCLK   =          14; // Feather 14,   A4, Thing+ 13 Master Clock
static const int MY_I2S_BCLK   =          36; // Feather 36,  SCK, Thing+ 18 Bit Clock
static const int MY_I2S_LRCLK  =           8; // Feather  8,   A5, Thing+ 35 Left/Right Clock / Word Select
// Assembled with switched in/out
static const int MY_I2S_DOUT   =          37; // Feather 37, MISO, Thing+ 19 This is connected to DI on ES8388 (MISO)
static const int MY_I2S_DIN    =          35; // Feather 35, MOSI, Thing+ 23 This is connected to DO on ES8388 (MOSI)

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

// ===================== OLED =====================
#define OLED_SCREEN_WIDTH                128
#define OLED_SCREEN_HEIGHT                32
#define OLED_RESET                        -1
#define OLED_ADDR                       0x3C
#define OLED_INTERVAL_MS                 200
#define BATTERY_INTERVAL_MS             5000

Adafruit_SSD1306 display(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, &Wire, OLED_RESET);

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

bool          newPressure          =   false;
float         pressureZero         =    0.0f;
bool          pressureInitialized  =   false;
unsigned long awakeStartMs         =       0;
bool          autoShutoffTriggered =   false;


// ===================== ES8388 / I2S =====================
#define I2S_PORT                   I2S_NUM_0
// RATE_8k, RATE_11k, RATE_16k, RATE_22k,
// RATE_32k, RATE_44k, RATE_48k, RATE_64k,
// RATE_88k, RATE_96k, RATE_128k, RATE_176k, RATE_192k
#define AUDIO_SAMPLE_RATE           RATE_16K
#define AUDIO_DMA_BUF_LEN                128
#define AUDIO_DMA_BUF_COUNT                8
#define AUDIO_BLOCK_FRAMES               256
// CHANNELS2, CHANNELS9, CHANNELS16
#define AUDIO_NUM_CHANNELS         CHANNELS2
// BIT LENGTH_16BITS, BIT_LENGTH_18BITS
// BIT_LENGTH_20BITS, BIT_LENGTH_24BITS, BIT_LENGTH_32BITS
#define AUDIO_BIT_DEPTH    BIT_LENGTH_16BITS
#define AUDIO_ADC_IN         ADC_INPUT_LINE1
// DAC Output LINE1 (speaker)or LINE2 (headphone)
#define AUDIO_DAC_OUT       DAC_OUTPUT_LINE2

// Output: regValue = vol/3  -> 34 discrete levels (reg 0..33), 1.5 dB per step
// Vol step of 3 = 1 register step = 1.5 dB
#define AUDIO_OUTPUT_VOLUME               80
#define AUDIO_OUTPUT_VOLUME_STEP           6
// 0 to 100, internally it will map to 9 discrete levels
// Vol:   0,   13,  25,   38,   50,   63,   75,   88, 100
// dB:    0,    3,   6,    9,   12,   15,   18,   21,  24
// Power: 1,    2,   4,    8,   16,   32,   64,  128, 256  (power-of-2 per 3 dB step)
// Stepping is index-based (1 step = 3 dB) via stepInputVolume()
#define AUDIO_INPUT_VOLUME                75

// I2S_BITS_PER_SAMPLE_16BIT, I2S_BITS_PER_SAMPLE_24BIT
// I2S_BITS_PER_SAMPLE_32BIT;
#define AUDIO_I2S_BITS_PER_SAMPLE           I2S_BITS_PER_SAMPLE_16BIT

// ===================== DSP SETTINGS =====================
// Fixed DSP sample rate for coefficient calculations.
#define AUDIO_SAMPLE_RATE_HZ           16000.0f
#define HP_CUTOFF_MIN_HZ                   5.0f
#define HP_CUTOFF_MAX_HZ                  50.0f
#define HP_CUTOFF_DEFAULT_HZ              20.0f
#define LP_CUTOFF_MIN_HZ                1000.0f
#define LP_CUTOFF_MAX_HZ                4000.0f
#define LP_CUTOFF_DEFAULT_HZ            2000.0f
#define LP_STEP_HZ                       250.0f
#define HP_STEP_HZ                         1.0f
#define SW_GAIN_MIN                        0.5f
#define SW_GAIN_MAX                        4.0f
#define SW_GAIN_DEFAULT                    1.0f
#define SW_GAIN_STEP                       0.5f
#define NOISE_LEFT_SCALE_MIN               0.5f
#define NOISE_LEFT_SCALE_MAX               1.5f
#define NOISE_LEFT_SCALE_DEFAULT           1.0f
#define NOISE_LEFT_SCALE_STEP              0.05f
#define PLOT_DECIMATION_DEFAULT            4

int16_t audioBuffer[AUDIO_BLOCK_FRAMES * 2];
bool audioInitialized    =             false;
bool codecPinsConfigured =             false;

// Volumes
int outputVolume   =     AUDIO_OUTPUT_VOLUME;
int inputVolume    =      AUDIO_INPUT_VOLUME;

bool plotEnabled   =                   false;
int plotDecimation = PLOT_DECIMATION_DEFAULT;

bool  highPassEnabled    =             false;
bool  lowPassEnabled     =             false;
float highPassCutoffHz   =   HP_CUTOFF_DEFAULT_HZ;
float lowPassCutoffHz    =   LP_CUTOFF_DEFAULT_HZ;
float softwareGain       =   SW_GAIN_DEFAULT;
bool  noiseCancelEnabled =             false;
float noiseLeftScale     =   NOISE_LEFT_SCALE_DEFAULT;

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

Preferences appPrefs;

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

// ===================== HELPERS =====================
float clampf(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

void saveAppSettings() {
  appPrefs.begin(   "mb-stetho", false);
  appPrefs.putInt(  "outVol",    outputVolume);
  appPrefs.putInt(  "inVol",     inputVolume);
  appPrefs.putFloat("swGain",    softwareGain);
  appPrefs.putBool( "hpEn",      highPassEnabled);
  appPrefs.putBool ("lpEn",      lowPassEnabled);
  appPrefs.putFloat("hpFc",      highPassCutoffHz);
  appPrefs.putFloat("lpFc",      lowPassCutoffHz);
  appPrefs.putBool( "ncEn",      noiseCancelEnabled);
  appPrefs.putFloat("ncScale",   noiseLeftScale);
  appPrefs.putInt(  "plotDecim", plotDecimation);
  appPrefs.end();
}

void loadAppSettings() {
  appPrefs.begin("mb-stetho", true);
  outputVolume       = appPrefs.getInt(  "outVol",  AUDIO_OUTPUT_VOLUME);
  inputVolume        = appPrefs.getInt(  "inVol",   AUDIO_INPUT_VOLUME);
  softwareGain       = appPrefs.getFloat("swGain",  SW_GAIN_DEFAULT);
  highPassEnabled    = appPrefs.getBool( "hpEn",    false);
  lowPassEnabled     = appPrefs.getBool( "lpEn",    false);
  highPassCutoffHz   = appPrefs.getFloat("hpFc",    HP_CUTOFF_DEFAULT_HZ);
  lowPassCutoffHz    = appPrefs.getFloat("lpFc",    LP_CUTOFF_DEFAULT_HZ);
  noiseCancelEnabled = appPrefs.getBool( "ncEn",    false);
  noiseLeftScale     = appPrefs.getFloat("ncScale", NOISE_LEFT_SCALE_DEFAULT);
  plotDecimation     = appPrefs.getInt(  "plotDecim", PLOT_DECIMATION_DEFAULT);
  appPrefs.end();

  outputVolume   = (int)clampf((float)outputVolume, 0.0f, 100.0f);
  inputVolume    = (int)clampf((float)inputVolume, 0.0f, 100.0f);
  softwareGain   = clampf(softwareGain, SW_GAIN_MIN, SW_GAIN_MAX);
  noiseLeftScale = clampf(noiseLeftScale, NOISE_LEFT_SCALE_MIN, NOISE_LEFT_SCALE_MAX);
  if (plotDecimation != 1 && plotDecimation != 2 &&
      plotDecimation != 4 && plotDecimation != 8) plotDecimation = PLOT_DECIMATION_DEFAULT;
}

// Audio filtering functions:
// high-pass and low-pass filters
// software gain
//
// High-pass 1st order IIR filter
// Low-pass 2 cascaded first order IIR filters (2 pole Low-pass)

void updateFilterCoefficients() {
  highPassCutoffHz = clampf(highPassCutoffHz, HP_CUTOFF_MIN_HZ, HP_CUTOFF_MAX_HZ);
  lowPassCutoffHz  = clampf(lowPassCutoffHz, LP_CUTOFF_MIN_HZ, LP_CUTOFF_MAX_HZ);

  float hpRC = 1.0f / (2.0f * PI * highPassCutoffHz);
  float hpDt = 1.0f / AUDIO_SAMPLE_RATE_HZ;
  hpAlpha = hpRC / (hpRC + hpDt);

  float lpRC = 1.0f / (2.0f * PI * lowPassCutoffHz);
  float lpDt = 1.0f / AUDIO_SAMPLE_RATE_HZ;
  lpAlpha = lpDt / (lpRC + lpDt);
}

float applyAudioFilters(float inputSample, ChannelFilterState &state) {
  float x = inputSample;

  if (highPassEnabled) {
    float y = hpAlpha * (state.hpYPrev + x - state.hpXPrev);
    state.hpXPrev = x;
    state.hpYPrev = y;
    x = y;
  }

  x *= softwareGain;

  if (lowPassEnabled) {
    state.lp1State += lpAlpha * (x - state.lp1State);
    state.lp2State += lpAlpha * (state.lp1State - state.lp2State);
    x = state.lp2State;
  }

  return x;
}

void printAudioFilterStatus() {
  Serial.println("Audio processing:");
  snprintf(line, sizeof(line), "  %-20s = %6.2f",                    "SW gain",            softwareGain);
  Serial.println(line);
  snprintf(line, sizeof(line), "  %-20s = %-3s @ %6.1f Hz",          "HP filter",          highPassEnabled ? "ON" : "OFF", highPassCutoffHz);
  Serial.println(line);
  snprintf(line, sizeof(line), "  %-20s = %-3s @ %6.1f Hz",          "LP filter",          lowPassEnabled ? "ON" : "OFF", lowPassCutoffHz);
  Serial.println(line);
  snprintf(line, sizeof(line), "  %-20s = %-3s, left scale = %5.2f", "Noise cancel (R-L)", noiseCancelEnabled ? "ON" : "OFF", noiseLeftScale);
  Serial.println(line);
  snprintf(line, sizeof(line), "  %-20s = %d",                       "Plot decimation",    plotDecimation);
  Serial.println(line);
  Serial.println("=========================================================");
}

void printAllSettings() {
  Serial.println("Current settings:");
  const char *modeName = (appMode == APP_MODE_PRESSURE)    ? "Pressure"    :
                         (appMode == APP_MODE_STETHOSCOPE) ? "Stethoscope" :
                         (appMode == APP_MODE_BOTH)        ? "Both"        : "Unavailable";
  snprintf(line, sizeof(line), "  %-20s = %s", "Mode",       modeName);
  Serial.println(line);
  snprintf(line, sizeof(line), "  %-20s = %s", "Audio plot", plotEnabled ? "ON" : "OFF");
  Serial.println(line);

  printCodecVolumeStatus();
  printAudioFilterStatus();
}

// Vol values that correspond exactly to each of the 9 discrete gain indices.
// Using these avoids asymmetric jumps when stepping up vs. down.
static const int inputVolLevels[9] = {0, 13, 25, 38, 50, 63, 75, 88, 100};

// Step inputVolume by +1 or -1 gain index (3 dB per step), snapping to the
// nearest canonical level so every press changes gain by exactly 3 dB.
void stepInputVolume(int direction) {
  int idx = (inputVolume * 8) / 100;
  idx = constrain(idx + direction, 0, 8);
  inputVolume = inputVolLevels[idx];
}

void getInputGainInfo(int volSetting, int &idx, int &db, int &factor) {
  static const int dbTable[9] = {0, 3, 6, 9, 12, 15, 18, 21, 24};
  static const int factorTable[9] = {1, 2, 4, 8, 16, 32, 64, 128, 256};

  int vol = (int)clampf((float)volSetting, 0.0f, 100.0f);
  idx = (vol * 8) / 100;
  if (idx < 0) idx = 0;
  if (idx > 8) idx = 8;

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

void printCodecVolumeStatus() {
  int inIdx, inDb, inFactor;
  getInputGainInfo(inputVolume, inIdx, inDb, inFactor);

  int outReg;
  float outDb, outFactor;
  getOutputGainInfo(outputVolume, outReg, outDb, outFactor);

  snprintf(line, sizeof(line), "%-22s = %3d -> reg %2d, %+6.1f dB, x%.3f",
           "Output volume setting", outputVolume, outReg, outDb, outFactor);
  Serial.println(line);
  snprintf(line, sizeof(line), "%-22s = %3d -> idx %2d, %+6d dB, x%d",
           "Input volume setting ", inputVolume,  inIdx,   inDb, inFactor);
  Serial.println(line);
  Serial.println("=========================================================");
}

bool initBatteryGauge() {
  if (!maxlipo.begin()) {
    Serial.println("MAX17048 init failed");
    return false;
  }
  Serial.println("MAX17048 battery gauge ready");
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

// Runtime display: mode header, pressure or audio info, battery %.
//
// Stethoscope mode layout (all size-1, 128x32):
//   y= 0  "Stethoscope"        + battery top-right
//   y= 8  "ADC L1 -> OUT L1"
//   y=16  "Btn -> Sleep"
//
// Pressure / Both mode layout:
//   y= 0  "Pressure" or "Both" + battery top-right  (size-1)
//   y=14  "XXX.X mmHg"                              (size-2, 16 px tall)
void updateRuntimeOLED() {
  if (appMode == APP_MODE_NONE) {
    oledMessage("System unavailable", "Check sensors/audio", "Btn -> Sleep", true);
  } else if (appMode == APP_MODE_STETHOSCOPE) {
#if AUDIO_ADC_IN == ADC_INPUT_LINE1
    oledMessage("Stethoscope", "ADC L1 -> OUT L1", "Btn -> Sleep", true);
#else
    oledMessage("Stethoscope", "ADC L2 -> OUT L1", "Btn -> Sleep", true);
#endif
  } else {
    // Pressure or Both: header in size-1, reading in size-2
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(appMode == APP_MODE_BOTH ? "Both" : "Pressure");
    drawBatteryPercent();
    display.setTextSize(2);
    display.setCursor(0, 14);
    display.print(pressure_mmHg, 1);
    display.print(" mmHg");
    display.display();
  }
}

void printHelp() {
  Serial.println("==========Stethoscope & BP Brick ===== " STR(APP_VERSION)" ==============");

  Serial.println("Runtime commands:");
  Serial.println(" Configure:");
  Serial.println("  p/s/b = Pressure/Stethoscope/Both");
  Serial.println("  ?     = Help");
  Serial.println("  v     = print settings");
  Serial.println("  j/J   = save/load settings");
  Serial.println("  d     = cycle plot decimation (1/2/4/8)");
  Serial.println("  t     = toggle plot on/off");
  Serial.println(" Audio:");
  Serial.println("  </> = output volume    down/up (" STR(AUDIO_OUTPUT_VOLUME_STEP) ")");
  Serial.println("  ,/. = input volume     down/up (3 dB/step)");
  Serial.println("  +/- = software gain    up/down (" STR(SW_GAIN_STEP) ")");
  Serial.println("  L/l = low-pass cutoff  up/down (" STR(LP_STEP_HZ) " Hz)");
  Serial.println("  H/h = high-pass cutoff up/down (" STR(HP_STEP_HZ) " Hz)");
  Serial.println("  C/c = left noise scale up/down (" STR(NOISE_LEFT_SCALE_STEP) ")");
  Serial.println("  G/g = high-pass    ON/OFF");
  Serial.println("  K/k = low-pass     ON/OFF");
  Serial.println("  N/n = noise cancel ON/OFF (R - L*scale)");
  Serial.println("=========================================================");
}

void waitForButtonRelease() {
  while (digitalRead(MY_BUTTON) == LOW) {
    delay(10);
  }
  delay(50);
}

void enterDeepSleep() {
  oledMessage("Sleeping...", "Release button", "Press to wake", false);
  delay(5000);

  display.clearDisplay();
  display.display();

  waitForButtonRelease();

  rtc_gpio_init(MY_BUTTON);
  rtc_gpio_set_direction(MY_BUTTON, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en(MY_BUTTON);
  rtc_gpio_pulldown_dis(MY_BUTTON);

  esp_sleep_enable_ext0_wakeup(MY_BUTTON, 0);
  delay(50);
  esp_deep_sleep_start();
}

void handleSleepButton() {
  static bool lastState = HIGH;
  bool currentState = digitalRead(MY_BUTTON);

  if (lastState == HIGH && currentState == LOW) {
    delay(30);
    if (digitalRead(MY_BUTTON) == LOW) {
      enterDeepSleep();
    }
  }

  lastState = currentState;
}

void printWakeReason() {
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  const char *reason = (cause == ESP_SLEEP_WAKEUP_EXT0) ? "EXT0 (button)" : "power-on/reset/other";
  snprintf(line, sizeof(line), "Wake reason: %s", reason);
  Serial.println(line);
}

bool initOLED() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed");
    return false;
  }

  display.setRotation(2);
  display.clearDisplay();
  display.display();
  return true;
}

bool initPressure() {
  if (!mpr.begin()) {
    Serial.println("MPRLS init failed");
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
      MY_I2S_DIN,                       // Data In
      MY_I2S_DOUT                       // Data Out
    );
    codecPinsConfigured = true;
  }

  audio_driver::CodecConfig cfg;
  cfg.output_device = AUDIO_DAC_OUT;
  cfg.input_device  = AUDIO_ADC_IN;
  cfg.i2s.bits      = AUDIO_BIT_DEPTH;
  cfg.i2s.rate      = AUDIO_SAMPLE_RATE;
  cfg.i2s.channels  = AUDIO_NUM_CHANNELS;
  cfg.i2s.fmt       = I2S_NORMAL;
  cfg.i2s.mode      = MODE_SLAVE;

  if (!audioBoard.begin(cfg)) {
    Serial.println("ES8388 init failed");
    return false;
  }

  audioBoard.setVolume(outputVolume);
  audioBoard.setInputVolume(inputVolume);

  Serial.println("ES8388 configured");
  return true;
}

bool initI2SDriver() {
  i2s_config_t i2s_config = {};

  i2s_config.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
  i2s_config.sample_rate          = AUDIO_SAMPLE_RATE;
  i2s_config.bits_per_sample      = AUDIO_I2S_BITS_PER_SAMPLE;
  i2s_config.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
  i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2s_config.intr_alloc_flags     = 0;
  i2s_config.dma_buf_count        = AUDIO_DMA_BUF_COUNT;
  i2s_config.dma_buf_len          = AUDIO_DMA_BUF_LEN;
  i2s_config.use_apll             = true;
  i2s_config.tx_desc_auto_clear   = true;
  i2s_config.fixed_mclk           = AUDIO_SAMPLE_RATE * 256;

  i2s_pin_config_t pin_config = {};
  pin_config.mck_io_num           = MY_I2S_MCLK;
  pin_config.bck_io_num           = MY_I2S_BCLK;
  pin_config.ws_io_num            = MY_I2S_LRCLK;
  pin_config.data_out_num         = MY_I2S_DOUT;
  pin_config.data_in_num          = MY_I2S_DIN;

  esp_err_t err;

  err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("i2s_driver_install failed: %d\n", err);
    return false;
  }

  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("i2s_set_pin failed: %d\n", err);
    i2s_driver_uninstall(I2S_PORT);
    return false;
  }

  err = i2s_set_clk(I2S_PORT, AUDIO_SAMPLE_RATE, AUDIO_I2S_BITS_PER_SAMPLE, I2S_CHANNEL_STEREO);
  if (err != ESP_OK) {
    Serial.printf("i2s_set_clk failed: %d\n", err);
    i2s_driver_uninstall(I2S_PORT);
    return false;
  }

  return true;
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

  Serial.println("Pressure subsystem ready");
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

  audioInitialized = true;
  Serial.println("Audio subsystem ready");
  return true;
}

void resetAudioFilterState() {
  leftFilterState = {0.0f, 0.0f, 0.0f, 0.0f};
  rightFilterState = {0.0f, 0.0f, 0.0f, 0.0f};
}

void shutdownAudio() {
  if (!audioInitialized) return;

  audioBoard.setMute(true);
  esp_err_t err = i2s_driver_uninstall(I2S_PORT);
  if (err != ESP_OK) {
    Serial.printf("i2s_driver_uninstall failed: %d\n", err);
  }
  audioBoard.end();
  audioInitialized = false;
  resetAudioFilterState();
  Serial.println("Audio subsystem stopped");
}

void showModeBanner() {
  if (appMode == APP_MODE_NONE) {
    oledMessage("System unavailable", "Check sensors/audio", "Btn -> Sleep");
  } else if (appMode == APP_MODE_PRESSURE) {
    oledMessage("Pressure mode", "Ready", "Btn -> Sleep");
  } else if (appMode == APP_MODE_STETHOSCOPE) {
#if AUDIO_ADC_IN  == ADC_INPUT_LINE1
    oledMessage("Recording...", "ADC L1 -> OUT L1", "Btn -> Sleep");
#else
    oledMessage("Recording...", "ADC L2 -> OUT L1", "Btn -> Sleep");
#endif
  } else {
    oledMessage("Both mode", "Pressure + Audio", "Btn -> Sleep");
  }
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

  int resolvedMode = APP_MODE_NONE;
  if (newMode == APP_MODE_BOTH) {
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
  lastOledMs = 0;
  lastPressureMs = 0;
  lastBatteryMs = 0;
  updateOLED = true;

  if (!plotEnabled) {
    if (resolvedMode != newMode) {
      Serial.println("Requested mode unavailable; using available subsystem");
    }
    const char *modeName = (appMode == APP_MODE_PRESSURE)    ? "Pressure"    :
                           (appMode == APP_MODE_STETHOSCOPE) ? "Stethoscope" :
                           (appMode == APP_MODE_BOTH)        ? "Both"        : "Unavailable";
    snprintf(line, sizeof(line), "Mode active: %s", modeName);
    Serial.println(line);
  }

  showModeBanner();
}

void handleSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r' || c == ' ') {
      continue;
    }

    if (c == 'p' || c == 'P') {
      setAppMode(APP_MODE_PRESSURE);
    } else if (c == 's' || c == 'S') {
      setAppMode(APP_MODE_STETHOSCOPE);
    } else if (c == 'b' || c == 'B') {
      setAppMode(APP_MODE_BOTH);
    } else if (c == '?') {
      printHelp();
      plotEnabled = false;
    } else if (c == '>') {
      outputVolume += AUDIO_OUTPUT_VOLUME_STEP;
      if (outputVolume > 100) outputVolume = 100;
      if (audioInitialized) audioBoard.setVolume(outputVolume);
      if (!plotEnabled) {
        printCodecVolumeStatus();
      }
    } else if (c == '<') {
      outputVolume -= AUDIO_OUTPUT_VOLUME_STEP;
      if (outputVolume < 0) outputVolume = 0;
      if (audioInitialized) audioBoard.setVolume(outputVolume);
      if (!plotEnabled) {
        printCodecVolumeStatus();
      }
    } else if (c == '.') {
      stepInputVolume(+1);
      if (audioInitialized) audioBoard.setInputVolume(inputVolume);
      if (!plotEnabled) {
        printCodecVolumeStatus();
      }
    } else if (c == ',') {
      stepInputVolume(-1);
      if (audioInitialized) audioBoard.setInputVolume(inputVolume);
      if (!plotEnabled) {
        printCodecVolumeStatus();
      }
    } else if (c == '+') {
      softwareGain += SW_GAIN_STEP;
      softwareGain = clampf(softwareGain, SW_GAIN_MIN, SW_GAIN_MAX);
      if (!plotEnabled) {
        snprintf(line, sizeof(line), "  %-20s = %6.2f", "Software gain", softwareGain);
        Serial.println(line);
      }
    } else if (c == '-') {
      softwareGain -= SW_GAIN_STEP;
      softwareGain = clampf(softwareGain, SW_GAIN_MIN, SW_GAIN_MAX);
      if (!plotEnabled) {
        snprintf(line, sizeof(line), "  %-20s = %6.2f", "Software gain", softwareGain);
        Serial.println(line);
      }
    } else if (c == 'L') {
      lowPassCutoffHz += LP_STEP_HZ;
      updateFilterCoefficients();
      if (!plotEnabled) {
        snprintf(line, sizeof(line), "  %-20s = %6.1f Hz", "LP cutoff", lowPassCutoffHz);
        Serial.println(line);
      }
    } else if (c == 'l') {
      lowPassCutoffHz -= LP_STEP_HZ;
      updateFilterCoefficients();
      if (!plotEnabled) {
        snprintf(line, sizeof(line), "  %-20s = %6.1f Hz", "LP cutoff", lowPassCutoffHz);
        Serial.println(line);
      }
    } else if (c == 'H') {
      highPassCutoffHz += HP_STEP_HZ;
      updateFilterCoefficients();
      if (!plotEnabled) {
        snprintf(line, sizeof(line), "  %-20s = %6.1f Hz", "HP cutoff", highPassCutoffHz);
        Serial.println(line);
      }
    } else if (c == 'h') {
      highPassCutoffHz -= HP_STEP_HZ;
      updateFilterCoefficients();
      if (!plotEnabled) {
        snprintf(line, sizeof(line), "  %-20s = %6.1f Hz", "HP cutoff", highPassCutoffHz);
        Serial.println(line);
      }
    } else if (c == 'G') {
      highPassEnabled = true;
      if (!plotEnabled) {
        Serial.println("High-pass filter = ON");
      }
    } else if (c == 'g') {
      highPassEnabled = false;
      if (!plotEnabled) {
        Serial.println("High-pass filter = OFF");
      }
    } else if (c == 'K') {
      lowPassEnabled = true;
      if (!plotEnabled) {
        Serial.println("Low-pass filter = ON");
      }
    } else if (c == 'k') {
      lowPassEnabled = false;
      if (!plotEnabled) {
        Serial.println("Low-pass filter = OFF");
      }
    } else if (c == 'N') {
      noiseCancelEnabled = true;
      if (!plotEnabled) {
        Serial.println("Noise cancel (R-L) = ON");
      }
    } else if (c == 'n') {
      noiseCancelEnabled = false;
      if (!plotEnabled) {
        Serial.println("Noise cancel (R-L) = OFF");
      }
    } else if (c == 'C') {
      noiseLeftScale += NOISE_LEFT_SCALE_STEP;
      noiseLeftScale = clampf(noiseLeftScale, NOISE_LEFT_SCALE_MIN, NOISE_LEFT_SCALE_MAX);
      if (!plotEnabled) {
        snprintf(line, sizeof(line), "  %-20s = %5.2f", "Noise left scale", noiseLeftScale);
        Serial.println(line);
      }
    } else if (c == 'c') {
      noiseLeftScale -= NOISE_LEFT_SCALE_STEP;
      noiseLeftScale = clampf(noiseLeftScale, NOISE_LEFT_SCALE_MIN, NOISE_LEFT_SCALE_MAX);
      if (!plotEnabled) {
        snprintf(line, sizeof(line), "  %-20s = %5.2f", "Noise left scale", noiseLeftScale);
        Serial.println(line);
      }
    } else if (c == 'd') {
      if (plotDecimation == 1) plotDecimation = 2;
      else if (plotDecimation == 2) plotDecimation = 4;
      else if (plotDecimation == 4) plotDecimation = 8;
      else plotDecimation = 1;
      if (!plotEnabled) {
        snprintf(line, sizeof(line), "  %-20s = %d", "Plot decimation", plotDecimation);
        Serial.println(line);
      }
    } else if (c == 'j') {
      saveAppSettings();
      if (!plotEnabled) {
        Serial.println("Settings saved");
      }
    } else if (c == 'J') {
      loadAppSettings();
      updateFilterCoefficients();
      if (audioInitialized) {
        audioBoard.setVolume(outputVolume);
        audioBoard.setInputVolume(inputVolume);
      }
      if (!plotEnabled) {
        Serial.println("Settings loaded");
        printAllSettings();
      }
    } else if (c == 'v' || c == 'V') {
      printAllSettings();
      plotEnabled = false;
    } else if (c == 't' || c == 'T') {
      plotEnabled = !plotEnabled;
      if (!plotEnabled) {
        Serial.println("Plot = OFF");
      }
    } else {
      snprintf(line, sizeof(line), "Unknown command: %c", c);
      Serial.println(line);
      printHelp();
      plotEnabled = false;
    }
  }
}

void enterAutoShutoffDeepSleep() {
  autoShutoffTriggered = true;

  oledMessage("AUTO SHUTOFF", "", "", false);
  delay(1200);   // brief message for the presentation

  display.clearDisplay();
  display.display();

  waitForButtonRelease();

  rtc_gpio_init(MY_BUTTON);
  rtc_gpio_set_direction(MY_BUTTON, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en(MY_BUTTON);
  rtc_gpio_pulldown_dis(MY_BUTTON);

  esp_sleep_enable_ext0_wakeup(MY_BUTTON, 0);
  delay(50);
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(BAUDRATE);
  delay(500);
  printWakeReason();

  loadAppSettings();
  updateFilterCoefficients();

  pinMode(MY_BUTTON, INPUT_PULLUP);
  Wire.begin(MY_I2C_SDA, MY_I2C_SCL);

  if (!initOLED()) {
    while (true) delay(100);
  }

  batteryGaugeInitialized = initBatteryGauge();

  printHelp();
  Serial.println("Booting in configured default mode");
  setAppMode(appMode);
  awakeStartMs = millis();
  autoShutoffTriggered = false;
}

void loop() {
  handleSleepButton();
  handleSerialCommands();

  unsigned long currentTime = millis();

  // ------------------------------------

  if (!autoShutoffTriggered && currentTime - awakeStartMs >= 16200000UL) {
     enterAutoShutoffDeepSleep();
  }

  // ------------------------------------
  // Audio:
  // Read stereo PCM,
  // process both channels,
  // play processed right on both output channels.
  if ((appMode == APP_MODE_STETHOSCOPE || appMode == APP_MODE_BOTH) && audioInitialized) {
    size_t bytesRead = 0;

    esp_err_t r = i2s_read(I2S_PORT, audioBuffer, sizeof(audioBuffer), &bytesRead, portMAX_DELAY);

    if (r == ESP_OK && bytesRead > 0) {
      int sampleCount = bytesRead / sizeof(int16_t);
      bool fastBypass = !noiseCancelEnabled && !highPassEnabled && !lowPassEnabled &&
                        fabsf(softwareGain - 1.0f) < 0.0001f;

      // Process -----
      for (int i = 0; i < sampleCount; i += 2) {
        float leftIn = (float)audioBuffer[i];
        float rightIn = (float)audioBuffer[i + 1];

        float leftY;
        float rightY;

        // Filter and apply gain ---
        // Subtract left channel (background) from right channel (stethoscope)
        if (fastBypass) {
          leftY  = leftIn;
          rightY = rightIn;
        } else {
          leftY  = applyAudioFilters(leftIn, leftFilterState);
          rightY = applyAudioFilters(rightIn, rightFilterState);

          if (noiseCancelEnabled) {
            // Unilateral cancellation: right minus scaled processed left.
            rightY -= noiseLeftScale * leftY;
          }
        }

        // Print the audio data if enabled ---
        static int plotCounter = 0;
        if (plotEnabled) {
          plotCounter++;
          if (plotCounter >= plotDecimation) {
            plotCounter = 0;
            int32_t leftPlot = (int32_t)lrintf(leftY);
            int32_t rightPlot = (int32_t)lrintf(rightY);
            if (appMode == APP_MODE_BOTH && newPressure) {
              snprintf(line, sizeof(line), "%d,%d,%.1f", (int)leftPlot, (int)rightPlot, pressure_mmHg);
              Serial.println(line);
              newPressure = false;
            } else {
              snprintf(line, sizeof(line), "%d,%d", (int)leftPlot, (int)rightPlot);
              Serial.println(line);
            }
          }
        }

        // prepare output
        int32_t rightOut32 = (int32_t)lrintf(rightY);
        int32_t leftOut32  = (int32_t)lrintf(leftY);

        if (rightOut32 >  32767) rightOut32 =  32767;
        if (rightOut32 < -32768) rightOut32 = -32768;
        if (leftOut32  >  32767) leftOut32  =  32767;
        if (leftOut32  < -32768) leftOut32  = -32768;

        // Keep DAC playback
        audioBuffer[i] = (int16_t)leftOut32;
        audioBuffer[i + 1] = (int16_t)rightOut32;
      }

      // Write the processed audio data back to the I2S interface.
      size_t txOffset = 0;
      while (txOffset < bytesRead) {
        size_t chunkWritten = 0;
        esp_err_t w = i2s_write(
          I2S_PORT,
          ((const uint8_t *)audioBuffer) + txOffset,
          bytesRead - txOffset,
          &chunkWritten,
          portMAX_DELAY
        );

        if (w != ESP_OK || chunkWritten == 0) {
          Serial.printf("i2s_write incomplete: err=%d sent=%u/%u\n",
                        (int)w,
                        (unsigned int)txOffset,
                        (unsigned int)bytesRead);
          break;
        }

        txOffset += chunkWritten;
      } // end write audio data
    } // end process loop
  } // end audio

  // ------------------------------------
  // Print pressure data if plotting is enabled
  if (plotEnabled && pressureInitialized && appMode == APP_MODE_PRESSURE && newPressure) {
    newPressure = false;
    snprintf(line, sizeof(line), "%.1f", pressure_mmHg);
    Serial.println(line);
  } // end plot pressure

  // ------------------------------------
  // Read pressure data
  if (
      (appMode == APP_MODE_PRESSURE || appMode == APP_MODE_BOTH) &&
      pressureInitialized &&
      currentTime - lastPressureMs >= MPRLS_INTERVAL_MS
     )
  {

    lastPressureMs = currentTime;

    pressure_mmHg = (mpr.readPressure()-pressureZero)/0.9762f;
    if (isnan(pressure_mmHg)) {
      oledMessage("Pressure error", "Read failed");
      return;
    }

    if (pressure_mmHg < 0.0f) pressure_mmHg = 0.0f;

    newPressure = true;
    updateOLED = true;
  } // end pressure reading

  // ------------------------------------
  // Read battery level
  if (batteryGaugeInitialized && currentTime - lastBatteryMs >= BATTERY_INTERVAL_MS) {
    lastBatteryMs = currentTime;
    float newPercent = maxlipo.cellPercent();
    if (!isnan(newPercent)) {
      newPercent = clampf(newPercent, 0.0f, 100.0f);
      if (newPercent != batteryPercent) {
        batteryPercent = newPercent;
        updateOLED = true;
      }
    }
  } // end battery read

  // ------------------------------------
  // Update the OLED display
  if (currentTime - lastOledMs >= OLED_INTERVAL_MS) {
    lastOledMs = currentTime;
    if (updateOLED) {
      updateOLED = false;
      updateRuntimeOLED();
    }
  } // end OLED update

} // end loop()
