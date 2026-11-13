/******************************************************************************************************/
// MPRLS Pressure Test: diagnostics, zeroing, pressure in mmHg
//
// Based on Sparkfun MicroPressure Sensor Example by Alex Wende
// Reads the raw 24 bit counts directly over I2C, the stock SparkFun library can not return raw counts.
//
// What it does after power up (cuff open to air, sensor not moving):
//  1. Diagnostics: bus, status byte, EOC pin (if wired), conversion time, read success rate,
//     offset of the counts from the expected zero and noise. Each check prints PASS, WARN or FAIL with a hint.
//  2. Zero: averages ZERO_SAMPLES readings, checks they are stable and near the expected zero, retries if not.
//  3. Reports pressure in mmHg as "pressure_mmHg:<value>,counts:<value>" (Arduino Serial Plotter compatible).
//
// mmHg = (counts - zero) / (counts per mmHg * P_CAL), the same calibration the Stethoscope firmware uses.
//
// Serial commands:  z = zero again (cuff open to air)   d = run diagnostics again   h = help
//
// Runtime error handling:
//  - every reading is classified (error bits below), bad readings are never used
//  - an error is reported once when it appears, repeated every REPORT_MS while it lasts,
//    and a message is printed when it clears (with the number of bad readings)
//  - after RECOVER_AFTER bad readings in a row the I2C bus is restarted
//  - at start-up the sketch keeps looking for the sensor and lists the I2C devices it does find
//
// How to read the errors:
//  - "bus reads all ones" (status 0xFF): sensor is not driving the bus. Check solder joints, VIN, GND,
//    SDA/SCL pins and pull-ups. This is a wiring problem, not a sensor measurement problem.
//  - "math saturation" with a valid status (0x41): sensor answers, but its internal math overflowed
//    (pressure beyond range). With the port open to air this suggests a damaged sensor.
//  - "I2C write failed": NACK on address means nothing answers at that address.
//
// Urs Utzinger 2024
/******************************************************************************************************/

#include <Wire.h>
#include <math.h>
#include <string.h>
#include "logger.h"

// Serial
#define BAUD_RATE          500000     // Up to 2,000,000 on ESP32, however more than 500 kBaud might be unreliable

// I2C speed
#define I2C_FAST           400000     // Fast   400,000
#define I2C_REGULAR        100000     // Normal 100,000
#define I2C_SLOW            50000     // Slow    50,000

// I2C clock stretch limit
#define I2C_DEFAULTSTRETCH     50     // 50ms , typical is 230 micro seconds
#define I2C_LONGSTRETCH       200     // 200ms

// Define I2C pins for ESP32-S3 Feather if not predefined
#ifndef SDA
  #define SDA 3
#endif

#ifndef SCL
  #define SCL 4
#endif

TwoWire myWire = TwoWire(0); // Create a TwoWire instance

/*
 * MPRLS pressure sensor, raw counts:
 *  - MAX COUNTS = 0xE66666 (transfer function A), 3774874 for transfer function B
 *  - MIN COUNTS = 0x19999A (transfer function A),  419430 for transfer function B
 *
 * pressure = [ (reading - MIN_COUNT) * (P_MAX-P_MIN) ] / (MAX_COUNTS - MIN_COUNTS) + P_MIN
 *
 * Status byte: bit6 powered, bit5 busy, bit2 memory integrity error, bit0 math saturation
 */

#define DEFAULT_ADDRESS 0x18

// Values for MPRLS0300YG with transfer function B
#define P_MIN 0   // PSI
#define P_MAX 300 // 5.80104 PSI or 300mmHg
#define MAX_COUNTS 3774874 // 22.5% of 2^24 = 3774874
#define MIN_COUNTS  414000 //  2.5% of 2^24 =  419430, measured 413800

// End Of Conversion pin: -1 = not connected, not checked. The Stethoscope firmware uses Feather D10 (10).
constexpr int8_t EOC_PIN = D10;
constexpr int8_t RST_PIN = D11;

// Calibration, same as the Stethoscope firmware: mmHg = (counts - zero) / (counts per mmHg * P_CAL)
constexpr float P_CAL         = 0.9762f;
constexpr float MMHG_PER_COUNT = (float)(P_MAX - P_MIN) / ((float)(MAX_COUNTS - MIN_COUNTS) * P_CAL);

// Output
constexpr uint32_t READ_INTERVAL_MS   = 10UL;    // sensor readings, up to 160 per second possible
constexpr uint32_t OUTPUT_INTERVAL_MS = 100UL;   // lines printed
constexpr float    LP_FC_HZ           = 2.0f;    // low-pass cutoff on the displayed pressure, as the firmware default
const float        LP_ALPHA           = 1.0f - expf(-2.0f * (float)PI * LP_FC_HZ * (float)READ_INTERVAL_MS / 1000.0f);
constexpr uint32_t STATS_MS           = 10000UL; // statistics line while running

// Diagnostics and zeroing limits. Defaults are plausible values, adjust after looking at your sensors.
constexpr uint16_t DIAG_SAMPLES       = 200U;    // readings taken by the diagnostics
constexpr uint16_t ZERO_SAMPLES       = 100U;    // readings averaged to find the zero
constexpr uint8_t  ZERO_RETRIES       = 5U;
constexpr uint32_t ZERO_BAND_COUNTS   = (MAX_COUNTS - MIN_COUNTS) / 50U;  // open to air the counts must be this close to MIN_COUNTS (2% of span, about 6 mmHg)
constexpr float    ZERO_STD_MAX_MMHG  = 2.0f;    // zero rejected if readings scatter more than this (cuff moving or pressurized)
constexpr float    NOISE_WARN_MMHG    = 1.0f;    // diagnostics noise limits (standard deviation)
constexpr float    NOISE_FAIL_MMHG    = 5.0f;
constexpr uint32_t CONV_WARN_US       = 15000UL; // conversion slower than this is suspicious

// Status byte bits
constexpr uint8_t ST_POWERED    = 0x40U;
constexpr uint8_t ST_BUSY       = 0x20U;
constexpr uint8_t ST_INTEGRITY  = 0x04U;
constexpr uint8_t ST_SATURATION = 0x01U;

// Error bits, one per kind of failure. E_RANGE is only a warning, the reading is still used.
constexpr uint8_t E_CMD     = 0x01U;   // I2C write (start conversion) failed
constexpr uint8_t E_READ    = 0x02U;   // I2C read returned fewer bytes than expected
constexpr uint8_t E_TIMEOUT = 0x04U;   // sensor still busy after BUSY_TIMEOUT_MS
constexpr uint8_t E_BUS     = 0x08U;   // status byte 0xFF: bus reads all ones
constexpr uint8_t E_POWER   = 0x10U;   // status says sensor is not powered
constexpr uint8_t E_MEM     = 0x20U;   // status says memory integrity error
constexpr uint8_t E_SAT     = 0x40U;   // status says math saturation
constexpr uint8_t E_RANGE   = 0x80U;   // counts far outside the expected MIN..MAX (warning)
constexpr uint8_t E_WARN    = E_RANGE; // errors that do not invalidate the reading

const char* const ERR_NAME[8] = {
  "I2C write failed", "I2C read short", "sensor busy timeout", "bus reads all ones (0xFF)",
  "not powered", "memory error", "math saturation", "counts out of range"
};
const char* const I2C_RC_NAME[6] = {   // return codes of Wire.endTransmission()
  "ok", "data too long", "NACK on address", "NACK on data", "other error", "timeout"
};

constexpr uint32_t BUSY_TIMEOUT_MS = 20UL;    // max wait for a conversion
constexpr uint32_t REPORT_MS       = 5000UL;  // repeat an ongoing error this often
constexpr uint16_t RECOVER_AFTER   = 50U;     // bad readings in a row before the I2C bus is restarted
constexpr uint32_t RECOVER_MIN_MS  = 2000UL;  // minimum time between I2C restarts
constexpr uint32_t RANGE_MARGIN    = (MAX_COUNTS - MIN_COUNTS) / 20U;  // 5% of span allowed beyond MIN/MAX

// Statistics of a series of readings
struct MprStats {
  uint16_t good, bad;
  uint16_t err_hist[8];                  // readings with each error bit set
  uint32_t cmin, cmax;                   // counts
  float    mean, std;                    // counts
  uint32_t conv_min_us, conv_max_us;     // time from start of conversion until the sensor reports ready
  float    conv_mean_us;
  uint8_t  last_status;
};

// Boot helper
// Prints message and waits until timeout or user sends a character on the serial terminal
void serialTrigger(const char* mess, int timeout) {
  uint32_t startTime = millis();
  Serial.println(); Serial.println(mess);
  while ( !Serial.available() && ( (millis() - startTime) < timeout ) ) {
    delay(500);
  }
  // Clear the serial input buffer
  while (Serial.available()) { Serial.read(); }
}

const char waitmsg[] = {"Open the cuff to air and keep the sensor still. Starting in 5 seconds, skip by sending enter"};

// State
float    zero_counts     = MIN_COUNTS;   // counts at zero pressure
bool     zero_ok         = false;        // zero was measured and passed the stability checks
float    filtered        = 0.0f;         // low-passed counts
bool     filter_valid    = false;
uint8_t  prev_err        = 0U;           // errors of the previous reading
uint16_t consec_bad      = 0U;           // bad readings in a row
uint32_t bad_since_ok    = 0U;           // bad readings since the last good one (reported when error clears)
uint32_t last_report_ms  = 0U;
uint32_t last_recover_ms = 0U;
uint32_t last_out_ms     = 0U;
uint32_t last_stats_ms   = 0U;
uint32_t next_read_ms    = 0U;
uint32_t n_good          = 0U;
uint32_t n_bad           = 0U;
uint32_t n_good_stats    = 0U;           // good readings since the last statistics line
uint32_t n_recoveries    = 0U;
uint32_t eoc_low         = 0U;           // times the EOC pin was low although the sensor reported the data ready

// One conversion. Returns the error bits (0 = good). counts, status, rc (I2C write result) and
// conv_us (time until ready) are always set, status is 0xFF when the sensor did not answer.
uint8_t readMPR(uint32_t &counts, uint8_t &status, uint8_t &rc, uint32_t &conv_us) {

  uint8_t err = 0U;
  counts  = 0U;
  status  = 0xFFU;
  conv_us = 0U;

  // Start conversion: command 0xAA 0x00 0x00
  myWire.beginTransmission(DEFAULT_ADDRESS);
  myWire.write((uint8_t)0xAA);
  myWire.write((uint8_t)0x00);
  myWire.write((uint8_t)0x00);
  rc = myWire.endTransmission();
  if (rc != 0U) { return E_CMD; }

  // Wait until not busy (status 0xFF means no answer), at most BUSY_TIMEOUT_MS
  uint32_t t0 = micros();
  do {
    delayMicroseconds(200);
    status = (myWire.requestFrom((uint8_t)DEFAULT_ADDRESS, (uint8_t)1) == 1) ? myWire.read() : 0xFFU;
  } while ( (status & ST_BUSY) && (status != 0xFFU) && ((micros() - t0) < (BUSY_TIMEOUT_MS * 1000UL)) );
  conv_us = micros() - t0;

  // EOC must be high when the data is ready
  if ( (EOC_PIN >= 0) && (status != 0xFFU) && !(status & ST_BUSY) && !digitalRead(EOC_PIN) ) { eoc_low++; }

  // Read status and 24 bit result
  if (myWire.requestFrom((uint8_t)DEFAULT_ADDRESS, (uint8_t)4) != 4) { return E_READ; }
  status  = myWire.read();
  counts  = ((uint32_t)myWire.read() << 16) | ((uint32_t)myWire.read() << 8) | (uint32_t)myWire.read();

  if (status == 0xFFU) { return E_BUS; }                // all bits set carries no information, do not decode
  if (!(status & ST_POWERED))   { err |= E_POWER; }
  if (  status & ST_BUSY)       { err |= E_TIMEOUT; }
  if (  status & ST_INTEGRITY)  { err |= E_MEM; }
  if (  status & ST_SATURATION) { err |= E_SAT; }
  if ( (counts + RANGE_MARGIN < (uint32_t)MIN_COUNTS) || (counts > (uint32_t)MAX_COUNTS + RANGE_MARGIN) ) { err |= E_RANGE; }
  return err;
}

// Takes n readings, READ_INTERVAL_MS apart, and collects statistics of the good ones.
void collect(uint16_t n, MprStats &st) {

  memset(&st, 0, sizeof(st));
  st.cmin        = 0xFFFFFFFFUL;
  st.conv_min_us = 0xFFFFFFFFUL;
  uint32_t x0 = 0U;                                  // sums are taken relative to the first reading to keep float precise
  float    s1 = 0.0f, s2 = 0.0f, c1 = 0.0f;

  for (uint16_t i = 0U; i < n; i++) {
    uint32_t counts, conv_us;
    uint8_t  status, rc;
    uint8_t  err = readMPR(counts, status, rc, conv_us);
    st.last_status = status;
    for (uint8_t b = 0U; b < 8U; b++) { if (err & (1U << b)) { st.err_hist[b]++; } }

    if (err & ~E_WARN) {
      st.bad++;
    } else {
      if (st.good == 0U) { x0 = counts; }
      float d = (float)((int32_t)counts - (int32_t)x0);
      s1 += d; s2 += d * d; c1 += (float)conv_us;
      if (counts  < st.cmin)        { st.cmin        = counts; }
      if (counts  > st.cmax)        { st.cmax        = counts; }
      if (conv_us < st.conv_min_us) { st.conv_min_us = conv_us; }
      if (conv_us > st.conv_max_us) { st.conv_max_us = conv_us; }
      st.good++;
    }
    delay(READ_INTERVAL_MS);
  }

  if (st.good > 0U) {
    st.mean         = (float)x0 + s1 / (float)st.good;
    st.conv_mean_us = c1 / (float)st.good;
    if (st.good > 1U) {
      float var = (s2 - s1 * s1 / (float)st.good) / (float)(st.good - 1U);
      st.std = (var > 0.0f) ? sqrtf(var) : 0.0f;
    }
  }
}

// Checks the sensor and prints PASS / WARN / FAIL for each check. Sensor must be open to air and still.
void runDiagnostics() {

  uint8_t fails = 0U, warns = 0U;
  LOGln("=== MPRLS diagnostics (sensor must be open to air and not moving) ===");

  // 1. Bus
  myWire.beginTransmission(DEFAULT_ADDRESS);
  uint8_t rc = myWire.endTransmission();
  if (rc != 0U) {
    LOGln("[FAIL] I2C: no ACK at 0x%02X: %s (code %u). Check wiring, power and address.",
          (unsigned)DEFAULT_ADDRESS, I2C_RC_NAME[rc < 6U ? rc : 4U], (unsigned)rc);
    LOGln("=== Result: FAIL (sensor not reachable) ===");
    return;
  }
  LOGln("[PASS] I2C: sensor answers at 0x%02X", (unsigned)DEFAULT_ADDRESS);

  // 2. Status byte while idle
  uint8_t status = (myWire.requestFrom((uint8_t)DEFAULT_ADDRESS, (uint8_t)1) == 1) ? myWire.read() : 0xFFU;
  if      (status == 0xFFU)           { LOGln("[FAIL] Status: 0xFF, bus reads all ones: SDA stuck high or sensor not driving it. Check solder joints and pull-ups."); fails++; }
  else if (!(status & ST_POWERED))    { LOGln("[FAIL] Status: 0x%02X, sensor reports not powered. Check VIN and GND.", (unsigned)status); fails++; }
  else if (status & ST_INTEGRITY)     { LOGln("[FAIL] Status: 0x%02X, memory integrity error: sensor internal memory is corrupt (damaged or unstable supply).", (unsigned)status); fails++; }
  else if (status & ST_SATURATION)    { LOGln("[FAIL] Status: 0x%02X, math saturation while idle: pressure over range or sensor damaged.", (unsigned)status); fails++; }
  else if (status & ST_BUSY)          { LOGln("[WARN] Status: 0x%02X, still busy while idle.", (unsigned)status); warns++; }
  else                                { LOGln("[PASS] Status: 0x%02X (powered, ready, no errors)", (unsigned)status); }

  // 3. Series of readings
  MprStats st;
  collect(DIAG_SAMPLES, st);
  float good_pct = 100.0f * (float)st.good / (float)DIAG_SAMPLES;
  if      (st.good == DIAG_SAMPLES) { LOGln("[PASS] Readings: %u of %u good", (unsigned)st.good, (unsigned)DIAG_SAMPLES); }
  else if (good_pct >= 90.0f)       { LOGln("[WARN] Readings: only %u of %u good (%.0f%%)", (unsigned)st.good, (unsigned)DIAG_SAMPLES, good_pct); warns++; }
  else                              { LOGln("[FAIL] Readings: only %u of %u good (%.0f%%)", (unsigned)st.good, (unsigned)DIAG_SAMPLES, good_pct); fails++; }
  for (uint8_t b = 0U; b < 8U; b++) {
    if (st.err_hist[b]) { LOGln("       %u x %s", (unsigned)st.err_hist[b], ERR_NAME[b]); }
  }

  if (st.good > 0U) {
    // 4. Offset from expected zero
    float off = st.mean - (float)MIN_COUNTS;
    if (fabsf(off) <= (float)ZERO_BAND_COUNTS) { LOGln("[PASS] Zero offset: mean %.0f counts, %+.0f from expected %d (%+.2f mmHg)", st.mean, off, (int)MIN_COUNTS, off * MMHG_PER_COUNT); }
    else { LOGln("[WARN] Zero offset: mean %.0f counts is %+.0f from expected %d (%+.1f mmHg). Cuff not open to air, or sensor offset is large.", st.mean, off, (int)MIN_COUNTS, off * MMHG_PER_COUNT); warns++; }

    // 5. Stuck values
    if (st.cmax == st.cmin && st.good > 10U) { LOGln("[FAIL] Counts constant at %lu: sensor output is stuck.", (unsigned long)st.cmin); fails++; }
    else if (st.cmin == 0U || st.cmax >= 0xFFFFFFUL) { LOGln("[FAIL] Counts reach %lu..%lu: rail value, sensor or wiring faulty.", (unsigned long)st.cmin, (unsigned long)st.cmax); fails++; }

    // 6. Noise
    float noise = st.std * MMHG_PER_COUNT;
    if      (noise <= NOISE_WARN_MMHG) { LOGln("[PASS] Noise: std %.1f counts (%.2f mmHg), range %lu..%lu", st.std, noise, (unsigned long)st.cmin, (unsigned long)st.cmax); }
    else if (noise <= NOISE_FAIL_MMHG) { LOGln("[WARN] Noise: std %.1f counts (%.2f mmHg) is high. Sensor moving or pressurized? Supply noise?", st.std, noise); warns++; }
    else                               { LOGln("[FAIL] Noise: std %.1f counts (%.2f mmHg) is very high. Pressure unstable, or sensor/supply faulty.", st.std, noise); fails++; }

    // 7. Conversion time
    if (st.conv_max_us <= CONV_WARN_US) { LOGln("[PASS] Conversion time: %lu..%lu us, mean %.0f us", (unsigned long)st.conv_min_us, (unsigned long)st.conv_max_us, st.conv_mean_us); }
    else { LOGln("[WARN] Conversion time: up to %lu us, mean %.0f us. Slow bus or sensor?", (unsigned long)st.conv_max_us, st.conv_mean_us); warns++; }
  }

  // 8. EOC pin
  if (EOC_PIN >= 0) {
    if (eoc_low == 0U) { LOGln("[PASS] EOC pin %d went high when data was ready", (int)EOC_PIN); }
    else { LOGln("[FAIL] EOC pin %d was low %lu times although data was ready: check the EOC wire.", (int)EOC_PIN, (unsigned long)eoc_low); fails++; }
  } else {
    LOGln("[SKIP] EOC pin not configured (EOC_PIN = -1)");
  }

  if      (fails > 0U) { LOGln("=== Result: FAIL (%u failed, %u warnings) ===", (unsigned)fails, (unsigned)warns); }
  else if (warns > 0U) { LOGln("=== Result: PASS with %u warnings ===", (unsigned)warns); }
  else                 { LOGln("=== Result: PASS ==="); }
}

// Measures the zero: ZERO_SAMPLES readings must be mostly good, stable and close to MIN_COUNTS.
// Retries ZERO_RETRIES times. A zero that never passes is kept if there is no better one, but flagged.
void zeroSensor() {

  LOGln("=== Zeroing: cuff must be open to air and the sensor still ===");
  MprStats st;
  for (uint8_t attempt = 1U; attempt <= ZERO_RETRIES; attempt++) {
    collect(ZERO_SAMPLES, st);
    float std_mmhg = st.std * MMHG_PER_COUNT;
    float off      = st.mean - (float)MIN_COUNTS;

    if (st.good < (ZERO_SAMPLES * 4U) / 5U) {
      LOGW("Zero attempt %u/%u: only %u of %u readings good (status 0x%02X).", (unsigned)attempt, (unsigned)ZERO_RETRIES, (unsigned)st.good, (unsigned)ZERO_SAMPLES, (unsigned)st.last_status);
    } else if (std_mmhg > ZERO_STD_MAX_MMHG) {
      LOGW("Zero attempt %u/%u: readings scatter %.2f mmHg (limit %.1f). Cuff moving or pressurized?", (unsigned)attempt, (unsigned)ZERO_RETRIES, std_mmhg, ZERO_STD_MAX_MMHG);
    } else if (fabsf(off) > (float)ZERO_BAND_COUNTS) {
      LOGW("Zero attempt %u/%u: mean %.0f counts is %+.0f from expected %d (%+.1f mmHg). Cuff open to air?", (unsigned)attempt, (unsigned)ZERO_RETRIES, st.mean, off, (int)MIN_COUNTS, off * MMHG_PER_COUNT);
    } else {
      zero_counts  = st.mean;
      zero_ok      = true;
      filter_valid = false;
      LOGln("Zero set: %.1f counts (%+.2f mmHg from nominal), scatter %.2f mmHg, %u readings.", zero_counts, off * MMHG_PER_COUNT, std_mmhg, (unsigned)st.good);
      return;
    }
    delay(1000);
  }

  // Never passed
  if (!zero_ok && st.good > 0U) { zero_counts = st.mean; }   // best available, better than the nominal zero
  filter_valid = false;
  LOGE("Zero NOT verified after %u attempts, using %.1f counts%s. Fix the cause and send z.", (unsigned)ZERO_RETRIES, zero_counts, zero_ok ? " (previous zero kept)" : "");
}

void setup() {

  Serial.begin(BAUD_RATE);
  Serial.setTimeout(1000);      // default is 1000
  serialTrigger(waitmsg, 5000); // give user time to open serial terminal and the cuff

  myWire.begin(SDA, SCL);                          // SDA, SCL
  myWire.setClock(I2C_REGULAR);                    // 100kHz or 400kHz speed, we need to use slowest of all sensors
  myWire.setTimeOut(I2C_DEFAULTSTRETCH);           // We need to use largest of all sensors
  if (EOC_PIN >= 0) { pinMode(EOC_PIN, INPUT); }
  LOGln("I2C setup complete, SDA=%d SCL=%d.", (int)SDA, (int)SCL);

  // Look for the sensor, keep trying (wiring can be fixed while the sketch is running)
  bool listed = false;
  while (true) {
    myWire.beginTransmission(DEFAULT_ADDRESS);
    uint8_t rc = myWire.endTransmission();
    if (rc == 0) { break; }

    LOGE("No MicroPressure sensor at 0x%02X: %s (code %u). Retrying every second.",
         (unsigned)DEFAULT_ADDRESS, I2C_RC_NAME[rc < 6U ? rc : 4U], (unsigned)rc);

    if (!listed) {   // once: list what does answer on the bus
      uint8_t found = 0U;
      for (uint8_t a = 0x08U; a < 0x78U; a++) {
        myWire.beginTransmission(a);
        if (myWire.endTransmission() == 0) { LOGW("I2C device found at 0x%02X", (unsigned)a); found++; }
      }
      if (found == 0U) { LOGW("No I2C device answers at all: check SDA/SCL wiring, pull-ups and power."); }
      listed = true;
    }
    delay(1000);
  }

  runDiagnostics();
  zeroSensor();
  LOGln("Running. Commands: z = zero, d = diagnostics, h = help");

  next_read_ms   = millis();
  last_stats_ms  = millis();
  eoc_low        = 0U;
}

void loop() {

  // Commands
  while (Serial.available()) {
    char c = Serial.read();
    if      (c == 'z') { zeroSensor(); }
    else if (c == 'd') { runDiagnostics(); }
    else if (c == 'h') { LOGln("Commands: z = zero (cuff open to air), d = diagnostics, h = help. Output: pressure_mmHg, counts"); }
    else               { continue; }
    next_read_ms  = millis();
    last_stats_ms = millis();
    n_good_stats  = 0U;
    prev_err      = 0U;
    consec_bad    = 0U;
    bad_since_ok  = 0U;
  }

  // Schedule
  uint32_t now = millis();
  if ((int32_t)(now - next_read_ms) < 0) { delay(1); return; }
  next_read_ms += READ_INTERVAL_MS;
  if ((int32_t)(now - next_read_ms) > 0) { next_read_ms = now + READ_INTERVAL_MS; }   // fell behind, do not catch up

  uint32_t counts, conv_us;
  uint8_t  status, rc;
  uint8_t  err = readMPR(counts, status, rc, conv_us);
  bool     bad = (err & ~E_WARN) != 0U;

  // Reporting: on change, then repeated while the error lasts, with a message when it clears
  if ( (err != prev_err) || (err && ((now - last_report_ms) >= REPORT_MS)) ) {
    if (err) {
      char msg[192];
      size_t n = 0U;
      for (uint8_t b = 0U; b < 8U && n < sizeof(msg); b++) {
        if (err & (1U << b)) { n += snprintf(msg + n, sizeof(msg) - n, "%s%s", n ? ", " : "", ERR_NAME[b]); }
      }
      if (bad) { LOGE("MPRLS: %s | status 0x%02X, counts %lu, I2C write: %s, bad %lu, good %lu, I2C restarts %lu",
                      msg, (unsigned)status, (unsigned long)counts, I2C_RC_NAME[rc < 6U ? rc : 4U],
                      (unsigned long)(bad_since_ok + 1U), (unsigned long)n_good, (unsigned long)n_recoveries); }
      else     { LOGW("MPRLS: %s | counts %lu (expected %d..%d)", msg, (unsigned long)counts, (int)MIN_COUNTS, (int)MAX_COUNTS); }
    } else {
      LOGI("MPRLS: recovered after %lu bad readings.", (unsigned long)bad_since_ok);
    }
    prev_err       = err;
    last_report_ms = now;
  }

  if (bad) {
    n_bad++;
    consec_bad++;
    bad_since_ok++;
    // Restart the I2C bus if the sensor stays silent
    if ( (consec_bad >= RECOVER_AFTER) && ((now - last_recover_ms) >= RECOVER_MIN_MS) ) {
      myWire.end();
      delay(10);
      myWire.begin(SDA, SCL);
      myWire.setClock(I2C_REGULAR);
      myWire.setTimeOut(I2C_DEFAULTSTRETCH);
      n_recoveries++;
      consec_bad      = 0U;
      last_recover_ms = now;
      LOGW("MPRLS: I2C bus restarted (%lu so far).", (unsigned long)n_recoveries);
    }
  } else {
    consec_bad   = 0U;
    bad_since_ok = 0U;
    n_good++;
    n_good_stats++;

    // Low-pass on the counts, started with the first reading
    if (filter_valid) { filtered += LP_ALPHA * ((float)counts - filtered); }
    else              { filtered = (float)counts; filter_valid = true; }

    if ((now - last_out_ms) >= OUTPUT_INTERVAL_MS) {
      last_out_ms = now;
      LOGln("pressure_mmHg:%.2f,counts:%.0f", (filtered - zero_counts) * MMHG_PER_COUNT, filtered);
    }
  }

  // Statistics
  if ((now - last_stats_ms) >= STATS_MS) {
    LOGln("MPRLS: %.0f readings/s, good %lu, bad %lu, I2C restarts %lu, zero %.0f counts %s%s",
          1000.0f * (float)n_good_stats / (float)(now - last_stats_ms), (unsigned long)n_good, (unsigned long)n_bad,
          (unsigned long)n_recoveries, zero_counts, zero_ok ? "(ok)" : "(NOT VERIFIED, send z)",
          (EOC_PIN >= 0 && eoc_low) ? ", EOC low seen" : "");
    last_stats_ms = now;
    n_good_stats  = 0U;
  }

}
