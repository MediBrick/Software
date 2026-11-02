/******************************************************************************************************/
// MPRLS Test Program
// 
// Based on Sparkfun MicroPressure Sensor Example by Alex Wende
//
// Urs Utzinger 2024
/******************************************************************************************************/

#include <Wire.h>
#include <SparkFun_MicroPressure.h>
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
 * Initialize Constructor
 * Optional parameters:
 *  - EOC_PIN: End Of Conversion (defualt: -1)
 *  - RST_PIN: Reset (default: -1)
 *  - MIN_PSI: Minimum Pressure (default: 0 PSI)
 *  - MAX_PSI: Maximum Pressure (default: 25 PSI)
 * 
 *  - MAX COUNTS = 0xE66666
 *  - MIN COUNTR = 0x19999A
 *
 * pressure = [ (reading - MIN_COUNT) * (P_MAX-P_MIN) ] / (MAX_COUNTS - MIN_COUNTS) + P_MIN
 *
 * Units available:
 * PSI  returns pressure
 * PA   returns pressure*6894.7573
 * KPA  returns pressure*6.89476
 * TORR returns pressure*51.7149
 * INHG returns pressure*2.03602
 * ATM  returns pressure*0.06805
 * BAR  returns pressure*0.06895
 */

#define EOC_PIN -1
#define RST_PIN -1
#define DEFAULT_ADDRESS 0x18

// Values for MPRLS0300YG with transfer function B
#define P_MIN 0   // PSI
#define P_MAX 300 // 5.80104 PSI or 300mmHg  
#define MAX_COUNTS 3774874 // 22.5% of 2^24 = 3774874
#define MIN_COUNTS  414000 //  2.5% of 2^24 =  419430, measured 413800

SparkFun_MicroPressure mpr(EOC_PIN, RST_PIN, P_MIN, P_MAX);

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

const char waitmsg[] = {"Waiting 5 seconds, skip by sending enter"};  // Allows user to open serial terminal to observe the debug output before the loop starts

void setup() {

  Serial.begin(BAUD_RATE);
  Serial.setTimeout(1000);      // default is 1000 
  serialTrigger(waitmsg, 5000); // give user time to open serial terminal

  myWire.begin(SDA, SCL);                          // SDA, SCL
  myWire.setClock(I2C_REGULAR);                           // 100kHz or 400kHz speed, we need to use slowest of all sensors
  myWire.setTimeOut(I2C_DEFAULTSTRETCH);        // We need to use largest of all sensors
  LOGln("I2C setup complete.");

  if(!mpr.begin(DEFAULT_ADDRESS, myWire))
  {
    LOGln("Cannot connect to MicroPressure sensor.");
    while(1);
  }
}

float avg = MIN_COUNTS;

void loop() {

  float reading = mpr.readPressure(RAW);
  if (reading == NAN) {

    uint8_t status      = mpr.readStatus();
    bool isPowered      = (status & 0b01000000) >> 6;
    bool isBusy         = (status & 0b00100000) >> 5;
    bool memoryError    = (status & 0b00000010) >> 2;
    bool mathSaturation = (status & 0b00000001);

    if (isPowered      == false) {LOGE("Device is not powerd")}   
    if (isBusy         == true)  {LOGE("Device is busy")}
    if (memoryError    == true)  {LOGE("Device memory error")}
    if (mathSaturation == true)  {LOGE("Device math saturation")}

  } else {
    avg = 0.99 * avg + 0.01 * reading;
    float pressure = ( float(reading - MIN_COUNTS) * float(P_MAX - P_MIN) / float(MAX_COUNTS - MIN_COUNTS) ) + P_MIN;
    LOGln("%.0f, %.2f", avg, pressure);
  }

  delay(10); // max 160 samples per second

}