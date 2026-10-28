#include <Arduino.h>
#include <SensirionI2cScd4x.h>
#include <Wire.h>
#include <logger.h>

SensirionI2cScd4x scd4x;

////////////////////////////////////////////////////
#define BAUDRATE      500000
#define I2CFrequency  100000
#define LOG_LEVEL     LOG_LEVEL_INFO
#define INTERVAL      1000
///////////////////////////////////////////////////

#ifdef  NO_ERROR
#undef  NO_ERROR
#endif
#define NO_ERROR          0

uint16_t error;
char errorMessage[64];

uint64_t serialNumber = 0;

// Measurement
uint16_t  co2         = 0;
float     temperature = 0.0f;
float     humidity    = 0.0f;
bool      isDataReady = false;

// Timings
unsigned long       currentTime;
unsigned long       lastSCD4XTime;
const unsigned long scd4XInterval = INTERVAL;

void PrintUint64(uint64_t& value) {
    Serial.print("0x");
    Serial.print((uint32_t)(value >> 32), HEX);
    Serial.print((uint32_t)(value & 0xFFFFFFFF), HEX);
}

void setup() {

    currentLogLevel = LOG_LEVEL;

    Serial.begin(BAUDRATE);
    while (!Serial) { delay(100); }
    LOGI("SCD41 Test Program");

    Wire.begin();
    Wire.setClock(I2CFrequency);

    scd4x.begin(Wire, SCD41_I2C_ADDR_62);
    delay(30);

    // Ensure sensor is in clean state
    error = scd4x.wakeUp();
    if (error != NO_ERROR) {
        errorToString(error, errorMessage, sizeof errorMessage);
        LOGE("Error trying to execute wakeUp(): %s", errorMessage);
    }
    error = scd4x.stopPeriodicMeasurement();
    if (error != NO_ERROR) {
        errorToString(error, errorMessage, sizeof errorMessage);
        LOGE("Error trying to execute stopPeriodicMeasurement(): %s", errorMessage);
    }

    error = scd4x.reinit();
    if (error != NO_ERROR) {
        errorToString(error, errorMessage, sizeof errorMessage);
        LOGE("Error trying to execute reinit(): %s", errorMessage);
    }

    // Read out information about the sensor
    error = scd4x.getSerialNumber(serialNumber);
    if (error != NO_ERROR) {
        errorToString(error, errorMessage, sizeof errorMessage);
        LOGE("Error trying to execute getSerialNumber(): %s", errorMessage);
        return;
    }

    Serial.print("serial number: ");
    PrintUint64(serialNumber);
    Serial.println();

    // Start Measurement
    error = scd4x.startPeriodicMeasurement();
    if (error) {
        errorToString(error, errorMessage, 256);
        LOGE("Error trying starting periodic measurement: %s", errorMessage);
    }

    lastSCD4XTime = 0;

}

//////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

    currentTime = millis();

    if ((currentTime - lastSCD4XTime) >= scd4XInterval) {
      lastSCD4XTime = currentTime;

      error = scd4x.getDataReadyStatus(isDataReady);
      if (error) {
          errorToString(error, errorMessage, 256);
          LOGE("Error checking data ready: %s", errorMessage);
          return;
      }
      if (!isDataReady) {
          return;
      }

      error = scd4x.readMeasurement(co2, temperature, humidity);
      if (error) {
          errorToString(error, errorMessage, 256);
          LOGE("Error reading data: %s", errorMessage);
      } else if (co2 == 0) {
          LOGE("Invalid sample detected, skipping.")
      } else {
          Serial.print("Co2:");         Serial.print(co2);         Serial.print(" ");
          Serial.print("Temperature:"); Serial.print(temperature); Serial.print(" ");
          Serial.print("Humidity:");    Serial.print(humidity);
          Serial.println();
      }
    }
}
