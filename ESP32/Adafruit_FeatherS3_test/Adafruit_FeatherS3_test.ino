/*
  Adafruit Feather ESP32-S3 hardware self-test

  Arduino IDE / arduino-cli board settings:
  - Board: "Adafruit Feather ESP32-S3 2MB PSRAM"
  - USB Mode: "USB-OTG (TinyUSB)"
  - Upload Mode: "UART0 / Hardware CDC"
  - USB CDC On Boot: "Enabled"
  - MSC On Boot: "Disabled"
  - DFU On Boot: "Disabled"
  - PSRAM: "QSPI PSRAM"
  - Flash Size: "4MB (32Mb)"
  - Flash Mode: "QIO 80MHz"
  - Partition Scheme: "Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)"
  - CPU Frequency: "240MHz (WiFi)"

  Notes:
  - This sketch assumes the standard ESP32 board package variant
    `esp32:esp32:adafruit_feather_esp32s3`.
  - LED tests are visual only. PSRAM, MAX17048, WiFi, and BLE are checked
    programmatically and reported over Serial at 115200 baud.
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include <NimBLEDevice.h>
#include <esp_heap_caps.h>

constexpr unsigned long SERIAL_BAUD = 115200;
constexpr size_t PSRAM_TEST_SIZE = 512 * 1024;
constexpr uint8_t LED_DELAY_MS = 250;

Adafruit_NeoPixel rgbLed(NEOPIXEL_NUM, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
SFE_MAX1704X fuelGauge(MAX1704X_MAX17048);

struct TestStatus {
  const char* name;
  bool passed;
  String detail;
};

TestStatus testResults[] = {
  {"Board ID", false, ""},
  {"Status LED", false, ""},
  {"RGB LED", false, ""},
  {"PSRAM", false, ""},
  {"Battery Gauge", false, ""},
  {"WiFi", false, ""},
  {"Bluetooth LE", false, ""}
};

constexpr size_t TEST_COUNT = sizeof(testResults) / sizeof(testResults[0]);

void setResult(size_t index, bool passed, const String& detail) {
  testResults[index].passed = passed;
  testResults[index].detail = detail;
}

void blinkBuiltinLed(uint8_t count) {
  pinMode(LED_BUILTIN, OUTPUT);
  for (uint8_t i = 0; i < count; ++i) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(LED_DELAY_MS);
    digitalWrite(LED_BUILTIN, LOW);
    delay(LED_DELAY_MS);
  }
}

void showRgbColor(uint8_t red, uint8_t green, uint8_t blue, uint16_t holdMs) {
  rgbLed.setPixelColor(0, rgbLed.Color(red, green, blue));
  rgbLed.show();
  delay(holdMs);
}

void printDivider() {
  Serial.println(F("------------------------------------------------------------"));
}

bool testBoardIdentity() {
  String detail = String("Chip=") + ESP.getChipModel() +
                  ", revision=" + String(ESP.getChipRevision()) +
                  ", cores=" + String(ESP.getChipCores()) +
                  ", flash=" + String(ESP.getFlashChipSize() / (1024 * 1024)) + "MB";
  setResult(0, true, detail);
  return true;
}

bool testStatusLed() {
  blinkBuiltinLed(3);
  setResult(1, true, String("Blink sent on GPIO ") + LED_BUILTIN + ". Visual confirmation required.");
  return true;
}

bool testRgbLed() {
#if defined(NEOPIXEL_POWER)
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, NEOPIXEL_POWER_ON);
#endif

  rgbLed.begin();
  rgbLed.setBrightness(32);
  showRgbColor(255, 0, 0, 300);
  showRgbColor(0, 255, 0, 300);
  showRgbColor(0, 0, 255, 300);
  showRgbColor(255, 255, 255, 300);
  showRgbColor(0, 0, 0, 0);

  setResult(2, true, String("NeoPixel pattern sent on GPIO ") + PIN_NEOPIXEL + ". Visual confirmation required.");
  return true;
}

bool testPsram() {
  if (!psramFound()) {
    setResult(3, false, "PSRAM not detected by the ESP32-S3 runtime.");
    return false;
  }

  const size_t totalPsram = ESP.getPsramSize();
  const size_t freePsram = ESP.getFreePsram();
  const size_t allocSize = min(PSRAM_TEST_SIZE, freePsram / 2);

  if (totalPsram < 2 * 1024 * 1024) {
    setResult(3, false, String("PSRAM detected but size is only ") + totalPsram + " bytes.");
    return false;
  }

  if (allocSize < 64 * 1024) {
    setResult(3, false, String("PSRAM detected but only ") + freePsram + " bytes are free.");
    return false;
  }

  uint8_t* buffer = static_cast<uint8_t*>(heap_caps_malloc(allocSize, MALLOC_CAP_SPIRAM));
  if (buffer == nullptr) {
    setResult(3, false, "PSRAM detected but test allocation failed.");
    return false;
  }

  for (size_t i = 0; i < allocSize; ++i) {
    buffer[i] = static_cast<uint8_t>(i & 0xFF);
  }

  for (size_t i = 0; i < allocSize; ++i) {
    if (buffer[i] != static_cast<uint8_t>(i & 0xFF)) {
      free(buffer);
      setResult(3, false, String("PSRAM verify failed at byte ") + i + ".");
      return false;
    }
  }

  free(buffer);

  String detail = String("Detected ") + totalPsram +
                  " bytes, free=" + freePsram +
                  ", tested=" + allocSize + " bytes.";
  setResult(3, true, detail);
  return true;
}

bool testBatteryGauge() {
#if defined(PIN_I2C_POWER)
  pinMode(PIN_I2C_POWER, OUTPUT);
  digitalWrite(PIN_I2C_POWER, HIGH);
#endif

  Wire.begin();
  delay(10);

  if (!fuelGauge.begin(Wire)) {
    setResult(4, false, "MAX17048 fuel gauge was not detected on I2C.");
    return false;
  }

  const uint8_t deviceId = fuelGauge.getID();
  const uint8_t version = fuelGauge.getVersion();
  const float voltage = fuelGauge.getVoltage();
  const float soc = fuelGauge.getSOC();
  const float changeRate = fuelGauge.getChangeRate();

  String detail = String("MAX17048 id=0x") + String(deviceId, HEX) +
                  ", ver=0x" + String(version, HEX) +
                  ", voltage=" + String(voltage, 3) + "V" +
                  ", soc=" + String(soc, 1) + "%" +
                  ", rate=" + String(changeRate, 2) + "%/hr";
  setResult(4, true, detail);
  return true;
}

bool testWifi() {
  WiFi.mode(WIFI_STA);
  delay(100);

  const int16_t networkCount = WiFi.scanNetworks(false, true, false, 300);
  const String macAddress = WiFi.macAddress();
  WiFi.scanDelete();
  WiFi.mode(WIFI_OFF);

  if (networkCount == WIFI_SCAN_FAILED) {
    setResult(5, false, String("WiFi scan failed. STA MAC=") + macAddress);
    return false;
  }

  String detail = String("STA MAC=") + macAddress +
                  ", visible APs=" + networkCount;
  setResult(5, true, detail);
  return true;
}

bool testBle() {
  NimBLEDevice::init("FeatherS3-HW-Test");
  NimBLEServer* server = NimBLEDevice::createServer();
  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();

  if (server == nullptr || advertising == nullptr) {
    NimBLEDevice::deinit(true);
    setResult(6, false, "BLE stack failed to create server or advertiser.");
    return false;
  }

  advertising->setName("FeatherS3-HW-Test");
  const bool started = advertising->start();
  delay(250);
  const bool advertisingNow = advertising->isAdvertising();
  if (advertisingNow) {
    advertising->stop();
  }
  NimBLEDevice::deinit(true);

  if (!started || !advertisingNow) {
    setResult(6, false, "BLE initialized but advertising did not start.");
    return false;
  }

  setResult(6, true, "BLE stack initialized and advertising started successfully.");
  return true;
}

void printResults() {
  printDivider();
  Serial.println(F("Hardware self-test results"));
  printDivider();

  bool allAutoTestsPassed = true;
  for (size_t i = 0; i < TEST_COUNT; ++i) {
    Serial.printf("[%s] %s: %s\n",
                  testResults[i].passed ? "PASS" : "FAIL",
                  testResults[i].name,
                  testResults[i].detail.c_str());

    if (!testResults[i].passed &&
        String(testResults[i].name) != "Status LED" &&
        String(testResults[i].name) != "RGB LED") {
      allAutoTestsPassed = false;
    }
  }

  printDivider();
  Serial.println(F("LED checks are visual only."));
  if (allAutoTestsPassed) {
    Serial.println(F("Automatic hardware checks passed."));
  } else {
    Serial.println(F("One or more automatic hardware checks failed."));
  }
  Serial.println(F("Observe the LEDs during startup to confirm they are present and working."));
  printDivider();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1500);
  while (!Serial && millis() < 4000) {
    delay(10);
  }

  Serial.println();
  Serial.println(F("Adafruit Feather ESP32-S3 hardware self-test"));
  Serial.println(F("Expected hardware: 2MB PSRAM, status LED, RGB NeoPixel, MAX17048, WiFi, BLE"));

  testBoardIdentity();
  testStatusLed();
  testRgbLed();
  testPsram();
  testBatteryGauge();
  testWifi();
  testBle();
  printResults();
}

void loop() {
  static unsigned long lastHeartbeat = 0;
  static bool ledState = false;

  if (millis() - lastHeartbeat >= 1000) {
    lastHeartbeat = millis();
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
  }
}
