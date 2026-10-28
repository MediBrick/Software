/*
BLE Serial Test Program

This program demonstrates the BLESerial library with a comprehensive example.
It sets up a BLE UART service, processes commands from a connected client,
and generates data lines at a controlled rate. It also includes LED status
indication. 

Urs Utzinger
November/December 2025
*/
#include <Arduino.h>
#include <BLESerial.h>
#include <Linereader.h>
#include <StreamString.h>

constexpr unsigned long BAUDRATE        = 2'000'000UL;
constexpr uint8_t       LED_PIN         = LED_BUILTIN;
constexpr bool          useTaskPump     = true;       // set to 'false' to use polling mode

BLESerial               ble;
LineReader<128>         lr;

char                    line[128];                 // command line buffer
char                    data[256];                 // data output buffer
bool                    paused          = true;    // do not generated data until user requests it
unsigned long           lastBlinkUs     = 0;       // last LED blink time (microseconds)
unsigned long           blinkIntervalUs = 100'000; // LED blink interval (microseconds)
bool                    ledState        = LOW;     // current LED state
unsigned long           lastDataUs      = 0;       // last data rate calc time (microseconds)
unsigned long           dataCount       = 0;
unsigned long           dataCountPrev   = 0;
unsigned long           currentTime;
unsigned long           rate            = 0;

#ifdef ARDUINO_ARCH_ESP32
const char* resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "poweron";
    case ESP_RST_EXT:       return "external";
    case ESP_RST_SW:        return "software";
    case ESP_RST_PANIC:     return "panic";
    case ESP_RST_INT_WDT:   return "interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "task watchdog";
    case ESP_RST_WDT:       return "watchdog";
    case ESP_RST_DEEPSLEEP: return "deep sleep";
    case ESP_RST_BROWNOUT:  return "brownout";
    case ESP_RST_SDIO:      return "sdio";
    default:                return "unknown";
  }
}
#endif

void setup() {
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(BAUDRATE);
  while (!Serial && millis() < 5000) { delay(10); }

  Serial.println(F("=================================================================="));
  Serial.println(F("BLE Serial Test Program"));
  Serial.println(F("=================================================================="));

  // Initialize PSRAM (optional check)
  #ifdef ARDUINO_ARCH_ESP32
    esp_reset_reason_t resetReason = esp_reset_reason();
    Serial.printf("Reset reason: %s (%d)\r\n",
                  resetReasonName(resetReason), static_cast<int>(resetReason));
    if (psramInit()) {
      Serial.printf("PSRAM: total=%lu free=%lu\r\n", (unsigned long)ESP.getPsramSize(), (unsigned long)ESP.getFreePsram());
    } else {
      Serial.println("No PSRAM available.");
    }
  #endif

  ble.setPumpMode(useTaskPump ? BLESerial::PumpMode::Task : BLESerial::PumpMode::Polling);
  // ble.setPreferredMTU(517);           // set local preference before begin; the central negotiates it

  if (!ble.begin(
        BLESerial::Mode::Fast,        /*Fast, LowPower, LongRange, Balanced*/
        "BLESerialDevice",            /*Name*/
        BLESerial::Security::None     /*None, JustWorks, PasskeyDisplay*/
       )
     )  {
    Serial.println(F("BLESerial begin() failed"));
    while (true) delay(1000);
  }

  // Increase verbosity on serial logging
  // LOG_LEVEL_DEBUG shows all compiled debug events; define DEBUG before including BLESerial.h to compile them.
  // LOG_LEVEL_INFO shows key events, warnings and errors.
  // LOG_LEVEL_WARN shows problems and errors.
  // LOG_LEVEL_ERROR shows errors only.
  // LOG_LEVEL_NONE turns off logging.
  ble.setLogLevel(LOG_LEVEL_INFO);

  // Optionally configure parameters
  // ble.setPower(BLE_TX_DBP9, PWR_ALL); // set transmit power, check src/BLESerial.h for options

}

void loop() {
  currentTime = micros();

  // BLE Serial Update
  // =======================================================
  if (ble.getPumpMode() == BLESerial::PumpMode::Polling) {
    ble.update(); // polling pump
  }

  // Command Receiver (String)
  // =======================================================
  if (lr.poll(ble, line, sizeof(line))) { 

    // Lambda function to send reply to both Serial and BLE
    auto reply = [&](const String& msg){
      Serial.println(msg);
      ble.println(msg);
    };

    // Pause
    if (strcasecmp(line, "pause") == 0) {
      paused = true;
      reply("TX paused");

    // Resume
    } else if (strcasecmp(line, "resume") == 0) {
      paused = false;
      reply("TX resumed");

    // Status Report
    } else if (strcasecmp(line, ".") == 0) {
      StreamString sink;
      ble.printStats(sink);
      reply(sink.c_str());

    // Help
    } else if (strcasecmp(line, "?") == 0) {
      reply("help:\r\n"
            "  pause  - pause data transmission\r\n"
            "  resume - resume data transmission\r\n"
            "  .      - BLESerial status report\r\n"
            "  ?      - this help message");

    // Unknown command
    } else {
      String err = String("Unknown command: ") + line;
      reply(err);
    }
  } // end command receiver

  // Data Generator (char buffer)
  // =======================================================
  if (!paused && ble.writeReady()) {

    // Emit a fixed-length line of exactly 36 bytes (on 32-bit unsigned long):
    //   "count="(6) + %10lu (10) + " rate="(6) + %10lu (10) + "/s\r\n"(4) = 36
    // %10lu right-aligns with leading spaces
    uint16_t dataWritten = ble.printfNonBlocking("count=%10lu rate=%10lu/s\r\n", dataCount, rate);
    if (dataWritten != 36) {
      Serial.printf("WARNING partial write %u/36 bytes\r\n",dataWritten);
      ble.printStats(Serial);
    } else {
      dataCount++;
    }

    // Calculate data rate once per second
    if (currentTime - lastDataUs >= 1'000'000UL) {
      rate          = dataCount - dataCountPrev;
      lastDataUs    = currentTime;
      dataCountPrev = dataCount;
    }

  } // end data generator

  // Blink LED
  // =======================================================
  if ((currentTime - lastBlinkUs) >= blinkIntervalUs) {
    lastBlinkUs = currentTime;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    blinkIntervalUs = ledState ? 100'000 : 900'000;
  } // end blink

} // end loop
