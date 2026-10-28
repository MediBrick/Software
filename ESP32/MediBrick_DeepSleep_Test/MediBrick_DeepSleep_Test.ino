#include <Arduino.h>

// ESP32 RTC and sleep control (guarded for editor linting on non-ESP32)
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

// Pins
const int BUTTON_PIN = 9;                          // Button (must be RTC-capable for EXT0 wake)
const gpio_num_t BUTTON_PIN_GPIO = GPIO_NUM_9;     // RTC IO mapping for EXT0
const int LED_PIN = 13;                            // Status LED

// OLED Display
//////////////////////////////////////////////////////////////////////////////////////////////
// Not attached here

// Define different states
typedef enum {
  Bat,
  Data
} stateDisplayType;

volatile stateDisplayType oledDisplay = Bat;

// LED
//////////////////////////////////////////////////////////////////////////////////////////////
constexpr unsigned long BLINK_DELAY_ON  = 200;  // ms LED ON duration
constexpr unsigned long BLINK_DELAY_OFF = 800;  // ms LED OFF duration
bool                    ledOn = false;

// Button
//////////////////////////////////////////////////////////////////////////////////////////////
const unsigned long DEBOUNCE_DELAY          = 50;    // ms debounce
const unsigned long GOSLEEP_BUTTON_DURATION = 2000;  // ms press duration to trigger deep sleep

volatile bool buttonChanged = false;  // Set by ISR on any button edge
int           buttonState   = LOW;    // Last debounced logical state

// Interrupt function for different states of OLED when button is pressed 
void IRAM_ATTR handleButtonPress() {
  buttonChanged = true;
}

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

// LOOP EXECUTION
//////////////////////////////////////////////////////////////////////////////////////////////

unsigned long lastButtonTime = 0;
unsigned long lastButtonHighTime = 0;
unsigned long lastBlinkTime = 0;

void setup() {
  #if defined(ARDUINO_ARCH_ESP32)
    // Release any previous deep-sleep holds so we can drive pins normally
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis((gpio_num_t)LED_PIN);
  #endif

  pinMode(BUTTON_PIN, INPUT_PULLDOWN);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Ensure the LED is off initially

  // Start Serial
  Serial.begin(57600);
  // delay(2000);
  delay(500);

  print_wakeup_reason();

  // Wake up configuration (EXT0 on BUTTON_PIN_GPIO, wake on HIGH level)
  #if defined(ARDUINO_ARCH_ESP32)
    rtc_gpio_init(BUTTON_PIN_GPIO);
    rtc_gpio_set_direction(BUTTON_PIN_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_dis(BUTTON_PIN_GPIO);
    rtc_gpio_pulldown_en(BUTTON_PIN_GPIO);
    esp_sleep_enable_ext0_wakeup(BUTTON_PIN_GPIO, 1);
  #endif

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, CHANGE);

  buttonState = digitalRead(BUTTON_PIN);

}

void loop() {

  unsigned long currentTime = millis();

  if (buttonChanged) {
    buttonChanged = false; // Reset flag set in ISR

    int reading = digitalRead(BUTTON_PIN);

    if ((currentTime - lastButtonTime) > DEBOUNCE_DELAY) {
      lastButtonTime = currentTime;

      if (reading == HIGH) {
        // Button pressed
        lastButtonHighTime = currentTime;
        digitalWrite(LED_PIN, HIGH); // Turn on the LED when the button is pressed
        ledOn = true;
        buttonState = HIGH;
      } else {
        // Button released
        buttonState = LOW;
        if ((currentTime - lastButtonHighTime) >= GOSLEEP_BUTTON_DURATION) {
          Serial.println("Long press detected; going to deep sleep...");
          Serial.flush();
          detachInterrupt(digitalPinToInterrupt(BUTTON_PIN));
          #if defined(ARDUINO_ARCH_ESP32)
          // Force LED low and hold it during deep sleep (if RTC-capable)
            gpio_hold_dis((gpio_num_t)LED_PIN);
            digitalWrite(LED_PIN, LOW);
            if (rtc_gpio_is_valid_gpio((gpio_num_t)LED_PIN)) {
              gpio_hold_en((gpio_num_t)LED_PIN);
              gpio_deep_sleep_hold_en();
            }
          #else
            digitalWrite(LED_PIN, LOW);
          #endif
          ledOn = false;
          delay(100);          
          esp_deep_sleep_start();
        } else {
          // Short press: switch display state
          Serial.println("Short press; toggling display state");
          oledDisplay = (oledDisplay == Bat) ? Data : Bat;
        }
        digitalWrite(LED_PIN, LOW);
        ledOn = false;
      }
    }
  }
  
  // Blink the LED when the button is not pressed
  if (buttonState == LOW) {
    unsigned long interval = ledOn ? BLINK_DELAY_ON : BLINK_DELAY_OFF;
    if ((currentTime - lastBlinkTime) >= interval) {
      ledOn = !ledOn;
      digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
      lastBlinkTime = currentTime;
    }
  }

}