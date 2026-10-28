#include <esp_system.h>
#include <esp_heap_caps.h>

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=== ESP32-S3 PSRAM Test ===");

    // Check if PSRAM is available
    if (psramFound()) {
        Serial.println("PSRAM detected!");
    } else {
        Serial.println("PSRAM not found or not enabled!");
        return;
    }

    // Display available PSRAM
    Serial.printf("Total PSRAM: %d bytes\n", ESP.getPsramSize());
    Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());

    // Allocate a test buffer in PSRAM (1MB)
    size_t test_size = 1024 * 1024; // 1MB
    uint8_t* psram_buffer = (uint8_t*)heap_caps_malloc(test_size, MALLOC_CAP_SPIRAM);

    if (psram_buffer == NULL) {
        Serial.println("Failed to allocate memory in PSRAM!");
        return;
    } else {
        Serial.println("PSRAM memory allocation successful.");
    }

    // Fill buffer with test pattern
    Serial.println("Writing test pattern...");
    for (size_t i = 0; i < test_size; i++) {
        psram_buffer[i] = (uint8_t)(i & 0xFF);  // Pattern 0x00 to 0xFF
    }

    // Verify data integrity
    Serial.println("Verifying data...");
    for (size_t i = 0; i < test_size; i++) {
        if (psram_buffer[i] != (uint8_t)(i & 0xFF)) {
            Serial.printf("PSRAM test failed at index %d!\n", (int)i);
            free(psram_buffer);
            return;
        }
    }

    Serial.println("PSRAM test passed! 🎉");
    
    // Free allocated memory
    free(psram_buffer);
}

void loop() {
    // Do nothing
}
