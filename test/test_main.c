/**
 * @file test_main.c
 * @brief Test runner main entry point
 * 
 * Unity test framework entry point for logger_ubx test suite
 */

#include <stdio.h>
#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "ubx_test_main";

/**
 * @brief Unity test setUp - called before each test
 */
void setUp(void) {
    // Per-test setup is handled in individual test files
}

/**
 * @brief Unity test tearDown - called after each test
 */
void tearDown(void) {
    // Per-test cleanup is handled in individual test files
}

/**
 * @brief Test application entry point
 */
void app_main(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  UBX GPS Logger Test Suite");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Testing components:");
    ESP_LOGI(TAG, "  - Circular buffer operations");
    ESP_LOGI(TAG, "  - UBX message parsing");
    ESP_LOGI(TAG, "  - Checksum validation");
    ESP_LOGI(TAG, "  - UART integration");
    ESP_LOGI(TAG, "");

    // Wait for system to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));

    // Run all Unity tests
    UNITY_BEGIN();

    // Tests are auto-registered via TEST_CASE macro
    // Unity will discover and run them
    unity_run_all_tests();

    UNITY_END();

    // Keep task alive for monitoring
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
