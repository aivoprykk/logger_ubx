/**
 * @file main.c
 * @brief UBX Message Handler Example Application
 * 
 * This example demonstrates the UBX message handling functionality without
 * using Unity test framework. It can work with either:
 * 1. Mock UART data (simulated UBX messages)
 * 2. Real UART connected to a UBX GPS module
 * 
 * Configure the mode in config.h by setting USE_REAL_UART
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

#include "config.h"
#include "ubx.h"
#include "ubx_msg.h"
#include "logger_common.h"

#if !USE_REAL_UART
#include "mock_uart.h"
#endif

static const char *TAG = "ubx_example";

// Demonstration context
typedef struct {
    ubx_ctx_t *ubx_ctx;
    ubx_msg_t ubx_msg;
    SemaphoreHandle_t msg_mutex;
    uint32_t messages_received;
    uint32_t messages_valid;
    uint32_t messages_invalid;
    uint32_t messages_dropped;      // Mutex timeout drops
    uint32_t messages_processed;    // Successfully processed
    int64_t demo_start_time;
    int64_t last_stats_time;
    uint32_t last_msg_count;        // For throughput calculation
    bool running;
} demo_ctx_t;

static demo_ctx_t g_demo_ctx = {0};

// Disabled for stress test - excessive logging
#if 0
/**
 * @brief Display message handler demonstration results
 */
static void display_message_info(const ubx_msg_byte_ctx_t *packet) {
    const char *msg_type = "UNKNOWN";
    uint8_t cls = packet->msg[0];
    uint8_t id = packet->msg[1];

    // Determine message type from class and ID
    if (cls == CLS_NAV && id == NAV_PVT) {
        msg_type = "NAV-PVT";
        nav_pvt_t *pvt = &packet->ubx_msg->navPvt;
        ESP_LOGI(TAG, "  Fix Type: %d, Satellites: %d", 
                 pvt->fixType, pvt->numSV);
        ESP_LOGI(TAG, "  Lat: %ld, Lon: %ld, Height: %ld mm",
                 pvt->lat, pvt->lon, pvt->height);
    }
    else if (cls == CLS_NAV && id == NAV_DOP) {
        msg_type = "NAV-DOP";
        nav_dop_t *dop = &packet->ubx_msg->navDOP;
        ESP_LOGI(TAG, "  gDOP: %d, pDOP: %d, hDOP: %d",
                 dop->gDOP, dop->pDOP, dop->hDOP);
    }
    else if (cls == CLS_ACK && id == ACK_ACK) {
        msg_type = "ACK-ACK";
    }
    else if (cls == CLS_ACK && id == ACK_NAK) {
        msg_type = "ACK-NAK";
    }
    else if (cls == CLS_MON && id == MON_VER) {
        msg_type = "MON-VER";
    }
    else if (cls == CLS_CFG) {
        msg_type = "CFG-MSG";
    }

    ESP_LOGI(TAG, "Message: %s (Class: 0x%02X, ID: 0x%02X, Len: %d)",
             msg_type, cls, id, packet->msg_len);
}
#endif // Disabled for stress test

/**
 * @brief Demonstrate circular buffer functionality
 */
#if 0  // Disabled for stress test
static void demo_circular_buffer(void) {
    ESP_LOGI(TAG, "\n=== Circular Buffer Demonstration ===");

    if (!g_demo_ctx.ubx_ctx) {
        ESP_LOGE(TAG, "UBX context not initialized");
        return;
    }

    ubx_ctx_t *ctx = g_demo_ctx.ubx_ctx;

    ESP_LOGI(TAG, "Buffer size: %d bytes", ctx->rx_buf_size);
    ESP_LOGI(TAG, "Buffer head: %d, tail: %d", ctx->rx_buf_head, ctx->rx_buf_tail);

    if (xSemaphoreTake(ctx->rx_buf_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        ESP_LOGI(TAG, "Buffer head: %d, tail: %d", ctx->rx_buf_head, ctx->rx_buf_tail);
        size_t available = (ctx->rx_buf_head >= ctx->rx_buf_tail) ?
                       (ctx->rx_buf_head - ctx->rx_buf_tail) :
                       (ctx->rx_buf_size - ctx->rx_buf_tail + ctx->rx_buf_head);
        xSemaphoreGive(ctx->rx_buf_mutex);
        ESP_LOGI(TAG, "Available data: %d bytes", available);
    } else {
        ESP_LOGW(TAG, "Could not access buffer state");
    }  

}
#endif // Disabled for stress test

/**
 * @brief Demonstrate message checksum validation (silent for performance)
 * @return true if checksum valid, false otherwise
 */
static bool demo_checksum_validation(const uint8_t *msg_data, size_t len) {
    // Silent checksum validation for performance testing
    if (len < 8) {
        return false;
    }

    // Verify UBX header
    if (msg_data[0] != 0xB5 || msg_data[1] != 0x62) {
        return false;
    }

    uint8_t CK_A = 0, CK_B = 0;

    // Calculate checksum over: Class + ID + Length + Payload
    // (skip header bytes 0-1, exclude checksum bytes at end)
    for (size_t i = 2; i < len - 2; i++) {
        CK_A += msg_data[i];
        CK_B += CK_A;
    }

    uint8_t expected_CK_A = msg_data[len - 2];
    uint8_t expected_CK_B = msg_data[len - 1];

    // Return validation result
    return (CK_A == expected_CK_A && CK_B == expected_CK_B);
}

/**
 * @brief Process received UBX data
 */
static esp_err_t process_ubx_data(const uint8_t *data, size_t len) {
    if (!data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    g_demo_ctx.messages_received++;

    // Create packet context for message handling
    ubx_msg_byte_ctx_t packet = {
        .msg = (uint8_t *)data,
        .msg_size = len,
        .msg_len = len,
        .ubx_msg = &g_demo_ctx.ubx_msg,
        .ubx_msg_type = MT_NONE
    };

    // Basic validation and display (simplified for stress testing - minimal logging)
    if (len >= 8) {
        bool checksum_valid = demo_checksum_validation(data, len);

        // Count valid vs invalid
        if (checksum_valid) {
            g_demo_ctx.messages_valid++;
        } else {
            g_demo_ctx.messages_invalid++;
            // Debug first failure with full message dump
            static bool first_failure_logged = false;
            if (!first_failure_logged) {
                ESP_LOGW(TAG, "Checksum failure - len=%d", len);
                ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, len, ESP_LOG_WARN);
                first_failure_logged = true;
            }
        }

        // Skip display for performance - only count messages
        // display_message_info(&packet);  // Disabled for stress test
    }

    return ESP_OK;
}

#if USE_REAL_UART
/**
 * @brief Initialize real UART for UBX communication
 */
static esp_err_t init_real_uart(void) {
    ESP_LOGI(TAG, "Initializing real UART for UBX module...");

    // Create UBX context
    g_demo_ctx.ubx_ctx = ubx_ctx_new();
    if (!g_demo_ctx.ubx_ctx) {
        ESP_LOGE(TAG, "Failed to create UBX context");
        return ESP_FAIL;
    }

    // Configure UART parameters (already set in UBX_DEFAULT_CTX)
    // But we can override if needed
    g_demo_ctx.ubx_ctx->uart_num = UART_NUM;
    g_demo_ctx.ubx_ctx->tx_pin = UART_TX_PIN;
    g_demo_ctx.ubx_ctx->rx_pin = UART_RX_PIN;
    g_demo_ctx.ubx_ctx->uart_conf.baud_rate = UART_BAUD_RATE;

    // Initialize UBX (this calls ubx_on which sets up UART)
    esp_err_t ret = ubx_setup(g_demo_ctx.ubx_ctx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UBX setup failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Real UART initialized successfully");
    ESP_LOGI(TAG, "UART: %d, TX: %d, RX: %d, Baud: %d",
             UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE);

    return ESP_OK;
}

/**
 * @brief Main task for real UART demonstration
 */
static void real_uart_demo_task(void *pvParameters) {
    ESP_LOGI(TAG, "Real UART demonstration started");

    uint8_t rx_buffer[256];
    int64_t last_log_time = 0;
    uint32_t mutex_failures = 0;  // Add counter

    while (g_demo_ctx.running) {
        // Check if message is ready (using ubx.c event-driven mechanism)
        if (g_demo_ctx.ubx_ctx->msg_ready) {
            if (xSemaphoreTake(g_demo_ctx.ubx_ctx->msg_ready, pdMS_TO_TICKS(1000)) == pdTRUE) {
                // Process the message from ubx_ctx
                if (xSemaphoreTake(g_demo_ctx.msg_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    // The message is already parsed in ubx_ctx->ubx_msg
                    ESP_LOGI(TAG, "Received message from UBX module");

                    // You can access the parsed message data here
                    // For full demonstration, we'd need to extract the raw bytes
                    // from the UBX context's circular buffer

                    xSemaphoreGive(g_demo_ctx.msg_mutex);
                }
                else {
                    mutex_failures++;
                    ESP_LOGW(TAG, "Mutex timeout - message dropped (%ld failures)", mutex_failures);
                }
            }
        }

        // Periodic status logging
        int64_t current_time = esp_timer_get_time() / 1000;
        if (current_time - last_log_time >= LOG_OUTPUT_INTERVAL_MS) {
            last_log_time = current_time;

            ESP_LOGI(TAG, "Status: Received=%ld, Valid=%ld, Invalid=%ld",
                     g_demo_ctx.messages_received,
                     g_demo_ctx.messages_valid,
                     g_demo_ctx.messages_invalid);

            demo_circular_buffer();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "Real UART demonstration ended");
    vTaskDelete(NULL);
}

#else // USE_REAL_UART

/**
 * @brief Performance monitoring task - tracks throughput and drops
 */
static void performance_monitor_task(void *pvParameters) {
    int64_t last_log_time = esp_timer_get_time() / 1000;
    uint32_t last_received = 0;
    uint32_t last_processed = 0;

    while (g_demo_ctx.running) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        int64_t current_time = esp_timer_get_time() / 1000;
        int64_t elapsed_ms = current_time - last_log_time;

        uint32_t current_received = g_demo_ctx.messages_received;
        uint32_t current_processed = g_demo_ctx.messages_processed;

        // Calculate throughput (messages per second)
        float receive_rate = (current_received - last_received) * 1000.0f / elapsed_ms;
        float process_rate = (current_processed - last_processed) * 1000.0f / elapsed_ms;

        ESP_LOGI(TAG, "=== PERFORMANCE METRICS ===");
        ESP_LOGI(TAG, "Receive Rate: %.1f msg/s | Process Rate: %.1f msg/s", 
                 receive_rate, process_rate);
        ESP_LOGI(TAG, "Total: RX=%ld PROC=%ld DROP=%ld VALID=%ld INVALID=%ld",
                 current_received, current_processed, 
                 g_demo_ctx.messages_dropped,
                 g_demo_ctx.messages_valid,
                 g_demo_ctx.messages_invalid);

        // Calculate drop percentage
        if (current_received > 0) {
            float drop_pct = (g_demo_ctx.messages_dropped * 100.0f) / current_received;
            ESP_LOGI(TAG, "Drop Rate: %.2f%%", drop_pct);
        }

        last_log_time = current_time;
        last_received = current_received;
        last_processed = current_processed;
    }

    vTaskDelete(NULL);
}

/**
 * @brief Main task for mock UART demonstration
 */
static void mock_uart_demo_task(void *pvParameters) {
    // Subscribe to watchdog
    esp_task_wdt_add(NULL);

    ESP_LOGI(TAG, "Mock UART demonstration started");
    ESP_LOGI(TAG, "Target rate: %d msg/sec (interval: %dms)", 
             1000/MOCK_DATA_INTERVAL_MS, MOCK_DATA_INTERVAL_MS);

    uint8_t data_buffer[256];

    while (g_demo_ctx.running) {
        // Generate mock data
        int bytes_generated = mock_uart_generate_data(data_buffer, sizeof(data_buffer));

        if (bytes_generated > 0) {
            // Try to process the data with timeout
            if (xSemaphoreTake(g_demo_ctx.msg_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                process_ubx_data(data_buffer, bytes_generated);
                g_demo_ctx.messages_processed++;
                xSemaphoreGive(g_demo_ctx.msg_mutex);
            } else {
                // Mutex timeout - message dropped
                g_demo_ctx.messages_dropped++;
                // Only log every 100 drops to avoid flooding
                if (g_demo_ctx.messages_dropped % 100 == 0) {
                    ESP_LOGW(TAG, "Mutex timeouts: %ld", g_demo_ctx.messages_dropped);
                }
            }
        }

        // Feed watchdog to prevent timeout
        esp_task_wdt_reset();

        // Yield to allow other tasks (including IDLE) to run
        taskYIELD();

        // Small delay for the configured interval (increased to reduce CPU load)
        vTaskDelay(pdMS_TO_TICKS(2));  // 2ms delay = ~500 msg/sec max
    }

    // Unsubscribe from watchdog before deleting task
    esp_task_wdt_delete(NULL);

    ESP_LOGI(TAG, "Mock UART demonstration ended");
    vTaskDelete(NULL);
}
#endif // USE_REAL_UART

/**
 * @brief Main application entry point
 */
void app_main(void) {
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "UBX Message Handler - STRESS TEST MODE");
    ESP_LOGI(TAG, "===========================================");

#if USE_REAL_UART
    ESP_LOGI(TAG, "Mode: REAL UART (connected to UBX GPS module)");
#else
    ESP_LOGI(TAG, "Mode: MOCK UART (HIGH SPEED - minimal logging)");
    ESP_LOGI(TAG, "Target: ~1000 msg/sec");
#endif

    ESP_LOGI(TAG, "Logging: Reduced for performance testing");
    ESP_LOGI(TAG, "Stats: Every %d seconds", STATS_INTERVAL_SECONDS);
    ESP_LOGI(TAG, "===========================================\n");

    // Initialize demonstration context
    memset(&g_demo_ctx, 0, sizeof(demo_ctx_t));
    memset(&g_demo_ctx.ubx_msg, 0, sizeof(ubx_msg_t));

    g_demo_ctx.msg_mutex = xSemaphoreCreateMutex();
    if (!g_demo_ctx.msg_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    g_demo_ctx.demo_start_time = esp_timer_get_time() / 1000;
    g_demo_ctx.running = true;

#if USE_REAL_UART
    // Initialize real UART
    esp_err_t ret = init_real_uart();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize real UART");
        return;
    }

    // Create demonstration task
    xTaskCreate(real_uart_demo_task, "real_uart_demo", 4096, NULL, 5, NULL);
#else
    // Initialize mock UART
    if (mock_uart_init() != 0) {
        ESP_LOGE(TAG, "Failed to initialize mock UART");
        return;
    }

    // Create performance monitoring task on CPU0
    xTaskCreatePinnedToCore(performance_monitor_task, "perf_monitor", 3072, NULL, 4, NULL, 0);

    // Create demonstration task on CPU1 to prevent starving IDLE0
    xTaskCreatePinnedToCore(mock_uart_demo_task, "mock_uart_demo", 4096, NULL, 5, NULL, 1);
#endif

    // Run demonstration for configured duration
    ESP_LOGI(TAG, "Running demonstration for %d ms...", DEMO_DURATION_MS);
    vTaskDelay(pdMS_TO_TICKS(DEMO_DURATION_MS));

    // Stop demonstration
    g_demo_ctx.running = false;
    vTaskDelay(pdMS_TO_TICKS(500)); // Allow task to finish

    // Display final results
    ESP_LOGI(TAG, "\n===========================================");
    ESP_LOGI(TAG, "Demonstration Complete");
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "Duration: %lld ms", 
             (esp_timer_get_time() / 1000) - g_demo_ctx.demo_start_time);
    ESP_LOGI(TAG, "Messages Received: %ld", g_demo_ctx.messages_received);
    ESP_LOGI(TAG, "Messages Valid:    %ld", g_demo_ctx.messages_valid);
    ESP_LOGI(TAG, "Messages Invalid:  %ld", g_demo_ctx.messages_invalid);

    if (g_demo_ctx.messages_received > 0) {
        float success_rate = (float)g_demo_ctx.messages_valid / 
                            (float)g_demo_ctx.messages_received * 100.0f;
        ESP_LOGI(TAG, "Success Rate: %.1f%%", success_rate);
    }

    // Cleanup
#if !USE_REAL_UART
    mock_uart_deinit();
#else
    if (g_demo_ctx.ubx_ctx) {
        ubx_off(g_demo_ctx.ubx_ctx);
        // Note: ubx_ctx_delete() would be needed if available
    }
#endif

    if (g_demo_ctx.msg_mutex) {
        vSemaphoreDelete(g_demo_ctx.msg_mutex);
    }

    ESP_LOGI(TAG, "Example finished. Restart to run again.");
}
