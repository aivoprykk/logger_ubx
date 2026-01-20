/**
 * @file test_ubx_mocks.c
 * @brief Mock implementations for UBX testing
 */

#include "test_ubx_mocks.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define MOCK_UART_BUF_SIZE 2048
#define MOCK_UART_EVENT_QUEUE_SIZE 20

// Mock UART buffers
static struct {
    uint8_t *rx_buffer;  // Simulates data from GPS module
    size_t rx_write_pos;
    size_t rx_read_pos;
    size_t rx_available;
    
    uint8_t *tx_buffer;  // Simulates data sent to GPS module
    size_t tx_write_pos;
    size_t tx_available;
    
    QueueHandle_t event_queue;
    bool initialized;
} mock_uart = {0};

void mock_uart_init(void) {
    if (mock_uart.initialized) {
        return;
    }
    
    mock_uart.rx_buffer = malloc(MOCK_UART_BUF_SIZE);
    mock_uart.tx_buffer = malloc(MOCK_UART_BUF_SIZE);
    mock_uart.event_queue = xQueueCreate(MOCK_UART_EVENT_QUEUE_SIZE, sizeof(uart_event_t));
    
    mock_uart.rx_write_pos = 0;
    mock_uart.rx_read_pos = 0;
    mock_uart.rx_available = 0;
    mock_uart.tx_write_pos = 0;
    mock_uart.tx_available = 0;
    
    mock_uart.initialized = true;
}

void mock_uart_deinit(void) {
    if (!mock_uart.initialized) {
        return;
    }
    
    if (mock_uart.rx_buffer) {
        free(mock_uart.rx_buffer);
        mock_uart.rx_buffer = NULL;
    }
    
    if (mock_uart.tx_buffer) {
        free(mock_uart.tx_buffer);
        mock_uart.tx_buffer = NULL;
    }
    
    if (mock_uart.event_queue) {
        vQueueDelete(mock_uart.event_queue);
        mock_uart.event_queue = NULL;
    }
    
    mock_uart.initialized = false;
}

int mock_uart_write_rx(const uint8_t *data, size_t len) {
    if (!mock_uart.initialized || !data || len == 0) {
        return 0;
    }
    
    size_t space = MOCK_UART_BUF_SIZE - mock_uart.rx_available;
    size_t to_write = (len < space) ? len : space;
    
    for (size_t i = 0; i < to_write; i++) {
        mock_uart.rx_buffer[mock_uart.rx_write_pos] = data[i];
        mock_uart.rx_write_pos = (mock_uart.rx_write_pos + 1) % MOCK_UART_BUF_SIZE;
        mock_uart.rx_available++;
    }
    
    // Post UART data event
    if (to_write > 0) {
        uart_event_t event = {
            .type = UART_DATA,
            .size = to_write
        };
        xQueueSend(mock_uart.event_queue, &event, 0);
    }
    
    return (int)to_write;
}

int mock_uart_read_tx(uint8_t *data, size_t len) {
    if (!mock_uart.initialized || !data || len == 0) {
        return 0;
    }
    
    size_t to_read = (len < mock_uart.tx_available) ? len : mock_uart.tx_available;
    
    memcpy(data, mock_uart.tx_buffer, to_read);
    
    // Shift remaining data
    if (to_read < mock_uart.tx_available) {
        memmove(mock_uart.tx_buffer, mock_uart.tx_buffer + to_read, 
                mock_uart.tx_available - to_read);
    }
    
    mock_uart.tx_available -= to_read;
    
    return (int)to_read;
}

size_t mock_uart_get_rx_available(void) {
    return mock_uart.initialized ? mock_uart.rx_available : 0;
}

void mock_uart_flush(void) {
    if (!mock_uart.initialized) {
        return;
    }
    
    mock_uart.rx_write_pos = 0;
    mock_uart.rx_read_pos = 0;
    mock_uart.rx_available = 0;
    mock_uart.tx_write_pos = 0;
    mock_uart.tx_available = 0;
    
    // Clear event queue
    if (mock_uart.event_queue) {
        xQueueReset(mock_uart.event_queue);
    }
}

void mock_uart_inject_error(uart_event_type_t error_type) {
    if (!mock_uart.initialized || !mock_uart.event_queue) {
        return;
    }
    
    uart_event_t event = {
        .type = error_type,
        .size = 0
    };
    
    xQueueSend(mock_uart.event_queue, &event, 0);
}

bool mock_uart_has_events(void) {
    if (!mock_uart.initialized || !mock_uart.event_queue) {
        return false;
    }
    
    return uxQueueMessagesWaiting(mock_uart.event_queue) > 0;
}

bool mock_uart_get_event(uart_event_t *event, uint32_t timeout_ms) {
    if (!mock_uart.initialized || !mock_uart.event_queue || !event) {
        return false;
    }
    
    return xQueueReceive(mock_uart.event_queue, event, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

/**
 * @brief Mock uart_read_bytes for testing
 * This can be used to override the real UART driver in tests
 */
int uart_read_bytes(uart_port_t uart_num, uint8_t *buf, uint32_t length, TickType_t ticks_to_wait) {
    (void)uart_num;
    (void)ticks_to_wait;
    
    if (!mock_uart.initialized || !buf || length == 0) {
        return 0;
    }
    
    size_t to_read = (length < mock_uart.rx_available) ? length : mock_uart.rx_available;
    
    for (size_t i = 0; i < to_read; i++) {
        buf[i] = mock_uart.rx_buffer[mock_uart.rx_read_pos];
        mock_uart.rx_read_pos = (mock_uart.rx_read_pos + 1) % MOCK_UART_BUF_SIZE;
    }
    
    mock_uart.rx_available -= to_read;
    
    return (int)to_read;
}

/**
 * @brief Mock uart_write_bytes for testing
 */
int uart_write_bytes(uart_port_t uart_num, const void *src, size_t size) {
    (void)uart_num;
    
    if (!mock_uart.initialized || !src || size == 0) {
        return 0;
    }
    
    size_t space = MOCK_UART_BUF_SIZE - mock_uart.tx_available;
    size_t to_write = (size < space) ? size : space;
    
    memcpy(mock_uart.tx_buffer + mock_uart.tx_write_pos, src, to_write);
    mock_uart.tx_write_pos += to_write;
    mock_uart.tx_available += to_write;
    
    return (int)to_write;
}

/**
 * @brief Mock uart_flush_input for testing
 */
esp_err_t uart_flush_input(uart_port_t uart_num) {
    (void)uart_num;
    
    if (!mock_uart.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    mock_uart.rx_write_pos = 0;
    mock_uart.rx_read_pos = 0;
    mock_uart.rx_available = 0;
    
    return ESP_OK;
}
