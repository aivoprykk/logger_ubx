/**
 * @file test_ubx_mocks.h
 * @brief Mock implementations and test helpers for UBX tests
 * 
 * Provides mock UART functions and test utilities for isolated testing
 */

#ifndef TEST_UBX_MOCKS_H
#define TEST_UBX_MOCKS_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Mock UART queue for testing
 */
typedef struct {
    uint8_t *buffer;
    size_t buffer_size;
    size_t write_pos;
    size_t read_pos;
    size_t available;
} mock_uart_queue_t;

/**
 * @brief Initialize mock UART
 */
void mock_uart_init(void);

/**
 * @brief Deinitialize mock UART
 */
void mock_uart_deinit(void);

/**
 * @brief Write data to mock UART RX buffer (simulates GPS sending data)
 * @param data Data to write
 * @param len Length of data
 * @return Number of bytes written
 */
int mock_uart_write_rx(const uint8_t *data, size_t len);

/**
 * @brief Read data from mock UART TX buffer (simulates reading what was sent to GPS)
 * @param data Buffer to read into
 * @param len Maximum bytes to read
 * @return Number of bytes read
 */
int mock_uart_read_tx(uint8_t *data, size_t len);

/**
 * @brief Get available bytes in mock UART RX buffer
 * @return Number of available bytes
 */
size_t mock_uart_get_rx_available(void);

/**
 * @brief Clear mock UART buffers
 */
void mock_uart_flush(void);

/**
 * @brief Simulate UART error condition
 * @param error_type UART error type
 */
void mock_uart_inject_error(uart_event_type_t error_type);

/**
 * @brief Check if mock UART has pending events
 * @return true if events pending
 */
bool mock_uart_has_events(void);

/**
 * @brief Get next mock UART event
 * @param event Output event
 * @param timeout_ms Timeout in milliseconds
 * @return true if event retrieved
 */
bool mock_uart_get_event(uart_event_t *event, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // TEST_UBX_MOCKS_H
