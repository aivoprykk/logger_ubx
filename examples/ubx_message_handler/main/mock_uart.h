#ifndef MOCK_UART_H
#define MOCK_UART_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Initialize mock UART data generator
 * @return 0 on success, negative on error
 */
int mock_uart_init(void);

/**
 * @brief Generate mock UBX data
 * @param buffer Output buffer for mock data
 * @param max_len Maximum buffer size
 * @return Number of bytes written, 0 if no data
 */
int mock_uart_generate_data(uint8_t *buffer, size_t max_len);

/**
 * @brief Cleanup mock UART
 */
void mock_uart_deinit(void);

#endif // MOCK_UART_H
