#ifndef EXAMPLE_CONFIG_H
#define EXAMPLE_CONFIG_H

// Select mode: 0 = Mock UART, 1 = Real UART
#define USE_REAL_UART 0

#if USE_REAL_UART
    // Real UART configuration (from ubx.c)
    #define UART_NUM UART_NUM_1
    #define UART_TX_PIN 17
    #define UART_RX_PIN 16
    #define UART_BAUD_RATE 9600
    #define UART_BUF_SIZE 1024
#else
    // Mock UART settings - stress test configuration
    // 1ms = ~1000 msg/sec, 5ms = ~200 msg/sec, 10ms = ~100 msg/sec
    #define MOCK_DATA_INTERVAL_MS 1  // Stress test: maximum rate
#endif

// Example demonstration settings
#define DEMO_DURATION_MS 60000  // Run demo for 60 seconds (stress test)
#define LOG_OUTPUT_INTERVAL_MS 1000  // Log status every 1 second
#define STATS_INTERVAL_SECONDS 5  // Performance stats interval

#endif // EXAMPLE_CONFIG_H
