idf_component_register(
    SRCS "main.c" "mock_uart.c"
    INCLUDE_DIRS "."
    REQUIRES logger_ubx logger_common esp_timer
)
