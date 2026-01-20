# Test suite for logger_ubx component
# Run with: idf.py build test

idf_component_register(
    SRCS 
        "test_ubx_circular_buffer.c"
        "test_ubx_msg_handler.c"
        "test_ubx_uart_integration.c"
        "test_ubx_mocks.c"
    INCLUDE_DIRS 
        "."
        "../"
        "../include"
    REQUIRES 
        unity
        logger_ubx
        logger_common
        freertos
        driver
    PRIV_REQUIRES
        logger_config
        strutil
)

# Set component dependencies
set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 11)
set_property(TARGET ${COMPONENT_LIB} PROPERTY C_STANDARD 99)

# Add test-specific compile definitions
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    CONFIG_UBLOX_ENABLED=1
    CONFIG_UBLOX_LOG_LEVEL=3
    UNITY_INCLUDE_CONFIG_H
)

# Enable warnings for test code
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    -Werror
    -Wno-unused-parameter
)
