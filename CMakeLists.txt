SET(SRCS ubx.c ubx_nav_mode.c ubx_msg_handler.c ubx_uart_event.c)
SET(INCS include)

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS ${INCS}
    REQUIRES esp_event
    PRIV_REQUIRES logger_common strutil
)

if(NOT ${IDF_TARGET} STREQUAL "linux")
    target_link_libraries(${COMPONENT_LIB} PUBLIC idf::driver)
endif()