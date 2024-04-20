SET(SRCS ubx.c ubx_msg_handler.c)
SET(INCS include)
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS ${INCS}
    REQUIRES driver esp_event
    PRIV_REQUIRES logger_common logger_str
)
