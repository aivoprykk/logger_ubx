
idf_component_register(
    SRCS ubx.c
    INCLUDE_DIRS "include"
    REQUIRES driver esp_event
    PRIV_REQUIRES logger_common
)
