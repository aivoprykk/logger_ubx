#ifndef A5F8A68E_84DA_4930_B4FB_2B117B977D6A
#define A5F8A68E_84DA_4930_B4FB_2B117B977D6A

#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <logger_common.h>

#define UBX_EVENT_BASE 0x80  // Component ID 8

// Declare an event base
ESP_EVENT_DECLARE_BASE(UBX_EVENT);        // declaration of the UBX_EVENT family
#define UBX_EVENT_ENUM(l) UBX_EVENT_##l,
#define UBX_EVENT_LIST(l) \
    l(DATETIME_SET) \
    l(UART_DEINIT_DONE) \
    l(UART_INIT_DONE) \
    l(UART_INIT_FAIL) \
    l(SETUP_DONE) \
    l(SETUP_FAIL) \
    l(MSG_RECIEVED) \
    l(SAMPLE_RATE_CHANGED) \
    l(CONFIG_CHANGED) \
    l(NAV_MODE_CHANGED)

// declaration of the specific events under the UBX_EVENT family
enum {                                       
    UBX_EVENT_LIST(UBX_EVENT_ENUM)
};

 const char * ubx_event_strings(int id);

#ifdef __cplusplus
}
#endif

#endif /* A5F8A68E_84DA_4930_B4FB_2B117B977D6A */
