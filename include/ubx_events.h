#ifndef A5F8A68E_84DA_4930_B4FB_2B117B977D6A
#define A5F8A68E_84DA_4930_B4FB_2B117B977D6A

#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <logger_common.h>

// Declare an event base
ESP_EVENT_DECLARE_BASE(UBX_EVENT);        // declaration of the UBX_EVENT family

#define UBX_EVENT_LIST(l) \
    l(UBX_EVENT_DATETIME_SET) \
    l(UBX_EVENT_PINS_INIT_DONE) \
    l(UBX_EVENT_PINS_INIT_FAIL) \
    l(UBX_EVENT_UART_DEINIT_DONE) \
    l(UBX_EVENT_UART_INIT_DONE) \
    l(UBX_EVENT_UART_INIT_FAIL) \
    l(UBX_EVENT_SETUP_DONE) \
    l(UBX_EVENT_SETUP_FAIL) \
    l(UBX_EVENT_MSG_RECIEVED)

// declaration of the specific events under the UBX_EVENT family
enum {                                       
    UBX_EVENT_LIST(ENUM)
};

 extern const char * const ubx_event_strings[];

#ifdef __cplusplus
}
#endif

#endif /* A5F8A68E_84DA_4930_B4FB_2B117B977D6A */
