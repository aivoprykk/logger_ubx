#ifndef A5F8A68E_84DA_4930_B4FB_2B117B977D6A
#define A5F8A68E_84DA_4930_B4FB_2B117B977D6A

#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

// Declare an event base
ESP_EVENT_DECLARE_BASE(UBX_EVENT);        // declaration of the UBX_EVENT family

// declaration of the specific events under the UBX_EVENT family
enum {                                       
    UBX_EVENT_DATETIME_SET,
    UBX_EVENT_PINS_INIT_DONE,
    UBX_EVENT_PINS_INIT_FAIL,
    UBX_EVENT_UART_DEINIT_DONE,
    UBX_EVENT_UART_INIT_DONE,
    UBX_EVENT_UART_INIT_FAIL,
    UBX_EVENT_SETUP_DONE,
    UBX_EVENT_SETUP_FAIL,
    UBX_EVENT_MSG_RECIEVED,
};

#ifdef __cplusplus
}
#endif

#endif /* A5F8A68E_84DA_4930_B4FB_2B117B977D6A */
