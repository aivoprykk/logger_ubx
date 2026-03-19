#ifndef E8FD891D_3201_47A6_94CA_E3060D52E3AD
#define E8FD891D_3201_47A6_94CA_E3060D52E3AD

/*
 * @brief This implementation is based on a library for the uBlox GPS
 * @see https://github.com/aedalzotto/esp32-ublox.git
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"
#if defined(CONFIG_UBLOX_ENABLED)

#include "ubx_events.h"
#include "ubx_msg.h"


#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "stdbool.h"
#include "stdint.h"

#include "config_ubx.h"
#include "ubx_nav_mode.h"

/**
 * @brief Navigation mode enum.
 *
 * @details
 *     - Portable (automatic)
 *     - Stationary (time-only)
 *     - Pedestrian/Automotive/Sea
 *     - Airborne (1G, 2G and 4G max modes)
 */

/**
 * @brief GPS configuration structure.
 *
 * @details Please do NOT change the structure directly.
 */

#define UBX_TYPE_LIST(l) \
    l(UBX_TYPE, M0, 0x00) \
    l(UBX_TYPE, M7, 0x07) \
    l(UBX_TYPE, M8, 0x08) \
    l(UBX_TYPE, M9, 0x09) \
    l(UBX_TYPE, M10, 0x0A)
typedef enum ubx_hw_e {
    UBX_TYPE_LIST(ENUM_VV)
} ubx_hw_t;

#define UBX_HW_COUNT 5
#define UBX_HW_TYPE_DEFAULT UBX_TYPE_M0

typedef struct ubx_ctx_s {
    uart_port_t uart_num;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    gpio_num_t en_pins[4];
    uart_config_t uart_conf;
    cfg_ubx_t * rtc_conf;
    ubx_msg_t ubx_msg;
    char Ublox_type[20];
    ubx_hw_t hw_type;
    uint8_t hw_id[8];
    uint8_t prot_ver;
    uint8_t gnss_count;
    /// UART event-driven infrastructure
    QueueHandle_t uart_event_queue;
    TaskHandle_t uart_event_task;
    uint8_t *rx_buffer;
    size_t rx_buf_size;
    size_t rx_buf_head;
    size_t rx_buf_tail;
    SemaphoreHandle_t rx_buf_mutex;
    uint8_t *uart_tmp_buf;            ///< scratch buffer for UART reads (dynamic, freed on deinit)
    SemaphoreHandle_t msg_ready;  // Binary semaphore: signaled when message is ready
    uint32_t last_rx_ms;
    uint32_t last_valid_ms;
    bool link_lost;
    /// state flags
    bool uart_is_on;
    bool setup_progress;
    bool initialized;
    bool ready;
    uint32_t ready_time;
    bool shutdown_requested;
    volatile bool reconfig_requested;  ///< Abort in-progress setup for new config
    volatile bool nav_mode_apply_requested;
} ubx_ctx_t;

/**
 * @brief Default GPS configuration structure.
 */

#define UBX_DEFAULT_CTX() {                        \
    .uart_num = CONFIG_UBLOX_UART_PORT,               \
    .tx_pin = CONFIG_UBLOX_UART_TXD,                  \
    .rx_pin = CONFIG_UBLOX_UART_RXD,                  \
    .en_pins = {CONFIG_UBLOX_UART_PWR_1, CONFIG_UBLOX_UART_PWR_2, CONFIG_UBLOX_UART_PWR_3, UART_PIN_NO_CHANGE}, \
    .uart_conf = {                                    \
        .baud_rate = UBX_BAUD_9600,                  \
        .data_bits = UART_DATA_8_BITS,                \
        .parity = UART_PARITY_DISABLE,                \
        .stop_bits = UART_STOP_BITS_1,                \
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,        \
        .rx_flow_ctrl_thresh = 122,                   \
    },                                                \
    .rtc_conf = NULL,                                 \
    .ubx_msg = UBX_MSG_DEFAULT,                     \
    .Ublox_type = "Ublox unknown...", \
    .hw_type = UBX_HW_TYPE_DEFAULT,               \
    .hw_id = {0,0,0,0,0,0},                           \
    .prot_ver = 0,                                   \
    .gnss_count = 0,                                 \
    .uart_event_queue = NULL,                        \
    .uart_event_task = NULL,                         \
    .rx_buffer = NULL,                               \
    .rx_buf_size = 0,                                \
    .rx_buf_head = 0,                                \
    .rx_buf_tail = 0,                                \
    .rx_buf_mutex = NULL,                            \
    .uart_tmp_buf = NULL,                            \
    .last_rx_ms = 0,                                 \
    .last_valid_ms = 0,                              \
    .link_lost = false,                              \
    .uart_is_on = false,                           \
    .setup_progress = false,                         \
    .initialized = false,                               \
    .ready = false,                                \
    .ready_time = 0,                                \
    .shutdown_requested = false,                     \
    .reconfig_requested = false,                     \
    .nav_mode_apply_requested = false,               \
}

/**
 * @brief Initializes the GPS configuration structure.
 *
 * @return
 * ubx_ctx_t*   Success
 * NULL            Parameter error
 */
ubx_ctx_t * ubx_ctx_new();

/**
 * @brief Deletes the GPS configuration structure.
 * 
 * @param *ubx is the address of the GPS configuration structure.
 * 
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t ubx_ctx_delete(ubx_ctx_t *ubx);

/**
 * @brief Powers on the GPS.
 *
 * @param *ubx is the address of the GPS configuration structure.
 * 
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t ubx_on(ubx_ctx_t *ubx);

/**
 * @brief Powers off the GPS.
 *
 * @param *ubx is the address of the GPS configuration structure.
 * 
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t ubx_off(ubx_ctx_t *ubx);

/**
 * @brief Sets the GPS navigation mode.
 *
 * @param *ubx is the address of the GPS configuration structure.
 *
 * @return
 *     - ESP_OK                Success
 *     - ESP_FAIL              Not all bytes sent
 *     - ESP_FAIL              Parameter error
 *     - ESP_ERR_TIMEOUT       Not all bytes read
 *     - ESP_ERR_TIMEOUT       No ACK received within ACK_TIMEOUT
 *     - ESP_INVALID_RESPONSE  NAK received
 *     - ESP_ERR_INVALID_CRC   Checksum for the wrong message received
 */
esp_err_t ubx_set_nav_mode(ubx_ctx_t *ubx, ubx_nav_mode_t nav_mode);
void ubx_request_nav_mode_apply(ubx_ctx_t *ubx);
esp_err_t ubx_apply_pending_nav_mode(ubx_ctx_t *ubx);

esp_err_t ubx_set_gnss_and_rate(ubx_ctx_t *ubx_dev, uint8_t gnss, uint8_t rate);

int8_t ubx_set_time(ubx_ctx_t *ubx, float time_offset);

esp_err_t ubx_setup(ubx_ctx_t *ubx);

// const char * ubx_chip_str(const ubx_ctx_t *ubx);
const char * ubx_get_dev_str(void);

const char * ubx_baud_str(const ubx_ctx_t *ubx);

// extern cfg_ubx_t rtc_config;

#endif

#ifdef __cplusplus
}
#endif

#endif /* E8FD891D_3201_47A6_94CA_E3060D52E3AD */
// Path: components/logger_ubx/include/ubx_msg.h
