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

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "stdbool.h"
#include "stdint.h"
#include "ubx_msg.h"
#include "ubx_events.h"

#define UBX_TYPE_LIST(l) l(UBX_TYPE, M0, 0x00) l(UBX_TYPE, M7, 0x07) l(UBX_TYPE, M8, 0x08) l(UBX_TYPE, M9, 0x09) l(UBX_TYPE, M10, 0x0A)
typedef enum ubx_hw_e {
    UBX_TYPE_LIST(ENUM_VV)
} ubx_hw_t;
#define UBX_HW_COUNT 5
#define UBX_HW_TYPE_DEFAULT UBX_TYPE_M0
// extern const char * const ubx_hw_type_strings[];
/**
 * @brief Navigation mode enum.
 *
 * @details
 *     - Portable (automatic)
 *     - Stationary (time-only)
 *     - Pedestrian/Automotive/Sea
 *     - Airborne (1G, 2G and 4G max modes)
 */

#define UBX_NAV_MODE_LIST(l) l(UBX_MODE_PORTABLE, 0x00) l(UBX_MODE_SEA, 0x01) l(UBX_MODE_AUTOMOTIVE, 0x02) l(UBX_MODE_STATIONARY, 0x03) l(UBX_MODE_PEDESTRIAN, 0x04) l(UBX_MODE_AIR_1G_MAX, 0x05) l(UBX_MODE_AIR_2G_MAX, 0x06) l(UBX_MODE_AIR_4G_MAX, 0x07)
typedef enum ubx_nav_mode_e {
    UBX_NAV_MODE_LIST(ENUM_V)
} ubx_nav_mode_t;

#define UBX_BAUD_RATE_LIST(l) l(UBX_BAUD_9600, 9600) l(UBX_BAUD_38400, 38400) l(UBX_BAUD_57600, 57600) l(UBX_BAUD_115200, 115200) l(UBX_BAUD_230400, 230400)
typedef enum ubx_baud_rate_e {
    UBX_BAUD_RATE_LIST(ENUM_V)
} ubx_baud_rate_t;
#define BAUD_RATE_COUNT 5
#define UBX_BAUD_RATE_DEFAULT UBX_BAUD_38400

/**
 * @brief Message output rate enum.
 */
#define UBX_OUTPUT_RATES_LIST(l) l(UBX_OUTPUT_1HZ, 0x01) l(UBX_OUTPUT_2HZ, 0x02) l(UBX_OUTPUT_5HZ, 0x05) l(UBX_OUTPUT_10HZ, 0x0a) l(UBX_OUTPUT_16HZ, 0x10) l(UBX_OUTPUT_20HZ, 0x14)
typedef enum ubx_output_rate_e {
    UBX_OUTPUT_RATES_LIST(ENUM_V)
} ubx_output_rate_t;
#define OUTPUT_RATE_COUNT 6
#define UBX_OUTPUT_RATE_DEFAULT UBX_OUTPUT_10HZ

/**
 * @brief NMEA Messages enum.
 */
#define UBX_MESSAGE_LIST(l) l(UBX_MSG_DTM, 0x0A) l(UBX_MSG_GBS, 0x09) l(UBX_MSG_GGA, 0x00) l(UBX_MSG_GLL, 0x01) l(UBX_MSG_GRS, 0x06) l(UBX_MSG_GSA, 0x02) l(UBX_MSG_GST, 0x07) l(UBX_MSG_GSV, 0x03) l(UBX_MSG_RMC, 0x04) l(UBX_MSG_VTG, 0x05) l(UBX_MSG_ZDA, 0x08)
typedef enum ubx_message_e {
    UBX_MESSAGE_LIST(ENUM_V)
} ubx_message_t;

#define UBX_MSG_TYPE_LIST(l) l(MT_NONE, 0x00) l(MT_NAV_DUMMY, 0x01) l(MT_NAV_PVT, 0x02) l(MT_NAV_ACK, 0x03) l(MT_NAV_NACK, 0x04) l(MT_NAV_ID, 0x05) l(MT_MON_GNSS, 0x06) l(MT_NAV_DOP, 0x07) l(MT_MON_VER, 0x08) l(MT_NAV_SAT, 0x09)
typedef enum ubx_msg_type_e {
    UBX_MSG_TYPE_LIST(ENUM_V)
} ubx_msg_type_t;

#define UBX_EN_PIN_LEN 4

typedef struct ubx_rtc_config_s {
    ubx_hw_t hw_type;
    uint32_t baud;
    ubx_output_rate_t output_rate;
    ubx_nav_mode_t nav_mode;
    uint8_t hw_id[6];
    uint8_t prot_ver;
    uint8_t gnss;
    uint8_t gnss_count;
    bool msgout_sat;
} ubx_rtc_config_t;

#define UBX_RTC_DEFAULT_CONFIG() {  \
    .hw_type = UBX_TYPE_M0,    \
    .baud = UBX_BAUD_115200,        \
    .output_rate = UBX_OUTPUT_10HZ, \
    .nav_mode = UBX_MODE_SEA,       \
    .hw_id = {0},                   \
    .prot_ver = 0,                  \
    .gnss = 111, /* 01101111 */      \
    .gnss_count = 4,                \
    .msgout_sat = true,            \
}

/**
 * @brief GPS configuration structure.
 *
 * @details Please do NOT change the structure directly.
 */
typedef struct ubx_config_s {
    uart_port_t uart_num;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    gpio_num_t en_pins[4];
    uart_config_t uart_conf;
    ubx_rtc_config_t * rtc_conf;
    ubx_msg_t ubx_msg;
    char Ublox_type[20];
    bool uart_setup_ok;
    bool config_ok;
    bool config_progress;
    bool ready;
    bool time_set;
    bool signal_ok;
    bool is_on;
    uint16_t first_fix;
    uint32_t next_time_sync;
    uint32_t ready_time;
} ubx_config_t;

/**
 * @brief Default GPS configuration structure.
 */
#define UBX_DEFAULT_CONFIG() {                        \
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
    .uart_setup_ok = false,                           \
    .config_ok = false,                               \
    .config_progress = false,                         \
    .ready = false,                                \
    .time_set = false,                                \
    .signal_ok = false,                               \
    .is_on = false,                                  \
    .first_fix = 0,                                   \
    .next_time_sync = 0,                              \
    .ready_time = 0,                                    \
}

struct ubx_msg_byte_ctx_s;

typedef esp_err_t (*ubx_handler_cb)(struct ubx_msg_byte_ctx_s*);

typedef struct ubx_msg_byte_ctx_s {
    uint8_t * msg;
    uint16_t msg_size;
    uint16_t msg_len;
    uint16_t msg_pos;
    bool msg_match_to_pos;
    bool expect_ubx_msg;
    uint8_t ubx_msg_type;
    ubx_handler_cb msg_type_handler;
    ubx_handler_cb msg_ready_handler;
    ubx_config_t * ubx;
} ubx_msg_byte_ctx_t;

#define UBX_MSG_BYTE_CTX_DEFAULT() { \
    .msg = &(ubx->ubx_msg.none[0]),                     \
    .msg_size = UBX_NONE_SIZE,                    \
    .msg_len = 0,                    \
    .msg_pos = 2,                    \
    .msg_match_to_pos = true,       \
    .expect_ubx_msg = true,       \
    .ubx_msg_type = 0,               \
    .msg_type_handler = ubx_msg_type_handler,        \
    .msg_ready_handler = NULL,       \
    .ubx = ubx,                     \
}

/**
 * @brief Initializes the GPS configuration structure.
 *
 * @return
 * ubx_config_t*   Success
 * NULL            Parameter error
 */
ubx_config_t * ubx_config_new();

/**
 * @brief Deletes the GPS configuration structure.
 * 
 * @param *ubx is the address of the GPS configuration structure.
 * 
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t ubx_config_delete(ubx_config_t *ubx);

/**
 * @brief Initializes the GPS configuration structure.
 * 
 * @param *ubx is the address of the GPS configuration structure.
 * 
 * @return
 *    - ESP_OK   Success
 *   - ESP_FAIL Parameter error
*/
esp_err_t ubx_config_init(ubx_config_t *ubx);

/**
 * @brief Deinitializes the GPS configuration structure.
 * 
 * @param *ubx is the address of the GPS configuration structure.
*/
esp_err_t ubx_config_deinit(ubx_config_t *ubx);

/**
 * @brief Initializes the serial communication for the GPS.
 *
 * @details PPS pin function not implemented yet.
 *
 * @param *ubx is the address of GPS configuration structure.
 * 
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t ubx_uart_init(ubx_config_t *ubx);

/**
 * @brief Deinitializes the serial communication for the GPS.
 *
 * @param *ubx is the address of GPS configuration structure.
 * 
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t ubx_uart_deinit(ubx_config_t *ubx);

/**
 * @brief Initializes the GPS enable pins.
 *
 * @param *ubx is the address of the GPS configuration structure.
 * 
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t ubx_pins_init(ubx_config_t *ubx);

/**
 * @brief Deinitializes the GPS enable pins.
 *
 * @param *ubx is the address of the GPS configuration structure.
 * 
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t ubx_pins_deinit(ubx_config_t *ubx);

/**
 * @brief Powers on the GPS.
 *
 * @param *ubx is the address of the GPS configuration structure.
 * 
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t ubx_on(ubx_config_t *ubx);

/**
 * @brief Powers off the GPS.
 *
 * @param *ubx is the address of the GPS configuration structure.
 * 
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t ubx_off(ubx_config_t *ubx);

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
esp_err_t ubx_set_nav_mode(ubx_config_t *ubx, ubx_nav_mode_t nav_mode);

/**
 * @brief Sets the GPS message output rate.
 *
 * @param *ubx is the address of the GPS configuration structure.
 *
 * @return
 *     - ESP_OK                Success
 *     - ESP_ERR_INVALID_ARG   Invalid rate parameter
 *     - ESP_FAIL              Not all bytes sent
 *     - ESP_FAIL              Parameter error
 *     - ESP_ERR_TIMEOUT       Not all bytes read
 *     - ESP_ERR_TIMEOUT       No ACK received within ACK_TIMEOUT
 *     - ESP_INVALID_RESPONSE  NAK received
 *     - ESP_ERR_INVALID_CRC   Checksum for the wrong message received
 */
esp_err_t ubx_set_output_rate(ubx_config_t *ubx);

/**
 * @brief Sets the GPS message rate to be output.
 *
 * @param *ubx is the address of the GPS configuration structure.
 * @param rate is the NMEA message.
 * @param active is if the message will be output.
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
esp_err_t ubx_set_message_rate(ubx_config_t *ubx, ubx_message_t message, bool active);

/**
 * @brief Set ublox to output UBX messages only.
 * 
 * @param *ubx is the address of the GPS configuration structure.
 * 
 * @return
 *   - ESP_OK                Success
 *   - ESP_FAIL              Not all bytes sent
*/
esp_err_t ubx_set_ubxout(ubx_config_t *ubx);

/**
 * @brief Ubx message handler
 * 
 * @param *ubx is the address of the GPS configuration structure.
 * @param *arg is the address of the message handler.
 * 
    * @return
    *  - ESP_OK                Success
    * - ESP_FAIL              Not all bytes sent
    * - ESP_FAIL              Parameter error
    * - ESP_ERR_TIMEOUT       Not all bytes read
    * - ESP_ERR_TIMEOUT       No ACK received within ACK_TIMEOUT
    * - ESP_INVALID_RESPONSE  NAK received
    * - ESP_ERR_INVALID_CRC   Checksum for the wrong message received
    * - ESP_ERR_INVALID_ARG   Invalid rate parameter
*/
esp_err_t ubx_msg_handler(ubx_msg_byte_ctx_t *);
esp_err_t ubx_msg_type_handler(ubx_msg_byte_ctx_t * mctx);
esp_err_t ubx_msg_checksum_handler(ubx_msg_byte_ctx_t * mctx);
esp_err_t ubx_msg_byte_ctx_reset(ubx_msg_byte_ctx_t * mctx);

int8_t ubx_set_time(ubx_config_t *ubx, float time_offset);

esp_err_t ubx_setup(ubx_config_t *ubx);

const char * ubx_chip_str(const ubx_config_t *ubx);

const char * ubx_baud_str(const ubx_config_t *ubx);

#ifdef __cplusplus
}
#endif

#endif /* E8FD891D_3201_47A6_94CA_E3060D52E3AD */
// Path: components/logger_ubx/include/ubx_msg.h
