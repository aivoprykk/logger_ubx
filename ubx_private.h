#ifndef F77B6D3D_E33D_4ED3_B35C_5404E7A31138
#define F77B6D3D_E33D_4ED3_B35C_5404E7A31138

#ifdef __cplusplus
extern "C" {
#endif


#include <string.h>
#include <stdint.h>
#include <esp_err.h>
#include <stdbool.h>

// #include "ubx.h"

/*!< Timeout for the ubx message */
#define MSG_READ_TIMEOUT 2000

/*!< UBX Protocol headers */
#define UBX_HDR_A 0xB5
#define UBX_HDR_B 0x62
#define UBX_HDR {UBX_HDR_A, UBX_HDR_B}

/*!< UBX Message classes */
#define CLS_NONE 0x00
#define CLS_NAV  0x01
#define CLS_INF  0x04
#define CLS_ACK  0x05
#define CLS_CFG  0x06
#define CLS_LOG  0x21
#define CLS_SEC  0x27
#define CLS_MON  0x0A

/*!< None Message IDs */
#define NONE_NONE 0x00

/*!< ACK Message IDs */
#define ACK_ACK 0x01
#define ACK_NAK 0x00

/*!< NAV Message IDs */
#define NAV_DOP 0x04
#define NAV_PVT 0x07
#define NAV_CLK 0x22
#define NAV_SAT 0x35
#define NAV_SAT_LEN 1120

/*!< CFG Message IDs */
#define CFG_PRT  0x00
#define CFG_MSG  0x01
#define CFG_RATE 0x08
#define CFG_CFG  0x09
#define CFG_NMEA 0x17
#define CFG_NAV5 0x24
#define CFG_VALSET 0x8a
#define CFG_VALGET 0x8b
#define CFG_VALDEL 0x8c
#define CFG_GNSS 0x3e

/*!< MON Message IDs */
#define MON_GNSS 0x28
#define MON_GNSS_LEN 16
#define MON_VER  0x04
#define MON_VER_LEN 228
#define MON_HW   0x09

/*!< SEC Message IDs */
#define SEC_UBX  0x03
#define SEC_UBX_LEN 18

#include "sdkconfig.h"
#if (defined(CONFIG_LOGGER_USE_GLOBAL_LOG_LEVEL) && CONFIG_LOGGER_GLOBAL_LOG_LEVEL < CONFIG_UBLOX_LOG_LEVEL)
#define C_LOG_LEVEL CONFIG_LOGGER_GLOBAL_LOG_LEVEL
#else
#define C_LOG_LEVEL CONFIG_UBLOX_LOG_LEVEL
#endif
#include "common_log.h"

struct ubx_config_s;
struct ubx_msg_byte_ctx_s;
/* typedef struct ubx_user_msg_s {
    uint8_t cls;
    uint8_t id;
    uint8_t *payload;
    size_t payload_len;
} ubx_user_msg_t;

typedef struct ubx_user_ctx_s {
    struct ubx_config_s *ubx;
    ubx_user_msg_t *tx;
    ubx_user_msg_t *rx;
    uint8_t read_from_byte;
    bool confirm_checksum;
} ubx_user_ctx_t;
 */
// prvate functions

static esp_err_t ubx_set_gnss(struct ubx_config_s *ubx, uint8_t mode);
static esp_err_t ubx_set_uart_out_rate(struct ubx_config_s *ubx);

static esp_err_t ubx_set_prot_msg_out(struct ubx_config_s *ubx, bool enable_nmea, bool enable_ubx);
static esp_err_t ubx_set_msgout(struct ubx_config_s *ubx);
static esp_err_t ubx_set_msgout_sat(struct ubx_config_s *ubx);

static esp_err_t ubx_get_hw_version(struct ubx_config_s *ubx);
static esp_err_t ubx_get_hw_id(struct ubx_config_s *ubx);
static esp_err_t ubx_get_gnss(struct ubx_config_s *ubx);
static esp_err_t ubx_get_nav_sat(struct ubx_config_s *ubx);

static esp_err_t ubx_initial_read(struct ubx_config_s *ubx, bool get_hw);

static esp_err_t ubx_cfg_send_m(struct ubx_config_s *ubx, uint8_t * msg, size_t msg_len, bool need_ack);
esp_err_t send_ubx_cfg_msg(struct ubx_config_s *ubx, uint8_t cls, uint8_t id, const uint8_t * payload, size_t payload_len, bool need_ack);
esp_err_t ubx_cfg_valset(struct ubx_config_s *ubx, const uint8_t * cfg, size_t payload_len, bool need_ack);
esp_err_t ubx_cfg_get(struct ubx_config_s *ubx, struct ubx_msg_byte_ctx_s * ubx_packet);
static esp_err_t ubx_uart_set_baud(struct ubx_config_s *ubx);
static esp_err_t ubx_set_uart_baud_rate(struct ubx_config_s *ubx, uint32_t baud);

esp_err_t write_ubx_msg(int uart_num, uint8_t *msg, size_t msg_len, bool need_checksum);

esp_err_t read_ubx_msg(struct ubx_config_s *ubx_dev, struct ubx_msg_byte_ctx_s * ubx_packet);

void print_ubx_msg(struct ubx_msg_byte_ctx_s * ubx_packet);
/**
 * @brief Gets the acknowledge of configuration message.
 * 
 * @param *ubx is the address of GPS configuration structure.
 * @param cls_id is the class to be acknowledged.
 * @param msg_id is the message to be acknowledged.
 * 
 * @return
 *     - ESP_OK                Success
 *     - ESP_FAIL              Parameter error
 *     - ESP_ERR_TIMEOUT       Not all bytes read
 *     - ESP_ERR_TIMEOUT       No ACK received within ACK_TIMEOUT
 *     - ESP_INVALID_RESPONSE  NAK received
 *     - ESP_ERR_INVALID_CRC   Checksum for the wrong message received
 */
esp_err_t ack_status(struct ubx_config_s *ubx, uint8_t cls_id, uint8_t msg_id);

/**
 * @brief Adds checksum to the message to be sent.
 * 
 * @details 8-Bit Fletcher Algorithm excluding UBX header.
 */
void add_checksum(uint8_t *message, uint16_t size, uint8_t *CK_A, uint8_t *CK_B);

/**
 * @brief Deinitializes the serial communication for the GPS.
 *
 * @param *ubx is the address of GPS configuration structure.
 * 
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
static esp_err_t ubx_uart_deinit(struct ubx_config_s *ubx);

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
static esp_err_t ubx_uart_init(struct ubx_config_s *ubx);

/**
 * @brief Initializes the GPS enable pins.
 *
 * @param *ubx is the address of the GPS configuration structure.
 * 
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
static esp_err_t ubx_pins_init(struct ubx_config_s *ubx);

/**
 * @brief Deinitializes the GPS enable pins.
 *
 * @param *ubx is the address of the GPS configuration structure.
 * 
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
static esp_err_t ubx_pins_deinit(struct ubx_config_s *ubx);

/**
 * @brief Initializes the GPS configuration structure.
 * 
 * @param *ubx is the address of the GPS configuration structure.
 * 
 * @return
 *    - ESP_OK   Success
 *   - ESP_FAIL Parameter error
*/
static esp_err_t ubx_config_init(struct ubx_config_s *ubx);

/**
 * @brief Deinitializes the GPS configuration structure.
 * 
 * @param *ubx is the address of the GPS configuration structure.
*/
static esp_err_t ubx_config_deinit(struct ubx_config_s *ubx);

#ifdef __cplusplus
}
#endif

inline esp_err_t encode_uint32(uint8_t *buf, uint32_t value) {
    uint32_t n = value;
    buf[0] = n & 0xFF;
    buf[1] = (n >> 8) & 0xFF;
    buf[2] = (n >> 16) & 0xFF;
    buf[3] = (n >> 24) & 0xFF;
    return ESP_OK;
}

inline esp_err_t encode_uint16(uint8_t *buf, uint16_t value) {
    uint16_t n = value;
    buf[0] = n & 0xFF;
    buf[1] = (n >> 8) & 0xFF;
    return ESP_OK;
}

inline void decode_uint16(const uint8_t* hex_string, uint16_t *output) {
    *output = (*(hex_string) + (*(hex_string+1) << 8));
}

#endif /* F77B6D3D_E33D_4ED3_B35C_5404E7A31138 */
