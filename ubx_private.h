#ifndef F77B6D3D_E33D_4ED3_B35C_5404E7A31138
#define F77B6D3D_E33D_4ED3_B35C_5404E7A31138

#include <stdint.h>
#include <esp_err.h>
#include "ubx.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!< Timeout for the ACK message */
#define ACK_TIMEOUT 1500

/*!< UBX Protocol headers */
#define UBX_HDR_A 0xB5
#define UBX_HDR_B 0x62
#define UBX_HDR {UBX_HDR_A, UBX_HDR_B}

/*!< UBX Message classes */
#define CLS_NAV  0x01
#define CLS_INF  0x04
#define CLS_ACK  0x05
#define CLS_CFG  0x06
#define CLS_LOG  0x21
#define CLS_SEC  0x27
#define CLS_MON  0x0A

/*!< ACK Message IDs */
#define ACK_ACK 0x01
#define ACK_NAK 0x00

/*!< NAV Message IDs */
#define NAV_DUMMY 0x00
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

/* typedef struct ubx_user_msg_s {
    uint8_t cls;
    uint8_t id;
    uint8_t *payload;
    size_t payload_len;
} ubx_user_msg_t;

typedef struct ubx_user_ctx_s {
    ubx_config_t *ubx;
    ubx_user_msg_t *tx;
    ubx_user_msg_t *rx;
    uint8_t read_from_byte;
    bool confirm_checksum;
} ubx_user_ctx_t;
 */
// prvate functions

esp_err_t ubx_set_gnss(ubx_config_t *ubx, uint8_t mode);
esp_err_t ubx_set_uart_out_rate(ubx_config_t *ubx);

esp_err_t ubx_set_prot_msg_out(ubx_config_t *ubx, bool enable_nmea, bool enable_ubx);
esp_err_t ubx_set_msgout(ubx_config_t *ubx);
esp_err_t ubx_set_msgout_sat(ubx_config_t *ubx);

esp_err_t ubx_get_hw_version(ubx_config_t *ubx);
esp_err_t ubx_get_hw_id(ubx_config_t *ubx);
esp_err_t ubx_get_gnss(ubx_config_t *ubx);
esp_err_t ubx_get_nav_sat(ubx_config_t *ubx);

esp_err_t ubx_initial_read(ubx_config_t *ubx, bool get_hw);

void payload_size(size_t value, uint8_t* buffer);
uint8_t hex_char_to_uint8_t(char c);
void hex_string_to_uint8_t(const char* hex_string, uint8_t* output, size_t output_size);

esp_err_t ubx_cfg_send_m(ubx_config_t *ubx, uint8_t * msg, size_t msg_len, bool need_ack);
esp_err_t send_ubx_cfg_msg(ubx_config_t *ubx, uint8_t cls, uint8_t id, uint8_t * payload, size_t payload_len, bool need_ack);
esp_err_t ubx_cfg_valset(ubx_config_t *ubx, uint8_t * cfg, size_t payload_len);
esp_err_t ubx_cfg_get(ubx_config_t *ubx, uint8_t cls, uint8_t id, uint8_t *msg, size_t msg_len);
esp_err_t ubx_uart_set_baud(ubx_config_t *ubx);
esp_err_t ubx_set_uart_baud_rate(ubx_config_t *ubx, uint32_t baud);

esp_err_t write_ubx_msg(ubx_config_t *ubx, uint8_t *msg, size_t msg_len, bool need_checksum);
esp_err_t read_ubx_msg(ubx_config_t *ubx, uint8_t *msg, size_t msg_len, bool verify_ubx_header);

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
esp_err_t ack_status(ubx_config_t *ubx, uint8_t cls_id, uint8_t msg_id);

/**
 * @brief Adds checksum to the message to be sent.
 * 
 * @details 8-Bit Fletcher Algorithm excluding UBX header.
 */
void add_checksum(uint8_t *message, uint16_t size, uint8_t *CK_A, uint8_t *CK_B);

#ifdef __cplusplus
}
#endif

#endif /* F77B6D3D_E33D_4ED3_B35C_5404E7A31138 */
