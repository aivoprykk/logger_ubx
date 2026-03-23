#ifndef F77B6D3D_E33D_4ED3_B35C_5404E7A31138
#define F77B6D3D_E33D_4ED3_B35C_5404E7A31138

#ifdef __cplusplus
extern "C" {
#endif


#include <string.h>
#include <stdint.h>
#include <esp_err.h>
#include <stdbool.h>

#include "sdkconfig.h"
#if (defined(CONFIG_LOGGER_USE_GLOBAL_LOG_LEVEL) && CONFIG_LOGGER_GLOBAL_LOG_LEVEL < CONFIG_UBLOX_LOG_LEVEL)
#define C_LOG_LEVEL CONFIG_LOGGER_GLOBAL_LOG_LEVEL
#else
#define C_LOG_LEVEL CONFIG_UBLOX_LOG_LEVEL
#endif
#include "common_log.h"
#include "unified_config.h"

#define CONFIG_UBX_TIMER_STATS_ENABLED 1

// #include "ubx.h"

/*!< UBX module lock functions for UART operations */
bool ubx_lock(int timeout_ms);
void ubx_unlock();

/*!< Timeout for the ubx message */
#define MSG_READ_TIMEOUT 3000

/*!< None Message IDs */
#define NONE_NONE 0x00

struct ubx_ctx_s;
struct ubx_msg_byte_ctx_s;
/* typedef struct ubx_user_msg_s {
    uint8_t cls;
    uint8_t id;
    uint8_t *payload;
    size_t payload_len;
} ubx_user_msg_t;

typedef struct ubx_user_ctx_s {
    struct ubx_ctx_s *ubx;
    ubx_user_msg_t *tx;
    ubx_user_msg_t *rx;
    uint8_t read_from_byte;
    bool confirm_checksum;
} ubx_user_ctx_t;
 */
// prvate functions

static esp_err_t ubx_set_gnss(struct ubx_ctx_s *ubx, uint8_t mode);
static esp_err_t ubx_set_uart_out_rate(struct ubx_ctx_s *ubx, uint8_t rate);

static esp_err_t ubx_set_prot_msg_out(struct ubx_ctx_s *ubx, bool enable_nmea, bool enable_ubx);
static esp_err_t ubx_set_msgout(struct ubx_ctx_s *ubx);
static esp_err_t ubx_set_msgout_sat(struct ubx_ctx_s *ubx);

static esp_err_t ubx_get_hw_version(struct ubx_ctx_s *ubx);
static esp_err_t ubx_get_hw_id(struct ubx_ctx_s *ubx);
static esp_err_t ubx_get_gnss(struct ubx_ctx_s *ubx);
static esp_err_t ubx_get_nav_sat(struct ubx_ctx_s *ubx);

static esp_err_t ubx_initial_read(struct ubx_ctx_s *ubx, bool get_hw);

static esp_err_t ubx_cfg_send_m(struct ubx_ctx_s *ubx, uint8_t * msg, size_t msg_len, bool need_ack);
esp_err_t send_ubx_cfg_msg(struct ubx_ctx_s *ubx, uint8_t cls, uint8_t id, const uint8_t * payload, size_t payload_len, bool need_ack);
esp_err_t ubx_cfg_valset(struct ubx_ctx_s *ubx, const uint8_t * cfg, size_t payload_len, bool need_ack);
esp_err_t ubx_cfg_valset_layers(struct ubx_ctx_s *ubx, const uint8_t *cfg,
                     size_t payload_len, uint8_t layers,
                     bool need_ack);
esp_err_t ubx_cfg_get(struct ubx_ctx_s *ubx, struct ubx_msg_byte_ctx_s * ubx_packet);
static esp_err_t ubx_uart_set_baud(struct ubx_ctx_s *ubx);
static esp_err_t ubx_set_uart_baud_rate(struct ubx_ctx_s *ubx, int baud);

esp_err_t write_ubx_msg(int uart_num, uint8_t *msg, size_t msg_len, bool need_checksum);

esp_err_t read_ubx_msg(struct ubx_ctx_s *ubx_dev, struct ubx_msg_byte_ctx_s * ubx_packet);

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
esp_err_t ack_status(struct ubx_ctx_s *ubx, uint8_t cls_id, uint8_t msg_id);

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
static esp_err_t ubx_uart_deinit(struct ubx_ctx_s *ubx);

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
static esp_err_t ubx_uart_init(struct ubx_ctx_s *ubx);

/**
 * @brief Initializes the GPS enable pins.
 *
 * @param *ubx is the address of the GPS configuration structure.
 * 
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
static esp_err_t ubx_pins_init(struct ubx_ctx_s *ubx);

/**
 * @brief Deinitializes the GPS enable pins.
 *
 * @param *ubx is the address of the GPS configuration structure.
 * 
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
static esp_err_t ubx_pins_deinit(struct ubx_ctx_s *ubx);

/**
 * @brief Initializes the GPS configuration structure.
 * 
 * @param *ubx is the address of the GPS configuration structure.
 * 
 * @return
 *    - ESP_OK   Success
 *   - ESP_FAIL Parameter error
*/
static esp_err_t ubx_ctx_init(struct ubx_ctx_s *ubx);

/**
 * @brief Deinitializes the GPS configuration structure.
 * 
 * @param *ubx is the address of the GPS configuration structure.
*/
static esp_err_t ubx_ctx_deinit(struct ubx_ctx_s *ubx);
static const char * ubx_chip_str(const struct ubx_ctx_s *ubx);

// Event-driven UART infrastructure
esp_err_t ubx_uart_event_init(struct ubx_ctx_s *ctx);
esp_err_t ubx_uart_event_deinit(struct ubx_ctx_s *ctx);
size_t ubx_rx_buf_read(struct ubx_ctx_s *ctx, uint8_t *dst, size_t len, uint32_t timeout_ms);

// Circular buffer helpers (for testing)
size_t ubx_rx_buf_available(struct ubx_ctx_s *ctx);
size_t ubx_rx_buf_free_space(struct ubx_ctx_s *ctx);
bool ubx_rx_has_complete_frame(struct ubx_ctx_s *ctx);

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

/*!< Test runner function for UBX circular buffer tests */
void run_ubx_circular_buffer_tests(void);

#endif /* F77B6D3D_E33D_4ED3_B35C_5404E7A31138 */
