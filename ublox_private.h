#ifndef F77B6D3D_E33D_4ED3_B35C_5404E7A31138
#define F77B6D3D_E33D_4ED3_B35C_5404E7A31138

#include <stdint.h>
#include <esp_err.h>
#include "ublox.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!< Timeout for the ACK message */
#define ACK_TIMEOUT 1500

/*!< UBX Protocol headers */
#define UBX_HDR_A 0xB5
#define UBX_HDR_B 0x62

/*!< UBX Message classes */
#define CLS_NAV  0x01
#define CLS_INF  0x04
#define CLS_ACK  0x05
#define CLS_CFG  0x06
#define CLS_LOG  0x21
#define CLS_SEC  0x27
#define CLS_MON  0x0A

/*!< NMEA Configuration class */
#define NMEA_STD 0xF0

/*!< ACK Message IDs */
#define ACK_ACK 0x01
#define ACK_NAK 0x00

/*!< NAV Message IDs */
#define NAV_DUMMY 0x00
#define NAV_DOP 0x04
#define NAV_PVT 0x07
#define NAV_PVT_LEN 92
#define NAV_CLK 0x22
#define NAV_CLK_LEN 20
#define NAV_SAT 0x35
#define NAC_SAT_LEN 8

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

/*!< MON Message IDs */
#define MON_GNSS 0x28
#define MON_VER  0x04
#define MON_HW   0x09

/*!< SEC Message IDs */
#define SEC_UBX  0x03

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
 * @brief Milliseconds passed since task started running.
 * 
 * @return Milliseconds passed since task started running.
 */
uint32_t millis();

/**
 * @brief Adds checksum to the message to be sent.
 * 
 * @details 8-Bit Fletcher Algorithm excluding UBX header.
 */
void add_checksum(uint8_t *message, uint8_t size, uint8_t *CK_A, uint8_t *CK_B);

#ifdef __cplusplus
}
#endif

#endif /* F77B6D3D_E33D_4ED3_B35C_5404E7A31138 */
