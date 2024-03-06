#ifndef E8FD891D_3201_47A6_94CA_E3060D52E3AD
#define E8FD891D_3201_47A6_94CA_E3060D52E3AD

/*
 * @brief This is a library for the uBlox GPS
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

typedef enum _ubx_hw_t {
    UBX_TYPE_M0 = 0,
    UBX_TYPE_UNKNOWN = 0,
    UBX_TYPE_M7 = 7,
    UBX_TYPE_M8 = 8,
    UBX_TYPE_M9 = 9,
    UBX_TYPE_M10 = 10
} ubx_hw_t;

/**
 * @brief Navigation mode enum.
 *
 * @details
 *     - Portable (automatic)
 *     - Stationary (time-only)
 *     - Pedestrian/Automotive/Sea
 *     - Airborne (1G, 2G and 4G max modes)
 */
typedef enum _ubx_nav_mode {
    UBX_MODE_PORTABLE,
    UBX_MODE_STATIONARY = 2,
    UBX_MODE_PEDESTRIAN,
    UBX_MODE_AUTOMOTIVE,
    UBX_MODE_SEA,
    UBX_MODE_AIR_1G_MAX,
    UBX_MODE_AIR_2G_MAX,
    UBX_MODE_AIR_4G_MAX
} ubx_nav_mode_t;

typedef enum _ubx_baud_rate_t {
    UBX_BAUD_4800,
    UBX_BAUD_9600,
    UBX_BAUD_19200,
    UBX_BAUD_38400,
    UBX_BAUD_57600,
    UBX_BAUD_115200
} ubx_baud_rate_t;

/**
 * @brief Message output rate enum.
 */
typedef enum _ubx_output_rate_t {
    UBX_OUTPUT_1HZ,
    UBX_OUTPUT_2HZ,
    UBX_OUTPUT_5HZ,
    UBX_OUTPUT_10HZ,
    UBX_OUTPUT_15HZ,
    UBX_OUTPUT_20HZ
} ubx_output_rate_t;

/**
 * @brief NMEA Messages enum.
 */
typedef enum _ubx_message_t {
    UBX_MSG_DTM = 0x0A,
    UBX_MSG_GBS = 0x09,
    UBX_MSG_GGA = 0x00,
    UBX_MSG_GLL = 0x01,
    UBX_MSG_GRS = 0x06,
    UBX_MSG_GSA = 0x02,
    UBX_MSG_GST = 0x07,
    UBX_MSG_GSV = 0x03,
    UBX_MSG_RMC = 0x04,
    UBX_MSG_VTG = 0x05,
    UBX_MSG_ZDA = 0x08
} ubx_message_t;

enum _ubx_msg_type_t {
    MT_NONE,
    MT_NAV_DUMMY,
    MT_NAV_PVT,
    MT_NAV_ACK,
    MT_NAV_NACK,
    MT_NAV_ID,
    MT_MON_GNSS,
    MT_NAV_DOP,
    MT_MON_VER,
    MT_NAV_SAT
} ubx_msg_type_t;

#define UBX_EN_PIN_LEN 4

/**
 * @brief GPS configuration structure.
 *
 * @details Please do NOT change the structure directly.
 */
typedef struct _ubx_config_t {
    uart_port_t uart_num;
    uart_port_t tx_pin;
    uart_port_t rx_pin;
    uart_port_t en_pins[4];
    ubx_hw_t hw_type;
    ubx_baud_rate_t baud;
    uart_config_t uart_conf;
    bool uart_setup_ok;
} ubx_config_t;

/**
 * @brief Initializes the GPS configuration structure.
 *
 * @param uart_num is the ESP32 UART port number.
 * @param baud is one of the possible GPS baud rates.
 * @param tx_pin is the ESP32 TX Pin (Connect to GPS RX).
 * @param rx_pin is the ESP32 RX Pin (Connect to GPS TX).
 * @param en_pin0 is the GPS enable pin. Manual set to low to power off.
 * @param en_pin1 is the GPS enable pin. Manual set to low to power off.
 * @param en_pin2 is the GPS enable pin. Manual set to low to power off.
 *
 * @return
 * ubx_config_t*   Success
 * NULL            Parameter error
 */
ubx_config_t * ubx_config_new(uart_port_t uart_num, ubx_baud_rate_t baud, gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t en_pin0, gpio_num_t en_pin1, gpio_num_t en_pin2);

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
 * @param uart_num is the ESP32 UART port number.
 * @param baud is one of the possible GPS baud rates.
 * @param tx_pin is the ESP32 TX Pin (Connect to GPS RX).
 * @param rx_pin is the ESP32 RX Pin (Connect to GPS TX).
 * @param en_pin0 is the GPS enable pin. Manual set to low to power off.
 * @param en_pin1 is the GPS enable pin. Manual set to low to power off.
 * 
 * @return
 *    - ESP_OK   Success
 *   - ESP_FAIL Parameter error
*/
esp_err_t ubx_config_init(ubx_config_t*ubx, uart_port_t uart_num, ubx_baud_rate_t baud, gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t en_pin0, gpio_num_t en_pin1, gpio_num_t en_pin2);

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
 * @param mode is the uBlox navigation mode.
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
esp_err_t ubx_set_nav_mode(ubx_config_t *ubx, ubx_nav_mode_t mode);

/**
 * @brief Sets the GPS message output rate.
 *
 * @param *ubx is the address of the GPS configuration structure.
 * @param rate is the uBlox output message frequency.
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
esp_err_t ubx_set_output_rate(ubx_config_t *ubx, ubx_output_rate_t rate);

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

#ifdef __cplusplus
}
#endif

#endif /* E8FD891D_3201_47A6_94CA_E3060D52E3AD */
