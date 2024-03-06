/**
 * @file ublox.c
 *
 * @brief This is the source code to the uBlox GPS library for ESP32.
 *
 * @see https://github.com/AngeloElias/esp32-ublox
 *
 */

#include "ublox_private.h"

#include <string.h>
#include <esp_log.h>
#include <driver/gpio.h>

static const char *TAG = "ublox";
SemaphoreHandle_t xMutex;

ubx_config_t *ubx_config_new(uart_port_t uart_num, ubx_baud_rate_t baud, gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t en_pin0, gpio_num_t en_pin1, gpio_num_t en_pin2) {
    ubx_config_t *ubx = (ubx_config_t *)malloc(sizeof(ubx_config_t));
    esp_err_t ret = ESP_OK;
    ret = ubx_config_init(ubx, uart_num, baud, tx_pin, rx_pin, en_pin0, en_pin1, en_pin2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_config_init failed", __FUNCTION__);
        free(ubx);
        return NULL;
    }
    return ubx;
}

esp_err_t ubx_config_delete(ubx_config_t *ubx) {
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ESP_OK;
    ret = ubx_config_deinit(ubx);
    free(ubx);
    return ret;
}

esp_err_t ubx_config_init(ubx_config_t *ubx, uart_port_t uart_num, ubx_baud_rate_t baud, gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t en_pin0, gpio_num_t en_pin1, gpio_num_t en_pin2) {
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ESP_OK;
    ubx->uart_num = uart_num;
    ubx->tx_pin = tx_pin;
    ubx->rx_pin = rx_pin;
    memset(ubx->en_pins, 0, sizeof(ubx->en_pins));
    ubx->en_pins[0] = en_pin0;
    if (en_pin1 > 0)
        ubx->en_pins[1] = en_pin1;
    if (en_pin2 > 0)
        ubx->en_pins[2] = en_pin2;
    ubx->baud = baud;
    if (xMutex == NULL)
        xMutex = xSemaphoreCreateMutex();
    return ret;
}

esp_err_t ubx_config_deinit(ubx_config_t *ubx) {
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    if (xMutex != NULL)
        vSemaphoreDelete(xMutex);
    return ESP_OK;
}

esp_err_t ubx_pins_init(ubx_config_t *ubx) {
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ESP_OK;
    uint8_t i = 0;
    while (i < UBX_EN_PIN_LEN) {
        if (ubx->en_pins[i] <= 0)
            continue;
        ret = gpio_set_direction(ubx->en_pins[i], GPIO_MODE_OUTPUT);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] gpio_set_direction failed", __FUNCTION__);
            return ret;
        }
        ret = gpio_set_level(ubx->en_pins[i], true);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] gpio_set_level failed", __FUNCTION__);
            return ret;
        }
        ret = gpio_set_drive_capability(ubx->en_pins[i], GPIO_DRIVE_CAP_3);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] gpio_set_drive_capability failed", __FUNCTION__);
            return ret;
        }
        i++;
    }
    return ret;
}

esp_err_t ubx_pins_deinit(ubx_config_t *ubx) {
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ESP_OK;
    uint8_t i = 0;
    if (ubx->uart_setup_ok)
        ubx_uart_deinit(ubx);
    while (i < UBX_EN_PIN_LEN) {
        if (ubx->en_pins[i] <= 0)
            continue;
        ret = gpio_set_level(ubx->en_pins[i], false);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] gpio_set_level failed", __FUNCTION__);
            return ret;
        }
        i++;
    }
    return ret;
}

esp_err_t ubx_uart_init(ubx_config_t *ubx) {
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ESP_OK;

    ret = ubx_pins_init(ubx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_pins_init failed", __FUNCTION__);
        return ret;
    }

    uint32_t baud_rate;
    switch (ubx->baud) {
        case UBX_BAUD_4800:
            baud_rate = 4800;
            break;
        case UBX_BAUD_9600:
            baud_rate = 9600;
            break;
        case UBX_BAUD_19200:
            baud_rate = 19200;
            break;
        case UBX_BAUD_57600:
            baud_rate = 57600;
            break;
        case UBX_BAUD_115200:
            baud_rate = 115200;
            break;
        case UBX_BAUD_38400:
        default:
            baud_rate = 38400;
            break;
    }

    ubx->uart_conf.baud_rate = baud_rate;
    ubx->uart_conf.data_bits = UART_DATA_8_BITS;
    ubx->uart_conf.parity = UART_PARITY_DISABLE;
    ubx->uart_conf.stop_bits = UART_STOP_BITS_1;
    ubx->uart_conf.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    ubx->uart_conf.source_clk = UART_SCLK_DEFAULT;

    ret = uart_param_config(ubx->uart_num, &(ubx->uart_conf));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] uart_param_config failed", __FUNCTION__);
        return ret;
    }
    ret = uart_set_pin(ubx->uart_num, ubx->tx_pin, ubx->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret == ESP_OK) {
        ret = uart_set_sw_flow_ctrl(ubx->uart_num, false, 0, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] uart_set_sw_flow_ctrl failed", __FUNCTION__);
            return ret;
        }
    } else {
        ESP_LOGW(TAG, "[%s] uart_set_pin failed", __FUNCTION__);
        // return ret;
    }
    int intr_alloc_flags = 0;
#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif
    ret = uart_driver_install(ubx->uart_num, 1024, 0, 0, NULL, intr_alloc_flags);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] uart_driver_install failed", __FUNCTION__);
        return ret;
    }
    ubx->uart_setup_ok = true;
    return ret;
}

esp_err_t ubx_uart_deinit(ubx_config_t *ubx) {
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ESP_OK;
    if (!ubx->uart_setup_ok)
        return ret;
    xSemaphoreTake(xMutex, portMAX_DELAY);
    {
        ret = uart_driver_delete(ubx->uart_num);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] uart_driver_delete failed", __FUNCTION__);
        }
    }
    xSemaphoreGive(xMutex);

    ret = ubx_pins_deinit(ubx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_pins_deinit failed", __FUNCTION__);
    }
    if (!ret)
        ubx->uart_setup_ok = 0;
    return ret;
}

esp_err_t ubx_on(ubx_config_t *ubx) {
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ESP_OK;
    ret = ubx_uart_init(ubx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_uart_init failed", __FUNCTION__);
    }
    return ret;
}

esp_err_t ubx_off(ubx_config_t *ubx) {
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ESP_OK;
    ret = ubx_uart_deinit(ubx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_uart_deinit failed", __FUNCTION__);
    }
    return ret;
}

esp_err_t ubx_set_nav_mode(ubx_config_t *ubx, ubx_nav_mode_t mode) {
    uint8_t nav_msg[44] = {UBX_HDR_A, UBX_HDR_B, CLS_CFG, CFG_NAV5,
                           /* payload size */ 0x24, 0x00,
                           /* mask */ 0xFF, 0xFF,
                           mode, /* auto 2D-3D */ 0x03,
                           /* fixedAlt */ 0x00, 0x00, 0x00, 0x00,
                           /* fixedAltVar */ 0x10, 0x27, 0x00, 0x00,
                           /* minElev */ 0x05, /* drLimit */ 0x00,
                           /* pDop */ 0xFA, 0x00,
                           /* tDop */ 0xFA, 0x00,
                           /* pAcc */ 0x64, 0x00,
                           /* tAcc */ 0x2C, 0x01,
                           /* staticHoldThresh */ 0x00,
                           /* dgpsTimeOut */ 0x00,
                           /* cnoThreshNumSVs */ 0x00,
                           /* cnoThresh */ 0x00,
                           /* reserved */ 0x00, 0x00, 0x00, 0x00, 0x00,
                           /* reserved */ 0x00, 0x00, 0x00, 0x00, 0x00,
                           /* chksum */ 0x00, 0x00};

    add_checksum(nav_msg, sizeof(nav_msg), &nav_msg[sizeof(nav_msg) - 2], &nav_msg[sizeof(nav_msg) - 1]);

    if (uart_write_bytes(ubx->uart_num, (char *)nav_msg, sizeof(nav_msg)) != sizeof(nav_msg))
        return ESP_FAIL;

    return ack_status(ubx, CLS_CFG, CFG_NAV5);
}

esp_err_t ubx_set_output_rate(ubx_config_t *ubx, ubx_output_rate_t rate) {
    uint8_t output_vec[2];
    uint8_t cfg = 0;
    uint8_t msg_sz = 0;
    uint8_t out_msg[24];
    switch (ubx->hw_type) {
        case UBX_TYPE_M8:
            cfg = CFG_RATE;
            switch (rate) {
                case UBX_OUTPUT_1HZ:
                    output_vec[0] = 0xE8;
                    output_vec[1] = 0x03;
                    break;
                case UBX_OUTPUT_2HZ:
                    output_vec[0] = 0xF4;
                    output_vec[1] = 0x01;
                    break;
                case UBX_OUTPUT_5HZ:
                    output_vec[0] = 0xC8;
                    output_vec[1] = 0x00;
                    break;
                case UBX_OUTPUT_10HZ:
                    output_vec[0] = 0xC8;
                    output_vec[1] = 0x00;
                    break;
                default:
                    return ESP_ERR_INVALID_ARG;
            }
            {const uint8_t msg[] = {UBX_HDR_A, UBX_HDR_B, CLS_CFG, cfg,
                                  /* payload size */ 0x06, 0x00,
                                  /* measRate 2b */ output_vec[0], output_vec[1],
                                  /* navRate always 1 */ 0x01, 0x00,
                                  /* timeRef UTC */ 0x01, 0x00,
                                  /* checksum */ 0x00, 0x00};
            msg_sz = sizeof(msg);

            memcpy(out_msg, msg, msg_sz);}
            break;
        case UBX_TYPE_M9:
        case UBX_TYPE_M10:
            cfg = CFG_VALSET;
            switch (rate) {
                case UBX_OUTPUT_1HZ:
                    output_vec[0] = 0xE8;
                    output_vec[1] = 0x03;
                    break;
                case UBX_OUTPUT_2HZ:
                    output_vec[0] = 0xF4;
                    output_vec[1] = 0x01;
                    break;
                case UBX_OUTPUT_5HZ:
                    output_vec[0] = 0xC8;
                    output_vec[1] = 0x00;
                    break;
                case UBX_OUTPUT_10HZ:
                    output_vec[0] = 0x64;
                    output_vec[1] = 0x00;
                    break;
                case UBX_OUTPUT_15HZ:
                    output_vec[0] = 0x43;
                    output_vec[1] = 0x00;
                    break;
                case UBX_OUTPUT_20HZ:
                    output_vec[0] = 0x32;
                    output_vec[1] = 0x00;
                    break;
                default:
                    return ESP_ERR_INVALID_ARG;
            }
            {const uint8_t msg[] = {UBX_HDR_A, UBX_HDR_B, CLS_CFG, cfg,
                                  /* payload size */ 0x0A, 0x00,
                                  /* navRate always 1 */ 0x01, 0x00,
                                  /* timeRef UTC */ 0x01, 0x00,
                                  0x00, 0x00,
                                  0x01, 0x00,
                                  0x21, 0x30,
                                  /* measRate 2b */ output_vec[0], output_vec[1],
                                  /* checksum */ 0x00, 0x00};
            msg_sz = sizeof(msg);
            memcpy(out_msg, msg, msg_sz);}
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    add_checksum(out_msg, msg_sz, &out_msg[msg_sz - 2], &out_msg[msg_sz - 1]);
    if (uart_write_bytes(ubx->uart_num, (char *)out_msg, msg_sz) != msg_sz)
        return ESP_FAIL;
    return ack_status(ubx, CLS_CFG, cfg);
}

esp_err_t ubx_set_ubxout(ubx_config_t *ubx) {
    uint8_t msg_sz = 0;
    uint8_t out_msg[32];
    switch (ubx->hw_type) {
        case UBX_TYPE_M8:
            {const uint8_t msg[] = {UBX_HDR_A, UBX_HDR_B, CLS_CFG, CFG_PRT,
                                  /* payload size */ 0x14, 0x00,
                                  /* portID */ 0x01, 0x00,
                                  /* reserved1 */ 0x00, 0x00,
                                  /* txReady 0x00, mode 0x08, invert 0x00, reserved2 0x00 */ 0xd0, 0x08, 0x00, 0x00,
                                  /* baudRate */ 0x80, 0x25, 0x00, 0x00,
                                  /* inProtoMask */ 0x23, 0x00,
                                  /* outProtoMask */ 0x01, 0x00,
                                  /* flags */ 0x00, 0x00,
                                  /* reserved3 */ 0x00, 0x00,
                                  /* checksum */ 0x00, 0x00};
            msg_sz = sizeof(msg);
            memcpy(out_msg, msg, msg_sz);}
            break;
        case UBX_TYPE_M9:
        case UBX_TYPE_M10:
            {const uint8_t msg[] = {UBX_HDR_A, UBX_HDR_B, CLS_CFG, CFG_VALSET,
                                  /* payload size */ 0x09, 0x00,
                                  /* version */ 0x01, 0x01,
                                  /* layers */ 0x00, 0x00,
                                  /* cfg_id[4], cfg value */ 0x01, 0x00, 0x74, 0x10, 0x01,
                                  /* checksum */ 0x00, 0x00};
            msg_sz = sizeof(msg);
            memcpy(out_msg, msg, msg_sz);}
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    add_checksum(out_msg, msg_sz, &out_msg[msg_sz - 2], &out_msg[msg_sz - 1]);
    if (uart_write_bytes(ubx->uart_num, (char *)out_msg, msg_sz) != msg_sz)
        return ESP_FAIL;
    return ack_status(ubx, CLS_CFG, CFG_PRT);
}

esp_err_t ubx_set_message_rate(ubx_config_t *ubx, ubx_message_t message, bool active) {
    uint8_t msg_vec[11] = {UBX_HDR_A, UBX_HDR_B, CLS_CFG, CFG_MSG,
                           /* payload size */ 0x03, 0x00,
                           /* msgClass */ NMEA_STD,
                           /* msgId */ message,
                           /* rate */ active,
                           /* checksum */ 0x00, 0x00};
    add_checksum(msg_vec, sizeof(msg_vec), &msg_vec[sizeof(msg_vec) - 2], &msg_vec[sizeof(msg_vec) - 1]);
    if (uart_write_bytes(ubx->uart_num, (char *)msg_vec, sizeof(msg_vec)) != sizeof(msg_vec))
        return ESP_FAIL;
    return ack_status(ubx, CLS_CFG, CFG_MSG);
}

esp_err_t ubx_get_hw_version(ubx_config_t *ubx, ubx_message_t message, bool active) {
    uint8_t msg_vec[8] = {UBX_HDR_A, UBX_HDR_B, CLS_MON, MON_VER,
                           /* payload size */ 0x00, 0x00,
                           /* checksum */ 0x00, 0x00};
    add_checksum(msg_vec, sizeof(msg_vec), &msg_vec[sizeof(msg_vec) - 2], &msg_vec[sizeof(msg_vec) - 1]);
    if (uart_write_bytes(ubx->uart_num, (char *)msg_vec, sizeof(msg_vec)) != sizeof(msg_vec))
        return ESP_FAIL;
    return ack_status(ubx, CLS_CFG, CFG_MSG);
}

// private functions

esp_err_t ack_status(ubx_config_t *ubx, uint8_t cls_id, uint8_t msg_id) {
    esp_err_t ret;
    uint8_t data = 0;
    uint8_t i = 0;
    size_t size = 0;
    uint8_t ack_pkt[10] = {UBX_HDR_A, UBX_HDR_B, CLS_ACK, ACK_ACK,
                           /* payload size */ 0x02, 0x00,
                           /* msgClass */ cls_id,
                           /* msgId */ msg_id,
                           /* checksum */ 0x00, 0x00};

    uint32_t then = millis();
    while (i < 10 && millis() - then < ACK_TIMEOUT) {
        ret = uart_get_buffered_data_len(ubx->uart_num, &size);
        if (ret)
            return ret;
        if (size) {
            if (!uart_read_bytes(ubx->uart_num, &data, 1, 20 / portTICK_PERIOD_MS))
                return ESP_ERR_TIMEOUT;

            if (data == ack_pkt[i])
                i++;
            else if (i > 2) {
                ack_pkt[i] = data;
                i++;
            }
        }
    }
    if (millis() - then >= 1500)
        return ESP_ERR_TIMEOUT;
    if (!ack_pkt[3])
        return ESP_ERR_INVALID_RESPONSE;

    uint8_t CK_A = 0;
    uint8_t CK_B = 0;
    add_checksum(ack_pkt, 10, &CK_A, &CK_B);

    if (cls_id == ack_pkt[6] && msg_id == ack_pkt[7] && CK_A == ack_pkt[8] && CK_B == ack_pkt[9])
        return ESP_OK;
    else
        return ESP_ERR_INVALID_CRC;
}

void add_checksum(uint8_t *message, uint8_t size, uint8_t *CK_A, uint8_t *CK_B) {
    for (uint8_t i = 2; i < size - 2; i++) {
        *CK_A = *CK_A + message[i];
        *CK_B = *CK_B + *CK_A;
    }
}

inline uint32_t millis() {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}
