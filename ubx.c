/**
 * @file ublox.c
 *
 * @brief This is the source code to the uBlox GPS library for ESP32.
 *
 * @see https://github.com/AngeloElias/esp32-ublox
 *
 */

#include "ubx_private.h"

#include <string.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <sys/time.h>
#include <logger_common.h>
#include "ubx_events.h"

ESP_EVENT_DEFINE_BASE(UBX_EVENTS);

static const char *TAG = "ublox";
SemaphoreHandle_t xMutex;
RTC_DATA_ATTR ubx_rtc_config_t rtc_config = UBX_RTC_DEFAULT_CONFIG();
const uint32_t ubx_baud_rates[] = UBX_BAUD_RATES;
const uint8_t ubx_hw_types[] = UBX_HW_TYPES;

ubx_config_t *ubx_config_new() {
    LOGR
    ubx_config_t *ubx = (ubx_config_t *)calloc(1, sizeof(ubx_config_t));
    esp_err_t ret = ESP_OK;
    ret = ubx_config_init(ubx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_config_init failed", __FUNCTION__);
        free(ubx);
        return NULL;
    }
    return ubx;
}

esp_err_t ubx_config_delete(ubx_config_t *ubx) {
    LOGR
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ESP_OK;
    ret = ubx_config_deinit(ubx);
    free(ubx);
    return ret;
}

esp_err_t ubx_config_init(ubx_config_t *ubx) {
    LOGR
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    if(ubx->config_ok)
        return ESP_OK;
    esp_err_t ret = ESP_OK;
    ubx_config_t cfg = UBX_DEFAULT_CONFIG();
    memcpy(ubx, &cfg, sizeof(ubx_config_t));
    ubx->rtc_conf = &rtc_config;
    ubx->uart_conf.baud_rate = ubx->rtc_conf->baud;
    if (xMutex == NULL)
        xMutex = xSemaphoreCreateMutex();
    ubx->config_ok = true;
    return ret;
}

esp_err_t ubx_config_deinit(ubx_config_t *ubx) {
    LOGR
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    if (xMutex != NULL)
        vSemaphoreDelete(xMutex);
    ubx->config_ok = false;
    return ESP_OK;
}

esp_err_t ubx_pins_init(ubx_config_t *ubx) {
    TIMER_S
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ESP_OK;
    uint8_t i = 0;
    while (i < UBX_EN_PIN_LEN) {
        if (ubx->en_pins[i] <= 0){
            goto next;
        }
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
        next:
        ++i;
    }
    TIMER_E
    return ret;
}

esp_err_t ubx_pins_deinit(ubx_config_t *ubx) {
    LOGR
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ESP_OK;
    uint8_t i = 0;
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
    TIMER_S
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    if (ubx->config_ok != true) {
        ESP_LOGE(TAG, "you must call ubx_config_init(cfg) first!!!!");
        assert(0);
    }
    esp_err_t ret = ESP_OK;

    ret = ubx_pins_init(ubx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_pins_init failed", __FUNCTION__);
        return ret;
    }

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
    ESP_LOGW(TAG, "[%s] uart_driver_install done", __FUNCTION__);
    ESP_ERROR_CHECK(esp_event_post(UBX_EVENTS, UBX_EVENT_UART_INIT_DONE, NULL,0, portMAX_DELAY));
    ubx->uart_setup_ok = true;
    TIMER_E
    delay_ms(50);
    return ret;
}

esp_err_t ubx_uart_deinit(ubx_config_t *ubx) {
    TIMER_S
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ESP_OK;
    if (!ubx->uart_setup_ok)
        return ret;
    xSemaphoreTake(xMutex, portMAX_DELAY);
    ret = uart_driver_delete(ubx->uart_num);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] uart_driver_delete failed", __FUNCTION__);
    }
    xSemaphoreGive(xMutex);

    ret = ubx_pins_deinit(ubx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_pins_deinit failed", __FUNCTION__);
    }
    if (!ret) {
        ESP_ERROR_CHECK(esp_event_post(UBX_EVENTS, UBX_EVENT_UART_DEINIT_DONE, NULL,0, portMAX_DELAY));
        ubx->uart_setup_ok = false;
    }
    TIMER_E
    return ret;
}

esp_err_t ubx_on(ubx_config_t *ubx) {
    LOGR
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ESP_OK;
    ret = ubx_uart_init(ubx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_uart_init failed", __FUNCTION__);
    }
    ESP_LOGW(TAG, "[%s] ubx_uart_init done", __FUNCTION__);
    return ret;
}

esp_err_t ubx_off(ubx_config_t *ubx) {
    LOGR
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ESP_OK;
    ret = ubx_uart_deinit(ubx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_uart_deinit failed", __FUNCTION__);
    }
    ubx->setup_ok = false;
    return ret;
}

esp_err_t ubx_setup(ubx_config_t *ubx) {
    TIMER_S
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ESP_OK;
    ret = ubx_on(ubx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_on failed", __FUNCTION__);
        return ret;
    }
    
    //delay_ms(100);
    // initial read
    ret = ubx_initial_read(ubx, false);
    // if (ret != ESP_OK) {
    //     ESP_LOGW(TAG, "[%s] ubx_initial_read failed.", __FUNCTION__);
    //     //return ret;
    // }

    // find hw model
    //if(ret<=0) {
        ret = ubx_get_hw_version(ubx);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] ubx_get_hw_version failed", __FUNCTION__);
            return ret;
        }
    //}

    // set ubx message protocol
    ret = ubx_set_prot_msg_out(ubx, false, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_prot_msg_out failed", __FUNCTION__);
        return ret;
    }
    
    if(ubx->rtc_conf->hw_type == UBX_TYPE_M8){
        ret = ubx_set_nav_mode(ubx, ubx->rtc_conf->nav_mode);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] ubx_set_nav_mode failed", __FUNCTION__);
        }
    }

    ret = ubx_set_msgout(ubx);
    if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] ubx_set_msgout failed", __FUNCTION__);
        }
    if(ubx->rtc_conf->msgout_sat){
       ret = ubx_set_msgout_sat(ubx);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] ubx_set_msgout_sat failed", __FUNCTION__);
        }
    }

    ret = ubx_set_gnss(ubx, ubx->rtc_conf->gnss);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_set_gnss failed", __FUNCTION__);
        return ret;
    }

    ret = ubx_set_uart_out_rate(ubx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_set_uart_out_rate failed", __FUNCTION__);
        return ret;
    }
    
    ret = ubx_get_hw_id(ubx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_get_hw_id failed", __FUNCTION__);
        return ret;
    }

    ret = ubx_get_gnss(ubx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_get_gnss failed", __FUNCTION__);
        return ret;
    }

    if(!ret){
        ESP_ERROR_CHECK(esp_event_post(UBX_EVENTS, UBX_EVENT_SETUP_DONE, NULL,0, portMAX_DELAY));
        ubx->setup_ok = true;
    }
    TIMER_E
    return ret;

}

void payload_size(size_t value, uint8_t* buffer) {
  if(value > 0xff) {
    buffer[1] = (value >> 8);
  }
  buffer[0] = (value & 0xff);
}

esp_err_t encode_uint32(uint8_t *buf, uint32_t value) {
    uint32_t n = value;
    buf[0] = n & 0xFF;
    buf[1] = (n >> 8) & 0xFF;
    buf[2] = (n >> 16) & 0xFF;
    buf[3] = (n >> 24) & 0xFF;
    return ESP_OK;
}

esp_err_t encode_uint16(uint8_t *buf, uint16_t value) {
    uint16_t n = value;
    buf[0] = n & 0xFF;
    buf[1] = (n >> 8) & 0xFF;
    return ESP_OK;
}

uint8_t hex_char_to_uint8_t(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    } else if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    } else if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    } else {
        return 0; // Invalid character
    }
}

void hex_string_to_uint8_t(const char* hex_string, uint8_t* output, size_t output_size) {
    for (size_t i = 0; i < output_size; ++i) {
        char c1 = hex_string[i * 2];
        char c2 = hex_string[i * 2 + 1];
        output[i] = (hex_char_to_uint8_t(c1) << 4) + hex_char_to_uint8_t(c2);
    }
}

esp_err_t ubx_cfg_send_m(ubx_config_t *ubx, uint8_t * msg, size_t msg_len, bool need_ack) {
    esp_err_t ret = ESP_OK;
    //xSemaphoreTake(xMutex, portMAX_DELAY);
    ret = write_ubx_msg(ubx, msg, msg_len, true);
    //xSemaphoreGive(xMutex);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] write_ubx_msg failed: %s", __FUNCTION__, esp_err_to_name(ret));
        return ret;
    }
    if(need_ack) 
        ret = ack_status(ubx, *(msg+2), *(msg+3));
    return ret;
}

esp_err_t send_ubx_cfg_msg(ubx_config_t *ubx, uint8_t cls, uint8_t id, uint8_t * payload, size_t len, bool need_ack) {
    TIMER_S
    uint8_t msgb[] = {UBX_HDR_A, UBX_HDR_B, cls, id, 0x00, 0x00, 0x00, 0x00}, *msg = 0;
    size_t msgb_len = sizeof(msgb), total_len = msgb_len + len;
    if(len){
        msg = calloc(total_len, sizeof(uint8_t));
        memcpy(msg, &(msgb[0]), 6); // copy header and class
        encode_uint16(msg+4, len); // add payload size
        memcpy(msg+6, payload, len); // copy payload
    }
    else
        msg = &(msgb[0]); // no payload
    assert(msg);
    esp_err_t ret = ubx_cfg_send_m(ubx, msg, total_len, need_ack);
    if(len)
        free(msg);
    TIMER_E
    return ret;
}

esp_err_t ubx_cfg_valset(ubx_config_t *ubx, uint8_t * payload, size_t len) {
    TIMER_S
    if(ubx->rtc_conf->hw_type < UBX_TYPE_M9)
        return ESP_ERR_INVALID_ARG;
    uint8_t *msg = calloc(len+4, sizeof(uint8_t));
    memcpy(msg, (uint8_t[]){0x01, 0x01, 0x00, 0x00}, 4);
    memcpy(msg+4, payload, len);
    esp_err_t ret = send_ubx_cfg_msg(ubx, CFG_CFG, CFG_VALSET, msg, len + 4, true);
    free(msg);
    TIMER_E
    return ret;
}

esp_err_t ubx_cfg_get(ubx_config_t *ubx, uint8_t cls, uint8_t id, uint8_t *msg, size_t msg_len) {
    TIMER_S
    esp_err_t ret = send_ubx_cfg_msg(ubx, cls, id, NULL, 0, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] send_ubx_cfg_msg failed: %s", __FUNCTION__, esp_err_to_name(ret));
        return ret;
    }
    msg[0] = cls;
    msg[1] = id;
    ret = read_ubx_msg(ubx, msg, msg_len, true); // this msg is without ubx header as ubx_msg_t parts start with class and id
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] read_ubx_msg failed: %s", __FUNCTION__, esp_err_to_name(ret));
    }
    TIMER_E
    return ret;
}

esp_err_t ubx_set_nav_mode(ubx_config_t *ubx, ubx_nav_mode_t nav_mode) {
    LOGR
    uint8_t msg[] = {
        /* mask */ 0xFF, 0xFF,
        nav_mode, /* auto 2D-3D */ 0x03,
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
        /* reserved */ 0x00, 0x00, 0x00, 0x00, 0x00
    };
    ESP_LOGI(TAG, "[%s] nav_mode: %u", __FUNCTION__, nav_mode);
    return send_ubx_cfg_msg(ubx, CLS_CFG, CFG_NAV5, (uint8_t*)&(msg[0]), 36, true);
}

esp_err_t ubx_set_prot_msg_out(ubx_config_t *ubx, bool enable_nmea, bool enable_ubx) {
    LOGR
    esp_err_t ret = ESP_OK;
    if(!enable_nmea && !enable_ubx)
        enable_ubx=true;
    ESP_LOGI(TAG, "[%s] going to enable_nmea: %u, enable_ubx: %u", __FUNCTION__, enable_nmea, enable_ubx);
    ret = ubx_cfg_valset(ubx, (uint8_t[]){
        0x02, 0x00, 0x74, 0x10, enable_nmea ? 0x01 : 0x00,
        0x01, 0x00, 0x74, 0x10, enable_ubx ? 0x01 : 0x00},10);
    if(!ret){
        ESP_LOGI(TAG, "[%s] message protocol set to %s", __FUNCTION__, enable_nmea && enable_ubx ? "NMEA and UBX" : enable_nmea ? "NMEA" : "UBX");
        return ret;}
    else {
        ESP_LOGE(TAG, "[%s] message protocol set failed, %s", __FUNCTION__, esp_err_to_name(ret));
    }
    return send_ubx_cfg_msg(ubx, CLS_CFG, CFG_NAV5, (uint8_t[]){
                                  /* portID, reserved1 */ 0x01, 0x00,
                                  /* txReady x2 */ 0x00, 0x00,
                                  /* mode x4 */ 0xd0, 0x08, 0x00, 0x00,
                                  /* baudRate u4 */ 0x80, 0x25, 0x00, 0x00,
                                  /* inProtoMask x2 */ 0x23, 0x00,
                                  /* outProtoMask x2 */ enable_nmea && enable_ubx ? 0x03 : enable_nmea ? 0x02 : 0x01, 0x00,
                                  /* flags x2 */ 0x00, 0x00,
                                  /* reserved3 u2 */ 0x00, 0x00
                                  }, 20, true);
}

esp_err_t ubx_set_uart_baud_rate(ubx_config_t *ubx, uint32_t baud) {
    LOGR
    esp_err_t ret = ESP_OK;
    uint8_t output_vec[4] = {0, 0, 0, 0};
    encode_uint32(output_vec, baud);
    switch (ubx->rtc_conf->hw_type) {
    case UBX_TYPE_M7:
    case UBX_TYPE_M8:
            ret = send_ubx_cfg_msg(ubx, CLS_CFG, CFG_PRT, (uint8_t[]){
                /* portID, reserved1 */ 0x01, 0x00,
                /* txReady x2 */ 0x00, 0x00,
                /* mode x4 */ 0xd0, 0x08, 0x00, 0x00,
                /* baudRate u4 */ output_vec[0], output_vec[1], output_vec[2], output_vec[3],
                /* inProtoMask x2 */ 0x23, 0x00,
                /* outProtoMask x2 */ 0x03, 0x00,
                /* flags x2 */ 0x00, 0x00,
                /* reserved3 u2 */ 0x00, 0x00
            }, 20, true);
            break;
        case UBX_TYPE_M9:
        case UBX_TYPE_M10:
            ret = ubx_cfg_valset(ubx, (uint8_t[]){
                0x01, 0x00, 0x21, 0x30, output_vec[0], output_vec[1], output_vec[2], output_vec[3]
                }, 8);
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    ubx->rtc_conf->baud = baud;
    //delay_ms(50);
    ret = ubx_uart_set_baud(ubx);
    if(!ret)
        ESP_LOGI(TAG, "[%s] baud rate set to %"PRIu32, __FUNCTION__, baud);
    return ESP_OK;
}

esp_err_t ubx_set_uart_out_rate(ubx_config_t *ubx) {
    LOGR
    esp_err_t ret = ESP_OK;
    uint8_t output_vec[2]={0,0};
    uint32_t baud = UBX_BAUD_38400;
    if(ubx->rtc_conf->output_rate<=UBX_OUTPUT_10HZ){
        switch (ubx->rtc_conf->output_rate) {
            case UBX_OUTPUT_1HZ:
                output_vec[0] = 0xE8;
                output_vec[1] = 0x03;
                break;
            case UBX_OUTPUT_2HZ:
                output_vec[0] = 0xF4;
                output_vec[1] = 0x01;
                break;
            case UBX_OUTPUT_10HZ:
                output_vec[0] = 0xC8;
                output_vec[1] = 0x00;
                baud = UBX_BAUD_115200;
                break;
            case UBX_OUTPUT_5HZ:
            default:
                output_vec[0] = 0xC8;
                output_vec[1] = 0x00;
                break;
        }
    }
    switch (ubx->rtc_conf->hw_type) {
        case UBX_TYPE_M7:
        case UBX_TYPE_M8:
            ret = send_ubx_cfg_msg(ubx, CLS_CFG, CFG_RATE, (uint8_t[]){
                /* measRate 2b */ output_vec[0], output_vec[1],
                /* navRate always 1 */ 0x01, 0x00,
                /* timeRef UTC */ 0x01, 0x00},6, true);
            if(!ret)
                ret = ubx_set_uart_baud_rate(ubx, baud);
            break;
        case UBX_TYPE_M9:
        case UBX_TYPE_M10:
            if(ubx->rtc_conf->output_rate>UBX_OUTPUT_10HZ){
                switch (ubx->rtc_conf->output_rate) {
                    case UBX_OUTPUT_20HZ:
                    default:
                        output_vec[0] = 0x32;
                        output_vec[1] = 0x00;
                        baud = UBX_BAUD_230400;
                        break;
                }
            }
            ret = ubx_cfg_valset(ubx, (uint8_t[]){
                0x01, 0x00, 0x21, 0x30, output_vec[0], output_vec[1]
                }, 6);
            if(!ret)
                ret = ubx_set_uart_baud_rate(ubx, baud);
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    if(!ret)
        ESP_LOGI(TAG, "[%s] output rate set to %d Hz", __FUNCTION__, ubx->rtc_conf->output_rate);
    return ret;
}

esp_err_t ubx_set_gnss(ubx_config_t *ubx, uint8_t mode) {
    LOGR
    uint8_t enable_galileo = 0x01;
    uint8_t enable_beidou = 0x01;
    uint8_t enable_glonass = 0x01;
    if(mode==2) {
        enable_galileo=0x00;
    }
    if(mode<4) {
        enable_beidou=0x00;
    }
    else if(mode==4) {
        enable_glonass = 0x00;
    }
    ESP_LOGI(TAG, "[%s] enable_galileo: %u, enable_beidou: %u, enable_glonass: %u", __FUNCTION__, enable_galileo, enable_beidou, enable_glonass);
    switch (ubx->rtc_conf->hw_type) {
        case UBX_TYPE_M7:
        case UBX_TYPE_M8:
            return send_ubx_cfg_msg(ubx, CLS_CFG, CFG_GNSS, (uint8_t[]){
                    /* msgVer, numTrkChHw, numTrkChUse */ 0x00, 0x20, 0x20,
                    /* numConfig */ 0x07, 
                    /* rep block: gnssID, resTrkCh, maxTrkCh, reserved0, flags */ 
                    /* gps     */ 0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 
                    /* sbas    */ 0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01, 
                    /* galileo */ 0x02, 0x04, 0x08, 0x00, enable_galileo, 0x00, 0x01, 0x01, 
                    /* beidou  */ 0x03, 0x08, 0x10, 0x00, enable_beidou, 0x00, 0x01, 0x01, 
                    /* imes    */ 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 
                    /* qzss    */ 0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01, 
                    /* glonass */ 0x06, 0x08, 0x0E, 0x00, enable_glonass, 0x00, 0x01, 0x01
                    }, 8 * 7 + 4, true);
            break;
        case UBX_TYPE_M9:
        case UBX_TYPE_M10:
            return ubx_cfg_valset(ubx, (uint8_t[]){
                    /* galileo */ 0x21, 0x00, 0x31, 0x10, enable_galileo,
                    /* beidou  */ 0x22, 0x00, 0x31, 0x10, enable_beidou,
                    /* glonass */ 0x25, 0x00, 0x31, 0x10, enable_glonass
                    }, 3*5);
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    return ESP_ERR_INVALID_ARG;
}

esp_err_t ubx_set_msgout(ubx_config_t *ubx) {
    LOGR
    esp_err_t ret = ESP_OK;
    uint8_t cfg_pvt_id = 0x07;
    uint8_t cfg_dop_id = 0x04;
    switch (ubx->rtc_conf->hw_type) {
        case UBX_TYPE_M7:
        case UBX_TYPE_M8:
            ret = send_ubx_cfg_msg(ubx, CLS_CFG, CFG_MSG, (uint8_t[]){
                /* msgClass, msgID */ 0x01, cfg_pvt_id,
                /* rate port 0 i2c */ 0x00, 
                /* rate port 1, 2 serial */ 0x01, 0x00, 
                /* rate port 3 usb, 4 spi, 5 reserved  */ 0x00, 0x00, 0x00,
                }, 8, true);
            if(ret != ESP_OK)
                return ret;
            ret = send_ubx_cfg_msg(ubx, CLS_CFG, CFG_MSG, (uint8_t[]){
                /* msgClass, msgID */ 0x01, cfg_dop_id,
                /* rate port 0 i2c */ 0x00, 
                /* rate port 1, 2 serial */ 0x01, 0x00, 
                /* rate port 3 usb, 4 spi, 5 reserved  */ 0x00, 0x00, 0x00
                }, 8, true);
            break;
        case UBX_TYPE_M9:
        case UBX_TYPE_M10:
            cfg_pvt_id = 0x07;
            cfg_dop_id = 0x39;
            return ubx_cfg_valset(ubx, (uint8_t[]){
                /* pvt id, cfg value */ cfg_pvt_id, 0x00, 0x91, 0x20, 0x01,
                /* dop id, cfg value */ cfg_dop_id, 0x00, 0x91, 0x20, 0x01,
                }, 10);
            break;
        default:
            ret = ESP_ERR_INVALID_ARG;
    }
    return ret;
}

esp_err_t ubx_set_msgout_sat(ubx_config_t *ubx) {
    LOGR
    /* Send rate is relative to the event a message is registered on. 
    For example, if the rate of a navigation message is set to 2, 
    the message is sent every second navigation solution. 
    For configuring NMEA messages, the section NMEA Messages 
    Overview describes class and identifier numbers used. */
    uint8_t cfg_rate = 0x0A;  // 1/10 sec rate
    uint8_t cfg_sat_id = 0x35;
    switch (ubx->rtc_conf->hw_type) {
        case UBX_TYPE_M7:
        case UBX_TYPE_M8:
            return send_ubx_cfg_msg(ubx, CLS_CFG, CFG_MSG, (uint8_t[]){
                /* msgClass, msgID */ 0x01, cfg_sat_id,
                /* rate port 0 i2c */ 0x00,
                /* rate port 1, 2 serial */ cfg_rate, 0x00, 
                /* rate port 3 usb, 4 spi, 5 reserved  */ 0x00, 0x00, 0x00
            }, 8, true);
            break;
        case UBX_TYPE_M9:
            cfg_rate=0x28;  // 1/40 sec rate, 25ms
            /* fall through */ 
        case UBX_TYPE_M10:
            cfg_sat_id = 0x16;
           return ubx_cfg_valset(ubx, (uint8_t[]){
                /* sat id, cfg value */ cfg_sat_id, 0x00, 0x91, 0x20, cfg_rate                    
            }, 5);
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    return ESP_ERR_INVALID_ARG;
}

#define IS_ALNUM(x) (((x)>='a' && (x) <= 'z')) ||((x)>='A' && (x) <= 'Z') || (((x)>='0' && (x) <= '9'))

esp_err_t ubx_get_hw_version(ubx_config_t *ubx) {
    LOGR
    uint8_t * msg = (uint8_t*)&ubx->ubx_msg.mon_ver, *p;
    esp_err_t ret = ubx_cfg_get(ubx, CLS_MON, MON_VER, msg, sizeof(ubx->ubx_msg.mon_ver));
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_cfg_get failed: %s", __FUNCTION__, esp_err_to_name(ret));
        return ret;
    }
    char ver[32] = {0};
    memcpy(ver, msg+4, 30);
    ESP_LOGI(TAG, "swver: [%s]", ver);
    memset(ver, 0, 30);
    memcpy(ver, msg+34, 10);
    ESP_LOGI(TAG, "hwver: [%s]", ver);
    for(int i=0; i<6; ++i) {
        p = msg + 44 + i * 30;
        if(IS_ALNUM((char)*p)){
            memset(ver, 0, 32);
            memcpy(ver, p, 30);
        }
        else {
            break;
        }
        ESP_LOGI(TAG, "verext %d: [%s]", i, ver);
    }
    ubx->rtc_conf->hw_type =*( msg + 34 + 3) == '8' ? UBX_TYPE_M8 :*( msg + 34 + 3) == '9' ? UBX_TYPE_M9 :*( msg + 34 + 3) == 'A' ? UBX_TYPE_M10 : UBX_TYPE_UNKNOWN;
    return ESP_OK;
}

esp_err_t ubx_get_hw_id(ubx_config_t *ubx) {
    LOGR
    uint8_t * msg = (uint8_t*)&ubx->ubx_msg.ubxId;
    esp_err_t ret = ubx_cfg_get(ubx, CLS_SEC, SEC_UBX, msg, sizeof(ubx->ubx_msg.ubxId));
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_cfg_get failed: %s", __FUNCTION__, esp_err_to_name(ret));
        return ret;
    }
    memcpy(&(ubx->rtc_conf->hw_id[0]), msg+8, 6);
    for(int i=0; i<6; ++i) {
        ESP_LOGI(TAG, "hw id[%d]: [%"PRIu8"]", i, *(msg + 8 + i));
    }
    return ESP_OK;
}

esp_err_t ubx_get_gnss(ubx_config_t *ubx) {
    LOGR
    uint8_t * msg = (uint8_t*)&ubx->ubx_msg.monGNSS;
    esp_err_t ret = ubx_cfg_get(ubx, CLS_MON, MON_GNSS, msg, sizeof(ubx->ubx_msg.monGNSS));
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_cfg_get failed: %s", __FUNCTION__, esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

esp_err_t ubx_get_nav_sat(ubx_config_t *ubx) {
    LOGR
    uint8_t *msg = (uint8_t*)&ubx->ubx_msg.nav_sat;
    esp_err_t ret = ubx_cfg_get(ubx, CLS_NAV, NAV_SAT, msg, sizeof(ubx->ubx_msg.nav_sat));
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_cfg_get failed: %s", __FUNCTION__, esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

esp_err_t ubx_uart_set_baud(ubx_config_t *ubx) {
    LOGR
    esp_err_t ret = ESP_OK;
    /* ESP_LOGW(TAG, "[%s] setting uart baud to %"PRIu32, __FUNCTION__, ubx->rtc_conf->baud);
    ret = uart_flush(ubx->uart_num);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] uart_flush failed: %s", __FUNCTION__, esp_err_to_name(ret));
    } */
    ret = uart_set_baudrate(ubx->uart_num, ubx->rtc_conf->baud);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] uart_set_baudrate failed: %s", __FUNCTION__, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t ubx_try_baud(ubx_config_t *ubx, uint8_t * msg, size_t msg_len) {
    LOGR
    esp_err_t ret = ESP_OK;
    uint32_t ubx_baud_rate_temp[6] = {0, 0, 0, 0, 0, 0};
    for(uint8_t i=0, j=sizeof(ubx_baud_rates)/sizeof(ubx_baud_rates[0]), k; i<=j; ++i, k=0) {
        ubx_baud_rate_temp[i] = ubx->rtc_conf->baud;
        if(i>0){
            if(!ubx_baud_rates[i-1])
                goto next;
            while(k<6) {
                if(ubx_baud_rate_temp[k] == ubx_baud_rates[i-1]) {
                    goto next;
                }
                ++k;
            }
            ubx->rtc_conf->baud = ubx_baud_rates[i-1];
            ret = ubx_uart_set_baud(ubx);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "[%s] ubx_uart_set_baud failed: %s", __FUNCTION__, esp_err_to_name(ret));
            }
            //delay_ms(10);
        }
        memset(msg, 0, msg_len);
        ret = read_ubx_msg(ubx, msg, msg_len, false); // just fill the msg buffer to check if we can read ubx or nmea message
        uint8_t * q = msg;
        char * p = 0;
        while(q<(msg+msg_len) && *q) {
            p = (char *)q;
            if(*q == UBX_HDR_A && *(q+1) == UBX_HDR_B) {
                ESP_LOGI(TAG, "found ubx msg hdr");
                return ESP_OK;
                break;
            }
            else if(*p == '$' && *(p+1) == 'G') {
                ESP_LOGI(TAG, "found nmea msg hdr");
                return ESP_OK;
                break;
            }
            ++q;
        }
        
        if (ret != ESP_OK || !msg[3]) {
            ESP_LOGW(TAG, "[%s] %"PRIu32" failed: %s", __FUNCTION__, ubx->rtc_conf->baud, esp_err_to_name(ret));
            if(i<=j) {
                continue;
            }
            return ret;
        }
        next:
    }
    return ret;
}

esp_err_t ubx_initial_read(ubx_config_t *ubx, bool get_hw) {
    LOGR
    esp_err_t ret = ESP_OK;
    uint8_t msg[384] = {0};
    size_t msg_len = sizeof(msg);
    //delay_ms(50);
    ret = ubx_try_baud(ubx, msg, msg_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_try_baud failed: %s", __FUNCTION__, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "ubx_initial_read got: [%s]", msg);
    if(get_hw) {
        const char *p=0;
        if(!(p=strstr((const char *)&(msg[0]), "$GNTXT"))) {
            return ESP_OK;
        }
        
        if(!(p = strstr(p, ",HW UBX"))) {
            return ESP_OK;
        }
        else {
            ret = 1;
            find_space:
            while(*(++p) == ' ')
                ++p;
            if(*p == '0' && *(++p)=='0')
                p+=2;
            else
                goto find_space;
            ubx->rtc_conf->hw_type = *p == '8' ? UBX_TYPE_M8 : *p == '9' ? UBX_TYPE_M9 : *p == 'A' ? UBX_TYPE_M10 : UBX_TYPE_UNKNOWN;
        }
        
        if(p && (p = strstr(p, ",PROTVER="))) {
            ubx->rtc_conf->prot_ver = (uint8_t) atoi(p+9);
        }
        if(p && (p = strstr(p, ",CHIPID="))) {
            p+=14;
            hex_string_to_uint8_t(p, &(ubx->rtc_conf->hw_id[0]), 6);
        }
    }
    ESP_LOGI(TAG, "ubx_initial_read ok.");
    return ret;
}

uint8_t ubx_msg_handler(ubx_config_t *ubx, void * arg) {
    ubx_msg_t *ubxMessage = &ubx->ubx_msg;
    nav_poll_t *navDummy = 0;
    size_t payloadSize = sizeof(ubxMessage->navDummy);
    uint8_t currentMsgType = MT_NONE;
    uint8_t *payloadPtr = (uint8_t *)navDummy;
    int len = 0, fpos = 0;
    uint8_t c;
    while (1) {
        // //xSemaphoreTake(xMutex, portMAX_DELAY);
        // {
        //     len = uart_read_bytes(ubx->uart_num, &c, 1, 20 / portTICK_PERIOD_MS);
        // }
        // //xSemaphoreGive(xMutex);
        if (len <= 0)
            break;
        else {
            if (fpos < 2) {
                // For the first two bytes we are simply looking for a match with
                // the UBX header bytes (0xB5,0x62)
                if (fpos==0 && c == UBX_HDR_A ) {
                    fpos++;
                } else if (fpos==1 && c == UBX_HDR_B) {
                    fpos++;
                    payloadPtr = (uint8_t *)&ubxMessage->navDummy;
                }
                else
                    fpos = 0;  // Reset to beginning state.
            } else {
                // If we come here then fpos >= 2, which means we have found a match
                // with the UBX_HEADER and we are now reading in the bytes that make
                // up the payload. Place the incoming byte into the
                // ubx_msg.navDummy struct. The position is fpos-2 because the
                // struct does not include the initial two-byte header (UBX_HEADER).
                // the struct does not include the 2 last bytes which are the
                // checksums checksums are not placed in the ubx_msg !!!
                if (((fpos - 2) < payloadSize) && payloadPtr) {
                    *(payloadPtr + fpos - 2) = c;
                }

                if (fpos == 3) {
                    // We have just received the second byte of the message type
                    // header, so now we can check to see what kind of message it
                    // is. We have to restore cls and id to the correct substructure
                    if (CLS_NAV == navDummy->cls && NAV_PVT == navDummy->id) {
                        currentMsgType = MT_NAV_PVT;
                        payloadSize = sizeof(struct nav_pvt_s);
                        payloadPtr = (uint8_t *)&ubxMessage->navPvt;
                        ubxMessage->navPvt.cls = navDummy->cls;
                        ubxMessage->navPvt.id = navDummy->id;
                        // ESP_LOGI(TAG, "hPVT ");
                    } else if (CLS_NAV == navDummy->cls && NAV_DOP == navDummy->id) {
                        currentMsgType = MT_NAV_DOP;
                        payloadSize = sizeof(struct nav_dop_s);
                        payloadPtr = (uint8_t *)&ubxMessage->navDOP;
                        ubxMessage->navDOP.cls = navDummy->cls;
                        ubxMessage->navDOP.id = navDummy->id;
                        // ESP_LOGI(TAG, "hDOP ");
                    } else if (CLS_NAV == navDummy->cls && NAV_SAT == navDummy->id) {
                        currentMsgType = MT_NAV_SAT;
                        payloadPtr = (uint8_t *)&ubxMessage->nav_sat;
                        ubxMessage->nav_sat.cls = navDummy->cls;
                        ubxMessage->nav_sat.id = navDummy->id;
                        // ESP_LOGI(TAG, "NAV_SAT");
                    } else if (CLS_MON == navDummy->cls && MON_GNSS == navDummy->id) {
                        currentMsgType = MT_MON_GNSS;
                        payloadSize = sizeof(struct mon_gnss_s);
                        payloadPtr = (uint8_t *)&ubxMessage->monGNSS;
                        ubxMessage->monGNSS.cls = navDummy->cls;
                        ubxMessage->monGNSS.id = navDummy->id;
                        // ESP_LOGI(TAG, "MT_MON_GNSS");
                    } else if (CLS_MON == navDummy->cls && MON_VER == navDummy->id) {
                        currentMsgType = MT_MON_VER;
                        payloadSize = sizeof(struct mon_ver_s);
                        payloadPtr = (uint8_t *)&ubxMessage->mon_ver;
                        ubxMessage->mon_ver.cls = navDummy->cls;
                        ubxMessage->mon_ver.id = navDummy->id;
                        // ESP_LOGI(TAG, "MT_MON_VER");
                    } else if (CLS_ACK == navDummy->cls && ACK_ACK == navDummy->id) {
                        currentMsgType = MT_NAV_ACK;
                        payloadSize = sizeof(struct nav_ack_s);
                        payloadPtr = (uint8_t *)&ubxMessage->navAck;
                        ubxMessage->navAck.cls = navDummy->cls;
                        ubxMessage->navAck.id = navDummy->id;
                        // ESP_LOGI(TAG, "NAV_ACK");
                    } else if (CLS_ACK == navDummy->cls && ACK_NAK == navDummy->id) {
                        currentMsgType = MT_NAV_NACK;
                        payloadSize = sizeof(struct nav_nack_s);
                        payloadPtr = (uint8_t *)&ubxMessage->navNack;
                        ubxMessage->navNack.cls = navDummy->cls;
                        ubxMessage->navNack.id = navDummy->id;
                        // ESP_LOGI(TAG, "NAV_NACK");
                    } else if (CLS_SEC == navDummy->cls && SEC_UBX == navDummy->id) {
                        currentMsgType = MT_NAV_ID;
                        ubxMessage->ubxId.cls = navDummy->cls;
                        ubxMessage->ubxId.id = navDummy->id;
                        payloadPtr = (uint8_t *)&ubxMessage->ubxId;
                        // ESP_LOGI(TAG, "NAV_ID");
                    } else {
                        // unknown message type, bail
                        payloadPtr = 0;
                        currentMsgType = MT_NONE;
                        fpos = 0;
                        continue;
                    }
                }
                // if (((fpos - 2) < payloadSize) && (fpos < 4) && payloadPtr) {
                //     *(payloadPtr + fpos - 2) = c;
                // }
                // if (((fpos - 2) < payloadSize) && (fpos >= 4) && payloadPtr) {
                //     *(payloadPtr + fpos - 2) = c;
                // }
                if (fpos == 6) {
                    if (currentMsgType == MT_NAV_PVT) {
                        ubxMessage->navPvt.len = payloadSize - 6;
                    }  // safety if .len is wrong
                    if (currentMsgType == MT_NAV_DOP) {
                        ubxMessage->navDOP.len = payloadSize - 6;
                    }  // safety if .len is wrong
                    if (currentMsgType == MT_NAV_ID) {
                        payloadSize = ubxMessage->ubxId.len + 6;
                    }  // .len = 9 bytes for M8, but 10 bytes for M10
                    if (currentMsgType == MT_MON_VER) {
                        if (ubxMessage->mon_ver.len + 6 < sizeof(ubxMessage->mon_ver)) {
                            payloadSize = ubxMessage->mon_ver.len + 6;
                        }  // M10 has extensions ??
                        else {
                            fpos = 0;
                        }  // something went wrong, start over again !!!
                    }
                    if (currentMsgType == MT_NAV_SAT) {
                        if (ubxMessage->nav_sat.len + 6 < sizeof(ubxMessage->nav_sat)) {  // safety if .len is wrong
                            payloadSize = ubxMessage->nav_sat.len + 6;
                        }  // payload is variable with nav_sat msg
                        else {
                            fpos = 0;
                        }  // something went wrong, start over again !!!
                    }
                }
                fpos++;
                if (fpos == (payloadSize)) {  // was (payloadSize+2)
                    // All payload bytes have now been received, so we can calculate
                    // the expected checksum value to compare with the next two
                    // incoming bytes. checksum has to calculated out of the correct
                    // substructure !!!
                    add_checksum(payloadPtr, payloadSize, payloadPtr+payloadSize-2, payloadPtr+payloadSize-1);  // was payload !!
                } else if (fpos == (payloadSize + 1)) {
                    // was (payloadSize+3)   fpos-3=c, of payloadsize+1-3=c, dus payloadsize-2
                    // First byte after the payload, ie. first byte of the checksum.
                    // Does it match the first byte of the checksum we calculated?

                    if (c != (payloadPtr[payloadSize - 2])) {
                        // Checksum doesn't match, reset to beginning state and try
                        // again.
                        ESP_LOGI(TAG, "CkA NIO");
                        /* if ((context->gps.Gps_time_set == true) && (context->gps.nav_pvt_message_nr > 10)) {
                            logERR(context, "ChecksumA_NIO\n");
                        } */
                        fpos = 0;
                    }
                } else if (fpos == (payloadSize + 2)) {  // was (payloadSize+4)  fpos-4=c, of payloadsize+1-4=c, dus payloadsize-1
                    // Second byte after the payload, ie. second byte of the checksum. 
                    // Does it match the second byte of the checksum we calculated?
                    fpos = 0;  // We will reset the state regardless of whether the checksum matches.
                    if (c == *(payloadPtr+payloadSize-1)) {
                        // Checksum matches, we have a valid message. iTow has a 18 s diff with UTC, issue with GPS Results !!
                         if(currentMsgType==MT_NAV_SAT){
                            ubxMessage->nav_sat.iTOW=ubxMessage->nav_sat.iTOW-18*1000; //to match 18s diff UTC nav pvt & GPS nav sat !!!
                            add_checksum(payloadPtr, payloadSize, payloadPtr+payloadSize-2, payloadPtr+payloadSize-1);
                            /* calcChecksum(checksum,currentMsgType,payloadSize-2);//have to calculate new checksum !!
                            ((unsigned char*)(&ubxMessage->nav_sat))[payloadSize-2]=checksum[0];//checksum is not on a fixed pos, depends from the payload !!!
                            ((unsigned char*)(&ubxMessage->nav_sat))[payloadSize-1]=checksum[1];//checksum is not on a fixed pos, depends from the payload !!!
                            */
                          }
                        return currentMsgType;
                    } else {
                        ESP_LOGI(TAG, "CkB NIO");
                        /* if ((context->gps.Gps_time_set == true) && (context->gps.nav_pvt_message_nr > 10)) {
                            logERR(context, "ChecksumB_NIO\n");
                        } */
                    }
                } else if (fpos > (payloadSize + 2)) {  // was (payloadSize+4)
                    // We have now read more bytes than both the expected payload
                    // and checksum together, so something went wrong. Reset to
                    // beginning state and try again.
                    fpos = 0;
                }
            }
        }
    }
    return MT_NONE;
}

static int64_t next_time_sync = 0;

int8_t ubx_set_time(ubx_config_t *ubx, float time_offset) {
    LOGR
    assert(ubx);
    ubx_msg_t *ubxMessage = &ubx->ubx_msg;
    if (!ubxMessage->navPvt.numSV || ubxMessage->navPvt.year < 2023) {
        /* if (ubxMessage->navPvt.year < 2023) {
            // ESP_LOGW(TAG, "GPS Reported year not ok (<2023)");
        } */
        return 0;
    }
    uint64_t millis = get_millis();
    if (ubx->time_set && millis < next_time_sync) {
        return 0;
    }
    ESP_LOGI(TAG, "[%s] sats: %" PRIu8, __FUNCTION__, ubxMessage->navPvt.numSV);
#if defined(DLS)
    // summertime is on march 26 2023 2 AM, see
    // https://www.di-mgt.com.au/wclock/help/wclo_tzexplain.html
    setenv("TZ", "CET0CEST,M3.5.0/2,M10.5.0/3", 1);  // timezone UTC = CET, Daylightsaving ON :
                                                     // TZ=CET-1CEST,M3.5.0/2,M10.5.0/3
    tzset();                                         // this works for CET, but TZ string is different for every Land /
                                                     // continent....
#endif
    // uint8_t ret = set_time_zone(time_offset);
    struct tm my_time={0};      // time elements structure
    time_t unix_timestamp = 0;  // a timestamp in seconds
    setenv("TZ", "UTC", 0);
    tzset();
    my_time.tm_sec = ubxMessage->navPvt.second;
    my_time.tm_hour = ubxMessage->navPvt.hour;
    my_time.tm_min = ubxMessage->navPvt.minute;
    my_time.tm_mday = ubxMessage->navPvt.day;
    my_time.tm_mon = ubxMessage->navPvt.month - 1;     // mktime needs months 0 - 11
    my_time.tm_year = ubxMessage->navPvt.year - 1900;  // mktime needs years since 1900, so deduct 1900
    unix_timestamp = mktime(&my_time);                // mktime returns local time, so TZ is important !!!
    int64_t utc_ms = unix_timestamp * 1000LL + (ubxMessage->navPvt.nano + 500000) / 1000000LL;
    ESP_LOGI(TAG, "GPS raw time: %d-%02d-%02d %02d:%02d:%02d %" PRId64, ubxMessage->navPvt.year, ubxMessage->navPvt.month, ubxMessage->navPvt.day, ubxMessage->navPvt.hour, ubxMessage->navPvt.minute, ubxMessage->navPvt.second, utc_ms);
    struct timeval tv = {
        .tv_sec = (time_t)(unix_timestamp + (time_offset * 3600)),
        .tv_usec = 0};  // clean utc time !!
    settimeofday(&tv, NULL);
    struct tm tms;
    localtime_r(&unix_timestamp, &tms);
    ESP_LOGI(TAG, "GPS time set: %d-%02d-%02d %02d:%02d:%02d", (tms.tm_year) + 1900, (tms.tm_mon) + 1, tms.tm_mday, tms.tm_hour, tms.tm_min, tms.tm_sec);
    if (tms.tm_year < 123) {
        ESP_LOGW(TAG, "GPS Reported year not plausible (<2023)!");
        return 0;
    }
    ESP_ERROR_CHECK(esp_event_post(UBX_EVENTS, UBX_EVENT_DATETIME_SET, NULL,0, portMAX_DELAY));
    ESP_LOGI(TAG, "GPS Local Time is set");
    next_time_sync = millis + (60 * 1000);
    ubx->time_set = 1;
    return 1;
}

// private functions

const char * ubx_chip_str(ubx_config_t *ubx) {
    switch (ubx->rtc_conf->hw_type) {
        case UBX_TYPE_M7:
            return "M7";
        case UBX_TYPE_M8:
            return "M8";
        case UBX_TYPE_M9:
            return "M9";
        case UBX_TYPE_M10:
            return "M10";
        default:
            return "UNKNOWN";
    }
}

const char * ubx_baud_str(ubx_config_t * ubx) {
    switch (ubx->rtc_conf->baud) {
        case UBX_BAUD_9600:
            return "9600";
        case UBX_BAUD_38400:
            return "38400";
        case UBX_BAUD_115200:
            return "115200";
        case UBX_BAUD_230400:
            return "230400";
        default:
            return "UNKNOWN";
    }
}

esp_err_t write_ubx_msg(ubx_config_t *ubx, uint8_t *msg, size_t size, bool need_checksum) {
    LOGR
    esp_err_t ret = ESP_OK;
    if(need_checksum)
        add_checksum(msg, size, msg + size - 2, msg + size - 1);
#ifdef DEBUG
    printf("ubx_cfg_m: [ ");
#endif
    for(uint16_t i=0;i<size;++i){ // write the message byte by byte
#ifdef DEBUG
        printf("0x%01x ", *(msg+i));
#endif
        if(uart_write_bytes(ubx->uart_num, msg+i, 1) != 1)
            ret = ESP_FAIL;
    }
#ifdef DEBUG
    printf("] (%u)\n", size);
#endif
    return ret;
}

static const uint8_t ubx_start[] = UBX_HDR;
esp_err_t read_ubx_msg(ubx_config_t * ubx, uint8_t * msg, size_t size, bool verify_ubx_header) {
    LOGR
    esp_err_t ret;
    uint8_t data = 0, got_header = 0;
    uint16_t i = 0, j = 0, k = 0;
    size_t len = 0;
    uint32_t then = get_millis(), now = 0, timeout = 1000;
    //xSemaphoreTake(xMutex, portMAX_DELAY);
    while (i < size) {
        j=0;
        now = get_millis();
        if ((now - then) >= timeout)
            goto done;
        ret = uart_get_buffered_data_len(ubx->uart_num, &len);
        if (ret)
            return ret;
        while (j<len) {
            if (!uart_read_bytes(ubx->uart_num, &data, 1, 20 / portTICK_PERIOD_MS))
                return ESP_ERR_TIMEOUT;
            if(data == ubx_start[0] && verify_ubx_header) { // check for UBX header
                if (!uart_read_bytes(ubx->uart_num, &data, 1, 20 / portTICK_PERIOD_MS))
                    return ESP_ERR_TIMEOUT;
                ++j;
                if(data == ubx_start[1]){
                    if(msg[1] == ubx_start[1] && msg[0] == ubx_start[0]) // check if msg is already filled with UBX header
                        i=2;
                    else // otherways fill from beginning of msg buffer
                        i=0;
                    got_header = 1; // found UBX header
                    //ESP_LOGI(TAG, "[%s] found ubx header at pos %"PRIu16", start read message.", __FUNCTION__, j);
                    goto next_byte;
                }
            }
            // if(i < verify_bytes && data!=*(msg+i)) {
            //     printf("verify bytes failed at pos %"PRIu16", expected: 0x%01x, got: 0x%01x\n", i, *(msg+i), data);
            //     got_header=false;
            // }
            if(verify_ubx_header && !got_header) { // skip until UBX header when verify_ubx_header set
                goto next_byte;
            }
            if (data != *(msg+i)){
                *(msg+i) = data; // fill msg buffer
            }
            ++i;
            if(i >= size) { // msg buffer full, read done
                goto done;
            }
            next_byte:
            ++j;
        }
        if(len && i) // log when got some bytes read
            ESP_LOGI(TAG, "[%s] read len:%u bytes, i:%"PRIu16" of msg size: %u used, outerloop: %"PRIu16, __FUNCTION__, len, i, size, k);
        ++k;
    }
    done:
    ESP_LOGI(TAG, "[%s] done read len:%u bytes, i:%"PRIu16" of msg size: %u used, {cls:%01x, id:%01x}", __FUNCTION__, len, i, size, msg[0], msg[1]);
    //xSemaphoreGive(xMutex);
    if ((now - then) >= timeout) // timeout
        return ESP_ERR_TIMEOUT;
    if(!msg[2]) // no data
        return ESP_ERR_INVALID_RESPONSE;
    return ESP_OK;

}

esp_err_t ack_status(ubx_config_t *ubx, uint8_t cls_id, uint8_t msg_id) {
    LOGR
    esp_err_t ret;
    uint8_t ack_pkt[] = {CLS_ACK, ACK_ACK,
            /* payload size */ 0x02, 0x00,
            /* msgClass, msgId */ cls_id, msg_id,
            /* checksum */ 0x00, 0x00};

    ret = read_ubx_msg(ubx, ack_pkt, sizeof(ack_pkt), true);
    if (ret != ESP_OK)
        return ret;
    uint8_t CK_A = 0, CK_B = 0;
    add_checksum(ack_pkt, sizeof(ack_pkt), &CK_A, &CK_B);

    if (cls_id == ack_pkt[4] && msg_id == ack_pkt[5] && CK_A == ack_pkt[6] && CK_B == ack_pkt[7])
        return ESP_OK;
    else
        return ESP_ERR_INVALID_CRC;
}

void add_checksum(uint8_t *message, uint16_t size, uint8_t *CK_A, uint8_t *CK_B) {
    LOGR
    uint16_t i = 0, j = size - 2;
    if(*message == UBX_HDR_A && *(message+1) == UBX_HDR_B) {
        i=2;
    }
    for (; i < j; i++) {
        *CK_A = *CK_A + message[i];
        *CK_B = *CK_B + *CK_A;
    }
}
