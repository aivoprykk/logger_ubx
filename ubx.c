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

ESP_EVENT_DEFINE_BASE(UBX_EVENT);

TIMER_INIT

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
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ESP_OK;
    uint8_t i = 0;
    while (i < UBX_EN_PIN_LEN) {
        if (ubx->en_pins[i] == GPIO_NUM_NC){
            goto next;
        }
        ret = gpio_set_direction(ubx->en_pins[i], GPIO_MODE_OUTPUT);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] gpio_set_direction failed, gpio:%d, i:%"PRIu8, __FUNCTION__, ubx->en_pins[i], i);
            goto done;
        }
        ret = gpio_set_level(ubx->en_pins[i], true);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] gpio_set_level failed, gpio:%d, i:%"PRIu8, __FUNCTION__, ubx->en_pins[i], i);
            goto done;
        }
        ret = gpio_set_drive_capability(ubx->en_pins[i], GPIO_DRIVE_CAP_3);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] gpio_set_drive_capability failed, gpio:%d, i:%"PRIu8, __FUNCTION__, ubx->en_pins[i], i);
            goto done;
        }
        next:
        ++i;
    }
    done:
    return ret;
}

esp_err_t ubx_pins_deinit(ubx_config_t *ubx) {
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ESP_OK;
    uint8_t i = 0;
    while (i < UBX_EN_PIN_LEN) {
        if (ubx->en_pins[i] == GPIO_NUM_NC)
            goto next;
        ret = gpio_set_level(ubx->en_pins[i], false);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] gpio_set_level failed", __FUNCTION__);
        }
        next:
        i++;
    }
    return ret;
}

esp_err_t ubx_uart_init(ubx_config_t *ubx) {
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    if(ubx->uart_setup_ok)
        return ESP_OK;
    if (ubx->config_ok != true) {
        ESP_LOGE(TAG, "you must call ubx_config_init(cfg) first!!!!");
        assert(0);
    }
    esp_err_t ret = ESP_OK;

    ret = ubx_pins_init(ubx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_pins_init failed: %s", __FUNCTION__, esp_err_to_name(ret));
        goto done;
    }

    ret = uart_param_config(ubx->uart_num, &(ubx->uart_conf));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] uart_param_config failed", __FUNCTION__);
        goto done;
    }
    ret = uart_set_pin(ubx->uart_num, ubx->tx_pin, ubx->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret == ESP_OK) {
        ret = uart_set_sw_flow_ctrl(ubx->uart_num, false, 0, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] uart_set_sw_flow_ctrl failed", __FUNCTION__);
            goto done;
        }
    } else {
        ESP_LOGW(TAG, "[%s] uart_set_pin failed", __FUNCTION__);
        goto done;
    }
    int intr_alloc_flags = 0;
#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif
    ret = uart_driver_install(ubx->uart_num, 1024, 0, 0, NULL, intr_alloc_flags);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] uart_driver_install failed", __FUNCTION__);
        goto done;
    }
    done:
    ESP_ERROR_CHECK(esp_event_post(UBX_EVENT, !ret ? UBX_EVENT_UART_INIT_DONE : UBX_EVENT_UART_INIT_FAIL, NULL,0, portMAX_DELAY));
    if(!ret) {
        ESP_LOGW(TAG, "[%s] uart_driver_install done", __FUNCTION__);
        ubx->uart_setup_ok = true;
    }
    delay_ms(50);
    return ret;
}

esp_err_t ubx_uart_deinit(ubx_config_t *ubx) {
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
        ESP_ERROR_CHECK(esp_event_post(UBX_EVENT, UBX_EVENT_UART_DEINIT_DONE, NULL,0, portMAX_DELAY));
        ubx->uart_setup_ok = false;
    }
    return ret;
}

esp_err_t ubx_on(ubx_config_t *ubx) {
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    if(ubx->uart_setup_ok)
        return ESP_OK;
    esp_err_t ret = ubx_uart_init(ubx);
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
    ubx->ready = false;
    return ret;
}

esp_err_t ubx_setup(ubx_config_t *ubx) {
    if (ubx == NULL)
        return ESP_ERR_INVALID_ARG;
    if(ubx->ready)
        return ESP_OK;
    TIMER_S    
    esp_err_t ret = ubx_on(ubx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_on failed", __FUNCTION__);
        goto done;
    }
    //delay_ms(100);
    // initial read
    ret = ubx_initial_read(ubx, false);
    
    // find hw model
    uint8_t try, max_tries=3;
    for(try = 0;try<=max_tries; ++try) {
        ret = ubx_get_hw_version(ubx);
        if(ret == ESP_OK)
            break;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_get_hw_version failed", __FUNCTION__);
        goto done;
    }

    // set ubx message protocol
    for(try = 0;try<=max_tries; ++try) {
        ret = ubx_set_prot_msg_out(ubx, false, true);
        if(ret == ESP_OK)
            break;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_prot_msg_out failed", __FUNCTION__);
        goto done;
    }
    
    if(ubx->rtc_conf->hw_type == UBX_TYPE_M8){
        for(try = 0;try<=max_tries; ++try) {
            ret = ubx_set_nav_mode(ubx, ubx->rtc_conf->nav_mode);
            if(ret == ESP_OK)
            break;
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] ubx_set_nav_mode failed", __FUNCTION__);
        }
    }

    for(try = 0;try<=max_tries; ++try) {
        ret = ubx_set_msgout(ubx);
        if(ret == ESP_OK)
            break;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_set_msgout failed", __FUNCTION__);
    }
    if(ubx->rtc_conf->msgout_sat){
        for(try = 0;try<=max_tries; ++try) {
            ret = ubx_set_msgout_sat(ubx);
            if(ret == ESP_OK)
            break;
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] ubx_set_msgout_sat failed", __FUNCTION__);
        }
    }

    for(try = 0;try<=max_tries; ++try) {
        ret = ubx_set_gnss(ubx, ubx->rtc_conf->gnss);
        if(ret == ESP_OK)
            break;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_set_gnss failed", __FUNCTION__);
        goto done;
    }

    for(try = 0;try<=max_tries; ++try) {
        ret = ubx_set_uart_out_rate(ubx);
        if(ret == ESP_OK)
            break;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_set_uart_out_rate failed", __FUNCTION__);
        goto done;
    }
    
    for(try = 0;try<=max_tries; ++try) {
        ret = ubx_get_hw_id(ubx);
        if(ret == ESP_OK)
            break;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_get_hw_id failed", __FUNCTION__);
        goto done;
    }

    for(try = 0;try<=max_tries; ++try) {
        ret = ubx_get_gnss(ubx);
        if(ret == ESP_OK)
            break;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_get_gnss failed", __FUNCTION__);
        goto done;
    }

    done:
    ESP_ERROR_CHECK(esp_event_post(UBX_EVENT, !ret ? UBX_EVENT_SETUP_DONE : UBX_EVENT_SETUP_FAIL, NULL,0, portMAX_DELAY));
    if(!ret){
        ubx->ready = true;
        ubx->ready_time = get_millis();
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

void decode_uint16(const uint8_t* hex_string, uint16_t *output) {
    *output = (*(hex_string) + (*(hex_string+1) << 8));
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

esp_err_t ubx_cfg_valset(ubx_config_t *ubx, uint8_t * payload, size_t len, bool need_ack) {
    TIMER_S
    if(ubx->rtc_conf->hw_type < UBX_TYPE_M9)
        return ESP_ERR_INVALID_ARG;
    uint8_t *msg = calloc(len+4, sizeof(uint8_t));
    memcpy(msg, (uint8_t[]){0x01, 0x01, 0x00, 0x00}, 4);
    memcpy(msg+4, payload, len);
    esp_err_t ret = send_ubx_cfg_msg(ubx, CLS_CFG, CFG_VALSET, msg, len + 4, need_ack);
    free(msg);
    TIMER_E
    return ret;
}

esp_err_t ubx_cfg_get(ubx_msg_byte_ctx_t * mctx) {
    TIMER_S
    assert(mctx);
    esp_err_t ret = send_ubx_cfg_msg(mctx->ubx, *mctx->msg, *(mctx->msg+1), NULL, 0, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] send_ubx_cfg_msg failed: %s", __FUNCTION__, esp_err_to_name(ret));
        return ret;
    }
    ret = read_ubx_msg(mctx); // this msg is without ubx header as ubx_msg_t parts start with class and id
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] read_ubx_msg failed: %s", __FUNCTION__, esp_err_to_name(ret));
    }
    TIMER_E
    return ret;
}

esp_err_t ubx_set_nav_mode(ubx_config_t *ubx, ubx_nav_mode_t nav_mode) {
    LOGR
    const uint8_t msg[] = {
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
        0x01, 0x00, 0x74, 0x10, enable_ubx ? 0x01 : 0x00},10, true);
    if(!ret){
        ESP_LOGI(TAG, "[%s] message protocol set to %s", __FUNCTION__, enable_nmea && enable_ubx ? "NMEA and UBX" : enable_nmea ? "NMEA" : "UBX");
        return ret;
    }
    // fallback hw m8 and below
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
    if(baud == ubx->rtc_conf->baud){
        ESP_LOGV(TAG, "[%s] baud rate already set to %"PRIu32", no changes made.", __FUNCTION__, baud);
        return ESP_OK;
    }
    esp_err_t ret = ESP_OK;
    uint8_t output_vec[4] = {0, 0, 0, 0};
    encode_uint32(output_vec, baud);
    ret = ubx_cfg_valset(ubx, (uint8_t[]){
        0x01, 0x00, 0x52, 0x40, output_vec[0], output_vec[1], output_vec[2], output_vec[3]
        }, 8, false);
    if(!ret)
        goto done;
    else {
        ESP_LOGE(TAG, "[%s] ubx_cfg_valset failed, fallback to old cfg_msg.", __FUNCTION__);
    }
    // fallback hw m8 and below
    ret = send_ubx_cfg_msg(ubx, CLS_CFG, CFG_PRT, (uint8_t[]){
        /* portID, reserved1 */ 0x01, 0x00,
        /* txReady x2 */ 0x00, 0x00,
        /* mode x4 */ 0xd0, 0x08, 0x00, 0x00,
        /* baudRate u4 */ output_vec[0], output_vec[1], output_vec[2], output_vec[3],
        /* inProtoMask x2 */ 0x23, 0x00,
        /* outProtoMask x2 */ 0x03, 0x00,
        /* flags x2 */ 0x00, 0x00,
        /* reserved3 u2 */ 0x00, 0x00
    }, 20, false);
    done:
    ubx->rtc_conf->baud = baud;
    //delay_ms(50);
    ret = ubx_uart_set_baud(ubx);
    if(!ret)
        ESP_LOGV(TAG, "[%s] baud rate set to %"PRIu32, __FUNCTION__, baud);
    return ESP_OK;
}

esp_err_t ubx_set_uart_out_rate(ubx_config_t *ubx) {
    LOGR
    esp_err_t ret = ESP_OK;
    uint8_t output_vec[2]={0,0};
    uint32_t baud = UBX_BAUD_38400;
    encode_uint16(&(output_vec[0]), (1000/ubx->rtc_conf->output_rate));
    if(ubx->rtc_conf->output_rate > UBX_OUTPUT_10HZ){
        baud = UBX_BAUD_230400;
    }
    else if(ubx->rtc_conf->output_rate==UBX_OUTPUT_10HZ){   
        baud = UBX_BAUD_115200;
    }
    else if(ubx->rtc_conf->output_rate==UBX_OUTPUT_5HZ){   
        baud = UBX_BAUD_57600;
    } 
    else {
        baud = UBX_BAUD_38400;
    }
    ESP_LOGI(TAG, "[%s] output rate: %"PRIu8", baud: %"PRIu32, __FUNCTION__, ubx->rtc_conf->output_rate, baud);
    ret = ubx_cfg_valset(ubx, (uint8_t[]){
        0x01, 0x00, 0x21, 0x30, output_vec[0], output_vec[1]
        }, 6, true);
    if(!ret)
        goto done;
    else {
        ESP_LOGE(TAG, "[%s] ubx_cfg_valset failed, fallback to old cfg_msg.", __FUNCTION__);
    }
    // fallback hw m8 and below
    ret = send_ubx_cfg_msg(ubx, CLS_CFG, CFG_RATE, (uint8_t[]){
                /* measRate 2b */ output_vec[0], output_vec[1],
                /* navRate always 1 */ 0x01, 0x00,
                /* timeRef UTC */ 0x01, 0x00},6, true);
    done:
    if(!ret)
        ret = ubx_set_uart_baud_rate(ubx, baud);
    if(!ret)
        ESP_LOGV(TAG, "[%s] output rate set to %d Hz", __FUNCTION__, ubx->rtc_conf->output_rate);
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
    if(enable_glonass==1 && enable_beidou==1 && enable_galileo==1)
        enable_beidou = 0x00; // default beidou disabled
    ESP_LOGI(TAG, "[%s] mode:%hhu, enable_galileo: %u, enable_beidou: %u, enable_glonass: %u", __FUNCTION__, mode, enable_galileo, enable_beidou, enable_glonass);
    esp_err_t ret = ubx_cfg_valset(ubx, (uint8_t[]){
                    /* galileo */ 0x21, 0x00, 0x31, 0x10, enable_galileo,
                    /* beidou  */ 0x22, 0x00, 0x31, 0x10, enable_beidou,
                    /* glonass */ 0x25, 0x00, 0x31, 0x10, enable_glonass
                    }, 3 * 5, true);
    if(!ret)
        return ret;
    else {
        ESP_LOGW(TAG, "[%s] ubx_cfg_valset failed, fallback to old cfg_msg.", __FUNCTION__);
    }
    // fallback hw m8 and below
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
}

esp_err_t ubx_set_msgout(ubx_config_t *ubx) {
    LOGR
    esp_err_t ret = ESP_OK;
    uint8_t cfg_pvt_id = 0x07;
    uint8_t cfg_dop_id = 0x04;
    if(ubx->rtc_conf->hw_type >= UBX_TYPE_M9){
            cfg_pvt_id = 0x07;
            cfg_dop_id = 0x39;
    }
    ESP_LOGV(TAG, "[%s] enable navpvt and navdop ubx messages.", __FUNCTION__);
    ret = ubx_cfg_valset(ubx, (uint8_t[]){
                /* pvt id, cfg value */ cfg_pvt_id, 0x00, 0x91, 0x20, 0x01,
                /* dop id, cfg value */ cfg_dop_id, 0x00, 0x91, 0x20, 0x01,
                }, 10, true);
    if(!ret)
        return ret;
    else {
        ESP_LOGW(TAG, "[%s] ubx_cfg_valset failed, fallback to old cfg_msg.", __FUNCTION__);
    }
    // fallback hw m8 and below
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
        case UBX_TYPE_M9:
            cfg_rate=0x28;  // 1/40 sec rate, 25ms
            /* fall through */ 
        case UBX_TYPE_M10:
            cfg_sat_id = 0x16;
            break;
        default:
            break;
    }
    esp_err_t ret =  ubx_cfg_valset(ubx, (uint8_t[]){
                /* sat id, cfg value */ cfg_sat_id, 0x00, 0x91, 0x20, cfg_rate                    
            }, 5, true);
    if(!ret)
        return ret;
    else {
        ESP_LOGW(TAG, "[%s] ubx_cfg_valset failed, fallback to old cfg_msg.", __FUNCTION__);
    }
    // fallback hw m8 and below
    return send_ubx_cfg_msg(ubx, CLS_CFG, CFG_MSG, (uint8_t[]){
                /* msgClass, msgID */ 0x01, cfg_sat_id,
                /* rate port 0 i2c */ 0x00,
                /* rate port 1, 2 serial */ cfg_rate, 0x00, 
                /* rate port 3 usb, 4 spi, 5 reserved  */ 0x00, 0x00, 0x00
            }, 8, true);
}

esp_err_t ubx_uart_save_cfg(ubx_config_t *ubx) {
    LOGR
    esp_err_t ret = send_ubx_cfg_msg(ubx, CLS_CFG, CFG_CFG, (uint8_t[]){
                0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x1c
            }, 13, true);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] send_ubx_cfg_msg failed: %s", __FUNCTION__, esp_err_to_name(ret));
    }
    return ret;
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

// get routines //

esp_err_t ubx_get_hw_version(ubx_config_t *ubx) {
    LOGR
    uint8_t * msg = (uint8_t*)&ubx->ubx_msg.mon_ver, *p;
    ubx_msg_byte_ctx_t mctx = UBX_MSG_BYTE_CTX_DEFAULT();
    *mctx.msg = CLS_MON;
    *(mctx.msg+1) = MON_VER;
    esp_err_t ret = ubx_cfg_get(&mctx);
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
    ubx_msg_byte_ctx_t mctx = UBX_MSG_BYTE_CTX_DEFAULT();
    *mctx.msg = CLS_SEC;
    *(mctx.msg+1) = SEC_UBX;
    esp_err_t ret = ubx_cfg_get(&mctx);
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
    ubx_msg_byte_ctx_t mctx = UBX_MSG_BYTE_CTX_DEFAULT();
    *mctx.msg = CLS_MON;
    *(mctx.msg+1) = MON_GNSS;
    esp_err_t ret = ubx_cfg_get(&mctx);
    if(ret != ESP_OK) {
        ESP_LOGI(TAG, "[%s] ubx_cfg_get failed: %s", __FUNCTION__, esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

esp_err_t ubx_get_nav_sat(ubx_config_t *ubx) {
    LOGR
    uint8_t *msg = (uint8_t*)&ubx->ubx_msg.nav_sat;
    ubx_msg_byte_ctx_t mctx = UBX_MSG_BYTE_CTX_DEFAULT();
    *mctx.msg = CLS_NAV;
    *(mctx.msg+1) = NAV_SAT;
    esp_err_t ret = ubx_cfg_get(&mctx);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_cfg_get failed: %s", __FUNCTION__, esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

esp_err_t ubx_try_baud(ubx_msg_byte_ctx_t * mctx) {
    LOGR
    esp_err_t ret = ESP_OK;
    uint32_t ubx_baud_rate_temp[6] = {0, 0, 0, 0, 0, 0};
    ubx_config_t *ubx = mctx->ubx;
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
        memset(mctx->msg, 0, mctx->msg_size);
        mctx->ubx = ubx;
        ret = read_ubx_msg(mctx); // just fill the msg buffer to check if we can read ubx or nmea message
        uint8_t * q = mctx->msg;
        char * p = 0;
        while(q<(mctx->msg+mctx->msg_size) && *q) {
            p = (char *)q;
            if(*q == UBX_HDR_A && *(q+1) == UBX_HDR_B) {
                return ESP_OK;
                break;
            }
            else if(*p == '$' && *(p+1) == 'G') {
                return ESP_OK;
                break;
            }
            ++q;
        }
        
        if (ret != ESP_OK || !*(mctx->msg+3)) {
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
    size_t msg_size = sizeof(msg);
    //delay_ms(50);
    ubx_msg_byte_ctx_t mctx = UBX_MSG_BYTE_CTX_DEFAULT();
    mctx.msg = msg;
    mctx.msg_size = msg_size;
    mctx.expect_ubx_msg = false;
    mctx.msg_type_handler = NULL;
    mctx.msg_pos = 0;
    mctx.msg_match_to_pos = false;
    ret = ubx_try_baud(&mctx);
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

// private functions

const char * ubx_chip_str(const ubx_config_t *ubx) {
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

const char * ubx_baud_str(const ubx_config_t * ubx) {
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

esp_err_t write_ubx_msg(const ubx_config_t *ubx, uint8_t *msg, size_t size, bool need_checksum) {
    LOGR
    esp_err_t ret = ESP_OK;
    if(need_checksum)
        add_checksum(msg, size, msg + size - 2, msg + size - 1);
#ifdef DEBUG
    printf("ubx_cfg_m: [ ");
#endif
    for(uint16_t i=0; i < size; ++i){ // write the message byte by byte
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


void add_checksum(uint8_t *message, uint16_t size, uint8_t *CK_A, uint8_t *CK_B) {
    uint16_t i = 0, j = size - 2;
    if(*message == UBX_HDR_A && *(message+1) == UBX_HDR_B) {
        i=2;
    }
    for (; i < j; i++) {
        *CK_A = *CK_A + message[i];
        *CK_B = *CK_B + *CK_A;
    }
}
