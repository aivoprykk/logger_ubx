/**
 * @file ublox.c
 *
 * @brief This is the source code to the uBlox GPS library for ESP32.
 *
 * @see https://github.com/AngeloElias/esp32-ublox
 *
 */

#include "ubx_private.h"

#if defined(CONFIG_UBLOX_ENABLED)
#include "ubx.h"
#include <esp_log.h>
#include <driver/gpio.h>
#include <sys/time.h>

static const char *TAG = "ublox";

ESP_EVENT_DEFINE_BASE(UBX_EVENT);

#if (CONFIG_LOGGER_UBX_LOG_LEVEL < 2 || CONFIG_LOGGER_GLOBAL_LOG_LEVEL < 2)
static const char * const _ubx_event_strings[] = { UBX_EVENT_LIST(STRINGIFY) };
const char * ubx_event_strings(int id) {
    return _ubx_event_strings[id];
}
#else
const char * ubx_event_strings(int id) {return "UBX_EVENT";}
#endif

RTC_DATA_ATTR ubx_rtc_config_t rtc_config = UBX_RTC_DEFAULT_CONFIG();
static const uint32_t ubx_baud_rates[] = {UBX_BAUD_RATE_LIST(NUMERIFY_V)};
static const uint8_t ubx_hw_types[] = { UBX_TYPE_LIST(NUMERIFY_VV) };
static const char * const ubx_hw_type_strings[] = { UBX_TYPE_LIST(STRINGIFY_M) };
static const char * const ubx_baud_rate_strings[] = { UBX_BAUD_RATE_LIST(STRINGIFY_L) };

ubx_config_t *ubx_config_new() {
    ILOG(TAG, "[%s]", __func__);
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

esp_err_t ubx_config_delete(ubx_config_t *ubx_dev) {
    ILOG(TAG, "[%s]", __func__);
    if (ubx_dev == NULL)
        return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ESP_OK;
    ret = ubx_config_deinit(ubx_dev);
    free(ubx_dev);
    return ret;
}

static esp_err_t ubx_config_init(ubx_config_t *ubx_dev) {
    ILOG(TAG, "[%s]", __func__);
    if (ubx_dev == NULL)
        return ESP_ERR_INVALID_ARG;
    if(ubx_dev->config_ok)
        return ESP_OK;
    esp_err_t ret = ESP_OK;
    ubx_config_t cfg = UBX_DEFAULT_CONFIG();
    memcpy(ubx_dev, &cfg, sizeof(ubx_config_t));
    ubx_dev->rtc_conf = &rtc_config;
    ubx_dev->uart_conf.baud_rate = ubx_dev->rtc_conf->baud;
    if (ubx_dev->xMutex == NULL)
        ubx_dev->xMutex = xSemaphoreCreateMutex();
    ubx_dev->config_ok = true;
    return ret;
}

static esp_err_t ubx_config_deinit(ubx_config_t *ubx_dev) {
    ILOG(TAG, "[%s]", __func__);
    if (ubx_dev == NULL)
        return ESP_ERR_INVALID_ARG;
    if (ubx_dev->xMutex != NULL){
        vSemaphoreDelete(ubx_dev->xMutex);
        ubx_dev->xMutex = NULL;
    }
    ubx_dev->config_ok = false;
    return ESP_OK;
}

static esp_err_t ubx_pins_init(ubx_config_t *ubx_dev) {
    ILOG(TAG, "[%s]", __func__);
    if (ubx_dev == NULL)
        return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ESP_OK;
    uint8_t i = 0;
    while (i < UBX_EN_PIN_LEN) {
        if (ubx_dev->en_pins[i] == GPIO_NUM_NC){
            goto next;
        }
        ret = gpio_set_direction(ubx_dev->en_pins[i], GPIO_MODE_OUTPUT);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] gpio_set_direction failed, gpio:%d, i:%"PRIu8, __FUNCTION__, ubx_dev->en_pins[i], i);
            goto done;
        }
        ret = gpio_set_level(ubx_dev->en_pins[i], true);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] gpio_set_level failed, gpio:%d, i:%"PRIu8, __FUNCTION__, ubx_dev->en_pins[i], i);
            goto done;
        }
        ret = gpio_set_drive_capability(ubx_dev->en_pins[i], GPIO_DRIVE_CAP_3);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] gpio_set_drive_capability failed, gpio:%d, i:%"PRIu8, __FUNCTION__, ubx_dev->en_pins[i], i);
            goto done;
        }
        next:
        ++i;
    }
    done:
    return ret;
}

static esp_err_t ubx_pins_deinit(ubx_config_t *ubx_dev) {
    ILOG(TAG, "[%s]", __func__);
    if (ubx_dev == NULL)
        return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ESP_OK;
    uint8_t i = 0;
    while (i < UBX_EN_PIN_LEN) {
        if (ubx_dev->en_pins[i] == GPIO_NUM_NC)
            goto next;
        ret = gpio_set_level(ubx_dev->en_pins[i], false);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] gpio_set_level failed", __FUNCTION__);
        }
        next:
        i++;
    }
    return ret;
}

static esp_err_t ubx_uart_init(ubx_config_t *ubx_dev) {
    ILOG(TAG, "[%s]", __func__);
    if (ubx_dev == NULL)
        return ESP_ERR_INVALID_ARG;
    if(ubx_dev->uart_setup_ok)
        return ESP_OK;
    if (ubx_dev->config_ok != true) {
        ESP_LOGE(TAG, "you must call ubx_config_init(cfg) first!!!!");
        assert(0);
    }
    esp_err_t ret = ESP_OK;

    if(xSemaphoreTake(ubx_dev->xMutex, portMAX_DELAY) == pdTRUE){

        ret = ubx_pins_init(ubx_dev);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] _dubx_dev_pins_init failed: %s", __FUNCTION__, esp_err_to_name(ret));
            goto done;
        }

        ret = uart_param_config(ubx_dev->uart_num, &(ubx_dev->uart_conf));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] uart_param_config failed", __FUNCTION__);
            goto done;
        }
        ret = uart_set_pin(ubx_dev->uart_num, ubx_dev->tx_pin, ubx_dev->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (ret == ESP_OK) {
            ret = uart_set_sw_flow_ctrl(ubx_dev->uart_num, false, 0, 0);
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
        ret = uart_driver_install(ubx_dev->uart_num, 1024, 0, 0, NULL, intr_alloc_flags);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] uart_driver_install failed", __FUNCTION__);
            goto done;
        }
        done:
        ESP_ERROR_CHECK(esp_event_post(UBX_EVENT, !ret ? UBX_EVENT_UART_INIT_DONE : UBX_EVENT_UART_INIT_FAIL, NULL,0, portMAX_DELAY));
        if(!ret) {
            ILOG(TAG, "[%s] done", __FUNCTION__);
            ubx_dev->uart_setup_ok = true;
        }
        xSemaphoreGive(ubx_dev->xMutex);
    }
    delay_ms(100);
    return ret;
}

static esp_err_t ubx_uart_deinit(ubx_config_t *ubx_dev) {
    ILOG(TAG, "[%s]", __func__);
    if (ubx_dev == NULL)
        return ESP_ERR_INVALID_ARG;
    if (!ubx_dev->uart_setup_ok)
        return ESP_OK;
    esp_err_t ret = ESP_OK;
    if(xSemaphoreTake(ubx_dev->xMutex, portMAX_DELAY) == pdTRUE){
        ret = uart_driver_delete(ubx_dev->uart_num);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] uart_driver_delete failed", __FUNCTION__);
        }

        ret = ubx_pins_deinit(ubx_dev);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] ubx_pins_deinit failed", __FUNCTION__);
        }
        if (!ret) {
            ESP_ERROR_CHECK(esp_event_post(UBX_EVENT, UBX_EVENT_UART_DEINIT_DONE, NULL,0, portMAX_DELAY));
            ubx_dev->uart_setup_ok = false;
        }
        xSemaphoreGive(ubx_dev->xMutex);
    }
    return ret;
}

esp_err_t ubx_on(ubx_config_t *ubx_dev) {
    ILOG(TAG, "[%s] mutex: %p", __func__, ubx_dev->xMutex);
    IMEAS_START();
    esp_err_t ret = ESP_OK;
    ret = ubx_uart_init(ubx_dev);
    IMEAS_END(TAG, "[%s] took %llu", __func__);
    ubx_dev->is_on = true;
    return ret;
}

esp_err_t ubx_off(ubx_config_t *ubx_dev) {
    ILOG(TAG, "[%s] mutex: %p", __func__, ubx_dev->xMutex);
    IMEAS_START();
    esp_err_t ret = ESP_OK;
    ret = ubx_uart_deinit(ubx_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_uart_deinit failed", __FUNCTION__);
    }
    ubx_dev->ready = false;
    ubx_dev->is_on = false;
    IMEAS_END(TAG, "[%s] took %llu", __func__);
    return ret;
}

static esp_err_t fix_config(ubx_config_t *ubx_dev) {
    uint8_t gnss = ubx_dev->rtc_conf->gnss;
    uint8_t gnss_count = 1;
    if(gnss<=5) ubx_dev->rtc_conf->gnss = gnss = ubx_dev->rtc_conf->hw_type >= UBX_TYPE_M9 ? 111 : 103;
    if(ubx_dev->rtc_conf->gnss == 111 && ubx_dev->rtc_conf->hw_type < UBX_TYPE_M9) ubx_dev->rtc_conf->gnss = gnss = 103;
    if((gnss & (1 << 2)) != 0) ++gnss_count; // galileo
    if((gnss & (1 << 3)) != 0) ++gnss_count; // beidou
    if((gnss & (1 << 6)) != 0) ++gnss_count; // glonass
    if(gnss_count > 4) gnss_count = 4;
    ubx_dev->rtc_conf->gnss_count = gnss_count;
    if(ubx_dev->rtc_conf->hw_type == UBX_TYPE_M8){
        if(ubx_dev->rtc_conf->gnss_count >= 2 && ubx_dev->rtc_conf->output_rate > UBX_OUTPUT_10HZ){
            ILOG(TAG, "[%s] 2 gnss, output rate > 10hz, fallback to 10hz", __FUNCTION__);
            ubx_dev->rtc_conf->output_rate = UBX_OUTPUT_10HZ;
        }
        else if(ubx_dev->rtc_conf->gnss_count == 1 && ubx_dev->rtc_conf->output_rate > UBX_OUTPUT_5HZ){
            ILOG(TAG, "[%s] 1 gnss, output rate > 5hz, fallback to 5hz", __FUNCTION__);
            ubx_dev->rtc_conf->output_rate = UBX_OUTPUT_5HZ;
        }
    }
    if(ubx_dev->rtc_conf->hw_type == UBX_TYPE_M10){
        if(ubx_dev->rtc_conf->gnss_count == 4 && ubx_dev->rtc_conf->output_rate > UBX_OUTPUT_10HZ){
            ILOG(TAG, "[%s] 4 gnss, output rate > 10hz, fallback to 10hz", __FUNCTION__);
            ubx_dev->rtc_conf->output_rate = UBX_OUTPUT_10HZ;
        }
        else if(ubx_dev->rtc_conf->gnss_count == 3 && ubx_dev->rtc_conf->output_rate > UBX_OUTPUT_16HZ){
            ILOG(TAG, "[%s] 3 gnss, output rate > 16hz, fallback to 16hz", __FUNCTION__);
            ubx_dev->rtc_conf->output_rate = UBX_OUTPUT_16HZ;
        }
        else if(ubx_dev->rtc_conf->gnss_count == 2 && ubx_dev->rtc_conf->output_rate > UBX_OUTPUT_20HZ){
            ILOG(TAG, "[%s] 2 gnss, output rate > 20hz, fallback to 20hz", __FUNCTION__);
            ubx_dev->rtc_conf->output_rate = UBX_OUTPUT_20HZ;
        }
    }
    return ESP_OK;
}

esp_err_t ubx_setup(ubx_config_t *ubx_dev) {
    ILOG(TAG, "[%s]", __func__);
    IMEAS_START();
    esp_err_t ret = ESP_OK;
    if (ubx_dev == NULL){
        ret  = ESP_ERR_INVALID_ARG;
        goto done;
    }
    if(ubx_dev->uart_setup_ok || ubx_dev->config_progress){
        goto done;
    }
    ubx_dev->config_progress = 1;
    ret = ubx_on(ubx_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_on failed", __FUNCTION__);
        goto fail;
    }
    // initial read
    ret = ubx_initial_read(ubx_dev, false);
    
    // find hw model
    uint8_t try, max_tries=3;
    for(try = 0;try<=max_tries; ++try) {
        ret = ubx_get_hw_version(ubx_dev);
        if(ret == ESP_ERR_TIMEOUT)
            goto fail;
        if(ret == ESP_OK)
            break;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_get_hw_version failed", __FUNCTION__);
        goto fail;
    }
    
    fix_config(ubx_dev);

    // set ubx message protocol
    for(try = 0;try<=max_tries; ++try) {
        ret = ubx_set_prot_msg_out(ubx_dev, false, true);
        if(ret == ESP_OK)
            break;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_prot_msg_out failed", __FUNCTION__);
        goto fail;
    }
    
    if(ubx_dev->rtc_conf->hw_type == UBX_TYPE_M8){
        for(try = 0;try<=max_tries; ++try) {
            ret = ubx_set_nav_mode(ubx_dev, ubx_dev->rtc_conf->nav_mode);
            if(ret == ESP_OK)
            break;
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] ubx_set_nav_mode failed", __FUNCTION__);
        }
    }

    for(try = 0;try<=max_tries; ++try) {
        ret = ubx_set_msgout(ubx_dev);
        if(ret == ESP_OK)
            break;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_set_msgout failed", __FUNCTION__);
    }
    if(ubx_dev->rtc_conf->msgout_sat){
        for(try = 0;try<=max_tries; ++try) {
            ret = ubx_set_msgout_sat(ubx_dev);
            if(ret == ESP_OK)
            break;
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] ubx_set_msgout_sat failed", __FUNCTION__);
        }
    }

    for(try = 0;try<=max_tries; ++try) {
        ret = ubx_set_gnss(ubx_dev, ubx_dev->rtc_conf->gnss);
        if(ret == ESP_OK)
            break;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_set_gnss failed", __FUNCTION__);
        goto fail;
    }

    for(try = 0;try<=max_tries; ++try) {
        ret = ubx_set_uart_out_rate(ubx_dev);
        if(ret == ESP_OK)
            break;
        else
            delay_ms(100);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_set_uart_out_rate failed", __FUNCTION__);
        goto fail;
    }
    
    for(try = 0;try<=max_tries; ++try) {
        ret = ubx_get_hw_id(ubx_dev);
        if(ret == ESP_OK)
            break;
        delay_ms(100);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_get_hw_id failed", __FUNCTION__);
        goto fail;
    }

    for(try = 0;try<=max_tries; ++try) {
        ret = ubx_get_gnss(ubx_dev);
        if(ret == ESP_OK)
            break;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_get_gnss failed", __FUNCTION__);
        goto fail;
    }
    fail:
    ubx_dev->config_progress = 0;
    done:
    ESP_ERROR_CHECK(esp_event_post(UBX_EVENT, !ret ? UBX_EVENT_SETUP_DONE : UBX_EVENT_SETUP_FAIL, NULL,0, portMAX_DELAY));
    if(!ret){
        ubx_dev->ready = true;
        ubx_dev->ready_time = get_millis();
    }
    IMEAS_END(TAG, "[%s] took %llu", __func__);
    return ret;

}

esp_err_t ubx_set_nav_mode(ubx_config_t *ubx, ubx_nav_mode_t nav_mode) {
    ILOG(TAG, "[%s]", __func__);
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
    ILOG(TAG, "[%s] nav_mode: %u", __FUNCTION__, nav_mode);
    return send_ubx_cfg_msg(ubx, CLS_CFG, CFG_NAV5, (uint8_t*)&(msg[0]), 36, true);
}

static esp_err_t ubx_set_prot_msg_out(ubx_config_t *ubx, bool enable_nmea, bool enable_ubx) {
    ILOG(TAG, "[%s]", __func__);
    esp_err_t ret = ESP_OK;
    if(!enable_nmea && !enable_ubx)
        enable_ubx=true;
    ILOG(TAG, "[%s] going to enable_nmea: %u, enable_ubx: %u", __FUNCTION__, enable_nmea, enable_ubx);
    ret = ubx_cfg_valset(ubx, (const uint8_t[]){
        0x02, 0x00, 0x74, 0x10, enable_nmea ? 0x01 : 0x00,
        0x01, 0x00, 0x74, 0x10, enable_ubx ? 0x01 : 0x00},10, true);
    if(!ret){
        ILOG(TAG, "[%s] message protocol set to %s", __FUNCTION__, enable_nmea && enable_ubx ? "NMEA and UBX" : enable_nmea ? "NMEA" : "UBX");
        return ret;
    }
    // fallback hw m8 and below
    return send_ubx_cfg_msg(ubx, CLS_CFG, CFG_NAV5, (const uint8_t[]){
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

static esp_err_t ubx_set_uart_baud_rate(ubx_config_t *ubx, uint32_t baud) {
    ILOG(TAG, "[%s]", __func__);
    if(baud == ubx->rtc_conf->baud){
        ILOG(TAG, "[%s] baud rate already set to %"PRIu32", no changes made.", __FUNCTION__, baud);
        return ESP_OK;
    }
    esp_err_t ret = ESP_OK;
    uint8_t output_vec[4] = {0, 0, 0, 0};
    encode_uint32(output_vec, baud);
    ret = ubx_cfg_valset(ubx, (const uint8_t[]){
        0x01, 0x00, 0x52, 0x40, output_vec[0], output_vec[1], output_vec[2], output_vec[3]
        }, 8, false);
    if(!ret)
        goto done;
    else {
        ESP_LOGE(TAG, "[%s] ubx_cfg_valset failed, fallback to old cfg_msg.", __FUNCTION__);
    }
    // fallback hw m8 and below
    ret = send_ubx_cfg_msg(ubx, CLS_CFG, CFG_PRT, (const uint8_t[]){
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
    ret = ubx_uart_set_baud(ubx);
    return ESP_OK;
}

static esp_err_t ubx_set_uart_out_rate(ubx_config_t *ubx) {
    ILOG(TAG, "[%s]", __func__);
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
    ILOG(TAG, "[%s] solutions:%hhu output rate: %"PRIu8", baud: %"PRIu32, __FUNCTION__, ubx->rtc_conf->gnss_count, ubx->rtc_conf->output_rate, baud);
    ret = ubx_cfg_valset(ubx, (const uint8_t[]){
        0x01, 0x00, 0x21, 0x30, output_vec[0], output_vec[1]
        }, 6, true);
    if(!ret)
        goto done;
    else {
        ESP_LOGE(TAG, "[%s] ubx_cfg_valset failed, fallback to old cfg_msg.", __FUNCTION__);
    }
    // fallback hw m8 and below
    ret = send_ubx_cfg_msg(ubx, CLS_CFG, CFG_RATE, (const uint8_t[]){
                /* measRate 2b */ output_vec[0], output_vec[1],
                /* navRate always 1 */ 0x01, 0x00,
                /* timeRef UTC */ 0x01, 0x00},6, true);
    done:
    if(!ret)
        ret = ubx_set_uart_baud_rate(ubx, baud);
    return ret;
}

static esp_err_t ubx_set_gnss(ubx_config_t *ubx, uint8_t mode) {
    ILOG(TAG, "[%s]", __func__);
    uint8_t enable_gps = 0x01; // us gps
    uint8_t enable_sbas = 0x01; // us sbas
    uint8_t enable_galileo = 0x00; // eu galileo
    uint8_t enable_beidou = 0x00; // cn beidou
    uint8_t enable_qzss = 0x01; // jp qzss
    uint8_t enable_glonass = 0x00; // ru glonass
    
    if((mode & (1 << 1)) == 0) {
        enable_sbas=0x00;
    }
    if((mode & (1 << 2)) != 0) {
        enable_galileo=0x01;
    }
    if((mode & (1 << 3)) != 0) {
        enable_beidou=0x01;
    }
    if((mode & (1 << 5)) == 0) {
        enable_qzss=0x00;
    }
    if((mode & (1 << 6)) != 0) {
        enable_glonass=0x01;
    }
    if(ubx->rtc_conf->gnss_count < 1) {
        ESP_LOGE(TAG, "[%s] count_solutions < 1, fallback to gps", __FUNCTION__);
        enable_gps = 0x01;
    }
    else if(ubx->rtc_conf->gnss_count > 4) {
        ESP_LOGE(TAG, "[%s] count_solutions > 4, fallback to gps+galileo+glonass+beidou", __FUNCTION__);
        enable_gps = 0x01;
        enable_galileo = 0x01;
        enable_glonass = 0x01;
        enable_beidou = 0x01;
    }
    uint8_t gnss_cmd[64] = {
                    /* gps     */     0x1f, 0x00, 0x31, 0x10, enable_gps,
                    /* sbas    */     0x20, 0x00, 0x31, 0x10, enable_sbas,
                    /* galileo */     0x21, 0x00, 0x31, 0x10, enable_galileo,
                    /* beidou  */     0x22, 0x00, 0x31, 0x10, enable_beidou,
                    /* qzss    */     0x24, 0x00, 0x31, 0x10, enable_qzss, 
                    /* glonass */     0x25, 0x00, 0x31, 0x10, enable_glonass
                    };
    uint8_t gnss_cursor = 6*5;
    uint8_t tmp[] = {0x00, 0x31, 0x10, 0x01};
    if(enable_gps) {
        gnss_cmd[gnss_cursor++] = 0x01; // enable gps l1c/a
        memcpy(&gnss_cmd[gnss_cursor], tmp, 4), gnss_cursor += 4;
    }
    if(enable_galileo) {
        gnss_cmd[gnss_cursor++] = 0x07; // enable galileo e1
        memcpy(&gnss_cmd[gnss_cursor], tmp, 4), gnss_cursor += 4;
    }
    if(enable_glonass) {
        gnss_cmd[gnss_cursor++] = 0x18; // enable glonass l1of
        memcpy(&gnss_cmd[gnss_cursor], tmp, 4), gnss_cursor += 4;
    }
    if(enable_beidou) {
        if(ubx->rtc_conf->hw_type <= UBX_TYPE_M9 || !enable_glonass) {
            gnss_cmd[gnss_cursor++] = 0x0d; // enable beidou b1l
            memcpy(&gnss_cmd[gnss_cursor], tmp, 4), gnss_cursor += 4;
        }
        else {
            gnss_cmd[gnss_cursor++] = 0x0d; // disable beidou b1l
            memcpy(&gnss_cmd[gnss_cursor], tmp, 3), gnss_cursor += 3;
            gnss_cmd[gnss_cursor++] = 0x00;
            gnss_cmd[gnss_cursor++] = 0x0f; // enable beidou b1c
            memcpy(&gnss_cmd[gnss_cursor], tmp, 4), gnss_cursor += 4;
        }
    }
    ILOG(TAG, "[%s] mode:%hhu, gps(us): %hhu, sbas(us): %hhu galileo(eu): %hhu, beidou(cn): %hhu, glonass(ru): %hhu, qzss(jp): %hhu", __FUNCTION__, mode, enable_gps, enable_sbas, enable_galileo, enable_beidou, enable_glonass, enable_qzss);
    esp_err_t ret = ubx_cfg_valset(ubx, gnss_cmd, gnss_cursor, true);
    if(!ret)
        return ret;
    else {
        ESP_LOGW(TAG, "[%s] ubx_cfg_valset failed, fallback to old cfg_msg.", __FUNCTION__);
    }
    // fallback hw m8 and below
    return send_ubx_cfg_msg(ubx, CLS_CFG, CFG_GNSS, (const uint8_t[]){
                    /* msgVer, numTrkChHw, numTrkChUse */ 0x00, 0x20, 0x20,
                    /* numConfig */ 0x07, 
                    /* rep block: gnssID, resTrkCh, maxTrkCh, reserved0, flags */ 
                    /* gps     */ 0x00, 0x08, 0x10, 0x00, enable_gps, 0x00, 0x01, 0x01, 
                    /* sbas    */ 0x01, 0x01, 0x03, 0x00, enable_sbas, 0x00, 0x01, 0x01, 
                    /* galileo */ 0x02, 0x04, 0x08, 0x00, enable_galileo, 0x00, 0x01, 0x01, 
                    /* beidou  */ 0x03, 0x08, 0x10, 0x00, enable_beidou, 0x00, 0x01, 0x01, 
                    /* qzss    */ 0x05, 0x00, 0x03, 0x00, enable_qzss, 0x00, 0x01, 0x01, 
                    /* glonass */ 0x06, 0x08, 0x0E, 0x00, enable_glonass, 0x00, 0x01, 0x01
                    }, 8 * 6 + 4, true);
}

static esp_err_t ubx_set_msgout(ubx_config_t *ubx) {
    ILOG(TAG, "[%s]", __func__);
    esp_err_t ret = ESP_OK;
    uint8_t cfg_pvt_id = 0x07;
    uint8_t cfg_dop_id = 0x04;
    if(ubx->rtc_conf->hw_type >= UBX_TYPE_M9){
            cfg_pvt_id = 0x07;
            cfg_dop_id = 0x39;
    }
    ILOG(TAG, "[%s] enable navpvt and navdop ubx messages.", __FUNCTION__);
    ret = ubx_cfg_valset(ubx, (const uint8_t[]){
                /* pvt id, cfg value */ cfg_pvt_id, 0x00, 0x91, 0x20, 0x01,
                /* dop id, cfg value */ cfg_dop_id, 0x00, 0x91, 0x20, 0x01,
                }, 10, true);
    if(!ret)
        return ret;
    else {
        ESP_LOGW(TAG, "[%s] ubx_cfg_valset failed, fallback to old cfg_msg.", __FUNCTION__);
    }
    // fallback hw m8 and below
    ret = send_ubx_cfg_msg(ubx, CLS_CFG, CFG_MSG, (const uint8_t[]){
                /* msgClass, msgID */ 0x01, cfg_pvt_id,
                /* rate port 0 i2c */ 0x00, 
                /* rate port 1, 2 serial */ 0x01, 0x00, 
                /* rate port 3 usb, 4 spi, 5 reserved  */ 0x00, 0x00, 0x00,
                }, 8, true);
    if(ret != ESP_OK)
        return ret;
    ret = send_ubx_cfg_msg(ubx, CLS_CFG, CFG_MSG, (const uint8_t[]){
                /* msgClass, msgID */ 0x01, cfg_dop_id,
                /* rate port 0 i2c */ 0x00, 
                /* rate port 1, 2 serial */ 0x01, 0x00, 
                /* rate port 3 usb, 4 spi, 5 reserved  */ 0x00, 0x00, 0x00
                }, 8, true);
    return ret;
}

static esp_err_t ubx_set_msgout_sat(ubx_config_t *ubx) {
    ILOG(TAG, "[%s]", __func__);
    /* Send rate is relative to the event a message is registered on. 
    For example, if the rate of a navigation message is set to 2, 
    the message is sent every second navigation solution. 
    For configuring NMEA messages, the section NMEA Messages 
    Overview describes class and identifier numbers used. */
    uint8_t cfg_rate = (ubx->rtc_conf->output_rate & 0xff);  // once in a second
    uint8_t cfg_sat_id = 0x16;
    esp_err_t ret =  ubx_cfg_valset(ubx, (const uint8_t[]){
                /* sat id, cfg value */ cfg_sat_id, 0x00, 0x91, 0x20, cfg_rate                    
            }, 5, true);
    if(!ret)
        return ret;
    else {
        ESP_LOGW(TAG, "[%s] ubx_cfg_valset failed, fallback to old cfg_msg.", __FUNCTION__);
    }
    // fallback hw m8 and below
    return send_ubx_cfg_msg(ubx, CLS_CFG, CFG_MSG, (const uint8_t[]){
                /* msgClass, msgID */ 0x01, cfg_sat_id,
                /* rate port 0 i2c */ 0x00,
                /* rate port 1, 2 serial */ cfg_rate, 0x00, 
                /* rate port 3 usb, 4 spi, 5 reserved  */ 0x00, 0x00, 0x00
            }, 8, true);
}

static esp_err_t ubx_uart_save_cfg(ubx_config_t *ubx) {
    ILOG(TAG, "[%s]", __func__);
    esp_err_t ret = send_ubx_cfg_msg(ubx, CLS_CFG, CFG_CFG, (const uint8_t[]){
                0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x1c
            }, 13, true);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] send_ubx_cfg_msg failed: %s", __FUNCTION__, esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t ubx_uart_set_baud(ubx_config_t *ubx_dev) {
    ILOG(TAG, "[%s] %lu", __func__, ubx_dev->rtc_conf->baud);
    esp_err_t ret = ESP_OK;
    delay_ms(10);
    if(xSemaphoreTake(ubx_dev->xMutex, portMAX_DELAY) == pdTRUE) {
        ret = uart_set_baudrate(ubx_dev->uart_num, ubx_dev->rtc_conf->baud);
        xSemaphoreGive(ubx_dev->xMutex);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] uart_set_baudrate failed: %s", __FUNCTION__, esp_err_to_name(ret));
    }
    delay_ms(50);
    return ret;
}

// get routines //

static esp_err_t ubx_get_hw_version(ubx_config_t *ubx) {
    ILOG(TAG, "[%s]", __func__);
    uint8_t * msg = (uint8_t*)&ubx->ubx_msg.mon_ver, *p;
    ubx_msg_byte_ctx_t ubx_packet = UBX_MSG_BYTE_CTX_DEFAULT(ubx->ubx_msg);
    *ubx_packet.msg = CLS_MON;
    *(ubx_packet.msg+1) = MON_VER;
    esp_err_t ret = ubx_cfg_get(ubx, &ubx_packet);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_cfg_get failed: %s", __FUNCTION__, esp_err_to_name(ret));
        return ret;
    }
    char ver[32] = {0};
    memcpy(ver, msg+4, 30);
    ILOG(TAG, "swver: [%s]", ver);
    memset(ver, 0, 30);
    memcpy(ver, msg+34, 10);
    ILOG(TAG, "hwver: [%s]", ver);
    for(int i=0; i<6; ++i) {
        p = msg + 44 + i * 30;
        if(IS_ALNUM((char)*p)){
            memset(ver, 0, 32);
            memcpy(ver, p, 30);
        }
        else {
            break;
        }
        ILOG(TAG, "verext %d: [%s]", i, ver);
    }
    ubx->rtc_conf->hw_type =*( msg + 34 + 3) == '8' ? UBX_TYPE_M8 : *( msg + 34 + 3) == '9' ? UBX_TYPE_M9 : *( msg + 34 + 3) == 'A' ? UBX_TYPE_M10 : UBX_TYPE_M0;
    return ESP_OK;
}

static esp_err_t ubx_get_hw_id(ubx_config_t *ubx) {
    ILOG(TAG, "[%s]", __func__);
    uint8_t * msg = (uint8_t*)&ubx->ubx_msg.ubxId;
    ubx_msg_byte_ctx_t ubx_packet = UBX_MSG_BYTE_CTX_DEFAULT(ubx->ubx_msg);
    *ubx_packet.msg = CLS_SEC;
    *(ubx_packet.msg+1) = SEC_UBX;
    esp_err_t ret = ubx_cfg_get(ubx, &ubx_packet);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_cfg_get failed: %s", __FUNCTION__, esp_err_to_name(ret));
        return ret;
    }
    memcpy(&(ubx->rtc_conf->hw_id[0]), msg+8, 6);
    for(int i=0; i<6; ++i) {
        ILOG(TAG, "hw id[%d]: [%"PRIu8"]", i, *(msg + 8 + i));
    }
    return ESP_OK;
}

static esp_err_t ubx_get_gnss(ubx_config_t *ubx) {
    ILOG(TAG, "[%s]", __func__);
    uint8_t * msg = (uint8_t*)&ubx->ubx_msg.monGNSS;
    ubx_msg_byte_ctx_t ubx_packet = UBX_MSG_BYTE_CTX_DEFAULT(ubx->ubx_msg);
    *ubx_packet.msg = CLS_MON;
    *(ubx_packet.msg+1) = MON_GNSS;
    esp_err_t ret = ubx_cfg_get(ubx, &ubx_packet);
    if(ret != ESP_OK) {
        ESP_LOGI(TAG, "[%s] ubx_cfg_get failed: %s", __FUNCTION__, esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

static esp_err_t ubx_get_nav_sat(ubx_config_t *ubx) {
    ILOG(TAG, "[%s]", __func__);
    uint8_t *msg = (uint8_t*)&ubx->ubx_msg.nav_sat;
    ubx_msg_byte_ctx_t ubx_packet = UBX_MSG_BYTE_CTX_DEFAULT(ubx->ubx_msg);
    *ubx_packet.msg = CLS_NAV;
    *(ubx_packet.msg+1) = NAV_SAT;
    esp_err_t ret = ubx_cfg_get(ubx, &ubx_packet);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_cfg_get failed: %s", __FUNCTION__, esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

static esp_err_t ubx_try_baud(ubx_config_t *ubx, ubx_msg_byte_ctx_t * ubx_packet) {
    ILOG(TAG, "[%s]", __func__);
    esp_err_t ret = ESP_OK;
    uint32_t ubx_baud_rate_temp[6] = {0, 0, 0, 0, 0, 0};
    for(uint8_t i=0, j=lengthof(ubx_baud_rates), k; i<=j; ++i, k=0) {
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
            delay_ms(50);
        }
#if (CONFIG_LOGGER_UBX_LOG_LEVEL < 2)
        printf("[%s] try read initial data with %"PRIu32"\n", __FUNCTION__, ubx->rtc_conf->baud);
#endif
        memset(ubx_packet->msg, 0, ubx_packet->msg_size);
        //ubx_packet->ubx_msg = &ubx->ubx_msg;
        ret = read_ubx_msg(ubx, ubx_packet); // just fill the msg buffer to check if we can read ubx or nmea message
        uint8_t * q = ubx_packet->msg;
        char * p = 0;
        while(q<(ubx_packet->msg+ubx_packet->msg_size) && *q) {
            p = (char *)q;
            if(*q == UBX_HDR_A && *(q+1) == UBX_HDR_B) {
                ILOG(TAG, "[%s] found UBX message at %d with baud: %"PRIu32, __FUNCTION__, p-(char*)ubx_packet->msg, ubx->rtc_conf->baud);
                return ESP_OK;
                break;
            }
            else if(*p == '$' && *(p+1) == 'G') {
                ILOG(TAG, "[%s] found NMEA message at %d with baud: %"PRIu32, __FUNCTION__, p-(char*)ubx_packet->msg, ubx->rtc_conf->baud);
                return ESP_OK;
                break;
            }
            ++q;
        }
        
        if (ret != ESP_OK || !*(ubx_packet->msg+3)) {
            ESP_LOGW(TAG, "[%s] %"PRIu32" failed: %s", __FUNCTION__, ubx->rtc_conf->baud, esp_err_to_name(ret));
            if(i<=j) {
                continue;
            }
            return ret;
        }
        next:
        UNUSED_PARAMETER(q);
    }
    return ret;
}

static uint8_t hex_char_to_uint8_t(char c) {
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

static void hex_string_to_uint8_t(const char* hex_string, uint8_t* output, size_t output_size) {
    for (size_t i = 0; i < output_size; ++i) {
        char c1 = hex_string[i * 2];
        char c2 = hex_string[i * 2 + 1];
        output[i] = (hex_char_to_uint8_t(c1) << 4) + hex_char_to_uint8_t(c2);
    }
}

static esp_err_t ubx_initial_read(ubx_config_t *ubx, bool get_hw) {
    ILOG(TAG, "[%s]", __func__);
    esp_err_t ret = ESP_OK;
    uint8_t msg[384] = {0};
    size_t msg_size = sizeof(msg);
    ubx_msg_byte_ctx_t ubx_packet = UBX_MSG_BYTE_CTX_DEFAULT(ubx->ubx_msg);
    ubx_packet.msg = msg;
    ubx_packet.msg_size = msg_size;
    ubx_packet.expect_ubx_msg = false;
    ubx_packet.msg_type_handler = NULL;
    ubx_packet.msg_pos = 0;
    ubx_packet.msg_match_to_pos = false;
    ret = ubx_try_baud(ubx, &ubx_packet);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] ubx_try_baud failed: %s", __FUNCTION__, esp_err_to_name(ret));
        return ret;
    }
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
            ubx->rtc_conf->hw_type = *p == '8' ? UBX_TYPE_M8 : *p == '9' ? UBX_TYPE_M9 : *p == 'A' ? UBX_TYPE_M10 : UBX_TYPE_M0;
        }
        
        if(p && (p = strstr(p, ",PROTVER="))) {
            ubx->rtc_conf->prot_ver = (uint8_t) atoi(p+9);
        }
        if(p && (p = strstr(p, ",CHIPID="))) {
            p+=14;
            hex_string_to_uint8_t(p, &(ubx->rtc_conf->hw_id[0]), 6);
        }
    }
    ILOG(TAG, "[%s] ok.", __func__);
    return ret;
}

// private functions

const char * ubx_chip_str(const ubx_config_t *ubx) {
    switch (ubx->rtc_conf->hw_type) {
        case UBX_TYPE_M7:
            return ubx_hw_type_strings[1];
        case UBX_TYPE_M8:
            return ubx_hw_type_strings[2];
        case UBX_TYPE_M9:
            return ubx_hw_type_strings[3];
        case UBX_TYPE_M10:
            return ubx_hw_type_strings[4];
        default:
            return "UNKNOWN";
    }
}

const char * ubx_baud_str(const ubx_config_t * ubx) {
    switch (ubx->rtc_conf->baud) {
        case UBX_BAUD_9600:
            return ubx_baud_rate_strings[0];
        case UBX_BAUD_38400:
            return ubx_baud_rate_strings[1];
        case UBX_BAUD_115200:
            return ubx_baud_rate_strings[2];
        case UBX_BAUD_230400:
            return ubx_baud_rate_strings[3];
        default:
            return "UNKNOWN";
    }
}


// #define MIN_numSV_FIRST_FIX 5      // alvorens start loggen, changed from 4 to 5 7.1/2023
// #define MAX_Sacc_FIRST_FIX 2       // alvorens start loggen
// #define MIN_numSV_GPS_SPEED_OK  4  // min aantal satellieten voor berekenen snelheid, anders
// #define MAX_Sacc_GPS_SPEED_OK  1   // max waarde Sacc voor berekenen snelheid, anders 0
// #define MAX_GPS_SPEED_OK  40       // max snelheid in m/s voor berekenen snelheid, anders 0
// #define MIN_SPEED_START_LOGGING 2000        //was 2000 min speed in mm/s over 2 s alvorens start loggen naar SD 
// #define TIME_DELAY_FIRST_FIX 10 //10 navpvt messages alvorens start loggen

#endif