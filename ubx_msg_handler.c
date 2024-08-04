#include <sys/time.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"

#include "ubx_events.h"
#include "ubx_private.h"
#include "ubx.h"
#include "ubx_msg.h"
#include "logger_common.h"

#define LOG_MSG_BITS 0
#define LOG_MSG_JSON 0

#define BITVAL(a, b) ((a >> b) & 0x01)
#define BITS(a, b, c) ((a >> b) & ((1 << c) - 1))
#define BITS_FROM_U8(source,lsb,msb) \
    ((uint8_t)((source) & ((uint8_t)(((uint8_t)(0xFF >> ((uint8_t)(7-((uint8_t)(msb) & 7))))) & ((uint8_t)(0xFF << ((uint8_t)(lsb) & 7)))))))

static const char *TAG = "ubx_msg_handler";

#if LOG_MSG_JSON == 1
#include "strbf.h"
#define json_obj_begin(msg) "{\"" msg "\":{ "
#define json_obj_end " }}"

esp_err_t nav_pvt_serialize_json(nav_pvt_t *nav_pvt, strbf_t * msgbf) { // at leqast 320bytes
    assert(nav_pvt && msgbf);
    strbf_puts(msgbf, json_obj_begin("NAV-PVT"));
    strbf_sprintf(msgbf, "iTOW:%"PRIu32", time:%02u:%02u:%02u.%"PRId32", date:%02u-%02u-%04u, ", nav_pvt->iTOW, nav_pvt->hour, nav_pvt->minute, nav_pvt->second, nav_pvt->nano, nav_pvt->day, nav_pvt->month, nav_pvt->year);
    strbf_sprintf(msgbf, "valid:%"PRIu8", validflags:{validDate:%x, validTime:%x, fullyResolved:%x, validMag:%x}, ", nav_pvt->valid, BITVAL(nav_pvt->valid, 0), BITVAL(nav_pvt->valid, 1), BITVAL(nav_pvt->valid, 2), BITVAL(nav_pvt->valid, 3));
    strbf_sprintf(msgbf, "tAcc:%"PRIu32", fixType:%02x, ", nav_pvt->tAcc, nav_pvt->fixType);
    strbf_sprintf(msgbf, "flags:{gnssFixOK:%x, diffSoln:%x, psmState:%"PRIu8", headVehValid:%x, carrSoln:%"PRIu8"}, ", BITVAL(nav_pvt->flags, 0), BITVAL(nav_pvt->flags, 1), BITS(nav_pvt->flags, 2, 4), BITVAL(nav_pvt->flags, 5),BITS(nav_pvt->flags, 6, 8));
    strbf_sprintf(msgbf, "flags2:{confirmedAvai:%x, confirmedDate:%x, confirmedTime:%x}, ", BITVAL(nav_pvt->flags2, 5), BITVAL(nav_pvt->flags2, 6), BITVAL(nav_pvt->flags2, 7));
    strbf_sprintf(msgbf, "numSV:%"PRIu8", lon:%"PRIu32", lat:%"PRIu32", ", nav_pvt->numSV, nav_pvt->lon, nav_pvt->lat);
    strbf_sprintf(msgbf, "height:%"PRIu32", hMSL:%"PRIu32", hAcc:%"PRIu32", vAcc:%"PRIu32", ", nav_pvt->height, nav_pvt->hMSL, nav_pvt->hAcc, nav_pvt->vAcc);
    strbf_sprintf(msgbf, "velN:%"PRId32", velE:%"PRId32", velD:%"PRId32", ", nav_pvt->velN, nav_pvt->velE, nav_pvt->velD);
    strbf_sprintf(msgbf, "gSpeed:%"PRId32", headMot:%"PRId32", sAcc:%"PRIu32", headAcc:%"PRIu32", ", nav_pvt->gSpeed, nav_pvt->heading, nav_pvt->sAcc, nav_pvt->headingAcc);
    strbf_sprintf(msgbf, "pDOP: %"PRIu16", ", nav_pvt->pDOP);
    strbf_sprintf(msgbf, "flags3:{invalidLlh:%x}", BITVAL(nav_pvt->flags3, 0));
    strbf_sprintf(msgbf, json_obj_end" (%"PRIu16")", msgbf->cur-msgbf->start);
    return ESP_OK;
}

esp_err_t nav_sat_serialize_json(nav_sat_t *nav_sat, strbf_t * msgbf) { // at least 128bytes
    assert(nav_sat && msgbf);
    strbf_puts(msgbf, json_obj_begin("NAV-SAT"));
    strbf_sprintf(msgbf, "\"iTOW\":%"PRIu32", \"version\":%x, \"numSvs\":%"PRIu8", \"reserved1\":%x, \"reserved2\":%x, ", 
        nav_sat->iTOW, nav_sat->version, nav_sat->numSvs, nav_sat->reserved1, nav_sat->reserved2);
    strbf_puts(msgbf, ", \"sat\":[\n");
    for(uint8_t i=0; i<nav_sat->numSvs; ++i) {
        if(i>MAX_SVS)
            break;
        if(i>0)
            strbf_puts(msgbf, ",\n");
        strbf_sprintf(msgbf, "{\"gnssId\":%"PRIu8", \"svId\":%"PRIu8", \"cno\":%"PRIu8", \"elev\":%"PRId8", \"azim\":%"PRId16", \"prRes\":%"PRId16", \"flags\":[", 
        nav_sat->sat[i].gnssId, nav_sat->sat[i].svId, nav_sat->sat[i].cno, nav_sat->sat[i].elev, nav_sat->sat[i].azim, nav_sat->sat[i].prRes); 
        strbf_sprintf(msgbf, "{qualityInd:%"PRIu8",svUsed:%x,health:%"PRIu8",diffCorr:%x}", BITS(nav_sat->sat[i].flags, 0, 2), BITVAL(nav_sat->sat[i].flags, 3),BITS(nav_sat->sat[i].flags,4,5),BITVAL(nav_sat->sat[i].flags, 6));
        strbf_sprintf(msgbf, "]}");
    }
    strbf_sprintf(msgbf, "]"json_obj_end" (%"PRIu16")", msgbf->cur-msgbf->start);
    return ESP_OK;
}

esp_err_t nav_dop_serialize_json(nav_dop_t * nav_dop, strbf_t * msgbf) { // at least 128bytes
    assert(nav_dop && msgbf);
    strbf_puts(msgbf, json_obj_begin("NAV-DOP"));
    strbf_sprintf(msgbf, "\"iTOW\":%"PRIu32", \"gDOP\":%"PRIu16", \"pDOP\":%"PRIu16", \"tDOP\":%"PRIu16", \"vDOP\":%"PRIu16", \"hDOP\":%"PRIu16", \"nDOP\":%"PRIu16", \"eDOP\":%"PRIu16, 
        nav_dop->iTOW, nav_dop->gDOP, nav_dop->pDOP, nav_dop->tDOP, nav_dop->vDOP, nav_dop->hDOP, nav_dop->nDOP, nav_dop->eDOP);
    strbf_sprintf(msgbf, json_obj_end" (%"PRIu16")", msgbf->cur-msgbf->start);
    return ESP_OK;
}

esp_err_t mon_gnss_serialize_json(mon_gnss_t * mon_gnss, strbf_t * msgbf) { // at least 128bytes
    assert(mon_gnss && msgbf);
    strbf_puts(msgbf, json_obj_begin("MON-GNSS"));
    strbf_sprintf(msgbf, "\"version\":%x,", mon_gnss->Version);
    strbf_sprintf(msgbf, "\"supported\":{\"GPSSup\":%x, \"GlonassSup\":%x, \"BeidouSup\":%x, \"GaileoSup\":%x}, ", 
        BITVAL(mon_gnss->supported_Gnss, 0), BITVAL(mon_gnss->supported_Gnss, 1), BITVAL(mon_gnss->supported_Gnss, 2), BITVAL(mon_gnss->supported_Gnss, 3));
    strbf_sprintf(msgbf, "\"defaultGnss\":{\"GPSDef\":%x, \"GlonassDef\":%x, \"BeidouDef\":%x, \"GaileoDef\":%x}, ", 
        BITVAL(mon_gnss->default_Gnss, 0), BITVAL(mon_gnss->default_Gnss, 1), BITVAL(mon_gnss->default_Gnss, 2), BITVAL(mon_gnss->default_Gnss, 3));
    strbf_sprintf(msgbf, "\"enabled\":{\"GPSEna\":%x, \"GlonassEna\":%x, \"BeidouEna\":%x, \"GaileoEna\":%x}, ", 
        BITVAL(mon_gnss->enabled_Gnss, 0), BITVAL(mon_gnss->enabled_Gnss, 1), BITVAL(mon_gnss->enabled_Gnss, 2), BITVAL(mon_gnss->enabled_Gnss, 3));
    strbf_sprintf(msgbf, "\"simultaneous\": %"PRIu8, mon_gnss->simultaneous);
    strbf_sprintf(msgbf, json_obj_end" (%"PRIu16")", msgbf->cur-msgbf->start);
    return ESP_OK;
}

esp_err_t mon_ver_serialize_json(mon_ver_t * mon_ver, strbf_t * msgbf) { // at least 128bytes
    assert(mon_ver && msgbf);
    strbf_puts(msgbf, json_obj_begin("MON-VER"));
    strbf_sprintf(msgbf, "\"swVersion\":\"%s\", \"hwVersion\":\"%s\", \"extension\":[", mon_ver->swVersion, mon_ver->hwVersion);
    uint8_t i=0;
    char *p;
    while(true) {
        p = mon_ver->ext[i++].extension;
        if(!p || !*p || (*p > 127)){ // ascii only
            break;
        }
        if(i>1)
            strbf_sprintf(msgbf, ",");
        strbf_sprintf(msgbf, " \"%s\"", p);
    }
    strbf_sprintf(msgbf, json_obj_end" (%"PRIu16")", msgbf->cur-msgbf->start);
    return ESP_OK;
}

esp_err_t nav_ack_serialize_json(nav_ack_t * nav_ack, strbf_t * msgbf) { // at least 128bytes
    assert(nav_ack && msgbf);
    strbf_puts(msgbf, json_obj_begin("NAV-ACK"));
    strbf_sprintf(msgbf, "\"cls\":%02x, \"id\":%02x", nav_ack->msg_cls, nav_ack->msg_id);
    strbf_sprintf(msgbf, json_obj_end" (%"PRIu16")", msgbf->cur-msgbf->start);
    return ESP_OK;
}

esp_err_t nav_id_serialize_json(nav_id_t * nav_id, strbf_t * msgbf) { // at least 128bytes
    assert(nav_id && msgbf);
    strbf_puts(msgbf, json_obj_begin("SEC-UNIQID"));
    strbf_sprintf(msgbf, "\"Version\":%x, \"uniqueId\":\"%02x%02x%02x%02x%02x",
        nav_id->Version, nav_id->ubx_id_1, nav_id->ubx_id_2, nav_id->ubx_id_3, nav_id->ubx_id_4, nav_id->ubx_id_5);
    // M10 has 6 bytes, M8 has 5 bytes
    strbf_sprintf(msgbf, "%02x" , nav_id->ubx_id_6);
    strbf_sprintf(msgbf, "\""json_obj_end" (%"PRIu16")", msgbf->cur-msgbf->start);
    return ESP_OK;
}

esp_err_t ubx_msg_serialize_json(ubx_msg_byte_ctx_t * mctx, strbf_t * msgbf) {
    assert(mctx && mctx->msg);
    switch(mctx->ubx_msg_type) {
        case MT_NAV_PVT:
            return nav_pvt_serialize_json((nav_pvt_t *)mctx->msg, msgbf);
        case MT_NAV_DOP:
           return nav_dop_serialize_json((nav_dop_t *)mctx->msg, msgbf);
        case MT_NAV_SAT:
            return nav_sat_serialize_json((nav_sat_t *)mctx->msg, msgbf);
        case MT_MON_GNSS:
            return mon_gnss_serialize_json((mon_gnss_t *)mctx->msg, msgbf);
        case MT_MON_VER:
            return mon_ver_serialize_json((mon_ver_t *)mctx->msg, msgbf);
        case MT_NAV_ACK:
            return nav_ack_serialize_json((nav_ack_t *)mctx->msg, msgbf);
        // case MT_NAV_NACK:
        //     return nav_nack_serialize_json((nav_nack_t *)mctx->msg, msgbf);
        case MT_NAV_ID:
            return nav_id_serialize_json((nav_id_t *)mctx->msg, msgbf);
        default:
            ESP_LOGW(TAG, "[%s] unknown ubx message type: %u", __FUNCTION__, mctx->ubx_msg_type);
            return ESP_ERR_NOT_SUPPORTED;
    };
    return ESP_OK;
}

#endif

esp_err_t ubx_msg_type_handler(struct ubx_msg_byte_ctx_s * mctx) {
    assert(mctx);
    if(!mctx->msg) {
        ESP_LOGW(TAG, "msg is NULL, can not handle message.");
        return ESP_ERR_INVALID_ARG;
    }
    assert(mctx->ubx);
    ubx_msg_t *msg = &mctx->ubx->ubx_msg;
     switch (*mctx->msg) {
        case CLS_NAV:
            switch (*(mctx->msg + 1)) {
                case NAV_PVT:
#if LOG_MSG_BITS == 2 && CONFIG_UBLOX_LOG_LEVEL < 1
                    printf(">> NAV_PVT >>\n");
#endif
                    mctx->ubx_msg_type = MT_NAV_PVT;
                    mctx->msg = (uint8_t *)&msg->navPvt;
                    mctx->msg_size = sizeof(struct nav_pvt_s);
                    // *mctx->msg = CLS_NAV;
                    // *(mctx->msg+1) = NAV_PVT;
                    break;
                case NAV_DOP:
#if LOG_MSG_BITS == 2 && CONFIG_UBLOX_LOG_LEVEL < 1
                    printf(">> NAV_DOP >>\n");
#endif
                    mctx->ubx_msg_type = MT_NAV_DOP;
                    mctx->msg = (uint8_t *)&msg->navDOP;
                    mctx->msg_size = sizeof(struct nav_dop_s);
                    // *mctx->msg = CLS_NAV;
                    // *(mctx->msg+1) = NAV_PVT;
                    break;
                case NAV_SAT:
#if LOG_MSG_BITS == 2 && CONFIG_UBLOX_LOG_LEVEL < 1
                    printf(">> NAV_SAT >>\n");
#endif
                    mctx->ubx_msg_type = MT_NAV_SAT;
                    mctx->msg = (uint8_t *)&msg->nav_sat;
                    mctx->msg_size = sizeof(struct nav_sat_s);
                    // *mctx->msg = CLS_NAV;
                    // *(mctx->msg+1) = NAV_PVT;
                    break;
                default:
#if LOG_MSG_BITS == 2 && CONFIG_UBLOX_LOG_LEVEL < 1
                    ESP_LOGW(TAG, "[%s] unknown NAV message type: %02x",__FUNCTION__ , *(mctx->msg + 1));
#endif
                    goto err;
                    //break;
            }
            break;
        case CLS_MON:
            switch (*(mctx->msg + 1)) {
                case MON_GNSS:
#if LOG_MSG_BITS == 2 && CONFIG_UBLOX_LOG_LEVEL < 1
                    printf(">> MON_GNSS >>\n");
#endif
                    mctx->ubx_msg_type = MT_MON_GNSS;
                    mctx->msg = (uint8_t *)&msg->monGNSS;
                    mctx->msg_size = sizeof(struct mon_gnss_s);
                    // *mctx->msg = CLS_NAV;
                    // *(mctx->msg+1) = NAV_PVT;
                    break;
                case MON_VER:
#if LOG_MSG_BITS == 2 && CONFIG_UBLOX_LOG_LEVEL < 1
                    printf(">> MON_VER >>\n");
#endif
                    mctx->ubx_msg_type = MT_MON_VER;
                    mctx->msg = (uint8_t *)&msg->mon_ver;
                    mctx->msg_size = sizeof(struct mon_ver_s);
                    // *mctx->msg = CLS_NAV;
                    // *(mctx->msg+1) = NAV_PVT;
                    break;
                default:
#if LOG_MSG_BITS == 2 && CONFIG_UBLOX_LOG_LEVEL < 1
                    ESP_LOGW(TAG, "[%s] unknown MON message type: %02x",__FUNCTION__ , *(mctx->msg + 1));
#endif
                    goto err;
                    //break;
            }
            break;
        case CLS_ACK:
            switch (*(mctx->msg + 1)) {
                case ACK_ACK:
#if LOG_MSG_BITS == 2 && CONFIG_UBLOX_LOG_LEVEL < 1
                    printf(">> ACK_ACK >>\n");
#endif
                    mctx->ubx_msg_type = MT_NAV_ACK;
                    mctx->msg = (uint8_t *)&msg->navAck;
                    mctx->msg_size = sizeof(struct nav_ack_s);
                    // *mctx->msg = CLS_NAV;
                    // *(mctx->msg+1) = NAV_PVT;
                    break;
                case ACK_NAK:
#if LOG_MSG_BITS == 2 && CONFIG_UBLOX_LOG_LEVEL < 1
                    printf(">> ACK_NAK >>\n");
#endif
                    mctx->ubx_msg_type = MT_NAV_NACK;
                    mctx->msg = (uint8_t *)&msg->navNack;
                    mctx->msg_size = sizeof(struct nav_nack_s);
                    // *mctx->msg = CLS_NAV;
                    // *(mctx->msg+1) = NAV_PVT;
                    break;
                default:
#if LOG_MSG_BITS == 2 && CONFIG_UBLOX_LOG_LEVEL < 1
                    ESP_LOGW(TAG, "[%s] unknown ACK message type: %02x", __FUNCTION__, *(mctx->msg + 1));
#endif
                    goto err;
                    //break;
            }
            break;
        case CLS_SEC:
            switch (*(mctx->msg + 1)) { // SEC_UBX  0x27
                case SEC_UBX:
#if LOG_MSG_BITS == 2 && CONFIG_UBLOX_LOG_LEVEL < 1
                    printf(">> SEC_UNIQID >>\n");
#endif
                    mctx->ubx_msg_type = MT_NAV_ID;
                    mctx->msg = (uint8_t *)&msg->ubxId;
                    mctx->msg_size = sizeof(struct nav_id_s);
                    // *mctx->msg = CLS_NAV;
                    // *(mctx->msg+1) = NAV_PVT;
                    break;
                default:
#if LOG_MSG_BITS == 2 && CONFIG_UBLOX_LOG_LEVEL < 1
                    ESP_LOGW(TAG, "[%s] unknown SEC message type: %02x", __FUNCTION__, *(mctx->msg + 1));
#endif
                    goto err;
                    //break;
            }
            break;
        default:
#if LOG_MSG_BITS == 2 && CONFIG_UBLOX_LOG_LEVEL < 1
            ESP_LOGW(TAG, "[%s] unknown message class: %02x", __FUNCTION__, *mctx->msg);
#endif
            goto err;
            //break;
    }
    mctx->msg_len = mctx->msg_size;
    return ESP_OK;
    err:
    ubx_msg_byte_ctx_reset(mctx);
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ubx_msg_byte_ctx_reset(ubx_msg_byte_ctx_t * mctx) {
    assert(mctx && mctx->ubx);
    if(mctx->ubx_msg_type!= MT_NONE)
        mctx->ubx_msg_type = MT_NONE;
    mctx->msg = &(mctx->ubx->ubx_msg.none[0]);
    mctx->msg_size = UBX_NONE_SIZE;
    for(uint8_t i=0; i<UBX_NONE_SIZE; ++i)
        *(mctx->msg+i) = 0;
    return ESP_OK;
}

esp_err_t msg_checksum_cb(ubx_msg_byte_ctx_t * mctx) {
    uint8_t * msg = mctx->msg;
    uint16_t size = mctx->msg_len ? mctx->msg_len : mctx->msg_size;
    uint8_t CK_A = 0, CK_B = 0;
    add_checksum(msg, size, &CK_A, &CK_B);
    if(CK_A == *(msg+size-2) && CK_B == *(msg+size-1)) {
        if(mctx->msg_len != mctx->msg_size && mctx->ubx_msg_type == MT_NAV_SAT) { // fix checksum fields for variable length message
            nav_sat_t * m = (nav_sat_t *)msg;
            m->chkA = CK_A;
            m->chkB = CK_B;
        }
        return ESP_OK;
    }
    else {
#if LOG_MSG_BITS == 1 && CONFIG_UBLOX_LOG_LEVEL < 2
        ESP_LOGE(TAG, "[%s] checksum failed cka:%02x ckb:%02x mcka:%02x mckb:%02x size:%"PRId16, __FUNCTION__, CK_A, CK_B, *(msg+size-2), *(msg+size-1), size);
#endif
        return ESP_ERR_INVALID_CRC;
    }
}

esp_err_t ubx_msg_checksum_handler(struct ubx_msg_byte_ctx_s * mctx) {
    esp_err_t ret = ESP_OK;
    if(!mctx->msg) {
#if LOG_MSG_BITS == 1 && CONFIG_UBLOX_LOG_LEVEL < 3
        ESP_LOGW(TAG, "msg is NULL, can not handle message.");
#endif
        return ESP_ERR_INVALID_ARG;
    }
    mctx->ubx->ubx_msg.count_msg++;
    ret = msg_checksum_cb(mctx);
    if(ret != ESP_OK) {
        mctx->ubx->ubx_msg.count_err++;
        if(mctx->ubx_msg_type == MT_NAV_PVT||mctx->ubx_msg_type == MT_NAV_SAT||mctx->ubx_msg_type== MT_NAV_DOP) {
            *(mctx->msg+4) = *(mctx->msg+5) = *(mctx->msg+6) = *(mctx->msg+7) = 0; // reset iTOW
#if CONFIG_UBLOX_LOG_LEVEL < 3
            ESP_LOGE(TAG,"[%s] fail, reset msg %hhu iTOW\n", __func__, mctx->ubx_msg_type);
#endif
        }
        mctx->ubx_msg_type = MT_NONE;
    }
    else {
        mctx->ubx->ubx_msg.count_ok++;
    }
    return ret;
}

esp_err_t ubx_msg_handler(ubx_msg_byte_ctx_t *mctx) {
    ubx_msg_byte_ctx_reset(mctx); // reset msg pointer and length to default
    esp_err_t err = read_ubx_msg(mctx);
    // if(err != ESP_OK) {
    //     ESP_LOGE(TAG, "[%s] read failed: %s", __FUNCTION__, esp_err_to_name(err));
    // }
    return err;
}


// #define MIN_numSV_FIRST_FIX 5      // alvorens start loggen, changed from 4 to 5 7.1/2023
// #define MAX_Sacc_FIRST_FIX 2       // alvorens start loggen
// #define MIN_numSV_GPS_SPEED_OK  4  // min aantal satellieten voor berekenen snelheid, anders
// #define MAX_Sacc_GPS_SPEED_OK  1   // max waarde Sacc voor berekenen snelheid, anders 0
// #define MAX_GPS_SPEED_OK  40       // max snelheid in m/s voor berekenen snelheid, anders 0
// #define MIN_SPEED_START_LOGGING 2000        //was 2000 min speed in mm/s over 2 s alvorens start loggen naar SD 
// #define TIME_DELAY_FIRST_FIX 10 //10 navpvt messages alvorens start loggen


int8_t ubx_set_time(ubx_config_t *ubx, float time_offset) {
    assert(ubx);
    nav_pvt_t *pvt = &ubx->ubx_msg.navPvt;
    if (!pvt->numSV || pvt->year < 2023) { // no sats or no valid time
        return 0;
    }
    uint32_t millis = get_millis();
    if (ubx->time_set && millis < ubx->next_time_sync) { // time already set and not time to sync
        return 0;
    }
    //ESP_LOGI(TAG, "[%s] sats: %" PRIu8, __FUNCTION__, pvt->numSV);
    struct tm my_time={0};      // time elements structure
    time_t unix_timestamp = 0;  // a timestamp in seconds
#if defined(DLS)
    // summertime is on march 26 2023 2 AM, see
    // https://www.di-mgt.com.au/wclock/help/wclo_tzexplain.html
    setenv("TZ", "CET0CEST,M3.5.0/2,M10.5.0/3", 1);  // timezone UTC = CET, Daylightsaving ON :
                                                     // TZ=CET-1CEST,M3.5.0/2,M10.5.0/3
    tzset();                                         // this works for CET, but TZ string is different for every Land /
                                                     // continent....
#else
    setenv("TZ", "UTC", 0);
    tzset();
#endif
    my_time.tm_sec = pvt->second;
    my_time.tm_hour = pvt->hour;
    my_time.tm_min = pvt->minute;
    my_time.tm_mday = pvt->day;
    my_time.tm_mon = pvt->month - 1;     // mktime needs months 0 - 11
    my_time.tm_year = pvt->year - 1900;  // mktime needs years since 1900, so deduct 1900
    unix_timestamp = mktime(&my_time);                // mktime returns local time, so TZ is important !!!
    int64_t utc_ms = unix_timestamp * 1000LL + (pvt->nano + 500000) / 1000000LL;
    ESP_LOGW(TAG, "GPS raw time: %d-%02d-%02d %02d:%02d:%02d %" PRId64, pvt->year, pvt->month, pvt->day, pvt->hour, pvt->minute, pvt->second, utc_ms);
    struct timeval tv = {
        .tv_sec = (time_t)(unix_timestamp + (time_offset * 3600)),
        .tv_usec = 0};  // clean utc time !!
    settimeofday(&tv, NULL);
    struct tm tms;
    localtime_r(&unix_timestamp, &tms);
    ESP_LOGW(TAG, "GPS time set: %d-%02d-%02d %02d:%02d:%02d", (tms.tm_year) + 1900, (tms.tm_mon) + 1, tms.tm_mday, tms.tm_hour, tms.tm_min, tms.tm_sec);
    if (tms.tm_year < 123) {
        ESP_LOGW(TAG, "GPS Reported year not plausible (<2023)!");
        return 0;
    }
    ESP_ERROR_CHECK(esp_event_post(UBX_EVENT, UBX_EVENT_DATETIME_SET, NULL,0, portMAX_DELAY));
    ubx->next_time_sync = millis + (60000); // 1 minute
    ubx->time_set = 1;
    return 1;
}

static const uint8_t ubx_msg_header[] = UBX_HDR;

esp_err_t read_ubx_msg(ubx_msg_byte_ctx_t * mctx) {
    assert(mctx);
    esp_err_t ret = ESP_OK;
    uint8_t got_header = 0;
    uint8_t data = 0;
    uint16_t i = 0, j = 0, timeout = MSG_READ_TIMEOUT, msg_len=mctx->msg_size;
    size_t len = 0;
    uint32_t then = get_millis(), elapsed = 0;
    ubx_config_t * ubx = mctx->ubx;
    assert(ubx);
    assert(mctx->msg);
    //xSemaphoreTake(xMutex, portMAX_DELAY);
    while (i < msg_len && elapsed < timeout) {
        j=0;
        ret = uart_get_buffered_data_len(ubx->uart_num, &len);
        if (ret)
            return ret;
#if LOG_MSG_BITS == 2
        if(len)
            printf(">>>>> len:%u >>>>>\n", len);
#endif
        while (j<len) {
            if (!uart_read_bytes(ubx->uart_num, &data, 1, 20 / portTICK_PERIOD_MS)) {
#if CONFIG_UBLOX_LOG_LEVEL < 2
                ESP_LOGW(TAG, "[%s] timeout, uart buffer full?", __FUNCTION__);
#endif
                return ESP_ERR_TIMEOUT;
            }
            if(got_header < 2 && mctx->expect_ubx_msg){
                if(data == ubx_msg_header[0]) { // check for UBX header
                    got_header = 1;
                }
                else if(data == ubx_msg_header[1] && got_header==1){
                    // reset i 
                    i = (*(mctx->msg+1) == ubx_msg_header[1] && *mctx->msg == ubx_msg_header[0]) ? 2 : 0; // make sure we start fill msg after UBX header
                    got_header = 2; // found UBX header
                }
                else {  
                    got_header = 0; // reset if not matching
                }
                goto next_byte;
            }
            else if(mctx->msg_match_to_pos && i<mctx->msg_pos && data != *(mctx->msg+i)) {
                //ESP_LOGI(TAG, "[%s] msg match to pos failed i:%u msg_pos:%"PRIu16" data:%02x j:%"PRIu16, __FUNCTION__, i, mctx->msg_pos, data, j);
                i = 0; // reset if not matching
                got_header = 0;
                goto next_byte;
            }
            else if(i==mctx->msg_pos && mctx->msg_type_handler) {
#if CONFIG_UBLOX_LOG_LEVEL < 1
                if(mctx->ubx_msg_type != MT_NONE) {
                    ESP_LOGW(TAG, "[%s] msg type already set to: 0x%02x, it seems that previous msg not finished...", __FUNCTION__, mctx->ubx_msg_type);
                }
#endif
                ret = mctx->msg_type_handler(mctx);
                if(ret != ESP_OK) { // set msg pointer and length point to right struct
#if CONFIG_UBLOX_LOG_LEVEL < 1
                    ESP_LOGW(TAG, "[%s] msg type handler failed", __FUNCTION__);
#endif
                    goto done;
                }
                else if(msg_len != mctx->msg_size) {
#if  LOG_MSG_BITS == 2 && CONFIG_UBLOX_LOG_LEVEL < 2
                    ESP_LOGI(TAG, "[%s] msg pointer changed from ubx, change also msg_len: %"PRIu16" to msg_size_%"PRIu16, __FUNCTION__, msg_len, mctx->msg_size);
#endif
                    msg_len = mctx->msg_size;
                }
            }
            if (data != *(mctx->msg+i)) {
#if LOG_MSG_BITS == 2 && CONFIG_UBLOX_LOG_LEVEL < 1
                if(i<=mctx->msg_pos)
                    printf("[%s] msg match to pos i:%u j:%"PRIu16" msg_pos:%"PRIu16" data:0x%02x msg before:0x%02x\n", __FUNCTION__, i, j, mctx->msg_pos, data, *(mctx->msg+i));
#endif
                *(mctx->msg+i) = data; // fill msg buffer
            }
            if(!mctx->msg_match_to_pos && got_header == 2 && i == 3) { // check if msg buffer is full
                decode_uint16(mctx->msg+2, &msg_len); // set msg length
#if LOG_MSG_BITS == 2 && CONFIG_UBLOX_LOG_LEVEL < 1
                printf("[%s] got pl_len:%"PRIu16" from ubx, mctx>msg_size:%"PRIu16", mctx>msg_len:%"PRIu16" ubxlen:0x%02x 0x%02x\n", __FUNCTION__, msg_len, mctx->msg_size, mctx->msg_len, *(mctx->msg+2), *(mctx->msg+3));
#endif
                msg_len += 6; // add 6 bytes for UBX header and checksum
                if(msg_len > mctx->msg_size) {
#if CONFIG_UBLOX_LOG_LEVEL < 1
                    ESP_LOGE(TAG, "[%s] msg size too big: msg_len:%u msg_size:%u", __FUNCTION__, msg_len, mctx->msg_size);
#endif
                }
                else if(msg_len != mctx->msg_size) {
                    mctx->msg_len = msg_len;
                }
            }
            ++i;
            if(i >= msg_len) // inner loop check if msg buffer is full
                goto done;
            next_byte:
            ++j;
        }
        elapsed = get_millis()-then;
    }
    done:
    if(mctx->msg_ready_handler) {
        ret = mctx->msg_ready_handler(mctx);
    }
#if CONFIG_UBLOX_LOG_LEVEL < 1
        print_ubx_msg(mctx);
#if LOG_MSG_BITS == 2
    ESP_LOGI(TAG, "[%s] done read len:%u bytes, i:%"PRIu16" of msg size: %u used, {cls:%02x, id:%02x}", __FUNCTION__, len, i, mctx->msg_size, *(mctx->msg), *(mctx->msg+1));
#endif
#endif
    //xSemaphoreGive(xMutex);
    if(ret == ESP_OK) {
        if ((elapsed) >= timeout) {// timeout
#if CONFIG_UBLOX_LOG_LEVEL < 2
            ESP_LOGW(TAG, "[%s] timeout, elapsed: %"PRIu32, __FUNCTION__, elapsed);
#endif
            ret = ESP_ERR_TIMEOUT;
        }
        else if(!*(mctx->msg+2)) {// no data
#if CONFIG_UBLOX_LOG_LEVEL < 1
            ESP_LOGW(TAG, "[%s] no data", __FUNCTION__);
#endif
            ret = ESP_ERR_INVALID_RESPONSE;
        }
    }
    return ret;

}

void print_ubx_msg(ubx_msg_byte_ctx_t * mctx) {

#if LOG_MSG_JSON == 1
    strbf_t msgbf;
    strbf_init(&msgbf);
    ubx_msg_serialize_json(mctx, &msgbf);
    printf("ubx_msg_json: %s\n", msgbf.start);
    strbf_free(&msgbf);
#endif

#if LOG_MSG_BITS == 1
    uint8_t * msg = mctx->msg;
    const char *m = "ubx_msg type: ", *n = ", msg: [ ";
    if(mctx->ubx_msg_type == MT_NAV_SAT) printf("%snav_sat%s", m, n);
    else if(mctx->ubx_msg_type == MT_NAV_PVT) printf("%snav_pvt%s", m, n);
    else if(mctx->ubx_msg_type == MT_NAV_DOP) printf("%snav_dop%s", m, n);
    else if(mctx->ubx_msg_type == MT_MON_GNSS) printf("%smon_gnss%s", m, n);
    else if(mctx->ubx_msg_type == MT_MON_VER) printf("%smon_ver%s", m, n);
    else if(mctx->ubx_msg_type == MT_NAV_ACK) printf("%snav_ack%s", m, n);
    else if(mctx->ubx_msg_type == MT_NAV_ID) printf("%snav_id%s", m, n);
    else goto done;
    uint16_t i=0, size = mctx->msg_len ? mctx->msg_len : mctx->msg_size;
    for(; i < size; ++i)
        printf("0x%02x ", *(msg+i));
    printf("] (%u)", size);
    if(mctx->msg_size > size){
        printf(" -> [ ");
        for(i=size; i < mctx->msg_size; ++i)
            printf("0x%02x ", *(msg+i));
        printf(" ] (%u)\n", mctx->msg_size);
    }
    else
        printf("\n");
    done:
#endif
}

esp_err_t ack_status(ubx_config_t *ubx, uint8_t cls_id, uint8_t msg_id) {
    DLOG(TAG, "[%s]", __func__);
    esp_err_t ret = ESP_OK;
    ubx->ubx_msg.navAck.msg_cls = cls_id;
    ubx->ubx_msg.navAck.msg_id = msg_id;
    ubx_msg_byte_ctx_t mctx = {
    .msg = (uint8_t*)&ubx->ubx_msg.navAck,
    .msg_size = sizeof(struct nav_ack_s),
    .msg_pos = 6,
    .msg_match_to_pos = true,
    .expect_ubx_msg = true,
    .ubx_msg_type = MT_NAV_ACK,
    .msg_ready_handler = msg_checksum_cb,
    .msg_type_handler = 0,
    .ubx = ubx,
    };
    ret = read_ubx_msg(&mctx);
    return ret;
}
