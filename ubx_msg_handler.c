
#include "ubx.h"
#include "ubx_events.h"
#include "ubx_msg.h"

#if defined(CONFIG_UBLOX_ENABLED)
#include <sys/time.h>
#include <stdint.h>

#include "esp_err.h"

#include "ubx_private.h"
#include "logger_common.h"

#define LOG_MSG_BITS 0
#define LOG_MSG_JSON 0

#define BITVAL(a, b) ((a >> b) & 1u)
#define BITS(a, b, c) ((a >> b) & ((1u << c) - 1u))
#define BITS_FROM_U8(source,lsb,msb) \
    ((uint8_t)((source) & ((uint8_t)(((uint8_t)(0xFFu >> ((uint8_t)(7u-((uint8_t)(msb) & 7u))))) & ((uint8_t)(0xFFu << ((uint8_t)(lsb) & 7u)))))))

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

esp_err_t ubx_msg_serialize_json(ubx_msg_byte_ctx_t * ubx_packet, strbf_t * msgbf) {
    assert(ubx_packet && ubx_packet->msg);
    switch(ubx_packet->ubx_msg_type) {
        case MT_NAV_PVT:
            return nav_pvt_serialize_json((nav_pvt_t *)ubx_packet->msg, msgbf);
        case MT_NAV_DOP:
           return nav_dop_serialize_json((nav_dop_t *)ubx_packet->msg, msgbf);
        case MT_NAV_SAT:
            return nav_sat_serialize_json((nav_sat_t *)ubx_packet->msg, msgbf);
        case MT_MON_GNSS:
            return mon_gnss_serialize_json((mon_gnss_t *)ubx_packet->msg, msgbf);
        case MT_MON_VER:
            return mon_ver_serialize_json((mon_ver_t *)ubx_packet->msg, msgbf);
        case MT_NAV_ACK:
            return nav_ack_serialize_json((nav_ack_t *)ubx_packet->msg, msgbf);
        // case MT_NAV_NACK:
        //     return nav_nack_serialize_json((nav_nack_t *)ubx_packet->msg, msgbf);
        case MT_NAV_ID:
            return nav_id_serialize_json((nav_id_t *)ubx_packet->msg, msgbf);
        default:
            ESP_LOGW(TAG, "[%s] unknown ubx message type: %u", __FUNCTION__, ubx_packet->ubx_msg_type);
            return ESP_ERR_NOT_SUPPORTED;
    };
    return ESP_OK;
}

#endif

esp_err_t ubx_msg_type_handler(struct ubx_msg_byte_ctx_s * ubx_packet) {
    assert(ubx_packet);
    if(!ubx_packet->msg) {
        ESP_LOGW(TAG, "msg is NULL, can not handle message.");
        return ESP_ERR_INVALID_ARG;
    }
    assert(ubx_packet->ubx_msg);
    ubx_msg_t *msg = ubx_packet->ubx_msg;
     switch (*ubx_packet->msg) {
        case CLS_NAV:
            switch (*(ubx_packet->msg + 1)) {
                case NAV_PVT:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL < 1)
                    printf(">> NAV_PVT >>\n");
#endif
                    ubx_packet->ubx_msg_type = MT_NAV_PVT;
                    ubx_packet->msg = (uint8_t *)&msg->navPvt;
                    ubx_packet->msg_size = sizeof(struct nav_pvt_s);
                    // *ubx_packet->msg = CLS_NAV;
                    // *(ubx_packet->msg+1) = NAV_PVT;
                    break;
                case NAV_DOP:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL < 1)
                    printf(">> NAV_DOP >>\n");
#endif
                    ubx_packet->ubx_msg_type = MT_NAV_DOP;
                    ubx_packet->msg = (uint8_t *)&msg->navDOP;
                    ubx_packet->msg_size = sizeof(struct nav_dop_s);
                    // *ubx_packet->msg = CLS_NAV;
                    // *(ubx_packet->msg+1) = NAV_PVT;
                    break;
                case NAV_SAT:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL < 1)
                    printf(">> NAV_SAT >>\n");
#endif
                    ubx_packet->ubx_msg_type = MT_NAV_SAT;
                    ubx_packet->msg = (uint8_t *)&msg->nav_sat;
                    ubx_packet->msg_size = sizeof(struct nav_sat_s);
                    // *ubx_packet->msg = CLS_NAV;
                    // *(ubx_packet->msg+1) = NAV_PVT;
                    break;
                default:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL < 1)
                    ESP_LOGW(TAG, "[%s] unknown NAV message type: %02x",__FUNCTION__ , *(ubx_packet->msg + 1));
#endif
                    goto err;
                    //break;
            }
            break;
        case CLS_MON:
            switch (*(ubx_packet->msg + 1)) {
                case MON_GNSS:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL < 1)
                    printf(">> MON_GNSS >>\n");
#endif
                    ubx_packet->ubx_msg_type = MT_MON_GNSS;
                    ubx_packet->msg = (uint8_t *)&msg->monGNSS;
                    ubx_packet->msg_size = sizeof(struct mon_gnss_s);
                    // *ubx_packet->msg = CLS_NAV;
                    // *(ubx_packet->msg+1) = NAV_PVT;
                    break;
                case MON_VER:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL < 1)
                    printf(">> MON_VER >>\n");
#endif
                    ubx_packet->ubx_msg_type = MT_MON_VER;
                    ubx_packet->msg = (uint8_t *)&msg->mon_ver;
                    ubx_packet->msg_size = sizeof(struct mon_ver_s);
                    // *ubx_packet->msg = CLS_NAV;
                    // *(ubx_packet->msg+1) = NAV_PVT;
                    break;
                default:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL < 1)
                    ESP_LOGW(TAG, "[%s] unknown MON message type: %02x",__FUNCTION__ , *(ubx_packet->msg + 1));
#endif
                    goto err;
                    //break;
            }
            break;
        case CLS_ACK:
            switch (*(ubx_packet->msg + 1)) {
                case ACK_ACK:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL < 1)
                    printf(">> ACK_ACK >>\n");
#endif
                    ubx_packet->ubx_msg_type = MT_NAV_ACK;
                    ubx_packet->msg = (uint8_t *)&msg->navAck;
                    ubx_packet->msg_size = sizeof(struct nav_ack_s);
                    // *ubx_packet->msg = CLS_NAV;
                    // *(ubx_packet->msg+1) = NAV_PVT;
                    break;
                case ACK_NAK:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL < 1)
                    printf(">> ACK_NAK >>\n");
#endif
                    ubx_packet->ubx_msg_type = MT_NAV_NACK;
                    ubx_packet->msg = (uint8_t *)&msg->navNack;
                    ubx_packet->msg_size = sizeof(struct nav_nack_s);
                    // *ubx_packet->msg = CLS_NAV;
                    // *(ubx_packet->msg+1) = NAV_PVT;
                    break;
                default:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL < 1)
                    ESP_LOGW(TAG, "[%s] unknown ACK message type: %02x", __FUNCTION__, *(ubx_packet->msg + 1));
#endif
                    goto err;
                    //break;
            }
            break;
        case CLS_SEC:
            switch (*(ubx_packet->msg + 1)) { // SEC_UBX  0x27
                case SEC_UNIQID:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL < 1)
                    printf(">> SEC_UNIQID >>\n");
#endif
                    ubx_packet->ubx_msg_type = MT_NAV_ID;
                    ubx_packet->msg = (uint8_t *)&msg->ubxId;
                    ubx_packet->msg_size = sizeof(struct nav_id_s);
                    // *ubx_packet->msg = CLS_NAV;
                    // *(ubx_packet->msg+1) = NAV_PVT;
                    break;
                default:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL < 1)
                    ESP_LOGW(TAG, "[%s] unknown SEC message type: %02x", __FUNCTION__, *(ubx_packet->msg + 1));
#endif
                    goto err;
                    //break;
            }
            break;
        default:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL < 1)
            ESP_LOGW(TAG, "[%s] unknown message class: %02x", __FUNCTION__, *ubx_packet->msg);
#endif
            goto err;
            //break;
    }
    ubx_packet->msg_len = ubx_packet->msg_size;
    return ESP_OK;
    err:
    ubx_msg_byte_ctx_reset(ubx_packet);
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ubx_msg_byte_ctx_reset(ubx_msg_byte_ctx_t * ubx_packet) {
    assert(ubx_packet);
    if(ubx_packet->ubx_msg_type!= MT_NONE)
        ubx_packet->ubx_msg_type = MT_NONE;
    ubx_packet->msg = &ubx_packet->ubx_msg->none[0];
    ubx_packet->msg_size = UBX_NONE_SIZE;
    for(uint8_t i=0; i<UBX_NONE_SIZE; ++i)
        *(ubx_packet->msg+i) = 0;
    return ESP_OK;
}

esp_err_t msg_checksum_cb(ubx_msg_byte_ctx_t * ubx_packet) {
    uint8_t * msg = ubx_packet->msg;
    uint16_t size = ubx_packet->msg_len ? ubx_packet->msg_len : ubx_packet->msg_size;
    uint8_t CK_A = 0, CK_B = 0;
    add_checksum(msg, size, &CK_A, &CK_B);
    if(CK_A == *(msg+size-2) && CK_B == *(msg+size-1)) {
        if(ubx_packet->msg_len != ubx_packet->msg_size && ubx_packet->ubx_msg_type == MT_NAV_SAT) { // fix checksum fields for variable length message
            nav_sat_t * m = (nav_sat_t *)msg;
            m->chkA = CK_A;
            m->chkB = CK_B;
        }
        return ESP_OK;
    }
    else {
#if LOG_MSG_BITS == 1 && (C_LOG_LEVEL < 2)
        ESP_LOGE(TAG, "[%s] checksum failed cka:%02x ckb:%02x mcka:%02x mckb:%02x size:%"PRId16, __FUNCTION__, CK_A, CK_B, *(msg+size-2), *(msg+size-1), size);
#endif
        return ESP_ERR_INVALID_CRC;
    }
}

esp_err_t ubx_msg_checksum_handler(struct ubx_msg_byte_ctx_s * ubx_packet) {
    esp_err_t ret = ESP_OK;
    if(!ubx_packet->msg) {
#if LOG_MSG_BITS == 1 && (C_LOG_LEVEL < 3)
        ESP_LOGW(TAG, "msg is NULL, can not handle message.");
#endif
        return ESP_ERR_INVALID_ARG;
    }
    ubx_packet->ubx_msg->count_msg++;
    ret = msg_checksum_cb(ubx_packet);
    if(ret != ESP_OK) {
        ubx_packet->ubx_msg->count_err++;
        if(ubx_packet->ubx_msg_type == MT_NAV_PVT||ubx_packet->ubx_msg_type == MT_NAV_SAT||ubx_packet->ubx_msg_type== MT_NAV_DOP) {
            *(ubx_packet->msg+4) = *(ubx_packet->msg+5) = *(ubx_packet->msg+6) = *(ubx_packet->msg+7) = 0; // reset iTOW
#if (C_LOG_LEVEL < 3)
            ESP_LOGE(TAG,"[%s] fail, reset msg %hhu iTOW\n", __func__, ubx_packet->ubx_msg_type);
#endif
        }
        ubx_packet->ubx_msg_type = MT_NONE;
    }
    else {
        ubx_packet->ubx_msg->count_ok++;
    }
    return ret;
}

esp_err_t ubx_msg_handler(ubx_config_t *ubx_dev, ubx_msg_byte_ctx_t *ubx_packet) {
    ubx_msg_byte_ctx_reset(ubx_packet); // reset msg pointer and length to default
    esp_err_t err = read_ubx_msg(ubx_dev, ubx_packet);
    // if(err != ESP_OK) {
    //     ESP_LOGE(TAG, "[%s] read failed: %s", __FUNCTION__, esp_err_to_name(err));
    // }
    return err;
}

static const uint8_t ubx_msg_header[] = UBX_HDR;

esp_err_t read_ubx_msg(ubx_config_t *ubx_dev, ubx_msg_byte_ctx_t * ubx_packet) {
    assert(ubx_packet);
    esp_err_t ret = ESP_OK;
    uint8_t got_header = 0;
    uint8_t data = 0;
    uint16_t i = 0, j = 0, timeout = MSG_READ_TIMEOUT, msg_len=ubx_packet->msg_size;
    size_t len = 0;
    uint32_t then = get_millis(), elapsed = 0;
    assert(ubx_dev);
    assert(ubx_packet->msg);
    //xSemaphoreTake(xMutex, portMAX_DELAY);
    while (i < msg_len && elapsed < timeout) {
        j=0;
        ret = uart_get_buffered_data_len(ubx_dev->uart_num, &len);
        if (ret)
            return ret;
// #if LOG_MSG_BITS == 2
//         if(len)
//             printf(">>>>> len:%u >>>>>\n", len);
// #endif
        while (j<len) {
            if (!uart_read_bytes(ubx_dev->uart_num, &data, 1, 20 / portTICK_PERIOD_MS)) {
#if (C_LOG_LEVEL < 2)
                ESP_LOGW(TAG, "[%s] timeout, uart buffer full?", __FUNCTION__);
#endif
                return ESP_ERR_TIMEOUT;
            }
            if(got_header < 2 && ubx_packet->expect_ubx_msg){
                if(data == ubx_msg_header[0]) { // check for UBX header
                    got_header = 1;
                }
                else if(data == ubx_msg_header[1] && got_header==1){
                    // reset i 
                    i = (*(ubx_packet->msg+1) == ubx_msg_header[1] && *ubx_packet->msg == ubx_msg_header[0]) ? 2 : 0; // make sure we start fill msg after UBX header
                    got_header = 2; // found UBX header
                }
                else {  
                    got_header = 0; // reset if not matching
                }
                goto next_byte;
            }
            else if(ubx_packet->msg_match_to_pos && i<ubx_packet->msg_pos && data != *(ubx_packet->msg+i)) {
                //ESP_LOGI(TAG, "[%s] msg match to pos failed i:%u msg_pos:%"PRIu16" data:%02x j:%"PRIu16, __FUNCTION__, i, ubx_packet->msg_pos, data, j);
                i = 0; // reset if not matching
                got_header = 0;
                goto next_byte;
            }
            else if(i==ubx_packet->msg_pos && ubx_packet->msg_type_handler) {
#if (C_LOG_LEVEL < 1)
                if(ubx_packet->ubx_msg_type != MT_NONE) {
                    ESP_LOGW(TAG, "[%s] msg type already set to: 0x%02x, it seems that previous msg not finished...", __FUNCTION__, ubx_packet->ubx_msg_type);
                }
#endif
                ret = ubx_packet->msg_type_handler(ubx_packet);
                if(ret != ESP_OK) { // set msg pointer and length point to right struct
#if (C_LOG_LEVEL < 1)
                    ESP_LOGW(TAG, "[%s] msg type handler failed", __FUNCTION__);
#endif
                    goto done;
                }
                else if(msg_len != ubx_packet->msg_size) {
#if  LOG_MSG_BITS == 2 && (C_LOG_LEVEL < 1)
                    ESP_LOGI(TAG, "[%s] msg pointer changed from ubx, change also msg_len: %"PRIu16" to msg_size_%"PRIu16, __FUNCTION__, msg_len, ubx_packet->msg_size);
#endif
                    msg_len = ubx_packet->msg_size;
                }
            }
            if (data != *(ubx_packet->msg+i)) {
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL < 1)
                if(i<=ubx_packet->msg_pos)
                    printf("[%s] msg match to pos i:%u j:%"PRIu16" msg_pos:%"PRIu16" data:0x%02x msg before:0x%02x\n", __FUNCTION__, i, j, ubx_packet->msg_pos, data, *(ubx_packet->msg+i));
#endif
                *(ubx_packet->msg+i) = data; // fill msg buffer
            }
            if(!ubx_packet->msg_match_to_pos && got_header == 2 && i == 3) { // check if msg buffer is full
                decode_uint16(ubx_packet->msg+2, &msg_len); // set msg length
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL < 1)
                printf("[%s] got pl_len:%"PRIu16" from ubx, ubx_packet>msg_size:%"PRIu16", ubx_packet>msg_len:%"PRIu16" ubxlen:0x%02x 0x%02x\n", __FUNCTION__, msg_len, ubx_packet->msg_size, ubx_packet->msg_len, *(ubx_packet->msg+2), *(ubx_packet->msg+3));
#endif
                msg_len += 6; // add 6 bytes for UBX header and checksum
                if(msg_len > ubx_packet->msg_size) {
#if (C_LOG_LEVEL < 1)
                    ESP_LOGE(TAG, "[%s] msg size too big: msg_len:%u msg_size:%u", __FUNCTION__, msg_len, ubx_packet->msg_size);
#endif
                }
                else if(msg_len != ubx_packet->msg_size) {
                    ubx_packet->msg_len = msg_len;
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
    if(ubx_packet->msg_ready_handler) {
        ret = ubx_packet->msg_ready_handler(ubx_packet);
    }
#if (C_LOG_LEVEL < 1)
        print_ubx_msg(ubx_packet);
#if LOG_MSG_BITS == 2
    ESP_LOGI(TAG, "[%s] done read len:%u bytes, i:%"PRIu16" of msg size: %u used, {cls:%02x, id:%02x}", __FUNCTION__, len, i, ubx_packet->msg_size, *(ubx_packet->msg), *(ubx_packet->msg+1));
#endif
#endif
    //xSemaphoreGive(xMutex);
    if(ret == ESP_OK) {
        if ((elapsed) >= timeout) {// timeout
#if (C_LOG_LEVEL < 2)
            ESP_LOGW(TAG, "[%s] timeout, elapsed: %"PRIu32, __FUNCTION__, elapsed);
#endif
            ret = ESP_ERR_TIMEOUT;
        }
        else if(!*(ubx_packet->msg+2)) {// no data
#if (C_LOG_LEVEL < 1)
            ESP_LOGW(TAG, "[%s] no data", __FUNCTION__);
#endif
            ret = ESP_ERR_INVALID_RESPONSE;
        }
    }
    return ret;

}

void print_ubx_msg(ubx_msg_byte_ctx_t * ubx_packet) {

#if LOG_MSG_JSON == 1
    strbf_t msgbf;
    strbf_init(&msgbf);
    ubx_msg_serialize_json(ubx_packet, &msgbf);
    printf("ubx_msg_json: %s\n", msgbf.start);
    strbf_free(&msgbf);
#endif

#if LOG_MSG_BITS == 1
    uint8_t * msg = ubx_packet->msg;
    const char *m = "ubx_msg type: ", *n = ", msg: [ ";
    if(ubx_packet->ubx_msg_type == MT_NAV_SAT) printf("%snav_sat%s", m, n);
    else if(ubx_packet->ubx_msg_type == MT_NAV_PVT) printf("%snav_pvt%s", m, n);
    else if(ubx_packet->ubx_msg_type == MT_NAV_DOP) printf("%snav_dop%s", m, n);
    else if(ubx_packet->ubx_msg_type == MT_MON_GNSS) printf("%smon_gnss%s", m, n);
    else if(ubx_packet->ubx_msg_type == MT_MON_VER) printf("%smon_ver%s", m, n);
    else if(ubx_packet->ubx_msg_type == MT_NAV_ACK) printf("%snav_ack%s", m, n);
    else if(ubx_packet->ubx_msg_type == MT_NAV_ID) printf("%snav_id%s", m, n);
    else goto done;
    uint16_t i=0, size = ubx_packet->msg_len ? ubx_packet->msg_len : ubx_packet->msg_size;
    for(; i < size; ++i)
        printf("0x%02x ", *(msg+i));
    printf("] (%u)", size);
    if(ubx_packet->msg_size > size){
        printf(" -> [ ");
        for(i=size; i < ubx_packet->msg_size; ++i)
            printf("0x%02x ", *(msg+i));
        printf(" ] (%u)\n", ubx_packet->msg_size);
    }
    else
        printf("\n");
    done:
#endif
}

esp_err_t ack_status(ubx_config_t *ubx_dev, uint8_t cls_id, uint8_t msg_id) {
#if C_LOG_LEVEL < 1
    DLOG(TAG, "[%s]", __func__);
#endif
    esp_err_t ret = ESP_OK;
    ubx_dev->ubx_msg.navAck.msg_cls = cls_id;
    ubx_dev->ubx_msg.navAck.msg_id = msg_id;
    ubx_msg_byte_ctx_t ubx_packet = {
    .msg = (uint8_t*)&ubx_dev->ubx_msg.navAck,
    .msg_size = sizeof(struct nav_ack_s),
    .msg_pos = 6,
    .msg_match_to_pos = true,
    .expect_ubx_msg = true,
    .ubx_msg_type = MT_NAV_ACK,
    .msg_ready_handler = msg_checksum_cb,
    .msg_type_handler = 0,
    .ubx_msg = &ubx_dev->ubx_msg,
    };
    ret = read_ubx_msg(ubx_dev, &ubx_packet);
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

esp_err_t write_ubx_msg(int uart_num, uint8_t *msg, size_t size, bool need_checksum) {
    ILOG(TAG, "[%s]", __func__);
    esp_err_t ret = ESP_OK;
    if(need_checksum)
        add_checksum(msg, size, msg + size - 2, msg + size - 1);
#if C_LOG_LEVEL < 1
    DLOG(TAG, "[%s]: [ ", __func__);
#endif
    for(uint16_t i=0; i < size; ++i){ // write the message byte by byte
#if C_LOG_LEVEL < 1
        DLOG(TAG, "0x%01x ", *(msg+i));
#endif
        if(uart_write_bytes(uart_num, msg+i, 1) != 1)
            ret = ESP_FAIL;
    }
#if C_LOG_LEVEL < 1
    DLOG(TAG, "] (%u)", size);
#endif
    return ret;
}

static esp_err_t ubx_cfg_send_m(ubx_config_t *ubx_dev, uint8_t * msg, size_t msg_len, bool need_ack) {
    ILOG(TAG, "[%s]", __func__);
    IMEAS_START();
    esp_err_t ret = ESP_OK;
    if (xSemaphoreTake(ubx_dev->xMutex, portMAX_DELAY) == pdTRUE) {
        ret = write_ubx_msg(ubx_dev->uart_num, msg, msg_len, true);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] write_ubx_msg failed: %s", __FUNCTION__, esp_err_to_name(ret));
            goto done;
        }
        delay_ms(100);
        if(need_ack) 
            ret = ack_status(ubx_dev, *(msg+2), *(msg+3));
    done:
        xSemaphoreGive(ubx_dev->xMutex);
    }
    IMEAS_END(TAG, "[%s] took %llu", __func__);
    return ret;
}

esp_err_t send_ubx_cfg_msg(ubx_config_t *ubx_dev, uint8_t cls, uint8_t id, const uint8_t * payload, size_t len, bool need_ack) {
    const uint8_t msgb[] = {UBX_HDR_A, UBX_HDR_B, cls, id, 0x00, 0x00, 0x00, 0x00};
    uint8_t *msg = 0;
    size_t msgb_len = sizeof(msgb), total_len = msgb_len + len;
    if(len){
        msg = calloc(total_len, sizeof(uint8_t));
        memcpy(msg, &(msgb[0]), 6); // copy header and class
        encode_uint16(msg+4, len); // add payload size
        memcpy(msg+6, payload, len); // copy payload
    }
    else
        msg = (uint8_t*)&(msgb[0]); // no payload
    assert(msg);
    esp_err_t ret = ubx_cfg_send_m(ubx_dev, msg, total_len, need_ack);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] failed: %s", __FUNCTION__, esp_err_to_name(ret));
    }
    if(len)
        free(msg);
    return ret;
}

esp_err_t ubx_cfg_valset(ubx_config_t *ubx_dev, const uint8_t * payload, size_t len, bool need_ack) {
    ILOG(TAG, "[%s]", __func__);
    if(ubx_dev->rtc_conf->hw_type < UBX_TYPE_M9)
        return ESP_ERR_INVALID_ARG;
    uint8_t *msg = calloc(len+4, sizeof(uint8_t));
    memcpy(msg, (const uint8_t[]){0x01, 0x01, 0x00, 0x00}, 4);
    memcpy(msg+4, payload, len);
    esp_err_t ret = send_ubx_cfg_msg(ubx_dev, CLS_CFG, CFG_VALSET, msg, len + 4, need_ack);
    free(msg);
    return ret;
}

esp_err_t ubx_cfg_get(ubx_config_t *ubx_dev, ubx_msg_byte_ctx_t * ubx_packet) {
    IMEAS_START();
    assert(ubx_packet && ubx_dev);
    esp_err_t ret = send_ubx_cfg_msg(ubx_dev, *ubx_packet->msg, *(ubx_packet->msg+1), NULL, 0, false);
    if(xSemaphoreTake(ubx_dev->xMutex, portMAX_DELAY)) {
        ret = read_ubx_msg(ubx_dev, ubx_packet); // this msg is without ubx header as ubx_msg_t parts start with class and id
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] read_ubx_msg failed: %s", __FUNCTION__, esp_err_to_name(ret));
        }
        xSemaphoreGive(ubx_dev->xMutex);
    }
    IMEAS_END(TAG, "[%s] took %llu", __func__);
    return ret;
}


#endif
