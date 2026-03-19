
#include "ubx.h"
// #include "ubx_events.h"
#include "ubx_msg.h"

#if defined(CONFIG_UBLOX_ENABLED)
#include <sys/time.h>
#include <stdint.h>
#include <string.h>
#include <esp_heap_caps.h>

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

#if defined(CONFIG_UBX_TIMER_STATS_ENABLED)
// UBX message statistics (module-private)
static ubx_msg_stats_t cur_msg_stats = {0};
static ubx_msg_stats_t prev_msg_stats = {0};
static ubx_msg_stats_t period_msg_stats = {0};
#endif

#if defined(CONFIG_UBX_TIMER_STATS_ENABLED)
// UART-level statistics: Track ALL UBX headers received from UART (before protocol decoding)
static ubx_msg_stats_t uart_rx_stats = {0};
static ubx_msg_stats_t uart_prev_stats = {0};
#endif


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
            WLOG(TAG, "[%s] unknown ubx message type: %u", __FUNCTION__, ubx_packet->ubx_msg_type);
            return ESP_ERR_NOT_SUPPORTED;
    };
    return ESP_OK;
}

#endif

esp_err_t ubx_msg_type_handler(struct ubx_msg_byte_ctx_s * ubx_packet) {
    // assert(ubx_packet);
    if(!ubx_packet->msg) {
        WLOG(TAG, "msg is NULL, can not handle message.");
        return ESP_ERR_INVALID_ARG;
    }
    // assert(ubx_packet->ubx_msg);
    ubx_msg_t *msg = ubx_packet->ubx_msg;
     switch (*ubx_packet->msg) {
        case CLS_NAV:
            switch (*(ubx_packet->msg + 1)) {
                case NAV_PVT:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL == LOG_TRACE_NUM)
                    TLOG(TAG, ">> NAV_PVT >>");
#endif
                    ubx_packet->ubx_msg_type = MT_NAV_PVT;
                    ubx_packet->msg = (uint8_t *)&msg->navPvt;
                    ubx_packet->msg_size = sizeof(struct nav_pvt_s);
                    break;
                case NAV_DOP:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL == LOG_TRACE_NUM)
                    TLOG(TAG, ">> NAV_DOP >>");
#endif
                    ubx_packet->ubx_msg_type = MT_NAV_DOP;
                    ubx_packet->msg = (uint8_t *)&msg->navDOP;
                    ubx_packet->msg_size = sizeof(struct nav_dop_s);
                    break;
                case NAV_SAT:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL == LOG_TRACE_NUM)
                    TLOG(TAG, ">> NAV_SAT >>\n");
#endif
                    ubx_packet->ubx_msg_type = MT_NAV_SAT;
                    ubx_packet->msg = (uint8_t *)&msg->nav_sat;
                    ubx_packet->msg_size = sizeof(struct nav_sat_s);
                    break;
                default:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL == LOG_TRACE_NUM)
                    WLOG(TAG, "[%s] unknown NAV message type: %02x",__FUNCTION__ , *(ubx_packet->msg + 1));
#endif
                    goto err;
                    //break;
            }
            break;
        case CLS_MON:
            switch (*(ubx_packet->msg + 1)) {
                case MON_GNSS:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL == LOG_TRACE_NUM)
                    TLOG(TAG, ">> MON_GNSS >>\n");
#endif
                    ubx_packet->ubx_msg_type = MT_MON_GNSS;
                    ubx_packet->msg = (uint8_t *)&msg->monGNSS;
                    ubx_packet->msg_size = sizeof(struct mon_gnss_s);
                    break;
                case MON_VER:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL == LOG_TRACE_NUM)
                    TLOG(TAG, ">> MON_VER >>\n");
#endif
                    ubx_packet->ubx_msg_type = MT_MON_VER;
                    ubx_packet->msg = (uint8_t *)&msg->mon_ver;
                    ubx_packet->msg_size = sizeof(struct mon_ver_s);
                    break;
                default:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL == LOG_TRACE_NUM)
                    WLOG(TAG, "[%s] unknown MON message type: %02x",__FUNCTION__ , *(ubx_packet->msg + 1));
#endif
                    goto err;
                    //break;
            }
            break;
        case CLS_ACK:
            switch (*(ubx_packet->msg + 1)) {
                case ACK_ACK:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL == LOG_TRACE_NUM)
                    TLOG(TAG, ">> ACK_ACK >>\n");
#endif
                    ubx_packet->ubx_msg_type = MT_NAV_ACK;
                    ubx_packet->msg = (uint8_t *)&msg->navAck;
                    ubx_packet->msg_size = sizeof(struct nav_ack_s);
                    break;
                case ACK_NAK:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL == LOG_TRACE_NUM)
                    TLOG(TAG, ">> ACK_NAK >>\n");
#endif
                    ubx_packet->ubx_msg_type = MT_NAV_NACK;
                    ubx_packet->msg = (uint8_t *)&msg->navNack;
                    ubx_packet->msg_size = sizeof(struct nav_nack_s);
                    break;
                default:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL == LOG_TRACE_NUM)
                    WLOG(TAG, "[%s] unknown ACK message type: %02x", __FUNCTION__, *(ubx_packet->msg + 1));
#endif
                    goto err;
                    //break;
            }
            break;
        case CLS_SEC:
            switch (*(ubx_packet->msg + 1)) { // SEC_UBX  0x27
                case SEC_UNIQID:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL == LOG_TRACE_NUM)
                    printf(">> SEC_UNIQID >>\n");
#endif
                    ubx_packet->ubx_msg_type = MT_NAV_ID;
                    ubx_packet->msg = (uint8_t *)&msg->ubxId;
                    ubx_packet->msg_size = sizeof(struct nav_id_s);
                    break;
                default:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL == LOG_TRACE_NUM)
                    WLOG(TAG, "[%s] unknown SEC message type: %02x", __FUNCTION__, *(ubx_packet->msg + 1));
#endif
                    goto err;
                    //break;
            }
            break;
        default:
#if LOG_MSG_BITS == 2 && (C_LOG_LEVEL == LOG_TRACE_NUM)
            WLOG(TAG, "[%s] unknown message class: %02x", __FUNCTION__, *ubx_packet->msg);
#endif
            goto err;
            //break;
    }
    /* Preserve actual length set by reader (needed for variable-length frames like NAV-SAT). */
    if (ubx_packet->msg_len == 0) {
        ubx_packet->msg_len = ubx_packet->msg_size;
    } else if (ubx_packet->msg_len > ubx_packet->msg_size) {
        ubx_packet->msg_len = ubx_packet->msg_size;
    }
    return ESP_OK;
    err:
    ubx_msg_byte_ctx_reset(ubx_packet);
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ubx_msg_byte_ctx_reset(ubx_msg_byte_ctx_t * ubx_packet) {
    // assert(ubx_packet);
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
#if LOG_MSG_BITS == 1 && (C_LOG_LEVEL <= LOG_DEBUG_NUM)
        ELOG(TAG, "[%s] checksum failed cka:%02x ckb:%02x mcka:%02x mckb:%02x size:%"PRId16, __FUNCTION__, CK_A, CK_B, *(msg+size-2), *(msg+size-1), size);
#endif
        return ESP_ERR_INVALID_CRC;
    }
}

#if (C_LOG_LEVEL <= LOG_INFO_NUM)
const char * const ubx_msg_type_strings[] = { UBX_MSG_TYPE_LIST(STRINGIFY_V) };
#endif

esp_err_t ubx_msg_checksum_handler(struct ubx_msg_byte_ctx_s * ubx_packet) {
    esp_err_t ret = ESP_OK;
    if(!ubx_packet->msg) {
#if LOG_MSG_BITS == 1 && (C_LOG_LEVEL <= LOG_INFO_NUM)
        WLOG(TAG, "msg is NULL, can not handle message.");
#endif
        return ESP_ERR_INVALID_ARG;
    }
#if defined(CONFIG_UBX_TIMER_STATS_ENABLED)
    cur_msg_stats.count++;
#endif
    ret = msg_checksum_cb(ubx_packet);
    if(ret != ESP_OK) {
#if defined(CONFIG_UBX_TIMER_STATS_ENABLED)
        cur_msg_stats.count_err++;
#endif
        if(ubx_packet->ubx_msg_type == MT_NAV_PVT||ubx_packet->ubx_msg_type == MT_NAV_SAT||ubx_packet->ubx_msg_type== MT_NAV_DOP) {
            *(ubx_packet->msg+4) = *(ubx_packet->msg+5) = *(ubx_packet->msg+6) = *(ubx_packet->msg+7) = 0; // reset iTOW
#if (C_LOG_LEVEL <= LOG_INFO_NUM)
            ELOG(TAG,"[%s] fail, reset msg %s iTOW", __func__,  ubx_msg_type_strings[ubx_packet->ubx_msg_type]);
#endif
        }
        ubx_packet->ubx_msg_type = MT_NONE;
    }
    else {
#if defined(CONFIG_UBX_TIMER_STATS_ENABLED)
        cur_msg_stats.count_ok++;
        // Track message types for comparison with GPS processing
        switch(ubx_packet->ubx_msg_type) {
            case MT_NAV_PVT: cur_msg_stats.count_nav_pvt++; break;
            case MT_NAV_SAT: cur_msg_stats.count_nav_sat++; break;
            case MT_NAV_DOP: cur_msg_stats.count_nav_dop++; break;
            default: break;
        }
#endif
        if (ubx_packet->ctx) {
            ubx_packet->ctx->last_valid_ms = get_millis();
            if (ubx_packet->ctx->link_lost) {
                ubx_packet->ctx->link_lost = false;
            }
        }
    }
    return ret;
}

esp_err_t ubx_msg_handler(ubx_ctx_t *ubx_dev, ubx_msg_byte_ctx_t *ubx_packet) {
    ubx_msg_byte_ctx_reset(ubx_packet); // reset msg pointer and length to default
    esp_err_t err = read_ubx_msg(ubx_dev, ubx_packet);
    // if(err != ESP_OK) {
    //     ELOG(TAG, "[%s] read failed: %s", __FUNCTION__, esp_err_to_name(err));
    // }
    return err;
}

static const uint8_t ubx_msg_header[] = UBX_HDR;

// Read exactly len bytes before deadline_ms; returns ESP_ERR_TIMEOUT on short read
static esp_err_t _uart_read_exact(ubx_ctx_t *ubx_dev, uint8_t *dst, size_t len, uint32_t deadline_ms) {
    uint32_t now = get_millis();
    if (now >= deadline_ms) {
        return ESP_ERR_TIMEOUT;
    }
    uint32_t remain_ms = deadline_ms - now;

    // Use event-driven buffer read
    size_t got = ubx_rx_buf_read(ubx_dev, dst, len, remain_ms);
    if (got < len) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

// Read one UBX frame from the RX buffer; does resync and copies cls/id/len into msg buffer.
static esp_err_t ubx_read_frame(ubx_ctx_t *ubx_dev, ubx_msg_byte_ctx_t *ubx_packet, uint32_t deadline) {
    esp_err_t ret = ESP_OK;

    // 1) Read UBX header with resync: skip non-UBX bytes until we see 0xB5 0x62 or time out.
    uint8_t header[2] = {0};
    while (true) {
        if (get_millis() >= deadline) {
            return ESP_ERR_TIMEOUT;
        }

        size_t got = ubx_rx_buf_read(ubx_dev, &header[0], 1, deadline - get_millis());
        if (got == 0) {
            return ESP_ERR_TIMEOUT;
        }
        if (header[0] != ubx_msg_header[0]) {
            continue;
        }

        if (get_millis() >= deadline) {
            return ESP_ERR_TIMEOUT;
        }
        got = ubx_rx_buf_read(ubx_dev, &header[1], 1, deadline - get_millis());
        if (got == 0) {
            return ESP_ERR_TIMEOUT;
        }
        if (header[1] != ubx_msg_header[1]) {
            continue;
        }
        break;
    }

    // 2) Read class, id, length (little-endian payload length)
    uint8_t fixed[4] = {0};
    ret = _uart_read_exact(ubx_dev, fixed, sizeof(fixed), deadline);
    if (ret != ESP_OK) {
        return ret;
    }

#if defined(CONFIG_UBX_TIMER_STATS_ENABLED)
    // UART layer: count frame consumed from buffer (before checksum validation)
    uart_rx_stats.count++;
    // Track message types from class/id bytes
    uint8_t cls = fixed[0];
    uint8_t id = fixed[1];
    if (cls == 0x01) {  // NAV class
        if (id == 0x07) uart_rx_stats.count_nav_pvt++;       // NAV-PVT
        else if (id == 0x35) uart_rx_stats.count_nav_sat++;  // NAV-SAT
        else if (id == 0x04) uart_rx_stats.count_nav_dop++;  // NAV-DOP
    }
#endif

    uint16_t payload_len = 0;
    decode_uint16(&fixed[2], &payload_len);
    uint16_t total_len = payload_len + 6; // cls/id/len + payload + ckA/ckB (no UBX header)

    // 3) Prepare buffer and possibly switch message target via handler
    ubx_packet->msg_len = total_len;
    uint8_t *msg = ubx_packet->msg;

    if (ubx_packet->msg_size >= 4) {
        msg[0] = fixed[0];
        msg[1] = fixed[1];
        msg[2] = fixed[2];
        msg[3] = fixed[3];
    }

    if (ubx_packet->msg_type_handler) {
        ret = ubx_packet->msg_type_handler(ubx_packet);
        if (ret != ESP_OK) {
            return ret;
        }
        msg = ubx_packet->msg;
        size_t copy_bytes = ubx_packet->msg_size < 4 ? ubx_packet->msg_size : 4;
        if (copy_bytes) {
            if (copy_bytes > 0) msg[0] = fixed[0];
            if (copy_bytes > 1) msg[1] = fixed[1];
            if (copy_bytes > 2) msg[2] = fixed[2];
            if (copy_bytes > 3) msg[3] = fixed[3];
        }
    }

    if (total_len > ubx_packet->msg_size) {
        FUNC_ENTRY_ARGSD(TAG, "msg size too big: msg_len:%u msg_size:%u", total_len, ubx_packet->msg_size);
    }

    // 4) Read remaining payload + checksum; copy into msg when space allows, discard overflow
    uint16_t offset = 4;
    uint16_t remaining = total_len > offset ? total_len - offset : 0;
    uint8_t chunk[64];
    while (remaining > 0) {
        uint16_t chunk_len = remaining > sizeof(chunk) ? sizeof(chunk) : remaining;
        ret = _uart_read_exact(ubx_dev, chunk, chunk_len, deadline);
        if (ret != ESP_OK) {
            return ret;
        }
        if (offset < ubx_packet->msg_size) {
            uint16_t writable = ubx_packet->msg_size - offset;
            uint16_t to_copy = chunk_len < writable ? chunk_len : writable;
            memcpy(msg + offset, chunk, to_copy);
        }
        offset += chunk_len;
        remaining -= chunk_len;
    }

    return ESP_OK;
}

esp_err_t read_ubx_msg(ubx_ctx_t *ubx_dev, ubx_msg_byte_ctx_t * ubx_packet) {
    if(!ubx_packet) return ESP_ERR_INVALID_ARG;
    if(!ubx_dev) return ESP_ERR_INVALID_ARG;
    if(!ubx_packet->msg) return ESP_ERR_INVALID_ARG;

    const uint16_t timeout = MSG_READ_TIMEOUT;
    const uint32_t deadline = get_millis() + timeout;

    // Sniff mode: when expect_ubx_msg==false we just harvest whatever is in the UART RX buffer
    // without trying to parse UBX framing. This is needed during initial autobaud when the
    // module may still output only NMEA; attempting to parse UBX length fields on NMEA bytes
    // would produce bogus sizes and timeouts.
    if (!ubx_packet->expect_ubx_msg) {
        size_t got = ubx_rx_buf_read(ubx_dev, ubx_packet->msg, ubx_packet->msg_size, timeout);
        ubx_packet->msg_len = (uint16_t)got;
        return got ? ESP_OK : ESP_ERR_TIMEOUT;
    }

    while (true) {
        esp_err_t ret = ubx_read_frame(ubx_dev, ubx_packet, deadline);
        if (ret != ESP_OK) {
            return ret;
        }

        if (ubx_packet->msg_ready_handler) {
            ret = ubx_packet->msg_ready_handler(ubx_packet);
            if (ret == ESP_ERR_INVALID_CRC || ret == ESP_ERR_NOT_SUPPORTED) {
                continue; // Try to resync to the next frame within the same deadline
            }
        }

#if (C_LOG_LEVEL == LOG_TRACE_NUM)
        print_ubx_msg(ubx_packet);
#endif

        return ret;
    }
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

esp_err_t ack_status(ubx_ctx_t *ubx_dev, uint8_t cls_id, uint8_t msg_id) {
    FUNC_ENTRYT(TAG);
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
    FUNC_ENTRYD(TAG);
    if(need_checksum)
        add_checksum(msg, size, msg + size - 2, msg + size - 1);
#if C_LOG_LEVEL == LOG_TRACE_NUM
    FUNC_ENTRY_ARGSD(TAG, "[ ");
    for(uint16_t i=0; i < size; ++i)
        printf("0x%01x ", *(msg+i));
    printf("] (%u)", size);
#endif
    // Write entire message buffer at once for better I/O performance
    int written = uart_write_bytes(uart_num, msg, size);
    return (written == (int)size) ? ESP_OK : ESP_FAIL;
}

static esp_err_t ubx_cfg_send_m(ubx_ctx_t *ubx_dev, uint8_t * msg, size_t msg_len, bool need_ack) {
    FUNC_ENTRYD(TAG);
    DMEAS_START();
    esp_err_t ret = ESP_OK;
    if (ubx_lock(500)) {
        ret = write_ubx_msg(ubx_dev->uart_num, msg, msg_len, true);
        if (ret != ESP_OK) {
            ELOG(TAG, "[%s] write_ubx_msg failed: %s", __FUNCTION__, esp_err_to_name(ret));
            goto done;
        }
        if(need_ack)
            ret = ack_status(ubx_dev, *(msg+2), *(msg+3));
    done:
       ubx_unlock();
    }
    DMEAS_END(TAG);
    return ret;
}

esp_err_t send_ubx_cfg_msg(ubx_ctx_t *ubx_dev, uint8_t cls, uint8_t id, const uint8_t * payload, size_t len, bool need_ack) {
    const uint8_t msgb[] = {UBX_HDR_A, UBX_HDR_B, cls, id, 0x00, 0x00, 0x00, 0x00};
    uint8_t *msg = 0;
    size_t msgb_len = sizeof(msgb), total_len = msgb_len + len;
    if(len){
        msg = heap_caps_calloc(total_len, sizeof(uint8_t),
            MALLOC_CAP_DEFAULT);
        if (!msg) {
            ELOG(TAG, "[%s] heap_caps_calloc failed", __FUNCTION__);
            return ESP_ERR_NO_MEM;
        }
        memcpy(msg, &(msgb[0]), 6); // copy header and class
        encode_uint16(msg+4, len); // add payload size
        memcpy(msg+6, payload, len); // copy payload
    }
    else
        msg = (uint8_t*)&(msgb[0]); // no payload
    // assert(msg);
    esp_err_t ret = ubx_cfg_send_m(ubx_dev, msg, total_len, need_ack);
    if (ret != ESP_OK) {
        ELOG(TAG, "[%s] failed: %s", __FUNCTION__, esp_err_to_name(ret));
    }
    if(len)
        heap_caps_free(msg);
    return ret;
}

esp_err_t ubx_cfg_valset(ubx_ctx_t *ubx_dev, const uint8_t * payload, size_t len, bool need_ack) {
    FUNC_ENTRYD(TAG);
    if(ubx_dev->hw_type < UBX_TYPE_M9)
        return ESP_ERR_INVALID_ARG;
    uint8_t *msg = heap_caps_calloc(len+4, sizeof(uint8_t),
        MALLOC_CAP_DEFAULT);
    if (!msg) {
        ELOG(TAG, "[%s] heap_caps_calloc failed", __FUNCTION__);
        return ESP_ERR_NO_MEM;
    }
    memcpy(msg, (const uint8_t[]){0x01, 0x01, 0x00, 0x00}, 4);
    memcpy(msg+4, payload, len);
    esp_err_t ret = send_ubx_cfg_msg(ubx_dev, CLS_CFG, CFG_VALSET, msg, len + 4, need_ack);
    heap_caps_free(msg);
    return ret;
}

esp_err_t ubx_cfg_get(ubx_ctx_t *ubx_dev, ubx_msg_byte_ctx_t * ubx_packet) {
    DMEAS_START();
    // assert(ubx_packet && ubx_dev);
    esp_err_t ret = send_ubx_cfg_msg(ubx_dev, *ubx_packet->msg, *(ubx_packet->msg+1), NULL, 0, false);
    if(ubx_lock(500)) {
        ret = read_ubx_msg(ubx_dev, ubx_packet); // this msg is without ubx header as ubx_msg_t parts start with class and id
        if (ret != ESP_OK) {
            ELOG(TAG, "[%s] read_ubx_msg failed: %s", __FUNCTION__, esp_err_to_name(ret));
        }
        ubx_unlock();
    }
    DMEAS_END(TAG);
    return ret;
}
#if defined(CONFIG_UBX_TIMER_STATS_ENABLED)
// Print UART-level message statistics (message throughput and types)
void ubx_uart_print_stats(uint32_t period_ms, uint8_t expected_hz) {
    // Calculate period statistics
    uint16_t period_count = uart_rx_stats.count - uart_prev_stats.count;
    uint16_t period_nav_pvt = uart_rx_stats.count_nav_pvt - uart_prev_stats.count_nav_pvt;
    uint16_t period_nav_sat = uart_rx_stats.count_nav_sat - uart_prev_stats.count_nav_sat;
    uint16_t period_nav_dop = uart_rx_stats.count_nav_dop - uart_prev_stats.count_nav_dop;

    // Update previous snapshot
    uart_prev_stats = uart_rx_stats;

    // Calculate throughput
    float period_s = (float)period_ms / 1000.0f;
    float throughput = period_s > 0.0f ? (float)period_count / period_s : 0.0f;
    float expected_count = expected_hz*2+1;  // Total messages expected in this period
    float loss_pct = expected_count > 0.0f ? (1.0f - throughput / expected_count) * 100.0f : 0.0f;
    if (loss_pct < 0.0f) loss_pct = 0.0f;  // Clamp negative loss (happens when rate > expected)

    printf("[UART] ========== UART RX LAYER STATS ==========\n");
    printf("[UART] Headers found (UART RX): %.1f msg/s (expected: %.1f msg/s at %" PRIu16 " Hz, loss: %.1f%%)\n",
        throughput, expected_count, expected_hz, loss_pct);
    printf("[UART] Message types (period): PVT=%" PRIu16 " SAT=%" PRIu16 " DOP=%" PRIu16 "\n",
        period_nav_pvt, period_nav_sat, period_nav_dop);
    printf("[UART] Totals: Headers=%" PRIu32 " PVT=%" PRIu32 " SAT=%" PRIu32 " DOP=%" PRIu32 "\n",
        uart_rx_stats.count, uart_rx_stats.count_nav_pvt, uart_rx_stats.count_nav_sat, uart_rx_stats.count_nav_dop);
    printf("[UART] ==========================================\n");
}

void ubx_print_stats(uint32_t period_ms, uint8_t expected_hz) {
    // Calculate period stats (difference from previous snapshot)
    period_msg_stats.count_err = cur_msg_stats.count_err - prev_msg_stats.count_err;
    period_msg_stats.count = cur_msg_stats.count - prev_msg_stats.count;
    period_msg_stats.count_nav_pvt = cur_msg_stats.count_nav_pvt - prev_msg_stats.count_nav_pvt;
    period_msg_stats.count_nav_sat = cur_msg_stats.count_nav_sat - prev_msg_stats.count_nav_sat;
    period_msg_stats.count_nav_dop = cur_msg_stats.count_nav_dop - prev_msg_stats.count_nav_dop;

    // Update previous snapshot
    prev_msg_stats = cur_msg_stats;

    // Calculate throughput metrics
    float period_s = (float)period_ms / 1000.0f;
    float throughput = period_s > 0.0f ? (float)period_msg_stats.count / period_s : 0.0f;
    float expected_count = expected_hz * 2 + 1; // Total messages expected in this period
    float loss_pct = period_msg_stats.count > 0 ? (float)period_msg_stats.count_err * 100.0f / (float)period_msg_stats.count : 0.0f;

    printf("[UBX] ========== UBX MODULE STATS ==========\n");
    printf("[UART] Messages: %.1f msg/s (expected: %.1f msg/s at %" PRIu16 " Hz, loss: %.1f%%)\n",
        throughput, expected_count, expected_hz, loss_pct);
    printf("[UBX] RX Throughput: %.1f msg/s | Loss: %.1f%% (%"PRIu32" err / %"PRIu32" msg)\n",
        throughput, loss_pct, period_msg_stats.count_err, period_msg_stats.count);
    printf("[UBX] Message types (period): PVT=%"PRIu32" SAT=%"PRIu32" DOP=%"PRIu32"\n",
        period_msg_stats.count_nav_pvt, period_msg_stats.count_nav_sat, period_msg_stats.count_nav_dop);
    printf("[UBX] Totals: msg=%"PRIu32" ok=%"PRIu32" err=%"PRIu32" | PVT=%"PRIu32" SAT=%"PRIu32" DOP=%"PRIu32"\n",
        cur_msg_stats.count, cur_msg_stats.count_ok, cur_msg_stats.count_err, 
        cur_msg_stats.count_nav_pvt, cur_msg_stats.count_nav_sat, cur_msg_stats.count_nav_dop);
    printf("[UBX] ==========================================\n");
}
#else
void ubx_uart_print_stats(uint32_t period_ms, uint8_t expected_hz) {}
void ubx_print_stats(uint32_t period_ms, uint8_t expected_hz) {}
#endif

#endif
