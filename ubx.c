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
#include <esp_heap_caps.h>
#include "config_observer.h"
#include "config_lock.h"
#include "ubx.h"
#include <driver/gpio.h>
#include <sys/time.h>

static const char *TAG = "ublox";

ESP_EVENT_DEFINE_BASE(UBX_EVENT);

#if (C_LOG_LEVEL <= LOG_DEBUG_NUM)
static const char *const _ubx_event_strings[] = {UBX_EVENT_LIST(STRINGIFY)};
const char *ubx_event_strings(int id) {
	return id < lengthof(_ubx_event_strings) ? _ubx_event_strings[id]
											 : "UBX_EVENT_UNKNOWN";
}
#else
const char *ubx_event_strings(int id) { return "UBX_EVENT"; }
#endif

SemaphoreHandle_t xMutex = NULL;
#define TIMEOUT_MAX portMAX_DELAY
// static const TickType_t timeout_immediate = 0;
static ubx_ctx_t *ubx_ctx_global = NULL;

// UBX config is now part of unified RTC config
// Access via: g_rtc_config.ubx
static const uint32_t ubx_baud_rates[] = {UBX_BAUD_RATE_LIST(NUMERIFY_V)};
// static const uint8_t ubx_hw_types[] = { UBX_TYPE_LIST(NUMERIFY_VV) };
static const char *const ubx_hw_type_strings[] = {UBX_TYPE_LIST(STRINGIFY_M)};
static const char *const ubx_baud_rate_strings[] = {
	UBX_BAUD_RATE_LIST(STRINGIFY_L)};

RTC_DATA_ATTR const char *ubx_dev_str = "UNKNOWN";

#define UBX_RTC_CACHE_MAGIC 0x55425843UL
#define UBX_RTC_CACHE_VERSION 2U
#define UBX_RTC_CACHE_FLAG_STATE_VALID (1U << 0)
#define UBX_RTC_CACHE_FLAG_CFG_PERSISTED (1U << 1)
#define UBX_RTC_CACHE_FLAG_SKIP_SAVE_CFG (1U << 2)
#define UBX_RTC_CACHE_FLAG_BOOT_BAUD_VALID (1U << 3)

#define UBX_SAVE_DEV_BBR 0x01U
#define UBX_SAVE_DEV_FLASH 0x02U
#define UBX_SAVE_DEV_BBR_FLASH (UBX_SAVE_DEV_BBR | UBX_SAVE_DEV_FLASH)

typedef struct {
	uint32_t magic;
	uint16_t version;
	uint16_t flags;
	uint32_t baud;
	uint32_t boot_baud;
	uint8_t hw_id[sizeof(((ubx_ctx_t *)0)->hw_id)];
	uint8_t prot_ver;
	uint8_t hw_type;
	uint8_t gnss;
	uint8_t output_rate;
	uint8_t nav_mode;
	uint8_t log_sat_details;
	uint8_t reserved[3];
} ubx_rtc_cache_t;

RTC_DATA_ATTR static ubx_rtc_cache_t s_ubx_rtc_cache = {0};

typedef struct {
	uint8_t enable_gps;
	uint8_t enable_sbas;
	uint8_t enable_galileo;
	uint8_t enable_beidou;
	uint8_t enable_qzss;
	uint8_t enable_glonass;
} ubx_gnss_selection_t;

static esp_err_t ubx_save_cfg(ubx_ctx_t *ubx);
static esp_err_t ubx_save_cfg_devices(ubx_ctx_t *ubx, uint8_t device_mask);
static uint8_t fix_config(ubx_ctx_t *ubx_ctx);

static esp_err_t ubx_build_gnss_valset_payload(const ubx_ctx_t *ubx,
					       uint8_t mode,
					       uint8_t *payload,
					       size_t payload_size,
					       size_t *payload_len,
					       ubx_gnss_selection_t *selection);

static esp_err_t ubx_build_nav_mode_valset_payload(ubx_nav_mode_t nav_mode,
						       uint8_t *payload,
						       size_t payload_size,
						       size_t *payload_len);

static esp_err_t ubx_build_prot_msg_out_valset_payload(bool enable_nmea,
							 bool enable_ubx,
							 uint8_t *payload,
							 size_t payload_size,
							 size_t *payload_len);

static esp_err_t ubx_build_msgout_valset_payload(const ubx_ctx_t *ubx,
						     uint8_t *payload,
						     size_t payload_size,
						     size_t *payload_len);

static esp_err_t ubx_build_msgout_sat_valset_payload(uint8_t cfg_rate,
							 uint8_t *payload,
							 size_t payload_size,
							 size_t *payload_len);

static esp_err_t ubx_build_rate_valset_payload(uint8_t rate,
						   uint8_t *payload,
						   size_t payload_size,
						   size_t *payload_len);

static esp_err_t ubx_build_baud_valset_payload(uint32_t baud,
						   uint8_t *payload,
						   size_t payload_size,
						   size_t *payload_len);

static esp_err_t ubx_build_legacy_prt_payload(uint32_t baud,
						      uint16_t out_proto_mask,
						      uint8_t *payload,
						      size_t payload_size,
						      size_t *payload_len);

static esp_err_t ubx_build_legacy_rate_payload(uint8_t rate,
						       uint8_t *payload,
						       size_t payload_size,
						       size_t *payload_len);

static esp_err_t ubx_build_legacy_msg_rate_payload(uint8_t msg_id,
							   uint8_t rate,
							   uint8_t *payload,
							   size_t payload_size,
							   size_t *payload_len);

static uint32_t ubx_rate_to_baud(uint8_t rate) {
	if (rate > UBX_OUTPUT_10HZ) {
		return UBX_BAUD_230400;
	}
	if (rate > UBX_OUTPUT_2HZ) {
		return UBX_BAUD_115200;
	}
	return UBX_BAUD_38400;
}

static esp_err_t ubx_build_nav_mode_valset_payload(ubx_nav_mode_t nav_mode,
						       uint8_t *payload,
						       size_t payload_size,
						       size_t *payload_len) {
	if (!payload || !payload_len || payload_size < 5) {
		return ESP_ERR_INVALID_ARG;
	}

	memcpy(payload, (const uint8_t[]){0x1c, 0x00, 0x11, 0x20, (uint8_t)nav_mode},
	       5);
	*payload_len = 5;
	return ESP_OK;
}

static esp_err_t ubx_build_prot_msg_out_valset_payload(bool enable_nmea,
							 bool enable_ubx,
							 uint8_t *payload,
							 size_t payload_size,
							 size_t *payload_len) {
	if (!payload || !payload_len || payload_size < 10) {
		return ESP_ERR_INVALID_ARG;
	}
	if (!enable_nmea && !enable_ubx) {
		enable_ubx = true;
	}

	memcpy(payload,
	       (const uint8_t[]){0x02, 0x00, 0x74, 0x10, enable_nmea ? 0x01 : 0x00,
				  0x01, 0x00, 0x74, 0x10, enable_ubx ? 0x01 : 0x00},
	       10);
	*payload_len = 10;
	return ESP_OK;
}

static esp_err_t ubx_build_msgout_valset_payload(const ubx_ctx_t *ubx,
						     uint8_t *payload,
						     size_t payload_size,
						     size_t *payload_len) {
	if (!ubx || !payload || !payload_len || payload_size < 10) {
		return ESP_ERR_INVALID_ARG;
	}

	const uint8_t cfg_pvt_id = 0x07;
	const uint8_t cfg_dop_id = ubx->hw_type >= UBX_TYPE_M9 ? 0x39 : 0x04;
	memcpy(payload,
	       (const uint8_t[]){cfg_pvt_id, 0x00, 0x91, 0x20, 0x01,
				  cfg_dop_id, 0x00, 0x91, 0x20, 0x01},
	       10);
	*payload_len = 10;
	return ESP_OK;
}

static esp_err_t ubx_build_msgout_sat_valset_payload(uint8_t cfg_rate,
							 uint8_t *payload,
							 size_t payload_size,
							 size_t *payload_len) {
	if (!payload || !payload_len || payload_size < 5) {
		return ESP_ERR_INVALID_ARG;
	}

	memcpy(payload, (const uint8_t[]){0x16, 0x00, 0x91, 0x20, cfg_rate}, 5);
	*payload_len = 5;
	return ESP_OK;
}

static esp_err_t ubx_build_rate_valset_payload(uint8_t rate,
						   uint8_t *payload,
						   size_t payload_size,
						   size_t *payload_len) {
	if (!payload || !payload_len || payload_size < 6) {
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t rate_vec[2] = {0};
	encode_uint16(rate_vec, HZ_TO_MS(rate));
	memcpy(payload,
	       (const uint8_t[]){0x01, 0x00, 0x21, 0x30, rate_vec[0], rate_vec[1]},
	       6);
	*payload_len = 6;
	return ESP_OK;
}

static esp_err_t ubx_build_baud_valset_payload(uint32_t baud,
						   uint8_t *payload,
						   size_t payload_size,
						   size_t *payload_len) {
	if (!payload || !payload_len || payload_size < 8) {
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t baud_vec[4] = {0};
	encode_uint32(baud_vec, baud);
	memcpy(payload,
	       (const uint8_t[]){0x01, 0x00, 0x52, 0x40,
				  baud_vec[0], baud_vec[1], baud_vec[2], baud_vec[3]},
	       8);
	*payload_len = 8;
	return ESP_OK;
}

static esp_err_t ubx_build_legacy_prt_payload(uint32_t baud,
						      uint16_t out_proto_mask,
						      uint8_t *payload,
						      size_t payload_size,
						      size_t *payload_len) {
	if (!payload || !payload_len || payload_size < 20) {
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t baud_vec[4] = {0};
	uint8_t out_proto_mask_vec[2] = {0};
	encode_uint32(baud_vec, baud);
	encode_uint16(out_proto_mask_vec, out_proto_mask);
	memcpy(payload,
	       (const uint8_t[]){0x01,
				  0x00,
				  0x00,
				  0x00,
				  0xd0,
				  0x08,
				  0x00,
				  0x00,
				  baud_vec[0],
				  baud_vec[1],
				  baud_vec[2],
				  baud_vec[3],
				  0x23,
				  0x00,
				  out_proto_mask_vec[0],
				  out_proto_mask_vec[1],
				  0x00,
				  0x00,
				  0x00,
				  0x00},
	       20);
	*payload_len = 20;
	return ESP_OK;
}

static esp_err_t ubx_build_legacy_rate_payload(uint8_t rate,
						       uint8_t *payload,
						       size_t payload_size,
						       size_t *payload_len) {
	if (!payload || !payload_len || payload_size < 6) {
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t rate_vec[2] = {0};
	encode_uint16(rate_vec, HZ_TO_MS(rate));
	memcpy(payload,
	       (const uint8_t[]){rate_vec[0], rate_vec[1], 0x01, 0x00, 0x01, 0x00},
	       6);
	*payload_len = 6;
	return ESP_OK;
}

static esp_err_t ubx_build_legacy_msg_rate_payload(uint8_t msg_id,
							   uint8_t rate,
							   uint8_t *payload,
							   size_t payload_size,
							   size_t *payload_len) {
	if (!payload || !payload_len || payload_size < 8) {
		return ESP_ERR_INVALID_ARG;
	}

	memcpy(payload,
	       (const uint8_t[]){0x01, msg_id, 0x00, rate, 0x00, 0x00, 0x00, 0x00},
	       8);
	*payload_len = 8;
	return ESP_OK;
}

static esp_err_t ubx_save_cfg_via_valset(ubx_ctx_t *ubx, uint8_t device_mask) {
	if (ubx == NULL) {
		return ESP_ERR_INVALID_ARG;
	}
	if (ubx->hw_type < UBX_TYPE_M9) {
		return ESP_ERR_NOT_SUPPORTED;
	}

	const uint8_t effective_rate = ubx->effective_output_rate
					 ? ubx->effective_output_rate
					 : fix_config(ubx);
	const uint8_t nav_mode = (uint8_t)ubx_nav_mode_get_effective();
	uint8_t prot_msg_payload[10] = {0};
	uint8_t nav_mode_payload[5] = {0};
	uint8_t msgout_payload[10] = {0};
	uint8_t msgout_sat_payload[5] = {0};
	uint8_t gnss_payload[64] = {0};
	uint8_t rate_payload[6] = {0};
	uint8_t baud_payload[8] = {0};
	size_t prot_msg_payload_len = 0;
	size_t nav_mode_payload_len = 0;
	size_t msgout_payload_len = 0;
	size_t msgout_sat_payload_len = 0;
	size_t gnss_payload_len = 0;
	size_t rate_payload_len = 0;
	size_t baud_payload_len = 0;

	esp_err_t ret = ubx_build_prot_msg_out_valset_payload(false, true,
								   prot_msg_payload,
								   sizeof(prot_msg_payload),
								   &prot_msg_payload_len);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = ubx_build_nav_mode_valset_payload((ubx_nav_mode_t)nav_mode,
							 nav_mode_payload,
							 sizeof(nav_mode_payload),
							 &nav_mode_payload_len);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = ubx_build_msgout_valset_payload(ubx, msgout_payload,
						      sizeof(msgout_payload),
						      &msgout_payload_len);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = ubx_build_msgout_sat_valset_payload(effective_rate,
						  msgout_sat_payload,
						  sizeof(msgout_sat_payload),
						  &msgout_sat_payload_len);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = ubx_build_rate_valset_payload(effective_rate, rate_payload,
						  sizeof(rate_payload),
						  &rate_payload_len);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = ubx_build_baud_valset_payload(ubx_rate_to_baud(effective_rate),
						  baud_payload,
						  sizeof(baud_payload),
						  &baud_payload_len);
	if (ret != ESP_OK) {
		return ret;
	}

	ret = ubx_cfg_valset_layers(ubx, prot_msg_payload, prot_msg_payload_len,
					    device_mask, true);
	if (ret != ESP_OK) {
		return ret;
	}

	ret = ubx_cfg_valset_layers(ubx, nav_mode_payload, nav_mode_payload_len,
					    device_mask, true);
	if (ret != ESP_OK) {
		return ret;
	}

	ret = ubx_cfg_valset_layers(ubx, msgout_payload, msgout_payload_len,
					    device_mask, true);
	if (ret != ESP_OK) {
		return ret;
	}

	ret = ubx_cfg_valset_layers(ubx, msgout_sat_payload,
					    msgout_sat_payload_len, device_mask, true);
	if (ret != ESP_OK) {
		return ret;
	}

	ret = ubx_build_gnss_valset_payload(ubx, g_rtc_config.ubx.gnss,
					     gnss_payload, sizeof(gnss_payload),
					     &gnss_payload_len, NULL);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = ubx_cfg_valset_layers(ubx, gnss_payload, gnss_payload_len,
					    device_mask, true);
	if (ret != ESP_OK) {
		return ret;
	}

	ret = ubx_cfg_valset_layers(ubx, rate_payload, rate_payload_len,
					    device_mask, true);
	if (ret != ESP_OK) {
		return ret;
	}

	return ubx_cfg_valset_layers(ubx, baud_payload, baud_payload_len,
					 device_mask, true);
}

static esp_err_t ubx_build_gnss_valset_payload(const ubx_ctx_t *ubx,
					       uint8_t mode,
					       uint8_t *payload,
					       size_t payload_size,
					       size_t *payload_len,
					       ubx_gnss_selection_t *selection) {
	if (!ubx || !payload || !payload_len || payload_size < 64) {
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t enable_gps = 0x01;
	uint8_t enable_sbas = 0x01;
	uint8_t enable_galileo = 0x00;
	uint8_t enable_beidou = 0x00;
	uint8_t enable_qzss = 0x01;
	uint8_t enable_glonass = 0x00;

	if (BIT_GET(mode, UBX_GNSS_SBAS) == 0) {
		enable_sbas = 0;
	}
	if (BIT_GET(mode, UBX_GNSS_GALILEO) != 0) {
		enable_galileo = 1;
	}
	if (BIT_GET(mode, UBX_GNSS_BEIDOU) != 0) {
		enable_beidou = 1;
	}
	if (BIT_GET(mode, UBX_GNSS_QZSS) == 0) {
		enable_qzss = 0;
	}
	if (BIT_GET(mode, UBX_GNSS_GLONASS) != 0) {
		enable_glonass = 1;
	}
	if (ubx->gnss_count < 1) {
		enable_gps = 1;
	} else if (ubx->gnss_count > 4) {
		enable_gps = 0x01;
		enable_galileo = 0x01;
		enable_glonass = 0x01;
		enable_beidou = 0x01;
	}

	memcpy(payload,
	       (const uint8_t[]){0x1f, 0x00, 0x31, 0x10, enable_gps,
				  0x20, 0x00, 0x31, 0x10, enable_sbas,
				  0x21, 0x00, 0x31, 0x10, enable_galileo,
				  0x22, 0x00, 0x31, 0x10, enable_beidou,
				  0x24, 0x00, 0x31, 0x10, enable_qzss,
				  0x25, 0x00, 0x31, 0x10, enable_glonass},
	       30);
	uint8_t cursor = 30;
	const uint8_t tmp[] = {0x00, 0x31, 0x10, 0x01};
	if (enable_gps) {
		payload[cursor++] = 0x01;
		memcpy(&payload[cursor], tmp, sizeof(tmp));
		cursor += sizeof(tmp);
	}
	if (enable_galileo) {
		payload[cursor++] = 0x07;
		memcpy(&payload[cursor], tmp, sizeof(tmp));
		cursor += sizeof(tmp);
	}
	if (enable_glonass) {
		payload[cursor++] = 0x18;
		memcpy(&payload[cursor], tmp, sizeof(tmp));
		cursor += sizeof(tmp);
	}
	if (enable_beidou) {
		if (ubx->hw_type <= UBX_TYPE_M9 || !enable_glonass) {
			payload[cursor++] = 0x0d;
			memcpy(&payload[cursor], tmp, sizeof(tmp));
			cursor += sizeof(tmp);
		} else {
			payload[cursor++] = 0x0d;
			memcpy(&payload[cursor], tmp, sizeof(tmp) - 1);
			cursor += sizeof(tmp) - 1;
			payload[cursor++] = 0x00;
			payload[cursor++] = 0x0f;
			memcpy(&payload[cursor], tmp, sizeof(tmp));
			cursor += sizeof(tmp);
		}
	}

	if (selection) {
		selection->enable_gps = enable_gps;
		selection->enable_sbas = enable_sbas;
		selection->enable_galileo = enable_galileo;
		selection->enable_beidou = enable_beidou;
		selection->enable_qzss = enable_qzss;
		selection->enable_glonass = enable_glonass;
	}

	*payload_len = cursor;
	return ESP_OK;
}

static bool ubx_baud_rate_valid(uint32_t baud) {
	for (size_t i = 0; i < lengthof(ubx_baud_rates); ++i) {
		if (ubx_baud_rates[i] == baud) {
			return true;
		}
	}
	return false;
}

static bool ubx_rtc_cache_header_valid(void) {
	return s_ubx_rtc_cache.magic == UBX_RTC_CACHE_MAGIC &&
		   s_ubx_rtc_cache.version == UBX_RTC_CACHE_VERSION;
}

static uint32_t ubx_rtc_cache_boot_baud(void) {
	if (!ubx_rtc_cache_header_valid()) {
		return 0;
	}
	if ((s_ubx_rtc_cache.flags & UBX_RTC_CACHE_FLAG_BOOT_BAUD_VALID) == 0) {
		return 0;
	}
	return ubx_baud_rate_valid(s_ubx_rtc_cache.boot_baud)
			   ? s_ubx_rtc_cache.boot_baud
			   : 0;
}

static void ubx_rtc_cache_invalidate_state(void) {
	if (!ubx_rtc_cache_header_valid()) {
		return;
	}

	s_ubx_rtc_cache.flags &= UBX_RTC_CACHE_FLAG_BOOT_BAUD_VALID;
}

static void ubx_rtc_cache_store_boot_baud(uint32_t boot_baud) {
	if (!ubx_baud_rate_valid(boot_baud)) {
		return;
	}
	s_ubx_rtc_cache.magic = UBX_RTC_CACHE_MAGIC;
	s_ubx_rtc_cache.version = UBX_RTC_CACHE_VERSION;
	s_ubx_rtc_cache.flags |= UBX_RTC_CACHE_FLAG_BOOT_BAUD_VALID;
	s_ubx_rtc_cache.boot_baud = boot_baud;
}

static void ubx_baud_candidate_append(uint32_t *candidates, size_t max_count,
					  size_t *count, uint32_t baud) {
	if (!count || !candidates || !ubx_baud_rate_valid(baud)) {
		return;
	}
	for (size_t i = 0; i < *count; ++i) {
		if (candidates[i] == baud) {
			return;
		}
	}
	if (*count < max_count) {
		candidates[(*count)++] = baud;
	}
}

static bool ubx_hw_type_valid(uint8_t hw_type) {
	switch ((ubx_hw_t)hw_type) {
	case UBX_TYPE_M7:
	case UBX_TYPE_M8:
	case UBX_TYPE_M9:
	case UBX_TYPE_M10:
		return true;
	default:
		return false;
	}
}

static bool ubx_rtc_cache_valid(void) {
	return ubx_rtc_cache_header_valid() &&
		   (s_ubx_rtc_cache.flags & UBX_RTC_CACHE_FLAG_STATE_VALID) != 0 &&
		   ubx_hw_type_valid(s_ubx_rtc_cache.hw_type) &&
		   s_ubx_rtc_cache.baud > 0;
}

static bool ubx_rtc_cache_trusted(void) {
	return ubx_rtc_cache_valid() &&
		   (s_ubx_rtc_cache.flags & UBX_RTC_CACHE_FLAG_CFG_PERSISTED) != 0;
}

static bool ubx_rtc_cache_should_skip_save_cfg(void) {
	return ubx_rtc_cache_valid() &&
		   (s_ubx_rtc_cache.flags & UBX_RTC_CACHE_FLAG_SKIP_SAVE_CFG) != 0;
}

static bool ubx_rtc_cache_matches_cfg(void) {
	if (!ubx_rtc_cache_trusted()) {
		return false;
	}
	FUNC_ENTRY(TAG);
	const uint8_t effective_rate =
		ubx_ctx_global && ubx_ctx_global->effective_output_rate
			? ubx_ctx_global->effective_output_rate
			: ubx_get_effective_output_rate();
	return s_ubx_rtc_cache.baud == g_rtc_config.ubx.baud &&
		   s_ubx_rtc_cache.gnss == g_rtc_config.ubx.gnss &&
		   s_ubx_rtc_cache.output_rate == effective_rate &&
		   s_ubx_rtc_cache.nav_mode == g_rtc_config.ubx.nav_mode;
}

static void ubx_rtc_cache_store(const ubx_ctx_t *ubx_ctx,
						  bool receiver_cfg_persisted,
						  bool skip_save_cfg) {
	if (!ubx_ctx || ubx_ctx->hw_type <= UBX_TYPE_M0) {
		return;
	}
	FUNC_ENTRY(TAG);
	const uint32_t boot_baud = ubx_baud_rate_valid(ubx_ctx->detected_boot_baud)
					 ? ubx_ctx->detected_boot_baud
					 : ubx_rtc_cache_boot_baud();
	s_ubx_rtc_cache.magic = UBX_RTC_CACHE_MAGIC;
	s_ubx_rtc_cache.version = UBX_RTC_CACHE_VERSION;
	s_ubx_rtc_cache.flags = UBX_RTC_CACHE_FLAG_STATE_VALID;
	if (receiver_cfg_persisted) {
		s_ubx_rtc_cache.flags |= UBX_RTC_CACHE_FLAG_CFG_PERSISTED;
	}
	if (skip_save_cfg) {
		s_ubx_rtc_cache.flags |= UBX_RTC_CACHE_FLAG_SKIP_SAVE_CFG;
	}
	if (ubx_baud_rate_valid(boot_baud)) {
		s_ubx_rtc_cache.flags |= UBX_RTC_CACHE_FLAG_BOOT_BAUD_VALID;
		s_ubx_rtc_cache.boot_baud = boot_baud;
	} else {
		s_ubx_rtc_cache.boot_baud = 0;
	}
	s_ubx_rtc_cache.baud = g_rtc_config.ubx.baud;
	memcpy(s_ubx_rtc_cache.hw_id, ubx_ctx->hw_id,
		   sizeof(s_ubx_rtc_cache.hw_id));
	s_ubx_rtc_cache.prot_ver = ubx_ctx->prot_ver;
	s_ubx_rtc_cache.hw_type = (uint8_t)ubx_ctx->hw_type;
	s_ubx_rtc_cache.gnss = g_rtc_config.ubx.gnss;
	s_ubx_rtc_cache.output_rate = ubx_ctx->effective_output_rate;
	s_ubx_rtc_cache.nav_mode = g_rtc_config.ubx.nav_mode;
	s_ubx_rtc_cache.log_sat_details = g_rtc_config.ubx.log_sat_details;
}

static bool ubx_rtc_cache_apply(ubx_ctx_t *ubx_ctx) {
	if (!ubx_ctx || !ubx_rtc_cache_trusted()) {
		return false;
	}

	g_rtc_config.ubx.baud = s_ubx_rtc_cache.baud;
	ubx_ctx->uart_conf.baud_rate = g_rtc_config.ubx.baud;
	ubx_ctx->prot_ver = s_ubx_rtc_cache.prot_ver;
	ubx_ctx->hw_type = (ubx_hw_t)s_ubx_rtc_cache.hw_type;
	ubx_ctx->effective_output_rate = s_ubx_rtc_cache.output_rate;
	ubx_ctx->detected_boot_baud = ubx_rtc_cache_boot_baud();
	memcpy(ubx_ctx->hw_id, s_ubx_rtc_cache.hw_id, sizeof(ubx_ctx->hw_id));
	if (ubx_ctx->hw_type > UBX_TYPE_M0) {
		ubx_dev_str = ubx_chip_str(ubx_ctx);
	}
	ILOG(TAG,
		 "[%s] using cached receiver state: hw_type=%u baud=%" PRIu32
		 " rate=%" PRIu8,
		 __func__, ubx_ctx->hw_type, g_rtc_config.ubx.baud,
		 ubx_ctx->effective_output_rate);
	return true;
}

static esp_err_t ubx_verify_current_stream(ubx_ctx_t *ubx) {
	if (!ubx) {
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t sniff[384] = {0};
	const size_t got = ubx_rx_buf_read(ubx, sniff, sizeof(sniff), 2000);
	if (got == 0) {
		return ESP_ERR_TIMEOUT;
	}

	for (size_t i = 0; i < got; ++i) {
		if (sniff[i] == UBX_HDR_A && (i + 1) < got && sniff[i + 1] == UBX_HDR_B) {
			return ESP_OK;
		}
		if (sniff[i] == '$' && (i + 1) < got && sniff[i + 1] == 'G') {
			return ESP_OK;
		}
	}

	return ESP_ERR_INVALID_RESPONSE;
}

uint8_t ubx_get_effective_output_rate(void) {
	if (ubx_ctx_global && ubx_ctx_global->effective_output_rate) {
		return ubx_ctx_global->effective_output_rate;
	}
	return g_rtc_config.ubx.output_rate ? g_rtc_config.ubx.output_rate
								 : UBX_OUTPUT_RATE_DEFAULT;
}

const char *ubx_get_dev_str(void) { return ubx_dev_str; }

bool ubx_lock(int timeout) {
	FUNC_ENTRY(TAG);
	if (!xMutex)
		return false;
	const TickType_t timeout_ticks =
		(timeout == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout);
	return xSemaphoreTake(xMutex, timeout_ticks) == pdTRUE;
}

void ubx_unlock() {
	FUNC_ENTRY(TAG);
	if (xMutex) {
		xSemaphoreGive(xMutex);
	}
}

ubx_ctx_t *ubx_ctx_new() {
	FUNC_ENTRY(TAG);
	ubx_ctx_t *ubx = (ubx_ctx_t *)heap_caps_calloc(
		1, sizeof(ubx_ctx_t), MALLOC_CAP_DEFAULT);
	if (!ubx) {
		ELOG(TAG, "[%s] heap_caps_calloc failed", __FUNCTION__);
		return NULL;
	}
	esp_err_t ret = ubx_ctx_init(ubx);
	if (ret != ESP_OK) {
		ELOG(TAG, "[%s] ubx_ctx_init failed", __FUNCTION__);
		heap_caps_free(ubx);
		return NULL;
	}
	return ubx;
}

esp_err_t ubx_ctx_delete(ubx_ctx_t *ubx_ctx) {
	FUNC_ENTRY(TAG);
	ubx_ctx_deinit(ubx_ctx);
	if (ubx_ctx) {
		heap_caps_free(ubx_ctx);
		return ESP_OK;
	}
	return ESP_ERR_INVALID_ARG;
}

void ubx_config_changed_cb(size_t group, size_t index) {
	FUNC_ENTRY(TAG);
	if (group != config_ubx_handle()) {
		return;
	}
	if (index == cfg_ubx_ubx_nav_mode) {
		ubx_nav_mode_on_base_mode_changed();
	}
	if (!ubx_ctx_global || !ubx_ctx_global->ready) {
		if (index != cfg_ubx_ubx_nav_mode) {
			ELOG(TAG, "[%s] ubx_ctx not ready", __FUNCTION__);
		}
		return;
	}
	FUNC_ENTRY_ARGS(TAG, "group: %zu, index: %zu", group, index);
	switch (index) {
	case cfg_ubx_ubx_gnss:
	case cfg_ubx_ubx_output_rate:
		FUNC_ENTRY_ARGS(TAG, "changed rate=%d gnss=%u",
						g_rtc_config.ubx.output_rate, g_rtc_config.ubx.gnss);
		// Post event to trigger async reconfiguration instead of blocking.
		// MUST NOT use portMAX_DELAY: this callback runs inside
		// config_observer_notify() while config_lock is held.  Blocking
		// here can deadlock if the event queue is full.
		if (esp_event_post(UBX_EVENT, UBX_EVENT_CONFIG_CHANGED, NULL, 0,
						   pdMS_TO_TICKS(100)) != ESP_OK) {
			WLOG(TAG, "EVT_FAIL: UBX_EVENT_CONFIG_CHANGED");
		}
		break;
	case cfg_ubx_ubx_nav_mode:
		FUNC_ENTRY_ARGS(TAG, "changed base_nav_mode=%d effective_nav_mode=%d",
				g_rtc_config.ubx.nav_mode,
				ubx_nav_mode_get_effective());
		ubx_request_nav_mode_apply(ubx_ctx_global);
		break;
	default:
		break;
	}
}

static esp_err_t ubx_ctx_init(ubx_ctx_t *ubx_ctx) {
	FUNC_ENTRY(TAG);
	if (ubx_ctx == NULL)
		return ESP_ERR_INVALID_ARG;
	if (ubx_ctx->initialized)
		return ESP_OK;
	esp_err_t ret = ESP_OK;
	ubx_ctx_t cfg = UBX_DEFAULT_CTX();
	memcpy(ubx_ctx, &cfg, sizeof(ubx_ctx_t));
	ubx_ctx->rtc_conf = &g_rtc_config.ubx;
	ubx_ctx->uart_conf.baud_rate = g_rtc_config.ubx.baud;
	if (xMutex == NULL)
		xMutex = xSemaphoreCreateMutex();
	config_observer_add(ubx_config_changed_cb);
	ubx_ctx->initialized = true;
	ubx_ctx_global = ubx_ctx;
	return ret;
}

static esp_err_t ubx_ctx_deinit(ubx_ctx_t *ubx_ctx) {
	FUNC_ENTRY(TAG);
	if (xMutex) {
		vSemaphoreDelete(xMutex);
		xMutex = NULL;
	}
	if (ubx_ctx)
		ubx_ctx->initialized = false;
	ubx_ctx_global = NULL;
	return ESP_OK;
}

static esp_err_t ubx_pins_init(ubx_ctx_t *ubx_ctx) {
	FUNC_ENTRY(TAG);
	if (ubx_ctx == NULL)
		return ESP_ERR_INVALID_ARG;
	esp_err_t ret = ESP_OK;
	uint8_t i = 0;
	while (i < UBX_EN_PIN_LEN) {
		if (ubx_ctx->en_pins[i] == GPIO_NUM_NC) {
			goto next;
		}
		ret = gpio_set_direction(ubx_ctx->en_pins[i], GPIO_MODE_OUTPUT);
		if (ret != ESP_OK) {
			ELOG(TAG, "[%s] gpio_set_direction failed, gpio:%d, i:%" PRIu8,
				 __FUNCTION__, ubx_ctx->en_pins[i], i);
			goto done;
		}
		ret = gpio_set_level(ubx_ctx->en_pins[i], true);
		if (ret != ESP_OK) {
			ELOG(TAG, "[%s] gpio_set_level failed, gpio:%d, i:%" PRIu8,
				 __FUNCTION__, ubx_ctx->en_pins[i], i);
			goto done;
		}
		ret = gpio_set_drive_capability(ubx_ctx->en_pins[i], GPIO_DRIVE_CAP_3);
		if (ret != ESP_OK) {
			ELOG(TAG,
				 "[%s] gpio_set_drive_capability failed, gpio:%d, i:%" PRIu8,
				 __FUNCTION__, ubx_ctx->en_pins[i], i);
			goto done;
		}
	next:
		++i;
	}
done:
	return ret;
}

static esp_err_t ubx_pins_deinit(ubx_ctx_t *ubx_ctx) {
	FUNC_ENTRY(TAG);
	if (ubx_ctx == NULL)
		return ESP_ERR_INVALID_ARG;
	esp_err_t ret = ESP_OK;
	uint8_t i = 0;
	while (i < UBX_EN_PIN_LEN) {
		if (ubx_ctx->en_pins[i] == GPIO_NUM_NC)
			goto next;
		ret = gpio_set_level(ubx_ctx->en_pins[i], false);
		if (ret != ESP_OK) {
			ELOG(TAG, "[%s] gpio_set_level failed", __FUNCTION__);
		}
	next:
		i++;
	}
	return ret;
}

static esp_err_t ubx_uart_init(ubx_ctx_t *ubx_ctx) {
	FUNC_ENTRY(TAG);
	if (ubx_ctx == NULL)
		return ESP_ERR_INVALID_ARG;
	if (ubx_ctx->uart_is_on)
		return ESP_OK;
	if (!ubx_ctx->initialized) {
		ELOG(TAG, "[%s] ubx_ctx_init(cfg) must be called first", __FUNCTION__);
	}
	esp_err_t ret = ESP_OK;

	if (config_lock(500)) {

		ret = ubx_pins_init(ubx_ctx);
		if (ret != ESP_OK) {
			ELOG(TAG, "[%s] _dubx_ctx_pins_init failed: %s", __FUNCTION__,
				 esp_err_to_name(ret));
			goto done;
		}

		ret = uart_param_config(ubx_ctx->uart_num, &(ubx_ctx->uart_conf));
		if (ret != ESP_OK) {
			ELOG(TAG, "[%s] uart_param_config failed", __FUNCTION__);
			goto done;
		}
		ret = uart_set_pin(ubx_ctx->uart_num, ubx_ctx->tx_pin, ubx_ctx->rx_pin,
						   UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
		if (ret == ESP_OK) {
			ret = uart_set_sw_flow_ctrl(ubx_ctx->uart_num, false, 0, 0);
			if (ret != ESP_OK) {
				ELOG(TAG, "[%s] uart_set_sw_flow_ctrl failed", __FUNCTION__);
				goto done;
			}
		} else {
			WLOG(TAG, "[%s] uart_set_pin failed", __FUNCTION__);
			goto done;
		}
		int intr_alloc_flags = 0;
#if CONFIG_UART_ISR_IN_IRAM
		intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif
		// Install UART driver with event queue for event-driven reading
		ret = uart_driver_install(ubx_ctx->uart_num, 2048, 0, 20,
								  &ubx_ctx->uart_event_queue, intr_alloc_flags);
		if (ret != ESP_OK) {
			ELOG(TAG, "[%s] uart_driver_install failed", __FUNCTION__);
			goto done;
		}

		// Initialize event-driven infrastructure (circular buffer and task)
		ret = ubx_uart_event_init(ubx_ctx);
		if (ret != ESP_OK) {
			ELOG(TAG, "[%s] ubx_uart_event_init failed", __FUNCTION__);
			uart_driver_delete(ubx_ctx->uart_num);
			goto done;
		}
	done:
		// Bounded timeout: this runs while config_lock is held;
		// portMAX_DELAY here could deadlock the config subsystem.
		if (esp_event_post(UBX_EVENT,
					   !ret ? UBX_EVENT_UART_INIT_DONE
							: UBX_EVENT_UART_INIT_FAIL,
					   NULL, 0, pdMS_TO_TICKS(100)) != ESP_OK) {
			WLOG(TAG, "EVT_FAIL: UART_INIT event");
		}
		if (!ret) {
			ubx_ctx->uart_is_on = true;
		}
#if (C_LOG_LEVEL <= LOG_INFO_NUM)
		else {
			ELOG(TAG, "[%s] ubx_uart_init failed", __FUNCTION__);
		}
		FUNC_ENTRY_ARGSD(TAG, "done");
#endif
		config_unlock();
	}
	vTaskDelay(pdMS_TO_TICKS(100));
	return ret;
}

static esp_err_t ubx_uart_deinit(ubx_ctx_t *ubx_ctx) {
	FUNC_ENTRY(TAG);
	if (ubx_ctx == NULL)
		return ESP_ERR_INVALID_ARG;
	// if (!ubx_ctx->uart_is_on)
	//     return ESP_OK;
	esp_err_t ret = ESP_OK;
	if (ubx_lock(1000)) {
		// Cleanup event-driven infrastructure first
		ret = ubx_uart_event_deinit(ubx_ctx);
		if (ret != ESP_OK) {
			ELOG(TAG, "[%s] ubx_uart_event_deinit failed", __FUNCTION__);
		}

		ret = uart_driver_delete(ubx_ctx->uart_num);
		if (ret != ESP_OK) {
			ELOG(TAG, "[%s] uart_driver_delete failed", __FUNCTION__);
		}

		ret = ubx_pins_deinit(ubx_ctx);
		if (ret != ESP_OK) {
			ELOG(TAG, "[%s] ubx_pins_deinit failed", __FUNCTION__);
		}
		if (!ret) {
			if (esp_event_post(UBX_EVENT, UBX_EVENT_UART_DEINIT_DONE,
							   NULL, 0,
							   pdMS_TO_TICKS(100)) != ESP_OK) {
				WLOG(TAG, "EVT_FAIL: UART_DEINIT_DONE");
			}
			ubx_ctx->uart_is_on = false;
		}
#if (C_LOG_LEVEL <= LOG_INFO_NUM)
		else {
			ELOG(TAG, "[%s] ubx_uart_deinit failed", __FUNCTION__);
		}
		FUNC_ENTRY_ARGSD(TAG, "done");
#endif
		ubx_unlock();
	}
	return ret;
}

esp_err_t ubx_on(ubx_ctx_t *ubx_ctx) {
	FUNC_ENTRY_ARGS(TAG, " %p", ubx_ctx);
	if (ubx_ctx == NULL)
		return ESP_ERR_INVALID_ARG;
	IMEAS_START();
	esp_err_t ret = ESP_OK;
	ret = ubx_uart_init(ubx_ctx);
	IMEAS_END(TAG);
	return ret;
}

esp_err_t ubx_off(ubx_ctx_t *ubx_ctx) {
	FUNC_ENTRY_ARGS(TAG, " %p", ubx_ctx);
	if (ubx_ctx == NULL)
		return ESP_ERR_INVALID_ARG;
	IMEAS_START();
	esp_err_t ret = ESP_OK;
	ret = ubx_uart_deinit(ubx_ctx);
	if (ret != ESP_OK) {
		ELOG(TAG, "[%s] ubx_uart_deinit failed", __FUNCTION__);
	}
	ubx_ctx->ready = false;
	ubx_ctx->ready_time = 0;
	ubx_ctx->shutdown_requested = false;
	ubx_ctx->reconfig_requested = false;
	ubx_ctx->nav_mode_apply_requested = false;
	IMEAS_END(TAG);
	return ret;
}

static uint8_t fix_config(ubx_ctx_t *ubx_ctx) {
	if (ubx_ctx == NULL)
		return UBX_OUTPUT_5HZ;
	uint8_t gnss = g_rtc_config.ubx.gnss;
	uint8_t gnss_count = 1;
	uint8_t effective_rate = g_rtc_config.ubx.output_rate;
	if (gnss <= 5)
		g_rtc_config.ubx.gnss = gnss =
			ubx_ctx->hw_type >= UBX_TYPE_M9 ? 111 : 103;
	if (g_rtc_config.ubx.gnss == 111 && ubx_ctx->hw_type < UBX_TYPE_M9)
		g_rtc_config.ubx.gnss = gnss = 103;
	if (BIT_GET(gnss, UBX_GNSS_GALILEO) != 0)
		++gnss_count; // galileo
	if (BIT_GET(gnss, UBX_GNSS_BEIDOU) != 0)
		++gnss_count; // beidou
	if (BIT_GET(gnss, UBX_GNSS_GLONASS) != 0)
		++gnss_count; // glonass
	if (gnss_count > 4)
		gnss_count = 4;
	ubx_ctx->gnss_count = gnss_count;

	// Fix invalid output_rate (0 or invalid value)
	if (effective_rate == 0 || effective_rate > UBX_OUTPUT_20HZ) {
		FUNC_ENTRY_ARGSD(
			TAG, "Invalid output_rate %d, setting to default UBX_OUTPUT_5HZ",
			effective_rate);
		effective_rate = UBX_OUTPUT_5HZ;
	}
	if (ubx_ctx->hw_type == UBX_TYPE_M8) {
		if (ubx_ctx->gnss_count >= 2 && effective_rate > UBX_OUTPUT_10HZ) {
			FUNC_ENTRY_ARGSD(TAG,
							 "2 gnss, output rate > 10hz, fallback to 10hz");
			effective_rate = UBX_OUTPUT_10HZ;
		} else if (ubx_ctx->gnss_count == 1 &&
				   effective_rate > UBX_OUTPUT_5HZ) {
			FUNC_ENTRY_ARGSD(TAG, "1 gnss, output rate > 5hz, fallback to 5hz");
			effective_rate = UBX_OUTPUT_5HZ;
		}
	}
	if (ubx_ctx->hw_type == UBX_TYPE_M10) {
		if (ubx_ctx->gnss_count == 4 && effective_rate > UBX_OUTPUT_10HZ) {
			FUNC_ENTRY_ARGSD(TAG,
							 "4 gnss, output rate > 10hz, fallback to 10hz");
			effective_rate = UBX_OUTPUT_10HZ;
		} else if (ubx_ctx->gnss_count == 3 &&
				   effective_rate > UBX_OUTPUT_16HZ) {
			FUNC_ENTRY_ARGSD(TAG,
							 "3 gnss, output rate > 16hz, fallback to 16hz");
			effective_rate = UBX_OUTPUT_16HZ;
		} else if (ubx_ctx->gnss_count == 2 &&
				   effective_rate > UBX_OUTPUT_20HZ) {
			FUNC_ENTRY_ARGSD(TAG,
							 "2 gnss, output rate > 20hz, fallback to 20hz");
			effective_rate = UBX_OUTPUT_20HZ;
		}
	}
	ubx_ctx->effective_output_rate = effective_rate;
	return effective_rate;
}

esp_err_t ubx_set_gnss_and_rate(ubx_ctx_t *ubx_ctx, uint8_t gnss,
								uint8_t rate) {
	FUNC_ENTRY(TAG);
	if (ubx_ctx == NULL)
		return ESP_ERR_INVALID_ARG;
	esp_err_t ret = ESP_OK;
	uint8_t try, max_tries = 3;
	rate = fix_config(ubx_ctx);
	// Ensure rate is valid
	if (rate == 0 || rate > UBX_OUTPUT_20HZ) {
		WLOG(TAG, "[%s] Invalid rate %d from config, using UBX_OUTPUT_5HZ",
			 __func__, rate);
		rate = UBX_OUTPUT_5HZ;
		ubx_ctx->effective_output_rate = rate;
	}
	FUNC_ENTRY_ARGSD(TAG, "fix done, gnss: %" PRIu8 ", rate: %" PRIu8 "", gnss,
					 rate);
	// NAV-SAT output is a runtime requirement; log_sat_details only affects saved detail data.
	for (try = 0; try <= max_tries; ++try) {
		if (ubx_ctx->shutdown_requested || ubx_ctx->reconfig_requested)
			goto fail;
		ret = ubx_set_msgout_sat(ubx_ctx);
		if (ret == ESP_OK)
			break;
	}
	if (ret != ESP_OK) {
		ELOG(TAG, "[%s] ubx_set_msgout_sat failed", __func__);
	}
	// }
	FUNC_ENTRY_ARGSD(TAG, "nav_sat msg output configured");
	for (try = 0; try <= max_tries; ++try) {
		if (ubx_ctx->shutdown_requested || ubx_ctx->reconfig_requested)
			goto fail;
		ret = ubx_set_gnss(ubx_ctx, gnss);
		if (ret == ESP_OK) {
			delay_ms(300); // 0.5s required to reset gnss
			break;
		}
	}
	if (ret != ESP_OK) {
		ELOG(TAG, "[%s] ubx_set_gnss failed", __func__);
		goto fail;
	}
	FUNC_ENTRY_ARGSD(TAG, "set_gnss done");

	for (try = 0; try <= max_tries; ++try) {
		if (ubx_ctx->shutdown_requested || ubx_ctx->reconfig_requested)
			goto fail;
		ret = ubx_set_uart_out_rate(ubx_ctx, rate);
		if (ret == ESP_OK)
			break;
	}
	if (ret != ESP_OK) {
		ELOG(TAG, "[%s] ubx_set_uart_out_rate failed", __func__);
		goto fail;
	}
	FUNC_ENTRY_ARGSD(TAG, "set_rate done");
	if (esp_event_post(UBX_EVENT, UBX_EVENT_SAMPLE_RATE_CHANGED, 0, 0,
					   pdMS_TO_TICKS(100)) != ESP_OK) {
		WLOG(TAG, "EVT_FAIL: SAMPLE_RATE_CHANGED");
	}
fail:
	FUNC_ENTRY_ARGSD(TAG, "done, status %d", ret);
	return ret;
}

#if (C_LOG_LEVEL <= LOG_DEBUG_NUM)
int print_ubx_ctx_state(ubx_ctx_t *ubx_ctx) {
	if (ubx_ctx == NULL) {
		ELOG(TAG, "[%s] invalid argument, no dev!!", __func__);
		return ESP_ERR_INVALID_ARG;
	}
	esp_err_t ret = ESP_OK;
	printf("----- UBX device state -----\n");
	printf("Ublox type: %s\n", ubx_ctx->Ublox_type);
	printf("HW type: %s (%d)\n", ubx_chip_str(ubx_ctx), ubx_ctx->hw_type);
	printf("HW id: %s\n", (char *)ubx_ctx->hw_id);
	printf("Baud rate: %s (%" PRIu32 ")\n",
		   g_rtc_config.ubx.baud <
				   sizeof(ubx_baud_rate_strings) / sizeof(char *)
			   ? ubx_baud_rate_strings[g_rtc_config.ubx.baud]
			   : "Unknown",
		   g_rtc_config.ubx.baud);
	printf("GNSS: %" PRIu8 " (count: %" PRIu8 ")\n", g_rtc_config.ubx.gnss,
		   ubx_ctx->gnss_count);
	printf("Output rate requested: %" PRIu8 " Hz\n",
		   g_rtc_config.ubx.output_rate == 0
			   ? 0
			   : (uint8_t)(1000 / HZ_TO_MS(g_rtc_config.ubx.output_rate)));
	printf("Output rate effective: %" PRIu8 " Hz\n",
		   ubx_ctx->effective_output_rate == 0
			   ? 0
			   : (uint8_t)(1000 / HZ_TO_MS(ubx_ctx->effective_output_rate)));
	printf("Nav mode: base=%d effective=%d\n", g_rtc_config.ubx.nav_mode,
		   ubx_nav_mode_get_effective());
	printf("Log sat details: %d\n", g_rtc_config.ubx.log_sat_details);

	printf("UBX Initialized: %s\n", ubx_ctx->initialized ? "true" : "false");
	printf("UART setup ok: %s\n", ubx_ctx->uart_is_on ? "true" : "false");
	printf("UBX Setup in progress: %s\n",
		   ubx_ctx->setup_progress ? "true" : "false");
	printf("UBX ready: %s\n", ubx_ctx->ready ? "true" : "false");
	printf("UBX Is on: %s\n", ubx_ctx->uart_is_on ? "true" : "false");

	printf("----- UBX device state -----\n");
	return ret;
}
#endif

// Helper macro for UBX setup operations with retry logic
#define UBX_SETUP_TRY_OP(op_call, op_name, critical)                           \
	do {                                                                       \
		for (try = 0; try <= max_tries; ++try) {                               \
			if (ubx_ctx->shutdown_requested ||                                  \
			    ubx_ctx->reconfig_requested)                                    \
				goto fail;                                                     \
			ret = (op_call);                                                   \
			if (ret == ESP_OK)                                                 \
				break;                                                         \
		}                                                                      \
		if (ret != ESP_OK) {                                                   \
			ELOG(TAG, "[%s] " op_name " failed", __FUNCTION__);                \
			if (critical)                                                      \
				goto fail;                                                     \
		}                                                                      \
	} while (0)

esp_err_t ubx_setup(ubx_ctx_t *ubx_ctx) {
	FUNC_ENTRY(TAG);
	esp_err_t ret = ESP_OK;
	bool used_rtc_cache = false;
	bool receiver_cfg_matches_cache = false;
	bool receiver_cfg_persisted = false;
	bool skip_save_cfg = false;
	if (ubx_ctx == NULL) {
		ret = ESP_ERR_INVALID_ARG;
		goto end;
	}
#if (C_LOG_LEVEL <= LOG_DEBUG_NUM)
	print_ubx_ctx_state(ubx_ctx);
#endif
	if (ubx_ctx->ready || ubx_ctx->setup_progress) {
		FUNC_ENTRY_ARGS(TAG, " already setup done or processing");
		goto end;
	}
	IMEAS_START();
	ubx_ctx->setup_progress = 1;
	used_rtc_cache = ubx_rtc_cache_apply(ubx_ctx);
	if (!used_rtc_cache) {
		const uint32_t boot_baud_hint = ubx_rtc_cache_boot_baud();
		if (boot_baud_hint > 0) {
			ubx_ctx->uart_conf.baud_rate = boot_baud_hint;
			ILOG(TAG, "[%s] using RTC boot baud hint: %" PRIu32,
				 __FUNCTION__, boot_baud_hint);
		}
	}

	// Critical operations - setup fails if these don't work
	ret = ubx_on(ubx_ctx);
	if (ret != ESP_OK) {
		ELOG(TAG, "[%s] ubx_on failed", __FUNCTION__);
		goto fail;
	}

	if (used_rtc_cache) {
		ret = ubx_verify_current_stream(ubx_ctx);
		if (ret != ESP_OK) {
			WLOG(TAG,
				 "[%s] cached receiver state produced no valid startup traffic, falling back to full probe/reconfigure",
				 __FUNCTION__);
			ubx_rtc_cache_invalidate_state();
			used_rtc_cache = false;
			ret = ESP_OK;
		}
	}

	if (!used_rtc_cache) {
		ret = ubx_initial_read(ubx_ctx, false);
		if (ret != ESP_OK) {
			ELOG(TAG, "[%s] ubx_initial_read failed", __FUNCTION__);
			goto fail;
		}
	}

	uint8_t try, max_tries = 3;

	if (!used_rtc_cache) {
		UBX_SETUP_TRY_OP(ubx_get_hw_version(ubx_ctx), "ubx_get_hw_version",
					 true);
	}

	(void)fix_config(ubx_ctx);
	skip_save_cfg = ubx_rtc_cache_should_skip_save_cfg();
	receiver_cfg_matches_cache = used_rtc_cache && ubx_rtc_cache_matches_cfg();

	if (!receiver_cfg_matches_cache) {
		UBX_SETUP_TRY_OP(ubx_set_prot_msg_out(ubx_ctx, false, true),
					 "ubx_set_prot_msg_out", true);

		// Non-critical operations - log errors but continue
		ubx_ctx->nav_mode_apply_requested = false;
		UBX_SETUP_TRY_OP(ubx_set_nav_mode(ubx_ctx, ubx_nav_mode_get_effective()),
					 "ubx_set_nav_mode", false);
		UBX_SETUP_TRY_OP(ubx_set_msgout(ubx_ctx), "ubx_set_msgout", false);

		// Critical GNSS setup
		ret = ubx_set_gnss_and_rate(ubx_ctx, g_rtc_config.ubx.gnss,
								g_rtc_config.ubx.output_rate);
		if (ret != ESP_OK) {
			ELOG(TAG, "[%s] ubx_set_gnss_and_rate failed", __FUNCTION__);
			goto fail;
		}

		if (skip_save_cfg) {
			ILOG(TAG,
				 "[%s] skipping ubx_save_cfg, RTC cache marks persistent save unsupported",
				 __FUNCTION__);
		} else {
			ret = ubx_save_cfg(ubx_ctx);
			if (ret == ESP_ERR_TIMEOUT || ret == ESP_ERR_NOT_SUPPORTED ||
				ret == ESP_ERR_INVALID_RESPONSE) {
				skip_save_cfg = true;
				WLOG(TAG,
					 "[%s] ubx_save_cfg unsupported on this receiver, will skip retry until RTC is cleared",
					 __FUNCTION__);
				ret = ESP_OK;
			} else if (ret != ESP_OK) {
				WLOG(TAG,
					 "[%s] ubx_save_cfg failed, receiver may fall back to RAM-only config",
					 __FUNCTION__);
				ret = ESP_OK;
			} else {
				receiver_cfg_persisted = true;
			}
		}
	} else {
		ILOG(TAG,
			 "[%s] skipping persisted receiver config reapply, cached config already matches",
			 __FUNCTION__);
		receiver_cfg_persisted = true;
	}

	FUNC_ENTRY_ARGSD(TAG, " ubx_ctx->hw_id: %s, ubx_ctx->hw_type: %d",
					 &ubx_ctx->hw_id[0], ubx_ctx->hw_type);

	if (!used_rtc_cache) {
		UBX_SETUP_TRY_OP(ubx_get_hw_id(ubx_ctx), "ubx_get_hw_id", false);
		UBX_SETUP_TRY_OP(ubx_get_gnss(ubx_ctx), "ubx_get_gnss", true);
	}

	if (!ubx_ctx->shutdown_requested && !ubx_ctx->reconfig_requested) {
		if (esp_event_post(UBX_EVENT, UBX_EVENT_SETUP_DONE, NULL, 0,
						   pdMS_TO_TICKS(100)) != ESP_OK) {
			WLOG(TAG, "EVT_FAIL: SETUP_DONE");
		}
		if (!ret) {
			WLOG(TAG, "[%s] setup done, device ready!", __func__);
			ubx_ctx->ready = true;
			ubx_ctx->ready_time = get_millis();
			if (receiver_cfg_persisted || skip_save_cfg) {
				ubx_rtc_cache_store(ubx_ctx, receiver_cfg_persisted,
						    skip_save_cfg);
			}
		}
	}
fail:
	ubx_ctx->setup_progress = 0;
	IMEAS_END(TAG);
end:
#if (C_LOG_LEVEL <= LOG_DEBUG_NUM)
	print_ubx_ctx_state(ubx_ctx);
#endif
	return ret;
}

#undef UBX_SETUP_TRY_OP

esp_err_t ubx_set_nav_mode(ubx_ctx_t *ubx, ubx_nav_mode_t nav_mode) {
	FUNC_ENTRY(TAG);
	if (ubx == NULL)
		return ESP_ERR_INVALID_ARG;
	uint8_t payload[5] = {0};
	size_t payload_len = 0;
	esp_err_t ret = ESP_OK;
	FUNC_ENTRY_ARGS(TAG, "going to set nav mode: %d", nav_mode);
	ret = ubx_build_nav_mode_valset_payload(nav_mode, payload, sizeof(payload),
						    &payload_len);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = ubx_cfg_valset(ubx, payload, payload_len, true);
	if (!ret) {
		FUNC_ENTRY_ARGSD(TAG, "nav mode set to %s",
						 nav_mode == 0	   ? "PORT"
						 : nav_mode == 2   ? "STAT"
						 : (nav_mode == 3) ? "PED"
						 : nav_mode == 4   ? "AUTOMOT"
										   : "SEA");
		return ret;
	}
	// fallback old cfg_msg as valset failed
	return send_ubx_cfg_msg(ubx, CLS_CFG, CFG_NAV5,
							(const uint8_t[]){/* mask */ 0xFF,
											  0xFF,
											  nav_mode,
											  /* auto 2D-3D */ 0x03,
											  /* fixedAlt */ 0x00,
											  0x00,
											  0x00,
											  0x00,
											  /* fixedAltVar */ 0x10,
											  0x27,
											  0x00,
											  0x00,
											  /* minElev */ 0x05,
											  /* drLimit */ 0x00,
											  /* pDop */ 0xFA,
											  0x00,
											  /* tDop */ 0xFA,
											  0x00,
											  /* pAcc */ 0x64,
											  0x00,
											  /* tAcc */ 0x2C,
											  0x01,
											  /* staticHoldThresh */ 0x00,
											  /* dgpsTimeOut */ 0x00,
											  /* cnoThreshNumSVs */ 0x00,
											  /* cnoThresh */ 0x00,
											  /* reserved */ 0x00,
											  0x00,
											  0x00,
											  0x00,
											  0x00,
											  /* reserved */ 0x00,
											  0x00,
											  0x00,
											  0x00,
											  0x00},
							36, true);
}

void ubx_request_nav_mode_apply(ubx_ctx_t *ubx) {
	if (!ubx)
		return;
	ubx->nav_mode_apply_requested = true;
}

esp_err_t ubx_apply_pending_nav_mode(ubx_ctx_t *ubx) {
	if (ubx == NULL)
		return ESP_ERR_INVALID_ARG;
	if (!ubx->nav_mode_apply_requested || !ubx->ready || ubx->setup_progress)
		return ESP_OK;

	ubx->nav_mode_apply_requested = false;
	const ubx_nav_mode_t nav_mode = ubx_nav_mode_get_effective();
	const esp_err_t ret = ubx_set_nav_mode(ubx, nav_mode);

	if (ret != ESP_OK && !ubx->shutdown_requested && !ubx->reconfig_requested) {
		ubx->nav_mode_apply_requested = true;
	}
	return ret;
}

static esp_err_t ubx_set_prot_msg_out(ubx_ctx_t *ubx, bool enable_nmea,
									  bool enable_ubx) {
	FUNC_ENTRY(TAG);
	if (ubx == NULL)
		return ESP_ERR_INVALID_ARG;
	uint8_t payload[10] = {0};
	uint8_t legacy_payload[20] = {0};
	size_t payload_len = 0;
	size_t legacy_payload_len = 0;
	esp_err_t ret = ESP_OK;
	uint16_t out_proto_mask = 0x01;
	if (!enable_nmea && !enable_ubx)
		enable_ubx = true;
	if (enable_nmea && enable_ubx) {
		out_proto_mask = 0x03;
	} else if (enable_nmea) {
		out_proto_mask = 0x02;
	}
	FUNC_ENTRY_ARGSD(TAG, "going to enable_nmea: %u, enable_ubx: %u",
					 enable_nmea, enable_ubx);
	ret = ubx_build_prot_msg_out_valset_payload(enable_nmea, enable_ubx,
							 payload, sizeof(payload),
							 &payload_len);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = ubx_cfg_valset(ubx, payload, payload_len, true);
	if (!ret) {
		FUNC_ENTRY_ARGSD(TAG, "message protocol set to %s",
						 enable_nmea && enable_ubx ? "NMEA and UBX"
						 : enable_nmea			   ? "NMEA"
												   : "UBX");
		return ret;
	}
	ret = ubx_build_legacy_prt_payload((uint32_t)g_rtc_config.ubx.baud,
						  out_proto_mask,
						  legacy_payload,
						  sizeof(legacy_payload),
						  &legacy_payload_len);
	if (ret != ESP_OK) {
		return ret;
	}
	return send_ubx_cfg_msg(ubx, CLS_CFG, CFG_PRT, legacy_payload,
				     legacy_payload_len, true);
	}

static esp_err_t ubx_set_uart_baud_rate(ubx_ctx_t *ubx, int baud) {
	FUNC_ENTRY(TAG);
	if (ubx == NULL)
		return ESP_ERR_INVALID_ARG;
	if (baud == g_rtc_config.ubx.baud) {
		FUNC_ENTRY_ARGSD(TAG, "baud rate already set to %d, no changes made.",
						 baud);
		return ESP_OK;
	}
	esp_err_t ret = ESP_OK;
	uint8_t payload[8] = {0};
	uint8_t legacy_payload[20] = {0};
	size_t payload_len = 0;
	size_t legacy_payload_len = 0;
	ret = ubx_build_baud_valset_payload((uint32_t)baud, payload,
						    sizeof(payload), &payload_len);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = ubx_cfg_valset(ubx, payload, payload_len, false);
	if (!ret)
		goto done;
#if (C_LOG_LEVEL <= LOG_INFO_NUM)
	else {
		WLOG(TAG, "[%s] fallback to old cfg_msg as valset failed ...",
			 __FUNCTION__);
	}
#endif // fallback hw m8 and below
	ret = ubx_build_legacy_prt_payload((uint32_t)baud, 0x03,
						  legacy_payload,
						  sizeof(legacy_payload),
						  &legacy_payload_len);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = send_ubx_cfg_msg(ubx, CLS_CFG, CFG_PRT, legacy_payload,
					   legacy_payload_len, false);
	if (ret != ESP_OK) {
		return ret;
	}
done:
	g_rtc_config.ubx.baud = baud;
	ubx->uart_conf.baud_rate = baud;
	ret = ubx_uart_set_baud(ubx);
	return ret;
}

static esp_err_t ubx_set_uart_out_rate(ubx_ctx_t *ubx, uint8_t rate) {
	FUNC_ENTRY(TAG);
	if (ubx == NULL)
		return ESP_ERR_INVALID_ARG;
	// Check for invalid rate values that would cause division by zero or
	// invalid operation
	if (rate == 0 || rate > UBX_OUTPUT_20HZ) {
		ELOG(TAG, "[%s] Invalid rate %d, setting to UBX_OUTPUT_5HZ", __func__,
			 rate);
		rate = UBX_OUTPUT_5HZ;
	}
	esp_err_t ret = ESP_OK;
	uint8_t payload[6] = {0};
	uint8_t legacy_payload[6] = {0};
	size_t payload_len = 0;
	size_t legacy_payload_len = 0;
	int baud = (int)ubx_rate_to_baud(rate);

	FUNC_ENTRY_ARGSD(TAG, "solutions:%" PRIu8 " output rate: %u, baud: %d",
					 ubx->gnss_count, rate, baud);
	ret = ubx_build_rate_valset_payload(rate, payload, sizeof(payload),
						    &payload_len);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = ubx_cfg_valset(ubx, payload, payload_len, true);
	if (!ret)
		goto done;
#if (C_LOG_LEVEL <= LOG_INFO_NUM)
	else {
		WLOG(TAG, "[%s] fallback to old cfg_msg as valset failed ...",
			 __FUNCTION__);
	}
#endif // fallback hw m8 and below
	ret = ubx_build_legacy_rate_payload(rate, legacy_payload,
						  sizeof(legacy_payload),
						  &legacy_payload_len);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = send_ubx_cfg_msg(ubx, CLS_CFG, CFG_RATE, legacy_payload,
					   legacy_payload_len, true);
done:
	if (!ret)
		ret = ubx_set_uart_baud_rate(ubx, baud);
	return ret;
}

static esp_err_t ubx_set_gnss(ubx_ctx_t *ubx, uint8_t mode) {
	FUNC_ENTRY(TAG);
	uint8_t gnss_cmd[64] = {0};
	size_t gnss_len = 0;
	ubx_gnss_selection_t selection = {0};
	esp_err_t ret = ubx_build_gnss_valset_payload(ubx, mode, gnss_cmd,
						      sizeof(gnss_cmd), &gnss_len,
						      &selection);
	if (ret != ESP_OK) {
		return ret;
	}
	FUNC_ENTRY_ARGS(TAG,
					" mode:%" PRIu8 ", gps(us): %" PRIu8 ", sbas(us): %" PRIu8
					" galileo(eu): %" PRIu8 ", beidou(cn): %" PRIu8
					", glonass(ru): %" PRIu8 ", qzss(jp): %" PRIu8 "",
					mode, selection.enable_gps, selection.enable_sbas,
					selection.enable_galileo, selection.enable_beidou,
					selection.enable_glonass, selection.enable_qzss);
	ret = ubx_cfg_valset(ubx, gnss_cmd, gnss_len, true);
	if (!ret)
		return ret;
#if (C_LOG_LEVEL <= LOG_INFO_NUM)
	else {
		WLOG(TAG, "[%s] fallback to old cfg_msg as valset failed ...",
			 __FUNCTION__);
	}
#endif
	// fallback hw m8 and below
	return send_ubx_cfg_msg(
		ubx, CLS_CFG, CFG_GNSS,
		(const uint8_t[]){
			/* msgVer, numTrkChHw, numTrkChUse */ 0x00, 0x20, 0x20,
			/* numConfig */ 0x07,
			/* rep block: gnssID, resTrkCh, maxTrkCh, reserved0, flags */
			/* gps     */ 0x00, 0x08, 0x10, 0x00, selection.enable_gps, 0x00, 0x01, 0x01,
			/* sbas    */ 0x01, 0x01, 0x03, 0x00, selection.enable_sbas, 0x00, 0x01, 0x01,
			/* galileo */ 0x02, 0x04, 0x08, 0x00, selection.enable_galileo, 0x00, 0x01,
			0x01,
			/* beidou  */ 0x03, 0x08, 0x10, 0x00, selection.enable_beidou, 0x00, 0x01,
			0x01,
			/* qzss    */ 0x05, 0x00, 0x03, 0x00, selection.enable_qzss, 0x00, 0x01, 0x01,
			/* glonass */ 0x06, 0x08, 0x0E, 0x00, selection.enable_glonass, 0x00, 0x01,
			0x01},
		8 * 6 + 4, true);
}

static esp_err_t ubx_set_msgout(ubx_ctx_t *ubx) {
	FUNC_ENTRY(TAG);
	esp_err_t ret = ESP_OK;
	uint8_t payload[10] = {0};
	uint8_t legacy_payload[8] = {0};
	size_t payload_len = 0;
	size_t legacy_payload_len = 0;
	uint8_t cfg_pvt_id = 0x07;
	uint8_t cfg_dop_id = 0x04;
	if (ubx->hw_type >= UBX_TYPE_M9) {
		cfg_pvt_id = 0x07;
		cfg_dop_id = 0x39;
	}
	FUNC_ENTRY_ARGS(TAG, " enable navpvt and navdop ubx messages.");
	ret = ubx_build_msgout_valset_payload(ubx, payload, sizeof(payload),
						      &payload_len);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = ubx_cfg_valset(ubx, payload, payload_len, true);
	if (!ret)
		return ret;
#if (C_LOG_LEVEL <= LOG_INFO_NUM)
	else {
		WLOG(TAG, "[%s] fallback to old cfg_msg as valset failed ...",
			 __FUNCTION__);
	}
#endif
	// fallback hw m8 and below
	ret = ubx_build_legacy_msg_rate_payload(cfg_pvt_id, 0x01, legacy_payload,
						      sizeof(legacy_payload),
						      &legacy_payload_len);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = send_ubx_cfg_msg(ubx, CLS_CFG, CFG_MSG, legacy_payload,
					   legacy_payload_len, true);
	if (ret != ESP_OK)
		return ret;
	ret = ubx_build_legacy_msg_rate_payload(cfg_dop_id, 0x01, legacy_payload,
						      sizeof(legacy_payload),
						      &legacy_payload_len);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = send_ubx_cfg_msg(ubx, CLS_CFG, CFG_MSG, legacy_payload,
					   legacy_payload_len, true);
	return ret;
}

static esp_err_t ubx_set_msgout_sat(ubx_ctx_t *ubx) {
	FUNC_ENTRY(TAG);
	/* Send rate is relative to the event a message is registered on.
	For example, if the rate of a navigation message is set to 2,
	the message is sent every second navigation solution.
	For configuring NMEA messages, the section NMEA Messages
	Overview describes class and identifier numbers used. */
	const uint8_t effective_rate =
		ubx->effective_output_rate ? ubx->effective_output_rate
						   : ubx_get_effective_output_rate();
	uint8_t cfg_rate = effective_rate;
	uint8_t cfg_sat_id = 0x16;
	uint8_t payload[5] = {0};
	uint8_t legacy_payload[8] = {0};
	size_t payload_len = 0;
	size_t legacy_payload_len = 0;
	esp_err_t ret = ubx_build_msgout_sat_valset_payload(
		cfg_rate, payload, sizeof(payload), &payload_len);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = ubx_cfg_valset(ubx, payload, payload_len, true);
	if (!ret)
		return ret;
#if (C_LOG_LEVEL <= LOG_INFO_NUM)
	else {
		WLOG(TAG, "[%s] ubx_cfg_valset failed, fallback to old cfg_msg.",
			 __FUNCTION__);
	}
#endif
	// fallback hw m8 and below
	ret = ubx_build_legacy_msg_rate_payload(cfg_sat_id, cfg_rate,
						      legacy_payload,
						      sizeof(legacy_payload),
						      &legacy_payload_len);
	if (ret != ESP_OK) {
		return ret;
	}
	return send_ubx_cfg_msg(ubx, CLS_CFG, CFG_MSG, legacy_payload,
				     legacy_payload_len, true);
}

static esp_err_t ubx_save_cfg_devices(ubx_ctx_t *ubx, uint8_t device_mask) {
	FUNC_ENTRY(TAG);
	if (ubx == NULL) {
		return ESP_ERR_INVALID_ARG;
	}
	if (ubx->hw_type >= UBX_TYPE_M9) {
		esp_err_t ret = ubx_save_cfg_via_valset(ubx, device_mask);
		if (ret == ESP_OK) {
			return ESP_OK;
		}
		WLOG(TAG,
			 "[%s] ubx_cfg_valset persistence failed: %s, falling back to CFG-CFG",
			 __FUNCTION__, esp_err_to_name(ret));
	}

	esp_err_t ret = send_ubx_cfg_msg(
		ubx, CLS_CFG, CFG_CFG,
		(const uint8_t[]){0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00,
					  0x00, 0x00, 0x00, device_mask},
		13, true);
	return ret;
}

static esp_err_t ubx_save_cfg(ubx_ctx_t *ubx) {
	esp_err_t ret = ubx_save_cfg_devices(ubx, UBX_SAVE_DEV_BBR_FLASH);
	if (ret == ESP_OK) {
		return ESP_OK;
	}

	WLOG(TAG,
		 "[%s] combined BBR|flash save failed: %s, retrying flash only",
		 __FUNCTION__, esp_err_to_name(ret));
	const esp_err_t flash_ret = ubx_save_cfg_devices(ubx, UBX_SAVE_DEV_FLASH);
	if (flash_ret == ESP_OK) {
		ILOG(TAG, "[%s] flash-only save succeeded", __FUNCTION__);
		return ESP_OK;
	}

	WLOG(TAG, "[%s] flash-only save failed: %s", __FUNCTION__,
		 esp_err_to_name(flash_ret));
	return flash_ret;
}

static esp_err_t ubx_uart_set_baud(ubx_ctx_t *ubx_ctx) {
	FUNC_ENTRY_ARGS(TAG, " %" PRIu32 "", g_rtc_config.ubx.baud);
	esp_err_t ret = ESP_OK;
	delay_ms(10);
	// Note: Caller must already hold config_lock
	ret = uart_set_baudrate(ubx_ctx->uart_num, g_rtc_config.ubx.baud);
#if (C_LOG_LEVEL <= LOG_INFO_NUM)
	if (ret != ESP_OK) {
		ELOG(TAG, "[%s] uart_set_baudrate failed: %s", __FUNCTION__,
			 esp_err_to_name(ret));
	}
#endif
	delay_ms(50);
	return ret;
}

// get routines //

static esp_err_t ubx_get_hw_version(ubx_ctx_t *ubx) {
	FUNC_ENTRY(TAG);
	uint8_t *msg = (uint8_t *)&ubx->ubx_msg.mon_ver, *p;
	ubx_msg_byte_ctx_t ubx_packet = UBX_MSG_BYTE_CTX_DEFAULT(ubx->ubx_msg);
	ubx_packet.ctx = ubx;
	*ubx_packet.msg = CLS_MON;
	*(ubx_packet.msg + 1) = MON_VER;
	esp_err_t ret = ubx_cfg_get(ubx, &ubx_packet);
	if (ret != ESP_OK) {
		return ret;
	}
	char ver[32] = {0};
	memcpy(ver, msg + 4, 30);
	FUNC_ENTRY_ARGS(TAG, "swver: [%s]", ver);
	memset(ver, 0, 30);
	memcpy(ver, msg + 34, 10);
	FUNC_ENTRY_ARGS(TAG, "hwver: [%s]", ver);
	for (int i = 0; i < 6; ++i) {
		p = msg + 44 + i * 30;
		if (IS_ALNUM((char)*p)) {
			memset(ver, 0, 32);
			memcpy(ver, p, 30);
		} else {
			break;
		}
		FUNC_ENTRY_ARGS(TAG, "verext %d: [%s]", i, ver);
	}
	ubx->hw_type = *(msg + 34 + 3) == '8'	? UBX_TYPE_M8
				   : *(msg + 34 + 3) == '9' ? UBX_TYPE_M9
				   : *(msg + 34 + 3) == 'A' ? UBX_TYPE_M10
											: UBX_TYPE_M0;
	if (ubx->hw_type)
		ubx_dev_str = ubx_chip_str(ubx);
	return ESP_OK;
}

static esp_err_t ubx_get_hw_id(ubx_ctx_t *ubx) {
	FUNC_ENTRY(TAG);
	uint8_t *msg = (uint8_t *)&ubx->ubx_msg.ubxId;
	ubx_msg_byte_ctx_t ubx_packet = UBX_MSG_BYTE_CTX_DEFAULT(ubx->ubx_msg);
	ubx_packet.ctx = ubx;
	*ubx_packet.msg = CLS_SEC;
	*(ubx_packet.msg + 1) = SEC_UNIQID;
	esp_err_t ret = ubx_cfg_get(ubx, &ubx_packet);
	if (ret != ESP_OK) {
		return ret;
	}
	memcpy(&(ubx->hw_id[0]), msg + 8, 6);
	ubx->hw_id[6] = '\0';
#if (C_LOG_LEVEL <= LOG_DEBUG_NUM)
	for (int i = 0; i < 6; ++i) {
		DLOG(TAG, "hw id[%d]: [%" PRIu8 "]", i, *(msg + 8 + i));
	}
#endif
	return ESP_OK;
}

static esp_err_t ubx_get_gnss(ubx_ctx_t *ubx) {
	FUNC_ENTRY(TAG);
	uint8_t *msg = (uint8_t *)&ubx->ubx_msg.monGNSS;
	ubx_msg_byte_ctx_t ubx_packet = UBX_MSG_BYTE_CTX_DEFAULT(ubx->ubx_msg);
	ubx_packet.ctx = ubx;
	*ubx_packet.msg = CLS_MON;
	*(ubx_packet.msg + 1) = MON_GNSS;
	esp_err_t ret = ubx_cfg_get(ubx, &ubx_packet);
	return ret;
}

static esp_err_t ubx_get_nav_sat(ubx_ctx_t *ubx) {
	FUNC_ENTRY(TAG);
	uint8_t *msg = (uint8_t *)&ubx->ubx_msg.nav_sat;
	ubx_msg_byte_ctx_t ubx_packet = UBX_MSG_BYTE_CTX_DEFAULT(ubx->ubx_msg);
	ubx_packet.ctx = ubx;
	*ubx_packet.msg = CLS_NAV;
	*(ubx_packet.msg + 1) = NAV_SAT;
	esp_err_t ret = ubx_cfg_get(ubx, &ubx_packet);
	return ret;
}

static esp_err_t ubx_try_baud(ubx_ctx_t *ubx, ubx_msg_byte_ctx_t *ubx_packet) {
	FUNC_ENTRY(TAG);
	esp_err_t ret = ESP_OK;
 	uint32_t candidates[lengthof(ubx_baud_rates) + 2] = {0};
	size_t candidate_count = 0;
	ubx_baud_candidate_append(candidates, lengthof(candidates),
				      &candidate_count, ubx_rtc_cache_boot_baud());
	ubx_baud_candidate_append(candidates, lengthof(candidates),
				      &candidate_count, g_rtc_config.ubx.baud);
	for (size_t i = 0; i < lengthof(ubx_baud_rates); ++i) {
		ubx_baud_candidate_append(candidates, lengthof(candidates),
				      &candidate_count, ubx_baud_rates[i]);
	}

	for (size_t i = 0; i < candidate_count; ++i) {
		g_rtc_config.ubx.baud = candidates[i];
		ret = ubx_uart_set_baud(ubx);
#if (C_LOG_LEVEL <= LOG_DEBUG_NUM)
		if (ret != ESP_OK) {
			WLOG(TAG, "[%s] ubx_uart_set_baud failed: %s", __FUNCTION__,
				 esp_err_to_name(ret));
		}
#endif
		delay_ms(50);
		FUNC_ENTRY_ARGSD(TAG, "try read initial data with %" PRIu32 "",
						 g_rtc_config.ubx.baud);
		memset(ubx_packet->msg, 0, ubx_packet->msg_size);
		// ubx_packet->ubx_msg = &ubx->ubx_msg;
		ret =
			read_ubx_msg(ubx, ubx_packet); // just fill the msg buffer to check
										   // if we can read ubx or nmea message
		uint8_t *q = ubx_packet->msg;
		char *p = 0;
		while (q < (ubx_packet->msg + ubx_packet->msg_size) && *q) {
			p = (char *)q;
			if (*q == UBX_HDR_A && *(q + 1) == UBX_HDR_B) {
				ubx->detected_boot_baud = g_rtc_config.ubx.baud;
				ubx_rtc_cache_store_boot_baud(ubx->detected_boot_baud);
				FUNC_ENTRY_ARGSD(
					TAG, "found UBX message at %d with baud: %" PRIu32 "",
					p - (char *)ubx_packet->msg, g_rtc_config.ubx.baud);
				return ESP_OK;
				break;
			} else if (*p == '$' && *(p + 1) == 'G') {
				ubx->detected_boot_baud = g_rtc_config.ubx.baud;
				ubx_rtc_cache_store_boot_baud(ubx->detected_boot_baud);
				FUNC_ENTRY_ARGSD(
					TAG, "found NMEA message at %d with baud: %" PRIu32 "",
					p - (char *)ubx_packet->msg, g_rtc_config.ubx.baud);
				return ESP_OK;
				break;
			}
			++q;
		}

		if (ret != ESP_OK || !*(ubx_packet->msg + 3)) {
			WLOG(TAG, "[%s] %" PRIu32 " failed: %s", __FUNCTION__,
				 g_rtc_config.ubx.baud, esp_err_to_name(ret));
			continue;
		}
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

static void hex_string_to_uint8_t(const char *hex_string, uint8_t *output,
								  size_t output_size) {
	for (size_t i = 0; i < output_size; ++i) {
		char c1 = hex_string[i * 2];
		char c2 = hex_string[i * 2 + 1];
		output[i] = (hex_char_to_uint8_t(c1) << 4) + hex_char_to_uint8_t(c2);
	}
}

static esp_err_t ubx_initial_read(ubx_ctx_t *ubx, bool get_hw) {
	FUNC_ENTRY(TAG);
	esp_err_t ret = ESP_OK;
	uint8_t msg[384] = {0};
	size_t msg_size = sizeof(msg);
	ubx_msg_byte_ctx_t ubx_packet = UBX_MSG_BYTE_CTX_DEFAULT(ubx->ubx_msg);
	ubx_packet.ctx = ubx;
	ubx_packet.msg = msg;
	ubx_packet.msg_size = msg_size;
	ubx_packet.expect_ubx_msg = false;
	ubx_packet.msg_type_handler = NULL;
	ubx_packet.msg_pos = 0;
	ubx_packet.msg_match_to_pos = false;
	ret = ubx_try_baud(ubx, &ubx_packet);
	if (ret != ESP_OK) {
#if (C_LOG_LEVEL <= LOG_INFO_NUM)
		ELOG(TAG, "[%s] ubx_try_baud failed: %s", __FUNCTION__,
			 esp_err_to_name(ret));
#endif
		return ret;
	}
	if (get_hw) {
		const char *p = 0;
		if (!(p = strstr((const char *)&(msg[0]), "$GNTXT"))) {
			return ESP_OK;
		}

		if (!(p = strstr(p, ",HW UBX"))) {
			return ESP_OK;
		} else {
			ret = 1;
		find_space:
			while (*(++p) == ' ')
				++p;
			if (*p == '0' && *(++p) == '0')
				p += 2;
			else
				goto find_space;
			ubx->hw_type = *p == '8'   ? UBX_TYPE_M8
						   : *p == '9' ? UBX_TYPE_M9
						   : *p == 'A' ? UBX_TYPE_M10
									   : UBX_TYPE_M0;
			if (ubx->hw_type)
				ubx_dev_str = ubx_chip_str(ubx);
		}

		if (p && (p = strstr(p, ",PROTVER="))) {
			ubx->prot_ver = (uint8_t)atoi(p + 9);
		}
		if (p && (p = strstr(p, ",CHIPID="))) {
			p += 14;
			hex_string_to_uint8_t(p, &(ubx->hw_id[0]), 6);
		}
	}
	FUNC_ENTRY_ARGS(TAG, " ok.");
	return ret;
}

// private functions

static const char *ubx_chip_str(const ubx_ctx_t *ubx) {
	if (!ubx)
		goto fail;
	switch (ubx->hw_type) {
	case UBX_TYPE_M7:
		return ubx_hw_type_strings[1];
	case UBX_TYPE_M8:
		return ubx_hw_type_strings[2];
	case UBX_TYPE_M9:
		return ubx_hw_type_strings[3];
	case UBX_TYPE_M10:
		return ubx_hw_type_strings[4];
	default:
	fail:
		return ubx_dev_str;
	}
}

const char *ubx_baud_str(const ubx_ctx_t *ubx) {
	switch (g_rtc_config.ubx.baud) {
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

// #define MIN_numSV_FIRST_FIX 5      // alvorens start loggen, changed from 4
// to 5 7.1/2023 #define MAX_Sacc_FIRST_FIX 2       // alvorens start loggen
// #define MIN_numSV_GPS_SPEED_OK  4  // min aantal satellieten voor berekenen
// snelheid, anders #define MAX_Sacc_GPS_SPEED_OK  1   // max waarde Sacc voor
// berekenen snelheid, anders 0 #define MAX_GPS_SPEED_OK  40       // max
// snelheid in m/s voor berekenen snelheid, anders 0 #define
// MIN_SPEED_START_LOGGING 2000        //was 2000 min speed in mm/s over 2 s
// alvorens start loggen naar SD #define TIME_DELAY_FIRST_FIX 10 //10 navpvt
// messages alvorens start loggen

#endif
