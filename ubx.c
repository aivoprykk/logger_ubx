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
#include "config.h"
#include "ubx.h"
#include <driver/gpio.h>
#include <esp_log.h>
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
	ubx_ctx_t *ubx = (ubx_ctx_t *)calloc(1, sizeof(ubx_ctx_t));
	esp_err_t ret = ESP_OK;
	ret = ubx_ctx_init(ubx);
	if (ret != ESP_OK) {
		ELOG(TAG, "[%s] ubx_ctx_init failed", __FUNCTION__);
		free(ubx);
		return NULL;
	}
	return ubx;
}

esp_err_t ubx_ctx_delete(ubx_ctx_t *ubx_ctx) {
	FUNC_ENTRY(TAG);
	ubx_ctx_deinit(ubx_ctx);
	if (ubx_ctx) {
		free(ubx_ctx);
		return ESP_OK;
	}
	return ESP_ERR_INVALID_ARG;
}

void ubx_config_changed_cb(size_t group, size_t index) {
	FUNC_ENTRY(TAG);
	if (group != config_ubx_handle()) {
		return;
	}
	if (!ubx_ctx_global || !ubx_ctx_global->ready) {
		ELOG(TAG, "[%s] ubx_ctx not ready", __FUNCTION__);
		return;
	}
	FUNC_ENTRY_ARGS(TAG, "group: %zu, index: %zu", group, index);
	switch (index) {
	case cfg_ubx_ubx_gnss:
	case cfg_ubx_ubx_output_rate:
		FUNC_ENTRY_ARGS(TAG, "changed rate=%d gnss=%u",
						g_rtc_config.ubx.output_rate, g_rtc_config.ubx.gnss);
		// Post event to trigger async reconfiguration instead of blocking
		esp_event_post(UBX_EVENT, UBX_EVENT_CONFIG_CHANGED, NULL, 0,
					   portMAX_DELAY);
		break;
	case cfg_ubx_ubx_nav_mode:
		FUNC_ENTRY_ARGS(TAG, "changed nav_mode=%d", g_rtc_config.ubx.nav_mode);
		// Post event to trigger async nav mode change instead of blocking
		esp_event_post(UBX_EVENT, UBX_EVENT_NAV_MODE_CHANGED, NULL, 0,
					   portMAX_DELAY);
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
			ESP_LOGW(TAG, "[%s] uart_set_pin failed", __FUNCTION__);
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
		esp_event_post(UBX_EVENT,
					   !ret ? UBX_EVENT_UART_INIT_DONE
							: UBX_EVENT_UART_INIT_FAIL,
					   NULL, 0, portMAX_DELAY);
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
			esp_event_post(UBX_EVENT, UBX_EVENT_UART_DEINIT_DONE, NULL, 0,
						   portMAX_DELAY);
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
	IMEAS_END(TAG);
	return ret;
}

static esp_err_t fix_config(ubx_ctx_t *ubx_ctx) {
	if (ubx_ctx == NULL)
		return ESP_ERR_INVALID_ARG;
	uint8_t gnss = g_rtc_config.ubx.gnss;
	uint8_t gnss_count = 1;
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
	if (g_rtc_config.ubx.output_rate == 0 ||
		g_rtc_config.ubx.output_rate > UBX_OUTPUT_20HZ) {
		FUNC_ENTRY_ARGSD(
			TAG, "Invalid output_rate %d, setting to default UBX_OUTPUT_5HZ",
			g_rtc_config.ubx.output_rate);
		g_rtc_config.ubx.output_rate = UBX_OUTPUT_5HZ;
	}
	if (ubx_ctx->hw_type == UBX_TYPE_M8) {
		if (ubx_ctx->gnss_count >= 2 &&
			g_rtc_config.ubx.output_rate > UBX_OUTPUT_10HZ) {
			FUNC_ENTRY_ARGSD(TAG,
							 "2 gnss, output rate > 10hz, fallback to 10hz");
			g_rtc_config.ubx.output_rate = UBX_OUTPUT_10HZ;
		} else if (ubx_ctx->gnss_count == 1 &&
				   g_rtc_config.ubx.output_rate > UBX_OUTPUT_5HZ) {
			FUNC_ENTRY_ARGSD(TAG, "1 gnss, output rate > 5hz, fallback to 5hz");
			g_rtc_config.ubx.output_rate = UBX_OUTPUT_5HZ;
		}
	}
	if (ubx_ctx->hw_type == UBX_TYPE_M10) {
		if (ubx_ctx->gnss_count == 4 &&
			g_rtc_config.ubx.output_rate > UBX_OUTPUT_10HZ) {
			FUNC_ENTRY_ARGSD(TAG,
							 "4 gnss, output rate > 10hz, fallback to 10hz");
			g_rtc_config.ubx.output_rate = UBX_OUTPUT_10HZ;
		} else if (ubx_ctx->gnss_count == 3 &&
				   g_rtc_config.ubx.output_rate > UBX_OUTPUT_16HZ) {
			FUNC_ENTRY_ARGSD(TAG,
							 "3 gnss, output rate > 16hz, fallback to 16hz");
			g_rtc_config.ubx.output_rate = UBX_OUTPUT_16HZ;
		} else if (ubx_ctx->gnss_count == 2 &&
				   g_rtc_config.ubx.output_rate > UBX_OUTPUT_20HZ) {
			FUNC_ENTRY_ARGSD(TAG,
							 "2 gnss, output rate > 20hz, fallback to 20hz");
			g_rtc_config.ubx.output_rate = UBX_OUTPUT_20HZ;
		}
	}
	return ESP_OK;
}

esp_err_t ubx_set_gnss_and_rate(ubx_ctx_t *ubx_ctx, uint8_t gnss,
								uint8_t rate) {
	FUNC_ENTRY(TAG);
	if (ubx_ctx == NULL)
		return ESP_ERR_INVALID_ARG;
	esp_err_t ret = ESP_OK;
	uint8_t try, max_tries = 3;
	fix_config(ubx_ctx);
	// Update rate parameter with the fixed value from config
	rate = g_rtc_config.ubx.output_rate;
	// Ensure rate is valid
	if (rate == 0 || rate > UBX_OUTPUT_20HZ) {
		WLOG(TAG, "[%s] Invalid rate %d from config, using UBX_OUTPUT_5HZ",
			 __func__, rate);
		rate = UBX_OUTPUT_5HZ;
		g_rtc_config.ubx.output_rate = rate;
	}
	FUNC_ENTRY_ARGSD(TAG, "fix done, gnss: %" PRIu8 ", rate: %" PRIu8 "", gnss,
					 rate);
	// if(g_rtc_config.ubx.msgout_sat){
	for (try = 0; try <= max_tries; ++try) {
		if (ubx_ctx->shutdown_requested)
			goto fail;
		ret = ubx_set_msgout_sat(ubx_ctx);
		if (ret == ESP_OK)
			break;
	}
	if (ret != ESP_OK) {
		ELOG(TAG, "[%s] ubx_set_msgout_sat failed", __func__);
	}
	// }
	FUNC_ENTRY_ARGSD(TAG, "msgout_sat done");
	for (try = 0; try <= max_tries; ++try) {
		if (ubx_ctx->shutdown_requested)
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
		if (ubx_ctx->shutdown_requested)
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
	esp_event_post(UBX_EVENT, UBX_EVENT_SAMPLE_RATE_CHANGED, 0, 0,
				   portMAX_DELAY);
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
	printf("HW type: %s (%d)\n",
		   ubx_ctx->hw_type < sizeof(ubx_hw_type_strings) / sizeof(char *)
			   ? ubx_hw_type_strings[ubx_ctx->hw_type]
			   : "Unknown",
		   ubx_ctx->hw_type);
	printf("HW id: %s\n", (char *)ubx_ctx->hw_id);
	printf("Baud rate: %s (%" PRIu32 ")\n",
		   g_rtc_config.ubx.baud <
				   sizeof(ubx_baud_rate_strings) / sizeof(char *)
			   ? ubx_baud_rate_strings[g_rtc_config.ubx.baud]
			   : "Unknown",
		   g_rtc_config.ubx.baud);
	printf("GNSS: %" PRIu8 " (count: %" PRIu8 ")\n", g_rtc_config.ubx.gnss,
		   ubx_ctx->gnss_count);
	printf("Output rate: %" PRIu8 " Hz\n",
		   g_rtc_config.ubx.output_rate == 0
			   ? 0
			   : (uint8_t)(1000 / HZ_TO_MS(g_rtc_config.ubx.output_rate)));
	printf("Nav mode: %d\n", g_rtc_config.ubx.nav_mode);
	printf("Message out sat: %d\n", g_rtc_config.ubx.msgout_sat);

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
			if (ubx_ctx->shutdown_requested)                                   \
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

	// Critical operations - setup fails if these don't work
	ret = ubx_on(ubx_ctx);
	if (ret != ESP_OK) {
		ELOG(TAG, "[%s] ubx_on failed", __FUNCTION__);
		goto fail;
	}

	ret = ubx_initial_read(ubx_ctx, false);
	if (ret != ESP_OK) {
		ELOG(TAG, "[%s] ubx_initial_read failed", __FUNCTION__);
		goto fail;
	}

	uint8_t try, max_tries = 3;

	UBX_SETUP_TRY_OP(ubx_get_hw_version(ubx_ctx), "ubx_get_hw_version", true);
	UBX_SETUP_TRY_OP(ubx_set_prot_msg_out(ubx_ctx, false, true),
					 "ubx_set_prot_msg_out", true);

	// Non-critical operations - log errors but continue
	UBX_SETUP_TRY_OP(ubx_set_nav_mode(ubx_ctx, g_rtc_config.ubx.nav_mode),
					 "ubx_set_nav_mode", false);
	UBX_SETUP_TRY_OP(ubx_set_msgout(ubx_ctx), "ubx_set_msgout", false);

	// Critical GNSS setup
	ret = ubx_set_gnss_and_rate(ubx_ctx, g_rtc_config.ubx.gnss,
								g_rtc_config.ubx.output_rate);
	if (ret != ESP_OK) {
		ELOG(TAG, "[%s] ubx_set_gnss_and_rate failed", __FUNCTION__);
		goto fail;
	}

	FUNC_ENTRY_ARGSD(TAG, " ubx_ctx->hw_id: %s, ubx_ctx->hw_type: %d",
					 &ubx_ctx->hw_id[0], ubx_ctx->hw_type);

	UBX_SETUP_TRY_OP(ubx_get_hw_id(ubx_ctx), "ubx_get_hw_id", false);
	UBX_SETUP_TRY_OP(ubx_get_gnss(ubx_ctx), "ubx_get_gnss", true);

	if (!ubx_ctx->shutdown_requested) {
		esp_event_post(UBX_EVENT, UBX_EVENT_SETUP_DONE, NULL, 0, portMAX_DELAY);
		if (!ret) {
			WLOG(TAG, "[%s] setup done, device ready!", __func__);
			ubx_ctx->ready = true;
			ubx_ctx->ready_time = get_millis();
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
	esp_err_t ret = ESP_OK;
	FUNC_ENTRY_ARGS(TAG, "going to set nav mode: %d", nav_mode);
	ret = ubx_cfg_valset(
		ubx, (const uint8_t[]){0x1c, 0x00, 0x11, 0x20, (uint8_t)nav_mode}, 5,
		true);
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

static esp_err_t ubx_set_prot_msg_out(ubx_ctx_t *ubx, bool enable_nmea,
									  bool enable_ubx) {
	FUNC_ENTRY(TAG);
	if (ubx == NULL)
		return ESP_ERR_INVALID_ARG;
	esp_err_t ret = ESP_OK;
	if (!enable_nmea && !enable_ubx)
		enable_ubx = true;
	FUNC_ENTRY_ARGSD(TAG, "going to enable_nmea: %u, enable_ubx: %u",
					 enable_nmea, enable_ubx);
	ret = ubx_cfg_valset(
		ubx,
		(const uint8_t[]){0x02, 0x00, 0x74, 0x10, enable_nmea ? 0x01 : 0x00,
						  0x01, 0x00, 0x74, 0x10, enable_ubx ? 0x01 : 0x00},
		10, true);
	if (!ret) {
		FUNC_ENTRY_ARGSD(TAG, "message protocol set to %s",
						 enable_nmea && enable_ubx ? "NMEA and UBX"
						 : enable_nmea			   ? "NMEA"
												   : "UBX");
		return ret;
	}
	// fallback hw m8 and below
	return send_ubx_cfg_msg(
		ubx, CLS_CFG, CFG_NAV5,
		(const uint8_t[]){/* portID, reserved1 */ 0x01,
						  0x00,
						  /* txReady x2 */ 0x00,
						  0x00,
						  /* mode x4 */ 0xd0,
						  0x08,
						  0x00,
						  0x00,
						  /* baudRate u4 */ 0x80,
						  0x25,
						  0x00,
						  0x00,
						  /* inProtoMask x2 */ 0x23,
						  0x00,
						  /* outProtoMask x2 */ enable_nmea && enable_ubx ? 0x03
						  : enable_nmea									  ? 0x02
										: 0x01,
						  0x00,
						  /* flags x2 */ 0x00,
						  0x00,
						  /* reserved3 u2 */ 0x00,
						  0x00},
		20, true);
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
	uint8_t output_vec[4] = {0, 0, 0, 0};
	encode_uint32(output_vec, baud);
	ret = ubx_cfg_valset(ubx,
						 (const uint8_t[]){0x01, 0x00, 0x52, 0x40,
										   output_vec[0], output_vec[1],
										   output_vec[2], output_vec[3]},
						 8, false);
	if (!ret)
		goto done;
#if (C_LOG_LEVEL <= LOG_INFO_NUM)
	else {
		WLOG(TAG, "[%s] fallback to old cfg_msg as valset failed ...",
			 __FUNCTION__);
	}
#endif // fallback hw m8 and below
	ret = send_ubx_cfg_msg(ubx, CLS_CFG, CFG_PRT,
						   (const uint8_t[]){/* portID, reserved1 */ 0x01,
											 0x00,
											 /* txReady x2 */ 0x00,
											 0x00,
											 /* mode x4 */ 0xd0,
											 0x08,
											 0x00,
											 0x00,
											 /* baudRate u4 */ output_vec[0],
											 output_vec[1],
											 output_vec[2],
											 output_vec[3],
											 /* inProtoMask x2 */ 0x23,
											 0x00,
											 /* outProtoMask x2 */ 0x03,
											 0x00,
											 /* flags x2 */ 0x00,
											 0x00,
											 /* reserved3 u2 */ 0x00,
											 0x00},
						   20, false);
done:
	g_rtc_config.ubx.baud = baud;
	ret = ubx_uart_set_baud(ubx);
	return ESP_OK;
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
	uint8_t output_vec[2] = {0, 0};
	int baud = UBX_BAUD_38400;

	encode_uint16(&(output_vec[0]), HZ_TO_MS(rate));
	if (rate > UBX_OUTPUT_10HZ) {
		baud = UBX_BAUD_230400;
	} else if (rate > UBX_OUTPUT_2HZ) {
		baud = UBX_BAUD_115200;
	} else {
		baud = UBX_BAUD_38400;
	}
	FUNC_ENTRY_ARGSD(TAG, "solutions:%" PRIu8 " output rate: %u, baud: %d",
					 ubx->gnss_count, rate, baud);
	ret = ubx_cfg_valset(
		ubx,
		(const uint8_t[]){0x01, 0x00, 0x21, 0x30, output_vec[0], output_vec[1]},
		6, true);
	if (!ret)
		goto done;
#if (C_LOG_LEVEL <= LOG_INFO_NUM)
	else {
		WLOG(TAG, "[%s] fallback to old cfg_msg as valset failed ...",
			 __FUNCTION__);
	}
#endif // fallback hw m8 and below
	ret = send_ubx_cfg_msg(ubx, CLS_CFG, CFG_RATE,
						   (const uint8_t[]){/* measRate 2b */ output_vec[0],
											 output_vec[1],
											 /* navRate always 1 */ 0x01, 0x00,
											 /* timeRef UTC */ 0x01, 0x00},
						   6, true);
done:
	if (!ret)
		ret = ubx_set_uart_baud_rate(ubx, baud);
	return ret;
}

static esp_err_t ubx_set_gnss(ubx_ctx_t *ubx, uint8_t mode) {
	FUNC_ENTRY(TAG);
	uint8_t enable_gps = 0x01;	   // us gps
	uint8_t enable_sbas = 0x01;	   // us sbas
	uint8_t enable_galileo = 0x00; // eu galileo
	uint8_t enable_beidou = 0x00;  // cn beidou
	uint8_t enable_qzss = 0x01;	   // jp qzss
	uint8_t enable_glonass = 0x00; // ru glonass

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
		ELOG(TAG, "[%s] count_solutions < 1, fallback to gps", __FUNCTION__);
		enable_gps = 1;
	} else if (ubx->gnss_count > 4) {
		ELOG(TAG,
			 "[%s] count_solutions > 4, fallback to gps+galileo+glonass+beidou",
			 __FUNCTION__);
		enable_gps = 0x01;
		enable_galileo = 0x01;
		enable_glonass = 0x01;
		enable_beidou = 0x01;
	}
	uint8_t gnss_cmd[64] = {
		/* gps     */ 0x1f, 0x00, 0x31, 0x10, enable_gps,
		/* sbas    */ 0x20, 0x00, 0x31, 0x10, enable_sbas,
		/* galileo */ 0x21, 0x00, 0x31, 0x10, enable_galileo,
		/* beidou  */ 0x22, 0x00, 0x31, 0x10, enable_beidou,
		/* qzss    */ 0x24, 0x00, 0x31, 0x10, enable_qzss,
		/* glonass */ 0x25, 0x00, 0x31, 0x10, enable_glonass};
	uint8_t gnss_cursor = 6 * 5;
	uint8_t tmp[] = {0x00, 0x31, 0x10, 0x01};
	if (enable_gps) {
		gnss_cmd[gnss_cursor++] = 0x01; // enable gps l1c/a
		memcpy(&gnss_cmd[gnss_cursor], tmp, 4), gnss_cursor += 4;
	}
	if (enable_galileo) {
		gnss_cmd[gnss_cursor++] = 0x07; // enable galileo e1
		memcpy(&gnss_cmd[gnss_cursor], tmp, 4), gnss_cursor += 4;
	}
	if (enable_glonass) {
		gnss_cmd[gnss_cursor++] = 0x18; // enable glonass l1of
		memcpy(&gnss_cmd[gnss_cursor], tmp, 4), gnss_cursor += 4;
	}
	if (enable_beidou) {
		if (ubx->hw_type <= UBX_TYPE_M9 || !enable_glonass) {
			gnss_cmd[gnss_cursor++] = 0x0d; // enable beidou b1l
			memcpy(&gnss_cmd[gnss_cursor], tmp, 4), gnss_cursor += 4;
		} else {
			gnss_cmd[gnss_cursor++] = 0x0d; // disable beidou b1l
			memcpy(&gnss_cmd[gnss_cursor], tmp, 3), gnss_cursor += 3;
			gnss_cmd[gnss_cursor++] = 0x00;
			gnss_cmd[gnss_cursor++] = 0x0f; // enable beidou b1c
			memcpy(&gnss_cmd[gnss_cursor], tmp, 4), gnss_cursor += 4;
		}
	}
	FUNC_ENTRY_ARGS(TAG,
					" mode:%" PRIu8 ", gps(us): %" PRIu8 ", sbas(us): %" PRIu8
					" galileo(eu): %" PRIu8 ", beidou(cn): %" PRIu8
					", glonass(ru): %" PRIu8 ", qzss(jp): %" PRIu8 "",
					mode, enable_gps, enable_sbas, enable_galileo,
					enable_beidou, enable_glonass, enable_qzss);
	esp_err_t ret = ubx_cfg_valset(ubx, gnss_cmd, gnss_cursor, true);
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
			/* gps     */ 0x00, 0x08, 0x10, 0x00, enable_gps, 0x00, 0x01, 0x01,
			/* sbas    */ 0x01, 0x01, 0x03, 0x00, enable_sbas, 0x00, 0x01, 0x01,
			/* galileo */ 0x02, 0x04, 0x08, 0x00, enable_galileo, 0x00, 0x01,
			0x01,
			/* beidou  */ 0x03, 0x08, 0x10, 0x00, enable_beidou, 0x00, 0x01,
			0x01,
			/* qzss    */ 0x05, 0x00, 0x03, 0x00, enable_qzss, 0x00, 0x01, 0x01,
			/* glonass */ 0x06, 0x08, 0x0E, 0x00, enable_glonass, 0x00, 0x01,
			0x01},
		8 * 6 + 4, true);
}

static esp_err_t ubx_set_msgout(ubx_ctx_t *ubx) {
	FUNC_ENTRY(TAG);
	esp_err_t ret = ESP_OK;
	uint8_t cfg_pvt_id = 0x07;
	uint8_t cfg_dop_id = 0x04;
	if (ubx->hw_type >= UBX_TYPE_M9) {
		cfg_pvt_id = 0x07;
		cfg_dop_id = 0x39;
	}
	FUNC_ENTRY_ARGS(TAG, " enable navpvt and navdop ubx messages.");
	ret = ubx_cfg_valset(ubx,
						 (const uint8_t[]){
							 /* pvt id, cfg value */ cfg_pvt_id,
							 0x00,
							 0x91,
							 0x20,
							 0x01,
							 /* dop id, cfg value */ cfg_dop_id,
							 0x00,
							 0x91,
							 0x20,
							 0x01,
						 },
						 10, true);
	if (!ret)
		return ret;
#if (C_LOG_LEVEL <= LOG_INFO_NUM)
	else {
		WLOG(TAG, "[%s] fallback to old cfg_msg as valset failed ...",
			 __FUNCTION__);
	}
#endif
	// fallback hw m8 and below
	ret = send_ubx_cfg_msg(ubx, CLS_CFG, CFG_MSG,
						   (const uint8_t[]){
							   /* msgClass, msgID */ 0x01,
							   cfg_pvt_id,
							   /* rate port 0 i2c */ 0x00,
							   /* rate port 1, 2 serial */ 0x01,
							   0x00,
							   /* rate port 3 usb, 4 spi, 5 reserved  */ 0x00,
							   0x00,
							   0x00,
						   },
						   8, true);
	if (ret != ESP_OK)
		return ret;
	ret = send_ubx_cfg_msg(
		ubx, CLS_CFG, CFG_MSG,
		(const uint8_t[]){/* msgClass, msgID */ 0x01, cfg_dop_id,
						  /* rate port 0 i2c */ 0x00,
						  /* rate port 1, 2 serial */ 0x01, 0x00,
						  /* rate port 3 usb, 4 spi, 5 reserved  */ 0x00, 0x00,
						  0x00},
		8, true);
	return ret;
}

static esp_err_t ubx_set_msgout_sat(ubx_ctx_t *ubx) {
	FUNC_ENTRY(TAG);
	/* Send rate is relative to the event a message is registered on.
	For example, if the rate of a navigation message is set to 2,
	the message is sent every second navigation solution.
	For configuring NMEA messages, the section NMEA Messages
	Overview describes class and identifier numbers used. */
	uint8_t cfg_rate =
		(((uint8_t)g_rtc_config.ubx.output_rate) & 0xff); // once in a second
	uint8_t cfg_sat_id = 0x16;
	esp_err_t ret =
		ubx_cfg_valset(ubx,
					   (const uint8_t[]){/* sat id, cfg value */ cfg_sat_id,
										 0x00, 0x91, 0x20, cfg_rate},
					   5, true);
	if (!ret)
		return ret;
#if (C_LOG_LEVEL <= LOG_INFO_NUM)
	else {
		WLOG(TAG, "[%s] ubx_cfg_valset failed, fallback to old cfg_msg.",
			 __FUNCTION__);
	}
#endif
	// fallback hw m8 and below
	return send_ubx_cfg_msg(
		ubx, CLS_CFG, CFG_MSG,
		(const uint8_t[]){/* msgClass, msgID */ 0x01, cfg_sat_id,
						  /* rate port 0 i2c */ 0x00,
						  /* rate port 1, 2 serial */ cfg_rate, 0x00,
						  /* rate port 3 usb, 4 spi, 5 reserved  */ 0x00, 0x00,
						  0x00},
		8, true);
}

static esp_err_t ubx_uart_save_cfg(ubx_ctx_t *ubx) {
	FUNC_ENTRY(TAG);
	esp_err_t ret = send_ubx_cfg_msg(
		ubx, CLS_CFG, CFG_CFG,
		(const uint8_t[]){0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00,
						  0x00, 0x00, 0x00, 0x1c},
		13, true);
	return ret;
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
	uint32_t ubx_baud_rate_temp[6] = {0, 0, 0, 0, 0, 0};
	for (uint8_t i = 0, j = lengthof(ubx_baud_rates), k; i <= j; ++i, k = 0) {
		ubx_baud_rate_temp[i] = g_rtc_config.ubx.baud;
		if (i > 0) {
			if (!ubx_baud_rates[i - 1])
				goto next;
			while (k < 6) {
				if (ubx_baud_rate_temp[k] == ubx_baud_rates[i - 1]) {
					goto next;
				}
				++k;
			}
			g_rtc_config.ubx.baud = ubx_baud_rates[i - 1];
			ret = ubx_uart_set_baud(ubx);
#if (C_LOG_LEVEL <= LOG_DEBUG_NUM)
			if (ret != ESP_OK) {
				WLOG(TAG, "[%s] ubx_uart_set_baud failed: %s", __FUNCTION__,
					 esp_err_to_name(ret));
			}
#endif
			delay_ms(50);
		}
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
				FUNC_ENTRY_ARGSD(
					TAG, "found UBX message at %d with baud: %" PRIu32 "",
					p - (char *)ubx_packet->msg, g_rtc_config.ubx.baud);
				return ESP_OK;
				break;
			} else if (*p == '$' && *(p + 1) == 'G') {
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
			if (i <= j) {
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
