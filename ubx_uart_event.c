// UART event-driven reading implementation for UBX GPS module
// Circular buffer helpers and UART event handler task

#include "ubx_private.h"
#include "ubx.h"

#if defined(CONFIG_UBLOX_ENABLED)

static const char *TAG = "ubx_uart_event";

#if defined(CONFIG_UBX_TIMER_STATS_ENABLED)
// UART-level statistics: Track ALL UBX headers received from UART (before protocol decoding)
static size_t rx_buf_used_max = 0;
static size_t rx_buf_used_min = SIZE_MAX;
#endif

#define UBX_RX_BUF_SIZE 2048  // Larger circular buffer to absorb bursts (2KB for 20Hz+ GPS)
#define UBX_UART_QUEUE_SIZE 20
#define UBX_UART_TMP_BUF_SIZE 1024  // Temporary buffer for UART reads
#define UBX_MSG_READY_DEPTH 32       // Counting semaphore depth for msg_ready
#define UBX_HDR_A 0xB5
#define UBX_HDR_B 0x62
#define UBX_LINK_LOSS_MS 3000       // Consider link stale if no valid frames in this window
#define UBX_RX_BACKPRESSURE_PCT 75  // Flush when stale link and buffer exceeds this percent

// Circular buffer helper: available bytes
inline size_t ubx_rx_buf_available(ubx_ctx_t *ctx) {
    if (ctx->rx_buf_head >= ctx->rx_buf_tail) {
        return ctx->rx_buf_head - ctx->rx_buf_tail;
    } else {
        return ctx->rx_buf_size - ctx->rx_buf_tail + ctx->rx_buf_head;
    }
}

// Circular buffer helper: free space
inline size_t ubx_rx_buf_free_space(ubx_ctx_t *ctx) {
    return ctx->rx_buf_size - ubx_rx_buf_available(ctx) - 1;
}

// Peek a byte at logical offset from tail without consuming (caller holds mutex)
static inline uint8_t ubx_rx_peek(ubx_ctx_t *ctx, size_t offset) {
    size_t pos = (ctx->rx_buf_tail + offset) % ctx->rx_buf_size;
    return ctx->rx_buffer[pos];
}

// Detect if a complete UBX frame is present in buffer (caller may or may not hold mutex)
bool ubx_rx_has_complete_frame(ubx_ctx_t *ctx) {
    bool has = false;
    if (!ctx || !ctx->rx_buffer) return false;

    // Minimal timeout for high-rate GPS (1-30Hz)
    // At 30Hz (33ms period), even 1ms = 3% of budget
    // False negative is safe - GPS task will retry next iteration
    if (xSemaphoreTake(ctx->rx_buf_mutex, pdMS_TO_TICKS(1)) != pdTRUE) {
        return false;  // Conservative: assume no frame if can't quickly check
    }

    size_t avail = ubx_rx_buf_available(ctx);
    if (avail < 6) {
        goto out;
    }

    // Scan buffer for UBX header and length
    for (size_t off = 0; off + 6 <= avail; off++) {
        if (ubx_rx_peek(ctx, off) != UBX_HDR_A) continue;
        if (ubx_rx_peek(ctx, off + 1) != UBX_HDR_B) continue;

        // Need length bytes
        if (off + 6 > avail) break;
        uint16_t payload_len = (uint16_t)ubx_rx_peek(ctx, off + 4) | ((uint16_t)ubx_rx_peek(ctx, off + 5) << 8);
        size_t total = 6u /*cls/id/len*/ + payload_len + 2u /*ck*/;

        if (off + total <= avail) {
            has = true;
            break;
        }
    }

out:
    xSemaphoreGive(ctx->rx_buf_mutex);
    return has;
}

// Read from circular buffer into destination (non-blocking, returns bytes read)
size_t ubx_rx_buf_read(ubx_ctx_t *ctx, uint8_t *dst, size_t len, uint32_t timeout_ms) {
    if (!ctx || !ctx->rx_buffer || !dst || len == 0) return 0;
    
    uint32_t start = get_millis();
    size_t total_read = 0;
    
    while (total_read < len) {
        if (xSemaphoreTake(ctx->rx_buf_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            size_t available = ubx_rx_buf_available(ctx);
            if (available > 0) {
                size_t to_read = (len - total_read) < available ? (len - total_read) : available;
                size_t tail = ctx->rx_buf_tail;
                size_t buf_size = ctx->rx_buf_size;
                
                // Optimized copy: handle wrap-around in two chunks max
                if (tail + to_read <= buf_size) {
                    // Contiguous read - no wrap
                    memcpy(&dst[total_read], &ctx->rx_buffer[tail], to_read);
                    ctx->rx_buf_tail = (tail + to_read) % buf_size;
                } else {
                    // Wrapped read - two memcpy calls
                    size_t first_chunk = buf_size - tail;
                    memcpy(&dst[total_read], &ctx->rx_buffer[tail], first_chunk);
                    memcpy(&dst[total_read + first_chunk], &ctx->rx_buffer[0], to_read - first_chunk);
                    ctx->rx_buf_tail = to_read - first_chunk;
                }
                total_read += to_read;
            }
            xSemaphoreGive(ctx->rx_buf_mutex);
            
            if (total_read >= len) break;
        }
        
        // Check timeout
        if (timeout_ms > 0 && (get_millis() - start) >= timeout_ms) {
            break;
        }
        
        // Yield briefly if we haven't read enough yet
        if (total_read < len) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    
    return total_read;
}

// UART event handler task
static void ubx_uart_event_task(void *arg) {
    ubx_ctx_t *ctx = (ubx_ctx_t *)arg;
    uart_event_t event;
    uint8_t *dtmp = (uint8_t *)malloc(UBX_UART_TMP_BUF_SIZE);
    
    if (!dtmp) {
        ELOG(TAG, "[%s] Failed to allocate temp buffer", __FUNCTION__);
        vTaskDelete(NULL);
        return;
    }
    
    ILOG(TAG, "[%s] UART event task started", __FUNCTION__);
    
    while (ctx && ctx->uart_num >= 0) {
        // Wait for UART event (blocks until event or timeout)
        if (xQueueReceive(ctx->uart_event_queue, &event, pdMS_TO_TICKS(100))) {
            switch (event.type) {
                case UART_DATA:
                    // Read available data from UART
                    if (event.size > 0) {
                        int len = uart_read_bytes(ctx->uart_num, dtmp, 
                                                 event.size > UBX_UART_TMP_BUF_SIZE ? UBX_UART_TMP_BUF_SIZE : event.size, 
                                                 pdMS_TO_TICKS(10));
                        
                        if (len > 0) {
                            // Try to acquire mutex with longer timeout to avoid dropping data
                            if (xSemaphoreTake(ctx->rx_buf_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                                // Write to circular buffer
                                size_t free_space = ubx_rx_buf_free_space(ctx);
                                if ((size_t)len > free_space) {
                                    size_t drop = (size_t)len - free_space;
                                    WLOG(TAG, "[%s] RX buffer overflow: dropping %zu bytes (avail=%zu used=%zu)",
                                         __FUNCTION__, drop, free_space, ubx_rx_buf_available(ctx));
                                    len = free_space;
                                }

                                size_t head = ctx->rx_buf_head;
                                size_t buf_size = ctx->rx_buf_size;

                                // Write in at most two chunks to handle wrap-around efficiently
                                size_t first_chunk = len;
                                if (head + first_chunk > buf_size) {
                                    first_chunk = buf_size - head;
                                }
                                size_t second_chunk = len - first_chunk;

                                memcpy(&ctx->rx_buffer[head], dtmp, first_chunk);
                                if (second_chunk > 0) {
                                    memcpy(&ctx->rx_buffer[0], dtmp + first_chunk, second_chunk);
                                }

                                ctx->rx_buf_head = (head + len) % buf_size;
                                ctx->last_rx_ms = get_millis();
                                size_t used = ubx_rx_buf_available(ctx);
                                size_t high_water = (ctx->rx_buf_size * UBX_RX_BACKPRESSURE_PCT) / 100;

#if defined(CONFIG_UBX_TIMER_STATS_ENABLED)
                                // Track buffer usage for stats
                                if (used > rx_buf_used_max) rx_buf_used_max = used;
                                if (used < rx_buf_used_min) rx_buf_used_min = used;
#endif

                                // If we have not seen a valid frame for a while and the buffer is filling, drop stale bytes
                                if (ctx->last_valid_ms && used > high_water && (ctx->last_rx_ms - ctx->last_valid_ms) > UBX_LINK_LOSS_MS) {
                                    WLOG(TAG, "[%s] RX link stale, flushing buffer (used=%zu/%zu)", __FUNCTION__, used, ctx->rx_buf_size);
                                    ctx->rx_buf_head = 0;
                                    ctx->rx_buf_tail = 0;
                                    ctx->link_lost = true;
                                    used = 0;
                                }
                                
                                xSemaphoreGive(ctx->rx_buf_mutex);

                                // Always signal consumer when data is written (even if frame check fails due to mutex)
                                // GPS task will do its own frame check with proper mutex handling
                                // This prevents deadlock where UART can't check frame (mutex busy) so doesn't signal,
                                // but GPS is waiting for signal to check buffer
                                if (ctx->msg_ready) {
                                    xSemaphoreGive(ctx->msg_ready);
                                }
                            } else {
                                // Mutex contention - consumer is holding buffer too long
                                WLOG(TAG, "[%s] Mutex timeout: dropping %d bytes (consumer blocked?)", 
                                     __FUNCTION__, len);
                            }
                        }
                    }
                    break;
                    
                case UART_FIFO_OVF:
                    WLOG(TAG, "[%s] UART FIFO overflow", __FUNCTION__);
                    uart_flush_input(ctx->uart_num);
                    xQueueReset(ctx->uart_event_queue);
                    break;
                    
                case UART_BUFFER_FULL:
                    WLOG(TAG, "[%s] UART ring buffer full", __FUNCTION__);
                    uart_flush_input(ctx->uart_num);
                    xQueueReset(ctx->uart_event_queue);
                    break;
                    
                case UART_BREAK:
                case UART_PARITY_ERR:
                case UART_FRAME_ERR:
                    // Silently ignore - GPS data can trigger false positives
                    break;
                    
                default:
                    break;
            }
        }
    }
    
    free(dtmp);
    ctx->uart_event_task = NULL;
    ILOG(TAG, "[%s] UART event task stopped", __FUNCTION__);
    vTaskDelete(NULL);
}

// Initialize event-driven UART infrastructure
esp_err_t ubx_uart_event_init(ubx_ctx_t *ctx) {
    FUNC_ENTRY(TAG);
    
    if (!ctx) return ESP_ERR_INVALID_ARG;
    
    // Check if already initialized
    if (ctx->uart_event_task || ctx->rx_buffer || ctx->rx_buf_mutex) {
        WLOG(TAG, "[%s] Already initialized, skipping", __FUNCTION__);
        return ESP_OK;
    }
    
    // Allocate circular buffer
    ctx->rx_buf_size = UBX_RX_BUF_SIZE;
    ctx->rx_buffer = (uint8_t *)heap_caps_malloc(ctx->rx_buf_size, MALLOC_CAP_8BIT);
    if (!ctx->rx_buffer) {
        ELOG(TAG, "[%s] Failed to allocate RX buffer", __FUNCTION__);
        return ESP_ERR_NO_MEM;
    }
    
    ctx->rx_buf_head = 0;
    ctx->rx_buf_tail = 0;
    
    // Create mutex for buffer access
    ctx->rx_buf_mutex = xSemaphoreCreateMutex();
    if (!ctx->rx_buf_mutex) {
        ELOG(TAG, "[%s] Failed to create mutex", __FUNCTION__);
        free(ctx->rx_buffer);
        ctx->rx_buffer = NULL;
        return ESP_ERR_NO_MEM;
    }
    
    // Create counting semaphore for message ready notification (prevents loss when many frames arrive)
    ctx->msg_ready = xSemaphoreCreateCounting(UBX_MSG_READY_DEPTH, 0);
    if (!ctx->msg_ready) {
        ELOG(TAG, "[%s] Failed to create msg_ready semaphore", __FUNCTION__);
        vSemaphoreDelete(ctx->rx_buf_mutex);
        free(ctx->rx_buffer);
        ctx->rx_buffer = NULL;
        ctx->rx_buf_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }
    
    // UART event queue already created during uart_driver_install
    // Just create the event handler task
    BaseType_t ret = xTaskCreatePinnedToCore(
        ubx_uart_event_task,
        "ubx_uart_evt",
        4096,
        ctx,
        20,  // High priority for event handling
        &ctx->uart_event_task,
        1   // Core 1 to separate from WiFi on Core 0
    );
    
    if (ret != pdPASS) {
        ELOG(TAG, "[%s] Failed to create UART event task", __FUNCTION__);
        vSemaphoreDelete(ctx->rx_buf_mutex);
        free(ctx->rx_buffer);
        ctx->rx_buffer = NULL;
        ctx->rx_buf_mutex = NULL;
        return ESP_FAIL;
    }
    
    ILOG(TAG, "[%s] Event-driven UART initialized (buffer: %zu bytes)", __FUNCTION__, ctx->rx_buf_size);
    return ESP_OK;
}

// Cleanup event-driven UART infrastructure
esp_err_t ubx_uart_event_deinit(ubx_ctx_t *ctx) {
    FUNC_ENTRY(TAG);
    
    if (!ctx) return ESP_ERR_INVALID_ARG;
    
    // Stop event task by marking uart_num as invalid
    if (ctx->uart_event_task) {
        int saved_uart_num = ctx->uart_num;
        ctx->uart_num = -1;  // Signal task to exit
        
        // Wait for task to self-delete
        uint32_t timeout = get_millis() + 1000;
        while (ctx->uart_event_task && get_millis() < timeout) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        ctx->uart_num = saved_uart_num;
        
        if (ctx->uart_event_task) {
            WLOG(TAG, "[%s] Force-deleting UART event task", __FUNCTION__);
            vTaskDelete(ctx->uart_event_task);
            ctx->uart_event_task = NULL;
        }
    }
    
    // Free resources
    if (ctx->msg_ready) {
        vSemaphoreDelete(ctx->msg_ready);
        ctx->msg_ready = NULL;
    }
    
    if (ctx->rx_buf_mutex) {
        vSemaphoreDelete(ctx->rx_buf_mutex);
        ctx->rx_buf_mutex = NULL;
    }
    
    if (ctx->rx_buffer) {
        free(ctx->rx_buffer);
        ctx->rx_buffer = NULL;
    }
    
    ctx->rx_buf_size = 0;
    ctx->rx_buf_head = 0;
    ctx->rx_buf_tail = 0;
    
    ILOG(TAG, "[%s] Event-driven UART cleaned up", __FUNCTION__);
    return ESP_OK;
}

#if defined(CONFIG_UBX_TIMER_STATS_ENABLED)
// Print circular buffer health statistics
void ubx_uart_print_buffer_stats(ubx_ctx_t *ctx) {
    if (!ctx) return;
    
    // Snapshot RX ring buffer usage (non-blocking if mutex busy)
    size_t buf_size = ctx->rx_buf_size;
    size_t buf_avail = 0;
    size_t buf_free = 0;
    if (ctx->rx_buf_mutex && xSemaphoreTake(ctx->rx_buf_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        size_t head = ctx->rx_buf_head;
        size_t tail = ctx->rx_buf_tail;
        if (head >= tail) {
            buf_avail = head - tail;
        } else {
            buf_avail = buf_size - tail + head;
        }
        buf_free = buf_size > 0 ? buf_size - buf_avail - 1 : 0;
        
        // Update tracking
        if (buf_avail > rx_buf_used_max) rx_buf_used_max = buf_avail;
        if (buf_avail < rx_buf_used_min) rx_buf_used_min = buf_avail;
        
        xSemaphoreGive(ctx->rx_buf_mutex);
    }
    float buf_used_pct = buf_size > 0 ? ((float)buf_avail * 100.0f) / (float)buf_size : 0.0f;
    float buf_used_max_pct = buf_size > 0 ? ((float)rx_buf_used_max * 100.0f) / (float)buf_size : 0.0f;
    
    printf("[UART] ========== UART BUFFER STATS ===========\n");
    printf("[UART] RX Buffer: size=%zu avail=%zu free=%zu used=%.1f%%\n",
        buf_size, buf_avail, buf_free, buf_used_pct);
    printf("[UART] Usage: max=%.1f%% min=%zu bytes\n",
        buf_used_max_pct, rx_buf_used_min);
    printf("[UART] ==========================================\n");
}
#else
void ubx_uart_print_buffer_stats(ubx_ctx_t *ctx) {}
#endif

#endif // CONFIG_UBLOX_ENABLED
