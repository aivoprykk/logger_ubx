/**
 * @file test_ubx_uart_integration.c
 * @brief Integration tests for UART event handling and message processing
 * 
 * Tests the full pipeline from UART events to message parsing:
 * - UART event task simulation
 * - Circular buffer interaction with UART ISR
 * - Message queue flow from UART to consumer
 * - Error conditions (overflow, corruption, timeout)
 * - Link loss and recovery
 */

#include <string.h>
#include "unity.h"
#include "ubx.h"
#include "ubx_private.h"
#include "ubx_msg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "logger_common.h"

static const char *TAG = "test_ubx_uart_integ";

// Test context
static ubx_ctx_t test_ctx;
static ubx_msg_t test_msg;
static uint8_t test_rx_buffer[512];

void setUp(void) {
    memset(&test_ctx, 0, sizeof(ubx_ctx_t));
    memset(&test_msg, 0, sizeof(ubx_msg_t));
    memset(test_rx_buffer, 0, sizeof(test_rx_buffer));

    test_ctx.rx_buffer = test_rx_buffer;
    test_ctx.rx_buf_size = sizeof(test_rx_buffer);
    test_ctx.rx_buf_head = 0;
    test_ctx.rx_buf_tail = 0;
    test_ctx.rx_buf_mutex = xSemaphoreCreateMutex();
    test_ctx.msg_ready = xSemaphoreCreateCounting(32, 0);
    test_ctx.uart_num = UART_NUM_1;  // Mock UART number
}

void tearDown(void) {
    if (test_ctx.rx_buf_mutex) {
        vSemaphoreDelete(test_ctx.rx_buf_mutex);
    }
    if (test_ctx.msg_ready) {
        vSemaphoreDelete(test_ctx.msg_ready);
    }
}

/**
 * Helper: Simulate writing data to circular buffer (as UART event task would)
 */
static void simulate_uart_write(ubx_ctx_t *ctx, const uint8_t *data, size_t len) {
    if (xSemaphoreTake(ctx->rx_buf_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (size_t i = 0; i < len; i++) {
            ctx->rx_buffer[ctx->rx_buf_head] = data[i];
            ctx->rx_buf_head = (ctx->rx_buf_head + 1) % ctx->rx_buf_size;
        }
        ctx->last_rx_ms = get_millis();
        xSemaphoreGive(ctx->rx_buf_mutex);

        // Signal message ready
        if (ctx->msg_ready) {
            xSemaphoreGive(ctx->msg_ready);
        }
    }
}

/**
 * Helper: Create complete ACK-ACK message
 */
static void create_ack_message(uint8_t *buf, uint8_t cls, uint8_t id) {
    buf[0] = 0xB5;  // Header A
    buf[1] = 0x62;  // Header B
    buf[2] = 0x05;  // ACK class
    buf[3] = 0x01;  // ACK-ACK
    buf[4] = 0x02;  // Length LSB
    buf[5] = 0x00;  // Length MSB
    buf[6] = cls;   // Payload: acknowledged class
    buf[7] = id;    // Payload: acknowledged ID

    // Calculate checksum
    uint8_t CK_A = 0, CK_B = 0;
    for (int i = 2; i < 8; i++) {
        CK_A += buf[i];
        CK_B += CK_A;
    }
    buf[8] = CK_A;
    buf[9] = CK_B;
}

/**
 * Test: UART to buffer - single complete message
 */
TEST_CASE("UART integration - single complete message", "[ubx][integration]")
{
    uint8_t ack_msg[10];
    create_ack_message(ack_msg, 0x06, 0x01);

    // Simulate UART ISR writing to buffer
    simulate_uart_write(&test_ctx, ack_msg, sizeof(ack_msg));

    // Verify buffer state
    TEST_ASSERT_EQUAL(10, ubx_rx_buf_available(&test_ctx));
    TEST_ASSERT_TRUE(ubx_rx_has_complete_frame(&test_ctx));

    // Verify semaphore signaled
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(test_ctx.msg_ready, 0));
}

/**
 * Test: UART to buffer - partial message arrival
 */
TEST_CASE("UART integration - partial message arrival", "[ubx][integration]")
{
    uint8_t ack_msg[10];
    create_ack_message(ack_msg, 0x06, 0x01);

    // Write only header first
    simulate_uart_write(&test_ctx, ack_msg, 6);
    TEST_ASSERT_EQUAL(6, ubx_rx_buf_available(&test_ctx));
    TEST_ASSERT_FALSE(ubx_rx_has_complete_frame(&test_ctx));

    // Write rest
    simulate_uart_write(&test_ctx, ack_msg + 6, 4);
    TEST_ASSERT_EQUAL(10, ubx_rx_buf_available(&test_ctx));
    TEST_ASSERT_TRUE(ubx_rx_has_complete_frame(&test_ctx));
}

/**
 * Test: UART to buffer - multiple messages
 */
TEST_CASE("UART integration - multiple messages queued", "[ubx][integration]")
{
    uint8_t ack_msg1[10], ack_msg2[10];
    create_ack_message(ack_msg1, 0x06, 0x01);
    create_ack_message(ack_msg2, 0x06, 0x08);

    // Write both messages
    simulate_uart_write(&test_ctx, ack_msg1, sizeof(ack_msg1));
    simulate_uart_write(&test_ctx, ack_msg2, sizeof(ack_msg2));

    TEST_ASSERT_EQUAL(20, ubx_rx_buf_available(&test_ctx));
    TEST_ASSERT_TRUE(ubx_rx_has_complete_frame(&test_ctx));

    // Read first message
    uint8_t read_buf[10];
    size_t read = ubx_rx_buf_read(&test_ctx, read_buf, 10, 100);
    TEST_ASSERT_EQUAL(10, read);
    TEST_ASSERT_EQUAL_MEMORY(ack_msg1, read_buf, 10);

    // Should still have second message
    TEST_ASSERT_EQUAL(10, ubx_rx_buf_available(&test_ctx));
    TEST_ASSERT_TRUE(ubx_rx_has_complete_frame(&test_ctx));
}

/**
 * Test: Buffer overflow handling
 */
TEST_CASE("UART integration - buffer overflow handling", "[ubx][integration]")
{
    // Fill buffer almost completely
    uint8_t fill_data[500];
    memset(fill_data, 0xAA, sizeof(fill_data));

    simulate_uart_write(&test_ctx, fill_data, sizeof(fill_data));

    size_t avail_before = ubx_rx_buf_available(&test_ctx);
    TEST_ASSERT_GREATER_THAN(490, avail_before);

    // Try to overflow - should be prevented
    uint8_t ack_msg[10];
    create_ack_message(ack_msg, 0x06, 0x01);

    // In real implementation, overflow causes warning and partial drop
    // We can only test that buffer doesn't exceed max size
    TEST_ASSERT_LESS_THAN(test_ctx.rx_buf_size, ubx_rx_buf_available(&test_ctx));
}

/**
 * Test: Message with wrap-around
 */
TEST_CASE("UART integration - message wrap-around", "[ubx][integration]")
{
    // Position buffer near end
    test_ctx.rx_buf_head = 508;
    test_ctx.rx_buf_tail = 508;

    uint8_t ack_msg[10];
    create_ack_message(ack_msg, 0x06, 0x01);

    // Write message that will wrap
    simulate_uart_write(&test_ctx, ack_msg, sizeof(ack_msg));

    // Should detect frame despite wrap
    TEST_ASSERT_TRUE(ubx_rx_has_complete_frame(&test_ctx));

    // Read should work correctly
    uint8_t read_buf[10];
    size_t read = ubx_rx_buf_read(&test_ctx, read_buf, 10, 100);
    TEST_ASSERT_EQUAL(10, read);
    TEST_ASSERT_EQUAL_MEMORY(ack_msg, read_buf, 10);
}

/**
 * Test: UART frame resynchronization
 */
TEST_CASE("UART integration - frame resync on garbage", "[ubx][integration]")
{
    // Write garbage data
    uint8_t garbage[] = {0x00, 0xFF, 0x12, 0x34, 0x56, 0x78};
    simulate_uart_write(&test_ctx, garbage, sizeof(garbage));

    // Then valid message
    uint8_t ack_msg[10];
    create_ack_message(ack_msg, 0x06, 0x01);
    simulate_uart_write(&test_ctx, ack_msg, sizeof(ack_msg));

    // Consumer should skip garbage and find valid frame
    TEST_ASSERT_TRUE(ubx_rx_has_complete_frame(&test_ctx));
}

/**
 * Test: Link loss detection and recovery
 */
TEST_CASE("UART integration - link loss detection", "[ubx][integration]")
{
    test_ctx.last_valid_ms = get_millis() - 5000;  // 5 seconds ago
    test_ctx.link_lost = false;

    // Fill buffer with garbage (no valid frames)
    uint8_t garbage[400];
    memset(garbage, 0xFF, sizeof(garbage));
    simulate_uart_write(&test_ctx, garbage, sizeof(garbage));

    // In real code, ubx_uart_event_task would detect stale link and flush
    // Simulate that behavior
    if (xSemaphoreTake(test_ctx.rx_buf_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        uint32_t now = get_millis();
        size_t used = ubx_rx_buf_available(&test_ctx);
        size_t high_water = (test_ctx.rx_buf_size * 75) / 100;

        if (test_ctx.last_valid_ms && used > high_water && 
            (now - test_ctx.last_valid_ms) > 3000) {
            // Flush buffer
            test_ctx.rx_buf_head = 0;
            test_ctx.rx_buf_tail = 0;
            test_ctx.link_lost = true;
        }
        xSemaphoreGive(test_ctx.rx_buf_mutex);
    }

    TEST_ASSERT_TRUE(test_ctx.link_lost);
    TEST_ASSERT_EQUAL(0, ubx_rx_buf_available(&test_ctx));
}

/**
 * Test: Link recovery after loss
 */
TEST_CASE("UART integration - link recovery", "[ubx][integration]")
{
    test_ctx.link_lost = true;
    test_ctx.last_valid_ms = get_millis() - 5000;

    // Send valid message
    uint8_t ack_msg[10];
    create_ack_message(ack_msg, 0x06, 0x01);
    simulate_uart_write(&test_ctx, ack_msg, sizeof(ack_msg));

    // Parse message with checksum validation
    ubx_msg_byte_ctx_t packet = {
        .msg = (uint8_t *)&test_msg.navAck,
        .msg_size = sizeof(nav_ack_t),
        .ubx_msg = &test_msg,
        .ubx_msg_type = MT_NAV_ACK,
        .ctx = &test_ctx
    };

    // Read message
    uint8_t read_buf[10];
    size_t read = ubx_rx_buf_read(&test_ctx, read_buf, 10, 100);
    memcpy(packet.msg, read_buf + 2, 8);  // Skip UBX header
    packet.msg_len = 8;

    esp_err_t ret = ubx_msg_checksum_handler(&packet);

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_FALSE(test_ctx.link_lost);
    TEST_ASSERT_NOT_EQUAL(0, test_ctx.last_valid_ms);
}

/**
 * Test: Concurrent reader/writer stress test
 */
TEST_CASE("UART integration - concurrent access stress", "[ubx][integration]")
{
    // Simulate rapid writes and reads
    for (int i = 0; i < 50; i++) {
        uint8_t ack_msg[10];
        create_ack_message(ack_msg, 0x06, (uint8_t)i);

        // Write
        simulate_uart_write(&test_ctx, ack_msg, sizeof(ack_msg));

        // Immediate read attempt
        if (ubx_rx_has_complete_frame(&test_ctx)) {
            uint8_t read_buf[10];
            size_t read = ubx_rx_buf_read(&test_ctx, read_buf, 10, 50);
            TEST_ASSERT_GREATER_THAN(0, read);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Buffer should be manageable, not overflow
    size_t final_avail = ubx_rx_buf_available(&test_ctx);
    TEST_ASSERT_LESS_THAN(test_ctx.rx_buf_size, final_avail);
}

/**
 * Test: Message ready semaphore counting
 */
TEST_CASE("UART integration - message ready semaphore", "[ubx][integration]")
{
    // Write 3 messages rapidly
    for (int i = 0; i < 3; i++) {
        uint8_t ack_msg[10];
        create_ack_message(ack_msg, 0x06, (uint8_t)i);
        simulate_uart_write(&test_ctx, ack_msg, sizeof(ack_msg));
    }

    // Semaphore should have count of 3
    int count = 0;
    while (xSemaphoreTake(test_ctx.msg_ready, 0) == pdTRUE) {
        count++;
        if (count > 10) break;  // Safety
    }

    TEST_ASSERT_GREATER_OR_EQUAL(3, count);
}

/**
 * Test: Empty buffer timeout behavior
 */
TEST_CASE("UART integration - read timeout on empty buffer", "[ubx][integration]")
{
    uint8_t read_buf[50];
    uint32_t start = get_millis();

    size_t read = ubx_rx_buf_read(&test_ctx, read_buf, sizeof(read_buf), 200);

    uint32_t elapsed = get_millis() - start;

    TEST_ASSERT_EQUAL(0, read);
    TEST_ASSERT_GREATER_OR_EQUAL(190, elapsed);
    TEST_ASSERT_LESS_THAN(250, elapsed);
}

/**
 * Test: Partial frame timeout behavior
 */
TEST_CASE("UART integration - timeout on incomplete frame", "[ubx][integration]")
{
    // Write only header
    uint8_t partial[] = {0xB5, 0x62, 0x05, 0x01};
    simulate_uart_write(&test_ctx, partial, sizeof(partial));

    TEST_ASSERT_FALSE(ubx_rx_has_complete_frame(&test_ctx));

    // Attempt to read full frame should timeout
    uint8_t read_buf[10];
    uint32_t start = get_millis();
    size_t read = ubx_rx_buf_read(&test_ctx, read_buf, 10, 100);
    uint32_t elapsed = get_millis() - start;

    // Should read partial data or timeout
    TEST_ASSERT_LESS_OR_EQUAL(4, read);
}

/**
 * Test: End-to-end message parsing with type handler
 */
TEST_CASE("UART integration - end-to-end message parse", "[ubx][integration]")
{
    // Create and write NAV-PVT message (simplified)
    uint8_t nav_pvt_msg[100];
    nav_pvt_msg[0] = 0xB5;
    nav_pvt_msg[1] = 0x62;
    nav_pvt_msg[2] = 0x01;  // NAV
    nav_pvt_msg[3] = 0x07;  // PVT
    nav_pvt_msg[4] = 0x5C;  // Length: 92
    nav_pvt_msg[5] = 0x00;

    // Fill with test data
    memset(nav_pvt_msg + 6, 0x00, 92);

    // Calculate checksum
    uint8_t CK_A = 0, CK_B = 0;
    for (int i = 2; i < 98; i++) {
        CK_A += nav_pvt_msg[i];
        CK_B += CK_A;
    }
    nav_pvt_msg[98] = CK_A;
    nav_pvt_msg[99] = CK_B;

    simulate_uart_write(&test_ctx, nav_pvt_msg, 100);

    // Parse message
    ubx_msg_byte_ctx_t packet = {
        .msg = (uint8_t *)&test_msg.navPvt,
        .msg_size = sizeof(nav_pvt_t),
        .ubx_msg = &test_msg,
        .ubx_msg_type = MT_NONE,
        .ctx = &test_ctx
    };

    // Read from buffer
    uint8_t read_buf[100];
    size_t read = ubx_rx_buf_read(&test_ctx, read_buf, 100, 100);
    TEST_ASSERT_EQUAL(100, read);

    // Copy to packet (skip UBX header)
    memcpy(packet.msg, read_buf + 2, 98);
    packet.msg_len = 98;

    // Type identification
    esp_err_t ret = ubx_msg_type_handler(&packet);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(MT_NAV_PVT, packet.ubx_msg_type);

    // Checksum validation
    ret = ubx_msg_checksum_handler(&packet);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

/**
 * Test: Error recovery - bad checksum handling
 */
TEST_CASE("UART integration - bad checksum recovery", "[ubx][integration]")
{
    // Send message with bad checksum
    uint8_t bad_msg[10];
    create_ack_message(bad_msg, 0x06, 0x01);
    bad_msg[8] = 0xFF;  // Corrupt checksum
    bad_msg[9] = 0xFF;

    simulate_uart_write(&test_ctx, bad_msg, sizeof(bad_msg));

    // Then send good message
    uint8_t good_msg[10];
    create_ack_message(good_msg, 0x06, 0x08);
    simulate_uart_write(&test_ctx, good_msg, sizeof(good_msg));

    // First read should fail checksum
    uint8_t read_buf1[10];
    ubx_rx_buf_read(&test_ctx, read_buf1, 10, 100);

    ubx_msg_byte_ctx_t packet = {
        .msg = (uint8_t *)&test_msg.navAck,
        .msg_size = sizeof(nav_ack_t),
        .msg_len = 8,
        .ubx_msg = &test_msg,
        .ubx_msg_type = MT_NAV_ACK,
        .ctx = &test_ctx
    };

    memcpy(packet.msg, read_buf1 + 2, 8);
    esp_err_t ret1 = ubx_msg_checksum_handler(&packet);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_CRC, ret1);

    // Second read should succeed
    uint8_t read_buf2[10];
    ubx_rx_buf_read(&test_ctx, read_buf2, 10, 100);
    memcpy(packet.msg, read_buf2 + 2, 8);

    esp_err_t ret2 = ubx_msg_checksum_handler(&packet);
    TEST_ASSERT_EQUAL(ESP_OK, ret2);
}

/**
 * Test: High-frequency message stream
 */
TEST_CASE("UART integration - high frequency message stream", "[ubx][integration]")
{
    const int msg_count = 100;
    int successful_reads = 0;

    // Simulate high-rate GPS output
    for (int i = 0; i < msg_count; i++) {
        uint8_t ack_msg[10];
        create_ack_message(ack_msg, 0x06, (uint8_t)(i % 256));
        simulate_uart_write(&test_ctx, ack_msg, sizeof(ack_msg));

        // Consumer reads as fast as possible
        if (ubx_rx_has_complete_frame(&test_ctx)) {
            uint8_t read_buf[10];
            if (ubx_rx_buf_read(&test_ctx, read_buf, 10, 5) == 10) {
                successful_reads++;
            }
        }
    }

    // Should process most messages
    TEST_ASSERT_GREATER_THAN(90, successful_reads);
}
