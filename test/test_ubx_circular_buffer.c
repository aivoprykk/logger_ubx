/**
 * @file test_ubx_circular_buffer.c
 * @brief Unit tests for UBX circular buffer operations
 * 
 * Tests circular buffer helpers used in ubx_uart_event.c:
 * - ubx_rx_buf_available()
 * - ubx_rx_buf_free_space()
 * - ubx_rx_buf_read()
 * - ubx_rx_has_complete_frame()
 */

#include <string.h>
#include "unity.h"
#include "ubx.h"
#include "ubx_private.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "logger_common.h"

static const char *TAG = "test_ubx_circ_buf";

// Forward declarations for test functions
void test_empty_buffer_has_zero_available(void);
void test_available_bytes_without_wrap(void);
void test_available_bytes_with_wrap_around(void);
void test_nearly_full_buffer(void);
void test_free_space_without_wrap(void);
void test_free_space_with_wrap(void);
void test_read_from_empty_buffer(void);
void test_read_contiguous_data(void);
void test_read_wrapped_data(void);
void test_partial_read(void);
void test_detect_complete_ubx_frame(void);
void test_incomplete_frame_header_only(void);
void test_incomplete_frame_partial_payload(void);
void test_no_frame_in_garbage_data(void);
void test_frame_detection_with_wrap(void);
void test_multiple_frames_detection(void);
void test_read_timeout(void);
void test_null_pointer_checks(void);

// Mock context for testing
static ubx_ctx_t test_ctx;
static uint8_t test_buffer[256];

void setUp(void) {
    memset(&test_ctx, 0, sizeof(ubx_ctx_t));
    memset(test_buffer, 0, sizeof(test_buffer));
    
    test_ctx.rx_buffer = test_buffer;
    test_ctx.rx_buf_size = sizeof(test_buffer);
    test_ctx.rx_buf_head = 0;
    test_ctx.rx_buf_tail = 0;
    test_ctx.rx_buf_mutex = xSemaphoreCreateMutex();
    TEST_ASSERT_NOT_NULL(test_ctx.rx_buf_mutex);
}

void tearDown(void) {
    if (test_ctx.rx_buf_mutex) {
        vSemaphoreDelete(test_ctx.rx_buf_mutex);
        test_ctx.rx_buf_mutex = NULL;
    }
}

/**
 * Test: Empty buffer reports 0 available bytes
 */
void test_empty_buffer_has_zero_available(void)
{
    size_t avail = ubx_rx_buf_available(&test_ctx);
    TEST_ASSERT_EQUAL(0, avail);
}

/**
 * Test: Buffer with data reports correct available bytes (no wrap)
 */
void test_available_bytes_without_wrap(void)
{
    // Simulate writing 50 bytes
    test_ctx.rx_buf_head = 50;
    test_ctx.rx_buf_tail = 0;
    
    size_t avail = ubx_rx_buf_available(&test_ctx);
    TEST_ASSERT_EQUAL(50, avail);
}

/**
 * Test: Buffer with wrap-around reports correct available bytes
 */
void test_available_bytes_with_wrap_around(void)
{
    // Simulate wrap: head=10, tail=200
    // Available = (256 - 200) + 10 = 66
    test_ctx.rx_buf_head = 10;
    test_ctx.rx_buf_tail = 200;
    
    size_t avail = ubx_rx_buf_available(&test_ctx);
    TEST_ASSERT_EQUAL(66, avail);
}

/**
 * Test: Full buffer reports correct available bytes
 */
void test_nearly_full_buffer(void)
{
    // head=255, tail=0 means 255 bytes used (one byte reserved)
    test_ctx.rx_buf_head = 255;
    test_ctx.rx_buf_tail = 0;
    
    size_t avail = ubx_rx_buf_available(&test_ctx);
    TEST_ASSERT_EQUAL(255, avail);
}

/**
 * Test: Free space calculation without wrap
 */
void test_free_space_without_wrap(void)
{
    test_ctx.rx_buf_head = 50;
    test_ctx.rx_buf_tail = 0;
    
    size_t free_space = ubx_rx_buf_free_space(&test_ctx);
    // Free = 256 - 50 - 1 = 205
    TEST_ASSERT_EQUAL(205, free_space);
}

/**
 * Test: Free space calculation with wrap
 */
void test_free_space_with_wrap(void)
{
    test_ctx.rx_buf_head = 10;
    test_ctx.rx_buf_tail = 200;
    
    // Available = 66, so free = 256 - 66 - 1 = 189
    size_t free_space = ubx_rx_buf_free_space(&test_ctx);
    TEST_ASSERT_EQUAL(189, free_space);
}

/**
 * Test: Read from empty buffer returns 0
 */
void test_read_from_empty_buffer(void)
{
    uint8_t dst[50];
    size_t read = ubx_rx_buf_read(&test_ctx, dst, sizeof(dst), 10);
    TEST_ASSERT_EQUAL(0, read);
}

/**
 * Test: Read contiguous data (no wrap)
 */
void test_read_contiguous_data(void)
{
    // Write test pattern
    for (int i = 0; i < 50; i++) {
        test_ctx.rx_buffer[i] = (uint8_t)(i + 1);
    }
    test_ctx.rx_buf_head = 50;
    test_ctx.rx_buf_tail = 0;
    
    uint8_t dst[30];
    size_t read = ubx_rx_buf_read(&test_ctx, dst, sizeof(dst), 100);
    
    TEST_ASSERT_EQUAL(30, read);
    TEST_ASSERT_EQUAL(30, test_ctx.rx_buf_tail);
    
    // Verify data
    for (int i = 0; i < 30; i++) {
        TEST_ASSERT_EQUAL(i + 1, dst[i]);
    }
}

/**
 * Test: Read wrapped data
 */
void test_read_wrapped_data(void)
{
    // Simulate wrap: data at end and start of buffer
    test_ctx.rx_buf_tail = 240;
    test_ctx.rx_buf_head = 20;
    
    // Write pattern at end (240-255)
    for (int i = 240; i < 256; i++) {
        test_ctx.rx_buffer[i] = (uint8_t)(i - 240 + 1);
    }
    // Write pattern at start (0-19)
    for (int i = 0; i < 20; i++) {
        test_ctx.rx_buffer[i] = (uint8_t)(i + 17);
    }
    
    uint8_t dst[36];
    size_t read = ubx_rx_buf_read(&test_ctx, dst, sizeof(dst), 100);
    
    TEST_ASSERT_EQUAL(36, read);
    TEST_ASSERT_EQUAL(20, test_ctx.rx_buf_tail);
    
    // Verify wrapped data continuity
    for (int i = 0; i < 16; i++) {
        TEST_ASSERT_EQUAL(i + 1, dst[i]);
    }
    for (int i = 16; i < 36; i++) {
        TEST_ASSERT_EQUAL(i + 1, dst[i]);
    }
}

/**
 * Test: Partial read when buffer has less than requested
 */
void test_partial_read(void)
{
    // Only 20 bytes available
    for (int i = 0; i < 20; i++) {
        test_ctx.rx_buffer[i] = (uint8_t)(i + 100);
    }
    test_ctx.rx_buf_head = 20;
    test_ctx.rx_buf_tail = 0;
    
    uint8_t dst[50];
    size_t read = ubx_rx_buf_read(&test_ctx, dst, sizeof(dst), 10);
    
    // Should only read 20 bytes
    TEST_ASSERT_EQUAL(20, read);
    TEST_ASSERT_EQUAL(20, test_ctx.rx_buf_tail);
}

/**
 * Test: Complete UBX frame detection - valid frame present
 */
void test_detect_complete_ubx_frame(void)
{
    // Create minimal NAV-PVT frame: B5 62 01 07 5C 00 [92 bytes payload] CK_A CK_B
    test_ctx.rx_buffer[0] = 0xB5;  // Header A
    test_ctx.rx_buffer[1] = 0x62;  // Header B
    test_ctx.rx_buffer[2] = 0x01;  // Class NAV
    test_ctx.rx_buffer[3] = 0x07;  // ID PVT
    test_ctx.rx_buffer[4] = 0x5C;  // Length LSB (92)
    test_ctx.rx_buffer[5] = 0x00;  // Length MSB
    
    // Total frame = 6 (header+cls+id+len) + 92 (payload) + 2 (checksum) = 100 bytes
    test_ctx.rx_buf_head = 100;
    test_ctx.rx_buf_tail = 0;
    
    bool has_frame = ubx_rx_has_complete_frame(&test_ctx);
    TEST_ASSERT_TRUE(has_frame);
}

/**
 * Test: Incomplete frame detection - header only
 */
void test_incomplete_frame_header_only(void)
{
    test_ctx.rx_buffer[0] = 0xB5;
    test_ctx.rx_buffer[1] = 0x62;
    test_ctx.rx_buf_head = 2;
    
    bool has_frame = ubx_rx_has_complete_frame(&test_ctx);
    TEST_ASSERT_FALSE(has_frame);
}

/**
 * Test: Incomplete frame - header + class/id but not enough payload
 */
void test_incomplete_frame_partial_payload(void)
{
    test_ctx.rx_buffer[0] = 0xB5;
    test_ctx.rx_buffer[1] = 0x62;
    test_ctx.rx_buffer[2] = 0x01;
    test_ctx.rx_buffer[3] = 0x07;
    test_ctx.rx_buffer[4] = 0x5C;  // 92 bytes payload
    test_ctx.rx_buffer[5] = 0x00;
    
    // Only 50 bytes after header (need 6+92+2=100 total)
    test_ctx.rx_buf_head = 50;
    
    bool has_frame = ubx_rx_has_complete_frame(&test_ctx);
    TEST_ASSERT_FALSE(has_frame);
}

/**
 * Test: No frame - garbage data
 */
void test_no_frame_in_garbage_data(void)
{
    for (int i = 0; i < 50; i++) {
        test_ctx.rx_buffer[i] = (uint8_t)(i % 200); // Avoid UBX header
    }
    test_ctx.rx_buf_head = 50;
    
    bool has_frame = ubx_rx_has_complete_frame(&test_ctx);
    TEST_ASSERT_FALSE(has_frame);
}

/**
 * Test: Frame detection with wrapped data
 */
void test_frame_detection_with_wrap(void)
{
    // Place frame across wrap boundary
    test_ctx.rx_buf_tail = 250;
    
    // Header at end
    test_ctx.rx_buffer[250] = 0xB5;
    test_ctx.rx_buffer[251] = 0x62;
    test_ctx.rx_buffer[252] = 0x05;  // ACK class
    test_ctx.rx_buffer[253] = 0x01;  // ACK-ACK
    test_ctx.rx_buffer[254] = 0x02;  // Payload len = 2
    test_ctx.rx_buffer[255] = 0x00;
    
    // Payload + checksum at start
    test_ctx.rx_buffer[0] = 0x06;   // cls
    test_ctx.rx_buffer[1] = 0x01;   // id
    test_ctx.rx_buffer[2] = 0xAA;   // CK_A (dummy)
    test_ctx.rx_buffer[3] = 0xBB;   // CK_B (dummy)
    
    test_ctx.rx_buf_head = 4;  // Total = (256-250) + 4 = 10 bytes
    
    bool has_frame = ubx_rx_has_complete_frame(&test_ctx);
    TEST_ASSERT_TRUE(has_frame);
}

/**
 * Test: Multiple frames in buffer - should detect first complete one
 */
void test_multiple_frames_detection(void)
{
    // First ACK frame (10 bytes total)
    test_ctx.rx_buffer[0] = 0xB5;
    test_ctx.rx_buffer[1] = 0x62;
    test_ctx.rx_buffer[2] = 0x05;
    test_ctx.rx_buffer[3] = 0x01;
    test_ctx.rx_buffer[4] = 0x02;
    test_ctx.rx_buffer[5] = 0x00;
    test_ctx.rx_buffer[6] = 0x06;
    test_ctx.rx_buffer[7] = 0x01;
    test_ctx.rx_buffer[8] = 0xAA;
    test_ctx.rx_buffer[9] = 0xBB;
    
    // Second frame starts at 10
    test_ctx.rx_buffer[10] = 0xB5;
    test_ctx.rx_buffer[11] = 0x62;
    // ...
    
    test_ctx.rx_buf_head = 20;
    
    bool has_frame = ubx_rx_has_complete_frame(&test_ctx);
    TEST_ASSERT_TRUE(has_frame);
}

/**
 * Test: Read timeout behavior
 */
void test_read_timeout(void)
{
    uint8_t dst[50];
    
    // Start time
    uint32_t start_ms = get_millis();
    
    // Try to read with 100ms timeout from empty buffer
    size_t read = ubx_rx_buf_read(&test_ctx, dst, sizeof(dst), 100);
    
    uint32_t elapsed = get_millis() - start_ms;
    
    TEST_ASSERT_EQUAL(0, read);
    TEST_ASSERT_GREATER_OR_EQUAL(90, elapsed);  // Allow some tolerance
    TEST_ASSERT_LESS_THAN(150, elapsed);
}

/**
 * Test: Null pointer safety checks
 */
void test_null_pointer_checks(void)
{
    uint8_t dst[10];
    
    // Null context
    size_t read = ubx_rx_buf_read(NULL, dst, sizeof(dst), 10);
    TEST_ASSERT_EQUAL(0, read);
    
    // Null destination
    read = ubx_rx_buf_read(&test_ctx, NULL, 10, 10);
    TEST_ASSERT_EQUAL(0, read);
    
    // Null buffer in context
    test_ctx.rx_buffer = NULL;
    read = ubx_rx_buf_read(&test_ctx, dst, sizeof(dst), 10);
    TEST_ASSERT_EQUAL(0, read);
    
    // Frame detection with null
    bool has_frame = ubx_rx_has_complete_frame(NULL);
    TEST_ASSERT_FALSE(has_frame);
}

/**
 * Run all UBX circular buffer tests
 * This function can be called from main app to execute the tests
 */
void run_ubx_circular_buffer_tests(void) {
    UNITY_BEGIN();
    
    // Run all test cases manually since TEST_CASE doesn't auto-register in component context
    RUN_TEST(test_empty_buffer_has_zero_available);
    RUN_TEST(test_available_bytes_without_wrap);
    RUN_TEST(test_available_bytes_with_wrap_around);
    RUN_TEST(test_nearly_full_buffer);
    RUN_TEST(test_free_space_without_wrap);
    RUN_TEST(test_free_space_with_wrap);
    RUN_TEST(test_read_from_empty_buffer);
    RUN_TEST(test_read_contiguous_data);
    RUN_TEST(test_read_wrapped_data);
    RUN_TEST(test_partial_read);
    RUN_TEST(test_detect_complete_ubx_frame);
    RUN_TEST(test_incomplete_frame_header_only);
    RUN_TEST(test_incomplete_frame_partial_payload);
    RUN_TEST(test_no_frame_in_garbage_data);
    RUN_TEST(test_frame_detection_with_wrap);
    RUN_TEST(test_multiple_frames_detection);
    RUN_TEST(test_read_timeout);
    RUN_TEST(test_null_pointer_checks);
    
    UNITY_END();
}
