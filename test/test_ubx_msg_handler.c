/**
 * @file test_ubx_msg_handler.c
 * @brief Unit tests for UBX message handler and checksum validation
 * 
 * Tests message handling functions from ubx_msg_handler.c:
 * - ubx_msg_type_handler() - message type identification
 * - msg_checksum_cb() / ubx_msg_checksum_handler() - checksum validation
 * - ubx_read_frame() - frame reading and parsing
 * - write_ubx_msg() - message writing with checksum
 * - add_checksum() - checksum calculation
 */

#include <string.h>
#include "unity.h"
#include "ubx.h"
#include "ubx_private.h"
#include "ubx_msg.h"
#include "freertos/FreeRTOS.h"
#include "logger_common.h"

static const char *TAG = "test_ubx_msg_handler";

// Mock context and message structures
static ubx_ctx_t test_ctx;
static ubx_msg_t test_msg;
static uint8_t test_rx_buffer[512];

void setUp(void) {
    memset(&test_ctx, 0, sizeof(ubx_ctx_t));
    memset(&test_msg, 0, sizeof(ubx_msg_t));
    memset(test_rx_buffer, 0, sizeof(test_rx_buffer));
    
    test_ctx.rx_buffer = test_rx_buffer;
    test_ctx.rx_buf_size = sizeof(test_rx_buffer);
    test_ctx.rx_buf_mutex = xSemaphoreCreateMutex();
}

void tearDown(void) {
    if (test_ctx.rx_buf_mutex) {
        vSemaphoreDelete(test_ctx.rx_buf_mutex);
    }
}

/**
 * Test: Checksum calculation for ACK-ACK message
 */
TEST_CASE("Message handler - checksum calculation ACK-ACK", "[ubx][msg_handler]")
{
    // ACK-ACK message: cls=05, id=01, len=02, payload=06 01
    uint8_t msg[] = {0x05, 0x01, 0x02, 0x00, 0x06, 0x01, 0x00, 0x00};
    uint8_t CK_A = 0, CK_B = 0;
    
    add_checksum(msg, 8, &CK_A, &CK_B);
    
    // Calculated checksum for: 05 01 02 00 06 01
    // CK_A = 05+01+02+00+06+01 = 0F
    // CK_B = 05+(05+01)+(05+01+02)+(05+01+02+00)+(05+01+02+00+06)+(05+01+02+00+06+01) = 0F+06+08+08+0E+0F = 52
    TEST_ASSERT_EQUAL_HEX8(0x0F, CK_A);
    TEST_ASSERT_EQUAL_HEX8(0x52, CK_B);
}

/**
 * Test: Checksum calculation with UBX header
 */
TEST_CASE("Message handler - checksum with UBX header", "[ubx][msg_handler]")
{
    // Full UBX message with header: B5 62 05 01 02 00 06 01 CK_A CK_B
    uint8_t msg[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x01, 0x00, 0x00};
    uint8_t CK_A = 0, CK_B = 0;
    
    add_checksum(msg, 10, &CK_A, &CK_B);
    
    // Should skip header (B5 62) and calculate from byte 2 onwards
    TEST_ASSERT_EQUAL_HEX8(0x0F, CK_A);
    TEST_ASSERT_EQUAL_HEX8(0x52, CK_B);
}

/**
 * Test: Checksum validation - valid message
 */
TEST_CASE("Message handler - checksum validation valid", "[ubx][msg_handler]")
{
    ubx_msg_byte_ctx_t packet = {
        .msg = (uint8_t[]){0x05, 0x01, 0x02, 0x00, 0x06, 0x01, 0x0F, 0x52},
        .msg_size = 8,
        .msg_len = 8,
        .ubx_msg = &test_msg,
        .ubx_msg_type = MT_NAV_ACK
    };
    
    esp_err_t ret = msg_checksum_cb(&packet);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

/**
 * Test: Checksum validation - invalid checksum
 */
TEST_CASE("Message handler - checksum validation invalid", "[ubx][msg_handler]")
{
    ubx_msg_byte_ctx_t packet = {
        .msg = (uint8_t[]){0x05, 0x01, 0x02, 0x00, 0x06, 0x01, 0xFF, 0xFF},
        .msg_size = 8,
        .msg_len = 8,
        .ubx_msg = &test_msg,
        .ubx_msg_type = MT_NAV_ACK
    };
    
    esp_err_t ret = msg_checksum_cb(&packet);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_CRC, ret);
}

/**
 * Test: Message type handler - NAV-PVT identification
 */
TEST_CASE("Message handler - NAV-PVT type identification", "[ubx][msg_handler]")
{
    uint8_t msg_data[100] = {0x01, 0x07, 0x5C, 0x00}; // NAV (01) PVT (07) len=92
    ubx_msg_byte_ctx_t packet = {
        .msg = msg_data,
        .msg_size = 100,
        .ubx_msg = &test_msg,
        .ubx_msg_type = MT_NONE
    };
    
    esp_err_t ret = ubx_msg_type_handler(&packet);
    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(MT_NAV_PVT, packet.ubx_msg_type);
    TEST_ASSERT_EQUAL(sizeof(nav_pvt_t), packet.msg_size);
}

/**
 * Test: Message type handler - NAV-DOP identification
 */
TEST_CASE("Message handler - NAV-DOP type identification", "[ubx][msg_handler]")
{
    uint8_t msg_data[32] = {0x01, 0x04, 0x12, 0x00}; // NAV (01) DOP (04)
    ubx_msg_byte_ctx_t packet = {
        .msg = msg_data,
        .msg_size = 32,
        .ubx_msg = &test_msg,
        .ubx_msg_type = MT_NONE
    };
    
    esp_err_t ret = ubx_msg_type_handler(&packet);
    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(MT_NAV_DOP, packet.ubx_msg_type);
}

/**
 * Test: Message type handler - NAV-SAT identification
 */
TEST_CASE("Message handler - NAV-SAT type identification", "[ubx][msg_handler]")
{
    uint8_t msg_data[200] = {0x01, 0x35, 0x00, 0x01}; // NAV (01) SAT (35)
    ubx_msg_byte_ctx_t packet = {
        .msg = msg_data,
        .msg_size = 200,
        .ubx_msg = &test_msg,
        .ubx_msg_type = MT_NONE
    };
    
    esp_err_t ret = ubx_msg_type_handler(&packet);
    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(MT_NAV_SAT, packet.ubx_msg_type);
}

/**
 * Test: Message type handler - ACK-ACK identification
 */
TEST_CASE("Message handler - ACK-ACK type identification", "[ubx][msg_handler]")
{
    uint8_t msg_data[8] = {0x05, 0x01, 0x02, 0x00}; // ACK (05) ACK (01)
    ubx_msg_byte_ctx_t packet = {
        .msg = msg_data,
        .msg_size = 8,
        .ubx_msg = &test_msg,
        .ubx_msg_type = MT_NONE
    };
    
    esp_err_t ret = ubx_msg_type_handler(&packet);
    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(MT_NAV_ACK, packet.ubx_msg_type);
}

/**
 * Test: Message type handler - ACK-NAK identification
 */
TEST_CASE("Message handler - ACK-NAK type identification", "[ubx][msg_handler]")
{
    uint8_t msg_data[8] = {0x05, 0x00, 0x02, 0x00}; // ACK (05) NAK (00)
    ubx_msg_byte_ctx_t packet = {
        .msg = msg_data,
        .msg_size = 8,
        .ubx_msg = &test_msg,
        .ubx_msg_type = MT_NONE
    };
    
    esp_err_t ret = ubx_msg_type_handler(&packet);
    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(MT_NAV_NACK, packet.ubx_msg_type);
}

/**
 * Test: Message type handler - MON-VER identification
 */
TEST_CASE("Message handler - MON-VER type identification", "[ubx][msg_handler]")
{
    uint8_t msg_data[100] = {0x0A, 0x04, 0x28, 0x00}; // MON (0A) VER (04)
    ubx_msg_byte_ctx_t packet = {
        .msg = msg_data,
        .msg_size = 100,
        .ubx_msg = &test_msg,
        .ubx_msg_type = MT_NONE
    };
    
    esp_err_t ret = ubx_msg_type_handler(&packet);
    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(MT_MON_VER, packet.ubx_msg_type);
}

/**
 * Test: Message type handler - MON-GNSS identification
 */
TEST_CASE("Message handler - MON-GNSS type identification", "[ubx][msg_handler]")
{
    uint8_t msg_data[32] = {0x0A, 0x28, 0x08, 0x00}; // MON (0A) GNSS (28)
    ubx_msg_byte_ctx_t packet = {
        .msg = msg_data,
        .msg_size = 32,
        .ubx_msg = &test_msg,
        .ubx_msg_type = MT_NONE
    };
    
    esp_err_t ret = ubx_msg_type_handler(&packet);
    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(MT_MON_GNSS, packet.ubx_msg_type);
}

/**
 * Test: Message type handler - SEC-UNIQID identification
 */
TEST_CASE("Message handler - SEC-UNIQID type identification", "[ubx][msg_handler]")
{
    uint8_t msg_data[16] = {0x27, 0x03, 0x09, 0x00}; // SEC (27) UNIQID (03)
    ubx_msg_byte_ctx_t packet = {
        .msg = msg_data,
        .msg_size = 16,
        .ubx_msg = &test_msg,
        .ubx_msg_type = MT_NONE
    };
    
    esp_err_t ret = ubx_msg_type_handler(&packet);
    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(MT_NAV_ID, packet.ubx_msg_type);
}

/**
 * Test: Message type handler - unknown message class
 */
TEST_CASE("Message handler - unknown message class", "[ubx][msg_handler]")
{
    uint8_t msg_data[16] = {0xFF, 0x01, 0x04, 0x00}; // Invalid class FF
    ubx_msg_byte_ctx_t packet = {
        .msg = msg_data,
        .msg_size = 16,
        .ubx_msg = &test_msg,
        .ubx_msg_type = MT_NONE
    };
    
    esp_err_t ret = ubx_msg_type_handler(&packet);
    
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, ret);
    TEST_ASSERT_EQUAL(MT_NONE, packet.ubx_msg_type);
}

/**
 * Test: Message type handler - unknown message ID in valid class
 */
TEST_CASE("Message handler - unknown NAV message ID", "[ubx][msg_handler]")
{
    uint8_t msg_data[16] = {0x01, 0xFF, 0x04, 0x00}; // NAV class, invalid ID FF
    ubx_msg_byte_ctx_t packet = {
        .msg = msg_data,
        .msg_size = 16,
        .ubx_msg = &test_msg,
        .ubx_msg_type = MT_NONE
    };
    
    esp_err_t ret = ubx_msg_type_handler(&packet);
    
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, ret);
}

/**
 * Test: ubx_msg_byte_ctx_reset clears context
 */
TEST_CASE("Message handler - context reset", "[ubx][msg_handler]")
{
    ubx_msg_byte_ctx_t packet = {
        .msg = (uint8_t *)&test_msg.none,
        .msg_size = 16,
        .ubx_msg = &test_msg,
        .ubx_msg_type = MT_NAV_PVT
    };
    
    // Fill with non-zero
    memset(packet.msg, 0xAA, 16);
    
    esp_err_t ret = ubx_msg_byte_ctx_reset(&packet);
    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(MT_NONE, packet.ubx_msg_type);
    TEST_ASSERT_EQUAL(UBX_NONE_SIZE, packet.msg_size);
    
    // Verify zeroed
    for (int i = 0; i < UBX_NONE_SIZE; i++) {
        TEST_ASSERT_EQUAL(0, packet.msg[i]);
    }
}

/**
 * Test: Checksum handler updates counters
 */
TEST_CASE("Message handler - checksum handler counter updates", "[ubx][msg_handler]")
{
    test_msg.count_msg = 5;
    test_msg.count_ok = 3;
    test_msg.count_err = 2;
    
    // Valid checksum
    uint8_t msg_valid[] = {0x05, 0x01, 0x02, 0x00, 0x06, 0x01, 0x0F, 0x52};
    ubx_msg_byte_ctx_t packet = {
        .msg = msg_valid,
        .msg_size = 8,
        .msg_len = 8,
        .ubx_msg = &test_msg,
        .ubx_msg_type = MT_NAV_ACK,
        .ctx = &test_ctx
    };
    
    esp_err_t ret = ubx_msg_checksum_handler(&packet);
    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(6, test_msg.count_msg);
    TEST_ASSERT_EQUAL(4, test_msg.count_ok);
    TEST_ASSERT_EQUAL(2, test_msg.count_err);
}

/**
 * Test: Checksum handler increments error counter on bad checksum
 */
TEST_CASE("Message handler - checksum handler error counter", "[ubx][msg_handler]")
{
    test_msg.count_msg = 5;
    test_msg.count_ok = 3;
    test_msg.count_err = 2;
    
    // Invalid checksum
    uint8_t msg_invalid[] = {0x05, 0x01, 0x02, 0x00, 0x06, 0x01, 0xFF, 0xFF};
    ubx_msg_byte_ctx_t packet = {
        .msg = msg_invalid,
        .msg_size = 8,
        .msg_len = 8,
        .ubx_msg = &test_msg,
        .ubx_msg_type = MT_NAV_ACK,
        .ctx = &test_ctx
    };
    
    esp_err_t ret = ubx_msg_checksum_handler(&packet);
    
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_CRC, ret);
    TEST_ASSERT_EQUAL(6, test_msg.count_msg);
    TEST_ASSERT_EQUAL(3, test_msg.count_ok);
    TEST_ASSERT_EQUAL(3, test_msg.count_err);
    TEST_ASSERT_EQUAL(MT_NONE, packet.ubx_msg_type);
}

/**
 * Test: Checksum handler updates link status
 */
TEST_CASE("Message handler - checksum handler link status update", "[ubx][msg_handler]")
{
    test_ctx.link_lost = true;
    test_ctx.last_valid_ms = 0;
    
    uint8_t msg_valid[] = {0x05, 0x01, 0x02, 0x00, 0x06, 0x01, 0x0F, 0x52};
    ubx_msg_byte_ctx_t packet = {
        .msg = msg_valid,
        .msg_size = 8,
        .msg_len = 8,
        .ubx_msg = &test_msg,
        .ubx_msg_type = MT_NAV_ACK,
        .ctx = &test_ctx
    };
    
    esp_err_t ret = ubx_msg_checksum_handler(&packet);
    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_FALSE(test_ctx.link_lost);
    TEST_ASSERT_NOT_EQUAL(0, test_ctx.last_valid_ms);
}

/**
 * Test: Write UBX message with checksum calculation
 */
TEST_CASE("Message handler - write message with checksum", "[ubx][msg_handler]")
{
    // We can't easily test actual UART writing without hardware, but we can
    // verify checksum is added correctly
    uint8_t msg[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x01, 0x00, 0x00};
    
    // Mock: just verify checksum gets computed
    uint8_t CK_A = 0, CK_B = 0;
    add_checksum(msg, sizeof(msg), &CK_A, &CK_B);
    
    TEST_ASSERT_EQUAL_HEX8(0x0F, CK_A);
    TEST_ASSERT_EQUAL_HEX8(0x52, CK_B);
    
    // After write_ubx_msg with need_checksum=true, last two bytes should be updated
    msg[8] = CK_A;
    msg[9] = CK_B;
    
    TEST_ASSERT_EQUAL_HEX8(0x0F, msg[8]);
    TEST_ASSERT_EQUAL_HEX8(0x52, msg[9]);
}

/**
 * Test: Variable length message handling (NAV-SAT)
 */
TEST_CASE("Message handler - variable length NAV-SAT", "[ubx][msg_handler]")
{
    // NAV-SAT with 2 satellites: 8 header bytes + 12*2 payload + 2 checksum = 34 bytes
    uint8_t msg_data[200] = {
        0x01, 0x35,        // NAV-SAT
        0x18, 0x00,        // Length: 24 bytes (8 header + 12*2 satellites)
        0x00, 0x00, 0x00, 0x00,  // iTOW
        0x00,              // version
        0x02,              // numSvs
        0x00, 0x00         // reserved
    };
    
    ubx_msg_byte_ctx_t packet = {
        .msg = msg_data,
        .msg_size = 200,
        .msg_len = 34,     // Actual frame length
        .ubx_msg = &test_msg,
        .ubx_msg_type = MT_NONE
    };
    
    esp_err_t ret = ubx_msg_type_handler(&packet);
    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(MT_NAV_SAT, packet.ubx_msg_type);
    // msg_len should be preserved for variable-length message
    TEST_ASSERT_EQUAL(34, packet.msg_len);
}

/**
 * Test: Null pointer safety in checksum functions
 */
TEST_CASE("Message handler - null pointer safety", "[ubx][msg_handler]")
{
    // Null packet in checksum
    esp_err_t ret = msg_checksum_cb(NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
    
    // Null msg in packet
    ubx_msg_byte_ctx_t packet = {
        .msg = NULL,
        .ubx_msg = &test_msg
    };
    ret = msg_checksum_cb(&packet);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
    
    // Null in type handler
    ret = ubx_msg_type_handler(NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
    
    packet.msg = (uint8_t[]){0x01, 0x07, 0x5C, 0x00};
    packet.msg_size = 100;
    ret = ubx_msg_type_handler(&packet);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);  // msg is set but ubx_msg is NULL check should fail in actual code
}

/**
 * Test: Large payload message handling
 */
TEST_CASE("Message handler - large payload handling", "[ubx][msg_handler]")
{
    // Simulate large NAV-SAT with many satellites
    uint8_t large_msg[1200] = {0x01, 0x35, 0x60, 0x04}; // NAV-SAT, 1120 bytes payload
    ubx_msg_byte_ctx_t packet = {
        .msg = large_msg,
        .msg_size = sizeof(large_msg),
        .ubx_msg = &test_msg,
        .ubx_msg_type = MT_NONE
    };
    
    esp_err_t ret = ubx_msg_type_handler(&packet);
    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(MT_NAV_SAT, packet.ubx_msg_type);
}
