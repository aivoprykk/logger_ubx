/**
 * @file mock_uart.c
 * @brief Mock UART implementation for UBX message handler example
 * 
 * This file provides simulated UBX GNSS data for demonstration purposes.
 * It generates realistic UBX message sequences including NAV-PVT, NAV-DOP,
 * ACK messages, etc.
 */

#include "mock_uart.h"
#include "config.h"
#include <string.h>
#include <stdlib.h>
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "mock_uart";

// Sample UBX messages for demonstration
typedef struct {
    const uint8_t *data;
    size_t len;
    const char *description;
} ubx_sample_msg_t;

// UBX Header: 0xB5 0x62
// Format: Header(2) + Class(1) + ID(1) + Length(2) + Payload(N) + Checksum(2)

// ACK-ACK message (acknowledging CFG message)
static const uint8_t ack_ack_msg[] = {
    0xB5, 0x62,     // UBX header
    0x05, 0x01,     // Class=ACK, ID=ACK
    0x02, 0x00,     // Length=2
    0x06, 0x01,     // Payload: Class=CFG, ID=MSG
    0x0F, 0x38      // Checksum
};

// ACK-NAK message (negative acknowledgement)
static const uint8_t ack_nak_msg[] = {
    0xB5, 0x62,     // UBX header
    0x05, 0x00,     // Class=ACK, ID=NAK
    0x02, 0x00,     // Length=2
    0x06, 0x01,     // Payload: Class=CFG, ID=MSG
    0x0E, 0x33      // Checksum
};

// Complete NAV-PVT message (navigation position velocity time solution)
// Full 92-byte payload + header + checksum = 100 bytes total
static const uint8_t nav_pvt_msg[] = {
    0xB5, 0x62,     // UBX header (2 bytes)
    0x01, 0x07,     // Class=NAV(0x01), ID=PVT(0x07) (2 bytes)
    0x5C, 0x00,     // Length=92 bytes (2 bytes)
    // === Payload starts (92 bytes) ===
    0x10, 0x27, 0x00, 0x00,  // iTOW (GPS time of week) ms
    0xE7, 0x07, 0x0C, 0x1F,  // year=2023, month=12, day=31
    0x17, 0x1E, 0x0F,        // hour=23, min=30, sec=15
    0x07,                    // valid (all flags valid)
    0xD0, 0x07, 0x00, 0x00,  // tAcc (time accuracy) ns
    0x00, 0x00, 0x00, 0x00,  // nano (fraction of second) ns
    0x03,                    // fixType = 3D-fix
    0x01,                    // flags (gnssFixOK)
    0x00,                    // flags2
    0x0C,                    // numSV = 12 satellites
    0x5A, 0xE9, 0xFD, 0x18,  // lon = 25.2795° * 1e7 (Tallinn longitude)
    0x5B, 0x5B, 0xCB, 0x03,  // lat = 59.4370° * 1e7 (Tallinn latitude)
    0xF4, 0x01, 0x00, 0x00,  // height = 500mm above ellipsoid
    0x2C, 0x01, 0x00, 0x00,  // hMSL = 300mm above sea level
    0xE8, 0x03, 0x00, 0x00,  // hAcc = 1000mm horizontal accuracy
    0xF4, 0x01, 0x00, 0x00,  // vAcc = 500mm vertical accuracy
    0x00, 0x00, 0x00, 0x00,  // velN = 0 mm/s north velocity
    0x00, 0x00, 0x00, 0x00,  // velE = 0 mm/s east velocity
    0x00, 0x00, 0x00, 0x00,  // velD = 0 mm/s down velocity
    0x00, 0x00, 0x00, 0x00,  // gSpeed = 0 mm/s ground speed
    0x00, 0x00, 0x00, 0x00,  // headMot = 0° * 1e-5 heading of motion
    0xE8, 0x03, 0x00, 0x00,  // sAcc = 1000 mm/s speed accuracy
    0x00, 0x00, 0x00, 0x00,  // headAcc = 0° * 1e-5 heading accuracy
    0x64, 0x00,              // pDOP = 1.00 * 0.01
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // flags3 + reserved
    0x00, 0x00, 0x00, 0x00,  // headVeh = 0° * 1e-5 vehicle heading
    0x00, 0x00,              // magDec = 0° * 1e-2 magnetic declination
    0x00, 0x00,              // magAcc = 0° * 1e-2 magnetic accuracy estimate
    // === Checksum (2 bytes) - calculated over bytes 2-97 ===
    0x13, 0xB3               // CK_A, CK_B
};

// NAV-DOP message (dilution of precision)
static const uint8_t nav_dop_msg[] = {
    0xB5, 0x62,     // UBX header
    0x01, 0x04,     // Class=NAV, ID=DOP
    0x12, 0x00,     // Length=18 bytes
    0x00, 0x00, 0x00, 0x00,  // iTOW
    0x64, 0x00,     // gDOP = 1.00
    0x64, 0x00,     // pDOP = 1.00
    0x64, 0x00,     // tDOP = 1.00
    0x64, 0x00,     // vDOP = 1.00
    0x64, 0x00,     // hDOP = 1.00
    0x64, 0x00,     // nDOP = 1.00
    0x64, 0x00,     // eDOP = 1.00
    0xD3, 0xB2      // Checksum
};

// MON-VER message (receiver/software version)
static const uint8_t mon_ver_msg[] = {
    0xB5, 0x62,     // UBX header
    0x0A, 0x04,     // Class=MON, ID=VER
    0x28, 0x00,     // Length=40 bytes (minimal version)
    'E','X','A','M','P','L','E',' ',
    'V','1','.','0','0',' ',' ',' ',
    ' ',' ',' ',' ',' ',' ',' ',' ',
    ' ',' ',' ',' ',' ',' ',' ',' ',
    'M','o','c','k',' ','G','P','S',
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x6B, 0xD6      // Checksum
};

// NAV-SAT message (satellite information for 8 satellites)
// Header(2) + Class/ID(2) + Length(2) + 8bytes + (12bytes * 8 sats) + Checksum(2) = 112 bytes
static const uint8_t nav_sat_msg[] = {
    0xB5, 0x62,     // UBX header
    0x01, 0x35,     // Class=NAV(0x01), ID=SAT(0x35)
    0x68, 0x00,     // Length=104 bytes (8 header + 12*8 satellites)
    // === Payload (104 bytes) ===
    0x10, 0x27, 0x00, 0x00,  // iTOW (GPS time of week) ms
    0x01,                    // version = 1
    0x08,                    // numSvs = 8 satellites
    0x00, 0x00,              // reserved
    // Satellite block 1 (GPS PRN 5)
    0x05, 0x00, 180, 45, 0x1B, 0x00, 0x00, 0x00, 42, 30, 0x07, 0x03,
    // Satellite block 2 (GPS PRN 12)
    0x0C, 0x00, 270, 30, 0x1C, 0x00, 0x00, 0x00, 45, 32, 0x07, 0x03,
    // Satellite block 3 (GPS PRN 15)
    0x0F, 0x00, 90, 60, 0x1D, 0x00, 0x00, 0x00, 38, 28, 0x07, 0x03,
    // Satellite block 4 (GPS PRN 24)
    0x18, 0x00, 45, 15, 0x1E, 0x00, 0x00, 0x00, 35, 25, 0x07, 0x03,
    // Satellite block 5 (GPS PRN 25)
    0x19, 0x00, 135, 45, 0x1F, 0x00, 0x00, 0x00, 40, 29, 0x07, 0x03,
    // Satellite block 6 (GPS PRN 29)
    0x1D, 0x00, 225, 60, 0x20, 0x00, 0x00, 0x00, 43, 31, 0x07, 0x03,
    // Satellite block 7 (GLONASS slot 1)
    0x41, 0x06, 315, 30, 0x10, 0x00, 0x00, 0x00, 36, 27, 0x07, 0x03,
    // Satellite block 8 (GLONASS slot 2)
    0x42, 0x06, 150, 50, 0x11, 0x00, 0x00, 0x00, 39, 28, 0x07, 0x03,
    // === Checksum (2 bytes) ===
    0xF2, 0x21               // CK_A, CK_B
};

static const ubx_sample_msg_t sample_messages[] = {
    {nav_pvt_msg, sizeof(nav_pvt_msg), "NAV-PVT"},       // 100 bytes
    {nav_sat_msg, sizeof(nav_sat_msg), "NAV-SAT"},       // 112 bytes
    {nav_dop_msg, sizeof(nav_dop_msg), "NAV-DOP"},       // 26 bytes
    {ack_ack_msg, sizeof(ack_ack_msg), "ACK-ACK"},       // 10 bytes
    {ack_nak_msg, sizeof(ack_nak_msg), "ACK-NAK"},       // 10 bytes
    {mon_ver_msg, sizeof(mon_ver_msg), "MON-VER"}        // 48 bytes
};

static const size_t num_sample_messages = sizeof(sample_messages) / sizeof(sample_messages[0]);
static size_t current_msg_index = 0;
static int64_t last_generation_time = 0;

int mock_uart_init(void) {
    ESP_LOGI(TAG, "Mock UART initialized");
    current_msg_index = 0;
    last_generation_time = 0;
    return 0;
}

int mock_uart_generate_data(uint8_t *buffer, size_t max_len) {
    if (!buffer || max_len == 0) {
        return 0;
    }

    int64_t current_time = esp_timer_get_time() / 1000; // Convert to ms

    // Generate data at configured interval
    if (current_time - last_generation_time < MOCK_DATA_INTERVAL_MS) {
        return 0;
    }

    last_generation_time = current_time;

    // Get next message from sequence
    const ubx_sample_msg_t *msg = &sample_messages[current_msg_index];

    if (msg->len > max_len) {
        ESP_LOGW(TAG, "Buffer too small for message %s", msg->description);
        return 0;
    }

    // Copy message to buffer
    memcpy(buffer, msg->data, msg->len);

    // Silent mode for stress testing - no logging
    // ESP_LOGI(TAG, "Generated mock message: %s (%d bytes)", msg->description, msg->len);

    // Move to next message (cycle through)
    current_msg_index = (current_msg_index + 1) % num_sample_messages;

    return (int)msg->len;
}

void mock_uart_deinit(void) {
    ESP_LOGI(TAG, "Mock UART deinitialized");
}
