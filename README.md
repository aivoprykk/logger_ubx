# Logger UBX GPS Component

A comprehensive ESP-IDF driver for u-blox GPS receivers, providing high-performance GPS data acquisition, configuration management, and power control for GPS tracking applications.

## Features

### Hardware Support
- **u-blox Chipsets**: M0, M7, M8, M9, M10 series support
- **UART Communication**: Configurable baud rates (9600-230400)
- **Power Management**: Multiple enable pins for power control
- **Hardware Auto-Detection**: Automatic chipset identification

### GNSS Constellations
- **GPS**: Global Positioning System
- **SBAS**: Satellite-Based Augmentation System
- **Galileo**: European GNSS
- **BeiDou**: Chinese GNSS
- **QZSS**: Quasi-Zenith Satellite System
- **GLONASS**: Russian GNSS
- **NavIC**: Indian Regional Navigation Satellite System

### Navigation Modes
- **Portable**: Automatic mode for general use
- **Pedestrian**: Optimized for walking speeds
- **Automotive**: Optimized for vehicle navigation
- **Sea**: Marine navigation mode
- **Airborne**: Aviation modes (1G, 2G, 4G acceleration limits)

### Data Output
- **Configurable Rates**: 1Hz, 2Hz, 5Hz, 10Hz, 16Hz, 20Hz
- **UBX Protocol**: Binary messages for high performance
- **NMEA Messages**: Standard text format support
- **Real-time Data**: Position, velocity, time, satellite information

### Message Types
- **NAV-PVT**: Position, velocity, time solution
- **NAV-SAT**: Satellite information and signal quality
- **NAV-DOP**: Dilution of precision
- **MON-GNSS**: GNSS system status
- **MON-VER**: Hardware/firmware version

## Installation

### ESP-IDF Integration
Add to your `main/CMakeLists.txt`:
```cmake
idf_component_register(SRCS "main.c"
                      INCLUDE_DIRS "."
                      REQUIRES logger_ubx)
```

### PlatformIO
Add to your `platformio.ini`:
```ini
[env]
lib_deps =
    https://github.com/aivoprykk/esp-gps-logger.git#components/logger_ubx
```

## Configuration

### Kconfig Options
Configure via `idf.py menuconfig`:

- **UBLOX_ENABLED**: Enable/disable UBX GPS module
- **UBLOX_UART_PORT**: UART port selection (default 1 or 2)
- **UART Pins**: TX/RX pin configuration
- **Power Pins**: Up to 3 enable pins for GPS power control

### Hardware Pin Configuration
```c
// Example for LilyGO T-Display S3
#define CONFIG_UBLOX_UART_PORT 2
#define CONFIG_UBLOX_UART_RXD 18
#define CONFIG_UBLOX_UART_TXD 17
#define CONFIG_UBLOX_UART_PWR_1 21
#define CONFIG_UBLOX_UART_PWR_2 16
#define CONFIG_UBLOX_UART_PWR_3 -1  // Not used
```

## Usage

### Basic Initialization
```c
#include "ubx.h"

// Create GPS configuration
ubx_ctx_t *gps_config = ubx_ctx_new();
if (!gps_config) {
    ESP_LOGE(TAG, "Failed to create GPS config");
    return;
}

// Power on GPS
esp_err_t err = ubx_on(gps_config);
if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to power on GPS: %s", esp_err_to_name(err));
    return;
}

// Setup GPS with default configuration
err = ubx_setup(gps_config);
if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to setup GPS: %s", esp_err_to_name(err));
    return;
}

// Wait for GPS to be ready
while (!gps_config->ready) {
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

ESP_LOGI(TAG, "GPS ready! Chip: %s", ubx_chip_str(gps_config));
```

### Configuration Setup
```c
// Configure GNSS constellations (GPS + Galileo + BeiDou + GLONASS)
uint8_t gnss_config = 0;
gnss_config |= (1 << UBX_GNSS_GPS);
gnss_config |= (1 << UBX_GNSS_GALILEO);
gnss_config |= (1 << UBX_GNSS_BEIDOU);
gnss_config |= (1 << UBX_GNSS_GLONASS);

// Set navigation mode and output rate
esp_err_t err = ubx_set_nav_mode(gps_config, UBX_MODE_AUTOMOTIVE);
if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set nav mode");
}

err = ubx_set_ggnss_and_rate(gps_config, gnss_config, UBX_OUTPUT_10HZ);
if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set GNSS config");
}
```

### GPS Data Access
```c
#include "ubx_msg.h"

// Access NAV-PVT data
nav_pvt_t *nav_data = &gps_config->ubx_msg.nav_pvt;

if (nav_data->fixType >= 2) {  // 2D or 3D fix
    float latitude = nav_data->lat / 10000000.0f;   // Convert to degrees
    float longitude = nav_data->lon / 10000000.0f;  // Convert to degrees
    float altitude = nav_data->hMSL / 1000.0f;      // Convert to meters
    float speed = nav_data->gSpeed / 1000.0f;       // Convert to m/s
    float heading = nav_data->heading / 100000.0f;  // Convert to degrees

    ESP_LOGI(TAG, "Position: %.6f, %.6f", latitude, longitude);
    ESP_LOGI(TAG, "Altitude: %.1f m, Speed: %.1f m/s", altitude, speed);
    ESP_LOGI(TAG, "Heading: %.1f°, Satellites: %d", heading, nav_data->numSV);
}

// Access satellite information
if (gps_config->ubx_msg.nav_sat.count > 0) {
    ESP_LOGI(TAG, "Visible satellites: %d", gps_config->ubx_msg.nav_sat.count);
    for (int i = 0; i < gps_config->ubx_msg.nav_sat.count; i++) {
        ubx_sat_t *sat = &gps_config->ubx_msg.nav_sat.sats[i];
        ESP_LOGI(TAG, "Sat %d: PRN=%d, CNO=%d, Elevation=%d",
                 i, sat->gnssId * 32 + sat->svId, sat->cno, sat->elev);
    }
}
```

### Time Synchronization
```c
// Set system time from GPS
if (nav_data->valid & 0x03) {  // UTC date and time valid
    struct tm gps_time = {
        .tm_year = nav_data->year - 1900,
        .tm_mon = nav_data->month - 1,
        .tm_mday = nav_data->day,
        .tm_hour = nav_data->hour,
        .tm_min = nav_data->minute,
        .tm_sec = nav_data->second,
    };

    time_t gps_epoch = mktime(&gps_time);
    struct timeval tv = { .tv_sec = gps_epoch, .tv_usec = nav_data->nano / 1000 };
    settimeofday(&tv, NULL);

    ESP_LOGI(TAG, "System time synchronized from GPS");
}
```

### Event Handling
```c
#include "ubx_events.h"

// Register for GPS events
esp_event_handler_register(UBX_EVENT, ESP_EVENT_ANY_ID, gps_event_handler, NULL);

static void gps_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
    switch (event_id) {
        case UBX_EVENT_SETUP_DONE:
            ESP_LOGI(TAG, "GPS setup completed");
            break;
        case UBX_EVENT_MSG_RECIEVED:
            ESP_LOGI(TAG, "GPS message received");
            break;
        case UBX_EVENT_DATETIME_SET:
            ESP_LOGI(TAG, "GPS date/time set");
            break;
        case UBX_EVENT_SAMPLE_RATE_CHANGED:
            ESP_LOGI(TAG, "GPS sample rate changed");
            break;
    }
}
```

### Power Management
```c
// Power off GPS to save energy
esp_err_t err = ubx_off(gps_config);
if (err == ESP_OK) {
    ESP_LOGI(TAG, "GPS powered off");
}

// Power on GPS when needed
err = ubx_on(gps_config);
if (err == ESP_OK) {
    ESP_LOGI(TAG, "GPS powered on");
}
```

## API Reference

### Core Functions
- `ubx_ctx_new()` / `ubx_ctx_delete()`: Create/delete GPS configuration
- `ubx_on()` / `ubx_off()`: Power control
- `ubx_setup()`: Initialize GPS with configuration
- `ubx_set_nav_mode()`: Set navigation mode
- `ubx_set_ggnss_and_rate()`: Configure GNSS and output rate

### Configuration Enums
- **Hardware Types**: `UBX_TYPE_M0`, `UBX_TYPE_M7`, `UBX_TYPE_M8`, `UBX_TYPE_M9`, `UBX_TYPE_M10`
- **Navigation Modes**: `UBX_MODE_PORTABLE`, `UBX_MODE_PEDESTRIAN`, `UBX_MODE_AUTOMOTIVE`, etc.
- **GNSS Systems**: `UBX_GNSS_GPS`, `UBX_GNSS_GALILEO`, `UBX_GNSS_BEIDOU`, etc.
- **Output Rates**: `UBX_OUTPUT_1HZ`, `UBX_OUTPUT_5HZ`, `UBX_OUTPUT_10HZ`, etc.

### Message Structures
- **nav_pvt_t**: Position, velocity, time data (92 bytes)
- **nav_sat_t**: Satellite information
- **nav_dop_t**: Dilution of precision
- **mon_gnss_t**: GNSS system status

## NAV-PVT Data Fields

| Field | Type | Description | Units |
|-------|------|-------------|-------|
| `iTOW` | uint32_t | GPS time of week | ms |
| `year/month/day` | uint16/8/8 | UTC date | - |
| `hour/minute/second` | uint8 | UTC time | - |
| `lon/lat` | int32_t | Longitude/Latitude | ° × 10^7 |
| `height/hMSL` | int32_t | Height above ellipsoid/MSL | mm |
| `hAcc/vAcc` | uint32_t | Horizontal/Vertical accuracy | mm |
| `velN/velE/velD` | int32_t | NED velocity components | mm/s |
| `gSpeed` | int32_t | Ground speed (2D) | mm/s |
| `heading` | int32_t | Heading of motion | ° × 10^5 |
| `numSV` | uint8_t | Number of satellites used | - |
| `fixType` | uint8_t | GNSS fix type (0-5) | - |

### Fix Types
- **0**: No fix
- **1**: Dead reckoning only
- **2**: 2D fix
- **3**: 3D fix
- **4**: GNSS + dead reckoning
- **5**: Time only fix

## Performance Considerations

### UART Configuration
- **Baud Rate**: Higher rates (115200+) for high-frequency output
- **Buffer Size**: Adequate UART buffers for continuous data stream
- **Flow Control**: Hardware flow control for reliable high-speed communication

### Power Consumption
- **Active Mode**: ~25-50mA depending on chipset
- **Power Control**: Use enable pins to completely power down GPS
- **Duty Cycling**: Power on/off GPS based on application needs

### Data Rates
- **1-5Hz**: Suitable for most tracking applications
- **10-20Hz**: High-performance navigation and racing
- **Message Load**: Higher rates increase UART bandwidth requirements

## Troubleshooting

### Common Issues
1. **No GPS Fix**: Check antenna connection and sky visibility
2. **UART Communication**: Verify pin connections and UART configuration
3. **Power Issues**: Ensure adequate power supply and enable pin setup
4. **Configuration Errors**: Check GNSS constellation and navigation mode settings

### Debug Information
```c
// Check GPS status
ESP_LOGI(TAG, "GPS ready: %s", gps_config->ready ? "YES" : "NO");
ESP_LOGI(TAG, "UART setup: %s", gps_config->uart_is_on ? "OK" : "FAIL");
ESP_LOGI(TAG, "Chip type: %s", ubx_chip_str(gps_config));
ESP_LOGI(TAG, "Baud rate: %s", ubx_baud_str(gps_config));

// Check NAV-PVT data validity
ESP_LOGI(TAG, "Fix type: %d, Satellites: %d", nav_data->fixType, nav_data->numSV);
ESP_LOGI(TAG, "Valid flags: 0x%02X", nav_data->valid);
```

### Log Levels
Configure via Kconfig:
- TRACE: Detailed message parsing and UART communication
- DEBUG: Configuration and setup operations
- INFO: Major GPS events and status changes
- ERROR: Critical errors and communication failures

## Hardware Compatibility

### Supported u-blox Modules
- **NEO-M8N**: General-purpose GPS/GLONASS receiver
- **NEO-M8P**: High-precision RTK-capable receiver
- **NEO-M8Q**: Low-power GPS receiver
- **MAX-M8**: Automotive-grade receiver

### ESP32 Pin Requirements
- **UART**: TX/RX pins (any available UART pins)
- **Power**: 1-3 GPIO pins for enable/power control
- **Antenna**: Active/passive GPS antenna connection

### Power Supply
- **Voltage**: 3.3V or 5V depending on module
- **Current**: 25-50mA active, <10μA standby
- **Backup**: Optional backup battery for hot starts

## Dependencies

- ESP-IDF v4.4+
- logger_common component
- ESP32 UART driver

## License

See LICENSE file in component directory.
