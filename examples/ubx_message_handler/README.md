# UBX Message Handler Example

This example demonstrates the UBX message handling functionality from the `logger_ubx` component. It runs as a standalone application with its own `app_main()`, showing how to:

- Parse UBX protocol messages
- Validate message checksums
- Handle different message types (NAV-PVT, NAV-DOP, ACK-ACK, etc.)
- Work with circular buffers for UART data
- Process GNSS data from real or simulated sources

## Features

### Dual Operation Modes

1. **Mock UART Mode** (Default)
   - Generates simulated UBX messages
   - Perfect for testing without hardware
   - Demonstrates all message types
   - No GPS module required

2. **Real UART Mode**
   - Connects to actual UBX GPS module
   - Uses initialization functions from `ubx.c`
   - Full GPS functionality
   - Requires hardware connection

### Demonstrated Functionality

- ✅ **Circular Buffer Management**: Shows buffer state and data flow
- ✅ **Checksum Validation**: Demonstrates UBX checksum calculation and verification
- ✅ **Message Type Identification**: Automatically identifies message classes and IDs
- ✅ **Message Parsing**: Extracts and displays message payloads
- ✅ **Statistics Tracking**: Counts valid/invalid messages and success rates

## Configuration

Edit [`config.h`](config.h) to configure the example:

```c
// Select mode: 0 = Mock UART, 1 = Real UART
#define USE_REAL_UART 0

#if USE_REAL_UART
    // Real UART configuration
    #define UART_NUM UART_NUM_1
    #define UART_TX_PIN 17
    #define UART_RX_PIN 16
    #define UART_BAUD_RATE 9600
    #define UART_BUF_SIZE 1024
#else
    // Mock UART settings
    #define MOCK_DATA_INTERVAL_MS 100
#endif

// Example demonstration settings
#define DEMO_DURATION_MS 10000          // Run for 10 seconds
#define LOG_OUTPUT_INTERVAL_MS 500      // Status updates every 500ms
```

## Hardware Requirements

### Mock UART Mode
- No hardware required
- Runs on any ESP32 development board

### Real UART Mode
- ESP32 development board
- UBX-compatible GPS module (e.g., NEO-M8N, NEO-M9N)
- Connections:
  - ESP32 TX → GPS RX
  - ESP32 RX → GPS TX
  - GPS VCC → 3.3V or 5V (check module specs)
  - GPS GND → GND

## Building and Running

### Option 1: ESP-IDF (Recommended)

```bash
# Navigate to example directory
cd components/logger_ubx/examples/ubx_message_handler

# Set up ESP-IDF environment
source ~/esp/esp-idf/export.sh

# Build the example
idf.py build

# Flash to device
idf.py flash monitor
```

### Option 2: PlatformIO

Add to your `platformio.ini`:

```ini
[env:ubx_example]
platform = espressif32
board = esp32dev
framework = espidf
build_src_filter = 
    +<components/logger_ubx/examples/ubx_message_handler/>
```

Then build and upload:
```bash
pio run -e ubx_example --target upload --target monitor
```

## Expected Output

### Mock UART Mode

```
I (1234) ubx_example: ===========================================
I (1235) ubx_example: UBX Message Handler Example Application
I (1236) ubx_example: ===========================================
I (1237) ubx_example: Mode: MOCK UART (simulated UBX data)
I (1238) mock_uart: Mock UART initialized
I (1240) ubx_example: Running demonstration for 10000 ms...
I (1340) mock_uart: Generated mock message: ACK-ACK (10 bytes)
I (1341) ubx_example: Generated 10 bytes of mock UBX data
I (1342) ubx_example: 
=== Checksum Validation Demonstration ===
I (1343) ubx_example: Calculated: CK_A=0x0F, CK_B=0x52
I (1344) ubx_example: Expected:   CK_A=0x0F, CK_B=0x52
I (1345) ubx_example: ✓ Checksum VALID
I (1346) ubx_example: Message: ACK-ACK (Class: 0x05, ID: 0x01, Len: 10)
I (1440) mock_uart: Generated mock message: NAV-PVT (100 bytes)
I (1441) ubx_example: Generated 100 bytes of mock UBX data
I (1442) ubx_example: Message: NAV-PVT (Class: 0x01, ID: 0x07, Len: 100)
I (1443) ubx_example:   Fix Type: 3, Satellites: 8
I (1444) ubx_example:   Lat: 637534208, Lon: 452984832, Height: 1000 mm
I (1750) ubx_example: Status: Received=5, Valid=5, Invalid=0
I (1751) ubx_example: 
=== Circular Buffer Demonstration ===
I (1752) ubx_example: Buffer size: 0 bytes
I (1753) ubx_example: Buffer head: 0, tail: 0
I (1754) ubx_example: Available data: 0 bytes
...
I (11245) ubx_example: 
===========================================
I (11246) ubx_example: Demonstration Complete
I (11247) ubx_example: ===========================================
I (11248) ubx_example: Duration: 10005 ms
I (11249) ubx_example: Messages Received: 100
I (11250) ubx_example: Messages Valid:    100
I (11251) ubx_example: Messages Invalid:  0
I (11252) ubx_example: Success Rate: 100.0%
I (11253) ubx_example: Example finished. Restart to run again.
```

### Real UART Mode

```
I (1234) ubx_example: ===========================================
I (1235) ubx_example: UBX Message Handler Example Application
I (1236) ubx_example: ===========================================
I (1237) ubx_example: Mode: REAL UART (connected to UBX GPS module)
I (1238) ubx_example: Initializing real UART for UBX module...
I (1250) ubx: UART initialized on port 1 (TX: 17, RX: 16, Baud: 9600)
I (1251) ubx: UBX module detected: NEO-M8N
I (1252) ubx_example: Real UART initialized successfully
I (1253) ubx_example: UART: 1, TX: 17, RX: 16, Baud: 9600
I (1254) ubx_example: Running demonstration for 10000 ms...
I (2456) ubx_example: Received message from UBX module
I (2457) ubx_example: Message: NAV-PVT (Class: 0x01, ID: 0x07, Len: 92)
I (2458) ubx_example:   Fix Type: 3, Satellites: 12
I (2459) ubx_example:   Lat: 374128640, Lon: -1221939200, Height: 45320 mm
...
```

## Code Structure

```
ubx_message_handler/
├── main.c              # Main application with app_main()
├── config.h            # Configuration (mock vs real UART)
├── mock_uart.h         # Mock UART interface
├── mock_uart.c         # Mock UART implementation
├── CMakeLists.txt      # Build configuration
└── README.md           # This file
```

## Key Functions

### From `ubx.c` (Real UART Mode)
- `ubx_ctx_new()` - Create UBX context
- `ubx_setup()` - Initialize GPS module
- `ubx_on()` - Power on and configure UART
- `ubx_off()` - Power off module

### From `ubx_msg_handler.c`
- `ubx_msg_type_handler()` - Identify message type
- `msg_checksum_cb()` - Validate checksum
- `add_checksum()` - Calculate UBX checksum
- `ubx_read_frame()` - Read message frame

### Example-Specific
- `app_main()` - Main entry point
- `process_ubx_data()` - Process received messages
- `display_message_info()` - Show message details
- `demo_circular_buffer()` - Demonstrate buffer usage
- `demo_checksum_validation()` - Show checksum calculation

## Message Types Demonstrated

| Type | Class | ID | Description |
|------|-------|----|----|
| ACK-ACK | 0x05 | 0x01 | Message acknowledged |
| ACK-NAK | 0x05 | 0x00 | Message not acknowledged |
| NAV-PVT | 0x01 | 0x07 | Position, velocity, time solution |
| NAV-DOP | 0x01 | 0x04 | Dilution of precision |
| MON-VER | 0x0A | 0x04 | Receiver/software version |
| CFG-MSG | 0x06 | 0x01 | Message configuration |

## Extending the Example

### Adding New Message Types

1. Define the message structure in `ubx_msg.h` (if not already present)
2. Add case to `display_message_info()` in `main.c`
3. Add sample message to `mock_uart.c` (for mock mode)

### Customizing Mock Data

Edit [`mock_uart.c`](mock_uart.c) to add/modify sample messages:

```c
// Add new sample message
static const uint8_t my_custom_msg[] = {
    0xB5, 0x62,     // UBX header
    0xXX, 0xYY,     // Class and ID
    0xLL, 0xLL,     // Length (little endian)
    // ... payload ...
    0xCK_A, 0xCK_B  // Checksum
};
```

## Troubleshooting

### Mock Mode Issues

**Problem**: No messages generated
- **Solution**: Check `MOCK_DATA_INTERVAL_MS` in `config.h`

**Problem**: Checksum errors
- **Solution**: Verify message data in `mock_uart.c` matches expected format

### Real UART Mode Issues

**Problem**: UART initialization fails
- **Solution**: Check GPIO pin configuration in `config.h`
- **Solution**: Verify GPS module is powered and connected

**Problem**: No data received
- **Solution**: Check baud rate matches GPS module (typically 9600)
- **Solution**: Verify TX/RX pins are not swapped
- **Solution**: Ensure GPS module has clear view of sky for signal

**Problem**: Invalid checksums
- **Solution**: Check for electrical noise or poor connections
- **Solution**: Try lower baud rate
- **Solution**: Add pull-up resistors if needed

## References

- [u-blox Protocol Specification](https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf)
- [ESP-IDF UART Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html)
- Original implementation: [esp32-ublox](https://github.com/aedalzotto/esp32-ublox)

## License

This example is part of the esp-gps-logger project and follows the same license.
