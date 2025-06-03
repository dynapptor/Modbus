# Modbus Library

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform: AVR](https://img.shields.io/badge/Platform-AVR-blue.svg)](https://www.arduino.cc)

## Introduction

The **Modbus** library is a lightweight, high-performance Modbus client for Arduino and AVR microcontrollers, supporting Modbus RTU (over RS485, RS422, or RS232) and Modbus TCP (over Ethernet). Optimized for embedded systems, it provides asynchronous communication, multi-slave management, and robust error handling, making it ideal for industrial automation, IoT, sensor networks, PLC integration, and HVAC control.

### Key Features

- **Lightweight and AVR-Optimized**: Minimal memory footprint, no reliance on `std::` member functions for callbacks, ensuring compatibility with AVR platforms like Arduino Uno and Mega.
- **Flexible Protocol Support**: Supports Modbus RTU over RS485, RS422, RS232, as well as Modbus TCP over Ethernet, adaptable to various hardware setups.
- **Asynchronous Operation**: Non-blocking communication with callback-based response handling for real-time applications.
- **Multi-Slave Management**: Efficiently polls multiple slaves with configurable delays using the `Slaves` class, perfect for sensor arrays or distributed systems.
- **Exclusive Broadcast Mode**: Supports RTU broadcast (slave ID = 0) with scheduled repeats, simplifying mass configuration.
- **Type-Safe API**: Template-based methods (e.g., `readHoldingRegisters<T>`) support various data types for flexibility.
- **Robust Error Handling**: Comprehensive error codes (e.g., `MB_EX_SUCCESS`) and callbacks ensure reliable operation.
- **Low Latency**: Optimized for minimal communication overhead, critical for time-sensitive applications.
- **Configurable Timeouts**: Customizable frame, byte, and response timeouts for fine-tuned communication.
- **Cross-Platform**: Portable to other Arduino-compatible platforms (e.g., ESP32, SAMD).
- **Industrial-Grade**: Suitable for real-world industrial environments, such as PLC integration, sensor networks, and HVAC control.
- **Minimal Dependencies**: Requires only a custom `Callback` library and a lightweight `initializer_list` for AVR.
- **Extensive Examples**: Practical examples demonstrate usage, using `Serial1` for Modbus and `Serial` for debugging.
- **Community-Driven**: MIT-licensed, open to contributions via `CONTRIBUTING.md`.

## Installation

1. **Download**: Clone or download the repository from [GitHub](https://github.com/dynapptor/Modbus).
2. **Arduino IDE**: Install via the Library Manager (search for `Modbus`) or copy the folder to your Arduino libraries directory.
3. **Dependencies**:
   - [Callback](https://github.com/dynapptor/Callback): Custom callback library for asynchronous operations.
   - [utils](https://github.com/dynapptor/Utility-lib): Helper functions.
   - [Initializer List](https://github.com/dynapptor/initializer_list): Lightweight C++11 implementation for AVR, included in `lib/initializer_list`.
4. **AVR Note**: For AVR platforms (e.g., Arduino Uno, Mega), ensure the `initializer_list` header is included in your project. Copy `lib/initializer_list/initializer_list` to your sketch directory or library path if not automatically included.
5. **Include**: Add `#include <ModbusRTUMaster.h>` to your sketch or `#include <ModbusTCPMaster.h>` for TCP.

## Usage

Below is an example demonstrating Modbus RTU communication using `Serial1` for Modbus and `Serial` for debugging on an Arduino Mega.

```cpp
#include <ModbusRTUMaster.h>

ModbusRTUMaster master;

void setup() {
  Serial.begin(115200);  // Debugging output
  Serial1.begin(9600);   // Modbus RTU communication
  // Initialize RTU master: PDU size 253, queue size 1, Serial1, 9600 baud, 8N1
  master.begin(MB_PDU_MAX_SIZE, 1, &Serial1, 9600, UartConfig::Mode_8N1, 3, 4);

  // Read coil at address 0
  modbusCallback cb([](PDU& pdu) {
    if (pdu.getErr() == MB_EX_SUCCESS) {
      Serial.print("Coil value: ");
      Serial.println(pdu.getBit(0));
    } else {
      Serial.print("Error: ");
      Serial.println(pdu.getErr());
    }
  });

  master.readCoil(1, 0, cb);
}

void loop() {
  master.loop();
}

For broadcast operations, see the examples/BroadcastWriteCoilRTU example. Additional examples for multi-slave polling and register operations are in the examples folder.

Documentation
    API Reference: Detailed method descriptions in doc/api.md.
    Examples: Practical usage in examples/.
    Callback Library: See Callback for callback handling.

Contributing
Contributions are welcome! Please read CONTRIBUTING.md for guidelines. Report issues or suggest features on GitHub Issues.

License
This library is licensed under the MIT License. See LICENSE.txt for details.

Acknowledgments
Inspired by Modbus implementations like ArduinoModbus.
Thanks to contributors for their support and feedback.

```
