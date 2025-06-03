= Modbus Library API Reference
:author: Ferenc Mayer
:email: Ferenc Mayer<dynapptor@dynapptor.com>
:revnumber: 1.0.0
:revdate: 2025-05-28
:toc: left
:icons: font
:linkattrs:
:sectlinks:
:docinfo: shared

== Overview

The *Modbus* library is a lightweight, high-performance Modbus client for Arduino and AVR microcontrollers, supporting Modbus RTU (over RS485, RS422, or RS232) and Modbus TCP (over Ethernet). Optimized for embedded systems, it provides asynchronous communication, multi-slave management, and robust error handling, making it ideal for industrial automation, IoT, sensor networks, PLC integration, and HVAC control.

The library consists of several key classes:

* *ModbusMaster*: Abstract base class for Modbus communication, defining the core API for coil, register, and diagnostic operations.
* *ModbusRTUMaster*: Implements Modbus RTU over serial interfaces (RS485, RS422, RS232).
* *ModbusTCPClient*: Implements Modbus TCP over Ethernet.
* *Slaves*: Manages sets of slave IDs for multi-slave polling or broadcast operations.
* *PDU*: Represents a Modbus Protocol Data Unit, used in callbacks to handle responses.

Key features include type-safe templates, exclusive broadcast mode (slave ID = 0), and minimal dependencies (custom `Callback` library and `initializer_list` for AVR).

== Classes

=== ModbusMaster

Abstract base class for Modbus communication, providing a unified API for RTU and TCP operations.

==== Methods

===== writeSingleCoil

[source,cpp]
----
void writeSingleCoil(Slaves& slaves, uint16_t address, bool value, callback& cb);
void writeSingleCoil(uint8_t slave, uint16_t address, bool value, callback& cb);
----

*Description*: Writes a single coil at the specified address.

*Parameters*:
- `slaves`: `Slaves` object specifying one or more slave IDs.
- `slave`: Single slave ID (0–247, or 0 for RTU broadcast).
- `address`: Coil address (0–65535).
- `value`: Boolean value to write (`true` or `false`).
- `cb`: Callback function (`Callback<void, PDU&>`) to handle the response.

*Notes*:
- Broadcast (slave = 0) is supported for RTU, uses `MB_EX_SUCCESS` (0) in callback.
- Asynchronous; response is processed via `cb`.

*Example*:
[source,cpp]
----
master.writeSingleCoil(1, 0, true, [](PDU& pdu) {
  if (pdu.getErr() == MB_EX_SUCCESS) {
    Serial.println("Coil written");
  }
});
----

===== writeCoils

[source,cpp]
----
void writeCoils(Slaves& slaves, uint16_t address, const uint8_t* src, uint8_t byteCount, uint16_t coilCount, callback& cb);
void writeCoils(uint8_t slave, uint16_t address, const uint8_t* src, uint8_t byteCount, uint16_t coilCount, callback& cb);
void writeCoils(Slaves& slaves, uint16_t address, const bool* values, uint16_t count, callback& cb);
void writeCoils(uint8_t slave, uint16_t address, const bool* values, uint16_t count, callback& cb);
void writeCoils(Slaves& slaves, uint16_t address, std::initializer_list<bool> values, callback& cb);
void writeCoils(uint8_t slave, uint16_t address, std::initializer_list<bool> values, callback& cb);
----

*Description*: Writes multiple coils starting at the specified address.

*Parameters*:
- `slaves`, `slave`: Slave IDs or single ID (0 for RTU broadcast).
- `address`: Starting coil address (0–65535).
- `src`: Byte array (8 coils per byte) for coil values.
- `byteCount`: Number of bytes in `src`.
- `coilCount`: Number of coils to write.
- `values`: Boolean array or initializer list of coil values.
- `count`: Number of coils in `values`.
- `cb`: Response callback.

*Notes*:
- Supports byte arrays, boolean arrays, or initializer lists for flexibility.
- Broadcast supported for RTU.

*Example*:
[source,cpp]
----
master.writeCoils(1, 0, {true, false, true}, [](PDU& pdu) {
  if (pdu.getErr() == MB_EX_SUCCESS) {
    Serial.println("Coils written");
  }
});
----

===== readCoil

[source,cpp]
----
void readCoil(Slaves& slaves, uint16_t address, callback& cb);
void readCoil(uint8_t slave, uint16_t address, callback& cb);
----

*Description*: Reads a single coil at the specified address.

*Parameters*:
- `slaves`, `slave`: Slave IDs or single ID (1–247, broadcast not supported).
- `address`: Coil address (0–65535).
- `cb`: Response callback.

*Notes*:
- Broadcast (slave = 0) is invalid, triggers `MB_EX_LIB_INVALID_SLAVE`.

*Example*:
[source,cpp]
----
master.readCoil(1, 0, [](PDU& pdu) {
  if (pdu.getErr() == MB_EX_SUCCESS) {
    Serial.print("Coil: ");
    Serial.println(pdu.getBit(0));
  }
});
----

===== readCoils

[source,cpp]
----
void readCoils(Slaves& slaves, uint16_t address, uint16_t count, callback& cb);
void readCoils(uint8_t slave, uint16_t address, uint16_t count, callback& cb);
----

*Description*: Reads multiple coils starting at the specified address.

*Parameters*:
- `slaves`, `slave`: Slave IDs or single ID (1–247).
- `address`: Starting coil address (0–65535).
- `count`: Number of coils to read (1–2000).
- `cb`: Response callback.

*Notes*:
- Returns coil values in `pdu.getBit(index)`.

===== readCoilsByBytes

[source,cpp]
----
void readCoilsByBytes(Slaves& slaves, uint16_t address, uint8_t byteCount, callback& cb);
void readCoilsByBytes(uint8_t slave, uint16_t address, uint8_t byteCount, callback& cb);
----

*Description*: Reads coils as bytes (8 coils per byte).

*Parameters*:
- `slaves`, `slave`: Slave IDs or single ID (1–247).
- `address`: Starting coil address (0–65535).
- `byteCount`: Number of bytes to read (1–250).
- `cb`: Response callback.

*Notes*:
- Returns bytes in `pdu.getByte(index)`.

===== readDiscreteInput, readDiscreteInputs, readDiscreteInputsByBytes

Similar to `readCoil`, `readCoils`, and `readCoilsByBytes`, but for discrete inputs. Broadcast not supported.

===== writeSingleHoldingRegister

[source,cpp]
----
void writeSingleHoldingRegister(Slaves& slaves, uint16_t address, uint16_t value, callback& cb);
void writeSingleHoldingRegister(uint8_t slave, uint16_t address, uint16_t value, callback& cb);
----

*Description*: Writes a single holding register.

*Parameters*:
- `slaves`, `slave`: Slave IDs or single ID (0 for RTU broadcast).
- `address`: Register address (0–65535).
- `value`: 16-bit value to write.
- `cb`: Response callback.

*Notes*:
- Broadcast supported for RTU.

===== writeHoldingRegister (Template)

[source,cpp]
----
template<typename T>
void writeHoldingRegister(Slaves& slaves, uint16_t address, T& value, callback& cb);
template<typename T>
void writeHoldingRegister(uint8_t slave, uint16_t address, T& value, callback& cb);
----

*Description*: Writes a single holding register with a typed value.

*Parameters*:
- `slaves`, `slave`: Slave IDs or single ID (0 for RTU broadcast).
- `address`: Register address (0–65535).
- `value`: Value of type `T` (e.g., `uint16_t`, `float`).
- `cb`: Response callback.

*Notes*:
- Type-safe, supports various data types.

===== writeHoldingRegisters (Template)

[source,cpp]
----
template<typename T>
void writeHoldingRegisters(Slaves& slaves, uint16_t address, const T* values, uint16_t count, callback& cb);
template<typename T>
void writeHoldingRegisters(uint8_t slave, uint16_t address, const T* values, uint16_t count, callback& cb);
template<typename T>
void writeHoldingRegisters(Slaves& slaves, uint16_t address, std::initializer_list<T> values, callback& cb);
template<typename T>
void writeHoldingRegisters(uint8_t slave, uint16_t address, std::initializer_list<T> values, callback& cb);
----

*Description*: Writes multiple holding registers.

*Parameters*:
- `slaves`, `slave`: Slave IDs or single ID (0 for RTU broadcast).
- `address`: Starting register address (0–65535).
- `values`: Array or initializer list of values.
- `count`: Number of values to write.
- `cb`: Response callback.

===== readHoldingRegister, readHoldingRegisters (Template)

Similar to `readCoil` and `readCoils`, but for holding registers. Default type: `uint16_t`. Broadcast not supported.

===== readInputRegister, readInputRegisters (Template)

Similar to `readHoldingRegister` and `readHoldingRegisters`, but for input registers.

===== readExceptionStatus

[source,cpp]
----
void readExceptionStatus(Slaves& slaves, callback& cb);
void readExceptionStatus(uint8_t slave, callback& cb);
----

*Description*: Reads the exception status (serial line only).

*Parameters*:
- `slaves`, `slave`: Slave IDs or single ID (1–247).
- `cb`: Response callback.

*Notes*:
- Broadcast not supported.

===== maskWriteRegister

[source,cpp]
----
void maskWriteRegister(Slaves& slaves, uint16_t address, uint16_t andMask, uint16_t orMask, callback& cb);
void maskWriteRegister(uint8_t slave, uint16_t address, uint16_t andMask, uint16_t orMask, callback& cb);
----

*Description*: Performs a mask write operation on a holding register.

*Parameters*:
- `slaves`, `slave`: Slave IDs or single ID (0 for RTU broadcast).
- `address`: Register address (0–65535).
- `andMask`, `orMask`: 16-bit masks for the operation.
- `cb`: Response callback.

*Notes*:
- Broadcast supported for RTU.

===== diagnostic

[source,cpp]
----
void diagnostic(Slaves& slaves, uint16_t subFunction, uint16_t data, callback& cb);
void diagnostic(uint8_t slave, uint16_t subFunction, uint16_t data, callback& cb);
----

*Description*: Performs a diagnostic operation.

*Parameters*:
- `slaves`, `slave`: Slave IDs or single ID (1–247).
- `subFunction`: Diagnostic sub-function code.
- `data`: 16-bit data for the diagnostic.
- `cb`: Response callback.

*Notes*:
- Broadcast not supported.

===== readWriteMultipleRegisters (Template)

[source,cpp]
----
template<typename READ_T, typename WRITE_T>
void readWriteMultipleRegisters(Slaves& slaves, uint16_t readAddress, uint8_t readCount,
                                uint16_t writeAddress, const WRITE_T* writeData, uint16_t writeCount, callback& cb);
template<typename READ_T, typename WRITE_T>
void readWriteMultipleRegisters(uint8_t slave, uint16_t readAddress, uint8_t readCount,
                                uint16_t writeAddress, const WRITE_T* writeData, uint16_t writeCount, callback& cb);
----

*Description*: Reads and writes multiple registers in one transaction.

*Parameters*:
- `slaves`, `slave`: Slave IDs or single ID (1–247).
- `readAddress`: Starting address for reading.
- `readCount`: Number of registers to read.
- `writeAddress`: Starting address for writing.
- `writeData`: Array of values to write.
- `writeCount`: Number of registers to write.
- `cb`: Response callback.

===== loop

[source,cpp]
----
virtual void loop() = 0;
----

*Description*: Main loop for communication timing and response handling.

*Notes*: Must be called in the sketch’s `loop()` function.

=== ModbusRTUMaster

Implements Modbus RTU over serial interfaces (RS485, RS422, RS232).

==== Methods

===== begin

[source,cpp]
----
void begin(uint8_t PDUSize, uint8_t queueSize, Stream* stream, uint32_t baud, UartConfig cfg, int16_t re = -1, int16_t de = -1);
----

*Description*: Initializes the RTU master.

*Parameters*:
- `PDUSize`: Maximum PDU size (e.g., 253).
- `queueSize`: Size of the ADU queue.
- `stream`: Serial stream (e.g., `&Serial1`).
- `baud`: Baud rate (e.g., 115200).
- `cfg`: UART configuration (e.g., `UartConfig::Mode_8N1`).
- `re`, `de`: RS485 receive/transmit enable pins (-1 if not used).

*Example*:
[source,cpp]
----
master.begin(253, 5, &Serial1, 115200, UartConfig::Mode_8N1);
----

===== getFrameTimeout, setFrameTimeout

[source,cpp]
----
uint32_t getFrameTimeout();
void setFrameTimeout(uint32_t t);
----

*Description*: Gets or sets the frame timeout (µs) for RTU communication.

===== getResponseTimeout, setResponseTimeout

[source,cpp]
----
uint32_t getResponseTimeout();
void setResponseTimeout(uint32_t t);
----

*Description*: Gets or sets the response timeout (µs).

===== getByteTimeout, setByteTimeout

[source,cpp]
----
uint32_t getByteTimeout();
void setByteTimeout(uint32_t t);
----

*Description*: Gets or sets the byte timeout (µs).

=== ModbusTCPClient

Implements Modbus TCP over Ethernet.

==== Methods

===== begin

[source,cpp]
----
void begin(uint8_t ADUPoolSize, uint8_t PDUSize, uint8_t clientCount);
----

*Description*: Initializes the TCP client.

*Parameters*:
- `ADUPoolSize`: Size of the ADU pool.
- `PDUSize`: Maximum PDU size.
- `clientCount`: Number of simultaneous clients.

===== addClient

[source,cpp]
----
bool addClient(uint8_t id, bool allAtOnce, uint8_t queueSize, Client* client, IPAddress ip, uint16_t port, bool keepAlive);
----

*Description*: Adds a client for a slave.

*Parameters*:
- `id`: Client ID.
- `allAtOnce`: Send all queued requests simultaneously.
- `queueSize`: Request queue size.
- `client`: Ethernet client object.
- `ip`: Slave IP address.
- `port`: Slave port (e.g., 502).
- `keepAlive`: Keep TCP connection alive.

*Returns*: `true` if added successfully.

=== Slaves

Manages sets of Modbus slave IDs (1–247) or broadcast (ID = 0).

==== Constructors

[source,cpp]
----
Slaves();
Slaves(uint8_t slave);
Slaves(uint8_t slave, int32_t repeatDelay);
Slaves(std::initializer_list<uint8_t> list);
Slaves(std::initializer_list<uint8_t> list, int32_t delay);
Slaves(std::initializer_list<uint8_t> list, int32_t delay, int32_t repeatDelay);
----

*Description*: Initializes a slave set.

*Parameters*:
- `slave`: Single slave ID (0 for broadcast, 1–247 for slaves).
- `repeatDelay`: Delay between iteration cycles (ms, -1 disables).
- `list`: List of slave IDs (e.g., `{1,2,3}`).
- `delay`: Delay between individual slave processing (ms).

*Notes*:
- Broadcast mode (ID = 0) is exclusive; other IDs are ignored.

==== Methods

===== set

[source,cpp]
----
void set(uint8_t slaveId);
void set(uint8_t begin, uint8_t end);
void set(std::initializer_list<uint8_t> list);
----

*Description*: Adds slave IDs to the set.

*Parameters*:
- `slaveId`: Single slave ID (1–247).
- `begin`, `end`: Range of slave IDs (inclusive).
- `list`: List of slave IDs.

*Notes*:
- Ignored in broadcast mode.

===== remove

[source,cpp]
----
void remove(uint8_t slaveId);
----

*Description*: Removes a slave ID from the set.

===== isSet

[source,cpp]
----
bool isSet(uint8_t slaveId) const;
----

*Description*: Checks if a slave ID is active.

*Returns*: `true` if the ID is set.

===== clear

[source,cpp]
----
void clear();
----

*Description*: Clears all slave IDs and resets settings.

===== getRepeat

[source,cpp]
----
bool getRepeat() const;
----

*Description*: Checks if cyclic iteration is enabled.

*Returns*: `true` if `repeatDelay >= 0`.

===== setRepeatDelay, getRepeatDelay

[source,cpp]
----
void setRepeatDelay(int32_t repeatDelay);
int32_t getRepeatDelay();
----

*Description*: Sets or gets the delay between iteration cycles (ms).

===== setDelay, getDelay

[source,cpp]
----
void setDelay(int32_t delay);
int32_t getDelay() const;
----

*Description*: Sets or gets the delay between individual slave processing (ms).

*Notes*:
- Ignored in broadcast mode.

===== getNext, peek

[source,cpp]
----
uint8_t getNext();
uint8_t peek();
----

*Description*: Gets the next active slave ID (`getNext` updates `_active`, `peek` does not).

*Returns*: Slave ID or `0xFF` if no more slaves.

===== getActive

[source,cpp]
----
uint8_t getActive() const;
----

*Description*: Returns the current active slave ID.

===== resetActive

[source,cpp]
----
void resetActive();
----

*Description*: Resets iteration to the first active slave ID.

===== hasMore

[source,cpp]
----
bool hasMore() const;
----

*Description*: Checks if there are more active slave IDs to iterate.

*Returns*: `true` if more IDs are available.

== Types

- **callback**:
  ```cpp
  using callback = Callback<void, PDU&>;


Defined in ModbusCallbackTypes.h, used for asynchronous response handling.
UartConfig:
    Enumeration for UART configurations (e.g., Mode_8N1, Mode_8E1).
== Constants

Defined in ModbusDef.h:

Error Codes:
    MB_EX_SUCCESS (0): Operation successful.
    MB_EX_LIB_INVALID_SLAVE: Invalid slave ID (e.g., 0 for non-broadcast operations).
    MB_EX_LIB_NO_MORE_FREE_ADU: No free ADUs available.
    MB_EX_LIB_QUEUE_FULL: Request queue full.
    MB_EX_LIB_RESPONSE_TIMEOUT: Response timeout.
    Function Codes:
    MB_FC_READ_COILS (0x01)
    MB_FC_READ_DISCRETE_INPUTS (0x02)
    MB_FC_READ_HOLDING_REGISTERS (0x03)
    MB_FC_READ_INPUT_REGISTERS (0x04)
    MB_FC_WRITE_SINGLE_COIL (0x05)
    MB_FC_WRITE_SINGLE_REGISTER (0x06)
    MB_FC_WRITE_MULTIPLE_COILS (0x0F)
    MB_FC_WRITE_MULTIPLE_REGISTERS (0x10)
    MB_FC_MASK_WRITE_REGISTER (0x16)
    MB_FC_READ_WRITE_MULTIPLE_REGISTERS (0x17)
Timeouts:
    MB_RESPONSE_TIMEOUT: Default RTU response timeout.
    MB_TCP_RESPONSE_TIMEOUT: Default TCP response timeout.
    MB_RECONNECT: Reconnection timeout.

== Dependencies

Callback: Custom callback library for asynchronous operations (https://github.com/dynapptor/Callback[GitHub, window="_blank"]).
Initializer List: Lightweight C++11 implementation for AVR, included in lib/initializer_list.

