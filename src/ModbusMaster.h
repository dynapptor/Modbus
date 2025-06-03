/**
 * @file ModbusMaster.h
 * @brief Abstract base class for Modbus master communication.
 * @details Manages coil, register, and diagnostic operations for multiple slaves or single slave, with asynchronous callback support.
 */

#pragma once
#include <Arduino.h>

#include <initializer_list>

#include "ModbusCallbackTypes.h"
#include "Slaves.h"

class PDU;

/**
 * @class ModbusMaster
 * @brief Abstract base class for Modbus master communication.
 * @details Provides methods for reading/writing coils, registers, and diagnostics, supporting multiple slaves and broadcast (RTU only).
 */
class ModbusMaster {
 private:
  /**
   * @brief Checks if the function code is a write operation supporting broadcast.
   * @param functionCode Modbus function code.
   * @return bool True if the function code supports broadcast, false otherwise.
   */
  bool isWriteFunction(uint8_t functionCode) const;

 protected:
  /**
   * @brief Retrieves a free PDU instance for the operation.
   * @param cb Callback function for response handling.
   * @param slaves Set of slave IDs for the operation.
   * @return PDU* Pointer to the free PDU, or nullptr if none available.
   * @note Must be overridden by derived classes.
   */
  virtual PDU* getFreePDU(const modbusCallback& cb, const Slaves& slaves) = 0;

  /**
   * @brief Retrieves a free PDU instance for a single slave operation.
   * @param cb Callback function for response handling.
   * @param slave Slave ID (1-247, or 0 for broadcast).
   * @return PDU* Pointer to the free PDU, or nullptr if none available.
   * @note Must be overridden by derived classes.
   */
  virtual PDU* getFreePDU(const modbusCallback& cb, uint8_t slave) = 0;

  /**
   * @brief Sends the PDU to the specified slave.
   * @param pdu Pointer to the PDU to send.
   * @param slave Slave ID (1-247, or 0 for broadcast).
   * @return bool True if sent successfully, false otherwise.
   * @note Must be overridden by derived classes.
   */
  virtual bool sendPDU(PDU* pdu, uint8_t slave) = 0;

 public:
  /**
   * @brief Default constructor.
   * @details Initializes an empty ModbusMaster.
   */
  ModbusMaster();

  /**
   * @brief Virtual destructor.
   * @details Ensures proper cleanup in derived classes.
   */
  virtual ~ModbusMaster();

  /**
   * @brief Writes a single coil to the specified address for multiple slaves.
   * @param slaves Set of slave IDs.
   * @param address Coil address (0-65535).
   * @param value Coil value (true/false).
   * @param cb Callback function for response handling.
   */
  void writeSingleCoil(const Slaves& slaves, uint16_t address, bool value, const modbusCallback& cb);

  /**
   * @brief Writes a single coil to the specified address for a single slave or broadcast.
   * @param slave Slave ID (1-247, or 0 for broadcast, RTU only).
   * @param address Coil address (0-65535).
   * @param value Coil value (true/false).
   * @param cb Callback function for response handling.
   */
  void writeSingleCoil(uint8_t slave, uint16_t address, bool value, const modbusCallback& cb);

  /**
   * @brief Writes multiple coils from a byte array for multiple slaves.
   * @param slaves Set of slave IDs.
   * @param address Starting coil address (0-65535).
   * @param src Source byte array containing coil values.
   * @param byteCount Number of bytes in the source array.
   * @param coilCount Number of coils to write.
   * @param cb Callback function for response handling.
   */
  void writeCoils(const Slaves& slaves, uint16_t address, const uint8_t* src, uint8_t byteCount, uint16_t coilCount, const modbusCallback& cb);

  /**
   * @brief Writes multiple coils from a byte array for a single slave or broadcast.
   * @param slave Slave ID (1-247, or 0 for broadcast, RTU only).
   * @param address Starting coil address (0-65535).
   * @param src Source byte array containing coil values.
   * @param byteCount Number of bytes in the source array.
   * @param coilCount Number of coils to write.
   * @param cb Callback function for response handling.
   */
  void writeCoils(uint8_t slave, uint16_t address, const uint8_t* src, uint8_t byteCount, uint16_t coilCount, const modbusCallback& cb);

  /**
   * @brief Writes multiple coils from a boolean array for multiple slaves.
   * @param slaves Set of slave IDs.
   * @param address Starting coil address (0-65535).
   * @param values Source boolean array containing coil values.
   * @param count Number of coils to write.
   * @param cb Callback function for response handling.
   */
  void writeCoils(const Slaves& slaves, uint16_t address, const bool* values, uint16_t count, const modbusCallback& cb);

  /**
   * @brief Writes multiple coils from a boolean array for a single slave or broadcast.
   * @param slave Slave ID (1-247, or 0 for broadcast, RTU only).
   * @param address Starting coil address (0-65535).
   * @param values Source boolean array containing coil values.
   * @param count Number of coils to write.
   * @param cb Callback function for response handling.
   */
  void writeCoils(uint8_t slave, uint16_t address, const bool* values, uint16_t count, const modbusCallback& cb);

  /**
   * @brief Writes multiple coils from an initializer list for multiple slaves.
   * @param slaves Set of slave IDs.
   * @param address Starting coil address (0-65535).
   * @param values Initializer list of boolean coil values.
   * @param cb Callback function for response handling.
   */
  void writeCoils(const Slaves& slaves, uint16_t address, std::initializer_list<bool> values, const modbusCallback& cb);

  /**
   * @brief Writes multiple coils from an initializer list for a single slave or broadcast.
   * @param slave Slave ID (1-247, or 0 for broadcast, RTU only).
   * @param address Starting coil address (0-65535).
   * @param values Initializer list of boolean coil values.
   * @param cb Callback function for response handling.
   */
  void writeCoils(uint8_t slave, uint16_t address, std::initializer_list<bool> values, const modbusCallback& cb);

  /**
   * @brief Reads coils as bytes for multiple slaves.
   * @param slaves Set of slave IDs.
   * @param address Starting coil address (0-65535).
   * @param byteCount Number of bytes to read (8 coils per byte).
   * @param cb Callback function for response handling.
   */
  void readCoilsByBytes(const Slaves& slaves, uint16_t address, uint8_t byteCount, const modbusCallback& cb);

  /**
   * @brief Reads coils as bytes for a single slave.
   * @param slave Slave ID (1-247).
   * @param address Starting coil address (0-65535).
   * @param byteCount Number of bytes to read (8 coils per byte).
   * @param cb Callback function for response handling.
   */
  void readCoilsByBytes(uint8_t slave, uint16_t address, uint8_t byteCount, const modbusCallback& cb);

  /**
   * @brief Reads a single coil for multiple slaves.
   * @param slaves Set of slave IDs.
   * @param address Coil address (0-65535).
   * @param cb Callback function for response handling.
   */
  void readCoil(const Slaves& slaves, uint16_t address, const modbusCallback& cb);

  /**
   * @brief Reads a single coil for a single slave.
   * @param slave Slave ID (1-247).
   * @param address Coil address (0-65535).
   * @param cb Callback function for response handling.
   */
  void readCoil(uint8_t slave, uint16_t address, const modbusCallback& cb);

  /**
   * @brief Reads multiple coils for multiple slaves.
   * @param slaves Set of slave IDs.
   * @param address Starting coil address (0-65535).
   * @param count Number of coils to read.
   * @param cb Callback function for response handling.
   */
  void readCoils(const Slaves& slaves, uint16_t address, uint16_t count, const modbusCallback& cb);

  /**
   * @brief Reads multiple coils for a single slave.
   * @param slave Slave ID (1-247).
   * @param address Starting coil address (0-65535).
   * @param count Number of coils to read.
   * @param cb Callback function for response handling.
   */
  void readCoils(uint8_t slave, uint16_t address, uint16_t count, const modbusCallback& cb);

  /**
   * @brief Reads a single discrete input for multiple slaves.
   * @param slaves Set of slave IDs.
   * @param address Discrete input address (0-65535).
   * @param cb Callback function for response handling.
   */
  void readDiscreteInput(const Slaves& slaves, uint16_t address, const modbusCallback& cb);

  /**
   * @brief Reads a single discrete input for a single slave.
   * @param slave Slave ID (1-247).
   * @param address Discrete input address (0-65535).
   * @param cb Callback function for response handling.
   */
  void readDiscreteInput(uint8_t slave, uint16_t address, const modbusCallback& cb);

  /**
   * @brief Reads discrete inputs as bytes for multiple slaves.
   * @param slaves Set of slave IDs.
   * @param address Starting discrete input address (0-65535).
   * @param byteCount Number of bytes to read (8 inputs per byte).
   * @param cb Callback function for response handling.
   */
  void readDiscreteInputsByBytes(const Slaves& slaves, uint16_t address, uint8_t byteCount, const modbusCallback& cb);

  /**
   * @brief Reads discrete inputs as bytes for a single slave.
   * @param slave Slave ID (1-247).
   * @param address Starting discrete input address (0-65535).
   * @param byteCount Number of bytes to read (8 inputs per byte).
   * @param cb Callback function for response handling.
   */
  void readDiscreteInputsByBytes(uint8_t slave, uint16_t address, uint8_t byteCount, const modbusCallback& cb);

  /**
   * @brief Reads multiple discrete inputs for multiple slaves.
   * @param slaves Set of slave IDs.
   * @param address Starting discrete input address (0-65535).
   * @param count Number of discrete inputs to read.
   * @param cb Callback function for response handling.
   */
  void readDiscreteInputs(const Slaves& slaves, uint16_t address, uint16_t count, const modbusCallback& cb);

  /**
   * @brief Reads multiple discrete inputs for a single slave.
   * @param slave Slave ID (1-247).
   * @param address Starting discrete input address (0-65535).
   * @param count Number of discrete inputs to read.
   * @param cb Callback function for response handling.
   */
  void readDiscreteInputs(uint8_t slave, uint16_t address, uint16_t count, const modbusCallback& cb);

  /**
   * @brief Writes a single holding register for multiple slaves.
   * @param slaves Set of slave IDs.
   * @param address Register address (0-65535).
   * @param value Register value (16-bit).
   * @param cb Callback function for response handling.
   */
  void writeSingleHoldingRegister(const Slaves& slaves, uint16_t address, uint16_t value, const modbusCallback& cb);

  /**
   * @brief Writes a single holding register for a single slave or broadcast.
   * @param slave Slave ID (1-247, or 0 for broadcast, RTU only).
   * @param address Register address (0-65535).
   * @param value Register value (16-bit).
   * @param cb Callback function for response handling.
   */
  void writeSingleHoldingRegister(uint8_t slave, uint16_t address, uint16_t value, const modbusCallback& cb);

  /**
   * @brief Reads the exception status for multiple slaves.
   * @param slaves Set of slave IDs.
   * @param cb Callback function for response handling.
   * @note Serial line only (Function Code 0x07).
   */
  void readExceptionStatus(const Slaves& slaves, const modbusCallback& cb);

  /**
   * @brief Reads the exception status for a single slave.
   * @param slave Slave ID (1-247).
   * @param cb Callback function for response handling.
   * @note Serial line only (Function Code 0x07).
   */
  void readExceptionStatus(uint8_t slave, const modbusCallback& cb);

  /**
   * @brief Performs a mask write operation on a holding register for multiple slaves.
   * @param slaves Set of slave IDs.
   * @param address Register address (0-65535).
   * @param andMask AND mask for the register (16-bit).
   * @param orMask OR mask for the register (16-bit).
   * @param cb Callback function for response handling.
   */
  void maskWriteRegister(const Slaves& slaves, uint16_t address, uint16_t andMask, uint16_t orMask, const modbusCallback& cb);

  /**
   * @brief Performs a mask write operation for a single slave or broadcast.
   * @param slave Slave ID (1-247, or 0 for broadcast, RTU only).
   * @param address Register address (0-65535).
   * @param andMask AND mask for the register (16-bit).
   * @param orMask OR mask for the register (16-bit).
   * @param cb Callback function for response handling.
   */
  void maskWriteRegister(uint8_t slave, uint16_t address, uint16_t andMask, uint16_t orMask, const modbusCallback& cb);

  /**
   * @brief Performs a diagnostic operation for multiple slaves.
   * @param slaves Set of slave IDs.
   * @param subFunction Diagnostic subfunction code.
   * @param data Data value for the diagnostic request.
   * @param cb Callback function for response handling.
   */
  void diagnostic(const Slaves& slaves, uint16_t subFunction, uint16_t data, const modbusCallback& cb);

  /**
   * @brief Performs a diagnostic operation for a single slave.
   * @param slave Slave ID (1-247).
   * @param subFunction Diagnostic subfunction code.
   * @param data Data value for the diagnostic request.
   * @param cb Callback function for response handling.
   */
  void diagnostic(uint8_t slave, uint16_t subFunction, uint16_t data, const modbusCallback& cb);

  /**
   * @brief Reads and writes multiple registers for multiple slaves.
   * @tparam READ_T Type of the read register data.
   * @tparam WRITE_T Type of the write register data.
   * @param slaves Set of slave IDs.
   * @param readAddress Starting address for reading (0-65535).
   * @param readCount Number of registers to read.
   * @param writeAddress Starting address for writing (0-65535).
   * @param writeData Source array containing write data.
   * @param writeCount Number of registers to write.
   * @param cb Callback function for response handling.
   */
  template <typename READ_T, typename WRITE_T>
  void readWriteMultipleRegisters(const Slaves& slaves, uint16_t readAddress, uint8_t readCount,
                                  uint16_t writeAddress, const WRITE_T* writeData, uint16_t writeCount, const modbusCallback& cb);

  /**
   * @brief Reads and writes multiple registers for a single slave.
   * @tparam READ_T Type of the read register data.
   * @tparam WRITE_T Type of the write register data.
   * @param slave Slave ID (1-247).
   * @param readAddress Starting address for reading (0-65535).
   * @param readCount Number of registers to read.
   * @param writeAddress Starting address for writing (0-65535).
   * @param writeData Source array containing write data.
   * @param writeCount Number of registers to write.
   * @param cb Callback function for response handling.
   */
  template <typename READ_T, typename WRITE_T>
  void readWriteMultipleRegisters(uint8_t slave, uint16_t readAddress, uint8_t readCount,
                                  uint16_t writeAddress, const WRITE_T* writeData, uint16_t writeCount, const modbusCallback& cb);

  /**
   * @brief Writes a single holding register with a typed value for multiple slaves.
   * @tparam T Type of the register value.
   * @param slaves Set of slave IDs.
   * @param address Register address (0-65535).
   * @param value Register value.
   * @param cb Callback function for response handling.
   */
  template <typename T>
  void writeHoldingRegister(const Slaves& slaves, uint16_t address, T& value, const modbusCallback& cb);

  /**
   * @brief Writes a single holding register with a typed value for a single slave or broadcast.
   * @tparam T Type of the register value.
   * @param slave Slave ID (1-247, or 0 for broadcast, RTU only).
   * @param address Register address (0-65535).
   * @param value Register value.
   * @param cb Callback function for response handling.
   */
  template <typename T>
  void writeHoldingRegister(uint8_t slave, uint16_t address, T& value, const modbusCallback& cb);

  /**
   * @brief Writes multiple holding registers from a typed array for multiple slaves.
   * @tparam T Type of the register values.
   * @param slaves Set of slave IDs.
   * @param address Starting register address (0-65535).
   * @param values Source array containing register values.
   * @param count Number of registers to write.
   * @param cb Callback function for response handling.
   */
  template <typename T>
  void writeHoldingRegisters(const Slaves& slaves, uint16_t address, const T* values, uint16_t count, const modbusCallback& cb);

  /**
   * @brief Writes multiple holding registers from a typed array for a single slave or broadcast.
   * @tparam T Type of the register values.
   * @param slave Slave ID (1-247, or 0 for broadcast, RTU only).
   * @param address Starting register address (0-65535).
   * @param values Source array containing register values.
   * @param count Number of registers to write.
   * @param cb Callback function for response handling.
   */
  template <typename T>
  void writeHoldingRegisters(uint8_t slave, uint16_t address, const T* values, uint16_t count, const modbusCallback& cb);

  /**
   * @brief Writes multiple holding registers from an initializer list for multiple slaves.
   * @tparam T Type of the register values.
   * @param slaves Set of slave IDs.
   * @param address Starting register address (0-65535).
   * @param values Initializer list of register values.
   * @param cb Callback function for response handling.
   */
  template <typename T>
  void writeHoldingRegisters(const Slaves& slaves, uint16_t address, std::initializer_list<T> values, const modbusCallback& cb);

  /**
   * @brief Writes multiple holding registers from an initializer list for a single slave or broadcast.
   * @tparam T Type of the register values.
   * @param slave Slave ID (1-247, or 0 for broadcast, RTU only).
   * @param address Starting register address (0-65535).
   * @param values Initializer list of register values.
   * @param cb Callback function for response handling.
   */
  template <typename T>
  void writeHoldingRegisters(uint8_t slave, uint16_t address, std::initializer_list<T> values, const modbusCallback& cb);

  /**
   * @brief Reads a single holding register for multiple slaves.
   * @tparam T Type of the register data (default: uint16_t).
   * @param slaves Set of slave IDs.
   * @param address Register address (0-65535).
   * @param cb Callback function for response handling.
   */
  template <typename T = uint16_t>
  void readHoldingRegister(const Slaves& slaves, uint16_t address, const modbusCallback& cb);

  /**
   * @brief Reads a single holding register for a single slave.
   * @tparam T Type of the register data (default: uint16_t).
   * @param slave Slave ID (1-247).
   * @param address Register address (0-65535).
   * @param cb Callback function for response handling.
   */
  template <typename T = uint16_t>
  void readHoldingRegister(uint8_t slave, uint16_t address, const modbusCallback& cb);

  /**
   * @brief Reads multiple holding registers for multiple slaves.
   * @tparam T Type of the register data (default: uint16_t).
   * @param slaves Set of slave IDs.
   * @param address Starting register address (0-65535).
   * @param count Number of registers to read.
   * @param cb Callback function for response handling.
   */
  template <typename T = uint16_t>
  void readHoldingRegisters(const Slaves& slaves, uint16_t address, uint8_t count, const modbusCallback& cb);

  /**
   * @brief Reads multiple holding registers for a single slave.
   * @tparam T Type of the register data (default: uint16_t).
   * @param slave Slave ID (1-247).
   * @param address Starting register address (0-65535).
   * @param count Number of registers to read.
   * @param cb Callback function for response handling.
   */
  template <typename T = uint16_t>
  void readHoldingRegisters(uint8_t slave, uint16_t address, uint8_t count, const modbusCallback& cb);

  /**
   * @brief Reads a single input register for multiple slaves.
   * @tparam T Type of the register data (default: uint16_t).
   * @param slaves Set of slave IDs.
   * @param address Register address (0-65535).
   * @param cb Callback function for response handling.
   */
  template <typename T = uint16_t>
  void readInputRegister(const Slaves& slaves, uint16_t address, const modbusCallback& cb);

  /**
   * @brief Reads a single input register for a single slave.
   * @tparam T Type of the register data (default: uint16_t).
   * @param slave Slave ID (1-247).
   * @param address Register address (0-65535).
   * @param cb Callback function for response handling.
   */
  template <typename T = uint16_t>
  void readInputRegister(uint8_t slave, uint16_t address, const modbusCallback& cb);

  /**
   * @brief Reads multiple input registers for multiple slaves.
   * @tparam T Type of the register data (default: uint16_t).
   * @param slaves Set of slave IDs.
   * @param address Starting register address (0-65535).
   * @param count Number of registers to read.
   * @param cb Callback function for response handling.
   */
  template <typename T = uint16_t>
  void readInputRegisters(const Slaves& slaves, uint16_t address, uint8_t count, const modbusCallback& cb);

  /**
   * @brief Reads multiple input registers for a single slave.
   * @tparam T Type of the register data (default: uint16_t).
   * @param slave Slave ID (1-247).
   * @param address Starting register address (0-65535).
   * @param count Number of registers to read.
   * @param cb Callback function for response handling.
   */
  template <typename T = uint16_t>
  void readInputRegisters(uint8_t slave, uint16_t address, uint8_t count, const modbusCallback& cb);

  /**
   * @brief Main loop for communication timing and response handling.
   * @note Must be overridden by derived classes.
   */
  virtual void loop() = 0;
};

#include "ModbusMaster.tpp"