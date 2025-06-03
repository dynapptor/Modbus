/**
 * @file PDU.h
 * @brief Manages Modbus Protocol Data Unit (PDU) for RTU/TCP transactions.
 * @details Handles function codes, data, callbacks, endian conversion, and error handling for Modbus operations.
 */

#pragma once
#include <Arduino.h>
#include <Callback.h>

#include "ModbusCallbackTypes.h"

template <typename T>
class ADUQueue;

/**
 * @class PDU
 * @brief Manages Modbus PDU for RTU/TCP transactions.
 * @details Supports function code processing, data handling, endian conversion, and asynchronous callbacks for Modbus communication.
 */
class PDU {
  friend class ModbusMaster;     ///< Access to private buffers for transaction handling.
  friend class ModbusRTUMaster;  ///< Access to private buffers for RTU-specific operations.
  friend class ModbusTCPClient;  ///< Access to private buffers for TCP-specific operations.
  template <typename T>          ///< Access to private buffers for queue management.
  friend class ADUQueue;

 protected:
  modbusCallback _callback;             ///< Callback function for response handling.
  uint8_t* _TXPDUbuffer = nullptr;      ///< Transmit buffer for PDU data.
  uint8_t* _RXPDUbuffer = nullptr;      ///< Receive buffer for PDU data.
  uint8_t _TXPDUbufferLen = 0;          ///< Length of transmit buffer data.
  uint8_t* _PDUresponseHead = nullptr;  ///< Expected response header for validation.
  uint8_t _dataBegin = 0;               ///< Start index of data in RX buffer.
  uint8_t _dataLen = 0;                 ///< Length of data in RX buffer.
  uint16_t _err = 0;                    ///< Error code (MB_EX_* from ModbusDef.h).
  uint8_t _expectedResponseLen = 0;     ///< Expected response length.
  uint8_t _elemSize = 0;                ///< Element size for register data (used in endian conversion).
  boolean _used = false;                ///< Indicates if PDU is in use.
  uint8_t _PDUSize = 0;                 ///< Max PDU size, set by ADUTCP/ADURTU (user-defined, up to 253 bytes).
  uint32_t _queuedTime = 0;             ///< Time when PDU was queued (ms).
  uint32_t _delayToSend = 0;            ///< Delay before sending (ms).
  uint8_t _slave = 0;                   ///< Slave ID for error response.

  /**
   * @brief Processes the received PDU and calls callback.
   * @details Validates the response and invokes the callback if valid.
   * @return uint16_t Error code (MB_EX_*) or 0 if successful.
   */
  uint16_t invoke();

  /**
   * @brief Resets PDU state and clears buffers.
   * @details Clears all buffers and resets internal state.
   */
  virtual void clear();

  /**
   * @brief Executes the callback function if valid.
   * @details Calls the registered callback with the PDU reference.
   */
  void callCallback();

  /**
   * @brief Creates PDU for writing a single coil (Function Code 0x05).
   * @param addr Coil address (0-65535).
   * @param value Coil value (true/false).
   * @param cb Callback function for response handling.
   * @return uint16_t Error code (MB_EX_*) or 0 if successful.
   */
  uint16_t createWriteSingleCoil(uint16_t addr, bool value, const modbusCallback& cb);

  /**
   * @brief Creates PDU for writing a single register (Function Code 0x06).
   * @param addr Register address (0-65535).
   * @param value Register value (16-bit).
   * @param cb Callback function for response handling.
   * @return uint16_t Error code (MB_EX_*) or 0 if successful.
   */
  uint16_t createWriteSingleRegister(uint16_t addr, uint16_t value, const modbusCallback& cb);

  /**
   * @brief Creates PDU for writing multiple coils from byte array (Function Code 0x0F).
   * @param addr Starting coil address (0-65535).
   * @param src Source byte array containing coil values.
   * @param byteCount Number of bytes in the source array.
   * @param coilCount Number of coils to write.
   * @param cb Callback function for response handling.
   * @return uint16_t Error code (MB_EX_*) or 0 if successful.
   */
  uint16_t createWriteMultipleCoils(uint16_t addr, const uint8_t* src, uint8_t byteCount, uint16_t coilCount, const modbusCallback& cb);

  /**
   * @brief Creates PDU for writing multiple coils from bool array (Function Code 0x0F).
   * @param addr Starting coil address (0-65535).
   * @param src Source bool array containing coil values.
   * @param coilCount Number of coils to write.
   * @param cb Callback function for response handling.
   * @return uint16_t Error code (MB_EX_*) or 0 if successful.
   */
  uint16_t createWriteMultipleCoils(uint16_t addr, const bool* src, uint16_t coilCount, const modbusCallback& cb);

  /**
   * @brief Creates PDU for writing multiple holding registers (Function Code 0x10).
   * @tparam T Type of the register data (e.g., uint16_t, int32_t).
   * @param addr Starting register address (0-65535).
   * @param src Source array containing register values.
   * @param count Number of registers to write.
   * @param cb Callback function for response handling.
   * @return uint16_t Error code (MB_EX_*) or 0 if successful.
   */
  template <typename T>
  uint16_t createWriteHoldingRegister(uint16_t addr, const T* src, uint8_t count, const modbusCallback& cb);

  /**
   * @brief Creates PDU for mask write register (Function Code 0x16).
   * @param addr Register address (0-65535).
   * @param andMask AND mask for the register (16-bit).
   * @param orMask OR mask for the register (16-bit).
   * @param cb Callback function for response handling.
   * @return uint16_t Error code (MB_EX_*) or 0 if successful.
   */
  uint16_t createMaskWriteRegister(uint16_t addr, uint16_t andMask, uint16_t orMask, const modbusCallback& cb);

  /**
   * @brief Creates PDU for read/write multiple registers (Function Code 0x17).
   * @tparam READ_T Type of the read register data.
   * @tparam WRITE_T Type of the write register data.
   * @param readAddr Starting address for reading (0-65535).
   * @param readCount Number of registers to read.
   * @param writeAddr Starting address for writing (0-65535).
   * @param writeData Source array containing write data.
   * @param writeCount Number of registers to write.
   * @param cb Callback function for response handling.
   * @return uint16_t Error code (MB_EX_*) or 0 if successful.
   */
  template <typename READ_T, typename WRITE_T>
  uint16_t createReadWriteMultipleRegisters(uint16_t readAddr, uint8_t readCount, uint16_t writeAddr, const WRITE_T* writeData, uint16_t writeCount, const modbusCallback& cb);

  /**
   * @brief Creates PDU for reading exception status (Function Code 0x07).
   * @param cb Callback function for response handling.
   * @return uint16_t Error code (MB_EX_*) or 0 if successful.
   */
  uint16_t createReadExceptionStatus(const modbusCallback& cb);

  /**
   * @brief Creates PDU for diagnostics (Function Code 0x08).
   * @param subFunction Diagnostics sub-function code.
   * @param value Data value for the diagnostics request.
   * @param cb Callback function for response handling.
   * @return uint16_t Error code (MB_EX_*) or 0 if successful.
   */
  uint16_t createDiagnostics(uint16_t subFunction, uint16_t value, const modbusCallback& cb);

  /**
   * @brief Creates PDU for reading coils or inputs (Function Code 0x01 or 0x02).
   * @param fn Function code (0x01 for coils, 0x02 for inputs).
   * @param addr Starting address (0-65535).
   * @param count Number of coils/inputs to read.
   * @param cb Callback function for response handling.
   * @return uint16_t Error code (MB_EX_*) or 0 if successful.
   */
  uint16_t createReadState(uint8_t fn, uint16_t addr, uint16_t count, const modbusCallback& cb);

  /**
   * @brief Creates PDU for reading registers (Function Code 0x03 or 0x04).
   * @tparam T Type of the register data (e.g., uint16_t, int32_t).
   * @param fn Function code (0x03 for holding registers, 0x04 for input registers).
   * @param addr Starting address (0-65535).
   * @param count Number of registers to read.
   * @param cb Callback function for response handling.
   * @return uint16_t Error code (MB_EX_*) or 0 if successful.
   */
  template <typename T>
  uint16_t createReadRegisters(uint8_t fn, uint16_t addr, uint8_t count, const modbusCallback& cb);

  /**
   * @brief Converts byte count to register count.
   * @param byteCount Number of bytes.
   * @return uint8_t Number of registers (1 register = 2 bytes).
   */
  static uint8_t toRegisterCount(uint8_t byteCount);

  /**
   * @brief Converts data to big-endian format with padding.
   * @param src Source data buffer.
   * @param elemCount Number of elements.
   * @param elemSize Size of each element.
   * @param dest Destination buffer.
   * @param destLen Length of the destination buffer.
   * @return bool True if conversion succeeded, false otherwise.
   */
  static bool convertToBigEndianRegisters(const uint8_t* src, uint8_t elemCount, uint8_t elemSize, uint8_t* dest, uint16_t destLen);

  /**
   * @brief Converts 16-bit value to big-endian format.
   * @param src Source value.
   * @return uint16_t Big-endian value.
   */
  static uint16_t toBigEndian(uint16_t src);

  /**
   * @brief Converts received big-endian register data in-place.
   * @param buffer Data buffer to convert.
   * @param elemCount Number of elements.
   * @param elemSize Size of each element.
   * @return bool True if conversion succeeded, false otherwise.
   */
  bool convertFromBigEndianRegistersInPlace(uint8_t* buffer, uint8_t elemCount, uint8_t elemSize);

  /**
   * @brief Checks if PDU is in use.
   * @return bool True if PDU is in use, false otherwise.
   */
  bool isUsed() const;

  /**
   * @brief Sets PDU usage state.
   * @param v True to mark as used, false to mark as unused.
   */
  void setUsed(bool v);

  /**
   * @brief Handles cyclic iteration for slaves.
   * @details Virtual method overridden in ADUTCP/ADURTU for slave iteration.
   * @return bool True if iteration should continue, false otherwise.
   */
  virtual bool repeatIfNeeded();

 public:
  /**
   * @brief Default constructor.
   * @details Initializes an empty PDU.
   */
  PDU();

  /**
   * @brief Constructor for error response with slave ID.
   * @param slave Slave ID for error response.
   */
  PDU(uint8_t slave);

  /**
   * @brief Virtual destructor.
   * @details Memory management handled by derived classes.
   */
  virtual ~PDU();

  /**
   * @brief Returns the current error code.
   * @return uint16_t Error code (MB_EX_*).
   */
  uint16_t getErr() const;

  /**
   * @brief Returns the slave ID.
   * @details Defaults to 0, overridden in ADUTCP/ADURTU.
   * @return uint8_t Slave ID.
   */
  virtual uint8_t getSlaveId() const;

  /**
   * @brief Returns the function code from the RX buffer.
   * @return uint8_t Function code.
   */
  uint8_t getFunction() const;

  /**
   * @brief Retrieves a single value from the RX buffer.
   * @tparam T Type of the value.
   * @param ix Index of the value (default: 0).
   * @return T The value at the specified index.
   */
  template <typename T>
  const T getData(uint16_t ix = 0) const;

  /**
   * @brief Returns a pointer to the RX buffer data as an array.
   * @tparam T Type of the array elements.
   * @return const T* Pointer to the data array.
   */
  template <typename T>
  const T* getDataArray() const;

  /**
   * @brief Retrieves a single bit from the RX buffer.
   * @param ix Bit index.
   * @return bool The bit value.
   */
  bool getBit(uint16_t ix) const;

  /**
   * @brief Returns the number of elements in the RX buffer.
   * @tparam T Type of the elements.
   * @return uint8_t Number of elements.
   */
  template <typename T>
  uint8_t getLen() const;

  /**
   * @brief Returns the byte length of the data in the RX buffer.
   * @return uint8_t Byte length.
   */
  uint8_t getByteLen() const;
};

#include "PDU.tpp"
