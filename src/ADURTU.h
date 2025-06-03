/**
 * @file ADURTU.h
 * @brief Manages Modbus RTU Application Data Unit (ADU).
 * @details Extends PDU with RTU-specific headers and CRC handling, supports cyclic slave iteration via Slaves.
 */

#pragma once
#include "ModbusDef.h"
#include "PDU.h"
#include "Slaves.h"

template <typename T>
class ADUQueue;
class ModbusRTUMaster;

/**
 * @class ADURTU
 * @brief Manages Modbus RTU ADU with slave ID and CRC handling.
 * @details Extends PDU to include RTU-specific headers (slave ID) and CRC for Modbus RTU communication.
 */
class ADURTU : public PDU {
  friend class ModbusRTUMaster;  // Access to private buffers and Slaves for RTU operations
  template <typename T>
  friend class ADUQueue;  // Access to private buffers for queue management

 private:
  uint8_t* _TXADURTUframe = nullptr;                                            ///< Transmit buffer for RTU ADU (slave ID + PDU + CRC).
  uint8_t* _RXADURTUframe = nullptr;                                            ///< Receive buffer for RTU ADU.
  uint8_t _responseRTUHead[MB_ADU_RTU_HEADER_LEN + MB_PDU_MAX_RESPONSE_LEN]{};  ///< Expected response header.
  uint16_t _responseLen = 0;                                                    ///< Length of received ADU.
  Slaves _slaves;                                                               ///< Manages slave IDs for cyclic iteration.
  ModbusRTUMaster* _modbusRTUMaster = nullptr;                                  ///< Pointer to RTU master for repeat logic.

  /**
   * @brief Resets the ADURTU state and clears buffers.
   * @details Overrides PDU::clear to reset RTU-specific fields.
   */
  void clear() override;

  /**
   * @brief Returns the total length of the transmit ADU.
   * @return uint16_t Length of the ADU (header + PDU + CRC).
   */
  uint16_t getTXADULen() const;

  /**
   * @brief Returns the expected response ADU length.
   * @return uint16_t Length of the expected response (header + PDU + CRC).
   */
  uint16_t getExpectedResponseLen() const;

  /**
   * @brief Returns the slave ID from the TX buffer.
   * @return uint8_t Slave ID.
   */
  uint8_t getSlaveId() const override;

 protected:
  /**
   * @brief Sets the RTU header with the specified slave ID.
   * @param slave Slave ID to set in the header.
   */
  void setHead(uint8_t slave);

  /**
   * @brief Sets the CRC for the transmit ADU.
   */
  void setCRC();

  /**
   * @brief Validates the response header (slave ID).
   * @return bool True if the header is valid, false otherwise.
   */
  bool checkResponseHead();

  /**
   * @brief Validates the response CRC.
   * @return bool True if the CRC is valid, false otherwise.
   */
  bool checkResponseCRC();

  /**
   * @brief Handles cyclic iteration over slaves.
   * @details Overrides PDU::repeatIfNeeded for RTU-specific slave iteration.
   * @return bool True if iteration should continue, false otherwise.
   */
  bool repeatIfNeeded() override;

 public:
  /**
   * @brief Default constructor.
   * @details Initializes an empty ADURTU object.
   */
  ADURTU();

  /**
   * @brief Destructor.
   * @details Frees dynamically allocated buffers.
   */
  ~ADURTU();

  /**
   * @brief Initializes buffers with user-defined PDU size.
   * @param PDUSize PDU size (16-253 bytes).
   */
  void init(uint8_t PDUSize);
};