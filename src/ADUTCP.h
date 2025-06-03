/**
 * @file ADUTCP.h
 * @brief Manages Modbus TCP Application Data Unit (ADU).
 * @details Extends PDU with TCP-specific MBAP header and transaction ID handling, supports cyclic slave iteration via Slaves.
 */

#pragma once
#include "PDU.h"
#include "Slaves.h"

class ModbusTCPClient;

/**
 * @class ADUTCP
 * @brief Manages Modbus TCP ADU with MBAP header and transaction ID.
 * @details Extends PDU to include TCP-specific MBAP header and slave ID handling for Modbus TCP communication.
 */
class ADUTCP : public PDU {
  friend class ModbusTCPClient;  // Access to private buffers and Slaves for TCP operations
  friend class ClientItem;       // Access to private buffers for client management
  friend class ADUTCPSent;       // Access to private buffers for sent ADU tracking

 private:
  uint8_t* _TXADUTCPframe = nullptr;                                      ///< Transmit buffer for TCP ADU (MBAP + PDU).
  uint8_t* _RXADUTCPframe = nullptr;                                      ///< Receive buffer for TCP ADU.
  uint8_t _responseTCPHead[MB_ADU_MBAP_LEN + MB_PDU_MAX_RESPONSE_LEN]{};  ///< Expected response MBAP header.
  static uint16_t _transactionId;                                         ///< Transaction ID counter for TCP MBAP.
  uint32_t _sentTime = 0;                                                 ///< Time when ADU was sent (ms).
  uint32_t _responseLen = 0;                                              ///< Length of received ADU.
  Slaves _slaves;                                                         ///< Manages slave IDs for cyclic iteration.
  uint8_t _slave = 0xFF;                                                  ///< Slave ID for the current ADU.
  ModbusTCPClient* _modbusTCPClient = nullptr;                            ///< Pointer to TCP client for repeat logic.

 protected:
  /**
   * @brief Sets the MBAP header with transaction ID and slave ID.
   * @param slave Slave ID to set in the MBAP header.
   */
  void setMBAP(uint8_t slave);

  /**
   * @brief Validates the response MBAP header.
   * @details Checks transaction ID, protocol ID, and unit ID.
   * @return bool True if the header is valid, false otherwise.
   */
  bool checkResponseMBAP();

  /**
   * @brief Returns the current transaction ID.
   * @return uint16_t Transaction ID.
   */
  uint16_t getTransactionId() const;

  /**
   * @brief Returns the slave ID from the TX buffer.
   * @return uint8_t Slave ID.
   */
  uint8_t getId() const;

  /**
   * @brief Returns the total length of the transmit ADU.
   * @return uint16_t Length of the ADU (MBAP + PDU).
   */
  uint16_t getTXADULen() const;

  /**
   * @brief Returns the expected response ADU length.
   * @return uint16_t Length of the expected response (MBAP + PDU).
   */
  uint16_t getExpectedResponseLen() const;

  /**
   * @brief Handles cyclic iteration over slaves.
   * @details Overrides PDU::repeatIfNeeded for TCP-specific slave iteration.
   * @return bool True if iteration should continue, false otherwise.
   */
  bool repeatIfNeeded() override;

 public:
  /**
   * @brief Default constructor.
   * @details Initializes an empty ADUTCP object.
   */
  ADUTCP();

  /**
   * @brief Destructor.
   * @details Frees dynamically allocated buffers (typically static).
   */
  ~ADUTCP();

  /**
   * @brief Initializes buffers with user-defined PDU size.
   * @param PDUSize PDU size (16-253 bytes).
   */
  void init(uint8_t PDUSize);
};