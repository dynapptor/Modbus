/**
 * @file ModbusTCPClient.h
 * @brief Modbus TCP client implementation for multiple slave communication.
 * @details Extends ModbusMaster to manage TCP-specific PDU (ADUTCP) sending and receiving for multiple slaves.
 */

#pragma once
#include <Client.h>

#include "ModbusMaster.h"

class ClientItem;
class ADUTCP;

/**
 * @class ModbusTCPClient
 * @brief Manages Modbus TCP communication with multiple slaves.
 * @details Handles client connections, ADU pool, and response timeouts for TCP-based Modbus communication.
 */
class ModbusTCPClient : public ModbusMaster {
  friend class ADUTCP;  ///< Access to private members for ADUTCP callback handling.

 private:
  ADUTCP** _adu = nullptr;                          ///< Array of ADUTCP pointers for the ADU pool.
  uint8_t _ADUPoolSize = 0;                         ///< Size of the ADU pool.
  ClientItem* _clients = nullptr;                   ///< Array of client items (slaves).
  uint8_t _clientCount = 0;                         ///< Number of client slots.
  uint32_t _responseTimeout = MB_RESPONSE_TIMEOUT;  ///< Response timeout (ms).

  /**
   * @brief Retrieves a free PDU instance for the operation.
   * @param cb Callback function for response handling.
   * @param slaves Set of slave IDs for the operation.
   * @return PDU* Pointer to the free PDU (ADUTCP), or nullptr if none available.
   */
  PDU* getFreePDU(const modbusCallback& cb, const Slaves& slaves) override;

  /**
   * @brief Retrieves a free PDU instance for a single slave operation.
   * @param cb Callback function for response handling.
   * @param slave Slave ID (1-247, or 0 for broadcast).
   * @return PDU* Pointer to the free PDU (ADUTCP), or nullptr if none available.
   */
  PDU* getFreePDU(const modbusCallback& cb, uint8_t slave) override;

  /**
   * @brief Sends the ADUTCP to the specified slave.
   * @param pdu Pointer to the PDU (ADUTCP) to send.
   * @param slave Slave ID (1-247).
   * @return bool True if sent successfully, false otherwise.
   */
  bool sendPDU(PDU* pdu, uint8_t slave) override;

 public:
  /**
   * @brief Default constructor.
   * @details Initializes an empty ModbusTCPClient.
   */
  ModbusTCPClient();

  /**
   * @brief Destructor.
   * @details Frees ADUTCP objects and client array.
   */
  ~ModbusTCPClient();

  /**
   * @brief Initializes the TCP client with specified parameters.
   * @param ADUPoolSize Size of the ADU pool.
   * @param PDUSize PDU buffer size (16-253 bytes).
   * @param clientCount Maximum number of slaves.
   */
  void begin(uint8_t ADUPoolSize, uint8_t PDUSize, uint8_t clientCount);

  /**
   * @brief Adds a client for a slave.
   * @param id Slave ID (1-247, or 0 for broadcast).
   * @param allAtOnce Send all ready ADUs at once if true.
   * @param queueSize ADU queue capacity.
   * @param client Pointer to the TCP client instance.
   * @param ip Slave IP address.
   * @param port TCP port (default: 502).
   * @param keepAlive Reconnect if connection lost.
   * @return bool True if added, false if no free slot or ID is not unique.
   */
  bool addClient(uint8_t id, bool allAtOnce, uint8_t queueSize, Client* client, IPAddress ip,
                 uint16_t port = 502, bool keepAlive = true);

  /**
   * @brief Main loop for communication.
   * @details Delegates to ClientItem loop for processing.
   */
  void loop() override;

  /**
   * @brief Gets the response timeout.
   * @return uint32_t Response timeout (ms).
   */
  uint32_t getResponseTimeout() const;

  /**
   * @brief Sets the response timeout.
   * @param t Response timeout (ms).
   */
  void setResponseTimeout(uint32_t t);
};