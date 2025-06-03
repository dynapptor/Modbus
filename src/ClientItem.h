/**
 * @file ClientItem.h
 * @brief Manages a single Modbus TCP slave connection.
 * @details Handles TCP client, ADU queue, sent ADUs, and timeout management for a specific Modbus TCP slave.
 */

#pragma once
#include <Arduino.h>
#include <Client.h>

#include "ADUQueue.h"
#include "ADUTCPSent.h"
#include "ModbusDef.h"

class ADUTCP;

/**
 * @class ClientItem
 * @brief Manages a single Modbus TCP slave connection with queue and timeout handling.
 * @details Maintains a TCP connection to a Modbus slave, manages ADU queue, and tracks sent ADUs.
 */
class ClientItem {
  friend class ModbusTCPClient;  // Access to private members for client management

 private:
  uint8_t _id = 0;                                      ///< Slave ID (0 if invalid).
  uint8_t _maxCount = 0;                                ///< Maximum ADU queue size.
  Client* _client = nullptr;                            ///< TCP client instance.
  IPAddress _ip;                                        ///< Slave IP address.
  uint16_t _port = 502;                                 ///< TCP port (default: 502).
  bool _keepAlive = true;                               ///< Reconnect if connection lost.
  uint32_t _lastReconnectAttempt = 0;                   ///< Last reconnect attempt timestamp (ms).
  uint32_t _reconnectInterval = MB_RECONNECT;           ///< Reconnect interval (ms).
  uint32_t _responseTimeout = MB_TCP_RESPONSE_TIMEOUT;  ///< Response timeout (ms).
  ADUTCP* _currentADU = nullptr;                        ///< Currently processed ADU.
  bool _allAtOnce = false;                              ///< Send all ready ADUs at once.
  int16_t _incomingByte = 0;                            ///< Expected incoming bytes for response.
  ADUTCPSent _sent;                                     ///< Buffer for sent ADUs awaiting response.
  ADUQueue<ADUTCP> _queue;                              ///< Queue for pending ADUs.

  /**
   * @brief Sends an ADU over the TCP connection.
   * @param adu Pointer to the ADUTCP object to send.
   */
  void send(ADUTCP* adu);

  /**
   * @brief Clears the TCP client buffer.
   * @return uint16_t Number of bytes cleared.
   */
  uint16_t clearBuffer();

  /**
   * @brief Maintains connection and reconnects if needed.
   * @return bool True if connection is active, false otherwise.
   */
  bool keepAlive();

  /**
   * @brief Attempts to reconnect to the slave.
   * @return bool True if reconnection succeeded, false otherwise.
   */
  bool reconnect();

  /**
   * @brief Resets current ADU and incoming byte count.
   */
  void reset();

 public:
  /**
   * @brief Default constructor.
   * @details Initializes an empty ClientItem.
   */
  ClientItem();

  /**
   * @brief Destructor.
   * @details Cleans up resources, does not delete the TCP client.
   */
  ~ClientItem();

  /**
   * @brief Configures the client item.
   * @param id Slave ID (1-247, or 0 for broadcast).
   * @param allAtOnce Send all ready ADUs at once if true.
   * @param maxCount Maximum ADU queue size.
   * @param client Pointer to the TCP client instance.
   * @param ip Slave IP address.
   * @param port TCP port (default: 502).
   * @param keepAlive Reconnect if connection lost.
   */
  void set(uint8_t id, bool allAtOnce, uint8_t maxCount, Client* client, IPAddress ip,
           uint16_t port = 502, bool keepAlive = true);

  /**
   * @brief Main loop for connection, sending, and response handling.
   * @details Processes queued ADUs, sends data, and handles responses.
   */
  void loop();

  /**
   * @brief Checks if the client item is valid.
   * @return bool True if slave ID is non-zero, false otherwise.
   */
  bool isValid() const;
};