/**
 * @file ModbusRTUMaster.h
 * @brief Modbus RTU master implementation for serial communication.
 * @details Extends ModbusMaster to manage RTU-specific PDU (ADURTU) sending and receiving over RS-485.
 */

#pragma once
#include <Arduino.h>

#include "ADUQueue.h"
#include "ModbusMaster.h"

class ADURTU;
class PDU;

/**
 * @enum UartConfig
 * @brief UART configuration options for Modbus RTU communication.
 */
enum class UartConfig : uint8_t {
  Mode_8N1 = 0x06,  ///< 8 data bits, no parity, 1 stop bit.
  Mode_8E1 = 0x26,  ///< 8 data bits, even parity, 1 stop bit.
  Mode_8O1 = 0x36,  ///< 8 data bits, odd parity, 1 stop bit.
  Mode_8N2 = 0x0E,  ///< 8 data bits, no parity, 2 stop bits.
  Mode_8E2 = 0x2E   ///< 8 data bits, even parity, 2 stop bits.
};

/**
 * @class ModbusRTUMaster
 * @brief Manages Modbus RTU communication over serial (RS-485).
 * @details Handles UART configuration, ADU queue, and asynchronous state machine for RTU communication.
 */
class ModbusRTUMaster : public ModbusMaster {
  friend class ADURTU;  ///< Access to private members for ADURTU callback handling.

 private:
  /**
   * @enum AsyncState
   * @brief Asynchronous state machine states for RTU communication.
   */
  enum {
    MB_ASYNC_STATE_IDLE,         ///< Waiting for a PDU to send.
    MB_ASYNC_STATE_RECEIVE,      ///< Receiving response data.
    MB_ASYNC_STATE_HEADCHEKD,    ///< Slave ID checked.
    MB_ASYNC_STATE_BUFFER_CLEAR  ///< Clearing serial buffer.
  };

  Stream* _stream = nullptr;                               ///< Serial stream for communication.
  UartConfig _cfg = UartConfig::Mode_8N1;                  ///< UART configuration.
  uint32_t _baud = 115200;                                 ///< Baud rate (default: 115200).
  int16_t _re = -1;                                        ///< RS-485 RE pin (-1 if not used).
  int16_t _de = -1;                                        ///< RS-485 DE pin (-1 if not used).
  uint32_t _byteTimeout = 0;                               ///< Byte timeout (µs, per Modbus RTU standard).
  uint32_t _frameTimeout = 0;                              ///< Frame timeout (µs, per Modbus RTU standard).
  uint32_t _lastByteTime = 0;                              ///< Timestamp of last byte (µs).
  uint32_t _responseTimeout = MB_RESPONSE_TIMEOUT * 1000;  ///< Response timeout (µs).
  uint8_t _queueSize = 0;                                  ///< Size of ADU queue.
  ADURTU* _currentADU = nullptr;                           ///< Currently processed ADU.
  ADURTU** _adu = nullptr;                                 ///< Array of ADURTU pointers.
  ADUQueue<ADURTU> _queue;                                 ///< Queue for pending ADUs.
  uint8_t _state = MB_ASYNC_STATE_IDLE;                    ///< Current state machine state.
  bool _errorReceive = false;                              ///< Error response receive flag.

  /**
   * @brief Resets the RTU master state.
   * @details Clears current ADU and state machine.
   */
  void reset();

  /**
   * @brief Calculates byte and frame timeouts based on UART configuration.
   * @param data Number of data bits.
   * @param parity Parity bit (0 for none, 1 for even/odd).
   * @param stopBit Number of stop bits.
   */
  void calcTimeout(uint8_t data, uint8_t parity, uint8_t stopBit);

  /**
   * @brief Sends a raw buffer over the serial stream.
   * @param buffer Pointer to the byte buffer.
   * @param len Length of the buffer.
   */
  void send(uint8_t* buffer, uint16_t len);

  /**
   * @brief Initiates RS-485 transmission.
   * @details Sets DE/RE pins for transmission.
   */
  void beginTransaction();

  /**
   * @brief Completes RS-485 transmission.
   * @details Resets DE/RE pins after transmission.
   */
  void endTransaction();

  /**
   * @brief Clears the serial buffer.
   * @return uint16_t Number of bytes cleared.
   */
  uint16_t clearBuffer();

  /**
   * @brief Retrieves a free PDU instance for the operation.
   * @param cb Callback function for response handling.
   * @param slaves Set of slave IDs for the operation.
   * @return PDU* Pointer to the free PDU (ADURTU), or nullptr if none available.
   */
  PDU* getFreePDU(const modbusCallback& cb, const Slaves& slaves) override;

  /**
   * @brief Retrieves a free PDU instance for a single slave operation.
   * @param cb Callback function for response handling.
   * @param slave Slave ID (1-247, or 0 for broadcast).
   * @return PDU* Pointer to the free PDU (ADURTU), or nullptr if none available.
   */
  PDU* getFreePDU(const modbusCallback& cb, uint8_t slave) override;

  /**
   * @brief Sends the ADURTU to the specified slave.
   * @param pdu Pointer to the PDU (ADURTU) to send.
   * @param slave Slave ID (1-247, or 0 for broadcast).
   * @return bool True if sent successfully, false otherwise.
   */
  bool sendPDU(PDU* pdu, uint8_t slave) override;

 public:
  /**
   * @brief Default constructor.
   * @details Initializes an empty ModbusRTUMaster.
   */
  ModbusRTUMaster();

  /**
   * @brief Destructor.
   * @details Frees ADURTU objects and array.
   */
  ~ModbusRTUMaster();

  /**
   * @brief Initializes the RTU master with specified parameters.
   * @param PDUSize PDU buffer size (16-253 bytes).
   * @param queueSize ADU queue capacity.
   * @param stream Pointer to the serial stream.
   * @param baud Baud rate (default: 115200).
   * @param cfg UART configuration (default: Mode_8N1).
   * @param re RS-485 RE pin (-1 if not used).
   * @param de RS-485 DE pin (-1 if not used).
   */
  void begin(uint8_t PDUSize, uint8_t queueSize, Stream* stream, uint32_t baud = 115200,
             UartConfig cfg = UartConfig::Mode_8N1, int16_t re = -1, int16_t de = -1);

  /**
   * @brief Gets the frame timeout.
   * @return uint32_t Frame timeout (µs).
   */
  uint32_t getFrameTimeout() const;

  /**
   * @brief Sets the frame timeout.
   * @param frameTimeout Frame timeout (µs).
   */
  void setFrameTimeout(uint32_t frameTimeout);

  /**
   * @brief Gets the byte timeout.
   * @return uint32_t Byte timeout (µs).
   */
  uint32_t getByteTimeout() const;

  /**
   * @brief Sets the byte timeout.
   * @param byteTimeout Byte timeout (µs).
   */
  void setByteTimeout(uint32_t byteTimeout);

  /**
   * @brief Gets the response timeout.
   * @return uint32_t Response timeout (µs).
   */
  uint32_t getResponseTimeout() const;

  /**
   * @brief Sets the response timeout.
   * @param t Response timeout (µs).
   */
  void setResponseTimeout(uint32_t t);

  /**
   * @brief Main loop for communication timing and response handling.
   * @details Processes queued ADUs and handles responses.
   */
  void loop() override;
};