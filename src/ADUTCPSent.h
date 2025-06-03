/**
 * @file ADUTCPSent.h
 * @brief Manages a buffer of sent Modbus TCP ADUs awaiting response.
 * @details Tracks sent ADUs by transaction ID or timeout for response matching.
 */

#pragma once
#include <stdint.h>

class ADUTCP;

/**
 * @class ADUTCPSent
 * @brief Fixed-size buffer for tracking sent Modbus TCP ADUs.
 * @details Stores sent ADUs and matches responses by transaction ID or detects timeouts.
 */
class ADUTCPSent {
 public:
  /**
   * @brief Default constructor.
   * @details Initializes an empty buffer.
   */
  ADUTCPSent();

  /**
   * @brief Destructor.
   * @details Frees the ADU pointer array (ADUTCP objects managed by ModbusTCPClient).
   */
  ~ADUTCPSent();

  /**
   * @brief Initializes the buffer with the specified size.
   * @param size Maximum number of ADUs the buffer can hold.
   */
  void init(uint8_t size);

  /**
   * @brief Adds a sent ADU to the buffer.
   * @param adu Pointer to the ADUTCP object to add.
   * @return bool True if added, false if buffer is full or ADU is null.
   */
  bool add(ADUTCP* adu);

  /**
   * @brief Retrieves an ADU by transaction ID, removing it from the buffer.
   * @param adu Reference to store the retrieved ADU.
   * @param tranID Transaction ID to match.
   * @return bool True if found, false if no matching ADU.
   */
  bool read(ADUTCP*& adu, uint16_t tranID);

  /**
   * @brief Retrieves the first ADU that has timed out (FIFO order).
   * @param adu Reference to store the retrieved ADU.
   * @param timeout Timeout threshold (ms).
   * @return bool True if found, false if no timed-out ADU.
   */
  bool readNextTimeout(ADUTCP*& adu, uint32_t timeout);

  /**
   * @brief Checks if the buffer is empty.
   * @return bool True if the buffer is empty, false otherwise.
   */
  bool isEmpty() const;

  /**
   * @brief Checks if there is free space in the buffer.
   * @return bool True if the buffer has free space, false otherwise.
   */
  bool hasFree() const;

 private:
  ADUTCP** _adu = nullptr;  ///< Array of pointers to sent ADUTCP objects.
  uint8_t _size = 0;        ///< Buffer capacity.
};