/**
 * @file Slaves.h
 * @brief Manages a set of Modbus slave IDs for cyclic iteration.
 * @details Uses a 256-bit bitmap to store slave IDs (1–247, or 0 for broadcast) and supports delays for multi-slave polling.
 */

#pragma once
#include <Arduino.h>

#include <initializer_list>

#include "ModbusDef.h"

/**
 * @def SLAVE_NULL
 * @brief Represents a null/invalid slave ID (0xFD).
 */
#define SLAVE_NULL 0xFD

/**
 * @def SLAVE_EOF
 * @brief Indicates end of slave iteration (0xFE).
 */
#define SLAVE_EOF 0xFE

/**
 * @def SLAVE_BOF
 * @brief Indicates beginning of slave iteration (0xFF).
 */
#define SLAVE_BOF 0xFF

/**
 * @class Slaves
 * @brief Manages a set of Modbus slave IDs with cyclic iteration.
 * @details Uses a 256-bit bitmap for slave IDs and supports delays for polling multiple slaves or broadcasting.
 */
class Slaves {
 private:
  uint8_t _mask[32]{};          ///< 256-bit bitmap for slave IDs (0–255, 248–255 reserved).
  int32_t _delay = 0;           ///< Delay (ms) between individual slave ID processing.
  int32_t _repeatDelay = -1;    ///< Delay (ms) between iteration cycles, -1 disables repeat.
  uint8_t _active = SLAVE_BOF;  ///< Tracks the last active slave ID during iteration.

 public:
  /**
   * @brief Default constructor.
   * @details Initializes an empty slave set.
   */
  Slaves();

  /**
   * @brief Constructor with a single slave ID.
   * @param slave Slave ID (1–247, or 0 for broadcast).
   */
  explicit Slaves(uint8_t slave);

  /**
   * @brief Constructor with a single slave ID and repeat delay.
   * @param slave Slave ID (1–247, or 0 for broadcast).
   * @param repeatDelay Delay (ms) between iteration cycles, -1 disables repeat.
   */
  Slaves(uint8_t slave, int32_t repeatDelay);

  /**
   * @brief Constructor with multiple slave IDs.
   * @param list List of slave IDs (e.g., {1, 2, 3}).
   */
  Slaves(std::initializer_list<uint8_t> list);

  /**
   * @brief Constructor with multiple slave IDs and item delay.
   * @param list List of slave IDs.
   * @param delay Delay (ms) between individual slave processing.
   */
  Slaves(std::initializer_list<uint8_t> list, int32_t delay);

  /**
   * @brief Constructor with multiple slave IDs, item delay, and repeat delay.
   * @param list List of slave IDs.
   * @param delay Delay (ms) between individual slave processing.
   * @param repeatDelay Delay (ms) between iteration cycles.
   */
  Slaves(std::initializer_list<uint8_t> list, int32_t delay, int32_t repeatDelay);

  /**
   * @brief Sets the delay between individual slave processing.
   * @param delay Delay in milliseconds.
   */
  void setDelay(int32_t delay);

  /**
   * @brief Sets the delay between iteration cycles.
   * @param repeatDelay Delay (ms), -1 disables repetition.
   */
  void setRepeatDelay(int32_t repeatDelay);

  /**
   * @brief Gets the delay between iteration cycles.
   * @return int32_t Repeat delay (ms).
   */
  int32_t getRepeatDelay();

  /**
   * @brief Checks if cyclic iteration is enabled.
   * @return bool True if repeatDelay >= 0, false otherwise.
   */
  bool getRepeat() const;

  /**
   * @brief Gets the delay between individual slave processing.
   * @return int32_t Delay (ms).
   */
  virtual int32_t getDelay() const;

  /**
   * @brief Adds a single slave ID to the bitmap.
   * @param slaveId Slave ID (1–247, or 0 for broadcast).
   */
  void set(uint8_t slaveId);

  /**
   * @brief Adds a range of slave IDs to the bitmap.
   * @param begin Starting slave ID.
   * @param end Ending slave ID.
   */
  void set(uint8_t begin, uint8_t end);

  /**
   * @brief Adds multiple slave IDs from an initializer list.
   * @param list List of slave IDs.
   */
  void set(std::initializer_list<uint8_t> list);

  /**
   * @brief Removes a slave ID from the bitmap.
   * @param slaveId Slave ID to remove.
   */
  void remove(uint8_t slaveId);

  /**
   * @brief Checks if a slave ID is active in the bitmap.
   * @param slaveId Slave ID to check.
   * @return bool True if the slave ID is set, false otherwise.
   */
  bool isSet(uint8_t slaveId) const;

  /**
   * @brief Clears all slave IDs from the bitmap.
   * @details Resets _mask, _active, and delays.
   */
  void clear();

  /**
   * @brief Gets the next active slave ID.
   * @details Updates _active to the next valid slave ID.
   * @return uint8_t Next slave ID, or SLAVE_EOF if no more slaves.
   */
  uint8_t getNext();

  /**
   * @brief Peeks at the next active slave ID without updating _active.
   * @return uint8_t Next slave ID, or SLAVE_EOF if no more slaves.
   */
  uint8_t peek() const;

  /**
   * @brief Returns the current active slave ID.
   * @return uint8_t Current _active value.
   */
  uint8_t getActive() const;

  /**
   * @brief Resets iteration to the start.
   * @details Sets _active to SLAVE_BOF.
   */
  void resetActive();

  /**
   * @brief Checks if there are more active slave IDs to iterate.
   * @return bool True if active IDs remain, false otherwise.
   */
  bool hasMore() const;

  /**
   * @brief Checks if the slave set is valid.
   * @return bool True if the set contains data, false otherwise.
   */
  bool valid() const;
};