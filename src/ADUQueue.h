/**
 * @file ADUQueue.h
 * @brief Circular buffer queue for managing Modbus ADUs.
 * @details Template-based queue for storing and processing Modbus ADUs (e.g., ADURTU, ADUTCP) with timed sending.
 * @tparam T Type of ADU (must have _queuedTime, _delayToSend, and clear() method).
 */

#pragma once
#include <Arduino.h>

/**
 * @class ADUQueue
 * @brief Circular buffer queue for managing Modbus ADUs with timed sending.
 * @details Stores ADUs in a fixed-size queue and prioritizes sending based on _delayToSend.
 * @tparam T Type of ADU (must have _queuedTime, _delayToSend, and clear() method).
 */
template <typename T>
class ADUQueue {
 private:
  uint8_t _head = 0;       ///< Index of the first item in the queue.
  uint8_t _tail = 0;       ///< Index where the next item will be added.
  uint8_t _count = 0;      ///< Number of items in the queue.
  uint8_t _queueSize = 0;  ///< Maximum queue capacity (set by init).
  T** _items = nullptr;    ///< Array of pointers to ADU objects.

 public:
  /**
   * @brief Default constructor.
   * @details Initializes an empty queue with no allocated storage.
   */
  ADUQueue();

  /**
   * @brief Destructor.
   * @details Frees the item array (ADU pointers managed externally).
   */
  ~ADUQueue();

  /**
   * @brief Initializes the queue with the specified size.
   * @param queueSize Maximum number of ADUs the queue can hold.
   */
  void init(uint8_t queueSize);

  /**
   * @brief Adds an ADU to the queue.
   * @param item Pointer to the ADU to add.
   * @return bool True if added, false if queue is full or item is null.
   */
  bool add(T* item);

  /**
   * @brief Reads and removes the head ADU from the queue.
   * @param item Reference to store the retrieved ADU.
   * @return bool True if read, false if queue is empty or head is null.
   */
  bool read(T*& item);

  /**
   * @brief Peeks at the head ADU without removing it.
   * @param item Reference to store the peeked ADU.
   * @return bool True if peeked, false if queue is empty.
   */
  bool peek(T*& item) const;

  /**
   * @brief Reads and removes the first ADU ready to send.
   * @details Prioritizes the ADU with the lowest _delayToSend.
   * @param item Reference to store the retrieved ADU.
   * @return bool True if read, false if no ready ADU.
   */
  bool readReady(T*& item);

  /**
   * @brief Checks if there is an ADU ready to send.
   * @return bool True if a ready ADU exists, false otherwise.
   */
  bool hasReady() const;

  /**
   * @brief Checks if the queue is empty.
   * @return bool True if the queue is empty, false otherwise.
   */
  bool isEmpty() const;

  /**
   * @brief Clears the queue, resetting all items.
   * @details Calls clear() on all ADUs and resets queue state.
   */
  void clear();

  /**
   * @brief Returns the current number of items in the queue.
   * @return uint8_t Number of ADUs in the queue.
   */
  uint8_t count() const;
};

#include "ADUQueue.tpp"