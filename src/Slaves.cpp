#include "Slaves.h"

#include <string.h>

Slaves::Slaves() {}

Slaves::Slaves(uint8_t slave) {
  set(slave);  // Add the specified slave ID to the bitmask
}

Slaves::Slaves(uint8_t slave, int32_t repeatDelay) : Slaves(slave) {
  _repeatDelay = repeatDelay;  // Set delay between iteration cycles (ms)
}

Slaves::Slaves(std::initializer_list<uint8_t> list) {
  for (const uint8_t& value : list) set(value);  // Add valid slave IDs
}

Slaves::Slaves(std::initializer_list<uint8_t> list, int32_t delay) {
  for (const uint8_t& value : list) set(value);  // Add valid slave IDs
  _delay = delay;                                // Set delay between individual slave processing (ms)
}

Slaves::Slaves(std::initializer_list<uint8_t> list, int32_t delay, int32_t repeatDelay) : Slaves(list, delay) {
  _repeatDelay = repeatDelay;  // Set delay between iteration cycles (ms)
}

void Slaves::setRepeatDelay(int32_t repeatDelay) {
  _repeatDelay = repeatDelay;  // Allow in broadcast mode for scheduled broadcasts
}

int32_t Slaves::getRepeatDelay() {
  return _repeatDelay;
}

bool Slaves::getRepeat() const {
  return _repeatDelay > -1;
}

void Slaves::setDelay(int32_t delay) {
  _delay = delay;
}

int32_t Slaves::getDelay() const {
  return _delay;
}

void Slaves::set(std::initializer_list<uint8_t> list) {
  for (const uint8_t& value : list) set(value);  // Add valid slave IDs
}

void Slaves::set(uint8_t slaveId) {
  if (slaveId > MB_MAX_SLAVE_ID) return;       // Validate ID
  _mask[slaveId / 8] |= (1 << (slaveId % 8));  // Set bit in bitmask
}

void Slaves::set(uint8_t begin, uint8_t end) {
  if (begin > end || end > MB_MAX_SLAVE_ID) return;  // Validate range

  uint8_t startByte = begin / 8;
  uint8_t endByte = end / 8;
  uint8_t startBit = begin % 8;
  uint8_t endBit = end % 8;

  if (startByte == endByte) {
    // Set bits in a single byte
    for (uint8_t i = startBit; i <= endBit; ++i)
      _mask[startByte] |= (1 << i);
    return;
  }

  // Set remaining bits in the first byte
  for (uint8_t i = startBit; i < 8; ++i)
    _mask[startByte] |= (1 << i);

  // Fill intermediate bytes
  if (endByte > startByte + 1)
    memset(&_mask[startByte + 1], 0xFF, endByte - startByte - 1);

  // Set bits in the last byte
  for (uint8_t i = 0; i <= endBit; ++i)
    _mask[endByte] |= (1 << i);
}

void Slaves::remove(uint8_t slaveId) {
  if (slaveId > MB_MAX_SLAVE_ID) return;        // Validate ID
  _mask[slaveId / 8] &= ~(1 << (slaveId % 8));  // Clear bit in bitmask
}

bool Slaves::isSet(uint8_t slaveId) const {
  if (slaveId > MB_MAX_SLAVE_ID) return false;       // Validate ID
  return _mask[slaveId / 8] & (1 << (slaveId % 8));  // Check bit
}

void Slaves::clear() {
  memset(_mask, 0, sizeof(_mask));  // Zero out bitmask
  _active = SLAVE_BOF;              // Reset active ID
  _delay = 0;                       // Reset item delay
  _repeatDelay = -1;                // Disable cyclic repetition
}

uint8_t Slaves::peek() const {
  uint8_t start = (_active == SLAVE_BOF) ? 0 : _active + 1;  // Handle SLAVE_BOF overflow
  for (uint8_t i = start; i <= MB_MAX_SLAVE_ID; ++i) {
    if (isSet(i)) return i;  // Return next active ID
  }
  if (getRepeat()) {
    for (uint8_t i = 0; i <= _active; ++i) {
      if (isSet(i)) return i;  // Restart cycle if repeat enabled
    }
  }
  return SLAVE_EOF;  // No active IDs found
}

uint8_t Slaves::getNext() {
  uint8_t start = (_active == SLAVE_BOF) ? 0 : _active + 1;  // Handle SLAVE_BOF overflow
  for (uint8_t i = start; i <= MB_MAX_SLAVE_ID; ++i) {
    if (isSet(i)) {
      _active = i;  // Update active ID
      return _active;
    }
  }
  if (getRepeat()) {
    for (uint8_t i = 0; i <= MB_MAX_SLAVE_ID; ++i) {
      if (isSet(i)) {
        _active = i;  // Restart cycle and update active ID
        return _active;
      }
    }
  }
  return SLAVE_EOF;  // No active IDs found
}

uint8_t Slaves::getActive() const {
  return _active;
}

bool Slaves::hasMore() const {
  if (getRepeat()) return true;                              // Always more IDs if repeat is enabled
  uint8_t start = (_active == SLAVE_BOF) ? 0 : _active + 1;  // Handle SLAVE_BOF overflow
  for (uint8_t i = start; i <= MB_MAX_SLAVE_ID; ++i) {
    if (isSet(i)) return true;  // Check for active IDs
  }
  return false;
}

void Slaves::resetActive() {
  _active = SLAVE_BOF;
}

bool Slaves::valid() const {
  for (size_t i = 0; i < 32; i++) {
    if (_mask[i] > 0) return true;
  }
  return false;
}