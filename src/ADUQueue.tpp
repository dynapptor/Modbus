#pragma once
#include "ADUQueue.h"
#include "ModbusUtility.h"

template <typename T>
ADUQueue<T>::ADUQueue() {}

template <typename T>
ADUQueue<T>::~ADUQueue() {
  delete[] _items;  // Note: T* pointers are managed by the caller (e.g., ModbusRTUMaster)
}

template <typename T>
void ADUQueue<T>::init(uint8_t queueSize) {
  _queueSize = queueSize;
  _items = new T* [_queueSize] {};  // Allocate and initialize to nullptr
}

template <typename T>
bool ADUQueue<T>::add(T* item) {
  if (_count >= _queueSize || item == nullptr) return false;
  _items[_tail] = item;
  _tail = (_tail + 1) % _queueSize;
  _count++;
  return true;
}

template <typename T>
bool ADUQueue<T>::peek(T*& item) const {
  if (_count == 0) return false;
  item = _items[_head];
  return true;
}

template <typename T>
bool ADUQueue<T>::read(T*& item) {
  if (_count == 0 || !_items[_head]) return false;
  item = _items[_head];
  _items[_head] = nullptr;
  _head = (_head + 1) % _queueSize;
  _count--;
  return true;
}

template <typename T>
bool ADUQueue<T>::readReady(T*& item) {
  if (_count == 0) return false;
  uint8_t current = _head;
  uint8_t readyIndex = 255;        // Invalid index
  uint32_t minDelay = UINT32_MAX;  // Track smallest delay
  for (uint8_t i = 0; i < _count; ++i) {
    T* candidate = _items[current];
    if (candidate && on_ms(&candidate->_queuedTime, candidate->_delayToSend, false)) {
      if (candidate->_delayToSend < minDelay) {
        minDelay = candidate->_delayToSend;
        readyIndex = current;
      }
    }
    current = (current + 1) % _queueSize;
  }
  if (readyIndex == 255) return false;  // No ready ADU found
  // Move ready item to head
  if (readyIndex != _head) {
    T* temp = _items[readyIndex];
    _items[readyIndex] = _items[_head];
    _items[_head] = temp;
  }
  return read(item);  // Read from head
}

template <typename T>
bool ADUQueue<T>::hasReady() const {
  if (_count == 0) return false;
  uint8_t current = _head;
  for (uint8_t i = 0; i < _count; ++i) {
    if (_items[current] && on_ms(&_items[current]->_queuedTime, _items[current]->_delayToSend, false)) {
      return true;
    }
    current = (current + 1) % _queueSize;
  }
  return false;
}

template <typename T>
bool ADUQueue<T>::isEmpty() const {
  return _count == 0;
}

template <typename T>
void ADUQueue<T>::clear() {
  for (uint8_t i = 0; i < _queueSize; ++i) {
    if (_items[i]) {
      _items[i]->clear();  // Reset ADU state
      _items[i] = nullptr;
    }
  }
  _head = _tail = _count = 0;
}

template <typename T>
uint8_t ADUQueue<T>::count() const {
  return _count;
}