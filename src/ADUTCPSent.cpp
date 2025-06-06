#include "ADUTCPSent.h"

#include "ADUTCP.h"
#include "ModbusUtility.h"

ADUTCPSent::ADUTCPSent() {}

ADUTCPSent::~ADUTCPSent() {
  delete[] _adu;
}

void ADUTCPSent::init(uint8_t size) {
  _size = size;
  _adu = new ADUTCP*[size]();  // Elements initialized to nullptr
}

bool ADUTCPSent::add(ADUTCP* adu) {
  if (!adu) return false;
  for (uint8_t i = 0; i < _size; ++i) {
    if (_adu[i] == nullptr) {
      _adu[i] = adu;
      adu->_sentTime = millis();
      return true;
    }
  }
  return false;  // No free space
}

bool ADUTCPSent::read(ADUTCP*& adu, uint16_t tranID) {
  for (uint8_t i = 0; i < _size; ++i) {
    if (_adu[i] && _adu[i]->getTransactionId() == tranID) {
      adu = _adu[i];
      _adu[i] = nullptr;
      return true;
    }
  }
  return false;  // No matching ADU
}

bool ADUTCPSent::readNextTimeout(ADUTCP*& adu, uint32_t timeout) {
  for (uint8_t i = 0; i < _size; ++i) {
    if (_adu[i] && on_ms(&_adu[i]->_sentTime, timeout, false)) {
      adu = _adu[i];
      _adu[i] = nullptr;
      return true;
    }
  }
  return false;  // No timed-out ADU
}

bool ADUTCPSent::isEmpty() const {
  for (uint8_t i = 0; i < _size; ++i) {
    if (_adu[i]) return false;
  }
  return true;
}

bool ADUTCPSent::hasFree() const {
  for (uint8_t i = 0; i < _size; ++i) {
    if (!_adu[i]) return true;
  }
  return false;
}