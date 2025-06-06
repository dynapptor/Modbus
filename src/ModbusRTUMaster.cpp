#include "ModbusRTUMaster.h"

#include "ADURTU.h"
#include "Crc16.h"
#include "ModbusUtility.h"

ModbusRTUMaster::ModbusRTUMaster() {}

ModbusRTUMaster::~ModbusRTUMaster() {
  if (_adu) {
    for (uint8_t i = 0; i < _queueSize; i++) {
      delete _adu[i];  // Free ADURTU objects
    }
    delete[] _adu;  // Free the array
  }
}

bool ModbusRTUMaster::sendPDU(PDU* pdu, uint8_t slave) {
  ADURTU* adu = static_cast<ADURTU*>(pdu);
  adu->setHead(slave);
  adu->setCRC();
  adu->_responseLen = 0;
  if (!_queue.add(adu)) {
    adu->_err = MB_EX_LIB_QUEUE_FULL;
    adu->callCallback();
    return false;
  }
  return true;
}

PDU* ModbusRTUMaster::getFreePDU(const modbusCallback& cb, const Slaves& slaves) {
  for (uint8_t i = 0; i < _queueSize; ++i) {
    if (!_adu[i]->_used) {
      _adu[i]->_used = true;
      ADURTU* adu = _adu[i];
      adu->_callback = cb;
      adu->_slaves = slaves;
      adu->_slaves.getNext();
      return adu;
    }
  }
  PDU ret(slaves.peek());
  ret._err = MB_EX_LIB_NO_MORE_FREE_ADU;
  modbusCallback c = cb;
  c(ret);
  return nullptr;  // No free ADU
}

PDU* ModbusRTUMaster::getFreePDU(const modbusCallback& cb, uint8_t slave) {
  for (uint8_t i = 0; i < _queueSize; ++i) {
    if (!_adu[i]->_used) {
      _adu[i]->_used = true;
      ADURTU* adu = _adu[i];
      adu->_callback = cb;
      adu->_slaves.clear();
      adu->_slave = slave;
      return adu;
    }
  }
  PDU ret(slave);
  ret._err = MB_EX_LIB_NO_MORE_FREE_ADU;
  modbusCallback c = cb;
  c(ret);
  return nullptr;  // No free ADU
}

void ModbusRTUMaster::calcTimeout(uint8_t data, uint8_t parity, uint8_t stopBit) {
  _lastByteTime = 0;  // Reset last frame timestamp
  if (_baud <= 19200) {
    // Calculate character time: 1000000 / (baud / total bits) in µs
    // Total bits = 8 data bits + 1 start bit + parity bits + stop bits
    // Byte timeout = 1.5 character times, frame timeout = 3.5 character times
    uint32_t charTime = 1000000 / (_baud / (data + 1 + parity + stopBit));
    _byteTimeout = charTime * 1.5;
    _frameTimeout = charTime * 3.5;
  } else {
    // Fixed timeouts for higher baud rates (Modbus RTU standard)
    _byteTimeout = 750;    // 750 µs
    _frameTimeout = 1750;  // 1750 µs
  }
}

void ModbusRTUMaster::begin(uint8_t PDUSize, uint8_t queueSize, Stream* stream, uint32_t baud, UartConfig cfg, int16_t re, int16_t de) {
  _queueSize = queueSize;
  _queue.init(queueSize);
  _adu = new ADURTU*[_queueSize];
  for (size_t i = 0; i < _queueSize; i++) {
    _adu[i] = new ADURTU();
    _adu[i]->init(PDUSize);
    _adu[i]->_modbusRTUMaster = this;
  }
  _stream = stream;
  _baud = baud;
  _cfg = cfg;
  _re = re;
  _de = de;
  if (_de != -1) pinMode(_de, OUTPUT);
  if (_re != -1) pinMode(_re, OUTPUT);
  endTransaction();
  calcTimeout(5 + (((uint8_t)cfg >> 1) & 0x03), ((uint8_t)cfg >> 5) & 0x03, 1 + (((uint8_t)cfg >> 3) & 0x01));
  setIsBigEndian();
  clearBuffer();
}

void ModbusRTUMaster::beginTransaction() {
  if (_de > -1) digitalWrite(_de, HIGH);
  if (_re > -1) digitalWrite(_re, HIGH);
}

void ModbusRTUMaster::endTransaction() {
  _stream->flush();
  if (_de > -1) digitalWrite(_de, LOW);
  if (_re > -1) digitalWrite(_re, LOW);
}

uint16_t ModbusRTUMaster::clearBuffer() {
  uint16_t count = 0;
  while (_stream->available()) {
    _stream->read();
    count++;
  }
  return count;
}

void ModbusRTUMaster::send(uint8_t* buffer, uint16_t len) {
  beginTransaction();
  _stream->write(buffer, len);
  endTransaction();
  _lastByteTime = micros();
}

uint32_t ModbusRTUMaster::getFrameTimeout() const { return _frameTimeout; }
void ModbusRTUMaster::setFrameTimeout(uint32_t frameTimeout) { _frameTimeout = frameTimeout; }
uint32_t ModbusRTUMaster::getByteTimeout() const { return _byteTimeout; }
void ModbusRTUMaster::setByteTimeout(uint32_t byteTimeout) { _byteTimeout = byteTimeout; }
uint32_t ModbusRTUMaster::getResponseTimeout() const { return _responseTimeout; }
void ModbusRTUMaster::setResponseTimeout(uint32_t t) { _responseTimeout = t; }

void ModbusRTUMaster::reset() {
  _currentADU = nullptr;
  _errorReceive = false;
}

void ModbusRTUMaster::loop() {
  switch (_state) {
    case MB_ASYNC_STATE_BUFFER_CLEAR: {
      if (_stream->available()) {
        clearBuffer();  // Clear invalid or stale data
        _lastByteTime = micros();
      } else if (on_us(&_lastByteTime, _frameTimeout, false)) {
        _state = MB_ASYNC_STATE_IDLE;
      }
      break;
    }
    case MB_ASYNC_STATE_IDLE: {
      if (!_queue.isEmpty()) {
        if (on_us(&_lastByteTime, _frameTimeout, false)) {
          if (_queue.readReady(_currentADU)) {
            send(_currentADU->_TXADURTUframe, _currentADU->getTXADULen());
            // printBuffer(_currentADU->_TXADURTUframe, _currentADU->getTXADULen());
            if (_currentADU->getSlaveId() == 0) {
              _currentADU->callCallback();
              reset();
              return;
            }
            _state = MB_ASYNC_STATE_RECEIVE;
          }
        }
      } else {
        // Ensure immediate retry with a conservative delay (half byte time)
        _lastByteTime = micros() - _frameTimeout;
      }
      break;
    }
    case MB_ASYNC_STATE_RECEIVE: {
      uint16_t received = _stream->available();
      if (received) {
        _stream->readBytes(_currentADU->_RXADURTUframe + _currentADU->_responseLen, received);
        _currentADU->_responseLen += received;
        // printBuffer(_currentADU->_RXADURTUframe, _currentADU->_responseLen);
        _lastByteTime = micros();
        if (_currentADU->_responseLen >= 2) {
          if (!_currentADU->checkResponseHead()) {
            if (clearBuffer()) {
              _state = MB_ASYNC_STATE_BUFFER_CLEAR;
            } else {
              _state = MB_ASYNC_STATE_IDLE;
            }
            reset();
            return;
          }
          if (_currentADU->_RXADURTUframe[1] == _currentADU->_responseRTUHead[1] + 0x80) _errorReceive = true;
          _state = MB_ASYNC_STATE_HEADCHEKD;
        }
      } else {  // nothing received jet, chek timeout
        if (on_us(&_lastByteTime, _responseTimeout, false)) {
          if (clearBuffer()) {
            _state = MB_ASYNC_STATE_BUFFER_CLEAR;
          } else {
            _state = MB_ASYNC_STATE_IDLE;
          }
          _currentADU->_err = MB_EX_LIB_RESPONSE_TIMEOUT;
          _currentADU->callCallback();
          reset();
          return;
        }
      }
    }
    case MB_ASYNC_STATE_HEADCHEKD: {
      uint16_t received = _stream->available();
      if (received) {
        _stream->readBytes(_currentADU->_RXADURTUframe + _currentADU->_responseLen, received);
        _currentADU->_responseLen += received;
        // printBuffer(_currentADU->_RXADURTUframe, _currentADU->_responseLen);
        _lastByteTime = micros();
      }
      if (_currentADU->getExpectedResponseLen() == _currentADU->_responseLen || (_errorReceive && _currentADU->_responseLen == 5)) {
        if (!_currentADU->checkResponseCRC()) {
          if (clearBuffer()) {
            _state = MB_ASYNC_STATE_BUFFER_CLEAR;
          } else {
            _state = MB_ASYNC_STATE_IDLE;
          }
          _currentADU->callCallback();
          reset();
          return;
        }
        // printBuffer(_currentADU->_RXADURTUframe, _currentADU->_responseLen);
        _currentADU->invoke();
        _state = MB_ASYNC_STATE_IDLE;
        reset();
        return;
      } else {  // Chek byte timeout but only if nothing received jet
        if (_currentADU->_responseLen != 0 && on_us(&_lastByteTime, _byteTimeout, false)) {
          if (clearBuffer()) {
            _state = MB_ASYNC_STATE_BUFFER_CLEAR;
          } else {
            _state = MB_ASYNC_STATE_IDLE;
          }
          _currentADU->_err = MB_EX_LIB_RESPONSE_TIMEOUT;
          _currentADU->callCallback();
          reset();
          return;
        }
      }
      break;
    }
  }
}