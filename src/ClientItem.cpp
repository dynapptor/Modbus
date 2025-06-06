#include "ClientItem.h"

#include "ADUQueue.h"
#include "ADUTCP.h"
#include "ModbusUtility.h"

ClientItem::ClientItem() {}

ClientItem::~ClientItem() {}

void ClientItem::set(uint8_t id, bool allAtOnce, uint8_t maxCount, Client* client, IPAddress ip, uint16_t port, bool keepAlive) {
  _id = id;
  _allAtOnce = allAtOnce;
  _maxCount = maxCount;
  _client = client;
  _ip = ip;
  _port = port;
  _keepAlive = keepAlive;
  _sent.init(_maxCount);
  _queue.init(_maxCount);
  _lastReconnectAttempt = millis();
}

bool ClientItem::reconnect() {
  if (!_client->connected()) {
    if (on_ms(&_lastReconnectAttempt, _reconnectInterval, true)) {
      if (!_client->connect(_ip, _port)) {
        return false;
      }
      return true;
    }
  }
  return _client->connected();
}

bool ClientItem::keepAlive() {
  if (_keepAlive) {
    return reconnect();
  }
  return _client->connected();
}

void ClientItem::loop() {
  if (!keepAlive()) return;  // Ensure connection is active
  if (_allAtOnce) {          // Send all ready ADUs at once
    if (_queue.hasReady() && reconnect()) {
      ADUTCP* adu;
      while (_queue.hasReady()) {
        if (_queue.readReady(adu)) {
          if (_sent.hasFree()) {
            send(adu);
            _sent.add(adu);
          } else {
            adu->_err = MB_EX_LIB_TCP_SENT_BUFFER_FULL;
            adu->callCallback();
            return;
          }
        } else {
          break;  // No more ADUs to send
        }
      }
    }
  } else {  // Send one ADU at a time
    if (!_currentADU && _queue.hasReady()) {
      if (_queue.readReady(_currentADU)) {
        send(_currentADU);
      }
    }
  }
  if (!_currentADU && _sent.isEmpty()) return;  // No ADU awaiting response
  if (_incomingByte == 0) {                     // Waiting for MBAP header
    uint8_t mbap[MB_ADU_MBAP_LEN];
    if (_client->available() >= MB_ADU_MBAP_LEN) {
      _client->read(mbap, MB_ADU_MBAP_LEN);
      uint16_t tranId = (static_cast<uint16_t>(mbap[0]) << 8) | mbap[1];
      if (_allAtOnce) {
        if (!_sent.read(_currentADU, tranId)) {
          clearBuffer();  // Unknown transaction ID
          reset();
          return;  // No callback, general error handling could be added
        }
      }
      if (_currentADU) {
        _incomingByte = ((mbap[4] << 8) | mbap[5]) - 1;  // Exclude slave ID byte
        memcpy(_currentADU->_RXADUTCPframe, mbap, MB_ADU_MBAP_LEN);
        if (!_currentADU->checkResponseMBAP()) {
          clearBuffer();
          reset();
        }
      } else {
        clearBuffer();
        reset();
        return;  // No callback, general error handling could be added
      }
    }
  }
  if (_incomingByte && _client->available() >= _incomingByte) {
    _client->read(_currentADU->_RXADUTCPframe + MB_ADU_MBAP_LEN, _incomingByte);
    _currentADU->invoke();
    reset();
  }
  // Check timeouts
  if (_allAtOnce && !_sent.isEmpty()) {
    ADUTCP* adu;
    while (_sent.readNextTimeout(adu, _responseTimeout)) {
      adu->_err = MB_EX_LIB_RESPONSE_TIMEOUT;
      adu->callCallback();
    }
  } else if (_currentADU) {
    if (on_ms(&_currentADU->_sentTime, _responseTimeout, false)) {
      _currentADU->_err = MB_EX_LIB_RESPONSE_TIMEOUT;
      _currentADU->callCallback();
      reset();
    }
  }
}

bool ClientItem::isValid() const {
  return _id != 0;
}

void ClientItem::send(ADUTCP* adu) {
  if (_client && _client->connected()) {
    _client->write(adu->_TXADUTCPframe, adu->getTXADULen());
    adu->_sentTime = millis();
  }
}

uint16_t ClientItem::clearBuffer() {
  uint16_t count = 0;
  while (_client->available()) {
    _client->read();
    count++;
  }
  return count;
}

void ClientItem::reset() {
  _currentADU = nullptr;
  _incomingByte = 0;
}