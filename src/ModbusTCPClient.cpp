#include "ModbusTCPClient.h"

#include "ADUTCP.h"
#include "ClientItem.h"

ModbusTCPClient::ModbusTCPClient() {}

ModbusTCPClient::~ModbusTCPClient() {
  if (_adu) {
    for (uint8_t i = 0; i < _ADUPoolSize; i++) {
      delete _adu[i];  // Free ADUTCP objects
    }
    delete[] _adu;  // Free the array
  }
  delete[] _clients;  // Free the client array
}

void ModbusTCPClient::begin(uint8_t ADUPoolSize, uint8_t PDUSize, uint8_t clientCount) {
  _ADUPoolSize = ADUPoolSize;
  _clientCount = clientCount;
  _adu = new ADUTCP*[_ADUPoolSize];
  for (size_t i = 0; i < _ADUPoolSize; i++) {
    _adu[i] = new ADUTCP();
    _adu[i]->init(PDUSize);
    _adu[i]->_modbusTCPClient = this;
  }
  _clients = new ClientItem[_clientCount];
  _responseTimeout = MB_RESPONSE_TIMEOUT;
  setIsBigEndian();
}

bool ModbusTCPClient::addClient(uint8_t id, bool allAtOnce, uint8_t queueSize, Client* client, IPAddress ip, uint16_t port, bool keepAlive) {
  // Check for unique slave ID
  for (uint8_t i = 0; i < _clientCount; ++i) {
    if (_clients[i].isValid() && _clients[i]._id == id) {
      return false;  // ID already exists
    }
  }
  // Find free slot
  for (uint8_t i = 0; i < _clientCount; ++i) {
    if (!_clients[i].isValid()) {
      _clients[i].set(id, allAtOnce, queueSize, client, ip, port, keepAlive);
      return true;
    }
  }
  return false;  // No free slot
}

bool ModbusTCPClient::sendPDU(PDU* pdu, uint8_t slave) {
  ADUTCP* adu = static_cast<ADUTCP*>(pdu);
  adu->setMBAP(slave);
  for (size_t i = 0; i < _clientCount; i++) {
    if (_clients[i]._id == slave) {
      if (!_clients[i]._queue.add(adu)) {
        adu->_err = MB_EX_LIB_QUEUE_FULL;
        adu->callCallback();
        return false;
      }
      return true;
    }
  }
  adu->_err = MB_EX_LIB_TCP_NO_CLIENT_AVAILABLE_FOR_THE_SLAVE;
  adu->callCallback();
  return false;
}

PDU* ModbusTCPClient::getFreePDU(const modbusCallback& cb, const Slaves& slaves) {
  for (uint8_t i = 0; i < _ADUPoolSize; ++i) {
    if (!_adu[i]->_used) {
      _adu[i]->_used = true;
      ADUTCP* adu = _adu[i];
      adu->_callback = cb;
      adu->_slaves = slaves;
      return adu;
    }
  }
  PDU ret;
  ret._err = MB_EX_LIB_NO_MORE_FREE_ADU;
  cb(ret);
  return nullptr;  // No free ADU
}

PDU* ModbusTCPClient::getFreePDU(const modbusCallback& cb, uint8_t slave) {
  for (uint8_t i = 0; i < _ADUPoolSize; ++i) {
    if (!_adu[i]->_used) {
      _adu[i]->_used = true;
      ADUTCP* adu = _adu[i];
      adu->_callback = cb;
      adu->_slaves.clear();
      adu->_slave = slave;
      return adu;
    }
  }
  PDU ret(slave);
  ret._err = MB_EX_LIB_NO_MORE_FREE_ADU;
  cb(ret);
  return nullptr;  // No free ADU
}

void ModbusTCPClient::loop() {
  for (uint8_t i = 0; i < _clientCount; ++i) {
    if (_clients[i].isValid()) {
      _clients[i].loop();  // Process only valid clients
    }
  }
}

uint32_t ModbusTCPClient::getResponseTimeout() const { return _responseTimeout; }
void ModbusTCPClient::setResponseTimeout(uint32_t t) { _responseTimeout = t; }