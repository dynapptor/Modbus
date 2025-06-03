#include "ADUTCP.h"

#include "ModbusTCPClient.h"

ADUTCP::ADUTCP() {}

uint16_t ADUTCP::_transactionId = 0;

void ADUTCP::init(uint8_t PDUSize) {
  _PDUSize = PDUSize;  // Set PDU size (16-253 bytes, excluding MBAP header)
  _TXADUTCPframe = new uint8_t[MB_ADU_MBAP_LEN + PDUSize];
  _RXADUTCPframe = new uint8_t[MB_ADU_MBAP_LEN + PDUSize];
  _TXPDUbuffer = _TXADUTCPframe + MB_ADU_MBAP_LEN;        // Offset for PDU data
  _RXPDUbuffer = _RXADUTCPframe + MB_ADU_MBAP_LEN;        // Offset for PDU data
  _PDUresponseHead = _responseTCPHead + MB_ADU_MBAP_LEN;  // Offset for response header
}

ADUTCP::~ADUTCP() {
  delete[] _TXADUTCPframe;
  delete[] _RXADUTCPframe;
}

uint16_t ADUTCP::getTransactionId() const {
  return (_TXADUTCPframe[0] << 8) | _TXADUTCPframe[1];
}

uint8_t ADUTCP::getId() const {
  return _TXADUTCPframe[6];
}

void ADUTCP::setMBAP(uint8_t slave) {
  _transactionId++;
  _TXADUTCPframe[0] = _responseTCPHead[0] = highByte(_transactionId);
  _TXADUTCPframe[1] = _responseTCPHead[1] = lowByte(_transactionId);
  _TXADUTCPframe[2] = _responseTCPHead[2] = 0x00;                 // Protocol ID
  _TXADUTCPframe[3] = _responseTCPHead[3] = 0x00;                 // Protocol ID
  _TXADUTCPframe[4] = _responseTCPHead[4] = 0x00;                 // Length (high byte)
  _TXADUTCPframe[5] = _responseTCPHead[5] = _TXPDUbufferLen + 1;  // Length (low byte, PDU + unit ID)
  _TXADUTCPframe[6] = _responseTCPHead[6] = slave;                // Unit ID (slave ID)
}

bool ADUTCP::checkResponseMBAP() {
  if (_RXADUTCPframe[0] != _responseTCPHead[0] || _RXADUTCPframe[1] != _responseTCPHead[1]) {
    _err = MB_EX_LIB_INVALID_MBAP_TRANSACTION_ID;
    callCallback();
    return false;
  }
  if (_RXADUTCPframe[2] != _responseTCPHead[2] || _RXADUTCPframe[3] != _responseTCPHead[3]) {
    _err = MB_EX_LIB_INVALID_MBAP_PROTOCOL_ID;
    callCallback();
    return false;
  }
  if (_RXADUTCPframe[6] != _responseTCPHead[6]) {
    _err = MB_EX_LIB_INVALID_MBAP_UNIT_ID;
    callCallback();
    return false;
  }
  return true;
}

uint16_t ADUTCP::getTXADULen() const {
  return MB_ADU_MBAP_LEN + _TXPDUbufferLen;
}

uint16_t ADUTCP::getExpectedResponseLen() const {
  return (uint16_t)_expectedResponseLen + MB_ADU_MBAP_LEN;
}

bool ADUTCP::repeatIfNeeded() {
  if (!_slaves.valid()) return false;
  uint8_t prev = _slaves.getActive();
  uint8_t next = _slaves.getNext();
  if (next != SLAVE_EOF && next != SLAVE_NULL) {
    _queuedTime = millis();
    if (prev > next || prev == next) {
      _delayToSend = _slaves.getRepeatDelay();
    } else {
      _delayToSend = _slaves.getDelay();
    }
    // _modbusTCPClient is initialized later by ModbusTCPClient for repeat logic
    return (_modbusTCPClient) && _modbusTCPClient->sendPDU(this, next);
  }
  return false;
}