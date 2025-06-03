#include "ADURTU.h"

#include "Crc16.h"
#include "ModbusRTUMaster.h"

ADURTU::ADURTU() {}

ADURTU::~ADURTU() {
  delete[] _TXADURTUframe;
  delete[] _RXADURTUframe;
}

void ADURTU::clear() {
  _responseLen = 0;
  PDU::clear();
}

void ADURTU::init(uint8_t PDUSize) {
  _PDUSize = PDUSize;  // Set PDU size (16-253 bytes, excluding RTU header and CRC)
  _TXADURTUframe = new uint8_t[MB_ADU_RTU_HEADER_LEN + PDUSize + MB_ADU_RTU_CRC_LEN];
  _RXADURTUframe = new uint8_t[MB_ADU_RTU_HEADER_LEN + PDUSize + MB_ADU_RTU_CRC_LEN];
  _TXPDUbuffer = _TXADURTUframe + MB_ADU_RTU_HEADER_LEN;        // Offset for PDU data
  _RXPDUbuffer = _RXADURTUframe + MB_ADU_RTU_HEADER_LEN;        // Offset for PDU data
  _PDUresponseHead = _responseRTUHead + MB_ADU_RTU_HEADER_LEN;  // Offset for response header
}

void ADURTU::setHead(uint8_t slave) {
  _TXADURTUframe[0] = _responseRTUHead[0] = slave;
}

void ADURTU::setCRC() {
  crc16Set(_TXADURTUframe, _TXPDUbufferLen + MB_ADU_RTU_HEADER_LEN);
}

bool ADURTU::checkResponseHead() {
  if (_RXADURTUframe[0] == _responseRTUHead[0]) {
    return true;
  }
  _err = MB_EX_LIB_INVALID_SLAVE;
  callCallback();
  return false;
}

bool ADURTU::checkResponseCRC() {
  if (crc16Check(_RXADURTUframe, _responseLen)) {
    return true;
  }
  _err = MB_EX_LIB_CRC;
  callCallback();
  return false;
}

uint16_t ADURTU::getTXADULen() const {
  return MB_ADU_RTU_HEADER_LEN + _TXPDUbufferLen + MB_ADU_RTU_CRC_LEN;
}

uint16_t ADURTU::getExpectedResponseLen() const {
  return (uint16_t)_expectedResponseLen + MB_ADU_RTU_HEADER_LEN + MB_ADU_RTU_CRC_LEN;
}

bool ADURTU::repeatIfNeeded() {
  if (!_slaves.valid()) return false;
  uint8_t prev = _slaves.getActive();
  uint8_t next = _slaves.getNext();
  if (next != SLAVE_EOF && next != SLAVE_NULL) {
    _queuedTime = millis();
    _delayToSend = (prev > next || prev == next) ? _slaves.getRepeatDelay() : _slaves.getDelay();
    return _modbusRTUMaster && _modbusRTUMaster->sendPDU(this, next);
  }
  return false;
}

uint8_t ADURTU::getSlaveId() const {
  return _TXADURTUframe[0];
}