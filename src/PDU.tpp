#pragma once
#include "ModbusDef.h"
#include "ModbusUtility.h"
#include "PDU.h"

template <typename T>
uint16_t PDU::createWriteHoldingRegister(uint16_t addr, const T* src, uint8_t count, const modbusCallback& cb) {
  _callback = cb;
  constexpr uint8_t elemSize = sizeof(T);
  constexpr uint8_t paddedSize = (elemSize % 2 == 0) ? elemSize : elemSize + 1;
  const uint16_t totalBytes = count * paddedSize;
  const uint16_t regCount = totalBytes / 2;
  if (count == 0) return _err = MB_EX_LIB_TOO_FEW_DATA;
  if (regCount > MB_MAX_WRITE_REGISTERS) return _err = MB_EX_LIB_TOO_MANY_DATA;
  if (_PDUSize < 6 + totalBytes) return _err = MB_EX_LIB_BUFFER_IS_TOO_SMALL;
  _TXPDUbuffer[0] = _PDUresponseHead[0] = MB_FC_WRITE_MULTIPLE_REGISTERS;
  _TXPDUbuffer[1] = _PDUresponseHead[1] = highByte(addr);
  _TXPDUbuffer[2] = _PDUresponseHead[2] = lowByte(addr);
  _TXPDUbuffer[3] = _PDUresponseHead[3] = highByte(regCount);
  _TXPDUbuffer[4] = _PDUresponseHead[4] = lowByte(regCount);
  _TXPDUbuffer[5] = totalBytes;
  const uint8_t* srcBytes = reinterpret_cast<const uint8_t*>(src);
  uint8_t* dst = _TXPDUbuffer + 6;
  if (!convertToBigEndianRegisters(srcBytes, count, elemSize, dst, _PDUSize - 6)) {
    _err = MB_EX_LIB_INVALID_DATA;
    return _err;
  }
  _TXPDUbufferLen = 6 + totalBytes;
  _expectedResponseLen = 5;
  return MB_EX_SUCCESS;
}

template <typename READ_T, typename WRITE_T>
uint16_t PDU::createReadWriteMultipleRegisters(uint16_t readAddr, uint8_t readCount, uint16_t writeAddr, const WRITE_T* writeData, uint16_t writeCount, const modbusCallback& cb) {
  _callback = cb;
  _elemSize = sizeof(READ_T);  // Used for response data decompression
  constexpr uint8_t writeElemSize = sizeof(WRITE_T);
  constexpr uint8_t paddedSize = (writeElemSize % 2 == 0) ? writeElemSize : writeElemSize + 1;
  const uint16_t totalWriteBytes = writeCount * paddedSize;
  const uint16_t writeRegCount = totalWriteBytes / 2;
  const uint16_t readByteCount = readCount * _elemSize;
  const uint16_t totalLen = 10 + totalWriteBytes;
  if (readCount == 0 || writeCount == 0) return _err = MB_EX_LIB_TOO_FEW_DATA;
  if (readCount > MB_MAX_READ_REGISTERS) return _err = MB_EX_LIB_TOO_MANY_DATA;
  if (writeRegCount > MB_MAX_WRITE_READ_REGISTERS) return _err = MB_EX_LIB_TOO_MANY_DATA;
  if (_PDUSize < totalLen) return _err = MB_EX_LIB_BUFFER_IS_TOO_SMALL;
  _TXPDUbuffer[0] = _PDUresponseHead[0] = MB_FC_READ_AND_WRITE_REGISTERS;
  _TXPDUbuffer[1] = highByte(readAddr);
  _TXPDUbuffer[2] = lowByte(readAddr);
  _TXPDUbuffer[3] = highByte(readCount);
  _TXPDUbuffer[4] = lowByte(readCount);
  _TXPDUbuffer[5] = highByte(writeAddr);
  _TXPDUbuffer[6] = lowByte(writeAddr);
  _TXPDUbuffer[7] = highByte(writeRegCount);
  _TXPDUbuffer[8] = lowByte(writeRegCount);
  _TXPDUbuffer[9] = totalWriteBytes;
  _PDUresponseHead[1] = readByteCount;
  const uint8_t* srcBytes = reinterpret_cast<const uint8_t*>(writeData);
  uint8_t* dst = _TXPDUbuffer + 10;
  if (!convertToBigEndianRegisters(srcBytes, writeCount, writeElemSize, dst, _PDUSize - 10)) {
    _err = MB_EX_LIB_INVALID_DATA;
    return _err;
  }
  _TXPDUbufferLen = totalLen;
  _expectedResponseLen = 2 + readByteCount;
  return MB_EX_SUCCESS;
}

template <typename T>
uint16_t PDU::createReadRegisters(uint8_t fn, uint16_t addr, uint8_t count, const modbusCallback& cb) {
  _callback = cb;
  _elemSize = sizeof(T);
  if (count == 0) {
    _err = MB_EX_LIB_TOO_FEW_DATA;
    return _err;
  }
  uint8_t regCount = count * toRegisterCount(_elemSize);
  uint16_t byteCount = regCount * 2;
  if (regCount > MB_MAX_READ_REGISTERS) {
    _err = MB_EX_LIB_TOO_MANY_DATA;
    return _err;
  }
  if ((_PDUSize < 5) || (_PDUSize < 2 + byteCount)) {
    _err = MB_EX_LIB_BUFFER_IS_TOO_SMALL;
    return _err;
  }
  _TXPDUbuffer[0] = _PDUresponseHead[0] = fn;
  _TXPDUbuffer[1] = highByte(addr);
  _TXPDUbuffer[2] = lowByte(addr);
  _TXPDUbuffer[3] = highByte(regCount);
  _TXPDUbuffer[4] = lowByte(regCount);
  _TXPDUbufferLen = 5;
  _PDUresponseHead[1] = byteCount;
  _expectedResponseLen = 2 + byteCount;
  return MB_EX_SUCCESS;
}

template <typename T>
const T PDU::getData(uint16_t ix) const {
  static const T dummy{};  // Default zero-initialized value
  const uint16_t offset = _dataBegin + (ix * sizeof(T));
  if (offset + sizeof(T) > _PDUSize || ix >= (_dataLen / sizeof(T))) {
    return dummy;
  }
  return reinterpret_cast<const T*>(&_RXPDUbuffer[_dataBegin])[ix];
}

template <typename T>
const T* PDU::getDataArray() const {
  static_assert(sizeof(T) > 0, "Invalid type size");
  if (!_RXPDUbuffer) return nullptr;
  return reinterpret_cast<const T*>(_RXPDUbuffer + _dataBegin);
}

template <typename T>
uint8_t PDU::getLen() const {
  return _dataLen / sizeof(T);
}