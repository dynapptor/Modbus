#include "PDU.h"

#include "ModbusDef.h"
#include "ModbusUtility.h"

PDU::PDU() : _PDUSize(0) {}

PDU::PDU(uint8_t slave) { _slave = slave; }

PDU::~PDU() {}

void PDU::callCallback() {
  if (_callback.valid()) {
    _callback(*this);
    if (!repeatIfNeeded()) {
      clear();
    }
  }
}

bool PDU::repeatIfNeeded() { return false; }

uint16_t PDU::invoke() {
  if (_err != 0) {
    _dataBegin = 0;
    _dataLen = 0;
    callCallback();
    return _err;
  }
  _err = 0;
  // Check for exception response (function code + 0x80)
  if (_RXPDUbuffer[0] == _PDUresponseHead[0] + 0x80) {
    _err = _RXPDUbuffer[1];
    _dataBegin = 0;
    _dataLen = 0;
    callCallback();
    return _err;
  }
  // Validate function code
  if (_RXPDUbuffer[0] != _PDUresponseHead[0]) {
    _err = MB_EX_LIB_INVALID_FUNCTION;
    _dataBegin = 0;
    _dataLen = 0;
    callCallback();
    return _err;
  }
  switch (_RXPDUbuffer[0]) {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUTS:
    case MB_FC_READ_HOLDING_REGISTERS:
    case MB_FC_READ_INPUT_REGISTERS:
    case MB_FC_READ_AND_WRITE_REGISTERS: {
      // Validate byte count
      if (_RXPDUbuffer[1] != _PDUresponseHead[1]) {
        _err = MB_EX_LIB_INVALID_BYTE_LENGTH;
        callCallback();
        return _err;
      }
      _dataBegin = 2;
      _dataLen = _RXPDUbuffer[1];
      // Perform endian conversion for register-based responses
      if (_elemSize > 0 && (_dataLen % 2 == 0)) {
        const uint8_t elemCount = _dataLen / _elemSize;
        convertFromBigEndianRegistersInPlace(_RXPDUbuffer + _dataBegin, elemCount, _elemSize);
      }
      callCallback();
      return _err;
    }
    case MB_FC_WRITE_SINGLE_COIL:
    case MB_FC_WRITE_SINGLE_REGISTER: {
      // Validate address and data
      if (_RXPDUbuffer[1] != _PDUresponseHead[1] || _RXPDUbuffer[2] != _PDUresponseHead[2]) {
        _err = MB_EX_LIB_INVALID_ADDRESS;
        callCallback();
        return _err;
      }
      if (_RXPDUbuffer[3] != _PDUresponseHead[3] || _RXPDUbuffer[4] != _PDUresponseHead[4]) {
        _err = MB_EX_LIB_INVALID_DATA;
        callCallback();
        return _err;
      }
      callCallback();
      break;
    }
    case MB_FC_READ_EXCEPTION_STATUS: {
      _dataBegin = 1;
      _dataLen = 1;
      callCallback();
      break;
    }
    case MB_FC_DIAGNOSTICS: {
      // Validate subfunction code
      if (_RXPDUbuffer[1] != _PDUresponseHead[1] || _RXPDUbuffer[2] != _PDUresponseHead[2]) {
        _err = MB_EX_LIB_INVALID_SUB_FUNCTION;
        callCallback();
        return _err;
      }
      _dataBegin = 3;
      _dataLen = 2;
      callCallback();
      break;
    }
    case MB_FC_WRITE_MULTIPLE_COILS:
    case MB_FC_WRITE_MULTIPLE_REGISTERS: {
      // Validate address and quantity
      if (_RXPDUbuffer[1] != _PDUresponseHead[1] || _RXPDUbuffer[2] != _PDUresponseHead[2]) {
        _err = MB_EX_LIB_INVALID_ADDRESS;
        callCallback();
        return _err;
      }
      if (_RXPDUbuffer[3] != _PDUresponseHead[3] || _RXPDUbuffer[4] != _PDUresponseHead[4]) {
        _err = MB_EX_LIB_INVALID_BYTE_LENGTH;
        callCallback();
        return _err;
      }
      callCallback();
      break;
    }
    case MB_FC_MASK_WRITE_REGISTER: {
      // Validate address and masks
      if (_RXPDUbuffer[1] != _PDUresponseHead[1] || _RXPDUbuffer[2] != _PDUresponseHead[2]) {
        _err = MB_EX_LIB_INVALID_ADDRESS;
        callCallback();
        return _err;
      }
      if (_RXPDUbuffer[3] != _PDUresponseHead[3] || _RXPDUbuffer[4] != _PDUresponseHead[4] ||
          _RXPDUbuffer[5] != _PDUresponseHead[5] || _RXPDUbuffer[6] != _PDUresponseHead[6]) {
        _err = MB_EX_LIB_INVALID_DATA;
        callCallback();
        return _err;
      }
      callCallback();
      break;
    }
    default:
      _err = MB_EX_LIB_NOT_SUPPORTED;
      callCallback();
      break;
  }
  return _err;
}

uint16_t PDU::createWriteSingleCoil(uint16_t addr, bool value, const modbusCallback& cb) {
  _callback = cb;
  if (_PDUSize < 5) {
    _err = MB_EX_LIB_BUFFER_IS_TOO_SMALL;
    return _err;
  }
  _TXPDUbuffer[0] = _PDUresponseHead[0] = MB_FC_WRITE_SINGLE_COIL;
  _TXPDUbuffer[1] = _PDUresponseHead[1] = highByte(addr);
  _TXPDUbuffer[2] = _PDUresponseHead[2] = lowByte(addr);
  _TXPDUbuffer[3] = _PDUresponseHead[3] = value ? 0xFF : 0x00;
  _TXPDUbuffer[4] = _PDUresponseHead[4] = 0x00;
  _TXPDUbufferLen = 5;
  _expectedResponseLen = 5;
  return MB_EX_SUCCESS;
}

uint16_t PDU::createWriteSingleRegister(uint16_t addr, uint16_t value, const modbusCallback& cb) {
  _callback = cb;
  if (_PDUSize < 5) {
    _err = MB_EX_LIB_BUFFER_IS_TOO_SMALL;
    return _err;
  }
  _TXPDUbuffer[0] = _PDUresponseHead[0] = MB_FC_WRITE_SINGLE_REGISTER;
  _TXPDUbuffer[1] = _PDUresponseHead[1] = highByte(addr);
  _TXPDUbuffer[2] = _PDUresponseHead[2] = lowByte(addr);
  _TXPDUbuffer[3] = _PDUresponseHead[3] = highByte(value);
  _TXPDUbuffer[4] = _PDUresponseHead[4] = lowByte(value);
  _TXPDUbufferLen = 5;
  _expectedResponseLen = 5;
  return MB_EX_SUCCESS;
}

uint16_t PDU::createWriteMultipleCoils(uint16_t addr, const uint8_t* src, uint8_t byteCount, uint16_t coilCount, const modbusCallback& cb) {
  _callback = cb;
  if (byteCount == 0) {
    _err = MB_EX_LIB_TOO_FEW_DATA;
    return _err;
  }
  if (byteCount > MB_MAX_WRITE_COILS_IN_BYTES) {
    _err = MB_EX_LIB_TOO_MANY_DATA;
    return _err;
  }
  if (_PDUSize < 6 + byteCount) {
    _err = MB_EX_LIB_BUFFER_IS_TOO_SMALL;
    return _err;
  }
  _TXPDUbuffer[0] = _PDUresponseHead[0] = MB_FC_WRITE_MULTIPLE_COILS;
  _TXPDUbuffer[1] = _PDUresponseHead[1] = highByte(addr);
  _TXPDUbuffer[2] = _PDUresponseHead[2] = lowByte(addr);
  _TXPDUbuffer[3] = _PDUresponseHead[3] = highByte(coilCount);
  _TXPDUbuffer[4] = _PDUresponseHead[4] = lowByte(coilCount);
  _TXPDUbuffer[5] = byteCount;
  memcpy(_TXPDUbuffer + 6, src, byteCount);
  _TXPDUbufferLen = 6 + byteCount;
  _expectedResponseLen = 5;
  return MB_EX_SUCCESS;
}

uint16_t PDU::createWriteMultipleCoils(uint16_t addr, const bool* src, uint16_t coilCount, const modbusCallback& cb) {
  _callback = cb;
  if (coilCount == 0) {
    _err = MB_EX_LIB_TOO_FEW_DATA;
    return _err;
  }
  if (coilCount > MB_MAX_WRITE_COILS) {
    _err = MB_EX_LIB_TOO_MANY_DATA;
    return _err;
  }
  uint8_t byteCount = (coilCount + 7) / 8;
  if (byteCount > MB_MAX_WRITE_COILS_IN_BYTES) {
    _err = MB_EX_LIB_TOO_MANY_DATA;
    return _err;
  }
  if (_PDUSize < 6 + byteCount) {
    _err = MB_EX_LIB_BUFFER_IS_TOO_SMALL;
    return _err;
  }
  _TXPDUbuffer[0] = _PDUresponseHead[0] = MB_FC_WRITE_MULTIPLE_COILS;
  _TXPDUbuffer[1] = _PDUresponseHead[1] = highByte(addr);
  _TXPDUbuffer[2] = _PDUresponseHead[2] = lowByte(addr);
  _TXPDUbuffer[3] = _PDUresponseHead[3] = highByte(coilCount);
  _TXPDUbuffer[4] = _PDUresponseHead[4] = lowByte(coilCount);
  _TXPDUbuffer[5] = byteCount;
  memset(_TXPDUbuffer + 6, 0, byteCount);  // Clear buffer to avoid residual data
  for (uint16_t i = 0; i < coilCount; i++) {
    if (src[i]) {
      _TXPDUbuffer[6 + (i / 8)] |= (1 << (i % 8));
    }
  }
  _TXPDUbufferLen = 6 + byteCount;
  _expectedResponseLen = 5;
  return MB_EX_SUCCESS;
}

uint16_t PDU::createMaskWriteRegister(uint16_t addr, uint16_t andMask, uint16_t orMask, const modbusCallback& cb) {
  _callback = cb;
  if (_PDUSize < 7) {
    _err = MB_EX_LIB_BUFFER_IS_TOO_SMALL;
    return _err;
  }
  _TXPDUbuffer[0] = _PDUresponseHead[0] = MB_FC_MASK_WRITE_REGISTER;
  _TXPDUbuffer[1] = _PDUresponseHead[1] = highByte(addr);
  _TXPDUbuffer[2] = _PDUresponseHead[2] = lowByte(addr);
  _TXPDUbuffer[3] = _PDUresponseHead[3] = highByte(andMask);
  _TXPDUbuffer[4] = _PDUresponseHead[4] = lowByte(andMask);
  _TXPDUbuffer[5] = _PDUresponseHead[5] = highByte(orMask);
  _TXPDUbuffer[6] = _PDUresponseHead[6] = lowByte(orMask);
  _TXPDUbufferLen = 7;
  _expectedResponseLen = 7;
  return MB_EX_SUCCESS;
}

uint16_t PDU::createReadExceptionStatus(const modbusCallback& cb) {
  _callback = cb;
  if (_PDUSize < 2) {
    _err = MB_EX_LIB_BUFFER_IS_TOO_SMALL;
    return _err;
  }
  _TXPDUbuffer[0] = _PDUresponseHead[0] = MB_FC_READ_EXCEPTION_STATUS;
  _TXPDUbufferLen = 1;
  _expectedResponseLen = 2;
  return MB_EX_SUCCESS;
}

uint16_t PDU::createDiagnostics(uint16_t subFunction, uint16_t value, const modbusCallback& cb) {
  _callback = cb;
  if (subFunction > MB_FC_SUB_CLEAR_OVERRUN_CHARACTER_AND_FLAG ||
      (subFunction > MB_FC_SUB_FORCE_LISTEN_ONLY_MODE && subFunction < MB_FC_SUB_CLEAR_COUNTERS_AND_DIAGNOSTIC_REGISTER)) {
    _err = MB_EX_LIB_INVALID_SUB_FUNCTION;
    return _err;
  }
  if (_PDUSize < 5) {
    _err = MB_EX_LIB_BUFFER_IS_TOO_SMALL;
    return _err;
  }
  _TXPDUbuffer[0] = _PDUresponseHead[0] = MB_FC_DIAGNOSTICS;
  _TXPDUbuffer[1] = _PDUresponseHead[1] = highByte(subFunction);
  _TXPDUbuffer[2] = _PDUresponseHead[2] = lowByte(subFunction);
  _TXPDUbuffer[3] = _PDUresponseHead[3] = highByte(value);
  _TXPDUbuffer[4] = _PDUresponseHead[4] = lowByte(value);
  _TXPDUbufferLen = 5;
  _expectedResponseLen = 5;
  return MB_EX_SUCCESS;
}

uint16_t PDU::createReadState(uint8_t fn, uint16_t addr, uint16_t count, const modbusCallback& cb) {
  _callback = cb;
  if (count == 0) {
    _err = MB_EX_LIB_TOO_FEW_DATA;
    return _err;
  }
  if (count > MB_MAX_READ_COILS) {
    _err = MB_EX_LIB_TOO_MANY_DATA;
    return _err;
  }
  if ((_PDUSize < 5) || (_PDUSize < 2 + ((count + 7) / 8))) {
    _err = MB_EX_LIB_BUFFER_IS_TOO_SMALL;
    return _err;
  }
  _TXPDUbuffer[0] = _PDUresponseHead[0] = fn;
  _TXPDUbuffer[1] = highByte(addr);
  _TXPDUbuffer[2] = lowByte(addr);
  _TXPDUbuffer[3] = highByte(count);
  _TXPDUbuffer[4] = lowByte(count);
  _TXPDUbufferLen = 5;
  _PDUresponseHead[1] = (count + 7) / 8;
  _expectedResponseLen = 2 + ((count + 7) / 8);
  return MB_EX_SUCCESS;
}

void PDU::clear() {
  _callback.clear();
  _TXPDUbufferLen = 0;
  _dataBegin = 0;
  _dataLen = 0;
  _err = 0;
  _expectedResponseLen = 0;
  _elemSize = 0;
  _used = false;
  _delayToSend = 0;
  _queuedTime = 0;
  _slave = 0;
}

uint16_t PDU::getErr() const { return _err; }

uint16_t PDU::toBigEndian(uint16_t src) {
  if (isBigEndian) return src;
  return ((src & 0xFF) << 8) | ((src >> 8) & 0xFF);
}

bool PDU::convertToBigEndianRegisters(const uint8_t* src, uint8_t elemCount, uint8_t elemSize, uint8_t* dest, uint16_t destLen) {
  if (!src || !dest || elemSize == 0) return false;
  const uint8_t paddedSize = (elemSize % 2 == 0) ? elemSize : elemSize + 1;
  const uint16_t paddedTotal = elemCount * paddedSize;
  if (destLen < paddedTotal) return false;
  if (isBigEndian) {
    for (uint8_t i = 0; i < elemCount; ++i) {
      memcpy(dest + i * paddedSize, src + i * elemSize, elemSize);
      if (paddedSize > elemSize) dest[i * paddedSize + paddedSize - 1] = 0x00;  // Add padding
    }
    return true;
  }
  // Little-endian: swap bytes and add padding
  for (uint8_t i = 0; i < elemCount; ++i) {
    const uint8_t* p = src + i * elemSize;
    uint8_t* d = dest + i * paddedSize;
    for (uint8_t j = 0; j < paddedSize; j += 2) {
      const uint8_t lo = (j < elemSize) ? p[j] : 0x00;
      const uint8_t hi = (j + 1 < elemSize) ? p[j + 1] : 0x00;
      d[j] = hi;
      d[j + 1] = lo;
    }
  }
  return true;
}

uint8_t PDU::toRegisterCount(uint8_t byteCount) { return (byteCount + 1) / 2; }

bool PDU::convertFromBigEndianRegistersInPlace(uint8_t* buffer, uint8_t elemCount, uint8_t elemSize) {
  if (!buffer || elemSize == 0) return false;
  const uint8_t paddedSize = (elemSize % 2 == 0) ? elemSize : elemSize + 1;
  const uint16_t srcTotal = elemCount * paddedSize;
  const uint16_t destTotal = elemCount * elemSize;
  if (srcTotal > _PDUSize || destTotal > _PDUSize) return false;
  // Temporary buffer for one element (assumes elemSize <= _PDUSize / elemCount)
  uint8_t temp[32];  // Reasonable limit based on Modbus constraints
  if (elemSize > sizeof(temp)) return false;
  for (uint8_t i = 0; i < elemCount; ++i) {
    const uint8_t* src = buffer + i * paddedSize;
    for (uint8_t j = 0; j < paddedSize; j += 2) {
      const uint8_t hi = src[j];
      const uint8_t lo = src[j + 1];
      const uint16_t reg = (hi << 8) | lo;
      const uint8_t loByte = isBigEndian ? lo : lowByte(reg);
      const uint8_t hiByte = isBigEndian ? hi : highByte(reg);
      if (j < elemSize) temp[j] = loByte;
      if (j + 1 < elemSize) temp[j + 1] = hiByte;
    }
    memcpy(buffer + i * elemSize, temp, elemSize);  // Copy back without padding
  }
  // Adjust _dataLen to reflect the actual data size after stripping padding
  _dataLen = elemCount * elemSize;
  return true;
}

bool PDU::isUsed() const { return _used; }

void PDU::setUsed(bool v) { _used = v; }

bool PDU::getBit(uint16_t ix) const {
  if (ix >= _dataLen * 8) return false;  // Out-of-bounds
  const uint8_t byteIndex = _dataBegin + (ix / 8);
  const uint8_t bitIndex = ix % 8;
  return (_RXPDUbuffer[byteIndex] >> bitIndex) & 0x01;
}

uint8_t PDU::getSlaveId() const {
  return 0xFF;
}

uint8_t PDU::getFunction() const { return _RXPDUbuffer[0]; }

uint8_t PDU::getByteLen() const { return _dataLen; }
