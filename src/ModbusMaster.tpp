#pragma once
#include <Callback.h>

#include "ModbusDef.h"
#include "ModbusMaster.h"
#include "PDU.h"

template <typename READ_T, typename WRITE_T>
void ModbusMaster::readWriteMultipleRegisters(const Slaves& slaves, uint16_t readAddr, uint8_t readCount,
                                              uint16_t writeAddr, const WRITE_T* writeData, uint16_t writeCount, const modbusCallback& cb) {
  PDU* pdu = getFreePDU(cb, slaves);
  if (!pdu) return;
  if (pdu->createReadWriteMultipleRegisters<READ_T, WRITE_T>(readAddr, readCount, writeAddr, writeData, writeCount, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slaves.getActive());
}

template <typename READ_T, typename WRITE_T>
void ModbusMaster::readWriteMultipleRegisters(uint8_t slave, uint16_t readAddr, uint8_t readCount,
                                              uint16_t writeAddr, const WRITE_T* writeData, uint16_t writeCount, const modbusCallback& cb) {
  if (slave == 0) {
    PDU ret(slave);
    ret._err = MB_EX_LIB_INVALID_SLAVE;
    cb(ret);
    return;
  }
  PDU* pdu = getFreePDU(cb, slave);
  if (!pdu) return;
  if (pdu->createReadWriteMultipleRegisters<READ_T, WRITE_T>(readAddr, readCount, writeAddr, writeData, writeCount, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slave);
}

template <typename T>
void ModbusMaster::writeHoldingRegister(const Slaves& slaves, uint16_t addr, T& src, const modbusCallback& cb) {
  PDU* pdu = getFreePDU(cb, slaves);
  if (!pdu) return;
  if (pdu->createWriteHoldingRegister(addr, &src, 1, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slaves.getActive());
}

template <typename T>
void ModbusMaster::writeHoldingRegister(uint8_t slave, uint16_t addr, T& src, const modbusCallback& cb) {
  if (slave == 0 && !isWriteFunction(MB_FC_WRITE_MULTIPLE_REGISTERS)) {
    PDU ret(slave);
    ret._err = MB_EX_LIB_INVALID_SLAVE;
    cb(ret);
    return;
  }
  PDU* pdu = getFreePDU(cb, slave);
  if (!pdu) return;
  if (pdu->createWriteHoldingRegister(addr, &src, 1, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slave);
}

template <typename T>
void ModbusMaster::writeHoldingRegisters(const Slaves& slaves, uint16_t addr, const T* src, uint16_t srcCount, const modbusCallback& cb) {
  PDU* pdu = getFreePDU(cb, slaves);
  if (!pdu) return;
  if (pdu->createWriteHoldingRegister(addr, src, srcCount, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slaves.getActive());
}

template <typename T>
void ModbusMaster::writeHoldingRegisters(uint8_t slave, uint16_t addr, const T* src, uint16_t srcCount, const modbusCallback& cb) {
  if (slave == 0 && !isWriteFunction(MB_FC_WRITE_MULTIPLE_REGISTERS)) {
    PDU ret(slave);
    ret._err = MB_EX_LIB_INVALID_SLAVE;
    cb(ret);
    return;
  }
  PDU* pdu = getFreePDU(cb, slave);
  if (!pdu) return;
  if (pdu->createWriteHoldingRegister(addr, src, srcCount, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slave);
}

template <typename T>
void ModbusMaster::writeHoldingRegisters(const Slaves& slaves, uint16_t addr, std::initializer_list<T> list, const modbusCallback& cb) {
  PDU* pdu = getFreePDU(cb, slaves);
  if (!pdu) return;
  uint16_t count = list.size();
  T values[count];
  uint16_t i = 0;
  for (const T& value : list) {
    values[i++] = value;
  }
  if (pdu->createWriteHoldingRegister(addr, values, count, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slaves.getActive());
}

template <typename T>
void ModbusMaster::writeHoldingRegisters(uint8_t slave, uint16_t addr, std::initializer_list<T> list, const modbusCallback& cb) {
  if (slave == 0 && !isWriteFunction(MB_FC_WRITE_MULTIPLE_REGISTERS)) {
    PDU ret(slave);
    ret._err = MB_EX_LIB_INVALID_SLAVE;
    cb(ret);
    return;
  }
  PDU* pdu = getFreePDU(cb, slave);
  if (!pdu) return;
  uint16_t count = list.size();
  T values[count];
  uint16_t i = 0;
  for (const T& value : list) {
    values[i++] = value;
  }
  if (pdu->createWriteHoldingRegister(addr, values, count, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slave);
}

template <typename T>
void ModbusMaster::readHoldingRegister(const Slaves& slaves, uint16_t addr, const modbusCallback& cb) {
  PDU* pdu = getFreePDU(cb, slaves);
  if (!pdu) return;
  if (pdu->createReadRegisters<T>(MB_FC_READ_HOLDING_REGISTERS, addr, 1, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slaves.getActive());
}

template <typename T>
void ModbusMaster::readHoldingRegister(uint8_t slave, uint16_t addr, const modbusCallback& cb) {
  if (slave == 0) {
    PDU ret(slave);
    ret._err = MB_EX_LIB_INVALID_SLAVE;
    cb(ret);
    return;
  }
  PDU* pdu = getFreePDU(cb, slave);
  if (!pdu) return;
  if (pdu->createReadRegisters<T>(MB_FC_READ_HOLDING_REGISTERS, addr, 1, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slave);
}

template <typename T>
void ModbusMaster::readHoldingRegisters(const Slaves& slaves, uint16_t addr, uint8_t count, const modbusCallback& cb) {
  PDU* pdu = getFreePDU(cb, slaves);
  if (!pdu) return;
  if (pdu->createReadRegisters<T>(MB_FC_READ_HOLDING_REGISTERS, addr, count, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slaves.getActive());
}

template <typename T>
void ModbusMaster::readHoldingRegisters(uint8_t slave, uint16_t addr, uint8_t count, const modbusCallback& cb) {
  if (slave == 0) {
    PDU ret(slave);
    ret._err = MB_EX_LIB_INVALID_SLAVE;
    cb(ret);
    return;
  }
  PDU* pdu = getFreePDU(cb, slave);
  if (!pdu) return;
  if (pdu->createReadRegisters<T>(MB_FC_READ_HOLDING_REGISTERS, addr, count, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slave);
}

template <typename T>
void ModbusMaster::readInputRegister(const Slaves& slaves, uint16_t addr, const modbusCallback& cb) {
  PDU* pdu = getFreePDU(cb, slaves);
  if (!pdu) return;
  if (pdu->createReadRegisters<T>(MB_FC_READ_INPUT_REGISTERS, addr, 1, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slaves.getActive());
}

template <typename T>
void ModbusMaster::readInputRegister(uint8_t slave, uint16_t addr, const modbusCallback& cb) {
  if (slave == 0) {
    PDU ret(slave);
    ret._err = MB_EX_LIB_INVALID_SLAVE;
    cb(ret);
    return;
  }
  PDU* pdu = getFreePDU(cb, slave);
  if (!pdu) return;
  if (pdu->createReadRegisters<T>(MB_FC_READ_INPUT_REGISTERS, addr, 1, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slave);
}

template <typename T>
void ModbusMaster::readInputRegisters(const Slaves& slaves, uint16_t addr, uint8_t count, const modbusCallback& cb) {
  PDU* pdu = getFreePDU(cb, slaves);
  if (!pdu) return;
  if (pdu->createReadRegisters<T>(MB_FC_READ_INPUT_REGISTERS, addr, count, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slaves.getActive());
}

template <typename T>
void ModbusMaster::readInputRegisters(uint8_t slave, uint16_t addr, uint8_t count, const modbusCallback& cb) {
  if (slave == 0) {
    PDU ret(slave);
    ret._err = MB_EX_LIB_INVALID_SLAVE;
    cb(ret);
    return;
  }
  PDU* pdu = getFreePDU(cb, slave);
  if (!pdu) return;
  if (pdu->createReadRegisters<T>(MB_FC_READ_INPUT_REGISTERS, addr, count, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slave);
}