#include "ModbusMaster.h"

#include <Callback.h>

#include "ModbusDef.h"
#include "PDU.h"

ModbusMaster::ModbusMaster() {}

ModbusMaster::~ModbusMaster() {}

bool ModbusMaster::isWriteFunction(uint8_t functionCode) const {
  return functionCode == MB_FC_WRITE_SINGLE_COIL ||
         functionCode == MB_FC_WRITE_SINGLE_REGISTER ||
         functionCode == MB_FC_WRITE_MULTIPLE_COILS ||
         functionCode == MB_FC_WRITE_MULTIPLE_REGISTERS ||
         functionCode == MB_FC_MASK_WRITE_REGISTER;
}

void ModbusMaster::writeSingleCoil(const Slaves& slaves, uint16_t address, bool value, const modbusCallback& cb) {
  PDU* pdu = getFreePDU(cb, slaves);
  if (!pdu) return;
  if (pdu->createWriteSingleCoil(address, value, cb)) {
    cb(*pdu);  // Handle error via callback
    pdu->clear();
    return;
  }
  sendPDU(pdu, slaves.getActive());
}

void ModbusMaster::writeSingleCoil(uint8_t slave, uint16_t address, bool value, const modbusCallback& cb) {
  if (slave == 0 && !isWriteFunction(MB_FC_WRITE_SINGLE_COIL)) {
    PDU ret(slave);
    ret._err = MB_EX_LIB_INVALID_SLAVE;
    cb(ret);
    return;
  }
  PDU* pdu = getFreePDU(cb, slave);
  if (!pdu) return;
  if (pdu->createWriteSingleCoil(address, value, cb)) {
    cb(*pdu);  // Handle error via callback
    pdu->clear();
    return;
  }
  sendPDU(pdu, slave);
}

void ModbusMaster::writeCoils(const Slaves& slaves, uint16_t address, const uint8_t* src, uint8_t byteCount, uint16_t coilCount, const modbusCallback& cb) {
  PDU* pdu = getFreePDU(cb, slaves);
  if (!pdu) return;
  if (pdu->createWriteMultipleCoils(address, src, byteCount, coilCount, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slaves.getActive());
}

void ModbusMaster::writeCoils(uint8_t slave, uint16_t address, const uint8_t* src, uint8_t byteCount, uint16_t coilCount, const modbusCallback& cb) {
  if (slave == 0 && !isWriteFunction(MB_FC_WRITE_MULTIPLE_COILS)) {
    PDU ret(slave);
    ret._err = MB_EX_LIB_INVALID_SLAVE;
    cb(ret);
    return;
  }
  PDU* pdu = getFreePDU(cb, slave);
  if (!pdu) return;
  if (pdu->createWriteMultipleCoils(address, src, byteCount, coilCount, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slave);
}

void ModbusMaster::writeCoils(const Slaves& slaves, uint16_t address, const bool* src, uint16_t coilCount, const modbusCallback& cb) {
  PDU* pdu = getFreePDU(cb, slaves);
  if (!pdu) return;
  if (pdu->createWriteMultipleCoils(address, src, coilCount, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slaves.getActive());
}

void ModbusMaster::writeCoils(uint8_t slave, uint16_t address, const bool* src, uint16_t coilCount, const modbusCallback& cb) {
  if (slave == 0 && !isWriteFunction(MB_FC_WRITE_MULTIPLE_COILS)) {
    PDU ret(slave);
    ret._err = MB_EX_LIB_INVALID_SLAVE;
    cb(ret);
    return;
  }
  PDU* pdu = getFreePDU(cb, slave);
  if (!pdu) return;
  if (pdu->createWriteMultipleCoils(address, src, coilCount, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slave);
}

void ModbusMaster::writeCoils(const Slaves& slaves, uint16_t address, std::initializer_list<bool> list, const modbusCallback& cb) {
  PDU* pdu = getFreePDU(cb, slaves);
  if (!pdu) return;
  uint16_t count = list.size();
  bool values[count];
  uint16_t i = 0;
  for (const bool& value : list) {
    values[i++] = value;
  }
  if (pdu->createWriteMultipleCoils(address, values, count, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slaves.getActive());
}

void ModbusMaster::writeCoils(uint8_t slave, uint16_t address, std::initializer_list<bool> list, const modbusCallback& cb) {
  if (slave == 0 && !isWriteFunction(MB_FC_WRITE_MULTIPLE_COILS)) {
    PDU ret(slave);
    ret._err = MB_EX_LIB_INVALID_SLAVE;
    cb(ret);
    return;
  }
  PDU* pdu = getFreePDU(cb, slave);
  if (!pdu) return;
  uint16_t count = list.size();
  bool values[count];
  uint16_t i = 0;
  for (const bool& value : list) {
    values[i++] = value;
  }
  if (pdu->createWriteMultipleCoils(address, values, count, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slave);
}

void ModbusMaster::readCoilsByBytes(const Slaves& slaves, uint16_t address, uint8_t byteCount, const modbusCallback& cb) {
  PDU* pdu = getFreePDU(cb, slaves);
  if (!pdu) return;
  if (pdu->createReadState(MB_FC_READ_COILS, address, (uint16_t)byteCount * 8, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slaves.getActive());
}

void ModbusMaster::readCoilsByBytes(uint8_t slave, uint16_t address, uint8_t byteCount, const modbusCallback& cb) {
  if (slave == 0) {
    PDU ret(slave);
    ret._err = MB_EX_LIB_INVALID_SLAVE;
    cb(ret);
    return;
  }
  PDU* pdu = getFreePDU(cb, slave);
  if (!pdu) return;
  if (pdu->createReadState(MB_FC_READ_COILS, address, (uint16_t)byteCount * 8, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slave);
}

void ModbusMaster::readCoil(const Slaves& slaves, uint16_t address, const modbusCallback& cb) {
  PDU* pdu = getFreePDU(cb, slaves);
  if (!pdu) return;
  if (pdu->createReadState(MB_FC_READ_COILS, address, 1, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slaves.getActive());
}

void ModbusMaster::readCoil(uint8_t slave, uint16_t address, const modbusCallback& cb) {
  if (slave == 0) {
    PDU ret(slave);
    ret._err = MB_EX_LIB_INVALID_SLAVE;
    cb(ret);
    return;
  }
  PDU* pdu = getFreePDU(cb, slave);
  if (!pdu) return;
  if (pdu->createReadState(MB_FC_READ_COILS, address, 1, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slave);
}

void ModbusMaster::readCoils(const Slaves& slaves, uint16_t address, uint16_t coilCount, const modbusCallback& cb) {
  PDU* pdu = getFreePDU(cb, slaves);
  if (!pdu) return;
  if (pdu->createReadState(MB_FC_READ_COILS, address, coilCount, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slaves.getActive());
}

void ModbusMaster::readCoils(uint8_t slave, uint16_t address, uint16_t coilCount, const modbusCallback& cb) {
  if (slave == 0) {
    PDU ret(slave);
    ret._err = MB_EX_LIB_INVALID_SLAVE;
    cb(ret);
    return;
  }
  PDU* pdu = getFreePDU(cb, slave);
  if (!pdu) return;
  if (pdu->createReadState(MB_FC_READ_COILS, address, coilCount, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slave);
}

void ModbusMaster::readDiscreteInput(const Slaves& slaves, uint16_t address, const modbusCallback& cb) {
  PDU* pdu = getFreePDU(cb, slaves);
  if (!pdu) return;
  if (pdu->createReadState(MB_FC_READ_DISCRETE_INPUTS, address, 1, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slaves.getActive());
}

void ModbusMaster::readDiscreteInput(uint8_t slave, uint16_t address, const modbusCallback& cb) {
  if (slave == 0) {
    PDU ret(slave);
    ret._err = MB_EX_LIB_INVALID_SLAVE;
    cb(ret);
    return;
  }
  PDU* pdu = getFreePDU(cb, slave);
  if (!pdu) return;
  if (pdu->createReadState(MB_FC_READ_DISCRETE_INPUTS, address, 1, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slave);
}

void ModbusMaster::readDiscreteInputsByBytes(const Slaves& slaves, uint16_t address, uint8_t byteCount, const modbusCallback& cb) {
  PDU* pdu = getFreePDU(cb, slaves);
  if (!pdu) return;
  if (pdu->createReadState(MB_FC_READ_DISCRETE_INPUTS, address, (uint16_t)byteCount * 8, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slaves.getActive());
}

void ModbusMaster::readDiscreteInputsByBytes(uint8_t slave, uint16_t address, uint8_t byteCount, const modbusCallback& cb) {
  if (slave == 0) {
    PDU ret(slave);
    ret._err = MB_EX_LIB_INVALID_SLAVE;
    cb(ret);
    return;
  }
  PDU* pdu = getFreePDU(cb, slave);
  if (!pdu) return;
  if (pdu->createReadState(MB_FC_READ_DISCRETE_INPUTS, address, (uint16_t)byteCount * 8, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slave);
}

void ModbusMaster::readDiscreteInputs(const Slaves& slaves, uint16_t address, uint16_t coilCount, const modbusCallback& cb) {
  PDU* pdu = getFreePDU(cb, slaves);
  if (!pdu) return;
  if (pdu->createReadState(MB_FC_READ_DISCRETE_INPUTS, address, coilCount, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slaves.getActive());
}

void ModbusMaster::readDiscreteInputs(uint8_t slave, uint16_t address, uint16_t coilCount, const modbusCallback& cb) {
  if (slave == 0) {
    PDU ret(slave);
    ret._err = MB_EX_LIB_INVALID_SLAVE;
    cb(ret);
    return;
  }
  PDU* pdu = getFreePDU(cb, slave);
  if (!pdu) return;
  if (pdu->createReadState(MB_FC_READ_DISCRETE_INPUTS, address, coilCount, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slave);
}

void ModbusMaster::writeSingleHoldingRegister(const Slaves& slaves, uint16_t address, uint16_t src, const modbusCallback& cb) {
  PDU* pdu = getFreePDU(cb, slaves);
  if (!pdu) return;
  if (pdu->createWriteSingleRegister(address, src, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slaves.getActive());
}

void ModbusMaster::writeSingleHoldingRegister(uint8_t slave, uint16_t address, uint16_t src, const modbusCallback& cb) {
  if (slave == 0 && !isWriteFunction(MB_FC_WRITE_SINGLE_REGISTER)) {
    PDU ret(slave);
    ret._err = MB_EX_LIB_INVALID_SLAVE;
    cb(ret);
    return;
  }
  PDU* pdu = getFreePDU(cb, slave);
  if (!pdu) return;
  if (pdu->createWriteSingleRegister(address, src, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slave);
}

void ModbusMaster::readExceptionStatus(const Slaves& slaves, const modbusCallback& cb) {
  PDU* pdu = getFreePDU(cb, slaves);
  if (!pdu) return;
  if (pdu->createReadExceptionStatus(cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slaves.getActive());
}

void ModbusMaster::readExceptionStatus(uint8_t slave, const modbusCallback& cb) {
  if (slave == 0) {
    PDU ret(slave);
    ret._err = MB_EX_LIB_INVALID_SLAVE;
    cb(ret);
    return;
  }
  PDU* pdu = getFreePDU(cb, slave);
  if (!pdu) return;
  if (pdu->createReadExceptionStatus(cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slave);
}

void ModbusMaster::maskWriteRegister(const Slaves& slaves, uint16_t address, uint16_t andMask, uint16_t orMask, const modbusCallback& cb) {
  PDU* pdu = getFreePDU(cb, slaves);
  if (!pdu) return;
  if (pdu->createMaskWriteRegister(address, andMask, orMask, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slaves.getActive());
}

void ModbusMaster::maskWriteRegister(uint8_t slave, uint16_t address, uint16_t andMask, uint16_t orMask, const modbusCallback& cb) {
  if (slave == 0 && !isWriteFunction(MB_FC_MASK_WRITE_REGISTER)) {
    PDU ret(slave);
    ret._err = MB_EX_LIB_INVALID_SLAVE;
    cb(ret);
    return;
  }
  PDU* pdu = getFreePDU(cb, slave);
  if (!pdu) return;
  if (pdu->createMaskWriteRegister(address, andMask, orMask, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slave);
}

void ModbusMaster::diagnostic(const Slaves& slaves, uint16_t subFunction, uint16_t data, const modbusCallback& cb) {
  PDU* pdu = getFreePDU(cb, slaves);
  if (!pdu) return;
  if (pdu->createDiagnostics(subFunction, data, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slaves.getActive());
}

void ModbusMaster::diagnostic(uint8_t slave, uint16_t subFunction, uint16_t data, const modbusCallback& cb) {
  if (slave == 0) {
    PDU ret(slave);
    ret._err = MB_EX_LIB_INVALID_SLAVE;
    cb(ret);
    return;
  }
  PDU* pdu = getFreePDU(cb, slave);
  if (!pdu) return;
  if (pdu->createDiagnostics(subFunction, data, cb)) {
    cb(*pdu);
    pdu->clear();
    return;
  }
  sendPDU(pdu, slave);
}