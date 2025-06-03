#include "ModbusUtility.h"

bool isBigEndian;

bool setIsBigEndian() {
  union {
    uint16_t value;
    uint8_t bytes[2];
  } test{0x1234};
  static_assert(sizeof(uint16_t) == 2, "uint16_t must be 2 bytes");
  isBigEndian = test.bytes[0] == 0x12;
  return isBigEndian;
}

void setIsBigEndian(bool value) {
  isBigEndian = value;
}

void printBuffer(uint8_t* buffer, uint16_t len) {
  for (size_t i = 0; i < len; i++) {
    Serial.print(buffer[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}