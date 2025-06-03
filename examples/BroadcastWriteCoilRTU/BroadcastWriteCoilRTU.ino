
#include <Arduino.h>

// BroadcastWriteCoilRTU.ino
// Example for the Modbus library demonstrating an RTU broadcast write to a coil.
// Uses Serial1 for Modbus RTU communication and Serial for debugging.
// Compatible with Arduino Mega or boards with multiple serial ports.

// Include the Modbus RTU master library
#include <ModbusRTUMaster.h>

// Initialize the Modbus RTU master
ModbusRTUMaster master;

// Initialize the Slaves object for broadcast mode send every 5 sec
Slaves slaves(0, 1000);

modbusCallback cb([](PDU& pdu) {
  if (pdu.getErr() == MB_EX_SUCCESS) {
    Serial.println("Broadcast coil write sent");
  } else {
    Serial.print("Error: ");
    Serial.println(pdu.getErr());
  } });

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  // Initialize Serial1 for Modbus RTU communication
  Serial1.begin(115200);

  // Initialize the RTU master
  // Parameters: PDU size (20), queue size (5), Serial stream, baud rate, UART config
  master.begin(20, 5, &Serial1, 115200, UartConfig::Mode_8N1);

  // Perform a broadcast write to coil at address 0 every 5 seconds
  master.writeSingleCoil(slaves, 0, 1, cb);
}

void loop() {
  // Process Modbus communication
  master.loop();
}