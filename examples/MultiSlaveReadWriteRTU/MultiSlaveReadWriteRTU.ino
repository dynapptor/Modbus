// MultiSlaveReadWriteRTU.ino
// Demonstrates Modbus RTU communication with multiple slaves, writing and reading
// a custom data structure (MyData) using Serial1 for Modbus and Serial for debugging.
// Compatible with Arduino Mega or boards with multiple serial ports.

// Include Arduino core library for basic functionality
#include <Arduino.h>

// Include the Modbus RTU master library
#include <ModbusRTUMaster.h>

// Define a custom data structure for holding register operations
struct MyData {
  uint8_t val8;    // 8-bit value
  uint32_t val32;  // 32-bit value
  uint16_t val16;  // 16-bit value
};

// Array to store data for three slaves
MyData data[3];

// Initialize the Modbus RTU master
ModbusRTUMaster master;

// Initialize Slaves objects for writing and reading
// slavesWrite: Slaves 1, 2, 3 for writing, no delays
Slaves slavesWrite({1, 2, 3});
// slavesRead: Slaves 1, 2, 3 for reading, 0 ms item delay, 1000 ms repeat delay
Slaves slavesRead({1, 2, 3}, 0, 1000);

// Callback function to handle Modbus responses
void callback(PDU& pdu) {
  if (pdu.getErr() == MB_EX_SUCCESS) {  // Check for successful operation
    if (pdu.getFunction() == MB_FC_WRITE_MULTIPLE_REGISTERS) {
      // Handle successful write operation
      Serial.println("Write success");
    } else {
      // Handle successful read operation
      Serial.print("Received from slaveID: ");
      Serial.println(pdu.getSlaveId());  // Print the responding slave ID

      // Retrieve the array of MyData structures from the PDU
      const MyData* data = pdu.getDataArray<MyData>();
      for (size_t i = 0; i < 3; i++) {
        // Print each MyData instance's fields
        Serial.print(data[i].val8);
        Serial.print(' ');
        Serial.print(data[i].val32);
        Serial.print(' ');
        Serial.println(data[i].val16);
      }
    }
  } else {
    // Handle error cases
    Serial.print("Error: ");
    Serial.println(pdu.getErr());  // Print error code
  }
}

void setup() {
  // Initialize Serial for debugging at 115200 baud
  Serial.begin(115200);
  // Initialize Serial1 for Modbus RTU communication at 9600 baud
  Serial1.begin(9600);

  // Initialize data array with sample values for three slaves
  data[0].val8 = 11;
  data[0].val32 = 1111111111;
  data[0].val16 = 11111;
  data[1].val8 = 22;
  data[1].val32 = 2222222222;
  data[1].val16 = 22222;
  data[2].val8 = 33;
  data[2].val32 = 3333333333;
  data[2].val16 = 33333;

  // Initialize the RTU master
  // Parameters: max PDU size, queue size, Serial stream, baud rate, UART config,
  //             RS485 receive enable pin (3), transmit enable pin (4)
  master.begin(MB_PDU_MAX_SIZE, 10, &Serial1, 9600, UartConfig::Mode_8N1, 3, 4);

  // Create a callback object for asynchronous operations
  modbusCallback cb(callback);

  // Write holding registers to slaves 1, 2, 3 at address 0
  master.writeHoldingRegisters(slavesWrite, 0, data, 3, cb);

  // Read holding registers from slaves 1, 2, 3 at address 0, repeating every 1000 ms
  master.readHoldingRegisters<MyData>(slavesRead, 0, 3, cb);
}

void loop() {
  // Process Modbus communication in the main loop
  master.loop();
}