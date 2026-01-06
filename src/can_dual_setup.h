// Dual CAN setup for ESP32 + 2x MCP2515
// For use with BMW-i3-Balancing-Controller
// LilyGO T-2CAN with built-in MCP2515
// Using ACAN2515 library with larger software buffers

#include <ACAN2515.h>
#include <SPI.h>

// LilyGO T-2CAN Pin configuration
#ifndef MCP2515_SCK
  #define SPI_SCK 12
#else
  #define SPI_SCK MCP2515_SCK
#endif

#ifndef MCP2515_MISO
  #define SPI_MISO 13
#else
  #define SPI_MISO MCP2515_MISO
#endif

#ifndef MCP2515_MOSI
  #define SPI_MOSI 11
#else
  #define SPI_MOSI MCP2515_MOSI
#endif

#ifndef MCP2515_CS
  #define MCP2515_CS1 10
#else
  #define MCP2515_CS1 MCP2515_CS
#endif

#ifndef MCP2515_INT
  #define MCP2515_INT1 8
#else
  #define MCP2515_INT1 MCP2515_INT
#endif

// External debug flag from main.cpp
extern bool canDebugMcp2515Enabled;

// ACAN2515 CAN controller
ACAN2515* CAN2_ptr = nullptr;

// Early pin initialization
inline void initMCP2515Pins() {
  pinMode(MCP2515_CS1, OUTPUT);
  digitalWrite(MCP2515_CS1, HIGH);
  pinMode(MCP2515_INT1, INPUT);
}

// Initialize MCP2515 with ACAN2515 library
inline bool initMCP2515() {
  Serial.println("Initializing SPI and MCP2515 (ACAN2515)...");
  
  // Ensure pins are configured
  initMCP2515Pins();
  
  // Initialize SPI with custom pins
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, MCP2515_CS1);
  SPI.setFrequency(1000000); // 1MHz SPI
  delay(200);
  
  // Create ACAN2515 object
  if (CAN2_ptr == nullptr) {
    CAN2_ptr = new ACAN2515(MCP2515_CS1, SPI, MCP2515_INT1);
  }
  
  // Configure ACAN2515 settings
  // Constructor: (QuartzFrequency in Hz, DesiredBitRate in bps)
  ACAN2515Settings settings(8000000UL, 500UL * 1000UL); // 8MHz crystal, 500kbps
  
  // Increase buffer sizes for high traffic (default is 16 RX, 16 TX)
  settings.mReceiveBufferSize = 32;     // RX buffer: 32 messages
  settings.mTransmitBuffer0Size = 32;   // TX buffer 0: 32 messages
  
  // Initialize
  const uint32_t errorCode = CAN2_ptr->begin(settings, [] { CAN2_ptr->isr(); });
  
  if (errorCode == 0) {
    Serial.println("✓ MCP2515 init OK (ACAN2515, 8MHz, 500kbps, 1MHz SPI)");
    Serial.println("✓ TX/RX buffers: 32 messages each");
    return true;
  }
  
  Serial.print("✗ MCP2515 init FAILED! Error code: 0x");
  Serial.println(errorCode, HEX);
  if (errorCode == 0xFFFFFFFF) {
    Serial.println("⚠ No MCP2515 detected - check wiring, CS pin, and power");
  }
  return false;
}

// Send message on CAN2
inline bool sendCAN2(uint32_t id, uint8_t len, uint8_t *data) {
  if (CAN2_ptr == nullptr) {
    return false;
  }
  
  CANMessage message;
  message.id = id;
  message.len = len;
  message.ext = false;
  message.rtr = false;
  memcpy(message.data, data, len);
  
  bool success = CAN2_ptr->tryToSend(message);
  
  // Debug output if enabled
  if (canDebugMcp2515Enabled) {
    Serial.printf("[MCP2515 TX] 0x%03X [%d] ", id, len);
    for (int i = 0; i < len; i++) {
      Serial.printf("%02X ", data[i]);
    }
    Serial.println(success ? "OK" : "FAIL");
  }
  
  return success;
}

// Read message from CAN2
inline bool readCAN2(uint32_t &id, uint8_t &len, uint8_t *data) {
  if (CAN2_ptr == nullptr) {
    return false;
  }
  
  CANMessage message;
  if (CAN2_ptr->receive(message)) {
    id = message.id;
    len = message.len;
    memcpy(data, message.data, len);
    
    // Debug output if enabled
    if (canDebugMcp2515Enabled) {
      Serial.printf("[MCP2515 RX] 0x%03X [%d] ", id, len);
      for (int i = 0; i < len; i++) {
        Serial.printf("%02X ", data[i]);
      }
      Serial.println();
    }
    
    return true;
  }
  return false;
}
