// Dual CAN setup for ESP32 + 2x MCP2515
// For use with BMW-i3-Balancing-Controller

#include <mcp_can.h>
#include <SPI.h>

// Board-specific pin configuration
#ifdef LILYGO_T2CAN
  // LilyGO T-2CAN with built-in MCP2515
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
#else
  // Default T-CAN485 wiring (custom SPI pins matching battery emulator)
  #define SPI_SCK 12
  #define SPI_MISO 34
  #define SPI_MOSI 5
  #define MCP2515_CS1 18
  #define MCP2515_INT1 35
#endif

// Use pointer to avoid global constructor issues
MCP_CAN* CAN2_ptr = nullptr;
#define CAN2 (*CAN2_ptr)

// MCP2515 #2 (CAN3) - hvis du vil have 3 CAN, ellers ignorer
//#define MCP2515_CS2 19
//#define MCP2515_INT2 36
//MCP_CAN CAN3(MCP2515_CS2);

// Global flag to track MCP2515 status
bool mcp2515_available = false;

// Early pin initialization - call this BEFORE creating MCP_CAN object if needed
inline void initMCP2515Pins() {
  pinMode(MCP2515_CS1, OUTPUT);
  digitalWrite(MCP2515_CS1, HIGH);
  pinMode(MCP2515_INT1, INPUT);
}

// SPI init (kald i setup)
inline bool initMCP2515() {
  Serial.println("Initializing SPI and MCP2515...");
  
  // Ensure pins are configured
  initMCP2515Pins();
  
  // Create MCP_CAN object now (after pins are configured)
  if (CAN2_ptr == nullptr) {
    CAN2_ptr = new MCP_CAN(MCP2515_CS1);
  }
  
  // Initialize SPI with custom pins (matching battery emulator wiring)
  // SCK=12, MISO=34, MOSI=5, CS=18
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, MCP2515_CS1);
  
  // Set SPI to 1MHz (proven to work with 3.3V MCP2515 module)
  SPI.setFrequency(1000000);
  delay(200);
  
  // Initialize MCP2515 with 8MHz crystal, 500kbps CAN
  digitalWrite(MCP2515_CS1, HIGH);
  delay(50);
  
  if (CAN2.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("✓ MCP2515 init OK (8MHz crystal, 500kbps, 1MHz SPI)");
    CAN2.setMode(MCP_NORMAL);
    mcp2515_available = true;
    return true;
  }
  
  Serial.println("✗ MCP2515 init FAILED!");
  Serial.println("⚠ Check: wiring, power (5V/3.3V), and common GND");
  mcp2515_available = false;
  return false;
}

// Eksempel på at sende besked på CAN2
inline bool sendCAN2(uint32_t id, uint8_t len, uint8_t *data) {
  if (!mcp2515_available || CAN2_ptr == nullptr) {
    return false;
  }
  byte result = CAN2.sendMsgBuf(id, 0, len, data);
  return (result == CAN_OK);
}

// Eksempel på at læse besked fra CAN2
inline bool readCAN2(uint32_t &id, uint8_t &len, uint8_t *data) {
  if (!mcp2515_available || CAN2_ptr == nullptr) {
    return false;
  }
  if (CAN2.checkReceive() == CAN_MSGAVAIL) {
    long unsigned int rxId;
    unsigned char rxLen = 0;
    CAN2.readMsgBuf(&rxId, &rxLen, data);
    id = rxId;
    len = rxLen;
    return true;
  }
  return false;
}
