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

#ifndef MCP2515_RST
#define MCP2515_RST1 9
#else
#define MCP2515_RST1 MCP2515_RST
#endif

// External debug flag from main.cpp
extern bool canDebugMcp2515Enabled;
// External telnet functions from main.cpp
extern void telnetPrintf(const char *format, ...);
extern void telnetPrintln(const char *str);

// ACAN2515 CAN controller
ACAN2515 *CAN2_ptr = nullptr;

// Early pin initialization
inline void initMCP2515Pins()
{
  // Reset MCP2515
  pinMode(MCP2515_RST1, OUTPUT);
  digitalWrite(MCP2515_RST1, LOW);
  delay(10);
  digitalWrite(MCP2515_RST1, HIGH);
  delay(50);

  pinMode(MCP2515_CS1, OUTPUT);
  digitalWrite(MCP2515_CS1, HIGH);
  pinMode(MCP2515_INT1, INPUT);
}

// Initialize MCP2515 with ACAN2515 library
inline bool initMCP2515()
{
  Serial.println("Initializing SPI and MCP2515 (ACAN2515)...");

  // Ensure pins are configured
  initMCP2515Pins();

  // Initialize SPI with custom pins
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, MCP2515_CS1);
  SPI.setFrequency(10000000); // 10MHz SPI - maximum speed
  delay(200);

  // Create ACAN2515 object
  if (CAN2_ptr == nullptr)
  {
    CAN2_ptr = new ACAN2515(MCP2515_CS1, SPI, MCP2515_INT1);
  }

  // Configure ACAN2515 settings
  // Constructor: (QuartzFrequency in Hz, DesiredBitRate in bps)
  // LilyGO T-2CAN uses 16MHz crystal (not 8MHz!)
  ACAN2515Settings settings(16000000UL, 500UL * 1000UL); // 16MHz crystal, 500kbps

  // Increase buffer sizes for high traffic (default is 16 RX, 16 TX)
  settings.mReceiveBufferSize = 64;   // RX buffer: 64 messages
  settings.mTransmitBuffer0Size = 64; // TX buffer 0: 64 messages

  // Set to Normal mode (not listen-only) to allow TX
  settings.mRequestedMode = ACAN2515Settings::NormalMode;

  // Enable rollover from RXB0 to RXB1 if RXB0 is full
  settings.mRolloverEnable = true;

  // Initialize without filters (accepts all messages by default)
  const uint32_t errorCode = CAN2_ptr->begin(settings, []
                                             { CAN2_ptr->isr(); });

  if (errorCode == 0)
  {
    Serial.println("✓ MCP2515 init OK (ACAN2515, 16MHz, 500kbps, 10MHz SPI)");
    Serial.println("✓ TX/RX buffers: 64 messages each");
    Serial.println("✓ Filters: DISABLED (accept all messages)");
    return true;
  }

  Serial.print("✗ MCP2515 init FAILED! Error code: 0x");
  Serial.println(errorCode, HEX);
  if (errorCode == 0xFFFFFFFF)
  {
    Serial.println("⚠ No MCP2515 detected - check wiring, CS pin, and power");
  }
  return false;
}

// Send message on CAN2
inline bool sendCAN2(uint32_t id, uint8_t len, uint8_t *data)
{
  if (CAN2_ptr == nullptr)
  {
    return false;
  }

  CANMessage message;
  message.id = id;
  message.len = len;
  message.ext = false;
  message.rtr = false;
  memcpy(message.data, data, len);

  bool success = CAN2_ptr->tryToSend(message);

  // Debug output if enabled (throttled to reduce overhead)
  if (canDebugMcp2515Enabled)
  {
    static uint32_t lastDebug = 0;
    static uint32_t debugCount = 0;
    if (millis() - lastDebug > 200 || !success)
    { // Only every 200ms or on failure
      lastDebug = millis();
      char debugBuf[128];
      int offset = snprintf(debugBuf, sizeof(debugBuf), "[MCP2515 TX] 0x%03X [%d] ", id, len);
      for (int i = 0; i < len; i++)
      {
        offset += snprintf(debugBuf + offset, sizeof(debugBuf) - offset, "%02X ", data[i]);
      }
      snprintf(debugBuf + offset, sizeof(debugBuf) - offset, "%s (%lu total)\n", success ? "OK" : "FAIL", ++debugCount);
      telnetPrintf("%s", debugBuf);
      Serial.print(debugBuf);
    }
  }

  return success;
}

// Read message from CAN2
inline bool readCAN2(uint32_t &id, uint8_t &len, uint8_t *data)
{
  if (CAN2_ptr == nullptr)
  {
    return false;
  }

  CANMessage message;
  if (CAN2_ptr->receive(message))
  {
    id = message.id;
    len = message.len;
    memcpy(data, message.data, len);

    // Debug output for RX (always show to debug slave responses)
    if (canDebugMcp2515Enabled)
    {
      char debugBuf[128];
      int offset = snprintf(debugBuf, sizeof(debugBuf), "[MCP2515 RX] 0x%03X [%d] ", id, len);
      for (int i = 0; i < len; i++)
      {
        offset += snprintf(debugBuf + offset, sizeof(debugBuf) - offset, "%02X ", data[i]);
      }
      snprintf(debugBuf + offset, sizeof(debugBuf) - offset, "\n");
      telnetPrintf("%s", debugBuf);
      Serial.print(debugBuf);
    }

    return true;
  }
  return false;
}

// Check if MCP2515 has messages to read
inline bool canReadCAN2()
{
  if (CAN2_ptr == nullptr)
  {
    return false;
  }
  return CAN2_ptr->available() > 0;
}

// Get MCP2515 statistics
inline void printMCP2515Stats()
{
  if (CAN2_ptr == nullptr)
  {
    return;
  }

  telnetPrintf("[MCP2515 Stats] Available: %u\n",
                CAN2_ptr->available());
}

// Clear MCP2515 TX buffer by reading pending RX and checking status
inline void clearMCP2515Buffers()
{
  if (CAN2_ptr == nullptr)
  {
    return;
  }

  // Read all pending RX messages
  uint32_t id;
  uint8_t len;
  uint8_t data[8];
  int cleared = 0;

  while (readCAN2(id, len, data) && cleared++ < 100)
  {
    // Just drain the buffer
  }

  if (cleared > 0)
  {
    telnetPrintf("Cleared %d MCP2515 RX messages\n", cleared);
  }
}
