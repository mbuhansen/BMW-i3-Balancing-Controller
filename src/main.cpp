#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoOTA.h>
#include <Update.h>

#include "driver/twai.h"
#include "can_dual_setup.h" // MCP2515 dual CAN

// External flag from can_dual_setup.h
extern bool mcp2515_available;

// WiFi credentials - loaded from credentials.h (not tracked in git)
#include "credentials.h"

// ============================================================================
// BMW i3 Balancing Controller for LilyGO T-CAN485
// ============================================================================
// GATEWAY MODE: Forward messages between BMS and slave modules
// - MCP2515 (SPI): Connected to external BMS
// - TWAI (ESP32): Connected to BMW i3 slave modules
// - Normal: Forward BMS requests (0x080-0x08F) → slave modules, responses (0x100-0x1FF) → BMS
// - Balancing: Block BMS requests, send own balancing commands, forward responses to BMS
// - Monitors cell voltages and automatically balances when needed
// ============================================================================

// Pin definitions - Auto-configured based on board type
#ifdef LILYGO_T2CAN
  // LilyGO T-2CAN with dual native CAN controllers
  #ifndef CAN_TX_PIN
    #define CAN_TX_PIN GPIO_NUM_7
  #endif
  #ifndef CAN_RX_PIN
    #define CAN_RX_PIN GPIO_NUM_6
  #endif
  #ifndef LED_PIN
    #define LED_PIN GPIO_NUM_35
  #endif
  #define CAN_SE_PIN GPIO_NUM_NC // No standby pin on T-2CAN
#else
  // LilyGO T-CAN485 (default)
  #ifndef CAN_TX_PIN
    #define CAN_TX_PIN GPIO_NUM_27
  #endif
  #ifndef CAN_RX_PIN
    #define CAN_RX_PIN GPIO_NUM_26
  #endif
  #ifndef LED_PIN
    #define LED_PIN GPIO_NUM_4 // WS2812 RGB LED
  #endif
  #define CAN_SE_PIN GPIO_NUM_23
#endif

// Configuration
#define MAX_MODULES 8
#define CELLS_PER_MODULE 12
#define MIN_BALANCE_VOLTAGE 3.9f   // Minimum voltage to start balancing (V)
#define BALANCE_THRESHOLD_MV 10    // Start balancing if cells differ by more than 10mV
#define BALANCE_HYSTERESIS_MV 5    // Stop balancing when within 5mV
#define CAN_COMMAND_INTERVAL_MS 50 // Send commands every 50ms

// BMW CRC8 Lookup table and finalxor values (from SimpleBMS)
const uint8_t finalxor[12] = {0xCF, 0xF5, 0xBB, 0x81, 0x27, 0x1D, 0x53, 0x69, 0x02, 0x38, 0x76, 0x4C};

// Module data structure
struct BMWModule
{
  bool exists = false;
  bool balancing = false;
  uint32_t lastUpdate = 0;
  float cellVoltages[CELLS_PER_MODULE];
  float temperatures[4];
  float moduleVoltage = 0.0f;
  uint32_t errorCode = 0;
  uint16_t balanceStatus = 0;
};

// Global variables
BMWModule modules[MAX_MODULES];
bool balancingActive = false;
bool manualMode = false; // Start in AUTO mode - automatically balance when needed
bool gatewayMode = true; // GATEWAY: Forward messages between BMS and slave modules
bool externalMasterDetected = false;
bool canDebugEnabled = false; // CAN bus debug logging (disabled by default to prevent Serial overflow)
float targetBalanceVoltage = 4.0f;
uint8_t messageCounter = 0;
uint8_t nextMessage = 0;
uint32_t lastCommandTime = 0;
uint32_t lastExternalCommandTime = 0;
uint32_t lastDataUpdate = 0;

// Dual-core task handle - NO MUTEX (float reads are atomic enough on ESP32)
TaskHandle_t canGatewayTaskHandle = NULL;

// Queue for requesting broadcast from Core 0 to Core 1 (to avoid cross-core mutex in AsyncTCP)
QueueHandle_t broadcastQueue = NULL;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// LED control
Adafruit_NeoPixel led(1, LED_PIN, NEO_GRB + NEO_KHZ800);
uint8_t ledBrightness = 0;
uint32_t lastLedUpdate = 0;
bool ledAvailable = false; // Track if LED is working

// CRC8 calculation for BMW CAN messages
class CRC8
{
private:
  uint8_t crc8_table[256];

public:
  CRC8()
  {
    // Generate CRC8 table (SAE J1850)
    for (int i = 0; i < 256; i++)
    {
      uint8_t crc = i;
      for (int j = 0; j < 8; j++)
      {
        crc = (crc & 0x80) ? ((crc << 1) ^ 0x1D) : (crc << 1);
      }
      crc8_table[i] = crc;
    }
  }

  uint8_t get_crc8(const uint8_t *data, uint8_t len, uint8_t finalXor)
  {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++)
    {
      crc = crc8_table[crc ^ data[i]];
    }
    return crc ^ finalXor;
  }
};

CRC8 crc8;

// Calculate checksum for CAN message
uint8_t calculateChecksum(const twai_message_t &msg, uint8_t msgId)
{
  uint8_t canmes[11];
  int meslen = msg.data_length_code + 1;

  canmes[0] = msg.identifier >> 8;
  canmes[1] = msg.identifier & 0xFF;

  for (int i = 0; i < (msg.data_length_code - 1); i++)
  {
    canmes[i + 2] = msg.data[i];
  }

  return crc8.get_crc8(canmes, meslen, finalxor[msgId]);
}

// Initialize TWAI (CAN) driver
void initCAN()
{
  // Configure CAN SE (Silent Enable) pin - set LOW for normal mode
  // Only configure SE pin if it's not GPIO_NUM_NC (T-2CAN doesn't have SE pin)
  if (CAN_SE_PIN != GPIO_NUM_NC)
  {
    pinMode(CAN_SE_PIN, OUTPUT);
    digitalWrite(CAN_SE_PIN, LOW);
  }

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  g_config.rx_queue_len = 100;
  g_config.tx_queue_len = 100;

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
  {
    Serial.println("✓ CAN driver installed");
  }
  else
  {
    Serial.println("✗ Failed to install CAN driver");
    return;
  }

  if (twai_start() == ESP_OK)
  {
    Serial.println("✓ CAN driver started");
  }
  else
  {
    Serial.println("✗ Failed to start CAN driver");
  }
}

// Print CAN message to Serial for debugging
void printCANMessage(const twai_message_t &msg, bool isTX)
{
  if (!canDebugEnabled)
    return;

  // Only print RX messages to avoid Serial buffer overflow blocking TX transmission
  // TX messages create too much Serial output and prevent actual CAN transmission
  if (isTX)
    return;

  // Filter out noisy/repetitive messages to reduce Serial load
  uint32_t id = msg.identifier;

  // Skip empty status messages (0x100-0x107) - just 00 00 00 00 00 00
  if (id >= 0x100 && id <= 0x107)
    return;

  // Skip repetitive 0x22 messages (0x1C0-0x1C7, 0x1D0-0x1D7)
  if ((id >= 0x1C0 && id <= 0x1C7) || (id >= 0x1D0 && id <= 0x1D7))
    return;

  // Skip temperature messages for now (0x170-0x177) - not critical during balancing debug
  if (id >= 0x170 && id <= 0x177)
    return;

  char direction[4];
  strcpy(direction, isTX ? "TX" : "RX");

  char hexStr[32];
  sprintf(hexStr, "%03X", msg.identifier);

  Serial.printf("[CAN %s] ID: 0x%s [%d] ", direction, hexStr, msg.data_length_code);

  for (int i = 0; i < msg.data_length_code; i++)
  {
    Serial.printf("%02X ", msg.data[i]);
  }
  Serial.println();
}

// Parse incoming CAN messages from slave modules
void parseModuleMessage(const twai_message_t &msg)
{
  uint32_t id = msg.identifier;

  // Ignore control messages from other masters (0x080-0x08F)
  if ((id & 0xFF0) == 0x080)
  {
    return; // Don't parse control messages, only data messages
  }

  // BMW i3 modules use ID format 0x1XY where X is message type and Y is module number
  // Check if this is a valid BMW module message (0x100-0x1FF range)
  if ((id & 0xF00) != 0x100)
    return;

  uint8_t moduleId = (id & 0x00F);
  uint8_t messageType = (id & 0x0F0) >> 4;

  // Only process valid data message types (0, 2-7)
  // Ignore 0x1CX (type 12) and 0x1DX (type 13) - not battery data
  if (messageType == 1 || messageType > 7)
    return;

  if (moduleId >= MAX_MODULES)
    return;

  // Debug: Print module detection (only first time) - BEFORE taking mutex
  static bool moduleDetected[MAX_MODULES] = {false};
  bool isFirstDetection = false;
  if (!moduleDetected[moduleId])
  {
    moduleDetected[moduleId] = true;
    isFirstDetection = true;
  }

  // Direct access - float writes are atomic on ESP32
  BMWModule &module = modules[moduleId];
  module.exists = true;
  module.lastUpdate = millis();

  // Determine message type based on ID pattern
  // 0x10X = error/balance status (type 0)
  // 0x12X = cells 0-2 (type 2)
  // 0x13X = cells 3-5 (type 3)
  // 0x14X = cells 6-8 (type 4)
  // 0x15X = cells 9-11 (type 5)
  // 0x17X = temperatures (ignored for now)
  uint8_t typeId = messageType;

  if (typeId > 5 && typeId != 7)
  {
    return;
  }

  // Parse based on message type
  switch (typeId)
  {
  case 0:
    // Error and balance status (0x10X)
    module.errorCode = msg.data[0] | (msg.data[1] << 8) | (msg.data[2] << 16) | (msg.data[3] << 24);
    module.balanceStatus = ((msg.data[5] & 0x0F) << 8) | msg.data[4];
    break;

  case 2:
    // Cells 0-2 (0x12X)
    if (msg.data[1] < 0x40)
      module.cellVoltages[0] = (msg.data[0] + (msg.data[1] & 0x3F) * 256) / 1000.0f;
    if (msg.data[3] < 0x40)
      module.cellVoltages[1] = (msg.data[2] + (msg.data[3] & 0x3F) * 256) / 1000.0f;
    if (msg.data[5] < 0x40)
      module.cellVoltages[2] = (msg.data[4] + (msg.data[5] & 0x3F) * 256) / 1000.0f;
    break;

  case 3:
    // Cells 3-5 (0x13X)
    if (msg.data[1] < 0x40)
      module.cellVoltages[3] = (msg.data[0] + (msg.data[1] & 0x3F) * 256) / 1000.0f;
    if (msg.data[3] < 0x40)
      module.cellVoltages[4] = (msg.data[2] + (msg.data[3] & 0x3F) * 256) / 1000.0f;
    if (msg.data[5] < 0x40)
      module.cellVoltages[5] = (msg.data[4] + (msg.data[5] & 0x3F) * 256) / 1000.0f;
    break;

  case 4:
    // Cells 6-8 (0x14X)
    if (msg.data[1] < 0x40)
      module.cellVoltages[6] = (msg.data[0] + (msg.data[1] & 0x3F) * 256) / 1000.0f;
    if (msg.data[3] < 0x40)
      module.cellVoltages[7] = (msg.data[2] + (msg.data[3] & 0x3F) * 256) / 1000.0f;
    if (msg.data[5] < 0x40)
      module.cellVoltages[8] = (msg.data[4] + (msg.data[5] & 0x3F) * 256) / 1000.0f;
    break;

  case 5:
    // Cells 9-11 (0x15X)
    if (msg.data[1] < 0x40)
      module.cellVoltages[9] = (msg.data[0] + (msg.data[1] & 0x3F) * 256) / 1000.0f;
    if (msg.data[3] < 0x40)
      module.cellVoltages[10] = (msg.data[2] + (msg.data[3] & 0x3F) * 256) / 1000.0f;
    if (msg.data[5] < 0x40)
      module.cellVoltages[11] = (msg.data[4] + (msg.data[5] & 0x3F) * 256) / 1000.0f;
    break;

  case 6:
    // Balancing status (0x16X)
    // Byte 2-5 contain balancing data: non-zero means balancing is active
    module.balancing = (msg.data[2] != 0 || msg.data[3] != 0 || msg.data[4] != 0 || msg.data[5] != 0);
    break;
  }

  // Print detection message AFTER releasing mutex to avoid slow Serial operations in critical section
  if (isFirstDetection)
  {
    Serial.printf("Module %d detected (ID: 0x%03X, Type: %d)\n", moduleId, id, messageType);
  }
}

// GATEWAY MODE: Forward messages between BMS and slave modules
void readCANMessages()
{
  // ========== Read from MCP2515 (BMS side) ==========
  // BMS sends requests (0x080-0x08F) to slave modules
  if (mcp2515_available)
  {
    uint32_t mcp_id;
    uint8_t mcp_len;
    uint8_t mcp_data[8];

    int msgCount = 0;
    while (readCAN2(mcp_id, mcp_len, mcp_data) && msgCount++ < 20)
    {
      // BMS request to slave modules (0x080-0x08F)
      if ((mcp_id & 0xFF0) == 0x080)
      {
        // Only forward if NOT balancing (when balancing, we send our own commands)
        if (!balancingActive)
        {
          // Forward BMS request to slave modules via TWAI
          twai_message_t twai_msg;
          twai_msg.identifier = mcp_id;
          twai_msg.data_length_code = mcp_len;
          twai_msg.flags = TWAI_MSG_FLAG_NONE;
          twai_msg.extd = 0;
          twai_msg.rtr = 0;
          memcpy(twai_msg.data, mcp_data, mcp_len);
          
          esp_err_t result = twai_transmit(&twai_msg, pdMS_TO_TICKS(10));
          if (canDebugEnabled && result == ESP_OK)
          {
            Serial.printf("[GATEWAY] BMS→Slave: 0x%03X forwarded\n", mcp_id);
          }
        }
      }
      // Parse module data for monitoring (0x100-0x1FF)
      else if ((mcp_id & 0xF00) == 0x100)
      {
        twai_message_t temp_msg;
        temp_msg.identifier = mcp_id;
        temp_msg.data_length_code = mcp_len;
        memcpy(temp_msg.data, mcp_data, mcp_len);
        parseModuleMessage(temp_msg);
      }
    }
  }

  // ========== Read from TWAI (Slave Modules) ==========
  // Slave modules send responses (0x100-0x1FF) back
  twai_message_t twai_msg;
  while (twai_receive(&twai_msg, 0) == ESP_OK)
  {
    uint32_t id = twai_msg.identifier;

    if (canDebugEnabled)
    {
      static uint32_t lastTwaiDebug = 0;
      if (millis() - lastTwaiDebug > 100)
      {
        lastTwaiDebug = millis();
        Serial.printf("[TWAI RX] 0x%03X [%d] ", id, twai_msg.data_length_code);
        for (int i = 0; i < twai_msg.data_length_code; i++)
        {
          Serial.printf("%02X ", twai_msg.data[i]);
        }
        Serial.println();
      }
    }

    // Parse module data for monitoring (0x100-0x1FF)
    if ((id & 0xF00) == 0x100)
    {
      parseModuleMessage(twai_msg);
      
      // Forward slave module response back to BMS via MCP2515
      if (mcp2515_available)
      {
        sendCAN2(id, twai_msg.data_length_code, twai_msg.data);
        
        if (canDebugEnabled)
        {
          static uint32_t lastGatewayDebug = 0;
          if (millis() - lastGatewayDebug > 100)
          {
            lastGatewayDebug = millis();
            Serial.printf("[GATEWAY] Slave→BMS: 0x%03X forwarded\n", id);
          }
        }
      }
    }
  }
}

// Calculate lowest cell voltage across all modules
float getLowestCellVoltage()
{
  float lowest = 5.0f;
  for (int m = 0; m < MAX_MODULES; m++)
  {
    if (!modules[m].exists)
      continue;
    for (int c = 0; c < CELLS_PER_MODULE; c++)
    {
      if (modules[m].cellVoltages[c] > 0.5f && modules[m].cellVoltages[c] < lowest)
      {
        lowest = modules[m].cellVoltages[c];
      }
    }
  }

  return lowest;
}

// Calculate highest cell voltage across all modules
float getHighestCellVoltage()
{
  float highest = 0.0f;
  for (int m = 0; m < MAX_MODULES; m++)
  {
    if (!modules[m].exists)
      continue;
    for (int c = 0; c < CELLS_PER_MODULE; c++)
    {
      if (modules[m].cellVoltages[c] > highest)
      {
        highest = modules[m].cellVoltages[c];
      }
    }
  }

  return highest;
}

// Send balancing command AND voltage requests to slave modules via TWAI
// When balancing: Replaces BMS requests with our own (balance + data request)
// When not balancing: Does nothing (BMS requests are forwarded instead)
void sendBalancingCommand()
{
  // GATEWAY MODE: Only send when balancing is active
  // When not balancing, BMS requests are forwarded through gateway
  if (!balancingActive)
    return;

  // Throttle sending - send every 50ms
  if (millis() - lastCommandTime < CAN_COMMAND_INTERVAL_MS)
    return;

  // Cache target voltage to avoid calling getLowestCellVoltage() during critical operations
  static float cachedLowestVoltage = 4.0f;
  static uint32_t lastVoltageCacheUpdate = 0;

  // Update cached voltage every 500ms (not every send)
  if (millis() - lastVoltageCacheUpdate > 500)
  {
    cachedLowestVoltage = getLowestCellVoltage();
    lastVoltageCacheUpdate = millis();
  }

  // Cycle through message counter
  if (messageCounter == 0xF)
    messageCounter = 0;

  // Build command message with BOTH balancing target AND data request
  // This replaces the BMS request that we're blocking during balancing
  twai_message_t msg;
  msg.identifier = 0x080 | nextMessage;  // Send to next module (0-7)
  msg.data_length_code = 8;
  msg.flags = TWAI_MSG_FLAG_NONE;
  msg.extd = 0;
  msg.rtr = 0;

  // Byte 0-1: Target voltage for balancing (lowest cell + 2mV)
  targetBalanceVoltage = cachedLowestVoltage;
  uint16_t targetVoltage_mV = (uint16_t)((targetBalanceVoltage * 1000) + 2);
  msg.data[0] = targetVoltage_mV & 0xFF;
  msg.data[1] = (targetVoltage_mV >> 8) & 0xFF;

  // Cycle through different request types like BMW BMS does
  // This ensures modules send back voltage, temperature, and balance status
  static uint8_t requestCycle = 0;

  // Build request pattern - CRITICAL: This requests cell voltages from modules
  if (requestCycle == 0)
  {
    msg.data[2] = 0xFF;
    msg.data[3] = 0x5F; // Request voltage + temp + balance status
    msg.data[4] = 0x08; // Enable balancing
  }
  else if (requestCycle == 1)
  {
    msg.data[2] = 0xFF;
    msg.data[3] = 0x0F; // Request basic data
    msg.data[4] = 0x08; // Enable balancing
  }
  else
  {
    msg.data[2] = 0xFF;
    msg.data[3] = 0x5F; // Request voltage + temp + balance status
    msg.data[4] = 0x08; // Enable balancing
  }

  requestCycle = (requestCycle + 1) % 3;

  msg.data[5] = 0x00;
  msg.data[6] = messageCounter << 4;
  msg.data[7] = calculateChecksum(msg, nextMessage);

  // GATEWAY MODE: Send balancing command via TWAI to slave modules
  esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(10));
  bool sendSuccess = (result == ESP_OK);

  // Always show TX debug when debug is enabled (rate limited)
  if (canDebugEnabled)
  {
    static uint32_t lastTxDebug = 0;
    if (millis() - lastTxDebug > 200) // Show TX every 200ms
    {
      lastTxDebug = millis();
      if (sendSuccess)
      {
        Serial.printf("[BALANCE TX] 0x%03X [8] ", msg.identifier);
        for (int i = 0; i < 8; i++)
        {
          Serial.printf("%02X ", msg.data[i]);
        }
        Serial.printf(" Target: %.3fV (balance + request voltages)\n", targetVoltage_mV / 1000.0f);
      }
      else
      {
        Serial.println("✗ Balance TX FAILED");
      }
    }
  }
  else if (!sendSuccess)
  {
    Serial.println("✗ Failed to send balancing command via TWAI");
  }

  // Update lastCommandTime AFTER successful transmission
  lastCommandTime = millis();

  // Increment message counter
  messageCounter++;
  nextMessage = (nextMessage + 1) % 8;
}

// Update balancing logic
void updateBalancing()
{
  // Detect external master by checking if modules are sending frequent updates
  // This is only for display purposes - we still balance even if external master is active
  bool externalMasterActive = false;
  for (int m = 0; m < MAX_MODULES; m++)
  {
    if (modules[m].exists && (millis() - modules[m].lastUpdate < 2000))
    {
      externalMasterActive = true;
      break;
    }
  }

  if (externalMasterActive)
  {
    externalMasterDetected = true;
    lastExternalCommandTime = millis();
  }
  else if (millis() - lastExternalCommandTime > 60000) // Hold external BMS mode for 60 seconds
  {
    externalMasterDetected = false;
  }

  // Don't do auto-balancing in manual mode
  if (manualMode)
    return;

  float lowestVoltage = getLowestCellVoltage();
  float highestVoltage = getHighestCellVoltage();
  float difference_mV = (highestVoltage - lowestVoltage) * 1000.0f;

  // Only start balancing if highest cell is above minimum voltage
  if (!balancingActive && difference_mV > BALANCE_THRESHOLD_MV && highestVoltage > MIN_BALANCE_VOLTAGE)
  {
    // Start balancing
    balancingActive = true;
    targetBalanceVoltage = lowestVoltage;
    Serial.printf("Starting balancing: Lowest=%.3fV, Highest=%.3fV, Diff=%.1fmV\n",
                  lowestVoltage, highestVoltage, difference_mV);
  }
  else if (balancingActive && difference_mV < BALANCE_HYSTERESIS_MV)
  {
    // Stop balancing
    balancingActive = false;
    Serial.printf("Stopping balancing: Cells balanced within %.1fmV\n", difference_mV);
  }
  else if (balancingActive && highestVoltage < MIN_BALANCE_VOLTAGE)
  {
    // Stop balancing if voltage drops too low
    balancingActive = false;
    Serial.printf("Stopping balancing: Cell voltage too low (%.3fV)\n", highestVoltage);
  }

  // Update target voltage during balancing
  if (balancingActive)
  {
    targetBalanceVoltage = lowestVoltage;
  }
}

// Update LED based on status
void updateLED()
{
  // Skip if LED is not available
  if (!ledAvailable)
    return;

  if (millis() - lastLedUpdate < 50) // Update every 50ms
    return;
  lastLedUpdate = millis();

  // Check for errors (no modules detected)
  bool hasError = true;
  for (int m = 0; m < MAX_MODULES; m++)
  {
    if (modules[m].exists && (millis() - modules[m].lastUpdate < 5000))
    {
      hasError = false;
      break;
    }
  }

  if (hasError && (millis() > 5000)) // Give 5 seconds at startup
  {
    // Red pulsing - Error (no modules) - HIGHEST PRIORITY
    ledBrightness = (millis() / 10) % 256;
    if (ledBrightness > 127)
      ledBrightness = 255 - ledBrightness;
    ledBrightness = ledBrightness / 3; // Max brightness 40
    led.setPixelColor(0, led.Color(ledBrightness, 0, 0));
    led.show();
    return;
  }

  if (balancingActive)
  {
    // Blue pulsing - Balancing active
    ledBrightness = (millis() / 10) % 256;
    if (ledBrightness > 127)
      ledBrightness = 255 - ledBrightness;
    ledBrightness = ledBrightness / 3; // Max brightness 40
    led.setPixelColor(0, led.Color(0, 0, ledBrightness));
  }
  else
  {
    // Green solid - Standby/OK
    led.setPixelColor(0, led.Color(0, 20, 0));
  }

  led.show();
}

// WebSocket event handler
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  if (type == WS_EVT_CONNECT)
  {
    Serial.printf("WebSocket client #%u connected\n", client->id());
  }
  else if (type == WS_EVT_DISCONNECT)
  {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  }
  else if (type == WS_EVT_DATA)
  {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len)
    {
      data[len] = 0;
      String message = (char *)data;

      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, message);

      if (!error)
      {
        const char *command = doc["command"];

        if (strcmp(command, "start") == 0)
        {
          balancingActive = true;
          manualMode = true;
          targetBalanceVoltage = getLowestCellVoltage();
          Serial.println("Manual start - balancing active (blocks BMS requests)");
        }
        else if (strcmp(command, "stop") == 0)
        {
          balancingActive = false;
          manualMode = true;
          Serial.println("Manual stop - gateway mode (forwards BMS requests)");
        }
        else if (strcmp(command, "auto") == 0)
        {
          manualMode = false;
          Serial.println("Auto mode - smart balancing enabled");
        }
        else if (strcmp(command, "gateway") == 0)
        {
          balancingActive = false;
          manualMode = true;
          Serial.println("Gateway mode - forwarding BMS requests");
        }
        else if (strcmp(command, "restart") == 0)
        {
          Serial.println("Restart command received - restarting ESP32...");
          delay(500);
          ESP.restart();
        }
        else if (strcmp(command, "toggleDebug") == 0)
        {
          canDebugEnabled = !canDebugEnabled;
          Serial.printf("CAN debug logging %s\n", canDebugEnabled ? "enabled" : "disabled");
        }
        else if (strcmp(command, "setThreshold") == 0)
        {
          // Future: set threshold
        }
      }
    }
  }
}

// Send data to WebSocket clients - REQUEST from Core 0
void broadcastData()
{
  static uint32_t lastBroadcast = 0;
  if (millis() - lastBroadcast < 15000)
    return; // Broadcast every 15000ms (15 seconds)
  lastBroadcast = millis();

  // Simply send a trigger to Core 1 to do the actual broadcast
  // This avoids calling ws.textAll() from Core 0 which causes mutex issues
  uint8_t trigger = 1;
  if (broadcastQueue != NULL)
  {
    xQueueSend(broadcastQueue, &trigger, 0); // Don't block
  }
}

// Actually send data to WebSocket clients - CALLED from Core 1 only
void performBroadcast()
{
  JsonDocument doc;
  String jsonString;

  // Direct access - float reads are atomic enough on ESP32
  float lowestVoltage = 5.0f;
  float highestVoltage = 0.0f;

  for (int m = 0; m < MAX_MODULES; m++)
  {
    if (!modules[m].exists)
      continue;

    for (int c = 0; c < CELLS_PER_MODULE; c++)
    {
      if (modules[m].cellVoltages[c] > 0.5f && modules[m].cellVoltages[c] < lowestVoltage)
      {
        lowestVoltage = modules[m].cellVoltages[c];
      }
      if (modules[m].cellVoltages[c] > highestVoltage)
      {
        highestVoltage = modules[m].cellVoltages[c];
      }
    }
  }

  // Check for errors (no modules detected after 5 seconds)
  bool hasError = true;
  for (int m = 0; m < MAX_MODULES; m++)
  {
    if (modules[m].exists && (millis() - modules[m].lastUpdate < 5000))
    {
      hasError = false;
      break;
    }
  }

  // Determine status
  String status;
  if (hasError && (millis() > 5000))
  {
    status = "ERROR";
  }
  else if (balancingActive)
  {
    status = "BALANCING";
  }
  else
  {
    status = "GATEWAY";
  }

  // Determine mode
  String mode;
  if (manualMode)
  {
    mode = "MANUAL";
  }
  else
  {
    mode = "AUTO";
  }

  doc["status"] = status;
  doc["mode"] = mode;
  doc["targetVoltage"] = targetBalanceVoltage;
  doc["lowestVoltage"] = lowestVoltage;
  doc["highestVoltage"] = highestVoltage;
  doc["difference"] = (highestVoltage - lowestVoltage) * 1000.0f;
  doc["gatewayMode"] = gatewayMode;
  doc["externalMaster"] = externalMasterDetected && (millis() - lastExternalCommandTime < 60000);
  doc["mcp2515Available"] = mcp2515_available;
  doc["uptime"] = millis() / 1000;

  JsonArray modulesArray = doc["modules"].to<JsonArray>();

  for (int m = 0; m < MAX_MODULES; m++)
  {
    if (!modules[m].exists)
      continue;

    JsonObject moduleObj = modulesArray.add<JsonObject>();
    moduleObj["id"] = m + 1;
    moduleObj["balancing"] = (modules[m].balanceStatus != 0);
    moduleObj["error"] = modules[m].errorCode;

    JsonArray cellsArray = moduleObj["cells"].to<JsonArray>();
    for (int c = 0; c < CELLS_PER_MODULE; c++)
    {
      if (modules[m].cellVoltages[c] > 0.5f)
      {
        cellsArray.add(modules[m].cellVoltages[c]);
      }
    }
  }

  serializeJson(doc, jsonString);

  // Send data
  ws.textAll(jsonString);
}

// HTML page
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>BMW i3 Balancing Controller</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif;
            background: linear-gradient(135deg, #1e3c72 0%, #2a5298 100%);
            color: #fff;
            padding: 20px;
        }
        .container { max-width: 1200px; margin: 0 auto; }
        h1 { 
            text-align: center; 
            margin-bottom: 30px;
            font-size: 2.5em;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        .status-bar {
            background: rgba(255,255,255,0.1);
            backdrop-filter: blur(10px);
            border-radius: 15px;
            padding: 20px;
            margin-bottom: 20px;
            display: flex;
            justify-content: space-around;
            flex-wrap: wrap;
            gap: 15px;
        }
        .stat-item {
            text-align: center;
            flex: 1;
            min-width: 150px;
        }
        .stat-label {
            font-size: 0.9em;
            opacity: 0.8;
            margin-bottom: 5px;
        }
        .stat-value {
            font-size: 1.8em;
            font-weight: bold;
        }
        .stat-value.good { color: #4ade80; }
        .stat-value.warning { color: #fbbf24; }
        .stat-value.danger { color: #ef4444; }
        .controls {
            background: rgba(255,255,255,0.1);
            backdrop-filter: blur(10px);
            border-radius: 15px;
            padding: 20px;
            margin-bottom: 20px;
            text-align: center;
        }
        button {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            border: none;
            color: white;
            padding: 15px 30px;
            margin: 5px;
            border-radius: 10px;
            font-size: 1.1em;
            cursor: pointer;
            transition: transform 0.2s, box-shadow 0.2s;
        }
        button:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(0,0,0,0.3);
        }
        button:active {
            transform: translateY(0);
        }
        button.stop { background: linear-gradient(135deg, #ef4444 0%, #dc2626 100%); }
        button.auto { background: linear-gradient(135deg, #10b981 0%, #059669 100%); }
        button.restart { background: linear-gradient(135deg, #f59e0b 0%, #d97706 100%); font-size: 0.9em; }
        button.ota { background: linear-gradient(135deg, #8b5cf6 0%, #7c3aed 100%); font-size: 0.9em; }
        #otaFile { display: none; }
        .status-led-container {
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 10px;
            margin: 15px 0;
        }
        .status-led {
            width: 20px;
            height: 20px;
            border-radius: 50%;
            border: 2px solid rgba(255,255,255,0.3);
            box-shadow: 0 0 10px rgba(0,0,0,0.5);
            transition: background-color 0.3s, box-shadow 0.3s;
        }
        .status-led.green {
            background-color: #22c55e;
            box-shadow: 0 0 15px #22c55e;
        }
        .status-led.blue {
            background-color: #3b82f6;
            box-shadow: 0 0 15px #3b82f6;
            animation: pulse 1.5s ease-in-out infinite;
        }
        .status-led.red {
            background-color: #ef4444;
            box-shadow: 0 0 15px #ef4444;
        }
        @keyframes pulse {
            0%, 100% { opacity: 1; transform: scale(1); }
            50% { opacity: 0.6; transform: scale(0.9); }
        }
        .status-led-label {
            font-size: 0.9em;
            opacity: 0.8;
        }
        .modules {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 15px;
        }
        .module {
            background: rgba(255,255,255,0.1);
            backdrop-filter: blur(10px);
            border-radius: 15px;
            padding: 15px;
        }
        .module-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 10px;
            padding-bottom: 10px;
            border-bottom: 1px solid rgba(255,255,255,0.2);
        }
        .module-title {
            font-size: 1.2em;
            font-weight: bold;
        }
        .balance-indicator {
            padding: 5px 15px;
            border-radius: 20px;
            font-size: 0.85em;
            font-weight: bold;
        }
        .balance-indicator.active {
            background: #4ade80;
            color: #000;
            animation: pulse 2s infinite;
        }
        .balance-indicator.inactive {
            background: rgba(255,255,255,0.2);
        }
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.6; }
        }
        .cells {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 8px;
            margin-top: 10px;
        }
        .cell {
            background: rgba(0,0,0,0.3);
            padding: 12px;
            border-radius: 8px;
            text-align: center;
            transition: all 0.3s;
        }
        .cell:hover {
            background: rgba(0,0,0,0.5);
            transform: scale(1.05);
        }
        .cell-label {
            font-size: 0.75em;
            opacity: 0.7;
            margin-bottom: 4px;
        }
        .cell-voltage {
            font-weight: bold;
            font-size: 1.3em;
        }
        .cell-voltage.lowest {
            color: #fbbf24;
            text-shadow: 0 0 10px rgba(251, 191, 36, 0.5);
        }
        .cell-voltage.highest {
            color: #60a5fa;
            text-shadow: 0 0 10px rgba(96, 165, 250, 0.5);
        }
        .top-right-buttons {
            position: fixed;
            top: 20px;
            right: 20px;
            display: flex;
            gap: 10px;
            align-items: center;
        }
        .connection-status {
            padding: 10px 20px;
            border-radius: 10px;
            font-weight: bold;
            backdrop-filter: blur(10px);
        }
        .connection-status.connected {
            background: rgba(74, 222, 128, 0.3);
            color: #4ade80;
        }
        .connection-status.disconnected {
            background: rgba(239, 68, 68, 0.3);
            color: #ef4444;
        }
        .debug-button {
            position: fixed;
            bottom: 20px;
            right: 20px;
            z-index: 1000;
        }
        .debug-button button {
            background: linear-gradient(135deg, #6366f1 0%, #4f46e5 100%);
            font-size: 0.9em;
            padding: 12px 24px;
        }
        .cell-chart-container {
            background: rgba(255,255,255,0.1);
            backdrop-filter: blur(10px);
            border-radius: 15px;
            padding: 20px;
            margin-top: 20px;
        }
        .chart-title {
            text-align: center;
            font-size: 1.3em;
            font-weight: bold;
            margin-bottom: 15px;
        }
        .cell-chart {
            display: flex;
            align-items: flex-end;
            justify-content: space-between;
            height: 200px;
            background: rgba(0,0,0,0.3);
            border-radius: 10px;
            padding: 10px;
            gap: 2px;
            overflow-x: auto;
        }
        .cell-bar {
            flex: 1;
            min-width: 8px;
            background: linear-gradient(to top, #3b82f6, #60a5fa);
            border-radius: 3px 3px 0 0;
            transition: all 0.3s;
            cursor: pointer;
            position: relative;
        }
        .cell-bar:hover {
            background: linear-gradient(to top, #2563eb, #3b82f6);
            transform: scaleX(1.5);
            z-index: 10;
        }
        .cell-bar.lowest {
            background: linear-gradient(to top, #f59e0b, #fbbf24);
        }
        .cell-bar.highest {
            background: linear-gradient(to top, #10b981, #34d399);
        }
        .cell-bar-tooltip {
            position: absolute;
            bottom: 100%;
            left: 50%;
            transform: translateX(-50%);
            background: rgba(0,0,0,0.9);
            color: white;
            padding: 5px 10px;
            border-radius: 5px;
            font-size: 0.75em;
            white-space: nowrap;
            opacity: 0;
            pointer-events: none;
            transition: opacity 0.2s;
            margin-bottom: 5px;
        }
        .cell-bar:hover .cell-bar-tooltip {
            opacity: 1;
        }
        .chart-legend {
            display: flex;
            justify-content: center;
            gap: 20px;
            margin-top: 15px;
            font-size: 0.9em;
        }
        .legend-item {
            display: flex;
            align-items: center;
            gap: 8px;
        }
        .legend-color {
            width: 20px;
            height: 12px;
            border-radius: 3px;
        }
        .chart-info {
            text-align: center;
            margin-top: 10px;
            font-size: 0.9em;
            opacity: 0.8;
        }
    </style>
</head>
<body>
    <div class="top-right-buttons">
        <input type="file" id="otaFile" accept=".bin" onchange="uploadOTA()">
        <button class="ota" onclick="document.getElementById('otaFile').click()">⬆️ OTA Update</button>
        <button class="restart" onclick="restartDevice()">🔄 Restart</button>
        <div class="connection-status" id="connectionStatus">Connecting...</div>
    </div>
    
    <div class="debug-button">
        <button id="debugBtn" onclick="toggleDebug()">🐛 CAN Debug: OFF</button>
    </div>
    
    <div class="container">
        <h1>⚡ BMW i3 Balancing Controller</h1>
        
        <div class="status-led-container">
            <div id="statusLed" class="status-led green"></div>
            <span id="statusLedLabel" class="status-led-label">System OK</span>
        </div>
        
        <div class="status-bar">
            <div class="stat-item">
                <div class="stat-label">Status</div>
                <div class="stat-value" id="status">IDLE</div>
            </div>
            <div class="stat-item">
                <div class="stat-label">Mode</div>
                <div class="stat-value" id="mode">AUTO</div>
            </div>
            <div class="stat-item">
                <div class="stat-label">Lowest Cell</div>
                <div class="stat-value good" id="lowestVoltage">-.-V</div>
            </div>
            <div class="stat-item">
                <div class="stat-label">Highest Cell</div>
                <div class="stat-value good" id="highestVoltage">-.-V</div>
            </div>
            <div class="stat-item">
                <div class="stat-label">Difference</div>
                <div class="stat-value" id="difference">-- mV</div>
            </div>
        </div>
        
        <div class="controls">
            <button onclick="sendCommand('start')">▶ Start Balancing</button>
            <button class="stop" onclick="sendCommand('stop')">⏹ Stop / Gateway</button>
            <button class="auto" onclick="sendCommand('auto')">🔄 Auto Mode</button>
            <button onclick="sendCommand('gateway')">🔀 Gateway Mode</button>
        </div>
        
        <div class="modules" id="modules">
            <!-- Modules will be populated here -->
        </div>
        
        <div class="cell-chart-container">
            <div class="chart-title">All Cell Voltages (96 cells)</div>
            <div class="cell-chart" id="cellChart">
                <!-- Cell bars will be populated here -->
            </div>
            <div class="chart-legend">
                <div class="legend-item">
                    <div class="legend-color" style="background: linear-gradient(to right, #f59e0b, #fbbf24);"></div>
                    <span>Lowest</span>
                </div>
                <div class="legend-item">
                    <div class="legend-color" style="background: linear-gradient(to right, #3b82f6, #60a5fa);"></div>
                    <span>Normal</span>
                </div>
                <div class="legend-item">
                    <div class="legend-color" style="background: linear-gradient(to right, #10b981, #34d399);"></div>
                    <span>Highest</span>
                </div>
            </div>
            <div class="chart-info" id="chartInfo">Hover over bars to see details</div>
        </div>
    </div>
    
    <script>
        let ws;
        
        function connectWebSocket() {
            ws = new WebSocket('ws://' + window.location.hostname + '/ws');
            
            ws.onopen = () => {
                console.log('WebSocket connected');
                document.getElementById('connectionStatus').textContent = 'Connected';
                document.getElementById('connectionStatus').className = 'connection-status connected';
            };
            
            ws.onclose = () => {
                console.log('WebSocket disconnected');
                document.getElementById('connectionStatus').textContent = 'Disconnected';
                document.getElementById('connectionStatus').className = 'connection-status disconnected';
                setTimeout(connectWebSocket, 3000);
            };
            
            ws.onmessage = (event) => {
                const data = JSON.parse(event.data);
                updateUI(data);
            };
        }
        
        function updateUI(data) {
            const statusElement = document.getElementById('status');
            statusElement.textContent = data.status;
            
            // Color code status
            statusElement.className = 'stat-value';
            if (data.status === 'BALANCING') statusElement.className += ' good';
            else if (data.status === 'GATEWAY') statusElement.className += ' warning';
            
            // Update LED indicator
            const statusLed = document.getElementById('statusLed');
            const statusLedLabel = document.getElementById('statusLedLabel');
            
            statusLed.className = 'status-led';
            if (data.status === 'ERROR') {
                statusLed.className += ' red';
                statusLedLabel.textContent = 'Error';
            } else if (data.status === 'BALANCING') {
                statusLed.className += ' blue';
                statusLedLabel.textContent = 'Balancing';
            } else {
                statusLed.className += ' green';
                statusLedLabel.textContent = 'System OK';
            }
            
            document.getElementById('mode').textContent = data.mode;
            document.getElementById('lowestVoltage').textContent = data.lowestVoltage.toFixed(3) + 'V';
            document.getElementById('highestVoltage').textContent = data.highestVoltage.toFixed(3) + 'V';
            
            const diff = data.difference;
            const diffElement = document.getElementById('difference');
            diffElement.textContent = diff.toFixed(1) + ' mV';
            
            if (diff < 10) diffElement.className = 'stat-value good';
            else if (diff < 30) diffElement.className = 'stat-value warning';
            else diffElement.className = 'stat-value danger';
            
            const modulesContainer = document.getElementById('modules');
            modulesContainer.innerHTML = '';
            
            data.modules.forEach(module => {
                const moduleDiv = document.createElement('div');
                moduleDiv.className = 'module';
                
                const headerDiv = document.createElement('div');
                headerDiv.className = 'module-header';
                
                const titleDiv = document.createElement('div');
                titleDiv.className = 'module-title';
                titleDiv.textContent = 'Module ' + module.id;
                
                const indicatorDiv = document.createElement('div');
                indicatorDiv.className = 'balance-indicator ' + (module.balancing ? 'active' : 'inactive');
                indicatorDiv.textContent = module.balancing ? 'BALANCING' : 'IDLE';
                
                headerDiv.appendChild(titleDiv);
                headerDiv.appendChild(indicatorDiv);
                
                const cellsDiv = document.createElement('div');
                cellsDiv.className = 'cells';
                
                // Find lowest and highest voltage in this module
                let minVoltage = Math.min(...module.cells);
                let maxVoltage = Math.max(...module.cells);
                
                module.cells.forEach((voltage, index) => {
                    const cellDiv = document.createElement('div');
                    cellDiv.className = 'cell';
                    
                    const labelDiv = document.createElement('div');
                    labelDiv.className = 'cell-label';
                    labelDiv.textContent = 'Cell ' + (index + 1);
                    
                    const voltageDiv = document.createElement('div');
                    voltageDiv.className = 'cell-voltage';
                    
                    // Highlight lowest and highest cells
                    if (voltage === minVoltage && minVoltage !== maxVoltage) {
                        voltageDiv.className += ' lowest';
                    } else if (voltage === maxVoltage && minVoltage !== maxVoltage) {
                        voltageDiv.className += ' highest';
                    }
                    
                    voltageDiv.textContent = voltage.toFixed(3) + 'V';
                    
                    cellDiv.appendChild(labelDiv);
                    cellDiv.appendChild(voltageDiv);
                    cellsDiv.appendChild(cellDiv);
                });
                
                moduleDiv.appendChild(headerDiv);
                moduleDiv.appendChild(cellsDiv);
                modulesContainer.appendChild(moduleDiv);
            });
            
            // Update cell voltage chart
            updateCellChart(data.modules);
        }
        
        function updateCellChart(modules) {
            const chartContainer = document.getElementById('cellChart');
            chartContainer.innerHTML = '';
            
            // Collect all cell voltages
            let allCells = [];
            modules.forEach(module => {
                module.cells.forEach((voltage, cellIndex) => {
                    allCells.push({
                        moduleId: module.id,
                        cellIndex: cellIndex + 1,
                        voltage: voltage
                    });
                });
            });
            
            if (allCells.length === 0) return;
            
            // Find min and max voltages
            const voltages = allCells.map(c => c.voltage);
            const minVoltage = Math.min(...voltages);
            const maxVoltage = Math.max(...voltages);
            const voltageRange = maxVoltage - minVoltage;
            
            // Create bars for each cell
            allCells.forEach(cell => {
                const bar = document.createElement('div');
                bar.className = 'cell-bar';
                
                // Calculate height (percentage of max voltage)
                const heightPercent = voltageRange > 0 
                    ? ((cell.voltage - minVoltage) / voltageRange) * 100 
                    : 100;
                bar.style.height = Math.max(5, heightPercent) + '%';
                
                // Highlight lowest and highest
                if (cell.voltage === minVoltage) {
                    bar.classList.add('lowest');
                } else if (cell.voltage === maxVoltage) {
                    bar.classList.add('highest');
                }
                
                // Add tooltip
                const tooltip = document.createElement('div');
                tooltip.className = 'cell-bar-tooltip';
                tooltip.textContent = `M${cell.moduleId} C${cell.cellIndex}: ${cell.voltage.toFixed(3)}V`;
                bar.appendChild(tooltip);
                
                chartContainer.appendChild(bar);
            });
            
            // Update chart info
            const chartInfo = document.getElementById('chartInfo');
            chartInfo.textContent = `${allCells.length} cells | Range: ${minVoltage.toFixed(3)}V - ${maxVoltage.toFixed(3)}V | Δ${(voltageRange * 1000).toFixed(1)}mV`;
        }
        
        function sendCommand(cmd) {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({command: cmd}));
            }
        }
        
        function restartDevice() {
            if (confirm('Er du sikker på at du vil genstarte enheden?')) {
                sendCommand('restart');
                document.getElementById('connectionStatus').textContent = 'Restarting...';
                document.getElementById('connectionStatus').className = 'connection-status disconnected';
            }
        }
        
        let canDebugEnabled = false;
        function toggleDebug() {
            canDebugEnabled = !canDebugEnabled;
            sendCommand('toggleDebug');
            const btn = document.getElementById('debugBtn');
            btn.textContent = canDebugEnabled ? '🐛 CAN Debug: ON' : '🐛 CAN Debug: OFF';
        }
        
        function uploadOTA() {
            const fileInput = document.getElementById('otaFile');
            const file = fileInput.files[0];
            
            if (!file) return;
            
            if (!file.name.endsWith('.bin')) {
                alert('Vælg venligst en .bin fil');
                return;
            }
            
            if (!confirm('Upload ny firmware? Enheden vil genstarte efter upload.')) {
                fileInput.value = '';
                return;
            }
            
            const statusEl = document.getElementById('connectionStatus');
            statusEl.textContent = 'Uploading...';
            statusEl.className = 'connection-status disconnected';
            
            const formData = new FormData();
            formData.append('update', file);
            
            fetch('/update', {
                method: 'POST',
                body: formData
            })
            .then(response => response.text())
            .then(data => {
                statusEl.textContent = 'Update OK - Restarting...';
                fileInput.value = '';
                setTimeout(() => {
                    location.reload();
                }, 3000);
            })
            .catch(error => {
                statusEl.textContent = 'Update Failed!';
                statusEl.className = 'connection-status disconnected';
                alert('OTA update fejlede: ' + error);
                fileInput.value = '';
            });
        }
        
        connectWebSocket();
    </script>
</body>
</html>
)rawliteral";

// Forward declarations
void canGatewayTask(void *parameter);

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\n=================================");
  Serial.println("BMW i3 Balancing Controller");
  Serial.println("LilyGO T-CAN485 + MCP2515");
  Serial.println("Dual CAN Gateway Mode");
  Serial.println("=================================\n");

  // Pre-initialize MCP2515 pins before any library calls
  initMCP2515Pins();

  // Initialize LED (non-critical - continue if fails)
  // T-2CAN doesn't have onboard LED
  #ifndef LILYGO_T2CAN
  Serial.println("Initializing LED...");
  led.begin();

  // Test if LED is working - if not, just continue without it
  led.setPixelColor(0, led.Color(20, 20, 0)); // Yellow during startup
  if (!led.canShow())
  {
    Serial.println("⚠ LED initialization failed - continuing without LED");
    ledAvailable = false;
  }
  else
  {
    led.show();
    ledAvailable = true;
    Serial.println("✓ LED initialized");
  }
  #else
  Serial.println("⚠ T-2CAN board has no onboard LED - LED disabled");
  ledAvailable = false;
  #endif

  // Initialize TWAI CAN bus (Slave modules side)
  Serial.println("Initializing TWAI CAN bus (Slave modules)...");
  initCAN();

  // Initialize MCP2515 CAN bus (BMS side)
  Serial.println("Initializing MCP2515 CAN bus (BMS side)...");
  if (initMCP2515())
  {
    Serial.println("✓ MCP2515 initialized - gateway mode active");
  }
  else
  {
    Serial.println("⚠ MCP2515 not detected - limited functionality");
  }
  Serial.println("✓ Gateway ready");

  // Clear any initial CAN messages in buffers
  Serial.println("Clearing CAN buffers...");
  twai_message_t temp_msg;
  int cleared = 0;
  while (twai_receive(&temp_msg, 0) == ESP_OK)
  {
    cleared++;
  }
  if (cleared > 0)
  {
    Serial.printf("Cleared %d TWAI messages from buffer\n", cleared);
  }

  // Connect to WiFi Router
  Serial.println("\nConnecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30)
  {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\n✓ WiFi connected!");
    IPAddress IP = WiFi.localIP();
    Serial.print("IP address: ");
    Serial.println(IP);
    Serial.printf("Open browser: http://%s\n", IP.toString().c_str());
  }
  else
  {
    Serial.println("\n✗ WiFi connection failed!");
    Serial.println("Please check SSID and password in code");
  }

  // Setup OTA updates
  Serial.println("\nConfiguring OTA updates...");
  ArduinoOTA.setHostname("BMW-i3-Balancer");
  ArduinoOTA.setPassword("bmw123"); // Change to secure password

  ArduinoOTA.onStart([]()
                     {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {
      type = "filesystem";
    }
    Serial.println("OTA: Start updating " + type); });

  ArduinoOTA.onEnd([]()
                   { Serial.println("\nOTA: Update complete!"); });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        {
    static unsigned int lastPercent = 0;
    unsigned int percent = (progress / (total / 100));
    if (percent != lastPercent && percent % 10 == 0) {
      Serial.printf("OTA Progress: %u%%\n", percent);
      lastPercent = percent;
    }
    // LED purple pulsing during OTA
    uint8_t brightness = (millis() / 10) % 256;
    if (brightness > 127) brightness = 255 - brightness;
    brightness = brightness / 6;
    led.setPixelColor(0, led.Color(brightness, 0, brightness));
    led.show(); });

  ArduinoOTA.onError([](ota_error_t error)
                     {
    Serial.printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed"); });

  ArduinoOTA.begin();
  Serial.println("✓ OTA ready");

  // Setup WebSocket
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  // Setup web server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/html", index_html); });

  // OTA update endpoint
  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request)
            {
      bool shouldReboot = !Update.hasError();
      AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", 
        shouldReboot ? "OK" : "FAIL");
      response->addHeader("Connection", "close");
      request->send(response);
      if (shouldReboot) {
        delay(100);
        ESP.restart();
      } }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
            {
      if (!index) {
        Serial.printf("OTA Update Start: %s\n", filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
          Update.printError(Serial);
        }
      }
      if (Update.write(data, len) != len) {
        Update.printError(Serial);
      }
      if (final) {
        if (Update.end(true)) {
          Serial.printf("OTA Update Success: %u bytes\n", index + len);
        } else {
          Update.printError(Serial);
        }
      } });

  server.begin();
  Serial.println("Web server started");

  // Create broadcast queue for Core 0 -> Core 1 communication (avoid cross-core mutex)
  broadcastQueue = xQueueCreate(5, sizeof(uint8_t));
  if (broadcastQueue == NULL)
  {
    Serial.println("⚠ Failed to create broadcast queue");
  }

  Serial.println("\n✓ Setup complete!");
  Serial.println("\nGateway Architecture:");
  Serial.println("  MCP2515 (BMS) <-> ESP32 <-> TWAI (Slave Modules)");
  Serial.println("  Normal: Forward BMS requests → slaves, responses → BMS");
  Serial.println("  Balancing: Block BMS, send own commands, forward responses\n");

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.printf("Access the interface at: http://%s\n\n", WiFi.localIP().toString().c_str());
  }

  // Create Gateway Controller task on Core 0 (WiFi/WebSocket runs on Core 1)
  xTaskCreatePinnedToCore(
      canGatewayTask,        // Task function
      "Gateway_Controller",  // Task name
      8192,                  // Stack size (8KB)
      NULL,                  // Parameters
      1,                     // Priority (1 to avoid conflicts with WiFi tasks)
      &canGatewayTaskHandle, // Task handle
      0                      // Core 0 (Core 1 is used for WiFi/WebSocket)
  );

  Serial.println("\n✓ Gateway Controller task started on Core 0");
  Serial.println("✓ Web UI running on Core 1\n");
}

// Gateway Controller Task - Runs on Core 0
void canGatewayTask(void *parameter)
{
  Serial.println("[Core 0] Gateway Controller task running...");

  while (true)
  {
    // Read module data from TWAI
    readCANMessages();

    // Send data requests and balancing commands
    sendBalancingCommand();

    // Update balancing logic (auto mode)
    updateBalancing();

    // Small yield to prevent watchdog timeout and reduce mutex contention
    vTaskDelay(5); // 5ms delay (FreeRTOS) - gives more breathing room
  }
}

void loop()
{
  // Core 1: Handle Web UI, WiFi, and OTA (no CAN processing here)

  // Handle OTA updates
  ArduinoOTA.handle();

  // Clean up WebSocket clients
  ws.cleanupClients();

  // Update LED status
  updateLED();

  // Check if Core 0 requested a broadcast (via queue to avoid cross-core mutex)
  uint8_t trigger;
  if (broadcastQueue != NULL && xQueueReceive(broadcastQueue, &trigger, 0) == pdTRUE)
  {
    performBroadcast(); // Actually send data from Core 1 only
  }

  // Request broadcast from Core 0 (if needed)
  broadcastData();

  // Print status every 10 seconds
  static uint32_t lastStatusPrint = 0;
  if (millis() - lastStatusPrint > 10000)
  {
    lastStatusPrint = millis();

    // Cache voltage values to avoid multiple mutex acquisitions
    float lowestV = getLowestCellVoltage();
    float highestV = getHighestCellVoltage();

    Serial.printf("Status: %s | Mode: %s | Cells: %.3fV-%.3fV (Δ%.1fmV)\n",
                  balancingActive ? "BALANCING" : "GATEWAY",
                  manualMode ? "MANUAL" : "AUTO",
                  lowestV, highestV,
                  (highestV - lowestV) * 1000.0f);
  }

  // Small delay to reduce mutex contention with Core 0
  delay(5); // 5ms delay - balance between responsiveness and CPU usage
}
