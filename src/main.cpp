#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include <Preferences.h>
#include <PubSubClient.h>

#include "driver/twai.h"
#include "can_dual_setup.h" // MCP2515 dual CAN

// WiFi credentials - loaded from credentials.h (not tracked in git)
#include "credentials.h"

// ============================================================================
// BMW i3 Balancing Controller for LilyGO T-2CAN
// ============================================================================
// GATEWAY MODE: Forward messages between BMS and slave modules
// - TWAI (ESP32): Connected to external BMS
// - MCP2515 (SPI): Connected to BMW i3 slave modules
// - Normal: Forward BMS requests (0x080-0x08F) → slave modules, responses (0x100-0x1FF) → BMS
// - Balancing: Intercept BMS requests, modify byte 4 to 0x08, forward responses to BMS
// - Monitors cell voltages and automatically balances when needed
// ============================================================================

// Pin definitions - LilyGO T-2CAN with dual CAN controllers
#ifndef CAN_TX_PIN
#define CAN_TX_PIN GPIO_NUM_7
#endif
#ifndef CAN_RX_PIN
#define CAN_RX_PIN GPIO_NUM_6
#endif
#define CAN_SE_PIN GPIO_NUM_NC // No standby pin on T-2CAN

// Configuration
#define MAX_MODULES 8
#define CELLS_PER_MODULE 12
#define CAN_COMMAND_INTERVAL_MS 20 // Send commands every 20ms (match BMS rate)

// Balancing configuration (runtime adjustable)
float minBalanceVoltage = 3.99f;  // Minimum voltage to start balancing (V)
float balanceThresholdMv = 10.0f; // Start balancing if cells differ by more than this (mV)
float balanceHysteresisMv = 5.0f; // Stop balancing when within this (mV)
String controllerSuffix = "";     // Suffix for controller name (e.g. "1", "2")

// BMW CRC8 finalxor values for COMMAND messages (0x080-0x08F)
// Original values from SimpleBMS - used when modifying BMS commands
const uint8_t finalxor[12] = {0xCF, 0xF5, 0xBB, 0x81, 0x27, 0x1D, 0x53, 0x69, 0x02, 0x38, 0x76, 0x4C};

// BMW CRC8 Lookup table and finalxor values for RESPONSE messages (Derived from log analysis 2026-01-17)
// Rows: Message Type (0-7), Columns: Module ID (0-7)
// 0x10X: Status
// 0x11X: Control/Debug (Unknown, unused)
// 0x12X: Voltage 1
// 0x13X: Voltage 2
// 0x14X: Voltage 3
// 0x15X: Voltage 4
// 0x16X: Balance Status
// 0x17X: Temperatures (Unused/Unverified)
const uint8_t finalxor_table[8][8] = {
    {0xB9, 0x83, 0xCD, 0xF7, 0x51, 0x6B, 0x25, 0x1F}, // Type 0 (0x10X)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // Type 1 (0x11X) - Unknown/Unused
    {0xAA, 0x90, 0xDE, 0xE4, 0x42, 0x78, 0x36, 0x0C}, // Type 2 (0x12X)
    {0x2D, 0x17, 0x59, 0x63, 0xC5, 0xFF, 0xB1, 0x8B}, // Type 3 (0x13X)
    {0x9F, 0xA5, 0xEB, 0xD1, 0x77, 0x4D, 0x03, 0x39}, // Type 4 (0x14X)
    {0x18, 0x22, 0x6C, 0x56, 0xF0, 0xCA, 0x84, 0xBE}, // Type 5 (0x15X)
    {0x8C, 0xB6, 0xF8, 0xC2, 0x64, 0x5E, 0x10, 0x2A}, // Type 6 (0x16X)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}  // Type 7 (0x17X) - Unverified
};

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
bool manualMode = true;              // Start in MANUAL mode - gateway mode, no automatic balancing
bool autoModeAtStartup = false;      // Setting: Start in Auto mode?
bool gatewayMode = true;             // GATEWAY: Forward messages between BMS and slave modules
bool mqttEnabled = false;            // Setting: Enable/Disable MQTT
bool balancingFeedbackLimit = false; // Setting: Stop balancing if no feedback (0x10X)
bool externalMasterDetected = false;
bool canDebugTwaiEnabled = false;    // TWAI (BMS) debug logging - DISABLED by default
bool canDebugMcp2515Enabled = false; // MCP2515 (slave modules) debug logging - DISABLED by default
float targetBalanceVoltage = 4.0f;
float bmsTargetVoltage = 0.0f;    // Target voltage from BMS 0x08X messages
float activeTargetVoltage = 0.0f; // Actual target voltage sent to modules (overridden during balancing)
uint8_t messageCounter = 0;
uint8_t nextMessage = 0;
uint32_t lastCommandTime = 0;
uint32_t lastExternalCommandTime = 0;
uint32_t lastDataUpdate = 0;

// Balancing timeouts
unsigned long lastBalancingFeedbackTime = 0;
unsigned long balancingCooldownStartTime = 0;
const unsigned long BALANCING_TIMEOUT_MS = 15 * 60 * 1000;  // 15 minutes timeout if no feedback
const unsigned long BALANCING_COOLDOWN_MS = 60 * 60 * 1000; // 1 hour cooldown after timeout

// Balancing duty cycle (Run 15m / Pause 5m)
bool balancingPaused = false;
unsigned long balancingCycleTimer = 0;
const unsigned long BALANCE_RUN_TIME_MS = 15 * 60 * 1000;  // 15 minutes continuous
const unsigned long BALANCE_PAUSE_TIME_MS = 5 * 60 * 1000; // 5 minutes pause

// (Unused legacy variables removed)

// Single-core mode - all tasks run on Core 1 (WiFi core)
// No mutex needed - everything runs on same core

// Forward declarations
float getLowestCellVoltage();
float getHighestCellVoltage();

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Preferences preferences;

// MQTT Client
WiFiClient espClient;
PubSubClient mqttClient(espClient);
unsigned long lastMqttPublish = 0;
const unsigned long MQTT_PUBLISH_INTERVAL = 15000;

// Telnet server for remote logging (Port 23)
WiFiServer telnetServer(23);
WiFiClient telnetClient;
bool telnetConnected = false;

// Telnet logging functions
void telnetPrint(const char *str)
{
  Serial.print(str);
  if (telnetConnected && telnetClient && telnetClient.connected())
  {
    size_t len = strlen(str);
    telnetClient.write((const uint8_t *)str, len);
  }
}

void telnetPrintf(const char *format, ...)
{
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  Serial.print(buffer);
  if (telnetConnected && telnetClient && telnetClient.connected())
  {
    size_t len = strlen(buffer);
    telnetClient.write((const uint8_t *)buffer, len);
  }
}

void telnetPrintln(const char *str)
{
  Serial.println(str);
  if (telnetConnected && telnetClient && telnetClient.connected())
  {
    telnetClient.write((const uint8_t *)str, strlen(str));
    telnetClient.write((const uint8_t *)"\r\n", 2);
  }
}

void checkTelnetClient()
{
  // Check for new client connection
  if (telnetServer.hasClient())
  {
    WiFiClient newClient = telnetServer.accept();

    if (newClient)
    {
      // Disconnect old client if exists
      if (telnetClient && telnetClient.connected())
      {
        Serial.println("[Telnet] Disconnecting old client for new connection");
        telnetClient.write((const uint8_t *)"\n[Server] New client connecting - closing this session\n", 55);
        delay(100);
        telnetClient.stop();
      }

      telnetClient = newClient;
      telnetClient.setNoDelay(true); // Disable Nagle algorithm for immediate send
      telnetConnected = true;

      Serial.printf("[Telnet] Client connected from %s\n", telnetClient.remoteIP().toString().c_str());

      // Send welcome message directly
      const char *welcome = "=== BMW i3 Balancing Controller Telnet Log ===\r\n";
      telnetClient.write((const uint8_t *)welcome, strlen(welcome));

      char ipMsg[64];
      snprintf(ipMsg, sizeof(ipMsg), "IP: %s\r\n", WiFi.localIP().toString().c_str());
      telnetClient.write((const uint8_t *)ipMsg, strlen(ipMsg));

      char timeMsg[64];
      snprintf(timeMsg, sizeof(timeMsg), "Time: %lu seconds\r\n", millis() / 1000);
      telnetClient.write((const uint8_t *)timeMsg, strlen(timeMsg));

      const char *sep = "==============================================\r\n\n";
      telnetClient.write((const uint8_t *)sep, strlen(sep));

      Serial.println("[Telnet] Welcome message sent");
    }
  }

  // Check if client disconnected
  if (telnetConnected && telnetClient)
  {
    if (!telnetClient.connected())
    {
      Serial.println("[Telnet] Client disconnected");
      telnetClient.stop();
      telnetConnected = false;
    }
  }
}

// MQTT Helper: Get base topic with suffix
String getBaseTopic()
{
  String topic = "bmw_i3_bms";
  if (controllerSuffix.length() > 0)
  {
    topic += "_" + controllerSuffix;
  }
  return topic;
}

// MQTT Helper: Send Home Assistant Discovery payloads
void sendHADiscovery()
{
  String nodeId = "bmw_i3_bms";
  String deviceName = "BMW i3 BMS";
  if (controllerSuffix.length() > 0)
  {
    nodeId += "_" + controllerSuffix;
    deviceName += " " + controllerSuffix;
  }

  String baseTopic = getBaseTopic();
  String stateTopic = baseTopic + "/metrics";

  // Common device config
  JsonObject deviceParams; // Use object inside the loop directly or helper

  // Define sensors to discover
  struct SensorConfig
  {
    const char *id;
    const char *name;
    const char *unit;
    const char *devClass;
    const char *valueTpl;
  };

  SensorConfig sensors[] = {
      {"volt_min", "Lowest Voltage", "V", "voltage", "{{ value_json.voltage_lowest }}"},
      {"volt_max", "Highest Voltage", "V", "voltage", "{{ value_json.voltage_highest }}"},
      {"volt_diff", "Cell Diff", "mV", "voltage", "{{ value_json.voltage_diff_mv }}"},
      {"status", "Status", "", "", "{{ value_json.status }}"},
      {"mode", "Auto Mode", "", "", "{{ 'Auto' if value_json.auto_mode else 'Manual' }}"},
      {"sys_stat", "System Status", "", "", "{{ value_json.system_status }}"}};

  for (const auto &s : sensors)
  {
    JsonDocument doc;
    doc["name"] = s.name;
    doc["stat_t"] = stateTopic;
    doc["val_tpl"] = s.valueTpl;
    if (strlen(s.unit) > 0)
      doc["unit_of_meas"] = s.unit;
    if (strlen(s.devClass) > 0)
      doc["dev_cla"] = s.devClass;
    doc["uniq_id"] = nodeId + "_" + s.id;

    JsonObject dev = doc["dev"].to<JsonObject>();
    dev["ids"] = nodeId;
    dev["name"] = deviceName;
    dev["mf"] = "BMW/LilyGO";
    dev["mdl"] = "T-2CAN";
    dev["sw"] = "1.2";

    String topic = "homeassistant/sensor/" + nodeId + "/" + s.id + "/config";
    char buffer[1024];
    size_t n = serializeJson(doc, buffer);
    if (!mqttClient.publish(topic.c_str(), buffer, true))
    {
      telnetPrintf("MQTT: Failed to publish discovery for %s (len=%d)\n", s.id, n);
    }
    delay(10); // Slack for network
  }

  telnetPrintln("MQTT: Sent Home Assistant Discovery payloads");
}

// MQTT Functions
void reconnectMQTT()
{
  if (!mqttEnabled)
    return;

  if (!mqttClient.connected())
  {
    bool connected = false;

    // Construct unique Client ID based on suffix
    String clientId = "BMW-i3-BMS";
    if (controllerSuffix.length() > 0)
    {
      clientId += "-" + controllerSuffix;
    }

    // Check if MQTT credentials are provided
    if (String(MQTT_USER).length() > 0)
    {
      connected = mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD);
    }
    else
    {
      connected = mqttClient.connect(clientId.c_str());
    }

    if (connected)
    {
      telnetPrintln("MQTT Connected");
      String statusTopic = getBaseTopic() + "/status";
      mqttClient.publish(statusTopic.c_str(), "online");
      sendHADiscovery();
    }
  }
}

void processMQTT()
{
  if (!mqttEnabled)
    return;

  if (WiFi.status() != WL_CONNECTED)
    return;

  if (!mqttClient.connected())
  {
    static unsigned long lastReconnectAttempt = 0;
    if (millis() - lastReconnectAttempt > 5000)
    {
      lastReconnectAttempt = millis();
      reconnectMQTT();
    }
  }
  else
  {
    mqttClient.loop();

    if (millis() - lastMqttPublish > MQTT_PUBLISH_INTERVAL)
    {
      lastMqttPublish = millis();

      // Publish Status
      JsonDocument doc;

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
      if (millis() < 5000)
        hasError = false;

      // Basic Status
      doc["status"] = balancingActive ? (balancingPaused ? "BALANCING_PAUSED" : "BALANCING") : "IDLE";
      if (balancingCooldownStartTime > 0)
        doc["status"] = "COOLDOWN";

      doc["system_status"] = hasError ? "Error" : "System OK";

      doc["auto_mode"] = !manualMode;

      float lowest = getLowestCellVoltage();
      float highest = getHighestCellVoltage();

      doc["voltage_lowest"] = serialized(String(lowest, 3));
      doc["voltage_highest"] = serialized(String(highest, 3));
      doc["voltage_diff_mv"] = serialized(String((highest - lowest) * 1000.0f, 0));

      char buffer[512];
      serializeJson(doc, buffer);

      String topic = getBaseTopic() + "/metrics";
      mqttClient.publish(topic.c_str(), buffer);
    }
  }
}

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

// Calculate checksum for CAN message (Response messages 0x100-0x1FF)
// Uses finalxor_table for response messages
uint8_t calculateChecksum(const twai_message_t &msg)
{
  uint8_t canmes[11];
  int meslen = msg.data_length_code + 1;

  canmes[0] = msg.identifier >> 8;
  canmes[1] = msg.identifier & 0xFF;

  for (int i = 0; i < (msg.data_length_code - 1); i++)
  {
    canmes[i + 2] = msg.data[i];
  }

  uint8_t msgType = (msg.identifier >> 4) & 0x07; // Extract Type (0-7)
  uint8_t moduleId = msg.identifier & 0x07;       // Extract Module (0-7)

  return crc8.get_crc8(canmes, meslen, finalxor_table[msgType][moduleId]);
}

// Calculate checksum for COMMAND messages (0x080-0x08F)
// Uses the original finalxor array from SimpleBMS
uint8_t calculateChecksum_Command(const twai_message_t &msg, uint8_t msgId)
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
  if (!canDebugTwaiEnabled)
    return;

  char direction[4];
  strcpy(direction, isTX ? "TX" : "RX");

  Serial.printf("[TWAI %s] 0x%03X [%d] ", direction, msg.identifier, msg.data_length_code);

  for (int i = 0; i < msg.data_length_code; i++)
  {
    Serial.printf("%02X ", msg.data[i]);
  }
  Serial.println("");
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
  // 0x16X = balance status (type 6)
  // 0x17X = temperatures (type 7)
  uint8_t typeId = messageType;

  if (typeId > 7)
  {
    return;
  }

  // Parse based on message type
  switch (typeId)
  {
  case 0:
  {
    // Error and balance status (0x10X)
    module.errorCode = msg.data[0] | (msg.data[1] << 8) | (msg.data[2] << 16) | (msg.data[3] << 24);

    uint16_t oldBalanceStatus = module.balanceStatus;
    module.balanceStatus = ((msg.data[5] & 0x0F) << 8) | msg.data[4];

    // Update global balancing state from 0x10X status
    // Any non-zero balance status implies balancing is active
    module.balancing = (module.balanceStatus != 0);

    // Update feedback timer if any module reports balancing
    if (module.balancing)
    {
      lastBalancingFeedbackTime = millis();
    }

    break;
  }

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
  {
    // Module Total Voltage (0x16X)
    // Bytes 0-1: Total Voltage in mV (Little Endian)
    uint16_t voltageRaw = msg.data[0] | (msg.data[1] << 8);
    module.moduleVoltage = voltageRaw / 1000.0f;

    // Balancing status in bytes 2-5 (optional check)
    // PREVIOUSLY: used this for balancing detection.
    // NOW: Moved to 0x10X based on user request.
    break;
  }

  case 7:
    // Temperatures (0x17X)
    // BMW i3 uses offset of -40°C (common automotive standard)
    // 4 temperature sensors per module
    module.temperatures[0] = (float)msg.data[0] - 40.0f;
    module.temperatures[1] = (float)msg.data[1] - 40.0f;
    module.temperatures[2] = (float)msg.data[2] - 40.0f;
    module.temperatures[3] = (float)msg.data[3] - 40.0f;
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
  // ========== PRIORITY: Read from MCP2515 (Slave Module Responses) FIRST ==========
  // Read responses from slave modules to prevent TX buffer blocking
  uint32_t mcp_id;
  uint8_t mcp_len;
  uint8_t mcp_data[8];

  int mcp_rx_count = 0;
  while (readCAN2(mcp_id, mcp_len, mcp_data) && mcp_rx_count++ < 30)
  {
    // Parse module data for monitoring (0x100-0x1FF)
    if ((mcp_id & 0xF00) == 0x100)
    {
      twai_message_t temp_msg;
      temp_msg.identifier = mcp_id;
      temp_msg.data_length_code = mcp_len;
      memcpy(temp_msg.data, mcp_data, mcp_len);
      parseModuleMessage(temp_msg);

      // Forward ALL module responses to BMS (0x100-0x1FF)
      // Includes status, cell data, temperatures, and diagnostics
      bool shouldForward = (mcp_id >= 0x100 && mcp_id <= 0x1FF);

      if (shouldForward)
      {
        // Forward slave module response back to BMS via TWAI
        twai_message_t forward_msg;
        forward_msg.identifier = mcp_id;
        forward_msg.data_length_code = mcp_len;
        forward_msg.flags = TWAI_MSG_FLAG_NONE;
        forward_msg.extd = 0;
        forward_msg.rtr = 0;
        memcpy(forward_msg.data, mcp_data, mcp_len);

        // BMW slave modules set bit 7 in voltage bytes when balancing is active
        // BMS expects this bit clear, so mask it out for cell voltage messages (0x120-0x157)
        // NOTE: Byte 6 is a counter and must NOT be modified
        // NOTE: Do NOT mask 0x160-0x177 (balance status, temperatures, diagnostics)
        if (mcp_id >= 0x120 && mcp_id <= 0x157 && mcp_len >= 6)
        {
          int moduleIdx = mcp_id & 0x0F;
          uint8_t cellBase = 0;
          if (mcp_id >= 0x130 && mcp_id <= 0x13F)
            cellBase = 3;
          else if (mcp_id >= 0x140 && mcp_id <= 0x14F)
            cellBase = 6;
          else if (mcp_id >= 0x150 && mcp_id <= 0x15F)
            cellBase = 9;

          // Hvis vi balancerer, så ignorer de rå mcp_data og byg pakken fra
          // de stabile værdier vi har i 'modules' (dem som WebUI også bruger)
          if (balancingActive && !balancingPaused)
          {
            for (int i = 0; i < 3; i++)
            {
              // Konverter float spænding tilbage til det format BMS forventer (14-bit millivolt)
              uint16_t v = (uint16_t)(modules[moduleIdx].cellVoltages[cellBase + i] * 1000.0f);
              forward_msg.data[i * 2] = v & 0xFF;
              forward_msg.data[i * 2 + 1] = (v >> 8) & 0x3F; // Maskerer bit 6 og 7 (fjerner balance-flag)
            }
          }
          else
          {
            // Normal drift: Bare maskér bit 7 ud af de rå data
            for (int i = 0; i < mcp_len; i++)
            {
              forward_msg.data[i] = mcp_data[i];
            }
            forward_msg.data[1] &= 0x7F;
            forward_msg.data[3] &= 0x7F;
            forward_msg.data[5] &= 0x7F;
          }

          // Beregn ny checksum da data er ændret
          forward_msg.data[mcp_len - 1] = calculateChecksum(forward_msg);
          twai_transmit(&forward_msg, pdMS_TO_TICKS(10));
          continue; // Vigtigt: Hop videre så linje 715 ikke sender pakken igen
        }

        // Mask balance status in 0x10X messages (byte 3, 4 and 5 contain balance status)
        // Clear ALL THREE bytes to hide balancing from BMS
        if (mcp_id >= 0x100 && mcp_id <= 0x10F && mcp_len >= 6)
        {
          // Check if any balance status is present (byte 3, 4 or 5 non-zero)
          if (forward_msg.data[3] != 0 || forward_msg.data[4] != 0 || forward_msg.data[5] != 0)
          {
            uint8_t originalByte3 = forward_msg.data[3];
            uint8_t originalByte4 = forward_msg.data[4];
            uint8_t originalByte5 = forward_msg.data[5];

            // Clear ALL THREE bytes that contain balance status
            forward_msg.data[3] = 0x00;
            forward_msg.data[4] = 0x00;
            forward_msg.data[5] = 0x00;

            // Recalculate CRC after masking
            forward_msg.data[forward_msg.data_length_code - 1] = calculateChecksum(forward_msg);
          }
        }

        // OVERRIDE Module Voltage in 0x16X messages during balancing
        // When balancing, the measured module voltage (0x16X bytes 0-1) sags.
        // We replace it with the sum of the 12 individual cell voltages to report a stable voltage.
        if (mcp_id >= 0x160 && mcp_id <= 0x16F && mcp_len >= 8)
        {
          int moduleIndex = mcp_id & 0x0F;
          if (moduleIndex >= 0 && moduleIndex < MAX_MODULES)
          {
            // Check if balancing is active (based on 0x10X status or our command)
            // We use balanceStatus from the most recent 0x10X message
            if (modules[moduleIndex].balanceStatus != 0)
            {
              float sumVoltage = 0.0f;
              // Sum up all 12 cell voltages
              for (int c = 0; c < 12; c++)
              {
                sumVoltage += modules[moduleIndex].cellVoltages[c];
              }

              // 0x16X sends voltage in mV (approx 48000 mV for 48V)
              // Data is Little Endian (Byte 0 = LSB, Byte 1 = MSB)
              uint16_t voltageVal = (uint16_t)(sumVoltage * 1000.0f);

              // Update the message
              forward_msg.data[0] = voltageVal & 0xFF;
              forward_msg.data[1] = (voltageVal >> 8) & 0xFF;

              // Recalculate CRC
              forward_msg.data[forward_msg.data_length_code - 1] = calculateChecksum(forward_msg);
            }
          }
        }

        esp_err_t result = twai_transmit(&forward_msg, pdMS_TO_TICKS(10));

        // Debug output for forwarding
        if (canDebugTwaiEnabled)
        {
          char debugBuf[128];
          int offset = snprintf(debugBuf, sizeof(debugBuf), "[TWAI    TX] 0x%03X [%d] ", forward_msg.identifier, forward_msg.data_length_code);
          for (int i = 0; i < forward_msg.data_length_code; i++)
          {
            offset += snprintf(debugBuf + offset, sizeof(debugBuf) - offset, "%02X ", forward_msg.data[i]);
          }
          snprintf(debugBuf + offset, sizeof(debugBuf) - offset, "%s\n", result == ESP_OK ? "OK" : "FAIL");
          telnetPrintf("%s", debugBuf);
          Serial.print(debugBuf);
        }
      }
    }
  }

  // ========== Read from TWAI (BMS Requests) and forward to MCP2515 ==========
  // BMS sends requests (0x080-0x08F) to slave modules
  twai_message_t twai_msg;
  int twai_msg_count = 0;

  while (twai_receive(&twai_msg, 0) == ESP_OK && twai_msg_count++ < 10)
  {
    uint32_t id = twai_msg.identifier;

    // Debug output throttled to reduce Serial overhead
    if (canDebugTwaiEnabled)
    {
      static uint32_t lastTwaiDebug = 0;
      if (millis() - lastTwaiDebug > 200) // Only every 200ms
      {
        lastTwaiDebug = millis();
        char debugBuf[128];
        int offset = snprintf(debugBuf, sizeof(debugBuf), "[TWAI    RX] 0x%03X [%d] ", id, twai_msg.data_length_code);
        for (int i = 0; i < twai_msg.data_length_code; i++)
        {
          offset += snprintf(debugBuf + offset, sizeof(debugBuf) - offset, "%02X ", twai_msg.data[i]);
        }
        snprintf(debugBuf + offset, sizeof(debugBuf) - offset, "\n");
        telnetPrintf("%s", debugBuf);
        Serial.print(debugBuf);
      }
    }

    // BMS request to slave modules (0x080-0x08F)
    if ((id & 0xFF0) == 0x080)
    {
      // Extract target voltage from BMS command (byte 0-1)
      if (twai_msg.data_length_code >= 2)
      {
        uint16_t rawVoltage = twai_msg.data[0] | (twai_msg.data[1] << 8);
        bmsTargetVoltage = rawVoltage / 1000.0f; // Convert to volts
        // activeTargetVoltage = bmsTargetVoltage;  // Default removed by user request
      }

      // Forward BMS requests to slave modules via MCP2515
      // Modify byte 4 if balancing is active
      uint8_t send_data[8];
      memcpy(send_data, twai_msg.data, twai_msg.data_length_code);

      // Only enable balancing if active AND not in cooling pause
      if (balancingActive && !balancingPaused && twai_msg.data_length_code >= 8)
      {
        send_data[2] = 0x00;
        send_data[3] = 0x50;
        send_data[4] = 0x08; // Enable balancing

        // Set target voltage to lowest cell + 2mV
        // BMS gets wrong cell values during balancing, so we override the target
        float lowestCell = getLowestCellVoltage();
        if (lowestCell > 0.5f) // Valid cell voltage
        {
          float targetV = lowestCell + 0.002f; // Add 2mV
          uint16_t targetMV = (uint16_t)(targetV * 1000.0f);
          uint16_t originalTargetMV = twai_msg.data[0] | (twai_msg.data[1] << 8);
          send_data[0] = targetMV & 0xFF;        // Low byte
          send_data[1] = (targetMV >> 8) & 0xFF; // High byte
          activeTargetVoltage = targetV;         // Update active target for web UI
        }

        // Recalculate CRC using the ORIGINAL method for command messages (0x080-0x08F)
        // Uses finalxor[] array (not finalxor_table)
        twai_message_t temp_msg;
        temp_msg.identifier = id;
        temp_msg.data_length_code = twai_msg.data_length_code;
        memcpy(temp_msg.data, send_data, 8);
        uint8_t msgId = id & 0x0F;
        send_data[7] = calculateChecksum_Command(temp_msg, msgId);
      }

      // Try to send, but don't block if buffer is full
      bool sent = sendCAN2(id, twai_msg.data_length_code, send_data);

      // If send failed, read any pending RX to clear space
      if (!sent)
      {
        static uint32_t lastDropWarning = 0;
        static uint32_t failCount = 0;

        // Try to read pending RX messages to free TX buffer
        if (canReadCAN2())
        {
          uint32_t mcp_id;
          uint8_t mcp_len;
          uint8_t mcp_data[8];
          while (readCAN2(mcp_id, mcp_len, mcp_data))
          {
            // Forward ALL module responses to BMS (0x100-0x1FF)
            if (mcp_id >= 0x100 && mcp_id <= 0x1FF)
            {
              twai_message_t forward_msg;
              forward_msg.identifier = mcp_id;
              forward_msg.data_length_code = mcp_len;
              forward_msg.flags = TWAI_MSG_FLAG_NONE;
              forward_msg.extd = 0;
              forward_msg.rtr = 0;
              memcpy(forward_msg.data, mcp_data, mcp_len);
              twai_transmit(&forward_msg, pdMS_TO_TICKS(10));
            }
          }
        }

        failCount++;
        if (millis() - lastDropWarning > 1000)
        {
          Serial.printf("⚠ MCP2515 TX full - dropped %lu BMS requests (cleared RX buffer)\n", failCount);
          failCount = 0;
          lastDropWarning = millis();
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
      // Ignore Cell 96 (Module 8, Cell 12 in 1-based / m=7, c=11 in 0-based)
      if (m == 7 && c == 11)
        continue;

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
      // Ignore Cell 96 (Module 8, Cell 12 in 1-based / m=7, c=11 in 0-based)
      if (m == 7 && c == 11)
        continue;

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
// Balancing is now handled by intercepting BMS requests in readCANMessages()
// When balancingActive=true, we modify byte 4 to 0x08 in BMS requests

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

  // Duty Cycle Logic (15m Run / 5m Pause)
  if (balancingActive)
  {
    if (!balancingPaused)
    {
      if (millis() - balancingCycleTimer > BALANCE_RUN_TIME_MS)
      {
        balancingPaused = true;
        balancingCycleTimer = millis();
        // telnetPrintln("Balancing: Pausing (10m cool-down)");
      }
    }
    else
    {
      if (millis() - balancingCycleTimer > BALANCE_PAUSE_TIME_MS)
      {
        balancingPaused = false;
        balancingCycleTimer = millis();
        // telnetPrintln("Balancing: Resuming (30m run)");
      }
    }
  }
  else
  {
    balancingPaused = false;
    balancingCycleTimer = millis();
  }

  // Check timeout while balancing active
  if (balancingActive && balancingFeedbackLimit)
  {
    if (millis() - lastBalancingFeedbackTime > BALANCING_TIMEOUT_MS)
    {
      balancingActive = false;
      balancingCooldownStartTime = millis();
      telnetPrintf("Stopping balancing: Timeout (no feedback for 15m). Waiting 1h.\n");
      return;
    }
  }

  // Check cooldown if not active (prevent restart)
  if (!balancingActive && balancingCooldownStartTime > 0)
  {
    if (millis() - balancingCooldownStartTime < BALANCING_COOLDOWN_MS)
    {
      // In cooldown
      return;
    }
    else
    {
      // Cooldown expired
      balancingCooldownStartTime = 0;
      telnetPrintln("Balancing cooldown finished, ready to balance again.");
    }
  }

  float lowestVoltage = getLowestCellVoltage();

  // Always update active target to lowest + 2mV (User Request)
  if (lowestVoltage > 0.5f)
  {
    activeTargetVoltage = lowestVoltage + 0.002f;
  }

  float highestVoltage = getHighestCellVoltage();
  float difference_mV = (highestVoltage - lowestVoltage) * 1000.0f;

  // Only start balancing if highest cell is above minimum voltage
  if (!balancingActive && difference_mV > balanceThresholdMv && highestVoltage > minBalanceVoltage)
  {
    // Start balancing
    balancingActive = true;
    balancingCycleTimer = millis();       // Start duty cycle timer
    lastBalancingFeedbackTime = millis(); // Reset timeout timer
    telnetPrintf("Starting balancing: Lowest=%.3fV, Highest=%.3fV, Diff=%.1fmV\n",
                 lowestVoltage, highestVoltage, difference_mV);
  }
  else if (balancingActive && difference_mV < balanceHysteresisMv)
  {
    // Stop balancing
    balancingActive = false;
    telnetPrintf("Stopping balancing: Cells balanced within %.1fmV\n", difference_mV);
  }
  else if (balancingActive && highestVoltage < minBalanceVoltage)
  {
    // Stop balancing if voltage drops too low
    balancingActive = false;
    telnetPrintf("Stopping balancing: Cell voltage too low (%.3fV)\n", highestVoltage);
  }
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
          telnetPrintln("Manual start - balancing active (intercepts BMS requests)");
        }
        else if (strcmp(command, "stop") == 0)
        {
          balancingActive = false;
          manualMode = true;
          telnetPrintln("Manual stop - gateway mode (forwards BMS requests)");
        }
        else if (strcmp(command, "auto") == 0)
        {
          manualMode = false;
          telnetPrintln("Auto mode - smart balancing enabled");
        }
        else if (strcmp(command, "restart") == 0)
        {
          telnetPrintln("Restart command received - restarting ESP32...");
          delay(500);
          ESP.restart();
        }
        else if (strcmp(command, "toggleDebugTwai") == 0)
        {
          canDebugTwaiEnabled = !canDebugTwaiEnabled;
          telnetPrintf("TWAI debug logging %s\n", canDebugTwaiEnabled ? "enabled" : "disabled");
        }
        else if (strcmp(command, "toggleDebugMcp2515") == 0)
        {
          canDebugMcp2515Enabled = !canDebugMcp2515Enabled;
          telnetPrintf("MCP2515 debug logging %s\n", canDebugMcp2515Enabled ? "enabled" : "disabled");
        }
        else if (strcmp(command, "updateSettings") == 0)
        {
          bool changed = false;
          if (doc["minBalanceVoltage"].is<float>())
          {
            minBalanceVoltage = doc["minBalanceVoltage"];
            preferences.putFloat("minVolts", minBalanceVoltage);
            Serial.printf("Min balance voltage set to %.2fV\n", minBalanceVoltage);
            changed = true;
          }
          if (doc["balanceThreshold"].is<float>())
          {
            balanceThresholdMv = doc["balanceThreshold"];
            preferences.putFloat("threshold", balanceThresholdMv);
            Serial.printf("Balance threshold set to %.0fmV\n", balanceThresholdMv);
            changed = true;
          }
          if (doc["balanceHysteresis"].is<float>())
          {
            balanceHysteresisMv = doc["balanceHysteresis"];
            preferences.putFloat("hysteresis", balanceHysteresisMv);
            Serial.printf("Balance hysteresis set to %.0fmV\n", balanceHysteresisMv);
            changed = true;
          }
          if (doc["controllerSuffix"].is<const char *>())
          {
            controllerSuffix = doc["controllerSuffix"].as<String>();
            preferences.putString("suffix", controllerSuffix);
            Serial.printf("Controller suffix set to %s\n", controllerSuffix.c_str());
            changed = true;
          }
          if (doc["autoModeAtStartup"].is<bool>())
          {
            autoModeAtStartup = doc["autoModeAtStartup"];
            preferences.putBool("autoStart", autoModeAtStartup);
            Serial.printf("Auto Start set to %s\n", autoModeAtStartup ? "YES" : "NO");
            changed = true;
          }
          if (doc["balancingFeedbackLimit"].is<bool>())
          {
            balancingFeedbackLimit = doc["balancingFeedbackLimit"];
            preferences.putBool("feedbackLimit", balancingFeedbackLimit);
            Serial.printf("Feedback Limit set to %s\n", balancingFeedbackLimit ? "YES" : "NO");
            changed = true;
          }
          if (doc["mqttEnabled"].is<bool>())
          {
            mqttEnabled = doc["mqttEnabled"];
            preferences.putBool("mqttEnabled", mqttEnabled);
            Serial.printf("MQTT Enabled set to %s\n", mqttEnabled ? "YES" : "NO");
            changed = true;
          }

          if (changed)
          {
            Serial.println("Settings saved to flash memory");
          }
        }
        else if (strcmp(command, "setThreshold") == 0)
        {
          // Future: set threshold
        }
      }
    }
  }
}

// Send data to WebSocket clients
void performBroadcast()
{
  JsonDocument doc;
  String jsonString;

  // Direct access - float reads are atomic enough on ESP32
  float lowestVoltage = 5.0f;
  float highestVoltage = 0.0f;
  float totalBatteryVoltage = 0.0f;

  for (int m = 0; m < MAX_MODULES; m++)
  {
    if (!modules[m].exists)
      continue;

    totalBatteryVoltage += modules[m].moduleVoltage;

    for (int c = 0; c < CELLS_PER_MODULE; c++)
    {
      // Ignore Cell 96 (Module 8, Cell 12 in 1-based / m=7, c=11 in 0-based)
      if (m == 7 && c == 11)
        continue;

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
    if (balancingPaused)
    {
      status = "BALANCING (PAUSED)";
    }
    else
    {
      status = "BALANCING";
    }
  }
  else if (balancingCooldownStartTime > 0)
  {
    unsigned long remainingCooldown = (BALANCING_COOLDOWN_MS - (millis() - balancingCooldownStartTime)) / 1000 / 60;
    status = "COOLDOWN (" + String(remainingCooldown) + "m)";
  }
  else
  {
    if (manualMode)
    {
      status = "GATEWAY";
    }
    else
    {
      status = "IDLE";
    }
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
  doc["lowestVoltage"] = lowestVoltage;
  doc["highestVoltage"] = highestVoltage;
  doc["difference"] = (highestVoltage - lowestVoltage) * 1000.0f;
  doc["totalVoltage"] = totalBatteryVoltage;
  doc["bmsTargetVoltage"] = bmsTargetVoltage;
  doc["activeTargetVoltage"] = activeTargetVoltage;
  doc["gatewayMode"] = gatewayMode;
  doc["externalMaster"] = externalMasterDetected && (millis() - lastExternalCommandTime < 60000);
  doc["uptime"] = millis() / 1000;

  // Send current settings
  doc["minBalanceVoltage"] = minBalanceVoltage;
  doc["balanceThresholdMv"] = balanceThresholdMv;
  doc["balanceHysteresisMv"] = balanceHysteresisMv;
  doc["controllerSuffix"] = controllerSuffix;
  doc["autoModeAtStartup"] = autoModeAtStartup;
  doc["mqttEnabled"] = mqttEnabled;
  doc["balancingFeedbackLimit"] = balancingFeedbackLimit;

  JsonArray modulesArray = doc["modules"].to<JsonArray>();

  for (int m = 0; m < MAX_MODULES; m++)
  {
    if (!modules[m].exists)
      continue;

    JsonObject moduleObj = modulesArray.add<JsonObject>();
    moduleObj["id"] = m + 1;
    moduleObj["voltage"] = modules[m].moduleVoltage;
    moduleObj["balancing"] = (modules[m].balanceStatus != 0);
    moduleObj["balanceStatus"] = modules[m].balanceStatus; // Send the bitmask
    moduleObj["error"] = modules[m].errorCode;

    JsonArray cellsArray = moduleObj["cells"].to<JsonArray>();
    for (int c = 0; c < CELLS_PER_MODULE; c++)
    {
      if (modules[m].cellVoltages[c] > 0.5f)
      {
        cellsArray.add(modules[m].cellVoltages[c]);
      }
    }

    JsonArray tempsArray = moduleObj["temps"].to<JsonArray>();
    for (int t = 0; t < 4; t++)
    {
      tempsArray.add(modules[m].temperatures[t]);
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
        .balancing-cell-indicator {
            display: inline-block;
            margin-left: 5px;
            color: #4ade80;
            font-weight: bold;
            animation: blink 1s infinite;
        }
        @keyframes blink {
            0% { opacity: 1; }
            50% { opacity: 0; }
            100% { opacity: 1; }
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
        .module-temps {
            margin-top: 10px;
            padding-top: 10px;
            border-top: 1px solid rgba(255,255,255,0.1);
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 5px;
            font-size: 0.85em;
        }
        .temp-item {
            display: flex;
            justify-content: space-between;
            padding: 4px 8px;
            background: rgba(0,0,0,0.2);
            border-radius: 5px;
        }
        .temp-label {
            opacity: 0.7;
        }
        .temp-value {
            font-weight: bold;
        }
        .temp-value.cold {
            color: #60a5fa;
        }
        .temp-value.normal {
            color: #4ade80;
        }
        .temp-value.warm {
            color: #fbbf24;
        }
        .temp-value.hot {
            color: #ef4444;
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
        .settings-section {
            background: rgba(255,255,255,0.1);
            backdrop-filter: blur(10px);
            border-radius: 15px;
            padding: 20px;
            margin-top: 20px;
        }
        .settings-toggle-btn {
            background: linear-gradient(135deg, #8b5cf6 0%, #7c3aed 100%);
            border: none;
            padding: 12px 24px;
            border-radius: 10px;
            color: white;
            font-size: 1em;
            font-weight: bold;
            cursor: pointer;
            display: inline-flex;
            align-items: center;
            justify-content: center;
            gap: 8px;
            transition: all 0.3s ease;
        }
        .settings-toggle-btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(139, 92, 246, 0.4);
        }
        .settings-toggle-btn .arrow {
            transition: transform 0.3s ease;
            font-size: 0.8em;
        }
        .settings-toggle-btn.open .arrow {
            transform: rotate(180deg);
        }
        .settings-content {
            max-height: 0;
            overflow: hidden;
            transition: max-height 0.3s ease;
        }
        .settings-content.open {
            max-height: 800px;
            margin-top: 20px;
        }
        .settings-title {
            text-align: center;
            font-size: 1.3em;
            font-weight: bold;
            margin-bottom: 20px;
            color: #60a5fa;
        }
        .settings-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
            gap: 15px;
            margin-bottom: 20px;
        }
        .setting-item {
            background: rgba(0,0,0,0.3);
            padding: 15px;
            border-radius: 10px;
        }
        .setting-label {
            font-size: 0.9em;
            opacity: 0.8;
            margin-bottom: 8px;
        }
        .setting-input-group {
            display: flex;
            align-items: center;
            gap: 10px;
        }
        .setting-input-group input {
            flex: 1;
            background: rgba(255,255,255,0.1);
            border: 1px solid rgba(255,255,255,0.2);
            border-radius: 8px;
            padding: 10px;
            color: white;
            font-size: 1em;
        }
        .setting-input-group span {
            font-size: 0.9em;
            opacity: 0.7;
        }
        .settings-buttons {
            display: flex;
            justify-content: center;
            gap: 10px;
            flex-wrap: wrap;
        }
        .debug-button {
            position: fixed;
            bottom: 20px;
            right: 20px;
            z-index: 1000;
            display: none; /* Hidden - moved to settings section */
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
        <div class="connection-status" id="connectionStatus">Connecting...</div>
    </div>
    
    <div class="debug-button">
        <!-- Moved to settings section -->
    </div>
    
    <div class="container">
        <h1>⚡ BMW i3 Balancing Controller <span id="titleSuffix"></span></h1>
        
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
            <div style="display: flex; align-items: center; justify-content: center; gap: 20px; flex-wrap: wrap;">
                <div style="text-align: center;">
                    <div style="font-size: 0.9em; opacity: 0.8; margin-bottom: 5px;">BMS Target</div>
                    <div id="bmsTarget" style="font-size: 1.5em; font-weight: bold; color: #60a5fa;">-.-V</div>
                </div>
                <div style="text-align: center;">
                    <div style="font-size: 0.9em; opacity: 0.8; margin-bottom: 5px;">Active Target</div>
                    <div id="activeTarget" style="font-size: 1.5em; font-weight: bold; color: #34d399;">-.-V</div>
                </div>
                <div style="display: flex; gap: 10px; flex-wrap: wrap; justify-content: center;">
                    <button onclick="sendCommand('start')">▶ Start Balancing</button>
                    <button class="stop" onclick="sendCommand('stop')">⏹ Stop / Gateway</button>
                    <button class="auto" onclick="sendCommand('auto')">🔄 Auto Mode</button>
                </div>
            </div>
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
        
        <div class="settings-section">
            <div class="settings-buttons">
                <button id="debugBtnTwai" onclick="toggleDebugTwai()">🐛 TWAI Debug: OFF</button>
                <button id="debugBtnMcp" onclick="toggleDebugMcp2515()">🐛 MCP2515 Debug: OFF</button>
                <input type="file" id="otaFile" accept=".bin" onchange="uploadOTA()" style="display:none;">
                <button class="ota" onclick="document.getElementById('otaFile').click()">⬆️ OTA Update</button>
                <button class="restart" onclick="restartDevice()">🔄 Restart</button>
                <button class="settings-toggle-btn" onclick="toggleSettings()">
                    <span>⚙️ Settings</span>
                    <span class="arrow">▼</span>
                </button>
            </div>
            
            <div class="settings-content" id="settingsContent">
                <div class="settings-grid">
                    <div class="setting-item">
                        <div class="setting-label">Min Balance Voltage</div>
                        <div class="setting-input-group">
                            <input type="number" id="minBalanceVoltage" step="0.01" min="3.5" max="4.3" value="3.99">
                            <span>V</span>
                        </div>
                    </div>
                    <div class="setting-item">
                        <div class="setting-label">Balance Threshold</div>
                        <div class="setting-input-group">
                            <input type="number" id="balanceThreshold" step="1" min="1" max="50" value="10">
                            <span>mV</span>
                        </div>
                    </div>
                    <div class="setting-item">
                        <div class="setting-label">Balance Hysteresis</div>
                        <div class="setting-input-group">
                            <input type="number" id="balanceHysteresis" step="1" min="1" max="20" value="5">
                            <span>mV</span>
                        </div>
                    </div>
                    <div class="setting-item">
                        <div class="setting-label">Controller Name Suffix</div>
                        <div class="setting-input-group">
                            <input type="text" id="controllerSuffix" placeholder="e.g. 1" maxlength="5">
                        </div>
                    </div>
                    <div class="setting-item">
                        <div class="setting-label">Auto Mode at Startup</div>
                        <div class="setting-input-group">
                            <label class="switch" style="display: flex; align-items: center; gap: 10px; cursor: pointer;">
                                <input type="checkbox" id="autoModeAtStartup" style="width: 20px; height: 20px;">
                                <span style="font-size: 0.9em; opacity: 0.8;">Enable Auto Mode on boot</span>
                            </label>
                        </div>
                    </div>
                    <div class="setting-item">
                        <div class="setting-label">Feedback Timeout</div>
                        <div class="setting-input-group">
                            <label class="switch" style="display: flex; align-items: center; gap: 10px; cursor: pointer;">
                                <input type="checkbox" id="balancingFeedbackLimit" style="width: 20px; height: 20px;">
                                <span style="font-size: 0.9em; opacity: 0.8;">Stop if 15m without feedback</span>
                            </label>
                        </div>
                    </div>
                    <div class="setting-item">
                        <div class="setting-label">MQTT Integration</div>
                        <div class="setting-input-group">
                            <label class="switch" style="display: flex; align-items: center; gap: 10px; cursor: pointer;">
                                <input type="checkbox" id="mqttEnabled" style="width: 20px; height: 20px;">
                                <span style="font-size: 0.9em; opacity: 0.8;">Enable MQTT Reporting</span>
                            </label>
                        </div>
                    </div>
                </div>

                <div style="display: flex; justify-content: center; margin-top: 15px;">
                    <button onclick="updateSettings()">💾 Save Settings</button>
                </div>
            </div>
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
            
            // Update settings inputs (only if not currently focused)
            if (document.activeElement.id !== 'minBalanceVoltage' && data.minBalanceVoltage) {
                document.getElementById('minBalanceVoltage').value = data.minBalanceVoltage.toFixed(2);
            }
            if (document.activeElement.id !== 'balanceThreshold' && data.balanceThresholdMv) {
                document.getElementById('balanceThreshold').value = data.balanceThresholdMv.toFixed(0);
            }
            if (document.activeElement.id !== 'balanceHysteresis' && data.balanceHysteresisMv) {
                document.getElementById('balanceHysteresis').value = data.balanceHysteresisMv.toFixed(0);
            }
            if (data.controllerSuffix !== undefined) {
                if (document.activeElement.id !== 'controllerSuffix') {
                    document.getElementById('controllerSuffix').value = data.controllerSuffix;
                }
                const suffix = data.controllerSuffix ? ' ' + data.controllerSuffix : '';
                document.title = 'BMW i3 Balancing Controller' + suffix;
                document.getElementById('titleSuffix').textContent = data.controllerSuffix;
            }
            if (data.autoModeAtStartup !== undefined) {
                 document.getElementById('autoModeAtStartup').checked = data.autoModeAtStartup;
            }
            if (data.balancingFeedbackLimit !== undefined) {
                 document.getElementById('balancingFeedbackLimit').checked = data.balancingFeedbackLimit;
            }
            if (data.mqttEnabled !== undefined) {
                 document.getElementById('mqttEnabled').checked = data.mqttEnabled;
            }
            
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
            
            // Update BMS target voltage
            const bmsTargetElement = document.getElementById('bmsTarget');
            if (data.bmsTargetVoltage > 0) {
                bmsTargetElement.textContent = data.bmsTargetVoltage.toFixed(3) + 'V';
            } else {
                bmsTargetElement.textContent = '-.-V';
            }
            
            // Update active target voltage (what we actually send to modules)
            const activeTargetElement = document.getElementById('activeTarget');
            if (data.activeTargetVoltage > 0) {
                activeTargetElement.textContent = data.activeTargetVoltage.toFixed(3) + 'V';
                // Highlight if different from BMS target
                if (Math.abs(data.activeTargetVoltage - data.bmsTargetVoltage) > 0.001) {
                    activeTargetElement.style.color = '#fbbf24'; // Orange when overridden
                } else {
                    activeTargetElement.style.color = '#34d399'; // Green when same
                }
            } else {
                activeTargetElement.textContent = '-.-V';
            }
            
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
                // Display Module Voltage
                let voltageText = '';
                if (module.voltage > 0) {
                    voltageText = '<span style="font-size:0.8em; margin-left:10px; opacity:0.8; font-weight:normal;">' + module.voltage.toFixed(2) + 'V</span>';
                }
                titleDiv.innerHTML = 'Module ' + module.id + voltageText;
                
                headerDiv.appendChild(titleDiv);

                if (module.balancing) {
                    const indicatorDiv = document.createElement('div');
                    indicatorDiv.className = 'balance-indicator active';
                    indicatorDiv.textContent = 'BALANCING';
                    headerDiv.appendChild(indicatorDiv);
                }
                
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
                    labelDiv.innerHTML = 'Cell ' + (index + 1);

                    if (module.balanceStatus && ((module.balanceStatus >> index) & 1)) {
                        labelDiv.innerHTML += '<span class="balancing-cell-indicator">B</span>';
                    }
                    
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
                
                // Add temperatures if available
                if (module.temps && module.temps.length > 0) {
                    const tempsDiv = document.createElement('div');
                    tempsDiv.className = 'module-temps';
                    
                    module.temps.forEach((temp, index) => {
                        const tempItem = document.createElement('div');
                        tempItem.className = 'temp-item';
                        
                        const tempLabel = document.createElement('span');
                        tempLabel.className = 'temp-label';
                        tempLabel.textContent = 'Temp ' + (index + 1) + ':';
                        
                        const tempValue = document.createElement('span');
                        tempValue.className = 'temp-value';
                        
                        // Color code temperature
                        if (temp < 10) {
                            tempValue.className += ' cold';
                        } else if (temp < 30) {
                            tempValue.className += ' normal';
                        } else if (temp < 45) {
                            tempValue.className += ' warm';
                        } else {
                            tempValue.className += ' hot';
                        }
                        
                        tempValue.textContent = temp.toFixed(1) + '°C';
                        
                        tempItem.appendChild(tempLabel);
                        tempItem.appendChild(tempValue);
                        tempsDiv.appendChild(tempItem);
                    });
                    
                    moduleDiv.appendChild(headerDiv);
                    moduleDiv.appendChild(cellsDiv);
                    moduleDiv.appendChild(tempsDiv);
                } else {
                    moduleDiv.appendChild(headerDiv);
                    moduleDiv.appendChild(cellsDiv);
                }
                
                modulesContainer.appendChild(moduleDiv);
            });
            
            // Update cell voltage chart
            updateCellChart(data.modules, data);
        }
        
        function updateCellChart(modules, data) {
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
            
            // Calculate stats for scaling and highlighting separately
            // Ignore Module 8 Cell 12 (m=8, c=12) for highlighting statistics
            const validCells = allCells.filter(c => !(c.moduleId === 8 && c.cellIndex === 12));
            
            // If no valid cells (unlikely), fallback to all cells
            const cellsForStats = validCells.length > 0 ? validCells : allCells;
            const validVoltages = cellsForStats.map(c => c.voltage);
            
            const minVoltage = Math.min(...validVoltages);
            const maxVoltage = Math.max(...validVoltages);
            const voltageRange = maxVoltage - minVoltage;
            
            // Create bars for each cell
            allCells.forEach(cell => {
                const bar = document.createElement('div');
                bar.className = 'cell-bar';
                
                // Calculate height relative to the VALID voltage range
                // If the ignored cell is out of range, clamp it to 5% min height
                let heightPercent = 100;
                if (voltageRange > 0) {
                    heightPercent = ((cell.voltage - minVoltage) / voltageRange) * 100;
                }
                bar.style.height = Math.max(5, Math.min(100, heightPercent)) + '%';
                
                // Check if this cell is the ignored one
                const isIgnored = (cell.moduleId === 8 && cell.cellIndex === 12);
                
                // Highlight lowest and highest - ONLY if not the ignored cell
                if (!isIgnored) {
                    if (cell.voltage <= minVoltage + 0.001) { // Fuzzy match for float
                        bar.classList.add('lowest');
                    } else if (cell.voltage >= maxVoltage - 0.001) {
                        bar.classList.add('highest');
                    }
                } else {
                    // Optional: distinct visual style for ignored cell?
                    bar.style.opacity = '0.7'; 
                    bar.style.background = '#888'; // Greyout
                }
                
                // Add tooltip
                const tooltip = document.createElement('div');
                tooltip.className = 'cell-bar-tooltip';
                let tooltipText = `M${cell.moduleId} C${cell.cellIndex}: ${cell.voltage.toFixed(3)}V`;
                if (isIgnored) tooltipText += ' (Ignored)';
                tooltip.textContent = tooltipText;
                bar.appendChild(tooltip);
                
                chartContainer.appendChild(bar);
            });
            
            // Update chart info
            const chartInfo = document.getElementById('chartInfo');
            let totalVoltageStr = '';
            if (data.totalVoltage > 0) {
                totalVoltageStr = ' | Total: ' + data.totalVoltage.toFixed(2) + 'V';
            }
            chartInfo.textContent = `Range: ${minVoltage.toFixed(3)}V - ${maxVoltage.toFixed(3)}V | Δ${(voltageRange * 1000).toFixed(1)}mV | Status: ${data.status} | Target: ${data.activeTargetVoltage.toFixed(3)}V` + totalVoltageStr;
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
        
        function updateSettings() {
            const minBalanceVoltage = parseFloat(document.getElementById('minBalanceVoltage').value);
            const balanceThreshold = parseFloat(document.getElementById('balanceThreshold').value);
            const balanceHysteresis = parseFloat(document.getElementById('balanceHysteresis').value);
            const controllerSuffix = document.getElementById('controllerSuffix').value;
            const autoModeAtStartup = document.getElementById('autoModeAtStartup').checked;
            const balancingFeedbackLimit = document.getElementById('balancingFeedbackLimit').checked;
            const mqttEnabled = document.getElementById('mqttEnabled').checked;
            
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({
                    command: 'updateSettings',
                    minBalanceVoltage: minBalanceVoltage,
                    balanceThreshold: balanceThreshold,
                    balanceHysteresis: balanceHysteresis,
                    controllerSuffix: controllerSuffix,
                    autoModeAtStartup: autoModeAtStartup,
                    balancingFeedbackLimit: balancingFeedbackLimit,
                    mqttEnabled: mqttEnabled
                }));
                
                // Visual feedback
                const btn = event.target;
                const originalText = btn.textContent;
                btn.textContent = '✓ Saved!';
                setTimeout(() => {
                    btn.textContent = originalText;
                }, 1500);
            }
        }
        
        function toggleSettings() {
            const content = document.getElementById('settingsContent');
            const btn = event.target.closest('.settings-toggle-btn');
            
            content.classList.toggle('open');
            btn.classList.toggle('open');
        }
        
        let canDebugTwaiEnabled = false;
        let canDebugMcp2515Enabled = false;
        
        function toggleDebugTwai() {
            canDebugTwaiEnabled = !canDebugTwaiEnabled;
            sendCommand('toggleDebugTwai');
            const btn = document.getElementById('debugBtnTwai');
            btn.textContent = canDebugTwaiEnabled ? '🐛 TWAI Debug: ON' : '🐛 TWAI Debug: OFF';
        }
        
        function toggleDebugMcp2515() {
            canDebugMcp2515Enabled = !canDebugMcp2515Enabled;
            sendCommand('toggleDebugMcp2515');
            const btn = document.getElementById('debugBtnMcp');
            btn.textContent = canDebugMcp2515Enabled ? '🐛 MCP2515 Debug: ON' : '🐛 MCP2515 Debug: OFF';
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
            
            const uploadStartTime = Date.now();
            const formData = new FormData();
            formData.append('update', file);
            
            fetch('/update', {
                method: 'POST',
                body: formData
            })
            .then(response => {
                // If we get any response, assume success (ESP32 will restart)
                statusEl.textContent = 'Update OK - Restarting...';
                fileInput.value = '';
                // Wait longer before reload to give ESP32 time to restart
                setTimeout(() => {
                    location.reload();
                }, 5000);
                return response.text();
            })
            .catch(error => {
                // Network error is EXPECTED because ESP32 restarts immediately
                // Only show error if it happens very quickly (actual upload failure)
                const uploadTime = Date.now() - uploadStartTime;
                if (uploadTime < 1000) {
                    // Upload failed quickly - real error
                    statusEl.textContent = 'Update Failed!';
                    statusEl.className = 'connection-status disconnected';
                    alert('OTA update fejlede: ' + error);
                    fileInput.value = '';
                } else {
                    // Upload took time, connection lost - this is expected during restart
                    statusEl.textContent = 'Update OK - Restarting...';
                    fileInput.value = '';
                    setTimeout(() => {
                        location.reload();
                    }, 5000);
                }
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

  // Load settings from flash
  preferences.begin("bmw-i3-balance", false);
  minBalanceVoltage = preferences.getFloat("minVolts", 3.99f);
  balanceThresholdMv = preferences.getFloat("threshold", 10.0f);
  balanceHysteresisMv = preferences.getFloat("hysteresis", 5.0f);
  controllerSuffix = preferences.getString("suffix", "");
  autoModeAtStartup = preferences.getBool("autoStart", false);
  mqttEnabled = preferences.getBool("mqttEnabled", false);
  balancingFeedbackLimit = preferences.getBool("feedbackLimit", false);

  // Apply auto mode at startup if enabled
  if (autoModeAtStartup)
  {
    manualMode = false;
  }

  Serial.printf("Settings loaded from flash:\n");
  Serial.printf("- Min Voltage: %.2fV\n", minBalanceVoltage);
  Serial.printf("- Threshold: %.0fmV\n", balanceThresholdMv);
  Serial.printf("- Hysteresis: %.0fmV\n", balanceHysteresisMv);
  Serial.printf("- Suffix: '%s'\n", controllerSuffix.c_str());
  Serial.printf("- Auto Start: %s\n", autoModeAtStartup ? "YES" : "NO");
  Serial.printf("- MQTT Enabled: %s\n", mqttEnabled ? "YES" : "NO");
  Serial.printf("- Feedback Limit: %s\n", balancingFeedbackLimit ? "YES" : "NO");

  Serial.println("\n\n=================================");
  Serial.println("BMW i3 Balancing Controller");
  Serial.println("LilyGO T-2CAN");
  Serial.println("Dual CAN Gateway Mode");
  Serial.println("=================================");
  Serial.println("\nCAN Bus Connections:");
  Serial.println("  CAN A (MCP2515) → Slave Modules");
  Serial.println("  CAN B (TWAI)    → BMS");
  Serial.println("=================================\n");

  // Pre-initialize MCP2515 pins before any library calls
  initMCP2515Pins();

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

    // Configure MQTT Server
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

    IPAddress IP = WiFi.localIP();
    Serial.print("IP address: ");
    Serial.println(IP);
    Serial.printf("Open browser: http://%s\n", IP.toString().c_str());
    Serial.printf("Telnet logging: telnet %s 23\n", IP.toString().c_str());

    // Start telnet server
    telnetServer.begin();
    telnetServer.setNoDelay(true);
    Serial.println("✓ Telnet server started on port 23");
  }
  else
  {
    Serial.println("\n✗ WiFi connection failed!");
    Serial.println("Please check SSID and password in code");
  }

  // OTA DISABLED - causes FreeRTOS mutex conflicts after running for a while
  // Use USB serial for updates instead
  Serial.println("\n⚠ OTA updates DISABLED (prevents crashes)");

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

  Serial.println("\n✓ Setup complete!");
  Serial.println("\nGateway Architecture:");
  Serial.println("  TWAI (BMS) <-> ESP32 <-> MCP2515 (Slave Modules)");
  Serial.println("  Normal: Forward BMS requests → slaves, responses → BMS");
  Serial.println("  Balancing: Intercept BMS requests, modify byte 4 to 0x08\n");

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.printf("Access the interface at: http://%s\n\n", WiFi.localIP().toString().c_str());
  }
}

void loop()
{
  // OTA DISABLED - ArduinoOTA.handle() removed to prevent mutex conflicts

  // Check for telnet clients
  checkTelnetClient();

  // Process MQTT
  processMQTT();

  // Clean up WebSocket clients
  static uint32_t lastWSCleanup = 0;
  if (millis() - lastWSCleanup > 5000) // Every 5 seconds only
  {
    lastWSCleanup = millis();
    ws.cleanupClients();
  }

  // CAN processing (critical path)
  readCANMessages(); // Handles both gateway and balancing (via intercept)
  updateBalancing();

  // Debug: Print MCP2515 statistics periodically
  static uint32_t lastMCPDebug = 0;
  if (canDebugMcp2515Enabled && millis() - lastMCPDebug > 5000) // Every 5 seconds
  {
    lastMCPDebug = millis();
    printMCP2515Stats();
  }

  // Broadcast data to WebSocket clients - ONLY if clients are connected
  static uint32_t lastBroadcast = 0;
  if (ws.count() > 0 && millis() - lastBroadcast > 30000) // Only every 30 seconds
  {
    lastBroadcast = millis();
    performBroadcast();
  }

  // Print status every 10 seconds
  static uint32_t lastStatusPrint = 0;
  if (millis() - lastStatusPrint > 10000)
  {
    lastStatusPrint = millis();

    float lowestV = getLowestCellVoltage();
    float highestV = getHighestCellVoltage();

    telnetPrintf("Status: %s | Mode: %s | Cells: %.3fV-%.3fV (Δ%.1fmV)\n",
                 balancingActive ? "BALANCING" : "GATEWAY",
                 manualMode ? "MANUAL" : "AUTO",
                 lowestV, highestV,
                 (highestV - lowestV) * 1000.0f);
  }

  // Minimal delay - maximize CAN throughput
  yield();
}
