#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include "driver/twai.h"

// ============================================================================
// BMW i3 Balancing Controller for LilyGO T-CAN485
// ============================================================================
// This controller sends balancing commands directly to BMW i3 slave modules
// via CAN bus and monitors cell voltages in real-time.
// ============================================================================

// Pin definitions for LilyGO T-CAN485
#define CAN_TX_PIN GPIO_NUM_27
#define CAN_RX_PIN GPIO_NUM_26
#define CAN_SE_PIN GPIO_NUM_23
#define LED_PIN GPIO_NUM_4  // WS2812 RGB LED

// WiFi Configuration - ENTER YOUR OWN WIFI HERE
const char* WIFI_SSID = "YourWiFiName";      // Change to your WiFi name
const char* WIFI_PASSWORD = "YourWiFiPassword"; // Change to your WiFi password

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
bool manualMode = false;
bool passiveMode = false; // Start in auto mode - balance when needed
bool externalMasterDetected = false;
float targetBalanceVoltage = 4.0f;
uint8_t messageCounter = 0;
uint8_t nextMessage = 0;
uint32_t lastCommandTime = 0;
uint32_t lastExternalCommandTime = 0;
uint32_t lastDataUpdate = 0;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// LED control
Adafruit_NeoPixel led(1, LED_PIN, NEO_GRB + NEO_KHZ800);
uint8_t ledBrightness = 0;
uint32_t lastLedUpdate = 0;

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
  pinMode(CAN_SE_PIN, OUTPUT);
  digitalWrite(CAN_SE_PIN, LOW);
  
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
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

// Parse incoming CAN messages from slave modules
void parseModuleMessage(const twai_message_t &msg)
{
  uint32_t id = msg.identifier;

  // Ignore control messages from other masters (0x080-0x08F)
  if ((id & 0x0F0) == 0x080)
  {
    return; // Don't parse control messages, only data messages
  }

  uint8_t moduleId = (id & 0x00F) + 1;
  uint8_t messageType = (id & 0x0F0);

  if (moduleId > MAX_MODULES)
    return;

  BMWModule &module = modules[moduleId - 1];
  module.exists = true;
  module.lastUpdate = millis();

  // Determine message type based on ID pattern
  uint8_t typeId = 0;
  switch (messageType)
  {
  case 0x000:
    typeId = 0;
    break;
  case 0x020:
    typeId = 1;
    break;
  case 0x030:
    typeId = 2;
    break;
  case 0x040:
    typeId = 3;
    break;
  case 0x050:
    typeId = 4;
    break;
  default:
    return;
  }

  // Parse based on message type
  switch (typeId)
  {
  case 0:
    // Error and balance status
    module.errorCode = msg.data[0] | (msg.data[1] << 8) | (msg.data[2] << 16) | (msg.data[3] << 24);
    module.balanceStatus = ((msg.data[5] & 0x0F) << 8) | msg.data[4];
    break;

  case 1:
    // Cells 0-2
    if (msg.data[1] < 0x40)
      module.cellVoltages[0] = (msg.data[0] + (msg.data[1] & 0x3F) * 256) / 1000.0f;
    if (msg.data[3] < 0x40)
      module.cellVoltages[1] = (msg.data[2] + (msg.data[3] & 0x3F) * 256) / 1000.0f;
    if (msg.data[5] < 0x40)
      module.cellVoltages[2] = (msg.data[4] + (msg.data[5] & 0x3F) * 256) / 1000.0f;
    break;

  case 2:
    // Cells 3-5
    if (msg.data[1] < 0x40)
      module.cellVoltages[3] = (msg.data[0] + (msg.data[1] & 0x3F) * 256) / 1000.0f;
    if (msg.data[3] < 0x40)
      module.cellVoltages[4] = (msg.data[2] + (msg.data[3] & 0x3F) * 256) / 1000.0f;
    if (msg.data[5] < 0x40)
      module.cellVoltages[5] = (msg.data[4] + (msg.data[5] & 0x3F) * 256) / 1000.0f;
    break;

  case 3:
    // Cells 6-8
    if (msg.data[1] < 0x40)
      module.cellVoltages[6] = (msg.data[0] + (msg.data[1] & 0x3F) * 256) / 1000.0f;
    if (msg.data[3] < 0x40)
      module.cellVoltages[7] = (msg.data[2] + (msg.data[3] & 0x3F) * 256) / 1000.0f;
    if (msg.data[5] < 0x40)
      module.cellVoltages[8] = (msg.data[4] + (msg.data[5] & 0x3F) * 256) / 1000.0f;
    break;

  case 4:
    // Cells 9-11
    if (msg.data[1] < 0x40)
      module.cellVoltages[9] = (msg.data[0] + (msg.data[1] & 0x3F) * 256) / 1000.0f;
    if (msg.data[3] < 0x40)
      module.cellVoltages[10] = (msg.data[2] + (msg.data[3] & 0x3F) * 256) / 1000.0f;
    if (msg.data[5] < 0x40)
      module.cellVoltages[11] = (msg.data[4] + (msg.data[5] & 0x3F) * 256) / 1000.0f;
    break;
  }
}

// Read CAN messages
void readCANMessages()
{
  twai_message_t message;
  while (twai_receive(&message, 0) == ESP_OK)
  {
    parseModuleMessage(message);
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

// Send balancing command to slave modules
void sendBalancingCommand()
{
  if (millis() - lastCommandTime < CAN_COMMAND_INTERVAL_MS)
    return;
  lastCommandTime = millis();

  // Cycle through message counter
  if (messageCounter == 0xF)
    messageCounter = 0;

  twai_message_t msg;
  msg.identifier = 0x080 | nextMessage;
  msg.data_length_code = 8;
  msg.flags = TWAI_MSG_FLAG_NONE;
  msg.extd = 0;
  msg.rtr = 0;

  // Calculate target voltage (lowest cell + small margin)
  if (balancingActive)
  {
    uint16_t targetVoltage_mV = (uint16_t)((targetBalanceVoltage) * 1000) + 10;
    msg.data[0] = targetVoltage_mV & 0xFF;
    msg.data[1] = (targetVoltage_mV >> 8) & 0xFF;
  }
  else
  {
    msg.data[0] = 0x68;
    msg.data[1] = 0x10;
  }

  msg.data[2] = 0x00; // Balancing bits
  msg.data[3] = 0x50; // Request voltage and temp measurements

  if (balancingActive)
  {
    msg.data[4] = 0x08; // Request balancing
  }
  else
  {
    msg.data[4] = 0x00; // No balancing
  }

  msg.data[5] = 0x00;
  msg.data[6] = messageCounter << 4;
  msg.data[7] = calculateChecksum(msg, nextMessage);

  // Only send if balancing is active AND not in passive mode
  if (!balancingActive || passiveMode)
    return;

  // Update target voltage during balancing
  targetBalanceVoltage = getLowestCellVoltage();

  // Send CAN message
  twai_transmit(&msg, pdMS_TO_TICKS(10));

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
  else if (millis() - lastExternalCommandTime > 5000)
  {
    externalMasterDetected = false;
  }

  // Don't do auto-balancing in manual mode or passive mode
  if (manualMode || passiveMode)
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
  if (millis() - lastLedUpdate < 50)  // Update every 50ms
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

  if (hasError && (millis() > 10000))  // Give 10 seconds at startup
  {
    // Red solid - Error (no modules)
    led.setPixelColor(0, led.Color(40, 0, 0));
    led.show();
    return;
  }

  if (balancingActive)
  {
    // Blue pulsing - Balancing active
    ledBrightness = (millis() / 10) % 256;
    if (ledBrightness > 127) ledBrightness = 255 - ledBrightness;
    ledBrightness = ledBrightness / 3;  // Max brightness 40
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
          passiveMode = false;
          targetBalanceVoltage = getLowestCellVoltage();
          Serial.println("Manual start - balancing active");
        }
        else if (strcmp(command, "stop") == 0)
        {
          balancingActive = false;
          manualMode = true;
          Serial.println("Manual stop - balancing disabled");
        }
        else if (strcmp(command, "auto") == 0)
        {
          manualMode = false;
          passiveMode = false;
          Serial.println("Auto mode - smart control enabled");
        }
        else if (strcmp(command, "passive") == 0)
        {
          passiveMode = true;
          manualMode = true; // Også sæt manual mode så auto ikke overtager
          balancingActive = false;
          Serial.println("Passive mode - listen only");
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
void broadcastData()
{
  static uint32_t lastBroadcast = 0;
  if (millis() - lastBroadcast < 500)
    return; // Broadcast every 500ms
  lastBroadcast = millis();

  JsonDocument doc;
  
  // Check for errors (no modules detected after 10 seconds)
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
  if (hasError && (millis() > 10000))
  {
    status = "ERROR";
  }
  else if (passiveMode)
  {
    status = "PASSIVE";
  }
  else if (balancingActive)
  {
    status = "BALANCING";
  }
  else
  {
    status = "IDLE";
  }

  // Determine mode
  String mode;
  if (passiveMode)
  {
    mode = "PASSIVE";
  }
  else if (manualMode)
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
  doc["lowestVoltage"] = getLowestCellVoltage();
  doc["highestVoltage"] = getHighestCellVoltage();
  doc["difference"] = (getHighestCellVoltage() - getLowestCellVoltage()) * 1000.0f;
  doc["passive"] = passiveMode;
  doc["externalMaster"] = externalMasterDetected && (millis() - lastExternalCommandTime < 5000);
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

  String jsonString;
  serializeJson(doc, jsonString);
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
            grid-template-columns: repeat(2, 1fr);PASSIVE</div>
            </div>
            <div class="stat-item">
                <div class="stat-label">Mode</div>
                <div class="stat-value" id="mode">AUTO</div>
            </div>
            <div class="stat-item">
                <div class="stat-label">External BMS</div>
                <div class="stat-value" id="externalMaster">-
            padding: 10px;
            border-radius: 8px;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        .cell-voltage {
            font-weight: bold;
            font-size: 1.1em;
        }
        .connection-status {
            position: fixed;
            top: 20px;
            right: 20px;
            padding: 10px 20px;
            <button onclick="sendCommand('passive')">👁️ Passive Mode</button>
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
    </style>
</head>
<body>
    <div class="connection-status" id="connectionStatus">Connecting...</div>
    
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
            <button class="stop" onclick="sendCommand('stop')">⏹ Stop Balancing</button>
            <button class="auto" onclick="sendCommand('auto')">🔄 Auto Mode</button>
            <button onclick="sendCommand('passive')">👁️ Passive Mode</button>
        </div>
        
        <div class="modules" id="modules">
            <!-- Modules will be populated here -->
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
            else if (data.status === 'PASSIVE') statusElement.className += ' warning';
            
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
                
                module.cells.forEach((voltage, index) => {
                    const cellDiv = document.createElement('div');
                    cellDiv.className = 'cell';
                    
                    const labelSpan = document.createElement('span');
                    labelSpan.textContent = 'Cell ' + (index + 1);
                    
                    const voltageSpan = document.createElement('span');
                    voltageSpan.className = 'cell-voltage';
                    voltageSpan.textContent = voltage.toFixed(3) + 'V';
                    
                    cellDiv.appendChild(labelSpan);
                    cellDiv.appendChild(voltageSpan);
                    cellsDiv.appendChild(cellDiv);
                });
                
                moduleDiv.appendChild(headerDiv);
                moduleDiv.appendChild(cellsDiv);
                modulesContainer.appendChild(moduleDiv);
            });
        }
        
        function sendCommand(cmd) {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({command: cmd}));
            }
        }
        
        connectWebSocket();
    </script>
</body>
</html>
)rawliteral";

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\n=================================");
  Serial.println("BMW i3 Balancing Controller");
  Serial.println("LilyGO T-CAN485");
  Serial.println("=================================\n");

  // Initialize LED
  Serial.println("Initializing LED...");
  led.begin();
  led.setPixelColor(0, led.Color(0, 0, 40));  // Blue during startup
  led.show();

  // Initialize CAN bus
  Serial.println("Initializing CAN bus...");
  initCAN();

  // Connect to WiFi Router
  Serial.println("\nConnecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  Serial.print("Connecting");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi connected!");
    IPAddress IP = WiFi.localIP();
    Serial.print("IP address: ");
    Serial.println(IP);
    Serial.printf("Open browser: http://%s\n", IP.toString().c_str());
  } else {
    Serial.println("\n✗ WiFi connection failed!");
    Serial.println("Please check SSID and password in code");
  }

  // Setup WebSocket
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  // Setup web server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/html", index_html); });

  server.begin();
  Serial.println("Web server started");
  Serial.println("\n✓ Setup complete!");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Access the interface at: http://%s\n\n", WiFi.localIP().toString().c_str());
  }
}

void loop()
{
  // Read incoming CAN messages (always listen)
  readCANMessages();

  // Update balancing logic (decides if we should take control)
  updateBalancing();

  // Send balancing commands (only if balancing is active and not in passive mode)
  sendBalancingCommand();

  // Update LED status
  updateLED();

  // Broadcast data to web clients
  broadcastData();

  // Clean up WebSocket clients
  ws.cleanupClients();

  // Print status every 10 seconds
  static uint32_t lastStatusPrint = 0;
  if (millis() - lastStatusPrint > 10000)
  {
    lastStatusPrint = millis();
    Serial.printf("Status: %s | Mode: %s | Cells: %.3fV-%.3fV (Δ%.1fmV) | External BMS: %s\n",
                  balancingActive ? "ACTIVE" : (passiveMode ? "PASSIVE" : "IDLE"),
                  manualMode ? "MANUAL" : "AUTO",
                  getLowestCellVoltage(), getHighestCellVoltage(),
                  (getHighestCellVoltage() - getLowestCellVoltage()) * 1000.0f,
                  (externalMasterDetected && (millis() - lastExternalCommandTime < 5000)) ? "YES" : "NO");
  }

  delay(10);
}
