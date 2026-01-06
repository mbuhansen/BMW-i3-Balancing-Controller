# BMW i3 Balancing Controller

Gateway balancing controller for BMW i3 battery modules - forwards messages between BMS and slave modules.

## Features

- ✅ **Gateway mode**: Forwards BMS requests to slave modules and responses back to BMS
- ✅ **Balancing mode**: Blocks BMS requests, sends own balancing commands + data requests
- ✅ Real-time cell voltage monitoring (96 cells across 8 modules)
- ✅ Smart balancing with configurable voltage thresholds
- ✅ Automatic balancing based on cell voltage differences
- ✅ Manual balancing control via web interface
- ✅ WiFi connectivity - connects to your router
- ✅ WebSocket live data streaming
- ✅ Proper BMW CRC8 checksum calculation
- ✅ Dual-core operation: Gateway on Core 0, Web UI on Core 1

## Hardware Requirements

### Supported Boards

- **LilyGO T-CAN485** (ESP32 + CAN transceiver + MCP2515)
- **LilyGO T-2CAN** (ESP32-S3 + dual CAN controllers)

Choose the board that matches your hardware:
- T-CAN485: Single CAN, best for basic standalone operation
- T-2CAN: Dual CAN, supports gateway mode and dual-bus operation

### Additional Requirements

- **BMW i3 Battery** with slave modules (8 modules, 12 cells each = 96 cells total)
- **CAN connection** to PT-CAN2 bus (500 kbps)

## CAN Bus Arkitektur

```
[BMS] ←→ MCP2515 ←→ [ESP32 Gateway] ←→ TWAI ←→ [Slave Modules]
```

### Gateway Funktionalitet

**Normal Mode (ikke balancering):**
- BMS sender requests (0x080-0x08F) → videresender til slave moduler via TWAI
- Slave moduler sender svar (0x100-0x1FF) → videresender tilbage til BMS via MCP2515

**Balancing Mode (aktiv balancering):**
- BMS requests **blokeres** (ikke videresendt)
- ESP32 sender egne balancing commands + data requests til slave moduler
- Slave moduler sender svar (0x100-0x1FF) → videresender **stadig** tilbage til BMS

### LilyGO T-CAN485

```
TWAI CAN → BMW i3 Slave Modules
  GPIO 27 (TX)
  GPIO 26 (RX)
  GPIO 23 (SE - Standby Enable)

MCP2515 CAN → Ekstern BMS
  GPIO 18 (CS)
  GPIO 35 (INT)
  GPIO 12 (SCK)
  GPIO 5 (MOSI)
  GPIO 34 (MISO)
```

### LilyGO T-2CAN

```
TWAI CAN (CAN B) → BMW i3 Slave Modules
  GPIO 7 (TX)
  GPIO 6 (RX)

MCP2515 CAN (CAN A) → Ekstern BMS
  GPIO 10 (CS)
  GPIO 8 (INT)
  GPIO 12 (SCK)
  GPIO 11 (MOSI)
  GPIO 13 (MISO)
```

**Note**: 
- MCP2515 er påkrævet for gateway funktionalitet (forbindelse til BMS)
- TWAI forbindes direkte til BMW i3 slave moduler
- T-2CAN har indbyggede CAN transceivers
- T-2CAN har ingen onboard LED

## Installation

### 1. Install PlatformIO

Install [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) for VS Code.

### 2. Configure WiFi

Edit `src/main.cpp` and change WiFi credentials:

```cpp
const char* WIFI_SSID = "YourWiFiName";
const char* WIFI_PASSWORD = "YourWiFiPassword";
```

### 3. Upload Firmware

**For T-CAN485:**
```bash
cd BMW-i3-Balancing-Controller
pio run --target upload --environment lilygo_tcan485
```

**For T-2CAN:**
```bash
cd BMW-i3-Balancing-Controller
pio run --target upload --environment lilygo_t2can
```

### 4. Find IP Address

1. Open Serial Monitor (115200 baud)
2. Watch for WiFi connection and IP address
3. Note the IP address (e.g., 192.168.1.100)

### 5. Access Web Interface

Open browser and navigate to the IP address shown in Serial Monitor.

## How It Works

### Gateway Mode

The controller operates as a transparent gateway:

1. **Normal Operation**: 
   - Forwards BMS requests (0x080-0x08F) from MCP2515 → slave modules via TWAI
   - Forwards slave responses (0x100-0x1FF) from TWAI → BMS via MCP2515
   - Continuously monitors all 96 cell voltages

2. **Balancing Mode**:
   - Blocks BMS requests (not forwarded)
   - Sends own balancing commands + voltage requests to slave modules
   - Still forwards slave responses back to BMS
   - Balances cells when voltage difference exceeds threshold

3. **Smart Balancing**:
   - Auto mode: Starts balancing when cells differ by >10mV
   - Manual mode: User controls when to start/stop balancing
   - Always forwards module data back to BMS

This allows you to:
- Use as transparent gateway between BMS and slave modules
- Monitor cell voltages in real-time via web interface
- Take over balancing control when needed
- BMS still receives all slave module data

## Usage

### Web Interface

The responsive web interface displays:
- **System Status**: GATEWAY / BALANCING
- **Operation Mode**: AUTO / MANUAL
- **MCP2515 Status**: Shows if gateway CAN is available
- **Real-time cell voltages** for all 96 cells (8 modules × 12 cells)
- **Balancing status** per module
- **Lowest/Highest cell voltages** across entire pack
- **Cell voltage difference** in mV
- **Control buttons**:
  - ▶ Start Balancing (Manual)
  - ⏹ Stop / Gateway Mode
  - 🔄 Auto Mode
  - 🔀 Gateway Mode

### Automatic Mode (Default)

In Auto mode, the controller:
1. Continuously monitors all 96 cell voltages via gateway
2. Operates in gateway mode (forwards BMS ↔ slave module traffic)
3. Automatically starts balancing when:
   - Cell voltage difference > 10mV
   - Highest cell > 3.9V (MIN_BALANCE_VOLTAGE)
4. During balancing:
   - Blocks BMS requests temporarily
   - Sends own commands with balance target + voltage requests
   - Stops when cells are within 5mV of each other
5. Returns to gateway mode when balancing complete

### Manual Mode

- **Start Balancing**: Force balancing to start immediately (blocks BMS requests)
- **Stop / Gateway Mode**: Stop balancing, return to gateway mode (forwards BMS requests)
- **Auto Mode**: Return to smart automatic balancing
- **Gateway Mode**: Force gateway mode (forwards all traffic, no balancing)

## Configuration

Edit `src/main.cpp` to customize:

```cpp
// WiFi Settings
const char* WIFI_SSID = "YourWiFiName";
const char* WIFI_PASSWORD = "YourWiFiPassword";

// Balancing Parameters
#define BALANCE_THRESHOLD_MV 10    // Start balancing threshold
#define BALANCE_HYSTERESIS_MV 5    // Stop balancing threshold
#define CAN_COMMAND_INTERVAL_MS 50 // Command send rate

// Hardware Configuration
#define MAX_MODULES 8              // Number of battery modules
#define CELLS_PER_MODULE 12        // Cells per module (96 total)
```

## CAN Protocol Details

### Outgoing Messages (0x080-0x08F)

```
ID: 0x080 + module_number (0-7)
DLC: 8
Data:
  [0-1]: Target voltage (mV, little endian)
  [2]:   Balancing bits
  [3]:   0x50 = Request voltage + temp
  [4]:   0x08 = Enable balancing, 0x00 = Disable
  [5]:   0x00
  [6]:   Message counter << 4
  [7]:   CRC8 checksum
```

### Incoming Messages (0x000-0x05F)

Module responses contain:
- **0xXX0**: Error codes and balance status
- **0xX21-0xX25**: Cell voltages (groups of 3 cells)
- **Temperature data** (if requested)

## Troubleshooting

### WiFi not connecting
- Check SSID and password in code
- Verify router is powered on and in range
- Open Serial Monitor to see connection attempts
- Controller will retry for 15 seconds

### No modules detected
- Check CAN bus wiring (CAN_H, CAN_L, GND)
- Verify 120Ω termination resistors
- Check CAN bus voltage (2.5V idle on both lines)
- Ensure battery is awake (SimpleBMS sending queries)

### Balancing not starting
- Verify MCP2515 is connected and detected (shows on web interface)
- Check cell voltage difference > 10mV and highest cell > 3.9V
- Try Manual mode to force balancing
- Verify modules are responding (check Serial Monitor)
- Check module error codes in web interface

### Web interface not loading
- Verify WiFi connection in Serial Monitor
- Check IP address shown at startup
- Try pinging the IP address
- Ensure device is on same network
- Try different browser or clear cache

### Gateway not forwarding messages
- Check MCP2515 wiring and connections (see pinout above)
- Verify MCP2515 status shows "Available: Yes" in web interface
- Check CAN bus termination (120Ω on both ends)
- Monitor Serial output with CAN Debug enabled to see message flow
- Verify BMS is sending requests on MCP2515 CAN bus

## Serial Monitor Output

View debugging info via USB serial connection:
```bash
pio device monitor
```

Or use PuTTY:
- Connection type: Serial
- COM port: (check Device Manager)
- Baud rate: 115200

Output shows:
- CAN driver initialization status (TWAI + MCP2515)
- WiFi connection status and IP address
- Gateway message forwarding (when debug enabled)
- Module detection and cell voltages
- Balancing start/stop events
- Mode changes (GATEWAY/BALANCING)
- Status updates every 10 seconds

Example output:
```
BMW i3 Balancing Controller
✓ CAN driver started
✓ MCP2515 initialized - gateway mode active
✓ WiFi connected!
IP address: 192.168.1.100
[GATEWAY] BMS→Slave: 0x080 forwarded
[GATEWAY] Slave→BMS: 0x120 forwarded
Status: GATEWAY | Mode: AUTO | Cells: 3.850V-3.862V (Δ12.0mV)
```

## Safety Notes

⚠️ **IMPORTANT WARNINGS**:
- This controller acts as gateway between BMS and slave modules
- During balancing, BMS requests are temporarily blocked
- Monitor battery temperature during balancing
- Stop balancing if modules get hot (>45°C)
- Ensure proper ventilation
- Do not leave balancing unattended for extended periods
- Test gateway mode first to verify proper message forwarding

✅ **Safe Usage**:
- Gateway mode: Transparent forwarding of BMS ↔ slave module traffic
- Balancing mode: Controller takes over with own commands
- Web interface accessible in all modes
- Real-time monitoring of all 96 cells
- Manual override available when needed
- Slave responses always forwarded to BMS (even during balancing)

## License

MIT License - Use at your own risk

## Credits

Based on SimpleBMS by Tom de Bree (Tom-evnut)
- https://github.com/Tom-evnut/BMWI3BMS

## Support

For issues or questions:
1. Check serial monitor for errors
2. Verify CAN bus connections
3. Ensure battery is awake (modules responding)
4. Check module error codes in web interface
