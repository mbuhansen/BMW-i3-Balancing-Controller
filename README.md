# BMW i3 Balancing Controller

Standalone balancing controller for BMW i3 battery slave modules using LilyGO T-CAN485.

## Features

- ✅ Direct CAN communication with BMW i3 slave modules (0x080-0x08F commands)
- ✅ Real-time cell voltage monitoring (96 cells across 8 modules)
- ✅ Smart balancing with external BMS detection
- ✅ Passive listening mode - only sends commands when safe
- ✅ Automatic balancing based on cell voltage differences
- ✅ Manual balancing control via web interface
- ✅ WiFi connectivity - connects to your router
- ✅ WebSocket live data streaming
- ✅ Proper BMW CRC8 checksum calculation

## Hardware Requirements

- **LilyGO T-CAN485** (ESP32dev + CAN transceiver)
- **BMW i3 Battery** with slave modules
- **CAN connection** to PT-CAN2 bus (500 kbps)

## Pin Connections

```
ESP32 Dev Board → CAN Transceiver → BMW i3 Battery
──────────────────────────────────────────────────
GPIO 27 (CAN_TX) → CTX/TXD        → 
GPIO 26 (CAN_RX) → CRX/RXD        →
GPIO 23 (CAN_SE) → Silent Enable  → (optional)
3.3V             → VCC            →
GND              → GND            → Ground
                   CANH           → PT-CAN2 High
                   CANL           → PT-CAN2 Low
```

**Note**: Adjust GPIO pins in code if using different connections.

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

```bash
cd BMW-i3-Balancing-Controller
pio run --target upload --environment lilygo_tcan485
```

### 4. Find IP Address

1. Open Serial Monitor (115200 baud)
2. Watch for WiFi connection and IP address
3. Note the IP address (e.g., 192.168.1.100)

### 5. Access Web Interface

Open browser and navigate to the IP address shown in Serial Monitor.

## How It Works

### Smart Passive Mode

The controller operates intelligently:

1. **Always Listening**: Continuously monitors all 96 cell voltages from CAN bus
2. **External BMS Detection**: Detects if SimpleBMS or other master is sending balancing commands
3. **Priority System**:
   - If external master detected: Stay in **PASSIVE mode** (read-only)
   - If no external master for 5+ seconds: Switch to **ACTIVE mode** (can send commands)
4. **Safe Balancing**: Only sends CAN commands when it's safe and necessary

This allows you to:
- Use alongside SimpleBMS without conflicts
- Monitor cell voltages in real-time
- Take over balancing when SimpleBMS is not running
- Manually control balancing via web interface

## Usage

### Web Interface

The responsive web interface displays:
- **System Status**: PASSIVE / BALANCING / IDLE
- **Operation Mode**: AUTO / MANUAL
- **External BMS Detection**: Shows if SimpleBMS is active
- **Real-time cell voltages** for all 96 cells (8 modules × 12 cells)
- **Balancing status** per module
- **Lowest/Highest cell voltages** across entire pack
- **Cell voltage difference** in mV
- **Control buttons**:
  - ▶ Start Balancing (Manual)
  - ⏹ Stop Balancing
  - 🔄 Auto Mode
  - 👁️ Passive Mode

### Automatic Mode (Default)

In Auto mode, the controller:
1. Continuously monitors all 96 cell voltages
2. Detects if external BMS (SimpleBMS) is active
3. If no external master detected:
   - Starts balancing when difference > 10mV
   - Stops balancing when difference < 5mV
   - Targets the lowest cell voltage
4. If external master detected:
   - Stays in passive mode (read-only)
   - Does not interfere with external BMS

### Manual Mode

- **Start Balancing**: Force balancing to start immediately
- **Stop Balancing**: Stop all balancing operations
- **Auto Mode**: Return to smart automatic operation
- **Passive Mode**: Listen only, never send commands

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
- Check if external BMS is detected (shows on web interface)
- In Auto mode, balancing only starts if no external master
- Try Manual mode to force balancing
- Verify cell voltage difference > 10mV
- Check module error codes in web interface

### Web interface not loading
- Verify WiFi connection in Serial Monitor
- Check IP address shown at startup
- Try pinging the IP address
- Ensure device is on same network
- Try different browser or clear cache

### External BMS always detected
- SimpleBMS may be running - this is normal
- Controller will stay in passive mode (safe)
- Stop SimpleBMS to allow this controller to take over
- Or use Passive Mode button to force read-only mode

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
- CAN driver initialization status
- WiFi connection status and IP address
- Module detection and cell voltages
- Balancing start/stop events
- External BMS detection
- Mode changes (PASSIVE/ACTIVE)
- Status updates every 10 seconds

Example output:
```
BMW i3 Balancing Controller
✓ CAN driver started
✓ WiFi connected!
IP address: 192.168.1.100
Status: PASSIVE | Mode: AUTO | Cells: 3.850V-3.862V (Δ12.0mV) | External BMS: YES
```

## Safety Notes

⚠️ **IMPORTANT WARNINGS**:
- This controller can coexist with SimpleBMS safely
- **ALWAYS monitors passively** - only sends commands when safe
- Automatically detects external BMS and yields control
- Monitor battery temperature during balancing
- Stop balancing if modules get hot (>45°C)
- Ensure proper ventilation
- Do not leave balancing unattended for extended periods
- Test in PASSIVE mode first before enabling AUTO/MANUAL modes

✅ **Safe Usage**:
- Can run alongside SimpleBMS without conflicts
- Web interface accessible even in PASSIVE mode
- Real-time monitoring of all 96 cells
- Manual override available when needed

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
