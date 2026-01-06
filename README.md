# BMW i3 Balancing Controller

Standalone balancing controller for BMW i3 battery slave modules using LilyGO T-CAN485.

## Features

- ✅ Direct CAN communication with BMW i3 slave modules (0x080-0x08F commands)
- ✅ Real-time cell voltage monitoring (all 12 cells per module)
- ✅ Automatic balancing based on cell voltage differences
- ✅ Manual balancing control via web interface
- ✅ Beautiful responsive web interface
- ✅ WiFi Access Point (no router needed)
- ✅ WebSocket live data streaming
- ✅ Proper BMW CRC8 checksum calculation

## Hardware Requirements

- **LilyGO T-CAN485** (ESP32-S3 + CAN transceiver)
- **BMW i3 Battery** with slave modules
- **CAN connection** to PT-CAN2 bus (500 kbps)

## Pin Connections

```
LilyGO T-CAN485 → BMW i3 Battery
─────────────────────────────────
CAN_H (Pin 17) → PT-CAN2 High
CAN_L (Pin 18) → PT-CAN2 Low
GND            → Ground
```

## Installation

### 1. Install PlatformIO

Install [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) for VS Code.

### 2. Upload Firmware

```bash
cd BMW-i3-Balancing-Controller
pio run --target upload --environment lilygo_tcan485
```

### 3. Connect to WiFi

1. Power on the LilyGO T-CAN485
2. Connect to WiFi: **BMW-i3-Balance**
3. Password: **balance123**
4. Open browser: **http://192.168.4.1**

## Usage

### Web Interface

The web interface shows:
- **Real-time cell voltages** for all modules
- **Balancing status** per module
- **Lowest/Highest cell voltages**
- **Cell voltage difference** in mV
- **Control buttons** (Start/Stop/Auto mode)

### Automatic Mode (Default)

In Auto mode, the controller:
1. Monitors all cell voltages
2. Starts balancing when difference > 10mV
3. Stops balancing when difference < 5mV
4. Targets the lowest cell voltage

### Manual Mode

Click **Start Balancing** to force balancing on/off.
Click **Auto Mode** to return to automatic operation.

## Configuration

Edit `src/main.cpp` to change:

```cpp
#define BALANCE_THRESHOLD_MV 10    // Start balancing threshold
#define BALANCE_HYSTERESIS_MV 5    // Stop balancing threshold
#define CAN_COMMAND_INTERVAL_MS 50 // Command send rate
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

### No modules detected
- Check CAN bus wiring (CAN_H, CAN_L, GND)
- Verify 120Ω termination resistors
- Check CAN bus voltage (2.5V idle on both lines)

### Balancing not starting
- Verify cell voltage difference > 10mV
- Check module error codes in web interface
- Try Manual mode to force balancing

### Web interface not loading
- Reconnect to WiFi: BMW-i3-Balance
- Check IP address in Serial Monitor
- Try different browser

## Serial Monitor Output

View debugging info:
```bash
pio device monitor
```

Output shows:
- CAN driver status
- WiFi AP IP address
- Balancing start/stop events
- Cell voltage data

## Safety Notes

⚠️ **WARNINGS**:
- This controller **bypasses normal BMS safety checks**
- Only use for **diagnostics and testing**
- Monitor battery temperature during balancing
- Do not leave unattended
- Ensure proper ventilation
- Stop balancing if modules get hot (>45°C)

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
