# BMW i3 Balancing Controller

Gateway balancing controller for BMW i3 battery modules - forwards messages between BMS and slave modules.

## Features

- ✅ **Gateway mode**: Forwards BMS requests to slave modules and responses back to BMS
- ✅ **Balancing mode**: Blocks BMS requests, sends own balancing commands + data requests
- ✅ Real-time cell voltage monitoring (96 cells across 8 modules)
- ✅ Smart balancing with configurable voltage thresholds
- ✅ Automatic balancing based on cell voltage differences
- ✅ Manual balancing control via web interface
- ✅ **MQTT integration** with Home Assistant Discovery
- ✅ **Control via Home Assistant**: Start/Stop/Auto buttons via MQTT
- ✅ **Duty Cycle**: Configurable balancing pause/resume timing (4-9 min run, 1-10 min pause)
- ✅ WiFi connectivity - connects to your router
- ✅ WebSocket live data streaming
- ✅ Proper BMW CRC8 checksum calculation

## Hardware Requirements

### Supported Board

- **LilyGO T-2CAN** (ESP32-S3 + dual CAN controllers)

This board has the necessary dual CAN interface to handle gateway operations between BMS and battery modules.

### Additional Requirements

- **BMW i3 Battery** with slave modules (8 modules, 12 cells each = 96 cells total)
- **CAN connection** to PT-CAN bus (500 kbps)

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

### LilyGO T-2CAN

```
TWAI CAN (CAN B) → Ekstern BMS

MCP2515 CAN (CAN A) → BMW i3 Slave Modules
```

**Power Supply (CRITICAL)**:
- LilyGO must be powered by **12V from klemme 30C** (same power supply as battery)
- Use same ground connection as battery

### Wiring Diagram

```
                 12V (Klemme 30C) ──────────┐
                 GND ───────────────────┐   │
                                        │   │
                                    ┌───┼───┼────────────────────────────┐
                                    │   │   │  LilyGO T-2CAN (ESP32-S3)  │
                                    │  GND 12V                           │
                                    │  ┌──────────┐      ┌──────────┐    │
                                    │  │  TWAI    │      │ MCP2515  │    │
                                    │  │  (CAN B) │      │ (CAN A)  │    │
                                    │  │          │      │          │    │
                                    │  │          │      │          │    │
                                    │  └────┬─────┘      └─────┬────┘    │
                                    │       │                  │         │
                                    │       │                  │         │
                                    └───────┼──────────────────┼─────────┘
                                            │                  │
                                      ┌─────┴─────┐      ┌─────┴─────┐
                          BMS Pin 1   │  CAN_H    │      │  CAN_H    │ Yellow/red
                          BMS Pin 2   │  CAN_L    │      │  CAN_L    │ Yellow/brown
                                      │           │      │           │
                                      └─────┬─────┘      └─────┬─────┘
                                            │                  │
    ┌──────────────┐                        │                  │
    │  External    │  ◄─────────────────────┘                  │
    │  BMS         │  (Direct connection)                      │
    │  [120Ω term.]│                                           │
    └──────┬───────┘                                           │
           │                                                   │
           │  CAN Loop (Only used for termination)             │
           │  (CAN_H/CAN_L/)                                   │
           │                                                   │
           └──────────────────────────┐                        │
                                      │                        │
                               ┌──────▼───────────────┐        │
                               │  Slave Module 1      │        │
                               │  (0x080)             │        │
                               │                      │        │
                               └──────┬───────────────┘        │
                                      │                        │
                               ┌──────▼───────────────┐        │
                               │  Slave Module 2      │        │
                               │  (0x081)             │        │
                               └──────┬───────────────┘        │
                                      │                        │
                                      :                        │
                                      │                        │
                               ┌──────▼───────────────┐        │
                               │  Slave Module 8      │        │
                               │  (0x087)             │◄───────┘
                               └──────────────────────┘  (MCP2515 connection)


Power Connections:
  Battery Klemme 30C (12V) ──► LilyGO 12V input
  Battery GND ──────────────► LilyGO GND
```

**Connection Details:**
- **TWAI CAN B** (GPIO 7 TX, GPIO 6 RX) → External BMS
  - BMS CanH pin 1 Yellow/red
  - BMS CanL pin 2 Yellow/brown
  - Direct connection to BMS
  - Used for report data back to BMS
  - Lilygo has 120Ω termination resistor
  - no spare resistor nedded
  
- **MCP2515 CAN A** (Internal SPI connection) → Slave Module
  - Slave modul CanH Yellow/red wire to CanL A
  - Slave modul Can L Yellow/brown wire to CanH A
  - no spare resistor nedded
  - Connects to CAN loop at Slave Module 
  - CAN loop: BMS → Slave Module 1 → ... → Slave Module 8 → back to BMS
  - BMS has built-in 120Ω termination resistor
  - Lilygo has 120Ω termination resistor
  - Forms complete CAN bus loop through all slave modules
  
- **Power**: 12V from klemme 30C + GND from battery
  - Same power supply as battery pack
  - Ensures proper ground reference for CAN communication

**Note**: 
- TWAI er påkrævet for gateway funktionalitet (forbindelse til BMS)
- MCP2515 forbindes direkte til BMW i3 slave moduler

## Installation

### 1. Install PlatformIO

Install [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) for VS Code.

### 2. Configure WiFi and MQTT

Create `src/credentials.h` from `src/credentials.h.example`:

```cpp
// WiFi Settings
#define WIFI_SSID "YourWiFiName"
#define WIFI_PASSWORD "YourWiFiPassword"

// MQTT Settings (leave empty for disabled/anonymous)
#define MQTT_BROKER "192.168.1.100"
#define MQTT_USER "your_username"
#define MQTT_PASSWORD "your_password"
```

Or use the provided template as starting point.

### 3. Upload Firmware

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

## Startup Sequence (IMPORTANT)

The correct power-up sequence is critical to avoid slave module errors:

1. **Power battery to klemme 30C** (battery remains in standby/sleep mode)
2. **Wait for LilyGO to fully boot**
   - System will show error state initially (this is normal)
3. **Enable battery emulator** (or wake up battery from standby)
   - This starts module communication (system status will show "system OK")
   - Battery begins sending on CAN bus
   - Controller now receives slave module responses
   - Error state clears and system becomes operational

**If you encounter slave module errors:**
- Temporarily connect **both CAN bus lines** to MCP2515 CAN A
- Wait for errors to clear ( this can few min up to maybe a hour)
- Then move BMS CAN bus back to TWAI CAN B
- System will now operate normally

**Why this matters:**
- LilyGO must be fully initialized before battery communication starts
- Otherwise slave modules may not respond correctly
- Recovery sequence allows proper synchronization

## How It Works

### Gateway Mode

The controller operates as a transparent gateway:

1. **Normal Operation**: 
   - Forwards BMS requests (0x080-0x08F) from TWAI → slave modules via MCP2515
   - Forwards slave responses (0x100-0x1FF) from MCP2515 → BMS via TWAI 
   - Continuously monitors all 96 cell voltages

2. **Balancing Mode**:
   - Blocks BMS requests (not forwarded)
   - Sends own balancing commands + voltage requests and balancing target to slave modules
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
6. **Duty Cycle** (if enabled):
   - Runs balancing for configurable period (default 9 minutes)
   - Pauses for configurable period (default 2 minutes)
   - Allows battery to stabilize cell voltage reading more between balancing
   - When disabled: Slave modules run their own built-in duty cycle (9 min on, 1 min off)

### Manual Mode

- **Start Balancing**: Force balancing to start immediately (blocks BMS requests)
- **Stop / Gateway Mode**: Stop balancing, return to gateway mode (forwards BMS requests)
- **Auto Mode**: Return to smart automatic balancing
- **Gateway Mode**: Force gateway mode (forwards all traffic, no balancing)

## Configuration

### WiFi and MQTT Setup

All credentials are configured in `src/credentials.h`:

1. **Create credentials.h** from the example file:
   ```bash
   cp src/credentials.h.example src/credentials.h
   ```

2. **Edit credentials.h** with your settings:
   ```cpp
   // WiFi Settings
   #define WIFI_SSID "YourWiFiName"
   #define WIFI_PASSWORD "YourWiFiPassword"

   // MQTT Settings (leave empty for disabled or anonymous broker)
   #define MQTT_BROKER "192.168.1.xxx"
   #define MQTT_USER "your_username"      // Leave empty for anonymous
   #define MQTT_PASSWORD "your_password"
   ```

3. **Enable MQTT in web interface** after first boot

### MQTT Battery Emulator (Discharge Block)

The controller can read battery emulator data from MQTT and pause balancing while the battery is discharging. This prevents balancing during load and resumes automatically when discharge stops.

- Source: MQTT payload on BE/info from the battery emulator
- Used values: `battery_current` + `SOC` (or `battery_current_2` + `SOC_2` when controller suffix is `2`)
- Behavior: If discharge is detected or data becomes stale, balancing is blocked until values return to normal
- Setting: Enable **MQTT Discharge Block** in the web interface (default is OFF)

### Other Configuration

Balancing parameters can be changed via the **Settings** menu in the web interface:

```cpp
// Balancing Parameters (adjustable from web interface Settings)
MIN_BALANCE_VOLTAGE 3.99    // Minimum voltage to start balancing (V)
BALANCE_THRESHOLD_MV 10     // Start balancing if cells differ by more than this (mV)
BALANCE_HYSTERESIS_MV 5     // Stop balancing when within this (mV)
CELL_VOLTAGE_OFFSET 0.002   // Voltage offset added to target (V)

// Duty Cycle (adjustable from web interface Settings)
DUTY_CYCLE_ENABLED true     // Enable duty cycle (default: ON)
DUTY_CYCLE_ON_MIN 9         // Run time in minutes (4-9)
DUTY_CYCLE_PAUSE_MIN 2      // Pause time in minutes (1-10)
```

**Note**: These settings are saved to flash memory and persist after reboot.

## Home Assistant Integration

### Automatic Discovery

When MQTT is enabled and connected, the controller automatically publishes Home Assistant Discovery payloads.

### Available Sensors

Published MQTT metrics (topic: `homeassistant/home/bmw-i3-bms/metrics`):
- `voltage_lowest` - Lowest cell voltage (3 decimals)
- `voltage_highest` - Highest cell voltage (3 decimals)
- `voltage_diff_mv` - Cell voltage difference in mV
- `status` - Controller status (IDLE/BALANCING/BALANCING_PAUSED/ERROR)
- `auto_mode` - Operating mode (Auto/Manual)
- `system_status` - System health (System OK/Error)
- `module_1_balancing` through `module_8_balancing` - Per-module balancing status

### Control Buttons

Three buttons available for Home Assistant automation:
- **Start Balancing** - Force immediate balancing start
- **Stop / Gateway Mode** - Stop balancing and return to gateway mode
- **Auto Mode** - Enable automatic balancing

Usage in Home Assistant:
```yaml
# Example automation
automation:
  - alias: "Start BMW Balancing"
    trigger:
      platform: time
      at: "22:00:00"
    action:
      service: mqtt.publish
      data:
        topic: "homeassistant/home/bmw-i3-bms/cmd/start"
        payload: "1"
```

## MQTT Topics

Base topic: `homeassistant/home/bmw-i3-bms` (or with suffix if configured)

**Publish (from controller):**
- `.../metrics` - All telemetry data (JSON)
- `.../status` - Connection status (online/offline)

**Subscribe (from Home Assistant):**
- `.../cmd/start` - Start balancing
- `.../cmd/stop` - Stop balancing
- `.../cmd/auto` - Auto mode

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

### Incoming Messages (0x100-0x17F)

Module responses contain (where X is the module number 0-7):
- **0x10X**: Error codes and balance status
- **0x12X**: Cell voltages (Cells 1-3)
- **0x13X**: Cell voltages (Cells 4-6)
- **0x14X**: Cell voltages (Cells 7-9)
- **0x15X**: Cell voltages (Cells 10-12)
- **0x16X**: Module Total Voltage
- **0x17X**: Temperature data

## Troubleshooting

### WiFi not connecting
- Check SSID and password in code
- Verify router is powered on and in range
- Open Serial Monitor to see connection attempts
- Controller will retry for 15 seconds

### No modules detected
- Check CAN bus wiring (CAN_H, CAN_L)
- Verify 120Ω termination resistors
- Check CAN bus voltage (2.5V idle on both lines)
- Ensure battery is awake (battery emulator on)

### Balancing not starting
- Verify MCP2515 is connected and detected (shows on web interface)
- Check cell voltage difference > 10mV and highest cell > 3.9V
- Try Manual mode to force balancing
- Verify modules are responding (check Serial Monitor)
- Check module error codes in web interface
- If duty cycle is enabled, check if currently in pause phase

### MQTT not connecting
- Verify broker address and credentials in `credentials.h`
- Check WiFi connection is successful (IP shown on Serial Monitor)
- Enable MQTT in web interface settings
- Verify MQTT broker is accessible from network
- Check Serial Monitor for MQTT connection logs

### Home Assistant buttons not appearing
- Ensure MQTT is enabled and connected (check web interface)
- Check Home Assistant MQTT integration is configured
- Wait for Home Assistant to receive discovery payloads (30-60 seconds)
- Manually trigger discovery in Home Assistant if needed

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

### Battery contactor opens during debug mode
- High Serial.println load in debug mode can cause timing issues
- Battery contactor may open due to communication delays
- **Solution**: Restart battery emulator to re-close contactor
- **Prevention**: Reduce debug output or disable verbose logging during normal operation
- This is a known limitation when using extensive serial debugging

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
