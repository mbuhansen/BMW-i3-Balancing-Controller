# BMW i3 Balancing Controller - Dual CAN Gateway Arkitektur

## 🔌 Oversigt

ESP32'en fungerer nu som en **CAN Gateway/Proxy** mellem BMW BMS og slave modulerne med 2 separate CAN busser:

```
┌─────────────┐         ┌──────────────────┐         ┌──────────────┐
│  BMW BMS    │ ◄─────► │  ESP32 Gateway   │ ◄─────► │ Slave Modul  │
│             │ MCP2515 │  (LilyGO T-2CAN) │  TWAI   │   1-8        │
└─────────────┘         └──────────────────┘         └──────────────┘
```

## 🚌 CAN Bus Opdeling

### **CAN Bus 1 - MCP2515 (SPI)**
- **Hardware:** LilyGO T-2CAN indbygget MCP2515 CAN controller (CAN A)
- **Pin:** GPIO 10 (CS), GPIO 8 (INT), GPIO 12 (SCK), GPIO 11 (MOSI), GPIO 13 (MISO)
- **Forbindelse:** BMW BMS
- **Funktion:** 
  - Modtager kommandoer fra BMW BMS (ID `0x080-0x087`)
  - Sender modul svar tilbage til BMS (ID `0x100-0x1FF`)

### **CAN Bus 2 - TWAI (ESP32-S3 Indbygget)**
- **Hardware:** T-2CAN indbygget CAN controller (CAN B)
- **Pin:** GPIO 7 (TX), GPIO 6 (RX)
- **Forbindelse:** BMW i3 Slave Moduler
- **Funktion:**
  - Sender kommandoer til modulerne (ID `0x080-0x087`)
  - Modtager celledata fra modulerne (ID `0x100-0x1FF`)

## 🔄 CAN Gateway Funktionalitet

### Normal Drift (Passiv Mode)
ESP32 fungerer som transparent gateway:

1. **BMS → ESP32 (TWAI):** BMW BMS sender kommando til modul
2. **ESP32 → Modul (MCP2515):** Gateway videresender kommando
3. **Modul → ESP32 (MCP2515):** Modul sender celledata
4. **ESP32 → BMS (TWAI):** Gateway videresender data til BMS

```
BMS --[0x080]--> TWAI --[Gateway]--> MCP2515 --[0x080]--> Modul
BMS <--[0x100]-- TWAI <--[Gateway]-- MCP2515 <--[0x100]-- Modul
```

### Balancing Mode (Aktiv)
ESP32 overtager kontrollen:

1. **BMS → ESP32 (TWAI):** BMW BMS sender kommando
2. **ESP32:** **BLOKERER** BMS kommando (!)
3. **ESP32 → Modul (MCP2515):** Sender EGEN balancing kommando
4. **Modul → ESP32 (MCP2515):** Modul sender celledata
5. **ESP32 → BMS (TWAI):** Gateway sender data til BMS (som normalt)

```
BMS --[0x080]--> TWAI --[BLOCKED]
                   ↓
ESP32 --[Custom 0x080]--> MCP2515 --[0x080]--> Modul
BMS <--[0x100]-- TWAI <--[Gateway]-- MCP2515 <--[0x100]-- Modul
```

## 📋 Message Flow

### Kommando Beskeder (0x080-0x087)
- **Normal:** BMS kommandoer videresendes til moduler
- **Balancing:** ESP32 blokerer BMS og sender egne kommandoer

### Data Beskeder (0x100-0x1FF)
- **Altid:** Videresendes fra moduler → BMS
- ESP32 parser også data lokalt for web interface

## 🎛️ Modes

### 1. **Passive Mode** (Standard)
- Gateway videresender alt
- Ingen egne kommandoer
- BMS har fuld kontrol

### 2. **Manual Balancing**
- Gateway blokerer BMS kommandoer
- Sender egne balancing kommandoer
- Modul svar sendes stadig til BMS

### 3. **Auto Balancing**
- Som Manual, men automatisk aktivering
- Starter når celle-difference > 10mV
- Stopper når celle-difference < 5mV

## 🔧 Tekniske Detaljer

### CAN Message Format
```cpp
// Kommando til modul (0x080-0x087)
ID: 0x080 + modul_nummer
Data[0-1]: Target voltage (mV)
Data[2]: 0xFF (modul-specifik)
Data[3]: 0x5F (request flags)
Data[4]: 0x08 (balancing) eller 0x00 (ingen balancing)
Data[5]: 0x00
Data[6]: Message counter
Data[7]: CRC8 checksum
```

### Timing
- **BMS Kommandoer:** Videresendt med det samme
- **Balancing Kommandoer:** Sendt hver 50ms
- **Module Data:** Videresendt med det samme til BMS

### External BMS Detection
- ESP32 detekterer aktiv BMS ved at lytte efter kommandoer
- Hvis BMS er aktiv: `externalMasterDetected = true`
- Hvis ingen kommandoer i 60 sekunder: BMS anses offline

## ⚡ Balancing Logik

### Start Balancing
```
IF højeste celle > 3.9V
AND celle-difference > 10mV
AND mode = Auto/Manual
THEN
  - Bloker BMS kommandoer
  - Send egen kommando med target = laveste celle + 10mV
  - Send hver 50ms til alle 8 moduler
```

### Stop Balancing
```
IF celle-difference < 5mV
OR højeste celle < 3.9V
OR mode = Passive
THEN
  - Stop egen kommando
  - Tillad BMS kommandoer igen
```

## 🛡️ Sikkerhed

### Failsafe Mekanismer
1. **Passive Mode ved opstart:** Ingen automatisk balancing
2. **BMS svar altid sendt:** BMS ved altid module status
3. **Watchdog:** Hvis ESP32 crasher, går CAN busser fysisk idle
4. **LED indikatorer:** Visuelt feedback på status

### Debug Mode
- Slå CAN debug til/fra via web interface
- Logger alle beskeder på Serial
- Kan bruges til fejlfinding

## 📊 Web Interface

Se real-time:
- Cellevoltages fra alle moduler
- Balancing status
- Mode (Passive/Manual/Auto)
- BMS detection status

## 🔌 Hardware Opsætning

### Forbindelser
```
LilyGO T-2CAN:
  - MCP2515 CAN A (H/L) → BMW BMS CAN
  - GPIO 10 (CS) → MCP2515 CS
  - GPIO 8 (INT) → MCP2515 INT
  - SPI (GPIO 13 MISO / GPIO 11 MOSI / GPIO 12 SCK) → MCP2515 SPI
  - TWAI CAN B (GPIO 7 TX / GPIO 6 RX) → Slave Module CAN bus

MCP2515:
  - CAN H/L → BMS CAN bus
  - 8 MHz crystal
  - 500 kbps baud rate
```

## 📝 Logs

### Normal Gateway Operation
```
[TWAI->MCP2515] BMS Command: 0x080
  -> Forwarded to modules
[MCP2515 RX] Module Data: 0x120
  -> Forwarded to BMS via TWAI
```

### Balancing Active
```
[TWAI->MCP2515] BMS Command: 0x080
  -> BLOCKED (balancing active, using our own commands)
[BALANCING CMD -> MCP2515] ID: 0x080 Target: 3.950V
[MCP2515 RX] Module Data: 0x120
  -> Forwarded to BMS via TWAI
```

## 🚀 Fordele ved Dual CAN Arkitektur

1. **Isolation:** BMS og moduler er på separate busser
2. **Kontrol:** ESP32 kan intercepte og modificere beskeder
3. **Transparens:** BMS får altid modul svar
4. **Sikkerhed:** Kun balancing kommandoer modificeres
5. **Debugging:** Begge busser kan monitors uafhængigt

## ⚠️ Vigtige Noter

- **ALTID** send modul svar til BMS - selv under balancing
- **ALDRIG** bloker modul data - kun BMS kommandoer
- **HUSK** at BMS stadig skal vide modul status for at kunne arbejde korrekt
- **TEST** grundigt før brug på rigtig batteri pack!
