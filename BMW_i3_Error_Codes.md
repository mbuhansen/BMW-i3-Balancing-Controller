# BMW i3 Battery Module Error Codes (0x10X)

The `0x10X` message (where X is the module index 0-7) contains the module status and error flags in the first 4 bytes (Bytes 0-3).

## Error Bit Definitions (Based on BMSModuleManager.cpp)

The error code is received as a 32-bit field.
**Byte 0** corresponds to `Faults`.
**Byte 1** corresponds to `Alerts`.

### Byte 0: Faults
| Bit | Value (Hex) | Definition in BMSModuleManager |
|---|---|---|
| 0 | 0x01 | **Over Voltage** (Cell 1-6) |
| 1 | 0x02 | **Under Voltage** (Cell 1-6) |
| 2 | 0x04 | **CRC Error** (Packet reception) |
| 3 | 0x08 | **Power On Reset** |
| 4 | 0x10 | **Test Fault** |
| 5 | 0x20 | **Internal Registers Inconsistent** |
| 6 | 0x40 | *Use unknown / Reserved* |
| 7 | 0x80 | *Use unknown / Reserved* |

### Byte 1: Alerts (Shifted 8 bits in 32-bit error field)
| Bit | Value (Hex in Byte) | Value (Hex in 32-bit) | Definition in BMSModuleManager |
|---|---|---|---|
| 0 | 0x01 | 0x100 | **Over Temperature** on TS1 |
| 1 | 0x02 | 0x200 | **Over Temperature** on TS2 |
| 2 | 0x04 | 0x400 | **Sleep Mode** Active |
| 3 | 0x08 | 0x800 | **Thermal Shutdown** Active |
| 4 | 0x10 | 0x1000 | **Test Alert** |
| 5 | 0x20 | 0x2000 | **OTP EPROM Uncorrectable Error** (Critical) |
| 6 | 0x40 | 0x4000 | **GROUP3 Regs Invalid** |
| 7 | 0x80 | 0x8000 | **Address Not Registered** |

### Critical Error Check
The code checks specifically for `0x2000` (Bit 13) in `checkstatus()` as a critical fault indicator.

## References
- SimpBMS Project (BMSModuleManager.cpp)
