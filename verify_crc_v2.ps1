
# Script to verify CRC Final XOR values from a log file
param (
    [string]$LogFile = "c:\Users\stine\Desktop\BMW-i3-Balancing-Controller\temp_log.txt"
)

# 1. Initialize CRC8 Table (SAE J1850)
$crcTable = New-Object byte[] 256
for ($i = 0; $i -lt 256; $i++) {
    $crc = $i
    for ($j = 0; $j -lt 8; $j++) {
        if ($crc -band 0x80) {
            $crc = (($crc -shl 1) -bxor 0x1D) -band 0xFF
        } else {
            $crc = ($crc -shl 1) -band 0xFF
        }
    }
    $crcTable[$i] = $crc
}

# 2. Function to compute CRC with a specific final XOR
function Get-Crc($dataBytes, $finalXor) {
    $crc = 0xFF
    foreach ($byte in $dataBytes) {
        $idx = $crc -bxor $byte
        $crc = $crcTable[$idx]
    }
    return ($crc -bxor $finalXor) -band 0xFF
}

# 3. Function to find the Final XOR that satisfies the checksum
function Find-FinalXor($msgId, $payloadStr) {
    # Extract payload bytes
    $payload = $payloadStr -split ' ' | Where-Object { $_ -ne "" } | ForEach-Object { [byte]("0x" + $_) }
    
    if ($payload.Count -lt 2) { return -1 } # Need at least data + checksum

    # Last byte is the checksum
    $targetCrc = $payload[-1]
    
    # Data is ID bytes + payload (excluding checksum)
    $b1 = ($msgId -shr 8) -band 0xFF
    $b2 = $msgId -band 0xFF
    $data = @($b1, $b2) + $payload[0..($payload.Count - 2)]
    
    # Brute force Final XOR
    for ($fx = 0; $fx -lt 256; $fx++) {
        if ((Get-Crc $data $fx) -eq $targetCrc) {
            return $fx
        }
    }
    return -1
}

# 4. Parse Log File
Write-Host "Parsing log file: $LogFile"
if (-not (Test-Path $LogFile)) {
    Write-Error "Log file not found!"
    exit
}

$messageTypes = @{} # map ID -> list of FinalXor candidates

Get-Content $LogFile | Select-Object -First 5000 | ForEach-Object {
    if ($_ -match "\[MCP2515 RX\] (0x[0-9A-Fa-f]+) \[\d+\] ((?:[0-9A-Fa-f]{2} )+)") {
        $idStr = $matches[1]
        $dataStr = $matches[2].Trim()
        $id = [int]$idStr

        # Only process relevant IDs (0x100 - 0x17F)
        if ($id -ge 0x100 -and $id -le 0x17F) {
            $fx = Find-FinalXor $id $dataStr
            if ($fx -ne -1) {
               if (-not $messageTypes.ContainsKey($id)) {
                   $messageTypes[$id] = $fx
               } elseif ($messageTypes[$id] -ne $fx) {
                   Write-Warning "Conflict for ID $idStr! Found $fx but previously saw $($messageTypes[$id])"
               }
            }
        }
    }
}

# 5. Output Table
Write-Host "`n=== Final XOR Table Verification ==="
Write-Host "Row = Message Type (0x10X, 0x11X...)"
Write-Host "Col = Module ID (0-7)"
Write-Host ""
Write-Host "      mod0 mod1 mod2 mod3 mod4 mod5 mod6 mod7"

for ($type = 0; $type -lt 8; $type++) {
    $line = "Type$type "
    for ($mod = 0; $mod -lt 8; $mod++) {
        $id = 0x100 + ($type * 16) + $mod
        if ($messageTypes.ContainsKey($id)) {
            $line += "0x{0:X2} " -f $messageTypes[$id]
        } else {
            $line += ".... "
        }
    }
    Write-Host $line
}
