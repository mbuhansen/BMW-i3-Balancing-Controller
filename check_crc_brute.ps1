
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

function Get-Crc($dataBytes, $finalXor) {
    $crc = 0xFF
    foreach ($byte in $dataBytes) {
        $idx = $crc -bxor $byte
        $crc = $crcTable[$idx]
    }
    return ($crc -bxor $finalXor) -band 0xFF
}

# Target: E0
# Input: 01 23 D9 8F DB 8F DB 8F 60
$msgId = 0x123
$b1 = ($msgId -shr 8) -band 0xFF
$b2 = $msgId -band 0xFF
$prefix = @($b1, $b2)
$payloadOrig = @(0xD9, 0x8F, 0xDB, 0x8F, 0xDB, 0x8F, 0x60)
$dataOrig = $prefix + $payloadOrig

Write-Host "Checking matches for E0..."

Write-Host "Calculated CRC with Code's Logic (FX 0x81): 0x$('{0:X}' -f (Get-Crc $dataOrig 0x81))"

$payloadMasked = @(0xD9, 0x0F, 0xDB, 0x0F, 0xDB, 0x0F, 0x60)
$dataMasked = $prefix + $payloadMasked
Write-Host "Masked CRC with Code's Logic (FX 0x81): 0x$('{0:X}' -f (Get-Crc $dataMasked 0x81))"
Write-Host "Masked CRC with Slave Logic (FX 0xE4): 0x$('{0:X}' -f (Get-Crc $dataMasked 0xE4))"

# Brute force Final XOR
for ($fx = 0; $fx -lt 256; $fx++) {
    $crc = Get-Crc $dataOrig $fx
    if ($crc -eq 0xE0) {
        Write-Host "Match found with FinalXor: 0x$('{0:X}' -f $fx)"
    }
}
