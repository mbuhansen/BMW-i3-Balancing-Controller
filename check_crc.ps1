
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

$finalXorValues = @(0xCF, 0xF5, 0xBB, 0x81, 0x27, 0x1D, 0x53, 0x69, 0x02, 0x38, 0x76, 0x4C)
$msgId = 0x123
$finalXor = $finalXorValues[$msgId -band 0x0F]
$prefix = @((($msgId -shr 8) -band 0xFF), ($msgId -band 0xFF))

# Original
$payloadOrig = @(0xD9, 0x8F, 0xDB, 0x8F, 0xDB, 0x8F, 0x60)
$dataOrig = $prefix + $payloadOrig
$crcOrig = Get-Crc $dataOrig $finalXor

# Masked
$payloadMasked = @(0xD9, 0x0F, 0xDB, 0x0F, 0xDB, 0x0F, 0x60)
$dataMasked = $prefix + $payloadMasked
$crcMasked = Get-Crc $dataMasked $finalXor

Write-Host "Original CRC: $('{0:X}' -f $crcOrig)"
Write-Host "Masked CRC: $('{0:X}' -f $crcMasked)"
