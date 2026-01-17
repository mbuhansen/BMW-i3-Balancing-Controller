
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

# New Log Sample:
# [MCP2515 RX] 0x123 [8] D8 8F DB 8F DB 8F A0 88
# ID: 0x123
# Payload: D8 8F DB 8F DB 8F A0
# CRC: 88

$msgId = 0x123
$b1 = ($msgId -shr 8) -band 0xFF
$b2 = $msgId -band 0xFF
$prefix = @($b1, $b2)
$payload = @(0xD8, 0x8F, 0xDB, 0x8F, 0xDB, 0x8F, 0xA0)
$data = $prefix + $payload

Write-Output "Brute forcing Final XOR for ID 0x123 Data: D8 8F... CRC: 88"

for ($fx = 0; $fx -lt 256; $fx++) {
    $crc = Get-Crc $data $fx
    if ($crc -eq 0x88) {
        Write-Output "Match found! FinalXor should be: 0x$('{0:X}' -f $fx)"
    }
}
