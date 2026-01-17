
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

function Find-FinalXor($msgId, $payloadStr, $targetCrcHex) {
    $b1 = ($msgId -shr 8) -band 0xFF
    $b2 = $msgId -band 0xFF
    $prefix = @($b1, $b2)
    
    $payload = $payloadStr -split ' ' | ForEach-Object { [byte]('0x' + $_) }
    $data = $prefix + $payload
    
    $targetCrc = [byte]('0x' + $targetCrcHex)
    
    for ($fx = 0; $fx -lt 256; $fx++) {
        $crc = Get-Crc $data $fx
        if ($crc -eq $targetCrc) {
            return $fx
        }
    }
    return -1
}

$samples = @(
    @{ id=0x120; data="A3 0F A1 0F A5 0F 80"; crc="93" },
    @{ id=0x121; data="96 0F 97 0F 98 0F 80"; crc="F7" },
    @{ id=0x122; data="BE 0F C6 0F C5 0F 80"; crc="03" },
    @{ id=0x123; data="9D 0F A2 0F 9D 0F 80"; crc="71" },
    @{ id=0x124; data="C2 0F CB 0F C3 0F 80"; crc="5E" },
    @{ id=0x125; data="B5 0F C1 0F C0 0F 80"; crc="92" },
    @{ id=0x126; data="A8 0F A5 0F A8 0F 80"; crc="D5" },
    @{ id=0x127; data="93 0F 93 0F 98 0F 80"; crc="35" },
    @{ id=0x100; data="00 00 00 00 00 00 50"; crc="81" },
    @{ id=0x101; data="00 00 00 00 00 00 50"; crc="E4" },
    @{ id=0x102; data="00 00 00 00 00 00 50"; crc="4B" },
    @{ id=0x103; data="00 00 00 00 00 00 50"; crc="2E" },
    @{ id=0x104; data="00 00 00 00 00 00 50"; crc="08" },
    @{ id=0x105; data="00 00 00 00 00 00 50"; crc="6D" },
    @{ id=0x106; data="00 00 00 00 00 00 50"; crc="C2" },
    @{ id=0x107; data="00 00 00 00 00 00 50"; crc="A7" },
    @{ id=0x130; data="AA 0F AC 0F AD 0F 80"; crc="A6" },
    @{ id=0x131; data="97 0F 98 0F 97 0F 80"; crc="4E" },
    @{ id=0x132; data="C3 0F CA 0F CB 0F 80"; crc="38" },
    @{ id=0x133; data="A3 0F A1 0F A0 0F 80"; crc="AB" },
    @{ id=0x134; data="C3 0F C0 0F C9 0F 80"; crc="DB" },
    @{ id=0x135; data="B7 0F B7 0F B9 0F 80"; crc="2C" },
    @{ id=0x136; data="A4 0F B3 0F A2 0F 80"; crc="F0" },
    @{ id=0x137; data="92 0F 94 0F 98 0F 80"; crc="7D" },

    @{ id=0x140; data="A2 0F A4 0F AD 0F 80"; crc="59" },
    @{ id=0x141; data="98 0F 9B 0F 9B 0F 80"; crc="FC" },
    @{ id=0x142; data="C8 0F C6 0F CC 0F 80"; crc="1F" },
    @{ id=0x143; data="A1 0F A5 0F A4 0F 80"; crc="F8" },
    @{ id=0x144; data="B7 0F CB 0F C9 0F 80"; crc="29" },
    @{ id=0x145; data="C3 0F C2 0F BF 0F 80"; crc="7D" },
    @{ id=0x146; data="AA 0F B7 0F B3 0F 80"; crc="89" },
    @{ id=0x147; data="92 0F 93 0F 92 0F 80"; crc="23" },

    @{ id=0x150; data="AB 0F B1 0F A9 0F 80"; crc="FF" },
    @{ id=0x151; data="9A 0F 98 0F 99 0F 80"; crc="E5" },
    @{ id=0x152; data="CB 0F CB 0F CD 0F 80"; crc="C1" },
    @{ id=0x153; data="A5 0F A4 0F AB 0F 80"; crc="63" },
    @{ id=0x154; data="CA 0F C7 0F CA 0F 80"; crc="97" },
    @{ id=0x155; data="BF 0F B7 0F C1 0F 80"; crc="FE" },
    @{ id=0x156; data="AA 0F A2 0F BA 0F 80"; crc="25" },
    @{ id=0x157; data="92 0F 92 0F A2 0F 80"; crc="7F" }
)

Write-Host "Finding ID-Included Final XOR values..."
foreach ($s in $samples) {
    $fx = Find-FinalXor $s.id $s.data $s.crc
    if ($fx -ne -1) {
        Write-Host ("ID 0x{0:X}: FinalXor = 0x{1:X}" -f $s.id, $fx)
    } else {
        Write-Host ("ID 0x{0:X}: No match found" -f $s.id)
    }
}
