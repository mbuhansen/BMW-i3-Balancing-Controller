
const crcTable = new Uint8Array(256);
for (let i = 0; i < 256; i++) {
    let crc = i;
    for (let j = 0; j < 8; j++) {
        crc = (crc & 0x80) ? ((crc << 1) ^ 0x1D) : (crc << 1);
    }
    crcTable[i] = crc & 0xFF;
}

function getCrc(data, finalXor) {
    let crc = 0xFF;
    for (let i = 0; i < data.length; i++) {
        crc = crcTable[crc ^ data[i]];
    }
    return (crc ^ finalXor) & 0xFF;
}

const finalXorValues = [0xCF, 0xF5, 0xBB, 0x81, 0x27, 0x1D, 0x53, 0x69, 0x02, 0x38, 0x76, 0x4C];
const msgId = 0x123;
const finalXor = finalXorValues[msgId & 0x0F];
const prefix = [(msgId >> 8) & 0xFF, msgId & 0xFF];

const payloadOrig = [0xD9, 0x8F, 0xDB, 0x8F, 0xDB, 0x8F, 0x60];
const dataOrig = prefix.concat(payloadOrig);
const crcOrig = getCrc(dataOrig, finalXor);

const payloadMasked = [0xD9, 0x0F, 0xDB, 0x0F, 0xDB, 0x0F, 0x60];
const dataMasked = prefix.concat(payloadMasked);
const crcMasked = getCrc(dataMasked, finalXor);

console.log(`Original CRC: ${crcOrig.toString(16).toUpperCase()}`);
console.log(`Masked CRC: ${crcMasked.toString(16).toUpperCase()}`);
