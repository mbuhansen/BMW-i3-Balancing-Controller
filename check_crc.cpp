#include <stdio.h>
#include <stdint.h>

const uint8_t finalxor[12] = {0xCF, 0xF5, 0xBB, 0x81, 0x27, 0x1D, 0x53, 0x69, 0x02, 0x38, 0x76, 0x4C};

class CRC8 {
private:
    uint8_t crc8_table[256];
public:
    CRC8() {
        for (int i = 0; i < 256; i++) {
            uint8_t crc = i;
            for (int j = 0; j < 8; j++) {
                crc = (crc & 0x80) ? ((crc << 1) ^ 0x1D) : (crc << 1);
            }
            crc8_table[i] = crc;
        }
    }

    uint8_t get_crc8(const uint8_t *data, uint8_t len, uint8_t finalXor) {
        uint8_t crc = 0xFF;
        for (uint8_t i = 0; i < len; i++) {
            crc = crc8_table[crc ^ data[i]];
        }
        return crc ^ finalXor;
    }
};

int main() {
    CRC8 crc8;
    
    // Original: 0x123 [8] D9 8F DB 8F DB 8F 60 E0
    uint8_t data_orig[] = {0x01, 0x23, 0xD9, 0x8F, 0xDB, 0x8F, 0xDB, 0x8F, 0x60};
    uint8_t crc_orig = crc8.get_crc8(data_orig, 9, finalxor[3]);
    
    // Masked: 0x123 [8] D9 0F DB 0F DB 0F 60 ?
    uint8_t data_masked[] = {0x01, 0x23, 0xD9, 0x0F, 0xDB, 0x0F, 0xDB, 0x0F, 0x60};
    uint8_t crc_masked = crc8.get_crc8(data_masked, 9, finalxor[3]);
    
    printf("Original CRC: %02X\n", crc_orig);
    printf("Masked   CRC: %02X\n", crc_masked);
    
    return 0;
}
