
def calculate_crc():
    # CRC Table Generation
    crc_table = [0] * 256
    for i in range(256):
        crc = i
        for j in range(8):
            if (crc & 0x80):
                crc = ((crc << 1) ^ 0x1D) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        crc_table[i] = crc

    def get_crc(data_bytes, final_xor):
        crc = 0xFF
        for byte in data_bytes:
            crc = crc_table[crc ^ byte]
        return crc ^ final_xor

    final_xor_values = [0xCF, 0xF5, 0xBB, 0x81, 0x27, 0x1D, 0x53, 0x69, 0x02, 0x38, 0x76, 0x4C]
    
    # ID 0x123 -> msgId 3 -> final_xor index 3
    msg_id = 0x123
    final_xor = final_xor_values[msg_id & 0x0F] # 0x81
    
    # Prefix bytes: ID High, ID Low
    prefix = [(msg_id >> 8) & 0xFF, msg_id & 0xFF] # [0x01, 0x23]

    # Case 1: Original
    # D9 8F DB 8F DB 8F 60
    payload_orig = [0xD9, 0x8F, 0xDB, 0x8F, 0xDB, 0x8F, 0x60]
    full_data_orig = prefix + payload_orig
    crc_orig = get_crc(full_data_orig, final_xor)
    
    # Case 2: Masked
    # D9 0F DB 0F DB 0F 60
    payload_masked = [0xD9, 0x0F, 0xDB, 0x0F, 0xDB, 0x0F, 0x60]
    full_data_masked = prefix + payload_masked
    crc_masked = get_crc(full_data_masked, final_xor)

    print(f"Original CRC (Expected E0): {hex(crc_orig).upper()}")
    print(f"Masked CRC: {hex(crc_masked).upper()}")

if __name__ == "__main__":
    calculate_crc()
