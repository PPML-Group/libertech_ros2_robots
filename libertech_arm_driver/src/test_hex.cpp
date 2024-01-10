#include <iostream>
#include <iomanip>

void swapEndianness(uint8_t (&bytes)[4]) {
    std::swap(bytes[0], bytes[3]);
    std::swap(bytes[1], bytes[2]);
}

void int32ToHexBytes(int32_t intValue, uint8_t (&resultBytes)[4]) {
    for (int i = 0; i < 4; ++i) {
        resultBytes[i] = (intValue >> (i * 8)) & 0xFF;
    }
}

void printHexBytes(const uint8_t (&bytes)[4]) {
    std::cout << "Hex representation: ";
    for (int i = 3; i >= 0; --i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::endl;
}

int main() {
    int32_t value = -19; // 你的 int32_t 值-92842
    uint8_t result[4];

    int32ToHexBytes(value, result);
    // printHexBytes(result);

    std::cout << "Before endian swap:" << std::endl;
    printHexBytes(result);

    swapEndianness(result);

    std::cout << "After endian swap:" << std::endl;
    printHexBytes(result);

    return 0;
}
