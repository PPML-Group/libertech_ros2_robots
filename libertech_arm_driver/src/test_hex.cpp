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

float convertHexBytesToFloat(const uint8_t (&bytes)[4]) {
    int32_t int_x = 0;
    for (int i = 0; i < 4; ++i) {
        int_x |= static_cast<int32_t>(bytes[i]) << (i * 8);
    }
    // return *reinterpret_cast<float*>(&int_x);
    // return (float)int_x;
    return static_cast<float>(int_x);
}


void printHexBytes(const uint8_t (&bytes)[4]) {
    std::cout << "Hex representation: ";
    // for (int i = 3; i >= 0; --i) {
        
    //     std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(bytes[i]) << " ";
    // }
    for (int i = 0; i <= 3; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::endl;
}

int main() {
    int32_t value = 92842; // 你的 int32_t 值-19 -92842
    uint8_t result[4];

    std::cout << "int:" << value << std::endl;

    int32ToHexBytes(value, result);

    std::cout << "Before endian swap:" << std::endl;
    printHexBytes(result);


    float value_float = convertHexBytesToFloat(result);
    std::cout << "float:" << value_float << std::endl;

    swapEndianness(result);

    std::cout << "After endian swap:" << std::endl;
    printHexBytes(result);

    value_float = convertHexBytesToFloat(result);
    std::cout << "float:" << value_float << std::endl;

    return 0;
}
