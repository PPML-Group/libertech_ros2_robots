#include <iostream>
#include <iomanip>
#include <map>

void swapEndianness(uint8_t (&bytes)[4])
{
  std::swap(bytes[0], bytes[3]);
  std::swap(bytes[1], bytes[2]);
}

void int32ToHexBytes(int32_t intValue, uint8_t (&resultBytes)[4])
{
  for (int i = 0; i < 4; ++i)
  {
    resultBytes[i] = (intValue >> (i * 8)) & 0xFF;
  }
}

float convertHexBytesToFloat(const uint8_t (&bytes)[4])
{
  int32_t int_x = 0;
  for (int i = 0; i < 4; ++i)
  {
    int_x |= static_cast<int32_t>(bytes[i]) << (i * 8);
  }
  // return *reinterpret_cast<float*>(&int_x);
  // return (float)int_x;
  // std::cout << std::dec << "convertHexBytesToFloat int:" << static_cast<int>(int_x) << std::endl;
  return static_cast<float>(int_x);
}

void printHexBytes(const uint8_t (&bytes)[4])
{
  std::cout << "Hex representation: ";
  // for (int i = 3; i >= 0; --i) {

  //     std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(bytes[i]) << " ";
  // }
  for (int i = 0; i <= 3; ++i)
  {
    std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(bytes[i]) << " ";
  }
  std::cout << std::endl;
}

int main()
{
  int32_t value = 92842 ;  // 你的 int32_t 值-19 -92842
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

  std::map<int, std::array<uint8_t, 4>> id_to_u8_map;

  // 添加一些键值对
  id_to_u8_map[2] = { 0x01, 0x02, 0x03, 0x04 };
  id_to_u8_map[0] = { 0x11, 0x12, 0x13, 0x14 };
  id_to_u8_map[1] = { 0x21, 0x22, 0x23, 0x24 };

  // 遍历输出
  for (const auto& pair : id_to_u8_map)
  {
    std::cout << "Key: " << pair.first << ", Value: ";
    for (int i = 0; i < 4; ++i)
    {
      std::cout << std::hex << static_cast<int>(pair.second[i]) << " ";
    }
    std::cout << std::endl;
  }

  // 定义一个 std::array<uint8_t, 4>
  std::array<uint8_t, 4> arr = { 0x01, 0x02, 0x03, 0x04 };

  // 将 std::array 转换为 uint8_t 数组
  uint8_t u8[4];
  std::copy(arr.data(), arr.data() + arr.size(), u8);

  // 打印结果
  std::cout << "Converted array: ";
  for (int i = 0; i < 4; ++i)
  {
    std::cout << std::hex << static_cast<int>(u8[i]) << " ";
  }
  std::cout << std::endl;

  return 0;
}
