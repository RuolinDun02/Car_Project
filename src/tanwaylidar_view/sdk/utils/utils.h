#pragma once
#include <memory>
#include <regex>
namespace tanway {

inline bool isValidIp(const std::string &ip) {
  std::regex pattern("^(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)"
                     "\\."
                     "(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)"
                     "\\."
                     "(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)"
                     "\\."
                     "(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$");

  return regex_match(ip, pattern);
}

inline float FourHexToFloat(unsigned char high, unsigned char highmiddle,
                            unsigned char middle, unsigned char low) {
  unsigned char sz_float[4] = {low, middle, highmiddle, high};
  float addr = 0;
  memcpy(&addr, sz_float, 4);
  return addr;
}

inline int TwoHextoInt(unsigned char high, unsigned char low) {
  int addr = low & 0xFF;
  addr |= ((high << 8) & 0XFF00);
  return addr;
}

// 处理有符号两字节数据的函数
inline int SignedTwoHextoInt(unsigned char high, unsigned char low) {
  // 先组合成16位无符号整数
  int addr = low & 0xFF;
  addr |= ((high << 8) & 0xFF00);
  
  // 检查最高位(第15位)是否为1，如果是则进行符号扩展
  if (addr & 0x8000) {
    // 符号位为1，说明是负数，需要符号扩展到32位int
    addr |= 0xFFFF0000;
  }
  
  return addr;
}

inline unsigned int FourHexToInt(unsigned char high, unsigned char highmiddle,
                                 unsigned char middle, unsigned char low) {
  unsigned int addr = low & 0xFF;
  addr |= ((middle << 8) & 0xFF00);
  addr |= ((highmiddle << 16) & 0xFF0000);
  addr |= ((high << 24) & 0xFF000000);
  return addr;
}

inline unsigned short OneHexHalfHextoInt(unsigned char high,
                                         unsigned char halflow) {
  unsigned short addr = halflow & 0xF0;
  addr = addr >> 4;
  addr |= ((high << 4) & 0x0FF0);
  return addr;
}

} // namespace tanway
