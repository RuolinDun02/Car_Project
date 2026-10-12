#pragma once
#include <string>
namespace tanway {
enum ErrorCode {
  ERR_NONE = 0x00,                 /// NONE
  ERR_INIT_SOCKET = 0x01,          /// Init socket error!
  ERR_CREATE_SOCKET_POINT = 0x02,  /// Create point cloud socket error!
  ERR_CREATE_SOCKET_GPS = 0x03,    /// Create gps socket error!
  ERR_BIND_POINT = 0x04,           /// Bind port for point cloud socket error!
  ERR_BIND_GPS = 0x05,             /// Bind port for gps socket error!
  ERR_SETOPT_TIMEOUT_POINT = 0x06, /// Failed to set point cloud socket timeout!
  ERR_SETOPT_TIMEOUT_GPS = 0x07,   /// Failed to set gps socket timeout!
  ERR_SOCKET_RECV_POINT =
      0x08, /// The point cloud socket received failed and will exit!
  ERR_SOCKET_RECV_GPS = 0x09, /// The gps socket received failed and will exit!

  ERR_CREATE_SOCKET_DIF = 0x10,  /// Create dif socket error!
  ERR_BIND_DIF = 0x11,           /// Bind port for dif socket error!
  ERR_SETOPT_TIMEOUT_DIF = 0x12, /// Failed to set dif socket timeout!
  ERR_SOCKET_RECV_DIF = 0x13, /// The dif socket received failed and will exit!

  ERR_SELECT_SOCKET = 0x14,
  ERR_OPEN_PCAP_FAILED = 0x51,  /// Open pcap file failed!
  ERR_PCAP_FILE_INVALID = 0x52, /// The pcap file is invalid!
  ERR_TIME_WINDOW = 0x61,  ///The time window mode is disabled due to mismatch between launch parameters and web configuration
  ERR_UDP_COUNT = 0x62,   ///Timestamp failed to increment correctly

  ERR_SOCKET_RECV_IMU = 0x70, /// The imu socket received failed and will exit!
};

enum TipsCode {
  TIPS_NONE = 0x00,          /// NONE
  TIPS_TIMEOUT_POINT = 0x01, /// Receive point data time out!
  TIPS_TIMEOUT_GPS = 0x02,   /// Receive gps data time out!
  TIPS_EXIT_POINT = 0x03,  /// The point cloud data receiver thread has exited!
  TIPS_EXIT_GPS = 0x04,    /// The gps data receiver thread has exited!
  TIPS_EXIT_DECODE = 0x05, /// The decode package thread has exited!
  TIPS_TIMEOUT_DIF = 0x06, /// Receive dif data time out!
  TIPS_EXIT_DIF = 0x07,    /// The dif data receiver thread has exited!

  TIPS_TIMEOUT_SELECT = 0x08,

  TIPS_OPEN_PCAP_SUCCESS = 0x51, /// Open pcap file successed!
  TIPS_PCAP_EXIT = 0x52,         /// Exit reading the PCAP file!
  TIPS_REPEAT_PLAY = 0x53,       /// Repeat to read!
  TIPS_NOMATCH_DEVICE = 0x54,    /// Lidar type and protocol data do not match!
  TIPS_INVALID_DEVICE = 0x55,    /// Invalid device type!
};

class Exception {
public:
  Exception()
      : m_exceptionErrorCode(ERR_NONE), m_exceptionTipsCode(TIPS_NONE){};
  Exception(ErrorCode code, std::string msg)
      : m_exceptionErrorCode(code), m_msg(msg){};
  Exception(TipsCode code, std::string msg)
      : m_exceptionTipsCode(code), m_msg(msg){};
  ~Exception(){};

  std::string ToString() const { return m_msg; }
  ErrorCode GetErrorCode() const { return m_exceptionErrorCode; }
  TipsCode GetTipsCode() const { return m_exceptionTipsCode; }

private:
  ErrorCode m_exceptionErrorCode;
  TipsCode m_exceptionTipsCode;
  std::string m_msg;
};
} // namespace tanway
