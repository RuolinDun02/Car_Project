#pragma once
#include "ILidarAlgo.h"
#include "ILidarDevice.h"
#include "PackageList.h"
#include "about.h"
#include "twlog.h"
#include "utils.h"
#include <atomic>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <math.h>
#include <thread>
#include <unordered_map>
#ifdef __linux__
#include <dirent.h>

#endif

namespace tanway {
#if defined(__GNUC__) || defined(__clang__)
#define PRETTY_FUNCTION __PRETTY_FUNCTION__
#elif defined(_MSC_VER)
#define PRETTY_FUNCTION __FUNCSIG__
#endif

#define PRE_LOAD_PCAP_FILE_CALCULATE                                           \
  if (maxAngle != nullptr) {                                                   \
    if (horizontalAngle > *maxAngle)                                           \
      *maxAngle = horizontalAngle;                                             \
                                                                               \
    TWPoint basic_point;                                                       \
    basic_point.angle = horizontalAngle;                                       \
    basic_point.mirror = mirror;                                               \
    basic_point.distance = 1;                                                  \
    basic_point.t_sec = blockSecond;                                           \
    basic_point.t_usec = blockMicrosecond;                                     \
    points.push_back(basic_point);                                             \
                                                                               \
    continue;                                                                  \
  }

#define PRE_LOAD_PCAP_FILE_CALCULATE_EX                                        \
  if (maxAngle != nullptr) {                                                   \
    USEDECODE_COMMON_RETURN                                                    \
  }

#define USEDECODE_COMMON_RETURN                                                \
  {                                                                            \
    if (TwoHextoInt(udpData[6], udpData[7]) ==                                 \
        TwoHextoInt(udpData[21], udpData[22])) {                               \
      _has_max_count = true;                                                   \
      return true;                                                             \
    }                                                                          \
                                                                               \
    if (_period_count != TwoHextoInt(udpData[4], udpData[5]) &&                \
        _cur_udp_count > TwoHextoInt(udpData[6], udpData[7])) {                \
      _cur_udp_count = TwoHextoInt(udpData[6], udpData[7]);                    \
      _period_count = TwoHextoInt(udpData[4], udpData[5]);                     \
      if (!_has_max_count) {                                                   \
        return true;                                                           \
      }                                                                        \
      _has_max_count = false;                                                  \
    }                                                                          \
                                                                               \
    _cur_udp_count = TwoHextoInt(udpData[6], udpData[7]);                      \
    _period_count = TwoHextoInt(udpData[4], udpData[5]);                       \
                                                                               \
    return false;                                                              \
  }

#define ON_DECODE_EXCEPTION                                                    \
  {                                                                            \
    if (exception != NULL)                                                     \
      *exception = true;                                                       \
                                                                               \
    _lidarObserver->OnException(                                               \
        _lidarInfo,                                                            \
        Exception(TIPS_NOMATCH_DEVICE, "decode lidar data failed!"));          \
  }

enum {
  INPUT_SOCKET_POINT_CLOUD = 0,
  INPUT_SOCKET_DIF,
  INPUT_SOCKET_IMU,
  N_INPUT_SOCKETS
};

enum LD_STATUS {
  LD_STOPED = 0,
  LD_STARTED,
  LD_PARSING,
};

class LidarDevice : public ILidarDevice {
public:
  virtual bool Start(bool play);
  virtual void SetPlayRate(float rate);
  virtual void Stop();
  virtual void Play(bool play);
  virtual void SeekFrame(int index);
  virtual void UpdateFrame();
  virtual bool GetPlayMode();

  virtual void SetEchoNum(EchoNum echoNum);
  virtual void SetFrameID(const std::string &frameID);

  virtual void SetMirrorVerAngleOffset(float a, float b, float c);
  virtual void SetMirrorHorAngleOffset(float a, float b, float c);

  virtual void SetCoreVerAngleOffset(float l, float r);
  virtual void SetCoreHorAngleOffset(float l, float r);

  virtual void SetXYZRotateAngle(float x, float y, float z);
  virtual void SetXYZTransVec(float x, float y, float z);
  virtual void SetXForwardFlag();

  virtual void SetDistanceRange(double min, double max);
  virtual void SetAngleRange(double min, double max);

  virtual bool StartParsePcap();
  virtual void StopParsePcap();

  virtual int GetPcapFrameNum();
  virtual void SetLidarAlgo(ILidarAlgo *lidarAlgo);

  virtual void SetTimeStampType(const std::string &timestamp);
  virtual void SetLidarTime(bool lidartime);
  virtual void SetFrameSplit(bool frameSplit);
  virtual void SetTimeWindowMode(bool useTimeWindow);

private:
  LidarDevice(const std::string &lidarIPOrPcapPath,
              const std::string &hostIPOrLidarIPForFilter, int pointcloudPort,
              int DIFPort, ILidarObserver *lidarObserver, LidarType lidarType,
              bool repeat, int lidarID, int localIMUPort)
      : _lidarIPOrPcapPath(lidarIPOrPcapPath),
        _hostIPOrLidarIPForFilter(hostIPOrLidarIPForFilter),
        _pointcloudPort(pointcloudPort), _DIFPort(DIFPort),
        _IMUPort(localIMUPort), _lidarObserver(lidarObserver),
        _lidarType(lidarType), _repeat(repeat) {
    _frameIndex.store(0);

    _run.store(false);
    _play.store(false);
    _parsing.store(false);
    _seeking.store(false);
    _parsed.store(false);

    _lidarInfo.lidarID = lidarID;

    _lidarInfo.lidarIPOrPcapPath = lidarIPOrPcapPath;
    _lidarInfo.hostIPOrLidarIPForFilter = hostIPOrLidarIPForFilter;
    _lidarInfo.pointcloudPort = pointcloudPort;
    _lidarInfo.DIFPort = DIFPort;
    _lidarInfo.IMUPort = localIMUPort;
    _lidarInfo.lidarType = lidarType;
    _lidarInfo.online = isValidIp(_lidarIPOrPcapPath);

    InitBasicVariables();
  }

  LidarDevice(const LidarDevice &other){};
  virtual ~LidarDevice();

  void ReadFromNetworkThread();
  void ReadFromLocalThread();
  void DecodeThread();
  bool CreateAndBindSocket();
  void ParsePcapThread();
  bool ReadUdpPacket(std::ifstream &inStream, UDPPackage::Ptr &udpData,
                     bool fast = true);
  bool DecodeImp(const UDPPackage::Ptr &udpData, float *maxAngle = nullptr,
                 bool *exception = nullptr);

  bool PointCloudCallback(TWPointCloud::Points &points, float *maxAngle,
                          int mirror = -1, int leftRight = -1, int value = -1);
  void PointCloudCallback(TWPointCloud::Points &points, bool callback,
                          uint64_t frameIndex);

  void PointCloudCallback(bool callback, uint64_t frameIndex);

  bool SeekNextValidPacketOffset(std::ifstream &inStream);

  bool IsValidFrame(const char frame[], int length);

  bool IsBackPoint(const TWPoint &point);

private:
  void InitBasicVariables();
  bool DecodeTensor16(const char *udpData, unsigned int *t_sec,
                      unsigned int *t_usec, float *maxAngle,
                      uint64_t frameIndex = 0, bool framed = false);

  bool DecodeTensor32(const char *udpData, unsigned int *t_sec,
                      unsigned int *t_usec, float *maxAngle,
                      uint64_t frameIndex = 0, bool framed = false);

  bool DecodeScope192(const char *udpData, unsigned int *t_sec,
                      unsigned int *t_usec, float *maxAngle,
                      uint64_t frameIndex = 0, bool framed = false);

  bool DecodeDuetto(const char *udpData, unsigned int *t_sec,
                    unsigned int *t_usec, float *maxAngle,
                    uint64_t frameIndex = 0, bool framed = false);

  bool DecodeTempoA1(const char *udpData, unsigned int *t_sec,
                     unsigned int *t_usec, float *maxAngle,
                     uint64_t frameIndex = 0, bool framed = false);
  bool DecodeTempoA2(const char *udpData, unsigned int *t_sec,
                     unsigned int *t_usec, float *maxAngle,
                     uint64_t frameIndex = 0, bool framed = false);
  bool DecodeTempoA3(const char *udpData, unsigned int *t_sec,
                     unsigned int *t_usec, float *maxAngle,
                     uint64_t frameIndex = 0, bool framed = false);
  bool DecodeTempoA4(const char *udpData, unsigned int *t_sec,
                     unsigned int *t_usec, float *maxAngle,
                     uint64_t frameIndex = 0, bool framed = false);
  bool DecodeTempoA4Calib(const char *udpData, unsigned int *t_sec,
                          unsigned int *t_usec, float *maxAngle,
                          uint64_t frameIndex = 0, bool framed = false);

  bool DecodeTensor48(const char *udpData, unsigned int *t_sec,
                      unsigned int *t_usec, float *maxAngle,
                      uint64_t frameIndex = 0, bool framed = false);
  bool DecodeTensor48Calib(const char *udpData, unsigned int *t_sec,
                           unsigned int *t_usec, float *maxAngle,
                           uint64_t frameIndex = 0, bool framed = false);
  bool DecodeTensor48Depth(const char *udpData, unsigned int *t_sec,
                           unsigned int *t_usec, float *maxAngle,
                           uint64_t frameIndex = 0, bool framed = false);
  bool DecodeScope256(const char *udpData, unsigned int *t_sec,
                      unsigned int *t_usec, float *maxAngle,
                      uint64_t frameIndex = 0, bool framed = false);
  bool DecodeScope256Depth(const char *udpData, unsigned int *t_sec,
                           unsigned int *t_usec, float *maxAngle,
                           uint64_t frameIndex = 0, bool framed = false);

  bool DecodeFocusB1(const char *udpData, unsigned int *t_sec,
                     unsigned int *t_usec, float *maxAngle,
                     uint64_t frameIndex = 0, bool framed = false);

  bool DecodeFocusB2(const char *udpData, unsigned int *t_sec,
                     unsigned int *t_usec, float *maxAngle,
                     uint64_t frameIndex = 0, bool framed = false);
  bool DecodeFocusB2_64(const char *udpData, unsigned int *t_sec,
                        unsigned int *t_usec, float *maxAngle,
                        uint64_t frameIndex = 0, bool framed = false);

  bool DecodeFocusB2Calib(const char *udpData, unsigned int *t_sec,
                          unsigned int *t_usec, float *maxAngle,
                          uint64_t frameIndex = 0, bool framed = false);
  bool DecodeFocusT(const char *udpData, unsigned int *t_sec,
                    unsigned int *t_usec, float *maxAngle,
                    uint64_t frameIndex = 0, bool framed = false);
  bool DecodeFocusT_2(const char *udpData, unsigned int *t_sec,
                      unsigned int *t_usec, float *maxAngle,
                      uint64_t frameIndex = 0, bool framed = false);
  bool DecodeFocusTCalib(const char *udpData, unsigned int *t_sec,
                         unsigned int *t_usec, float *maxAngle,
                         uint64_t frameIndex = 0, bool framed = false);

  bool DecodeScope128(const char *udpData, unsigned int *t_sec,
                      unsigned int *t_usec, float *maxAngle,
                      uint64_t frameIndex = 0, bool framed = false);
  bool DecodeScope128_2(const char *udpData, unsigned int *t_sec,
                        unsigned int *t_usec, float *maxAngle,
                        uint64_t frameIndex = 0, bool framed = false);
  bool DecodeScope128Calib(const char *udpData, unsigned int *t_sec,
                           unsigned int *t_usec, float *maxAngle,
                           uint64_t frameIndex = 0, bool framed = false);

  bool DecodeScope128F(const char *udpData, unsigned int *t_sec,
                       unsigned int *t_usec, float *maxAngle,
                       uint64_t frameIndex = 0, bool framed = false);
  bool DecodeScope128FCalib(const char *udpData, unsigned int *t_sec,
                            unsigned int *t_usec, float *maxAngle,
                            uint64_t frameIndex = 0, bool framed = false);
  bool DecodeTW360(const char *udpData, unsigned int *t_sec,
                   unsigned int *t_usec, float *maxAngle,
                   uint64_t frameIndex = 0, bool framed = false);
  bool DecodeTW360_2(const char *udpData, unsigned int *t_sec,
                     unsigned int *t_usec, float *maxAngle,
                     uint64_t frameIndex = 0, bool framed = false);
  bool DecodeTW360Calib(const char *udpData, unsigned int *t_sec,
                        unsigned int *t_usec, float *maxAngle,
                        uint64_t frameIndex = 0, bool framed = false);

  void DecodeDIFData_Duetto(const char *udpData);
  void DecodeIMUData(const char *udpData);
  void DecodeIMUData_TW360(const char *udpData, unsigned int *t_sec,
                           unsigned int *t_usec); // 解析TW360的IMU数据
  void DecodeDIFData_Tensor48(const char *udpData);
  void DecodeDIFData_Scope256(const char *udpData);
  void DecodeDIFData_FocusB1(const char *udpData);
  void DecodeDIFData_FocusB2(const char *udpData);
  void DecodeDIFData_FocusT(const char *udpData);
  void DecodeDIFData_Scope128(const char *udpData);

  void DecodeDIFData_TempoA4(const char *udpData);
  void DecodeDIFData_TW360(const char *udpData);
  void SetDuettoVerticalAngleType(int type, double offsetVerAngleL,
                                  double offsetVerAngleR);

  void DecodeDIFImp(const char *udpData, DeviceInfoFrame &deviceInfoFrame);

private:
  virtual void UseDecodeTensor16(const char *udpData,
                                 TWPointCloud::Points &points,
                                 unsigned int *t_sec, unsigned int *t_usec,
                                 float *maxAngle);
  virtual void UseDecodeTensor32(const char *udpData,
                                 TWPointCloud::Points &points,
                                 unsigned int *t_sec, unsigned int *t_usec,
                                 float *maxAngle);
  virtual void UseDecodeScope192(const char *udpData,
                                 TWPointCloud::Points &points,
                                 unsigned int *t_sec, unsigned int *t_usec,
                                 float *maxAngle);
  virtual void UseDecodeDuetto(const char *udpData,
                               TWPointCloud::Points &points,
                               unsigned int *t_sec, unsigned int *t_usec,
                               float *maxAngle);
  virtual void UseDecodeTempoA1(const char *udpData,
                                TWPointCloud::Points &points,
                                unsigned int *t_sec, unsigned int *t_usec,
                                float *maxAngle);
  virtual void UseDecodeTempoA2(const char *udpData,
                                TWPointCloud::Points &points,
                                unsigned int *t_sec, unsigned int *t_usec,
                                float *maxAngle);
  virtual void UseDecodeTempoA3(const char *udpData,
                                TWPointCloud::Points &points,
                                unsigned int *t_sec, unsigned int *t_usec,
                                float *maxAngle);
  virtual void UseDecodeTempoA4(const char *udpData,
                                TWPointCloud::Points &points,
                                unsigned int *t_sec, unsigned int *t_usec,
                                float *maxAngle);
  virtual void UseDecodeTempoA4Calib(const char *udpData,
                                     TWPointCloud::Points &points,
                                     unsigned int *t_sec, unsigned int *t_usec,
                                     float *maxAngle);
  virtual void UseDecodeTensor48(const char *udpData,
                                 TWPointCloud::Points &points,
                                 unsigned int *t_sec, unsigned int *t_usec,
                                 float *maxAngle);
  virtual void UseDecodeTensor48_2(const char *udpData,
                                   TWPointCloud::Points &points,
                                   unsigned int *t_sec, unsigned int *t_usec,
                                   float *maxAngle);
  virtual void UseDecodeTensor48Calib(const char *udpData,
                                      TWPointCloud::Points &points,
                                      unsigned int *t_sec, unsigned int *t_usec,
                                      float *maxAngle);
  virtual void UseDecodeTensor48Depth(const char *udpData,
                                      TWPointCloud::Points &points,
                                      unsigned int *t_sec, unsigned int *t_usec,
                                      float *maxAngle);
  virtual bool UseDecodeScope256(const char *udpData,
                                 TWPointCloud::Points &points,
                                 unsigned int *t_sec, unsigned int *t_usec,
                                 float *maxAngle);
  virtual void WallThicknessScope256(char *udpData);
  virtual void UseDecodeScope256Depth(const char *udpData,
                                      TWPointCloud::Points &points,
                                      unsigned int *t_sec, unsigned int *t_usec,
                                      float *maxAngle);

  virtual void UseDecodeFocusB1(const char *udpData,
                                TWPointCloud::Points &points,
                                unsigned int *t_sec, unsigned int *t_usec,
                                float *maxAngle);

  virtual void UseDecodeFocusB2(const char *udpData,
                                TWPointCloud::Points &points,
                                unsigned int *t_sec, unsigned int *t_usec,
                                float *maxAngle);

  virtual void UseDecodeFocusB2Calib(const char *udpData,
                                     TWPointCloud::Points &points,
                                     unsigned int *t_sec, unsigned int *t_usec,
                                     float *maxAngle);
  virtual bool UseDecodeFocusT(const char *udpData,
                               TWPointCloud::Points &points,
                               unsigned int *t_sec, unsigned int *t_usec,
                               float *maxAngle);
  virtual bool UseDecodeFocusT_2(const char *udpData,
                                 TWPointCloud::Points &points,
                                 unsigned int *t_sec, unsigned int *t_usec,
                                 float *maxAngle);

  virtual bool UseDecodeFocusTCalib(const char *udpData,
                                    TWPointCloud::Points &points,
                                    unsigned int *t_sec, unsigned int *t_usec,
                                    float *maxAngle);

  virtual bool UseDecodeScope128(const char *udpData,
                                 TWPointCloud::Points &points,
                                 unsigned int *t_sec, unsigned int *t_usec,
                                 float *maxAngle);
  virtual bool UseDecodeScope128_2(const char *udpData,
                                   TWPointCloud::Points &points,
                                   unsigned int *t_sec, unsigned int *t_usec,
                                   float *maxAngle);
  virtual bool UseDecodeScope128Calib(const char *udpData,
                                      TWPointCloud::Points &points,
                                      unsigned int *t_sec, unsigned int *t_usec,
                                      float *maxAngle);

  virtual bool UseDecodeScope128F(const char *udpData,
                                  TWPointCloud::Points &points,
                                  unsigned int *t_sec, unsigned int *t_usec,
                                  float *maxAngle);
  virtual bool UseDecodeScope128FCalib(const char *udpData,
                                       TWPointCloud::Points &points,
                                       unsigned int *t_sec,
                                       unsigned int *t_usec, float *maxAngle);
  virtual bool UseDecodeTW360(const char *udpData, TWPointCloud::Points &points,
                              unsigned int *t_sec, unsigned int *t_usec,
                              float *maxAngle);
  virtual bool UseDecodeTW360_2(const char *udpData,
                                TWPointCloud::Points &points,
                                unsigned int *t_sec, unsigned int *t_usec,
                                float *maxAngle);

  virtual bool UseDecodeTW360Calib(const char *udpData,
                                   TWPointCloud::Points &points,
                                   unsigned int *t_sec, unsigned int *t_usec,
                                   float *maxAngle);

private:
  inline int pulseToIntensity(double pulse);
  inline int GetDuettoBlockNumber(double angle, int mirror, int lORr);
  inline bool IsEqualityFloat3(const double value1, const double value2);
  inline bool IsEqualityFloat5(const double value1, const double value2);
  inline void CalculateRotateAllPointCloud(TWPoint &point);
  inline void JointabcProcess(TWPoint &point);
  inline void CalNowTime(UDPPackage::Ptr &udpPackage) {
    //------------------------------------------------------重新在此计算UDP包接收的时间--------------------------------------------
#ifdef __linux__
    timeval start;              //
    gettimeofday(&start, NULL); // 现在系统时间
    udpPackage->t_sec = start.tv_sec;
    udpPackage->t_usec = start.tv_usec;
    // 输出时间信息观察是否正确：
    // std::cout << "t_sec:" << udpData->t_sec << "\t t_usec:" <<
    // udpData->t_usec
    // << std::endl;

#elif _WIN32
    time_t clock;
    struct tm tm;
    SYSTEMTIME wtm;
    GetLocalTime(&wtm);
    tm.tm_year = wtm.wYear - 1900;
    tm.tm_mon = wtm.wMonth - 1;
    tm.tm_mday = wtm.wDay;
    tm.tm_hour = wtm.wHour;
    tm.tm_min = wtm.wMinute;
    tm.tm_sec = wtm.wSecond;
    tm.tm_isdst = -1;
    clock = mktime(&tm);
    udpPackage->t_sec = (long)clock;
    udpPackage->t_usec = wtm.wMilliseconds * 1000;
#endif
    //-----------------------------------------------------------------------------------------------------------
  };
  TWLOG &logger = TWLOG::GetInstance();
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time;

  double m_startAngle = 30.0; // 30.0;  0.1
  double m_endAngle = 150.0;  // 150.0;  359.9

  double m_positive_back_startAngle = 0; // 正机芯背面点起始角度
  double m_positive_back_endAngle = 0;   // 正机芯背面点结束角度

  double m_negtive_back_startAngle = 0; // 负机芯背面点起始角度
  double m_negtive_back_endAngle = 0;   // 负机芯背面点结束角度

protected:
  double m_firstSeparateAngle = -1;
  double m_calRA = (double)(3.14159265f / 180.0f);
  double m_calPulse = 0.0305176;
  double m_calSimple = 0.004574; // 500 * 2.99792458 / 10.f / 16384.f / 2;

  double m_calSimpleFPGA = 0.004796; // 0.032 * 2.997924 / 10.f / 2;
  double m_calPulseFPGA = 0.032;

  double m_calSimpleFPGA_TW360 = 0.009368;
  double m_calGToMs2 = 9.80665; // 计算imu线性加速度时，转换为m/s²单位

  int m_blockNumberForDuetto[6] = {1 /*RA*/,    2001 /*RB*/, 4001 /*RC*/,
                                   3001 /*LA*/, 5001 /*LB*/, 1001 /*LC*/};

  // transform
  double m_transformRotateX = 0;
  double m_transformRotateY = 0;
  double m_transformRotateZ = 0;
  double m_transformMoveX = 0;
  double m_transformMoveY = 0;
  double m_transformMoveZ = 0;

  double m_rotationMatrix[3][3] = {
      {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
  bool m_x_forward_flag = false;

  // Tensor
  double m_verticalChannelsAngle_Tensor16[16] = {
      -5.274283f, -4.574258f, -3.872861f, -3.1703f,  -2.466783f, -1.762521f,
      -1.057726f, -0.352611f, 0.352611f,  1.057726f, 1.762521f,  2.466783f,
      3.1703f,    3.872861f,  4.574258f,  5.274283f};

  // TSP03 32
  double m_verticalChannelAngle16_cos_vA_RA[16] = {0.0};
  double m_verticalChannelAngle16_sin_vA_RA[16] = {0.0};
  double m_skewing_tsp_Angle[3] = {0.0, -6.0};
  double m_skewing_tsp_Angle_Correct[3] = {0.0};
  double m_skewing_sin_tsp[2] = {0.0};
  double m_skewing_cos_tsp[2] = {0.0};

  // Scope
  double m_verticalChannelAngle_Scope64[64] = {
      -14.64f, -14.17f, -13.69f, -13.22f, -12.75f, -12.28f, -11.81f, -11.34f,
      -10.87f, -10.40f, -9.93f,  -9.47f,  -9.00f,  -8.54f,  -8.07f,  -7.61f,
      -7.14f,  -6.68f,  -6.22f,  -5.76f,  -5.29f,  -4.83f,  -4.37f,  -3.91f,
      -3.45f,  -2.99f,  -2.53f,  -2.07f,  -1.61f,  -1.15f,  -0.69f,  -0.23f,
      0.23f,   0.69f,   1.15f,   1.61f,   2.07f,   2.53f,   2.99f,   3.45f,
      3.91f,   4.37f,   4.83f,   5.29f,   5.76f,   6.22f,   6.68f,   7.14f,
      7.61f,   8.07f,   8.54f,   9.00f,   9.47f,   9.93f,   10.40f,  10.87f,
      11.34f,  11.81f,  12.28f,  12.75f,  13.22f,  13.69f,  14.17f,  14.64f};
  double m_verticalChannelAngle_Scope64_cos_vA_RA[64] = {0.0};
  double m_verticalChannelAngle_Scope64_sin_vA_RA[64] = {0.0};
  double m_skewing_scope_Angle[3] = {-0.0, -0.12, -0.24};
  double m_skewing_scope_Angle_Correct[3] = {-0.0, -0.12, -0.24};
  double m_skewing_sin_scope[3] = {0.0};
  double m_skewing_cos_scope[3] = {0.0};
  double m_rotate_scope_sin = sin(-10.0 * m_calRA);
  double m_rotate_scope_cos = cos(-10.0 * m_calRA);

  // ScopeMiniA2-192
  float m_verticalChannelAngle_Scope64_A2[64] = {
      -12.368f, -11.986f, -11.603f, -11.219f, -10.834f, -10.448f, -10.061f,
      -9.674f,  -9.285f,  -8.896f,  -8.505f,  -8.115f,  -7.723f,  -7.331f,
      -6.938f,  -6.545f,  -6.151f,  -5.756f,  -5.361f,  -4.966f,  -4.570f,
      -4.174f,  -3.777f,  -3.381f,  -2.983f,  -2.586f,  -2.189f,  -1.791f,
      -1.393f,  -0.995f,  -0.597f,  -0.199f,  0.199f,   0.597f,   0.995f,
      1.393f,   1.791f,   2.189f,   2.586f,   2.983f,   3.381f,   3.777f,
      4.174f,   4.570f,   4.966f,   5.361f,   5.756f,   6.151f,   6.545f,
      6.938f,   7.331f,   7.723f,   8.115f,   8.505f,   8.896f,   9.285f,
      9.674f,   10.061f,  10.448f,  10.834f,  11.219f,  11.603f,  11.986f,
      12.368f};
  double m_verticalChannelAngle_ScopeMiniA2_cos_vA_RA[64] = {0.f};
  double m_verticalChannelAngle_ScopeMiniA2_sin_vA_RA[64] = {0.f};
  double m_skewing_scopeMiniA2_Angle[3] = {0.0, 0.1, 0.2};
  double m_skewing_scopeMiniA2_Angle_Correct[3] = {0.0, 0.1, 0.2};
  double m_skewing_sin_scopeMiniA2_192[3] = {0.0};
  double m_skewing_cos_scopeMiniA2_192[3] = {0.0};

  // Focus
  float m_verticalChannelAngle_Focus[64] = {
      -12.368f, -11.986f, -11.603f, -11.219f, -10.834f, -10.448f, -10.061f,
      -9.674f,  -9.285f,  -8.896f,  -8.505f,  -8.115f,  -7.723f,  -7.331f,
      -6.938f,  -6.545f,  -6.151f,  -5.756f,  -5.361f,  -4.966f,  -4.570f,
      -4.174f,  -3.777f,  -3.381f,  -2.983f,  -2.586f,  -2.189f,  -1.791f,
      -1.393f,  -0.995f,  -0.597f,  -0.199f,  0.199f,   0.597f,   0.995f,
      1.393f,   1.791f,   2.189f,   2.586f,   2.983f,   3.381f,   3.777f,
      4.174f,   4.570f,   4.966f,   5.361f,   5.756f,   6.151f,   6.545f,
      6.938f,   7.331f,   7.723f,   8.115f,   8.505f,   8.896f,   9.285f,
      9.674f,   10.061f,  10.448f,  10.834f,  11.219f,  11.603f,  11.986f,
      12.368f};
  double m_verticalChannelAngle_Focus_cos_vA_RA[64] = {0.f};
  double m_verticalChannelAngle_Focus_sin_vA_RA[64] = {0.f};
  double m_skewing_Focus_Angle_Offset[3] = {0.3, 0.1, 0.2};
  double m_skewing_Focus_Angle_Correct[3] = {0.0, 0.2, 0.1};
  double m_skewing_sin_Focus[3] = {0.0};
  double m_skewing_cos_Focus[3] = {0.0};

  // Duetto
  float m_verticalChannelsAngle_Duetto16L[16] = {
      -3.75782f, -3.25777f, -2.75729f, -2.25646f, -1.75533f, -1.25397f,
      -0.75245f, -0.25083f, 0.250827f, 0.752447f, 1.253969f, 1.755328f,
      2.256457f, 2.757293f, 3.25777f,  3.757823f};

  float m_verticalChannelsAngle_Duetto16R[16] = {
      -3.51152f, -3.00987f, -2.50823f, -2.00658f, -1.50494f, -1.00329f,
      -0.50165f, 0.0f,      0.501645f, 1.003224f, 1.504673f, 2.005925f,
      2.506916f, 3.00758f,  3.507853f, 4.00767f};
  double m_verticalChannelAngle_Duetto16L_cos_vA_RA[16] = {0};
  double m_verticalChannelAngle_Duetto16L_sin_vA_RA[16] = {0};
  double m_verticalChannelAngle_Duetto16R_cos_vA_RA[16] = {0};
  double m_verticalChannelAngle_Duetto16R_sin_vA_RA[16] = {0};

  // 左右机芯修正角度 Duetto
  double m_leftMoveAngle = 30.0;
  double m_rightMoveAngle = 30.0;

  double m_rotate_duetto_sinL;
  double m_rotate_duetto_cosL;
  double m_rotate_duetto_sinR;
  double m_rotate_duetto_cosR;

  double m_skewing_duetto_Angle[3] = {-4.5, 0.0, 4.5};
  double m_skewing_sin_duetto[3];
  double m_skewing_cos_duetto[3];
  double duettoPivotVector[3] = {0, 0, 1};
  double m_correction_movement_L[3] = {0.017, 0, 0};
  double m_correction_movement_R[3] = {-0.017, 0, 0};
  double m_offsetVerAngleL = 0.0;
  double m_offsetVerAngleR = 0.0;
  double m_duettoA1_correction_movement_L[3] = {0.025, 0, 0};
  double m_duettoA1_correction_movement_R[3] = {-0.025, 0, 0};

  // TSP48-Polar
  float m_verticalChannelsAngle_TSP48[16] = {
      -5.274283f, -4.574258f, -3.872861f, -3.1703f,  -2.466783f, -1.762521f,
      -1.057726f, -0.352611f, 0.352611f,  1.057726f, 1.762521f,  2.466783f,
      3.1703f,    3.872861f,  4.574258f,  5.274283f};
  float m_verticalChannelAngle_TSP48_cos_vA_RA[16] = {0.f};
  float m_verticalChannelAngle_TSP48_sin_vA_RA[16] = {0.f};
  double m_skewing_tsp48_Angle[3] = {-4.5, 0.0, 4.5};
  double m_skewing_sin_tsp48[3];
  double m_skewing_cos_tsp48[3];
  unsigned char PCF_type = 0x0;

  // Scope256
  bool m_decode_for_scope128H = false;
  double m_verticalChannelsAngle_SCP256L[64] = {
      -34.96, -34.43, -33.89, -33.35, -32.81, -32.27, -31.73, -31.19,
      -30.65, -30.11, -29.57, -29.03, -28.49, -27.95, -27.4,  -26.85,
      -26.3,  -25.75, -25.2,  -24.64, -24.08, -23.52, -22.95, -22.39,
      -21.82, -21.25, -20.67, -20.1,  -19.52, -18.95, -18.37, -17.79,
      -17.46, -16.93, -16.39, -15.85, -15.31, -14.77, -14.23, -13.69,
      -13.15, -12.61, -12.07, -11.53, -10.99, -10.45, -9.9,   -9.35,
      -8.8,   -8.25,  -7.7,   -7.14,  -6.58,  -6.02,  -5.45,  -4.89,
      -4.32,  -3.75,  -3.17,  -2.6,   -2.02,  -1.45,  -0.87,  -0.29};
  double m_verticalChannelsAngle_SCP256R[64] = {
      0.29,  0.87,  1.45,  2.02,  2.6,   3.17,  3.75,  4.32,  4.89,  5.45,
      6.02,  6.58,  7.14,  7.7,   8.25,  8.8,   9.35,  9.9,   10.45, 10.99,
      11.53, 12.07, 12.61, 13.15, 13.69, 14.23, 14.77, 15.31, 15.85, 16.39,
      16.93, 17.46, 17.79, 18.37, 18.95, 19.52, 20.1,  20.67, 21.25, 21.82,
      22.39, 22.95, 23.52, 24.08, 24.64, 25.2,  25.75, 26.3,  26.85, 27.4,
      27.95, 28.49, 29.03, 29.57, 30.11, 30.65, 31.19, 31.73, 32.27, 32.81,
      33.35, 33.89, 34.43, 34.96};
  double m_verticalChannelAngle_scp256L_sin_vA_RA[64] = {0.f}; // beta
  double m_verticalChannelAngle_scp256L_cos_vA_RA[64] = {0.f};
  double m_verticalChannelAngle_scp256R_sin_vA_RA[64] = {0.f};
  double m_verticalChannelAngle_scp256R_cos_vA_RA[64] = {0.f};

  double m_Scope256LeftLaserOffset[4] = {0, 0, 0, 0};
  double m_Scope256RightLaserOffset[4] = {0, 0, 0, 0};

  int m_scope256LaserVersion = 0;
  int m_scope256PLAngleCorrectFlag = 0;

  //
  double m_scp256MirrorABCAmend[3] = {0.2, 0.0, 0.0}; // delta
  double m_skewing_sin_scp256[3];
  double m_skewing_cos_scp256[3];

  double m_scp256MoveAngleL = 4.0; // gamma
  double m_scp256MoveAngleR = 4.0;
  double m_rotate_scp256L_sin;
  double m_rotate_scp256L_cos;
  double m_rotate_scp256R_sin;
  double m_rotate_scp256R_cos;

  double m_scp256OffsetVerAngleL = 0.0;
  double m_scp256OffsetVerAngleR = 0.0;
  double m_0[19] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  double m_1[60] = {
      0.07, 0.10, 0.11, 0.10, 0.09,  0.07,  0.04, 0.05, 0.05, 0.07, 0.07, 0.07,
      0.07, 0.05, 0.02, 0.01, -0.02, -0.04, 0.06, 0.12, 0.13, 0.13, 0.12, 0.11,
      0.10, 0.07, 0.09, 0.08, 0.09,  0.11,  0.12, 0.12, 0.09, 0.09, 0.08, 0.06,
      0.05, 0.04, 0.05, 0.05, 0.07,  0.06,  0.05, 0.04, 0.03, 0.03, 0.01, 0.01,
      0.01, 0.03, 0.05, 0.05, 0.05,  0.04,  0.04, 0.03, 0.02, 0.03, 0.08, 0.03};
  double m_2[60] = {0.11, 0.17, 0.21, 0.21, 0.20, 0.16, 0.14, 0.11, 0.09, 0.09,
                    0.14, 0.18, 0.19, 0.19, 0.17, 0.16, 0.14, 0.13, 0.10, 0.10,
                    0.12, 0.15, 0.15, 0.14, 0.13, 0.10, 0.11, 0.10, 0.10, 0.11,
                    0.12, 0.13, 0.12, 0.14, 0.13, 0.12, 0.11, 0.10, 0.09, 0.09,
                    0.10, 0.12, 0.13, 0.11, 0.11, 0.11, 0.10, 0.09, 0.08, 0.09,
                    0.10, 0.10, 0.12, 0.12, 0.12, 0.11, 0.09, 0.09, 0.11, 0.12};
  double m_3[60] = {0.10, 0.13, 0.15, 0.17, 0.17, 0.16, 0.14, 0.11, 0.09, 0.08,
                    0.08, 0.10, 0.11, 0.11, 0.10, 0.09, 0.07, 0.05, 0.03, 0.05,
                    0.11, 0.12, 0.13, 0.13, 0.12, 0.10, 0.09, 0.07, 0.05, 0.03,
                    0.02, 0.04, 0.08, 0.11, 0.11, 0.11, 0.09, 0.08, 0.07, 0.07,
                    0.08, 0.10, 0.10, 0.10, 0.10, 0.09, 0.08, 0.06, 0.04, 0.04,
                    0.04, 0.06, 0.09, 0.09, 0.09, 0.08, 0.07, 0.07, 0.07, 0.08};
  double m_4[60] = {0.07,  0.07,  0.09,  0.09,  0.10,  0.10, 0.09,  0.07, 0.06,
                    0.04,  0.01,  -0.01, -0.02, 0.02,  0.02, -0.01, 0.00, -0.01,
                    -0.02, -0.04, -0.05, -0.04, 0.05,  0.03, 0.01,  0.02, 0.01,
                    0.02,  0.01,  0.01,  0.00,  -0.03, 0.00, 0.01,  0.00, 0.00,
                    -0.01, 0.00,  -0.01, -0.02, -0.02, 0.00, 0.03,  0.04, 0.02,
                    0.00,  0.00,  0.01,  0.02,  0.02,  0.01, 0.00,  0.01, 0.02,
                    0.01,  -0.02, -0.01, -0.01, -0.01};
  double m_5[60] = {
      0.07, 0.10, 0.11, 0.10, 0.09,  0.07,  0.04, 0.05, 0.05, 0.07, 0.07, 0.07,
      0.07, 0.05, 0.02, 0.01, -0.02, -0.04, 0.06, 0.12, 0.13, 0.13, 0.12, 0.11,
      0.10, 0.07, 0.09, 0.08, 0.09,  0.11,  0.12, 0.12, 0.09, 0.09, 0.08, 0.06,
      0.05, 0.04, 0.05, 0.05, 0.07,  0.06,  0.05, 0.04, 0.03, 0.03, 0.01, 0.01,
      0.01, 0.03, 0.05, 0.05, 0.05,  0.04,  0.04, 0.03, 0.02, 0.03, 0.08, 0.03};
  double m_6[60] = {0.11, 0.17, 0.21, 0.21, 0.20, 0.16, 0.14, 0.11, 0.09, 0.09,
                    0.14, 0.18, 0.19, 0.19, 0.17, 0.16, 0.14, 0.13, 0.10, 0.10,
                    0.12, 0.15, 0.15, 0.14, 0.13, 0.10, 0.11, 0.10, 0.10, 0.11,
                    0.12, 0.13, 0.12, 0.14, 0.13, 0.12, 0.11, 0.10, 0.09, 0.09,
                    0.10, 0.12, 0.13, 0.11, 0.11, 0.11, 0.10, 0.09, 0.08, 0.09,
                    0.10, 0.10, 0.12, 0.12, 0.12, 0.11, 0.09, 0.09, 0.11, 0.12};
  double m_7[60] = {0.10, 0.13, 0.15, 0.17, 0.17, 0.16, 0.14, 0.11, 0.09, 0.08,
                    0.08, 0.10, 0.11, 0.11, 0.10, 0.09, 0.07, 0.05, 0.03, 0.05,
                    0.11, 0.12, 0.13, 0.13, 0.12, 0.10, 0.09, 0.07, 0.05, 0.03,
                    0.02, 0.04, 0.08, 0.11, 0.11, 0.11, 0.09, 0.08, 0.07, 0.07,
                    0.08, 0.10, 0.10, 0.10, 0.10, 0.09, 0.08, 0.06, 0.04, 0.04,
                    0.04, 0.06, 0.09, 0.09, 0.09, 0.08, 0.07, 0.07, 0.07, 0.08};
  double m_8[60] = {0.07,  0.07,  0.09,  0.09,  0.10,  0.10, 0.09,  0.07, 0.06,
                    0.04,  0.01,  -0.01, -0.02, 0.02,  0.02, -0.01, 0.00, -0.01,
                    -0.02, -0.04, -0.05, -0.04, 0.05,  0.03, 0.01,  0.02, 0.01,
                    0.02,  0.01,  0.01,  0.00,  -0.03, 0.00, 0.01,  0.00, 0.00,
                    -0.01, 0.00,  -0.01, -0.02, -0.02, 0.00, 0.03,  0.04, 0.02,
                    0.00,  0.00,  0.01,  0.02,  0.02,  0.01, 0.00,  0.01, 0.02,
                    0.01,  -0.02, -0.01, -0.01, -0.01};

  // Scope128，v1是老版β角，v2是75mm雷达更改后的β角
  const double m_verticalChannelsAngle_SCP128P_v1[32] = {
      -34.43, -33.35, -32.27, -31.19, -30.11, -29.03, -27.95, -26.85,
      -25.75, -24.64, -23.52, -22.39, -21.25, -20.1,  -18.95, -17.79,
      -16.93, -15.85, -14.77, -13.69, -12.61, -11.53, -10.45, -9.35,
      -8.25,  -7.14,  -6.02,  -4.89,  -3.775, -2.6,   -1.45,  -0.29};

  const double m_verticalChannelsAngle_SCP128N_v1[32] = {
      0.87,  2.02,  3.17,  4.32,  5.45,  6.58,  7.7,   8.8,
      9.9,   10.99, 12.07, 13.15, 14.23, 15.31, 16.39, 17.46,
      18.37, 19.52, 20.67, 21.82, 22.95, 24.08, 25.2,  26.3,
      27.4,  28.49, 29.57, 30.65, 31.73, 32.81, 33.89, 34.96};

  const double m_verticalChannelsAngle_SCP128P_v2[32] = {
      -34.73, -33.65, -32.57, -31.49, -30.41, -29.33, -28.25, -27.15,
      -26.05, -24.94, -23.82, -22.69, -21.55, -20.40, -19.25, -18.09,
      -17.23, -16.15, -15.07, -13.99, -12.91, -11.83, -10.75, -9.65,
      -8.55,  -7.44,  -6.32,  -5.19,  -4.075, -2.90,  -1.75,  -0.59};

  const double m_verticalChannelsAngle_SCP128N_v2[32] = {
      0.590,  1.750,  2.900,  4.075,  5.190,  6.320,  7.440,  8.550,
      9.650,  10.750, 11.830, 12.910, 13.990, 15.070, 16.150, 17.230,
      18.090, 19.250, 20.400, 21.550, 22.690, 23.820, 24.940, 26.050,
      27.150, 28.250, 29.330, 30.410, 31.490, 32.570, 33.650, 34.730};

  double m_verticalChannelsAngle_SCP128P[32] = {};
  double m_verticalChannelsAngle_SCP128N[32] = {};
  int m_scp128BetaAnglePS = 1; // PS标志物为1则使用新的β角，为0则使用老的β角

  double m_verticalChannelAngle_scp128P_sin_vA_RA[32] = {0.f}; // beta
  double m_verticalChannelAngle_scp128P_cos_vA_RA[32] = {0.f};
  double m_verticalChannelAngle_scp128N_sin_vA_RA[32] = {0.f};
  double m_verticalChannelAngle_scp128N_cos_vA_RA[32] = {0.f};

  double m_scp128MoveAngleP = 184.0; // gamma
  double m_scp128MoveAngleN = 356.0;
  double m_rotate_scp128P_sin;
  double m_rotate_scp128P_cos;
  double m_rotate_scp128N_sin;
  double m_rotate_scp128N_cos;

  double m_skewing_scope128_Angle[3] = {0.0, 0.4, 0.4};
  double m_skewing_sin_scope128[3];
  double m_skewing_cos_scope128[3];

  double m_Scope128PosLaserOffset[4] = {0, 0, 0, 0};
  double m_Scope128NegLaserOffset[4] = {0, 0, 0, 0};

  double m_Scope128PosBetaAngleOffset = 0.0;
  double m_Scope128NegBetaAngleOffset = 0.0;

  // TempoA4
  double m_tempoA4MirrorABCAmend[3] = {0.0};
  double m_skewing_sin_tempoA4[3];
  double m_skewing_cos_tempoA4[3];

  // FocusB2
  float m_verticalChannelAngle_FocusB2[64] = {
      -12.368f, -11.986f, -11.603f, -11.219f, -10.834f, -10.448f, -10.061f,
      -9.674f,  -9.285f,  -8.896f,  -8.505f,  -8.115f,  -7.723f,  -7.331f,
      -6.938f,  -6.545f,  -6.151f,  -5.756f,  -5.361f,  -4.966f,  -4.570f,
      -4.174f,  -3.777f,  -3.381f,  -2.983f,  -2.586f,  -2.189f,  -1.791f,
      -1.393f,  -0.995f,  -0.597f,  -0.199f,  0.199f,   0.597f,   0.995f,
      1.393f,   1.791f,   2.189f,   2.586f,   2.983f,   3.381f,   3.777f,
      4.174f,   4.570f,   4.966f,   5.361f,   5.756f,   6.151f,   6.545f,
      6.938f,   7.331f,   7.723f,   8.115f,   8.505f,   8.896f,   9.285f,
      9.674f,   10.061f,  10.448f,  10.834f,  11.219f,  11.603f,  11.986f,
      12.368f};
  double m_verticalChannelAngle_FocusB2_cos_vA_RA[64] = {0.f};
  double m_verticalChannelAngle_FocusB2_sin_vA_RA[64] = {0.f};
  double m_FocusB2MirrorABCAmend[3] = {0.0};
  double m_skewing_sin_FocusB2[3];
  double m_skewing_cos_FocusB2[3];

  // FocusT
  float m_verticalChannelAngle_FocusT[96] = {
      12.41f, 12.06f,  11.72f,  11.37f,  11.03f,  10.68f,  10.33f,  9.98f,
      9.64f,  9.46f,   9.29f,   9.12f,   8.94f,   8.77f,   8.59f,   8.42f,
      8.24f,  8.07f,   7.90f,   7.72f,   7.55f,   7.37f,   7.20f,   7.02f,
      6.85f,  6.67f,   6.50f,   6.32f,   6.15f,   5.97f,   5.80f,   5.62f,
      5.45f,  5.27f,   5.10f,   4.92f,   4.75f,   4.57f,   4.39f,   4.22f,
      4.04f,  3.87f,   3.69f,   3.52f,   3.34f,   3.17f,   2.99f,   2.81f,
      2.64f,  2.46f,   2.29f,   2.11f,   1.93f,   1.76f,   1.58f,   1.41f,
      1.23f,  0.88f,   0.53f,   0.18f,   -0.18f,  -0.53f,  -0.88f,  -1.23f,
      -1.58f, -1.93f,  -2.29f,  -2.64f,  -2.99f,  -3.34f,  -3.69f,  -4.04f,
      -4.39f, -4.75f,  -5.10f,  -5.45f,  -5.80f,  -6.15f,  -6.50f,  -6.85f,
      -7.20f, -7.55,   -7.90f,  -8.24f,  -8.59f,  -8.94f,  -9.29f,  -9.64f,
      -9.98f, -10.33f, -10.68f, -11.03f, -11.37f, -11.72f, -12.06f, -12.41f};
  double m_verticalChannelAngle_FocusT_cos_vA_RA[96] = {0.f};
  double m_verticalChannelAngle_FocusT_sin_vA_RA[96] = {0.f};

  double m_FocusTMirrorABCAmend[3] = {0.0};
  double m_skewing_sin_FocusT[3];
  double m_skewing_cos_FocusT[3];

  // 定义 alpha 值映射
  const std::unordered_map<int, double> m_alphaMap_FocusT = {
      {10, 0.002627512}, {25, 0.002627512}, {81, 0.002627512},
      {9, 0.007753735},  {26, 0.007753735}, {89, 0.007753735},
      {12, 0.012879959}, {27, 0.012879959}, {82, 0.012879959},
      {11, 0.018006182}, {28, 0.018006182}, {90, 0.018006182},
      {14, 0.023132406}, {29, 0.023132406}, {83, 0.023132406},
      {13, 0.02825863},  {30, 0.02825863},  {91, 0.02825863},
      {16, 0.033384853}, {31, 0.033384853}, {84, 0.033384853},
      {15, 0.038511077}, {32, 0.038511077}, {92, 0.038511077},
      {18, 0.0436373},   {33, 0.0436373},   {85, 0.0436373},
      {17, 0.048763524}, {34, 0.048763524}, {93, 0.048763524},
      {20, 0.053889748}, {35, 0.053889748}, {86, 0.053889748},
      {19, 0.059015971}, {36, 0.059015971}, {94, 0.059015971},
      {22, 0.064142195}, {37, 0.064142195}, {87, 0.064142195},
      {21, 0.069268418}, {38, 0.069268418}, {95, 0.069268418},
      {24, 0.074394642}, {39, 0.074394642}, {88, 0.074394642},
      {23, 0.079520866}, {40, 0.079520866}, {96, 0.079520866},
      {42, 0.084647089}, {65, 0.084647089}, {1, 0.084647089},
      {41, 0.089773313}, {73, 0.089773313}, {57, 0.089773313},
      {44, 0.094899536}, {66, 0.094899536}, {2, 0.094899536},
      {43, 0.10002576},  {74, 0.10002576},  {58, 0.10002576},
      {46, 0.105151984}, {67, 0.105151984}, {3, 0.105151984},
      {45, 0.110278207}, {75, 0.110278207}, {59, 0.110278207},
      {48, 0.115404431}, {68, 0.115404431}, {4, 0.115404431},
      {47, 0.120530654}, {76, 0.120530654}, {60, 0.120530654},
      {50, 0.125656878}, {69, 0.125656878}, {5, 0.125656878},
      {49, 0.130783101}, {77, 0.130783101}, {61, 0.130783101},
      {52, 0.135909325}, {70, 0.135909325}, {6, 0.135909325},
      {51, 0.141035549}, {78, 0.141035549}, {62, 0.141035549},
      {54, 0.146161772}, {71, 0.146161772}, {7, 0.146161772},
      {53, 0.151287996}, {79, 0.151287996}, {63, 0.151287996},
      {56, 0.156414219}, {72, 0.156414219}, {8, 0.156414219},
      {55, 0.161540443}, {80, 0.161540443}, {64, 0.161540443}};

  // TW360
  float m_verticalChannelAngle_TW360_47deg[48] = {
      -5.23f, -4.33f, -3.41f, -2.48f, -1.54f, -0.59f, 0.37f,  1.35f,
      2.33f,  3.32f,  4.33f,  5.34f,  6.36f,  7.39f,  8.43f,  9.47f,
      10.52f, 11.57f, 12.63f, 13.69f, 14.76f, 15.83f, 16.9f,  17.97f,
      19.03f, 20.1f,  21.17f, 22.24f, 23.31f, 24.37f, 25.43,  26.48f,
      27.53f, 28.57f, 29.61f, 30.64f, 31.66f, 32.67f, 33.68f, 34.67f,
      35.65f, 36.63f, 37.59f, 38.54f, 39.48f, 40.41f, 41.33f, 42.23f};
  float m_verticalChannelAngle_TW360_50deg[48] = {
      -5.02f, -4.08f, -3.13f, -2.16f, -1.17f, -0.18f, 0.83f,  1.86f,
      2.89f,  3.93f,  4.99f,  6.06f,  7.14f,  8.22f,  9.32f,  10.42f,
      11.53f, 12.65f, 13.77f, 14.9f,  16.03f, 17.16f, 18.3f,  19.44f,
      20.56f, 21.7f,  22.84f, 23.97f, 25.1f,  26.23f, 27.35f, 28.47f,
      29.58f, 30.68f, 31.78f, 32.87f, 33.94f, 35.01f, 36.07f, 37.11f,
      38.15f, 39.17f, 40.18f, 41.17f, 42.16f, 43.13f, 44.08f, 45.02f};
  float m_verticalChannelAngle_TW360_90deg[48] = {};
  //  float m_verticalChannelAngle_TW360_48[48] = {};
  //  float m_verticalChannelAngle_TW360_96[96] = {};
  //  float m_verticalChannelAngle_TW360_192[192] = {};
  float m_verticalChannelAngle_TW360[192] = {};
  double m_Z_TW360 = 0;   // 光心到原点的 Z 轴高度
  double m_R_TW360 = 0;   // 光心到原点的平面半径
  int m_verticalType = 1; // 垂直Fov类型，0-47°，1-50°，2-90°
  int m_channelNumber = 48; // 雷达通道，0-48通道，1-96通道，2-192通道
  double m_verticalOffset = 0; // 垂直角度补偿
  int m_laserBlock = 4;        // 激光分区数，1/2/4/8/12/24
  const std::unordered_map<uint8_t, int> m_laserBlockMap = {
      {1, 1}, {2, 2}, {8, 8}, {12, 12}, {24, 24}};
  double m_laserBlockOffset_1 = -1.16; // 激光分区补偿值1
  double m_laserBlockOffset_2 = 1.16;  // 激光分区补偿值2

  double m_verticalChannelAngle_TW360_cos_vA_RA[48] = {0.f};
  double m_verticalChannelAngle_TW360_sin_vA_RA[48] = {0.f};

  //  double m_TW360MirrorABCAmend[3] = {0.0};
  //  double m_skewing_sin_TW360[3];
  //  double m_skewing_cos_TW360[3];

  // 俯仰与水平修正角度
  double m_MirrorVerAngleOffset[3] = {0.0};
  double m_MirrorHorAngleOffset[3] = {0.0};

  // 机芯垂直与水平角度
  double m_LeftRightMechCoreVerAngleOffset[2] = {0.0};
  double m_LeftRightMechCoreHorAngleOffset[2] = {0.0};

  bool m_tempoA4UserMode = true;
  bool m_tsp48UserMode = true;

private:
  const int TensorPulseMapValue = 50;
  const int ScopePulseMapValue = 35;

  double m_intensityNormalizeL = 16.0;
  double m_intensityNormalizeR = 16.0;

private:
#ifdef __linux__
  SocketT sockets[N_INPUT_SOCKETS] = {(socklen_t)INVALID_SOCKET};
#elif _WIN32
  SocketT sockets[N_INPUT_SOCKETS] = {INVALID_SOCKET};
#endif

  int _period_count = -1;
  int _cur_udp_count = -1;
  bool _has_max_count = false;

  LidarType _lidarType;
  int _lidarTypeFromDIF = -1; // 从DIF中读到的雷达型号,需要转换成字符串
  std::string _lidarIPOrPcapPath;
  std::string _hostIPOrLidarIPForFilter;
  int _pointcloudPort;
  int _DIFPort;
  int _IMUPort;

  bool _repeat;
  float _rate = 1.0;
  uint64_t _stamp = 0;
  uint64_t _first_stamp = 0; // 记录单帧内第一个点云的时间戳
  std::string _stampType = "last_point"; // 判断帧时间戳用第一个点云、最后一个
  bool _is_lidar_time = true; // 判断是否使用雷达时间

  bool _cycle_count_frame_split = false; // 是否使用周期计数进行分帧
  int _pre_cycle_count = 0;              // 记录上一个udp包的周期计数
  bool _has_framed_angle = false; // 判断是否根据角度进行了分帧
  bool _has_framed_cycle_count =
      false; // 判断最终是否要进行分帧（角度分帧/周期计数分帧）
  bool _isLidarReady =
      false; // 雷达是否收到DIF帧，收到DIF帧后置为true，可以发布点云数据
  bool _set_time_windows = false;    // 网页端是否配置时间分帧模式
  bool _use_time_windows = false;    // launch中是否应用时间分帧模式
  bool _enable_time_windows = false; // 是否开启时间分帧模式
  int _frame_interval = 100; // 时间分帧模式的分帧间隔，默认每帧为100毫秒
  uint64_t _last_frame_time = 0;
  uint64_t _now_frame_time = 0;
  // int frame_id=0;
  uint64_t _udp_count = 0;
  uint64_t _frame_time = 0;

  ILidarObserver *_lidarObserver = nullptr;

  PackageList _packageList;

  std::vector<uint64_t> _seekPointCloud;
  std::atomic<uint64_t> _frameIndex;

  std::atomic<bool> _run;
  std::atomic<bool> _play;
  std::atomic<bool> _parsing;
  std::atomic<bool> _seeking;
  std::atomic<bool> _callbackdone;
  std::atomic<bool> _parsed;

  std::mutex _mutex;
  std::condition_variable _condi;
  LD_STATUS _status = LD_STOPED;

  ILidarAlgo *_lidarAlgo = nullptr;

  double _dis_min = DBL_MIN;
  double _dis_max = DBL_MAX;

  EchoNum _echoNum = Echo1;
  std::thread _workThread;
  std::thread _decodeThread;
  std::thread _parsePcapThread;

  UserPointCloud::Ptr _pointCloudPtr;

  int _validPointsTimes = 0;
  int _curIndex = 0;

  LidarInfo _lidarInfo;
  std::string _frameID = "TanwayTP";

  std::chrono::time_point<std::chrono::system_clock> _beginPlayTime =
      std::chrono::system_clock::now();
  u_int _last_tv_sec = 0;
  u_int _last_tv_usec = 0;

  double _encoding_interval = 0;
  int state = 0;
  int label = 0;

  friend class ILidarDevice;
// TempoA3通道计算
// 输入：片号1-3、单片通道号1-24（从上到下）
// 输出：单镜面通道号1-72（从下到上）
#define CHECKCH(P, Q) (73 - ((P - 1) * 24 + Q)) // 1~72
// 输入：单镜面通道号1-72、镜面值0-2
// 输出：整体通道号定义1-216
#define CHECKCH216(ch, mirror) ((ch - 1) * 3 + (3 - mirror))
  // TempoA3 校验通道数据定义
  typedef struct _check_data_info {
    _check_data_info(int _ch1, bool _ch1_value, int _ch2, bool _ch2_value)
        : ch1(_ch1), ch1_value(_ch1_value), ch2(_ch2), ch2_value(_ch2_value) {}
    _check_data_info() : ch1(0), ch1_value(false), ch2(0), ch2_value(false) {}
    int ch1;
    bool ch1_value;
    int ch2;
    bool ch2_value;
  } CheckDataInfo;

  CheckDataInfo m_tempoA3ChannelToCheckIndex[72]; // 校验索引
  double m_verticalChannelAngle_TempoA3_cos_vA_RA[72] = {0.f};
  double m_verticalChannelAngle_TempoA3_sin_vA_RA[72] = {0.f};
  double m_tempoA3MirrorABCAmend[3] = {0.08, 0.0, -0.08};
  double m_tempoA3RecvAmend[3] = {4.196, -4.196, 1.802};
  double m_skewing_sin_tempoA3[3];
  double m_skewing_cos_tempoA3[3];
};
} // namespace tanway
