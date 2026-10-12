#pragma once
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace tanway {

enum AlgoID {
  UNKNOW = 0,
  UNDISTORT_POINT,            // 1  雷达静止下，运动畸变滤除
  SUNLIGHT_NOISE,             // 2
  SUNLIGHT_NOISE_V2,          // 3
  RAIN_POINT,                 // 4
  UNDER_GROUND_POINT,         // 5
  MIRROR_MOTION_COMPENSATION, // 6
  OUTLIER_POINT,              // 7
  CROSSTALK,                  // 8
  DISTURB_POINT,              // 9
  JOINT_ABC,                  // 10
  REFLECT_CALIB,              // 11
  DRAG_POINT,                 // 12  拖点算法
  RAINFOG_FILTER,             // 13  雨雾滤除算法
  HIGHREF_FILTER,             // 14  高反串扰算法
  SUNLIGHT_NOISE_V3,          // 15  Focus阳光噪点算法
  DELETE_POINT,               // 16  Focus
  CROSSTALK2,                 // 17  scope256
  SMALLBLIND,                 // 18  Scope256_SmallBlind
  DISCRETE_POINT,             // 19  离散点滤除，十字形
  CloseRange_Noise,           // 20  近距离噪点滤除
  Motion_Point, // 21  运动状态分离算法（与连通域算法绑定）
  Two_Pass_Connection,       // 22  连通域聚类算法
  CloseRange_Noise_128,      // 23  128近距离噪点与孤立点
  HighRef_128,               // 24  128高反虚影滤除
  FixedNoise_128,            // 25  128固定噪点滤除
  LidarInterfere_128,        // 26  128雷达干扰滤除（同型号）
  UNDISTORT_POINT2,          // 27  雷达静止下，运动畸变滤除
  SMALLBLIND_SCOPE128,       // 28  Scope128_SmallBlind
  HIGHREF_FILTER_128,        // 29  Scope128高反串扰算法
  HIGHREF_FILTER_TSP48,      // 30  Tensor48高反串扰算法
  ROTATE_ALL_POINTCLOUD,     // 31  点云坐标变换
  FILTER_POINTS_OF_INTEREST, // 32  根据roi区域筛选点，路端使用
  HIGHREF_FILTER_FOCT,       // 33 FoucsT高反滤除算法
  UNDISTORT_POINT_MOVE,      // 34 车载运动补偿算法
  GROUND_DET                 // 35 地面检测算法
};

enum LidarType {
  LT_Unknow = -1,
  LT_Tensor16,            // 0
  LT_Tensor32,            // 1
  LT_Scope192,            // 2
  LT_Duetto,              // 3
  LT_TempoA1,             // 4
  LT_TempoA2,             // 5
  LT_ScopeMini,           // 6
  LT_TempoA3,             // 7
  LT_TempoA4,             // 8
  LT_Tensor48,            // 9
  LT_Tensor48_Depth,      // 10
  LT_Scope256,            // 11
  LT_Scope256_Depth,      // 12
  LT_FocusB1,             // 13
  LT_Scope256_SmallBlind, // 14
  LT_FocusB2_B3_MP,       // 15
  LT_Scope128H,           // 16
  LT_Scope128,            // 17
  LT_Scope128F,           // 18
  LT_FocusB2_64,          // 19 实验型号
  LT_FocusT,              // 20
  LT_TW360,               // 21
  LT_Total
};

enum EchoNum { EchoNone = 0, Echo1 = 0x01, Echo2 = 0x02 };

struct LidarInfo {
  int lidarID = 0;
  std::string lidarIPOrPcapPath;
  std::string hostIPOrLidarIPForFilter;
  int pointcloudPort;
  int DIFPort;
  int IMUPort;
  LidarType lidarType;
  bool online;
  uint64_t status = 0;
};

struct Header {
  /** \brief Sequence number */
  uint32_t seq = 0;
  /** \brief A timestamp associated with the time when the data was acquired
   *
   * The value represents microseconds since 1970-01-01 00:00:00 (the UNIX
   * epoch).
   */
  uint64_t stamp = 0;
  /** \brief Coordinate frame ID */
  std::string frame_id = "TanwayTP";
};

template <typename PointT>
#ifdef _MSC_VER
struct __declspec(align(16)) PointCloud
#elif __GNUC__
struct __attribute__((aligned(16))) PointCloud
#endif
{
  typedef std::vector<PointT> Points;
  typedef std::shared_ptr<PointCloud<PointT>> Ptr;

  void push_back(PointT point) { points.push_back(point); }
  int size() const { return points.size(); }
  void clear() { points.clear(); }
  void reserve(unsigned int count) { points.reserve(count); }

  uint32_t height = 0;
  uint32_t width = 0;
  Header header;
  Points points;
};

struct IMUData {
  IMUData() {
    memset(angular_velocity, 0, sizeof(float) * 3);
    memset(linear_acceleration, 0, sizeof(float) * 3);
  }
  uint64_t stamp = 0;
  std::string frame_id = "TanwayIMU";

  bool calibrate = false;
  float temperature = 0;
  float angular_velocity[3];
  float linear_acceleration[3];
  float gyro_noise = 0;
  float gyro_bias = 0;
  float accel_noise = 0;
  float accel_bias = 0;
};

typedef struct _ori_point {
  double x;
  double y;
  double z;

  double intensity = 0;
  double distance = 0;
  int channel = 0;
  double angle = 0;
  double pulse = 0;

  int echo = 1;
  int mirror = 0;
  int left_right = 0;
  int confidence = 0;
  int block = 0;
  unsigned int t_sec = 0;
  unsigned int t_usec = 0;
  double speed = 0;
  double encoding_interval = 0;
  double state = 0;
  double label = 0;
  bool framesplit = false;
  bool back_point = false;
  double apd_temp = 0;
  int pulseCodeInterval = 0;
  int cycle_count = 0; // 周期计数
} TWPoint;

typedef PointCloud<TWPoint> TWPointCloud;

#ifdef _MSC_VER
struct __declspec(align(16)) DeviceInfoFrame
#elif __GNUC__
struct __attribute__((aligned(16))) DeviceInfoFrame
#endif
{
  DeviceInfoFrame() {}
  DeviceInfoFrame(const DeviceInfoFrame &other) {
    this->header = other.header;
    this->deviceType = other.deviceType;
    this->deviceNumber = other.deviceNumber;
    this->psVersion = other.psVersion;
    this->plVersion = other.plVersion;
    this->workMode = other.workMode;
    this->workStatus = other.workStatus;
    memcpy(this->privateData, other.privateData, 1024);
  }

  DeviceInfoFrame &operator=(const DeviceInfoFrame &other) {
    if (this != &other) {
      this->header = other.header;
      this->deviceType = other.deviceType;
      this->deviceNumber = other.deviceNumber;
      this->psVersion = other.psVersion;
      this->plVersion = other.plVersion;
      this->workMode = other.workMode;
      this->workStatus = other.workStatus;
      memcpy(this->privateData, other.privateData, 1024);
    }

    return *this;
  }

  uint32_t header;
  uint32_t deviceType;
  uint32_t deviceNumber;
  std::string psVersion;
  std::string plVersion;
  uint8_t workMode;
  uint8_t workStatus;
  uint8_t privateData[1024];
};

#define MemberCheck(member)                                                    \
  template <typename T> struct has_member_##member {                           \
    template <typename _T>                                                     \
    static auto check(_T) -> typename std::decay<decltype(_T::member)>::type;  \
    static void check(...);                                                    \
    using type = decltype(check(std::declval<T>()));                           \
    enum { value = !std::is_void<type>::value };                               \
  };

MemberCheck(x);
MemberCheck(y);
MemberCheck(z);
MemberCheck(intensity);
MemberCheck(distance);
MemberCheck(channel);
MemberCheck(angle);
MemberCheck(pulse);
MemberCheck(echo);
MemberCheck(mirror);
MemberCheck(left_right);
MemberCheck(confidence);
MemberCheck(block);
MemberCheck(t_sec);
MemberCheck(t_usec);
MemberCheck(apd_temp);
MemberCheck(pulseCodeInterval);

#define PointT_HasMember(C, member) has_member_##member<C>::value

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, x)>::type
setX(PointT &point, const float &value) {}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, x)>::type
setX(PointT &point, const float &value) {
  point.x = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, y)>::type
setY(PointT &point, const float &value) {}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, y)>::type
setY(PointT &point, const float &value) {
  point.y = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, z)>::type
setZ(PointT &point, const float &value) {}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, z)>::type
setZ(PointT &point, const float &value) {
  point.z = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, intensity)>::type
setIntensity(PointT &point, const float &value) {}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, intensity)>::type
setIntensity(PointT &point, const float &value) {
  point.intensity = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, distance)>::type
setDistance(PointT &point, const float &value) {}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, distance)>::type
setDistance(PointT &point, const float &value) {
  point.distance = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, channel)>::type
setChannel(PointT &point, const int &value) {}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, channel)>::type
setChannel(PointT &point, const int &value) {
  point.channel = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, angle)>::type
setAngle(PointT &point, const float &value) {}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, angle)>::type
setAngle(PointT &point, const float &value) {
  point.angle = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, pulse)>::type
setPulse(PointT &point, const float &value) {}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, pulse)>::type
setPulse(PointT &point, const float &value) {
  point.pulse = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, echo)>::type
setEcho(PointT &point, const int &value) {}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, echo)>::type
setEcho(PointT &point, const int &value) {
  point.echo = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, mirror)>::type
setMirror(PointT &point, const unsigned int &value) {}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, mirror)>::type
setMirror(PointT &point, const unsigned int &value) {
  point.mirror = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, left_right)>::type
setLeftRight(PointT &point, const unsigned int &value) {}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, left_right)>::type
setLeftRight(PointT &point, const unsigned int &value) {
  point.left_right = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, confidence)>::type
setConfidence(PointT &point, const unsigned int &value) {}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, confidence)>::type
setConfidence(PointT &point, const unsigned int &value) {
  point.confidence = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, block)>::type
setBlock(PointT &point, const unsigned int &value) {}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, block)>::type
setBlock(PointT &point, const unsigned int &value) {
  point.block = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, t_sec)>::type
setT_sec(PointT &point, const unsigned int &value) {}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, t_sec)>::type
setT_sec(PointT &point, const unsigned int &value) {
  point.t_sec = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, t_usec)>::type
setT_usec(PointT &point, const unsigned int &value) {}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, t_usec)>::type
setT_usec(PointT &point, const unsigned int &value) {
  point.t_usec = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, apd_temp)>::type
setAPDTemp(PointT &point, double &value) {}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, apd_temp)>::type
setAPDTemp(PointT &point, double &value) {
  point.apd_temp = value;
}

template <typename PointT>
inline
    typename std::enable_if<!PointT_HasMember(PointT, pulseCodeInterval)>::type
    setPulseCodeInterval(PointT &point, const unsigned int &value) {}

template <typename PointT>
inline
    typename std::enable_if<PointT_HasMember(PointT, pulseCodeInterval)>::type
    setPulseCodeInterval(PointT &point, const unsigned int &value) {
  point.pulseCodeInterval = value;
}

} // namespace tanway
