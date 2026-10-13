#include "LidarDevice.h"
#include <fstream>
#include <functional>

#include <iostream>

namespace tanway {
constexpr int MAX_UDP_COUNT = 50000; // 时间分帧模式最大一帧内的udp包数
std::unique_ptr<ILidarDevice>
ILidarDevice::Create(const std::string &lidar, const std::string &localIP,
                     int localPointloudPort, int localDIFPort,
                     ILidarObserver *lidarObserver, LidarType lidarType,
                     bool repeat, int lidarID, int localIMUPort) {
  return std::unique_ptr<ILidarDevice>(
      new LidarDevice(lidar, localIP, localPointloudPort, localDIFPort,
                      lidarObserver, lidarType, repeat, lidarID, localIMUPort));
}

LidarDevice::~LidarDevice() { Stop(); }

std::string ILidarDevice::VersionNumber() { return versionNumber; }
std::string ILidarDevice::DebugString() { return debugString; }

bool LidarDevice::Start(bool play) {
  std::lock_guard<std::mutex> lock(_mutex);

  if (!_lidarObserver) {
    std::cout << "[lidar]: Lidar Observer can not be null!" << std::endl;
    return false;
  }

  if (_status != LD_STOPED) {
    std::cout << "[lidar]: Lidar wrong status!" << std::endl;
    return false;
  }

#ifdef _WIN32
  WSADATA wsd;
  int nResult = WSAStartup(MAKEWORD(2, 2), &wsd);
  if (nResult != NO_ERROR) {
    std::cout << "[lidar]: WSAStartup failed!" << std::endl;
    return false;
  }
#endif

  _pointCloudPtr = make_shared_point_cloud;
  _pointCloudPtr->header.frame_id = _frameID;
  _pointCloudPtr->reserve(10000);
  // _pointCloudBackUpPtr = make_shared_point_cloud;
  // make_shared_point_cloud->reserve(10000);

  _run.store(true);
  _callbackdone.store(true);
  _play.store(play);
  _parsing.store(false);
  _seeking.store(false);

  if (isValidIp(_lidarIPOrPcapPath))
    _workThread =
        std::thread(std::bind(&LidarDevice::ReadFromNetworkThread, this));
  else
    _workThread =
        std::thread(std::bind(&LidarDevice::ReadFromLocalThread, this));

  _decodeThread = std::thread(std::bind(&LidarDevice::DecodeThread, this));

  _status = LD_STARTED;
  return true;
}

void LidarDevice::SetPlayRate(float rate) {
  if (rate >= 0.5 && rate <= 4.0)
    _rate = rate;
}

void LidarDevice::Stop() {

  _parsing.store(false);
  _run.store(false);
  _play.store(false);

  if (_workThread.joinable())
    _workThread.join();
  if (_decodeThread.joinable())
    _decodeThread.join();
  if (_parsePcapThread.joinable())
    _parsePcapThread.join();

#ifdef _WIN32
  WSACleanup();
#endif

  std::lock_guard<std::mutex> lock(_mutex);
  _status = LD_STOPED;
  logger.Stop();
}

void LidarDevice::SetDistanceRange(double min, double max) {
  if (min < max) {
    _dis_min = min;
    _dis_max = max;
  }
}

void LidarDevice::SetAngleRange(double min, double max) {
  if (min < max) {
    m_startAngle = min;
    m_endAngle = max;
  }
}

bool LidarDevice::StartParsePcap() {
  std::lock_guard<std::mutex> lock(_mutex);

  if (_lidarObserver == nullptr) {
    std::cout << "[lidar]: _lidarObserver is nullptr" << std::endl;
    return false;
  }

  if (_parsing || _status != LD_STOPED) {
    _lidarObserver->OnException(
        _lidarInfo,
        Exception(ERR_PCAP_FILE_INVALID, "[lidar]: Lidar wrong status!"));
    return false;
  }

  if (isValidIp(_lidarIPOrPcapPath)) {
    _lidarObserver->OnException(
        _lidarInfo, Exception(ERR_PCAP_FILE_INVALID,
                              "[lidar]: online mode , can not parse pcap!"));
    return false;
  }

  std::ifstream inStream(_lidarIPOrPcapPath,
                         std::ios_base::in | std::ios_base::binary);
  if (!inStream) {
    _lidarObserver->OnException(
        _lidarInfo,
        Exception(ERR_OPEN_PCAP_FAILED, "[lidar]: Open pcap file failed!"));
    return false;
  }

  inStream.seekg(sizeof(Pcap_FileHeader), std::ios::beg);
  if (inStream.eof()) {
    _lidarObserver->OnException(
        _lidarInfo,
        Exception(ERR_PCAP_FILE_INVALID, "[lidar]: The pcap file is invalid!"));
    return false;
  } else {
    _lidarObserver->OnException(
        _lidarInfo,
        Exception(TIPS_OPEN_PCAP_SUCCESS,
                  std::string("[lidar]: ParsePcap Open pcap file ") +
                      _lidarIPOrPcapPath + " successed!"));
  }

  inStream.close();
  _parsing.store(true);
  _parsePcapThread =
      std::thread(std::bind(&LidarDevice::ParsePcapThread, this));
  _status = LD_PARSING;
  return true;
}

void LidarDevice::StopParsePcap() {
  _parsing.store(false);
  if (_parsePcapThread.joinable())
    _parsePcapThread.join();
}

void LidarDevice::SeekFrame(int index) {
  if (index > 0 && index < _seekPointCloud.size() && _parsed)
    _frameIndex.store(index);
}

void LidarDevice::UpdateFrame() { _frameIndex.store(_curIndex); }

bool LidarDevice::GetPlayMode() { return isValidIp(_lidarIPOrPcapPath); }

void LidarDevice::SetEchoNum(EchoNum echoNum) { _echoNum = echoNum; }

void LidarDevice::SetFrameID(const std::string &frameID) {
  _frameID = frameID;
  if (_pointCloudPtr)
    _pointCloudPtr->header.frame_id = _frameID;
}

void LidarDevice::SetMirrorVerAngleOffset(float a, float b, float c) {
  m_MirrorVerAngleOffset[0] = a;
  m_MirrorVerAngleOffset[1] = b;
  m_MirrorVerAngleOffset[2] = c;
}

void LidarDevice::SetMirrorHorAngleOffset(float a, float b, float c) {
  m_MirrorHorAngleOffset[0] = a;
  m_MirrorHorAngleOffset[1] = b;
  m_MirrorHorAngleOffset[2] = c;
}

void LidarDevice::SetCoreVerAngleOffset(float l, float r) {
  m_LeftRightMechCoreVerAngleOffset[0] = l;
  m_LeftRightMechCoreVerAngleOffset[1] = r;
}

void LidarDevice::SetCoreHorAngleOffset(float l, float r) {
  m_LeftRightMechCoreHorAngleOffset[0] = l;
  m_LeftRightMechCoreHorAngleOffset[1] = r;
}

void LidarDevice::SetXYZRotateAngle(float x, float y, float z) {
  m_transformRotateX = x;
  m_transformRotateY = y;
  m_transformRotateZ = z;

  // 计算各轴三角函数值
  const float sx = sin(x * m_calRA), cx = cos(x * m_calRA);
  const float sy = sin(y * m_calRA), cy = cos(y * m_calRA);
  const float sz = sin(z * m_calRA), cz = cos(z * m_calRA);

  // z（yaw）-y(pitch)-x(roll)顺序旋转
  m_rotationMatrix[0][0] = cz * cy;
  m_rotationMatrix[0][1] = cz * sy * sx - sz * cx;
  m_rotationMatrix[0][2] = cz * sy * cx + sz * sx;

  m_rotationMatrix[1][0] = sz * cy;
  m_rotationMatrix[1][1] = sz * sy * sx + cz * cx;
  m_rotationMatrix[1][2] = sz * sy * cx - cz * sx;

  m_rotationMatrix[2][0] = -sy;
  m_rotationMatrix[2][1] = cy * sx;
  m_rotationMatrix[2][2] = cy * cx;
}

void LidarDevice::SetXYZTransVec(float x, float y, float z) {
  m_transformMoveX = x;
  m_transformMoveY = y;
  m_transformMoveZ = z;
}

void LidarDevice::SetXForwardFlag() { m_x_forward_flag = true; }

int LidarDevice::GetPcapFrameNum() { return _seekPointCloud.size() - 1; }

void LidarDevice::SetLidarAlgo(ILidarAlgo *lidarAlgo) {
  _lidarAlgo = lidarAlgo;
}
// 设置雷达的时间戳类型
void LidarDevice::SetTimeStampType(const std::string &timestamp) {
  _stampType = timestamp;
}
// 设置使用雷达时间还是系统时间
void LidarDevice::SetLidarTime(bool lidartime) { _is_lidar_time = lidartime; }

void LidarDevice::SetTimeWindowMode(bool useTimeWindow) {
  _use_time_windows = useTimeWindow;
}

void LidarDevice::SetFrameSplit(bool frameSplit) {
  _cycle_count_frame_split = frameSplit;
}

void LidarDevice::Play(bool play) { _play.store(play); }

void LidarDevice::InitBasicVariables() {
  if (_lidarType == LT_Tensor16 || _lidarType == LT_Tensor32 ||
      _lidarType == LT_Scope192 || _lidarType == LT_TempoA1 ||
      _lidarType == LT_ScopeMini || _lidarType == LT_TempoA2 ||
      _lidarType == LT_TempoA3 || _lidarType == LT_Tensor48_Depth ||
      _lidarType == LT_Scope256_Depth)
    _isLidarReady =
        true; // 对于没有DIF帧的这些型号，_isLidarReady置为true可以在不解析DIF帧的情况下正常回调
  // SDK识别到TW360时，将默认FOV设置为0.01-360°
  if (_lidarType == LT_TW360) {
    m_startAngle = 0.01;
    m_endAngle = 360;
  }

  // transform

  m_transformMoveX = 0;
  m_transformMoveY = 0;
  m_transformMoveZ = 0;

  // Scope-192
  m_skewing_sin_scope[0] = sin(m_skewing_scope_Angle_Correct[0] * m_calRA);
  m_skewing_sin_scope[1] = sin(m_skewing_scope_Angle_Correct[1] * m_calRA);
  m_skewing_sin_scope[2] = sin(m_skewing_scope_Angle_Correct[2] * m_calRA);

  m_skewing_cos_scope[0] = cos(m_skewing_scope_Angle_Correct[0] * m_calRA);
  m_skewing_cos_scope[1] = cos(m_skewing_scope_Angle_Correct[1] * m_calRA);
  m_skewing_cos_scope[2] = cos(m_skewing_scope_Angle_Correct[2] * m_calRA);

  for (int i = 0; i < 64; i++) {
    double vA = m_verticalChannelAngle_Scope64[i];
    m_verticalChannelAngle_Scope64_cos_vA_RA[i] = cos(vA * m_calRA);
    m_verticalChannelAngle_Scope64_sin_vA_RA[i] = sin(vA * m_calRA);
  }

  // Scope-Mini-A2-192
  m_skewing_sin_scopeMiniA2_192[0] =
      sin(m_skewing_scopeMiniA2_Angle_Correct[0] * m_calRA);
  m_skewing_sin_scopeMiniA2_192[1] =
      sin(m_skewing_scopeMiniA2_Angle_Correct[1] * m_calRA);
  m_skewing_sin_scopeMiniA2_192[2] =
      sin(m_skewing_scopeMiniA2_Angle_Correct[2] * m_calRA);

  m_skewing_cos_scopeMiniA2_192[0] =
      cos(m_skewing_scopeMiniA2_Angle_Correct[0] * m_calRA);
  m_skewing_cos_scopeMiniA2_192[1] =
      cos(m_skewing_scopeMiniA2_Angle_Correct[1] * m_calRA);
  m_skewing_cos_scopeMiniA2_192[2] =
      cos(m_skewing_scopeMiniA2_Angle_Correct[2] * m_calRA);

  for (int i = 0; i < 64; i++) {
    double vA = m_verticalChannelAngle_Scope64_A2[i];
    m_verticalChannelAngle_ScopeMiniA2_cos_vA_RA[i] = cos(vA * m_calRA);
    m_verticalChannelAngle_ScopeMiniA2_sin_vA_RA[i] = sin(vA * m_calRA);
  }

  // FocusB1
  m_skewing_sin_Focus[0] = sin(m_skewing_Focus_Angle_Correct[0] * m_calRA);
  m_skewing_sin_Focus[1] = sin(m_skewing_Focus_Angle_Correct[1] * m_calRA);
  m_skewing_sin_Focus[2] = sin(m_skewing_Focus_Angle_Correct[2] * m_calRA);

  m_skewing_cos_Focus[0] = cos(m_skewing_Focus_Angle_Correct[0] * m_calRA);
  m_skewing_cos_Focus[1] = cos(m_skewing_Focus_Angle_Correct[1] * m_calRA);
  m_skewing_cos_Focus[2] = cos(m_skewing_Focus_Angle_Correct[2] * m_calRA);

  for (int i = 0; i < 64; i++) {
    double vA = m_verticalChannelAngle_Focus[i];
    m_verticalChannelAngle_Focus_cos_vA_RA[i] = cos(vA * m_calRA);
    m_verticalChannelAngle_Focus_sin_vA_RA[i] = sin(vA * m_calRA);
  }

  // TSP03-32
  m_skewing_sin_tsp[0] = sin(m_skewing_tsp_Angle[0] * m_calRA);
  m_skewing_sin_tsp[1] = sin(m_skewing_tsp_Angle[1] * m_calRA); //-6.0

  m_skewing_cos_tsp[0] = cos(m_skewing_tsp_Angle[0] * m_calRA);
  m_skewing_cos_tsp[1] = cos(m_skewing_tsp_Angle[1] * m_calRA);

  for (int i = 0; i < 16; i++) {
    double vA = m_verticalChannelsAngle_Tensor16[i];
    m_verticalChannelAngle16_cos_vA_RA[i] = cos(vA * m_calRA);
    m_verticalChannelAngle16_sin_vA_RA[i] = sin(vA * m_calRA);
  }

  // Duetto
  for (int i = 0; i < 16; i++) {
    double vA_L = m_verticalChannelsAngle_Duetto16L[i];
    m_verticalChannelAngle_Duetto16L_cos_vA_RA[i] = cos(vA_L * m_calRA);
    m_verticalChannelAngle_Duetto16L_sin_vA_RA[i] = sin(vA_L * m_calRA);

    double vA_R = m_verticalChannelsAngle_Duetto16R[i];
    m_verticalChannelAngle_Duetto16R_cos_vA_RA[i] = cos(vA_R * m_calRA);
    m_verticalChannelAngle_Duetto16R_sin_vA_RA[i] = sin(vA_R * m_calRA);
  }
  double DuettoA_Elevation = -4.5;
  double DuettoB_Elevation = 0.0;
  double DuettoC_Elevation = 4.5;
  m_skewing_sin_duetto[0] = sin(DuettoA_Elevation * m_calRA);
  m_skewing_sin_duetto[1] = sin(DuettoB_Elevation * m_calRA);
  m_skewing_sin_duetto[2] = sin(DuettoC_Elevation * m_calRA);
  m_skewing_cos_duetto[0] = cos(DuettoA_Elevation * m_calRA);
  m_skewing_cos_duetto[1] = cos(DuettoB_Elevation * m_calRA);
  m_skewing_cos_duetto[2] = cos(DuettoC_Elevation * m_calRA);
  m_rotate_duetto_sinL = sin(m_leftMoveAngle * m_calRA);  //
  m_rotate_duetto_cosL = cos(m_leftMoveAngle * m_calRA);  //
  m_rotate_duetto_sinR = sin(m_rightMoveAngle * m_calRA); //
  m_rotate_duetto_cosR = cos(m_rightMoveAngle * m_calRA); //

  // LT_Tensor48
  for (int i = 0; i < 16; i++) {
    double vA_L = m_verticalChannelsAngle_TSP48[i];
    m_verticalChannelAngle_TSP48_cos_vA_RA[i] = cos(vA_L * m_calRA);
    m_verticalChannelAngle_TSP48_sin_vA_RA[i] = sin(vA_L * m_calRA);
  }
  for (int i = 0; i < 3; i++) {
    m_skewing_sin_tsp48[i] = sin(m_skewing_tsp48_Angle[i] * m_calRA);
    m_skewing_cos_tsp48[i] = cos(m_skewing_tsp48_Angle[i] * m_calRA);
  }

  // LT_Scope256
  for (int i = 0; i < 64; i++) {
    double vA_L = m_verticalChannelsAngle_SCP256L[i] + m_scp256OffsetVerAngleL;
    m_verticalChannelAngle_scp256L_sin_vA_RA[i] = sin(vA_L * m_calRA);
    m_verticalChannelAngle_scp256L_cos_vA_RA[i] = cos(vA_L * m_calRA);

    double vA_R = m_verticalChannelsAngle_SCP256R[i] + m_scp256OffsetVerAngleR;
    m_verticalChannelAngle_scp256R_sin_vA_RA[i] = sin(vA_R * m_calRA);
    m_verticalChannelAngle_scp256R_cos_vA_RA[i] = cos(vA_R * m_calRA);
  }
  for (int i = 0; i < 3; i++) {
    m_skewing_sin_scp256[i] = sin(m_scp256MirrorABCAmend[i] * m_calRA);
    m_skewing_cos_scp256[i] = cos(m_scp256MirrorABCAmend[i] * m_calRA);
  }
  m_rotate_scp256L_sin = sin(m_scp256MoveAngleL * m_calRA);
  m_rotate_scp256L_cos = cos(m_scp256MoveAngleL * m_calRA);
  m_rotate_scp256R_sin = sin(m_scp256MoveAngleR * m_calRA);
  m_rotate_scp256R_cos = cos(m_scp256MoveAngleR * m_calRA);

  // LT_Scope128
  for (int i = 0; i < 32; i++) {
    m_verticalChannelsAngle_SCP128P[i] = m_verticalChannelsAngle_SCP128P_v2[i];
    m_verticalChannelsAngle_SCP128N[i] = m_verticalChannelsAngle_SCP128N_v2[i];

    double vA_P = m_verticalChannelsAngle_SCP128P[i]; // 默认使用新的β角
    m_verticalChannelAngle_scp128P_sin_vA_RA[i] = sin(vA_P * m_calRA);
    m_verticalChannelAngle_scp128P_cos_vA_RA[i] = cos(vA_P * m_calRA);
    double vA_N = m_verticalChannelsAngle_SCP128N[i];
    m_verticalChannelAngle_scp128N_sin_vA_RA[i] = sin(vA_N * m_calRA);
    m_verticalChannelAngle_scp128N_cos_vA_RA[i] = cos(vA_N * m_calRA);
  }

  for (int i = 0; i < 3; i++) {
    m_skewing_sin_scope128[i] = sin(m_skewing_scope128_Angle[i] * m_calRA);
    m_skewing_cos_scope128[i] = cos(m_skewing_scope128_Angle[i] * m_calRA);
  }

  m_rotate_scp128P_sin = sin(m_scp128MoveAngleP * m_calRA);
  m_rotate_scp128P_cos = cos(m_scp128MoveAngleP * m_calRA);
  m_rotate_scp128N_sin = sin(m_scp128MoveAngleN * m_calRA);
  m_rotate_scp128N_cos = cos(m_scp128MoveAngleN * m_calRA);

  // TempoA3
  for (int i = 0; i < 3; i++) {
    m_skewing_sin_tempoA3[i] = sin(m_tempoA3MirrorABCAmend[i] * m_calRA);
    m_skewing_cos_tempoA3[i] = cos(m_tempoA3MirrorABCAmend[i] * m_calRA);
  }

  // TempoA4
  for (int i = 0; i < 3; i++) {
    m_skewing_sin_tempoA4[i] = sin(m_tempoA4MirrorABCAmend[i] * m_calRA);
    m_skewing_cos_tempoA4[i] = cos(m_tempoA4MirrorABCAmend[i] * m_calRA);
  }

  // FocusB2
  for (int i = 0; i < 3; i++) {
    m_skewing_sin_FocusB2[i] = sin(m_FocusB2MirrorABCAmend[i] * m_calRA);
    m_skewing_cos_FocusB2[i] = cos(m_FocusB2MirrorABCAmend[i] * m_calRA);
  }

  for (int i = 0; i < 64; i++) {
    double vA = m_verticalChannelAngle_FocusB2[i];
    m_verticalChannelAngle_FocusB2_cos_vA_RA[i] = cos(vA * m_calRA);
    m_verticalChannelAngle_FocusB2_sin_vA_RA[i] = sin(vA * m_calRA);
  }

  // FocusT
  for (int i = 0; i < 3; i++) {
    m_skewing_sin_FocusT[i] = sin(m_FocusTMirrorABCAmend[i] * m_calRA);
    m_skewing_cos_FocusT[i] = cos(m_FocusTMirrorABCAmend[i] * m_calRA);
  }

  for (int i = 0; i < 96; i++) {
    double vA = m_verticalChannelAngle_FocusT[i];
    m_verticalChannelAngle_FocusT_cos_vA_RA[i] = cos(vA * m_calRA);
    m_verticalChannelAngle_FocusT_sin_vA_RA[i] = sin(vA * m_calRA);
  }

  // TW360  todo
  for (int i = 0; i < 48; i++) {
    m_verticalChannelAngle_TW360[i] = m_verticalChannelAngle_TW360_50deg[i];
    double vA = m_verticalChannelAngle_TW360[i];
    m_verticalChannelAngle_TW360_cos_vA_RA[i] = cos(vA * m_calRA);

    m_verticalChannelAngle_TW360_sin_vA_RA[i] = sin(vA * m_calRA);
  }
}

bool LidarDevice::CreateAndBindSocket() {

  SOCKADDR_IN localAddr;

  // piont cloud
  sockets[INPUT_SOCKET_POINT_CLOUD] = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

  if (sockets[INPUT_SOCKET_POINT_CLOUD] == INVALID_SOCKET)
    return false;

  localAddr.sin_family = AF_INET;
  localAddr.sin_port = htons(_pointcloudPort);

#ifdef __linux__
  localAddr.sin_addr.s_addr = inet_addr(_hostIPOrLidarIPForFilter.data());
#elif _WIN32
  inet_pton(AF_INET, _hostIPOrLidarIPForFilter.data(),
            &(localAddr.sin_addr.s_addr));
#endif

  if (bind(sockets[INPUT_SOCKET_POINT_CLOUD], (SOCKADDR *)&localAddr,
           sizeof(localAddr)) == SOCKET_ERROR)
    goto pointcloudfailed;

  // dif
  sockets[INPUT_SOCKET_DIF] = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

  if (sockets[INPUT_SOCKET_DIF] == INVALID_SOCKET)
    goto diffailed;

  localAddr.sin_port = htons(_DIFPort);

  if (bind(sockets[INPUT_SOCKET_DIF], (SOCKADDR *)&localAddr,
           sizeof(localAddr)) == SOCKET_ERROR)
    goto diffailed;

  // IMU
  if ((_lidarType == LT_Duetto || _lidarType == LT_TW360) &&
      _IMUPort != _DIFPort) {
    sockets[INPUT_SOCKET_IMU] = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockets[INPUT_SOCKET_IMU] == INVALID_SOCKET)
      goto imufailed;

    localAddr.sin_port = htons(_IMUPort);
    if (bind(sockets[INPUT_SOCKET_IMU], (SOCKADDR *)&localAddr,
             sizeof(localAddr)) == SOCKET_ERROR)
      goto imufailed;
  } else {
    // 端口相同，关闭IMU套接字
    sockets[INPUT_SOCKET_IMU] = INVALID_SOCKET;
  }
  return true;
imufailed:
#ifdef __linux__
  close(sockets[INPUT_SOCKET_IMU]);
#elif _WIN32
  closesocket(sockets[INPUT_SOCKET_IMU]);
#endif

diffailed:
#ifdef __linux__
  close(sockets[INPUT_SOCKET_DIF]);
#elif _WIN32
  closesocket(sockets[INPUT_SOCKET_DIF]);
#endif
pointcloudfailed:
#ifdef __linux__
  close(sockets[INPUT_SOCKET_POINT_CLOUD]);
#elif _WIN32
  closesocket(sockets[INPUT_SOCKET_POINT_CLOUD]);
#endif

  return false;
}

void LidarDevice::ReadFromNetworkThread() {
  int bind_times = 0;
  while (true) {
    if (!CreateAndBindSocket()) {
      if (bind_times >= 5) {
        _lidarObserver->OnException(
            _lidarInfo,
            Exception(ERR_INIT_SOCKET,
                      std::string("Create or bind socket error, retry bind ") +
                          std::to_string(bind_times) +
                          std::string(" times failed!")));
        return;
      }

      std::this_thread::sleep_for(std::chrono::seconds(2));
      _lidarObserver->OnException(
          _lidarInfo,
          Exception(TIPS_TIMEOUT_POINT,
                    std::string("Create or bind socket error, retry bind ") +
                        std::to_string(++bind_times) + std::string(" times!")));
    } else
      break;
  }

  SOCKADDR_IN sendAddr, recvAddr;
  socklen_t nRecvAddrSize = sizeof(recvAddr);

#ifdef __linux__
  sendAddr.sin_addr.s_addr = inet_addr(_lidarIPOrPcapPath.data());
#elif _WIN32
  inet_pton(AF_INET, _lidarIPOrPcapPath.data(), &(sendAddr.sin_addr.s_addr));
#endif

  _packageList.Clear();

  while (_run) {
    fd_set readfds;
    FD_ZERO(&readfds);
    int max_fd = 0;
    // 遍历所有套接字，筛选有效描述符并计算最大值
    for (int i = 0; i < N_INPUT_SOCKETS; i++) {
      if (sockets[i] != INVALID_SOCKET) { // 跳过无效套接字
        FD_SET(sockets[i], &readfds);
        if (sockets[i] > max_fd) {
          max_fd = sockets[i];
        }
      }
    }

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;

    int ret = select(max_fd + 1, &readfds, NULL, NULL, &tv);

    if (ret < 0) {

#ifdef __linux__
      _lidarObserver->OnException(
          _lidarInfo,
          Exception(ERR_SELECT_SOCKET,
                    std::string("select error ") + std::to_string(errno)));
#elif _WIN32
      _lidarObserver->OnException(
          _lidarInfo,
          Exception(ERR_SELECT_SOCKET, std::string("select error ") +
                                           std::to_string(GetLastError())));
#endif

      break;
    }

    if (ret == 0) {
      // time out
      _lidarObserver->OnException(
          _lidarInfo,
          Exception(TIPS_TIMEOUT_SELECT, std::string("select time out ")));
      continue;
    }
    // 处理点云数据
    if (FD_ISSET(sockets[INPUT_SOCKET_POINT_CLOUD], &readfds)) {
      UDPPackage::Ptr udpData = _packageList.PopFreePackage();
      if (!udpData) {
        udpData = _packageList.PopPackage();
      }

      udpData->m_length =
          recvfrom(sockets[INPUT_SOCKET_POINT_CLOUD], udpData->m_szData,
                   UDP_MAX_LENGTH, 0, (SOCKADDR *)&recvAddr, &nRecvAddrSize);
      if (udpData->m_length < 0) {

#ifdef __linux__
        _lidarObserver->OnException(
            _lidarInfo, Exception(ERR_SOCKET_RECV_POINT,
                                  std::string("recv point cloud failed err ") +
                                      std::to_string(errno)));
#elif _WIN32
        _lidarObserver->OnException(
            _lidarInfo, Exception(ERR_SOCKET_RECV_POINT,
                                  std::string("recv point cloud failed err ") +
                                      std::to_string(GetLastError())));
#endif
        break;
      }
      if (sendAddr.sin_addr.s_addr != recvAddr.sin_addr.s_addr ||
          !IsValidFrame(udpData->m_szData, udpData->m_length))
        _packageList.PushFreePackage(udpData);
      else {
        CalNowTime(udpData);
        _packageList.PushPackage(udpData);
      }
    }
    // 处理DIF数据
    if (FD_ISSET(sockets[INPUT_SOCKET_DIF], &readfds)) {
      UDPPackage::Ptr udpData = _packageList.PopFreePackage();
      if (!udpData) {
        udpData = _packageList.PopPackage();
      }
      udpData->m_length =
          recvfrom(sockets[INPUT_SOCKET_DIF], udpData->m_szData, UDP_MAX_LENGTH,
                   0, (SOCKADDR *)&recvAddr, &nRecvAddrSize);
      if (udpData->m_length < 0) {

#ifdef __linux__
        _lidarObserver->OnException(
            _lidarInfo, Exception(ERR_SOCKET_RECV_DIF,
                                  std::string("recv dif data failed err ") +
                                      std::to_string(errno)));
#elif _WIN32
        _lidarObserver->OnException(
            _lidarInfo, Exception(ERR_SOCKET_RECV_DIF,
                                  std::string("recv dif data failed err ") +
                                      std::to_string(GetLastError())));
#endif

        break;
      }
      if (sendAddr.sin_addr.s_addr != recvAddr.sin_addr.s_addr ||
          !IsValidFrame(udpData->m_szData, udpData->m_length))
        _packageList.PushFreePackage(udpData);
      else {
        CalNowTime(udpData);
        _packageList.PushPackage(udpData);
      }
    }

    // 处理IMU数据
    if (sockets[INPUT_SOCKET_IMU] != INVALID_SOCKET &&
        FD_ISSET(sockets[INPUT_SOCKET_IMU], &readfds)) {
      UDPPackage::Ptr udpData = _packageList.PopFreePackage();
      if (!udpData) {
        udpData = _packageList.PopPackage();
      }

      udpData->m_length =
          recvfrom(sockets[INPUT_SOCKET_IMU], udpData->m_szData, UDP_MAX_LENGTH,
                   0, (SOCKADDR *)&recvAddr, &nRecvAddrSize);
      if (udpData->m_length < 0) {
#ifdef __linux__
        _lidarObserver->OnException(
            _lidarInfo, Exception(ERR_SOCKET_RECV_IMU,
                                  std::string("recv IMU data failed err ") +
                                      std::to_string(errno)));
#elif _WIN32
        _lidarObserver->OnException(
            _lidarInfo, Exception(ERR_SOCKET_RECV_IMU,
                                  std::string("recv IMU data failed err ") +
                                      std::to_string(GetLastError())));
#endif
        break;
      }

      if (sendAddr.sin_addr.s_addr != recvAddr.sin_addr.s_addr ||
          !IsValidFrame(udpData->m_szData, udpData->m_length))
        _packageList.PushFreePackage(udpData);
      else {
        CalNowTime(udpData);
        _packageList.PushPackage(udpData);
      }
    }
  }

  // 安全关闭所有套接字
  for (int i = 0; i < N_INPUT_SOCKETS; i++) {
    if (sockets[i] != INVALID_SOCKET) {
#ifdef __linux__
      close(sockets[i]);
#elif _WIN32
      closesocket(sockets[i]);
#endif
    }
  }
}

void LidarDevice::ReadFromLocalThread() {
#ifdef __linux__
  DIR *dir;
  struct dirent *entry;
  std::vector<std::string> pcapFolder;

  if ((dir = opendir(_lidarIPOrPcapPath.c_str())) == NULL) {
    pcapFolder.push_back(_lidarIPOrPcapPath);
  } else {
    while ((entry = readdir(dir)) != NULL) {
      std::string fileName = entry->d_name;
      const std::string suffix = "pcap";
      if (fileName.length() > suffix.length() &&
          (0 == fileName.compare(fileName.length() - suffix.length(),
                                 suffix.length(), suffix))) {
        pcapFolder.push_back(_lidarIPOrPcapPath + '/' + fileName);
      }
    }
    std::sort(pcapFolder.begin(), pcapFolder.end());
  }

  closedir(dir);

  int pcap_num = pcapFolder.size();
  int pcap_idx = 0;
  while (_run) {
    std::ifstream inStream(pcapFolder[pcap_idx],
                           std::ios_base::in | std::ios_base::binary);
    if (!inStream) {
      _lidarObserver->OnException(
          _lidarInfo,
          Exception(ERR_OPEN_PCAP_FAILED,
                    "Open pcap file failed!," + pcapFolder[pcap_idx]));
      return;
    }
    inStream.seekg(sizeof(Pcap_FileHeader), std::ios::beg);
    if (inStream.eof()) {
      _lidarObserver->OnException(
          _lidarInfo,
          Exception(ERR_PCAP_FILE_INVALID, "The pcap file is invalid!"));
      return;
    }

    _curIndex = 1;
    while (_run) {
      if (!_seeking) {
        uint64_t index = _frameIndex.exchange(0);
        if (index) {
          while (!_callbackdone) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
          inStream.seekg(_seekPointCloud[index - 1], std::ios::beg);
          _seeking.store(true);
          _curIndex = index;
          _packageList.Clear();
          _pointCloudPtr->clear();
          _last_tv_sec = 0;
          _last_tv_usec = 0;
        } else if (!_play) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          _last_tv_sec = 0;
          _last_tv_usec = 0;
          continue;
        }
      }

      UDPPackage::Ptr udpData = _packageList.PopFreePackage();

      if (!udpData) {
        udpData = _packageList.PopPackage();
        udpData->frameIndex = 0;
        udpData->framed = false;
      }

      if (_parsed) {
        if (_seeking) {
          if (inStream.tellg() == _seekPointCloud[_curIndex]) {
            _seeking.store(false);
            udpData->framed = true;
            _callbackdone = false;
          }
        } else {
          if (_curIndex < _seekPointCloud.size() - 1 &&
              inStream.tellg() == _seekPointCloud[_curIndex + 1]) {
            udpData->framed = true;
            _curIndex++;
          }
        }
      }

      if (!ReadUdpPacket(inStream, udpData, _seeking))
        break;

      CalNowTime(udpData);
      udpData->frameIndex = _curIndex;
      _packageList.PushPackage(udpData);
    }

    _lidarObserver->OnException(
        _lidarInfo, Exception(TIPS_REPEAT_PLAY,
                              "end of pcap file:" + pcapFolder[pcap_idx]));

    inStream.clear();
    _seeking.store(false);

    pcap_idx++;
    if (pcap_idx == pcap_num) {
      pcap_idx = 0;
      _play.store(_repeat);
      if (_repeat) {
        _lidarObserver->OnException(
            _lidarInfo, Exception(TIPS_REPEAT_PLAY,
                                  "end of all pcap file, start repeat! "));
      } else {
        _lidarObserver->OnException(
            _lidarInfo, Exception(TIPS_REPEAT_PLAY,
                                  "end of all pcap file, stop read pcap! "));
      }
    }
  }

#elif _WIN32
  std::ifstream inStream(_lidarIPOrPcapPath,
                         std::ios_base::in | std::ios_base::binary);
  if (!inStream) {
    _lidarObserver->OnException(
        _lidarInfo, Exception(ERR_OPEN_PCAP_FAILED, "Open pcap file failed!"));
    return;
  }

  while (_run) {
    inStream.seekg(sizeof(Pcap_FileHeader), std::ios::beg);
    if (inStream.eof()) {
      _lidarObserver->OnException(
          _lidarInfo,
          Exception(ERR_PCAP_FILE_INVALID, "The pcap file is invalid!"));
      return;
    }

    _curIndex = 1;
    while (_run) {
      if (!_seeking) {
        uint64_t index = _frameIndex.exchange(0);
        if (index) {
          while (!_callbackdone) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
          inStream.seekg(_seekPointCloud[index - 1], std::ios::beg);
          _seeking.store(true);
          _curIndex = index;
          _packageList.Clear();
          _pointCloudPtr->clear();
          _last_tv_sec = 0;
          _last_tv_usec = 0;
        } else if (!_play) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          _last_tv_sec = 0;
          _last_tv_usec = 0;
          continue;
        }
      }

      UDPPackage::Ptr udpData = _packageList.PopFreePackage();

      if (!udpData) {
        udpData = _packageList.PopPackage();
        udpData->frameIndex = 0;
        udpData->framed = false;
      }

      if (_parsed) {
        if (_seeking) {
          if (inStream.tellg() == _seekPointCloud[_curIndex]) {
            _seeking.store(false);
            udpData->framed = true;
            _callbackdone = false;
          }
        } else {
          if (_curIndex < _seekPointCloud.size() - 1 &&
              inStream.tellg() == _seekPointCloud[_curIndex + 1]) {
            udpData->framed = true;
            _curIndex++;
          }
        }
      }

      if (!ReadUdpPacket(inStream, udpData, _seeking))
        break;

      CalNowTime(udpData);
      udpData->frameIndex = _curIndex;
      _packageList.PushPackage(udpData);
    }

    _lidarObserver->OnException(
        _lidarInfo,
        Exception(TIPS_REPEAT_PLAY, "end of pcap file, repeat play data!"));

    inStream.clear();
    _seeking.store(false);
    _play.store(_repeat);
  }

#endif
}

void LidarDevice::DecodeThread() {
  while (_run) {
    if (_packageList.Size() <= 0) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(1));
      continue;
    }

    UDPPackage::Ptr packagePtr = _packageList.PopPackage();

    if (!packagePtr)
      continue;

    if (packagePtr->m_length == 120) {
      // _packageList.PushFreePackage(packagePtr);
      continue;
    }

    DecodeImp(packagePtr);
    _packageList.PushFreePackage(packagePtr);
  }
}

void LidarDevice::ParsePcapThread() {
  _seekPointCloud.clear();
  _seekPointCloud.reserve(1000);
  _pointCloudPtr = make_shared_point_cloud;
  _pointCloudPtr->reserve(10000);
  uint64_t totalSize;
  std::ifstream inStream(_lidarIPOrPcapPath,
                         std::ios_base::in | std::ios_base::binary);

  inStream.seekg(0, std::ios::end);
  totalSize = inStream.tellg();
  inStream.seekg(sizeof(Pcap_FileHeader), std::ios::beg);
  UDPPackage::Ptr udpData(new UDPPackage);
  float maxAngel;
  _last_tv_sec = 0;
  _last_tv_usec = 0;
  int frameIndex = 0;
  while (ReadUdpPacket(inStream, udpData, true)) {
    if (udpData->m_length == 120) {
      continue;
    }
    bool exception = false;
    if (DecodeImp(udpData, &maxAngel, &exception)) {
      _seekPointCloud.push_back((uint64_t)inStream.tellg());
      _lidarObserver->OnParsePcapProcess(
          _lidarInfo, (float)inStream.tellg() * 100 / totalSize, frameIndex++,
          _stamp);
    }

    if (exception) {
      _parsed = false;
      std::lock_guard<std::mutex> lock(_mutex);
      _parsing.store(false);
      _status = LD_STOPED;
      return;
    }
  }

  _parsed.store(_parsing);
  _lidarObserver->OnParsePcapProcess(_lidarInfo, _parsed ? 100 : 0, 0, _stamp);
  std::lock_guard<std::mutex> lock(_mutex);
  _parsing.store(false);
  _status = LD_STOPED;
}

bool LidarDevice::ReadUdpPacket(std::ifstream &inStream,
                                UDPPackage::Ptr &udpData, bool fast) {
  while (true) {
    if (!_parsing && !_run) {
      std::lock_guard<std::mutex> lock(_mutex);
      _status = LD_STOPED;
      return false;
    }

    // pcap pack
    Pcap_PktHdr pcap_pkt_hdr;
    std::streamsize readSize =
        inStream.read((char *)(&pcap_pkt_hdr), sizeof(Pcap_PktHdr)).gcount();

    if (pcap_pkt_hdr.len > UDP_MAX_LENGTH) {
      if (SeekNextValidPacketOffset(inStream))
        continue;
      else
        break;
    }

    if (readSize != sizeof(Pcap_PktHdr))
      break;

    // net pack
    NETHdr net_hdr;
    readSize = inStream.read((char *)(&net_hdr), sizeof(NETHdr)).gcount();

    if (readSize != sizeof(NETHdr))
      break;
    // check invalid
    if (0x0008 != net_hdr.net_type) {
      // ignore
      inStream.seekg(pcap_pkt_hdr.len - sizeof(NETHdr), std::ios::cur);
      continue;
    }

    // IP pcak
    IPHdr ip_hdr;
    readSize = inStream.read((char *)(&ip_hdr), sizeof(IPHdr)).gcount();

    if (readSize != sizeof(IPHdr))
      break;
    // check invalid
    if (_hostIPOrLidarIPForFilter != ip_hdr.GetSourceIP() ||
        ip_hdr.protocol != 17) {
      // ignore
      inStream.seekg(pcap_pkt_hdr.len - sizeof(NETHdr) - sizeof(IPHdr),
                     std::ios::cur);

      continue;
    }

    // UDP pack
    UDPHdr udp_hdr;
    readSize = inStream.read((char *)(&udp_hdr), sizeof(UDPHdr)).gcount();

    if (readSize != sizeof(UDPHdr))
      break;
    // check invalid
    // 检查是否是点云数据端口
    bool isPointCloudData = (_pointcloudPort == udp_hdr.GetDestPort() ||
                             10110 == udp_hdr.GetDestPort());

    // 检查是否是我们需要处理的端口（任何一种）
    bool isValidPort = (isPointCloudData || _DIFPort == udp_hdr.GetDestPort() ||
                        _IMUPort == udp_hdr.GetDestPort());

    if (!isValidPort) {
      // ignore

      inStream.seekg(pcap_pkt_hdr.len - sizeof(NETHdr) - sizeof(IPHdr) -
                         sizeof(UDPHdr),
                     std::ios::cur);
      continue;
    }

    if (!fast && isPointCloudData) {
      if (0 == _last_tv_sec && 0 == _last_tv_usec) {
        _last_tv_sec = pcap_pkt_hdr.tv_sec;
        _last_tv_usec = pcap_pkt_hdr.tv_usec;
        _beginPlayTime = std::chrono::system_clock::now();

      } else {
        long long diff_usecond = 0;
        if (pcap_pkt_hdr.tv_sec == _last_tv_sec) {
          if (pcap_pkt_hdr.tv_usec <= _last_tv_usec)
            diff_usecond = 0;
          else
            diff_usecond = (long long)(pcap_pkt_hdr.tv_usec) - _last_tv_usec;
        } else if (pcap_pkt_hdr.tv_sec > _last_tv_sec) {
          diff_usecond = (pcap_pkt_hdr.tv_sec - _last_tv_sec - 1) * 1000000 +
                         (1000000 - _last_tv_usec) + pcap_pkt_hdr.tv_usec;
        } else {
          diff_usecond = 0;
        }

        _last_tv_sec = pcap_pkt_hdr.tv_sec;
        _last_tv_usec = pcap_pkt_hdr.tv_usec;

        _beginPlayTime += std::chrono::microseconds(int(diff_usecond / _rate));

        std::this_thread::sleep_until(_beginPlayTime);
      }
    }

    // data pack
    udpData->m_length = udp_hdr.GetLength() - sizeof(UDPHdr);

    if (udpData->m_length >
        UDP_MAX_LENGTH - sizeof(NETHdr) - sizeof(IPHdr) - sizeof(UDPHdr)) {
      if (SeekNextValidPacketOffset(inStream))
        continue;
      else
        break;
    }

    udpData->t_sec = pcap_pkt_hdr.tv_sec;
    udpData->t_usec = pcap_pkt_hdr.tv_usec;
    readSize = inStream.read(udpData->m_szData, udpData->m_length).gcount();

    if (readSize != udpData->m_length)
      break;

    if (!IsValidFrame(udpData->m_szData, udpData->m_length)) {
      if (SeekNextValidPacketOffset(inStream))
        continue;
      else
        break;
    }

    return true;
  }

  return false;
}

bool LidarDevice::DecodeImp(const UDPPackage::Ptr &udpData, float *maxAngle,
                            bool *exception) {
  bool ret = false;
  start_time = std::chrono::high_resolution_clock::now();
  switch (_lidarType) {
  case LT_Tensor16:
    if (udpData->m_length == 1440)
      ret = DecodeTensor16(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                           maxAngle, udpData->frameIndex, udpData->framed);
    else
      ON_DECODE_EXCEPTION
    break;
  case LT_Tensor32:
    if (udpData->m_length == 1440)
      ret = DecodeTensor32(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                           maxAngle, udpData->frameIndex, udpData->framed);
    else
      ON_DECODE_EXCEPTION
    break;
  case LT_Scope192:
    if (udpData->m_length == 1120)
      ret = DecodeScope192(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                           maxAngle, udpData->frameIndex, udpData->framed);
    else
      ON_DECODE_EXCEPTION
    break;
  case LT_Duetto:
    if (udpData->m_length == 1348)
      ret = DecodeDuetto(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                         maxAngle, udpData->frameIndex, udpData->framed);
    else if (udpData->m_length == 1024) {
      DecodeDIFData_Duetto(udpData->m_szData);
      PointCloudCallback(udpData->framed, udpData->frameIndex);
    } else if (udpData->m_length == 128)
      DecodeIMUData(udpData->m_szData);
    else
      ON_DECODE_EXCEPTION
    break;
  case LT_TempoA1:
    if (udpData->m_length == 1120)
      ret = DecodeTempoA1(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                          maxAngle, udpData->frameIndex, udpData->framed);
    else
      ON_DECODE_EXCEPTION
    break;
  case LT_ScopeMini:
  case LT_TempoA2:
    if (udpData->m_length == 1120)
      ret = DecodeTempoA2(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                          maxAngle, udpData->frameIndex, udpData->framed);
    else
      ON_DECODE_EXCEPTION
    break;
  case LT_TempoA3:
    if (udpData->m_length == 1120)
      ret = DecodeTempoA3(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                          maxAngle, udpData->frameIndex, udpData->framed);
    else
      ON_DECODE_EXCEPTION
    break;
  case LT_TempoA4:
    if (udpData->m_length == 1348)
      ret = DecodeTempoA4(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                          maxAngle, udpData->frameIndex, udpData->framed);
    else if (udpData->m_length == 1212)
      ret = DecodeTempoA4Calib(udpData->m_szData, &udpData->t_sec,
                               &udpData->t_usec, maxAngle, udpData->frameIndex,
                               udpData->framed);
    else if (udpData->m_length == 1024) {
      DecodeDIFData_TempoA4(udpData->m_szData);
      PointCloudCallback(udpData->framed, udpData->frameIndex);
    } else
      ON_DECODE_EXCEPTION
    break;
  case LT_Tensor48:
    if (udpData->m_length == 1348)
      ret = DecodeTensor48(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                           maxAngle, udpData->frameIndex, udpData->framed);
    else if (udpData->m_length == 1212)
      ret = DecodeTensor48Calib(udpData->m_szData, &udpData->t_sec,
                                &udpData->t_usec, maxAngle, udpData->frameIndex,
                                udpData->framed);
    else if (udpData->m_length == 1024) {
      DecodeDIFData_Tensor48(udpData->m_szData);
      if (_lidarTypeFromDIF != 1)
        ON_DECODE_EXCEPTION
      PointCloudCallback(udpData->framed, udpData->frameIndex);
    } else
      ON_DECODE_EXCEPTION
    break;
  case LT_Tensor48_Depth:
    if (udpData->m_length == 964)
      ret = DecodeTensor48Depth(udpData->m_szData, &udpData->t_sec,
                                &udpData->t_usec, maxAngle, udpData->frameIndex,
                                udpData->framed);
    else
      ON_DECODE_EXCEPTION
    break;
  case LT_Scope256:
  case LT_Scope256_SmallBlind:
    m_decode_for_scope128H = false;
    if (udpData->m_length == 1348)
      ret = DecodeScope256(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                           maxAngle, udpData->frameIndex, udpData->framed);
    else if (udpData->m_length == 1024) {
      DecodeDIFData_Scope256(udpData->m_szData);
      if (_lidarTypeFromDIF != 2)
        ON_DECODE_EXCEPTION
      PointCloudCallback(udpData->framed, udpData->frameIndex);
    } else
      ON_DECODE_EXCEPTION
    break;
  case LT_Scope128H:
    m_decode_for_scope128H = true;
    if (udpData->m_length == 1348)
      ret = DecodeScope256(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                           maxAngle, udpData->frameIndex, udpData->framed);
    else if (udpData->m_length == 1024) {
      DecodeDIFData_Scope256(udpData->m_szData);
      PointCloudCallback(udpData->framed, udpData->frameIndex);
    } else
      ON_DECODE_EXCEPTION
    break;
  case LT_Scope256_Depth:
    if (udpData->m_length == 964)
      ret = DecodeScope256Depth(udpData->m_szData, &udpData->t_sec,
                                &udpData->t_usec, maxAngle, udpData->frameIndex,
                                udpData->framed);
    else
      ON_DECODE_EXCEPTION
    break;
  case LT_FocusB1:
    _isLidarReady = true;
    if (udpData->m_length == 1120)
      ret = DecodeFocusB1(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                          maxAngle, udpData->frameIndex, udpData->framed);
    else if (udpData->m_length == 1024) {
      DecodeDIFData_FocusB1(udpData->m_szData);
      PointCloudCallback(udpData->framed, udpData->frameIndex);
    } else
      ON_DECODE_EXCEPTION
    break;
  case LT_FocusB2_B3_MP:
    if (udpData->m_length == 1120) {
      _isLidarReady =
          true; // 因为Focus_MP默认不传DIF帧，所以需要将该参数置为true，不影响正常回调
      ret = DecodeTempoA2(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                          maxAngle, udpData->frameIndex, udpData->framed);
    } else if (udpData->m_length == 1348)
      ret = DecodeFocusB2(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                          maxAngle, udpData->frameIndex, udpData->framed);
    else if (udpData->m_length == 1212)
      ret = DecodeFocusB2Calib(udpData->m_szData, &udpData->t_sec,
                               &udpData->t_usec, maxAngle, udpData->frameIndex,
                               udpData->framed);
    else if (udpData->m_length == 1024) {
      DecodeDIFData_FocusB2(udpData->m_szData);
      PointCloudCallback(udpData->framed, udpData->frameIndex);
    } else
      ON_DECODE_EXCEPTION
    break;
  case LT_FocusB2_64:
    if (udpData->m_length == 1212) {
      ret =
          DecodeFocusB2_64(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                           maxAngle, udpData->frameIndex, udpData->framed);
    } else if (udpData->m_length == 1024) {
      DecodeDIFData_FocusB2(udpData->m_szData);
      PointCloudCallback(udpData->framed, udpData->frameIndex);
    } else
      ON_DECODE_EXCEPTION
    break;
  case LT_FocusT:
    if (udpData->m_length == 1020) { // 双回波
      ret = DecodeFocusT(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                         maxAngle, udpData->frameIndex, udpData->framed);
    } else if (udpData->m_length == 1236) // 单回波
      ret = DecodeFocusT_2(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                           maxAngle, udpData->frameIndex, udpData->framed);
    else if (udpData->m_length == 1212) // 标定
      ret = DecodeFocusTCalib(udpData->m_szData, &udpData->t_sec,
                              &udpData->t_usec, maxAngle, udpData->frameIndex,
                              udpData->framed);
    else if (udpData->m_length == 1024) { // DIF
      DecodeDIFData_FocusT(udpData->m_szData);
      PointCloudCallback(udpData->framed, udpData->frameIndex);
    } else
      ON_DECODE_EXCEPTION
    break;
  case LT_Scope128:
    if (udpData->m_length == 1348)
      ret = DecodeScope128(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                           maxAngle, udpData->frameIndex, udpData->framed);
    else if (udpData->m_length == 1236)
      ret =
          DecodeScope128_2(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                           maxAngle, udpData->frameIndex, udpData->framed);
    else if (udpData->m_length == 1212)
      ret = DecodeScope128Calib(udpData->m_szData, &udpData->t_sec,
                                &udpData->t_usec, maxAngle, udpData->frameIndex,
                                udpData->framed);
    else if (udpData->m_length == 1024) {
      DecodeDIFData_Scope128(udpData->m_szData);
      if (_lidarTypeFromDIF != 8)
        ON_DECODE_EXCEPTION
      PointCloudCallback(udpData->framed, udpData->frameIndex);
    } else
      ON_DECODE_EXCEPTION
    break;
  case LT_Scope128F:
    if (udpData->m_length == 1348)
      ret =
          DecodeScope128F(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                          maxAngle, udpData->frameIndex, udpData->framed);
    else if (udpData->m_length == 1212)
      ret = DecodeScope128FCalib(udpData->m_szData, &udpData->t_sec,
                                 &udpData->t_usec, maxAngle,
                                 udpData->frameIndex, udpData->framed);
    else if (udpData->m_length == 1024) {
      DecodeDIFData_Scope256(udpData->m_szData);
      PointCloudCallback(udpData->framed, udpData->frameIndex);
    } else
      ON_DECODE_EXCEPTION
    break;
  case LT_TW360:
    if (udpData->m_length == 1348)
      ret = DecodeTW360(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                        maxAngle, udpData->frameIndex, udpData->framed);
    else if (udpData->m_length == 1236)
      ret = DecodeTW360_2(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                          maxAngle, udpData->frameIndex, udpData->framed);
    else if (udpData->m_length == 1212)
      ret =
          DecodeTW360Calib(udpData->m_szData, &udpData->t_sec, &udpData->t_usec,
                           maxAngle, udpData->frameIndex, udpData->framed);
    else if (udpData->m_length == 1024) {
      DecodeDIFData_TW360(udpData->m_szData);
      PointCloudCallback(udpData->framed, udpData->frameIndex);
    } else if (udpData->m_length == 36) {
      DecodeIMUData_TW360(udpData->m_szData, &udpData->t_sec, &udpData->t_usec);
      PointCloudCallback(udpData->framed, udpData->frameIndex);
    } else
      ON_DECODE_EXCEPTION
    break;
  default:
    ON_DECODE_EXCEPTION
    break;
  }

  return ret;
}

bool LidarDevice::PointCloudCallback(TWPointCloud::Points &points,
                                     float *maxAngle, int mirror, int leftRight,
                                     int value) {
  bool onFrame = false;
  if (_isLidarReady) {
    if (_lidarAlgo && !maxAngle &&
        points[0].angle != 0) // 空包的水平角度是0，不进入算法
      _lidarAlgo->Process(points);

    _validPointsTimes++;
    _udp_count++;
    int pointSize = points.size();
    for (int i = 0; i < pointSize; i++) {
      TWPoint &point = points[i];
      //  组帧条件
      if ((point.angle < m_startAngle &&
           (value == -1 ? true : IsEqualityFloat3(0.0, point.x)) &&
           (mirror == -1 ? true : mirror == point.mirror) &&
           (leftRight == -1 ? true : leftRight == point.left_right)) &&
          _pointCloudPtr->size() > 50 && _validPointsTimes > 50) {
        _has_framed_angle = true;
        _has_framed_cycle_count = true;
      }
      if (_pre_cycle_count != point.cycle_count &&
          _pointCloudPtr->size() > 50 && _validPointsTimes > 50) {
        if (!_has_framed_angle)
          _has_framed_cycle_count = true;
        else {
          _has_framed_angle = false;
        }
      }
      _pre_cycle_count = point.cycle_count;

      if (_has_framed_cycle_count && !_enable_time_windows) {
        _pointCloudPtr->height = 1;
        _pointCloudPtr->width = _pointCloudPtr->size();

        // 第一个点的时间戳
        if (_stampType == "first_point")
          _pointCloudPtr->header.stamp = _first_stamp;
        else
          _pointCloudPtr->header.stamp = _stamp;
        // 输出到控制台时间戳信息：
        //  std::string timeStr_first = std::to_string(_first_stamp);
        //  std::string timeStr_last = std::to_string(_stamp);

        //  std::string timeStr =
        //  std::to_string(_pointCloudPtr->header.stamp);

        // pcap包模式下，每次用下述语句来输出时间，包括雷达首点、雷达末点、系统首点、系统末点四种情况：
        // std::cout<<"Received  point cloud at time: "<<
        // timeStr.c_str()<<std::endl;

        // 实时模式下，每次用下述两条语句来输出时间，包括雷达（首、末点）和系统（首、末点）两种情况：
        //      std::cout << "Received first point cloud at time:"
        //                << timeStr_first.c_str() << std::endl;
        //      std::cout << "Received last  point cloud at time:"
        //                << timeStr_last.c_str() << std::endl;

        _pointCloudPtr->header.seq = _curIndex;

        if (!maxAngle) {
          _lidarObserver->OnPointCloud(_lidarInfo, *_pointCloudPtr);
        }
        _callbackdone.store(true);
        _pointCloudPtr->clear();
        onFrame = true; // 帧检测，是否检测到新的一帧的点云
        _has_framed_cycle_count = false;

        _validPointsTimes = 0;

        logger.LogTestTime(__FILE__, __LINE__, PRETTY_FUNCTION,
                           "$decodetimeinframe,%f,%d", _frame_time / 1000.0,
                           _udp_count);
        // std::cout<<"_frame_time:"<<_frame_time<<std::endl;
        // std::cout<<"******************************************"<<std::endl;
        _frame_time = 0;
        _udp_count = 0;
      }

      if (_enable_time_windows) {
        uint64_t t_msec =
            (uint64_t)(point.t_sec) * 1000 + point.t_usec / 1000; // 转换为ms
        _now_frame_time = t_msec / _frame_interval;
        if (_last_frame_time == 0) // 应记录雷达运行后第一个包的百毫秒位
        {
          _last_frame_time = _now_frame_time;
        }
        if (_now_frame_time > _last_frame_time ||
            _udp_count > MAX_UDP_COUNT) // 分帧
        {
          if (_udp_count > MAX_UDP_COUNT) {
            _lidarObserver->OnException(
                _lidarInfo,
                Exception(ERR_UDP_COUNT,
                          std::string("Timestamp abnormality caused the system "
                                      "to detect over ") +
                              std::to_string(MAX_UDP_COUNT) +
                              std::string(" packets in one frame!"))); // 报故障
          }
          _last_frame_time = _now_frame_time;
          _udp_count = 0;

          _pointCloudPtr->height = 1;
          _pointCloudPtr->width = _pointCloudPtr->size();

          // 第一个点的时间戳
          if (_stampType == "first_point")
            _pointCloudPtr->header.stamp = _first_stamp;
          else
            _pointCloudPtr->header.stamp = _stamp;

          _pointCloudPtr->header.seq = _curIndex;

          if (!maxAngle) {
            _lidarObserver->OnPointCloud(_lidarInfo, *_pointCloudPtr);
          }
          _callbackdone.store(true);
          _pointCloudPtr->clear();
          onFrame = true; // 帧检测，是否检测到新的一帧的点云
        }
      }

      if (point.angle < m_startAngle ||
          point.angle > m_endAngle) // 空包的水平角度=0，所以会被跳过
        continue;
      if (_enable_time_windows &&
          (_now_frame_time <
           _last_frame_time)) // 防止udp包时间不连续，如果该udp包时间向前跳，则不保留
        continue;

      // 第一个Fov范围内的点的时间戳
      if (_pointCloudPtr->size() == 0) {
        _first_stamp = (uint64_t)(point.t_sec) * 1000 * 1000 + point.t_usec;
      }

      // 调整位置：最后一个Fov范围内的点时间戳
      _stamp =
          (uint64_t)(point.t_sec) * 1000 * 1000 +
          point
              .t_usec; // 每次更新时间戳，这样当满足组帧条件时，使用的就是最后一个点云的时间戳

      if (point.distance < _dis_min || point.distance >= _dis_max)
        continue;

      CalculateRotateAllPointCloud(point);

      UserPoint basic_point;
      setX(basic_point, static_cast<float>(point.x));
      setY(basic_point, static_cast<float>(point.y));
      setZ(basic_point, static_cast<float>(point.z));

      setIntensity(basic_point, static_cast<float>(point.intensity));
      setDistance(basic_point, static_cast<float>(point.distance));
      setChannel(basic_point, point.channel);
      setAngle(basic_point, static_cast<float>(point.angle));
      setPulse(basic_point, static_cast<float>(point.pulse));

      setEcho(basic_point, point.echo);
      setMirror(basic_point, point.mirror);
      setLeftRight(basic_point, point.left_right);
      setConfidence(basic_point, point.confidence);
      setBlock(basic_point, point.block);
      setT_sec(basic_point, point.t_sec);
      setT_usec(basic_point, point.t_usec);
      setAPDTemp(basic_point, point.apd_temp);
      setPulseCodeInterval(basic_point, point.pulseCodeInterval);

      logger.LogTestPoint(__FILE__, __LINE__, PRETTY_FUNCTION,
                          "$xyzall,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%u,%d",
                          point.x, point.y, point.z, point.intensity,
                          point.distance, point.channel, point.angle,
                          point.pulse, point.echo, point.mirror,
                          point.left_right, point.t_sec, point.t_usec);
      _pointCloudPtr->push_back(std::move(basic_point));
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                        end_time - start_time)
                        .count();
    _frame_time += duration;
  }

  return onFrame;
}

void LidarDevice::PointCloudCallback(TWPointCloud::Points &points,
                                     bool callback, uint64_t frameIndex) {
  if (_isLidarReady) {
    if (_lidarAlgo)
      _lidarAlgo->Process(points);
    _udp_count++;

    int pointSize = points.size();

    for (int i = 0; i < pointSize; i++) {
      TWPoint &point = points[i];

      if (point.angle < m_startAngle || point.angle > m_endAngle)
        continue;

      if (_pointCloudPtr->size() == 0) {
        _first_stamp = (uint64_t)(point.t_sec) * 1000 * 1000 + point.t_usec;
      }
      // 调整位置：最后一个Fov范围内的点时间戳
      _stamp =
          (uint64_t)(point.t_sec) * 1000 * 1000 +
          point
              .t_usec; // 每次更新时间戳，这样当满足组帧条件时，使用的就是最后一个点云的时间戳

      if (point.distance < _dis_min || point.distance >= _dis_max)
        continue;

      CalculateRotateAllPointCloud(point);
      UserPoint basic_point;
      setX(basic_point, static_cast<float>(point.x));
      setY(basic_point, static_cast<float>(point.y));
      setZ(basic_point, static_cast<float>(point.z));

      setIntensity(basic_point, static_cast<float>(point.intensity));
      setDistance(basic_point, static_cast<float>(point.distance));
      setChannel(basic_point, point.channel);
      setAngle(basic_point, static_cast<float>(point.angle));
      setPulse(basic_point, static_cast<float>(point.pulse));

      setEcho(basic_point, point.echo);
      setMirror(basic_point, point.mirror);
      setLeftRight(basic_point, point.left_right);
      setConfidence(basic_point, point.confidence);
      setBlock(basic_point, point.block);
      setT_sec(basic_point, point.t_sec);
      setT_usec(basic_point, point.t_usec);
      setAPDTemp(basic_point, point.apd_temp);
      setPulseCodeInterval(basic_point, point.pulseCodeInterval);

      logger.LogTestPoint(__FILE__, __LINE__, PRETTY_FUNCTION,
                          "$xyzall,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%u,%d",
                          point.x, point.y, point.z, point.intensity,
                          point.distance, point.channel, point.angle,
                          point.pulse, point.echo, point.mirror,
                          point.left_right, point.t_sec, point.t_usec);

      _pointCloudPtr->push_back(std::move(basic_point));
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    // auto end_time_us =
    // std::chrono::time_point_cast<std::chrono::microseconds>(end_time).time_since_epoch().count();
    // std::cout << "End time (microseconds): " << end_time_us << std::endl;
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                        end_time - start_time)
                        .count();
    // std::cout<<"duration:"<<duration<<std::endl;
    _frame_time += duration;

    if (callback) {
      _pointCloudPtr->height = 1;
      _pointCloudPtr->width = _pointCloudPtr->size();

      if (_stampType == "first_point")
        _pointCloudPtr->header.stamp = _first_stamp;
      else
        _pointCloudPtr->header.stamp = _stamp;

      _pointCloudPtr->header.seq = frameIndex;

      logger.LogTestTime(__FILE__, __LINE__, PRETTY_FUNCTION,
                         "$decodetimeinframe,%f,%d", _frame_time / 1000.0,
                         _udp_count);
      // std::cout << "_frame_time:" << _frame_time << std::endl;
      _frame_time = 0;
      _udp_count = 0;
      _lidarObserver->OnPointCloud(_lidarInfo, *_pointCloudPtr);
      _pointCloudPtr->clear();
      _callbackdone.store(true);
    }
  }
}

void LidarDevice::PointCloudCallback(bool callback, uint64_t frameIndex) {

  if (callback && _isLidarReady) {
    _pointCloudPtr->height = 1;
    _pointCloudPtr->width = _pointCloudPtr->size();

    if (_stampType == "first_point")
      _pointCloudPtr->header.stamp = _first_stamp;
    else
      _pointCloudPtr->header.stamp = _stamp;

    _pointCloudPtr->header.seq = frameIndex;

    _lidarObserver->OnPointCloud(_lidarInfo, *_pointCloudPtr);
    _pointCloudPtr->clear();
    _callbackdone.store(true);
  }
}

bool LidarDevice::SeekNextValidPacketOffset(std::ifstream &inStream) {
  const uint8_t syncBytes[4] = {0x77, 0x88, 0x99, 0xaa};
  uint8_t c;
  int curIndex = 0;
  bool find = false;

  while (_run || _parsing) {
    if (inStream.read((char *)&c, sizeof(uint8_t)).gcount() != sizeof(uint8_t))
      break;

    if (c == syncBytes[curIndex++]) {
      if (curIndex == 4) {
        find = true;
        break;
      }
    } else {
      curIndex = 0;
      if (c == syncBytes[curIndex])
        curIndex++;
    }
  }

  return find;
}

bool LidarDevice::IsValidFrame(const char frame[], int length) {
  if (_lidarType == LT_Scope128 || _lidarType == LT_Scope256 ||
      _lidarType == LT_Scope256_SmallBlind || _lidarType == LT_Scope128H ||
      _lidarType == LT_TempoA4 || _lidarType == LT_Tensor48 ||
      _lidarType == LT_Duetto) {
    if ((int)(frame[length - 1] & 0xFF) != 0xaa ||
        (int)(frame[length - 2] & 0xFF) != 0x99 ||
        (int)(frame[length - 3] & 0xFF) != 0x88 ||
        (int)(frame[length - 4] & 0xFF) != 0x77)
      return false;
  }

  return true;
}

bool LidarDevice::IsBackPoint(const TWPoint &point) {
  if (point.left_right) {
    if ((point.angle >= m_positive_back_startAngle) &&
        (point.angle <= m_positive_back_endAngle))
      return true;
  } else {
    if ((point.angle >= m_negtive_back_startAngle) &&
        (point.angle <= m_negtive_back_endAngle))
      return true;
  }

  return false;
}

void LidarDevice::UseDecodeTensor16(const char *udpData,
                                    TWPointCloud::Points &points,
                                    unsigned int *sec, unsigned int *usec,
                                    float *maxAngle) {
  for (int blocks_num = 0; blocks_num < 20; blocks_num++) {
    int offset = blocks_num * 72;

    unsigned int HextoAngle =
        FourHexToInt(udpData[offset + 64], udpData[offset + 65],
                     udpData[offset + 66], udpData[offset + 67]);
    double horizontalAngle = HextoAngle * 0.00001;

    unsigned char hexBlockMicrosecond = udpData[offset + 68];
    hexBlockMicrosecond = hexBlockMicrosecond & 0x7F;
    unsigned int blockMicrosecond =
        FourHexToInt(hexBlockMicrosecond, udpData[offset + 69],
                     udpData[offset + 70], udpData[offset + 71]);

    unsigned char hexMirror = udpData[offset + 68];
    hexMirror = hexMirror >> 7;
    unsigned short mirror = hexMirror;

    for (int seq = 0; seq < 16; seq++) {
      unsigned short hexL = TwoHextoInt(udpData[offset + seq * 4 + 0],
                                        udpData[offset + seq * 4 + 1]);
      unsigned short hexPulse = TwoHextoInt(udpData[offset + seq * 4 + 2],
                                            udpData[offset + seq * 4 + 3]);

      double L = hexL * m_calSimple;
      int intensity =
          (hexPulse * m_calPulse) / TensorPulseMapValue * 255.0 + 0.5;
      intensity = intensity > 255 ? 255 : intensity;

      double cos_hA = cos(horizontalAngle * m_calRA);
      double sin_hA = sin(horizontalAngle * m_calRA);
      double vA = m_verticalChannelsAngle_Tensor16[seq];
      double cos_vA_RA = cos(vA * m_calRA);

      TWPoint basic_point;
      basic_point.x = L * cos_vA_RA * cos_hA;
      basic_point.y = L * cos_vA_RA * sin_hA;
      basic_point.z = L * sin(vA * m_calRA);

      basic_point.intensity = intensity;
      basic_point.distance = L;
      basic_point.channel = seq + 1;
      basic_point.angle = horizontalAngle;
      basic_point.pulse = hexPulse * m_calPulse;
      basic_point.mirror = mirror;
      basic_point.echo = 1;
      if (_is_lidar_time) {
        basic_point.t_sec = 0;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }

      points.push_back(std::move(basic_point));
    }
  }
}

void LidarDevice::UseDecodeTensor32(const char *udpData,
                                    TWPointCloud::Points &points,
                                    unsigned int *sec, unsigned int *usec,
                                    float *maxAngle) {
  for (int blocks_num = 0; blocks_num < 20; blocks_num++) {
    int offset = blocks_num * 72;

    unsigned int HextoAngle =
        FourHexToInt(udpData[offset + 64], udpData[offset + 65],
                     udpData[offset + 66], udpData[offset + 67]);
    double horizontalAngle = HextoAngle * 0.00001;

    unsigned char hexBlockMicrosecond = udpData[offset + 68];
    hexBlockMicrosecond = hexBlockMicrosecond & 0x7F;
    unsigned int blockMicrosecond =
        FourHexToInt(hexBlockMicrosecond, udpData[offset + 69],
                     udpData[offset + 70], udpData[offset + 71]);

    unsigned char hexMirror = udpData[offset + 68];
    hexMirror = hexMirror >> 7;
    unsigned short mirror = hexMirror;

    horizontalAngle += m_MirrorHorAngleOffset[mirror];

    double hA = 0.5 * horizontalAngle * m_calRA;
    double hA_sin = sin(hA);
    double hA_cos = cos(hA);

    if (m_skewing_tsp_Angle_Correct[mirror] !=
        (m_skewing_tsp_Angle[mirror] + m_MirrorVerAngleOffset[mirror])) {

      m_skewing_tsp_Angle_Correct[mirror] =
          m_skewing_tsp_Angle[mirror] + m_MirrorVerAngleOffset[mirror];

      m_skewing_cos_tsp[mirror] =
          cos(m_skewing_tsp_Angle_Correct[mirror] * m_calRA);

      m_skewing_sin_tsp[mirror] =
          sin(m_skewing_tsp_Angle_Correct[mirror] * m_calRA);
    }

    double x_cal_1 = 2.0 * m_skewing_cos_tsp[mirror] *
                         m_skewing_cos_tsp[mirror] * hA_cos * hA_cos -
                     1;
    double x_cal_2 =
        2.0 * m_skewing_sin_tsp[mirror] * m_skewing_cos_tsp[mirror] * hA_cos;

    double y_cal_1 = m_skewing_cos_tsp[mirror] * m_skewing_cos_tsp[mirror] *
                     2.0 * hA_sin * hA_cos;
    double y_cal_2 =
        2.0 * m_skewing_sin_tsp[mirror] * m_skewing_cos_tsp[mirror] * hA_sin;

    double z_cal_1 =
        2.0 * m_skewing_sin_tsp[mirror] * m_skewing_cos_tsp[mirror] * hA_cos;
    double z_cal_2 =
        2.0 * m_skewing_sin_tsp[mirror] * m_skewing_sin_tsp[mirror] - 1;

    for (int seq = 0; seq < 16; seq++) {
      unsigned short hexL = TwoHextoInt(udpData[offset + seq * 4 + 0],
                                        udpData[offset + seq * 4 + 1]);
      unsigned short hexPulse = TwoHextoInt(udpData[offset + seq * 4 + 2],
                                            udpData[offset + seq * 4 + 3]);

      double L = hexL * m_calSimple;
      double pulse = hexPulse * m_calPulse;
      int intensity = pulse / TensorPulseMapValue * 255.0 + 0.5;
      intensity = intensity > 255 ? 255 : intensity;

      double cos_vA_RA = m_verticalChannelAngle16_cos_vA_RA[seq];
      double sin_vA_RA = m_verticalChannelAngle16_sin_vA_RA[seq];

      if (_echoNum & Echo1) {
        TWPoint basic_point;
        basic_point.x = L * (cos_vA_RA * x_cal_1 + sin_vA_RA * x_cal_2);
        basic_point.y = L * (cos_vA_RA * y_cal_1 + sin_vA_RA * y_cal_2);
        basic_point.z = -L * (cos_vA_RA * z_cal_1 + sin_vA_RA * z_cal_2);

        basic_point.distance = L;
        basic_point.channel = seq + 1;
        basic_point.angle = horizontalAngle;
        basic_point.pulse = pulse;
        basic_point.intensity = intensity;
        basic_point.mirror = mirror;
        basic_point.echo = 1;
        if (_is_lidar_time) {
          basic_point.t_sec = 0;
          basic_point.t_usec = blockMicrosecond;
        } else {
          basic_point.t_sec = *sec;
          basic_point.t_usec = *usec;
        }

        points.push_back(std::move(basic_point));
      }
    }
  }
}

void LidarDevice::UseDecodeScope192(const char *udpData,
                                    TWPointCloud::Points &points,
                                    unsigned int *sec, unsigned int *usec,
                                    float *maxAngle) {
  double horizontalAngle = 0;
  // face id
  unsigned short mirror = 0;
  double x_cal_1 = 0.0;
  double x_cal_2 = 0.0;
  double y_cal_1 = 0.0;
  double y_cal_2 = 0.0;
  double z_cal_1 = 0.0;
  double z_cal_2 = 0.0;

  for (int blocks_num = 0; blocks_num < 8; blocks_num++) {
    int offset = blocks_num * 140;
    if (0 == blocks_num || 4 == blocks_num) {
      // horizontal angle index: 128-131
      int HextoAngle =
          FourHexToInt(udpData[offset + 128], udpData[offset + 129],
                       udpData[offset + 130], udpData[offset + 131]);
      horizontalAngle = HextoAngle * 0.00001;

      unsigned char hexMirror = udpData[offset + 136];
      hexMirror = hexMirror << 2;
      mirror = hexMirror >> 6;

      horizontalAngle += m_MirrorHorAngleOffset[mirror];

      double hA = 0.5 * (horizontalAngle + 10.0) * m_calRA;
      double hA_sin = sin(hA);
      double hA_cos = cos(hA);

      double offsetAngle = 0;
      unsigned char hexACount = udpData[offset + 136];
      hexACount = hexACount << 4;
      unsigned short uACount = hexACount >> 4;
      offsetAngle = uACount * 0.04 - 0.3;

      // calculate
      if (mirror < 3 && fabs(m_skewing_scope_Angle_Correct[mirror] -
                             (m_skewing_scope_Angle[mirror] + offsetAngle +
                              m_MirrorVerAngleOffset[mirror])) > 0.001) {
        m_skewing_scope_Angle_Correct[mirror] = m_skewing_scope_Angle[mirror] +
                                                offsetAngle +
                                                m_MirrorVerAngleOffset[mirror];
        m_skewing_sin_scope[mirror] =
            sin(-1.0 * m_skewing_scope_Angle_Correct[mirror] * m_calRA);
        m_skewing_cos_scope[mirror] =
            cos(-1.0 * m_skewing_scope_Angle_Correct[mirror] * m_calRA);
      }

      x_cal_1 = 2.0 * m_skewing_cos_scope[mirror] *
                    m_skewing_cos_scope[mirror] * hA_cos * hA_cos -
                1;
      x_cal_2 = 2.0 * m_skewing_sin_scope[mirror] *
                m_skewing_cos_scope[mirror] * hA_cos;

      y_cal_1 = 2.0 * m_skewing_cos_scope[mirror] *
                m_skewing_cos_scope[mirror] * hA_sin * hA_cos;
      y_cal_2 = 2.0 * m_skewing_sin_scope[mirror] *
                m_skewing_cos_scope[mirror] * hA_sin;

      z_cal_1 = 2.0 * m_skewing_sin_scope[mirror] *
                m_skewing_cos_scope[mirror] * hA_cos;
      z_cal_2 =
          2.0 * m_skewing_sin_scope[mirror] * m_skewing_sin_scope[mirror] - 1;
    }

    unsigned int blockSecond =
        FourHexToInt(udpData[offset + 132], udpData[offset + 133],
                     udpData[offset + 134], udpData[offset + 135]);
    unsigned char hexBlockMicrosecond = udpData[offset + 137];
    hexBlockMicrosecond = hexBlockMicrosecond & 0x0F;
    unsigned int blockMicrosecond =
        FourHexToInt(0x00, hexBlockMicrosecond, udpData[offset + 138],
                     udpData[offset + 139]);

    // separate index
    unsigned char hexSepIndex = udpData[offset + 136];
    unsigned short sepIndex = hexSepIndex >> 6;

    for (int seq = 0; seq < 16; seq++) {
      double hexToInt1 = TwoHextoInt(udpData[offset + seq * 8 + 0],
                                     udpData[offset + seq * 8 + 1]);
      double hexPulse1 = TwoHextoInt(udpData[offset + seq * 8 + 2],
                                     udpData[offset + seq * 8 + 3]);
      double L_1 = hexToInt1 * m_calSimple;
      double pulse_1 = hexPulse1 * m_calPulse;
      int intensity1 = pulse_1 / ScopePulseMapValue * 255.0 + 0.5;
      intensity1 = intensity1 > 255 ? 255 : intensity1;

      double hexToInt2 = TwoHextoInt(udpData[offset + seq * 8 + 4],
                                     udpData[offset + seq * 8 + 5]);
      double hexPulse2 = TwoHextoInt(udpData[offset + seq * 8 + 6],
                                     udpData[offset + seq * 8 + 7]);
      double L_2 = hexToInt2 * m_calSimple;
      double pulse_2 = hexPulse2 * m_calPulse;
      int intensity2 = pulse_2 / ScopePulseMapValue * 255.0 + 0.5;
      intensity2 = intensity2 > 255 ? 255 : intensity2;

      int channel =
          65 - (16 * (blocks_num >= 4 ? blocks_num - 4 : blocks_num) + seq + 1);

      double cos_vA_RA = m_verticalChannelAngle_Scope64_cos_vA_RA[channel - 1];
      double sin_vA_RA = m_verticalChannelAngle_Scope64_sin_vA_RA[channel - 1];

      TWPoint basic_point;
      basic_point.angle = horizontalAngle;
      basic_point.mirror = mirror;
      basic_point.channel = channel;
      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }

      if (_echoNum & Echo1) {
        double x_tmp = L_1 * (cos_vA_RA * x_cal_1 + sin_vA_RA * x_cal_2);
        double y_tmp = L_1 * (cos_vA_RA * y_cal_1 + sin_vA_RA * y_cal_2);
        double z_tmp = -L_1 * (cos_vA_RA * z_cal_1 + sin_vA_RA * z_cal_2);
        basic_point.x = x_tmp * m_rotate_scope_cos - y_tmp * m_rotate_scope_sin;
        basic_point.y = x_tmp * m_rotate_scope_sin + y_tmp * m_rotate_scope_cos;
        basic_point.z = z_tmp;

        basic_point.echo = 1;
        basic_point.distance = L_1;
        basic_point.pulse = pulse_1;
        basic_point.intensity = intensity1;
        points.push_back(std::move(basic_point));
      }

      if (_echoNum & Echo2) {
        double x_tmp = L_2 * (cos_vA_RA * x_cal_1 + sin_vA_RA * x_cal_2);
        double y_tmp = L_2 * (cos_vA_RA * y_cal_1 + sin_vA_RA * y_cal_2);
        double z_tmp = -L_2 * (cos_vA_RA * z_cal_1 + sin_vA_RA * z_cal_2);
        basic_point.x = x_tmp * m_rotate_scope_cos - y_tmp * m_rotate_scope_sin;
        basic_point.y = x_tmp * m_rotate_scope_sin + y_tmp * m_rotate_scope_cos;
        basic_point.z = z_tmp;

        basic_point.echo = 2;
        basic_point.distance = L_2;
        basic_point.pulse = pulse_2;
        basic_point.intensity = intensity2;
        points.push_back(std::move(basic_point));
      }
    }
  }
}

void LidarDevice::UseDecodeTempoA1(const char *udpData,
                                   TWPointCloud::Points &points,
                                   unsigned int *sec, unsigned int *usec,
                                   float *maxAngle) {
  double horizontalAngle = 0;
  // face id
  unsigned short mirror = 0;
  double x_cal_1 = 0.0;
  double x_cal_2 = 0.0;
  double y_cal_1 = 0.0;
  double y_cal_2 = 0.0;
  double z_cal_1 = 0.0;
  double z_cal_2 = 0.0;

  for (int blocks_num = 0; blocks_num < 8; blocks_num++) {
    int offset = blocks_num * 140;
    if (0 == blocks_num || 4 == blocks_num) {
      // horizontal angle index: 128-131
      int HextoAngle =
          FourHexToInt(udpData[offset + 128], udpData[offset + 129],
                       udpData[offset + 130], udpData[offset + 131]);
      horizontalAngle = HextoAngle * 0.00001;

      unsigned char hexMirror = udpData[offset + 136];
      hexMirror = hexMirror << 2;
      mirror = hexMirror >> 6;

      horizontalAngle += m_MirrorHorAngleOffset[mirror];

      // offset angle m_skewing_scopeMiniA2_angle
      double offsetAngle = 0;
      unsigned char hexACount = udpData[offset + 136];
      hexACount = hexACount << 4;
      unsigned short uACount = hexACount >> 4;
      offsetAngle = uACount * 0.04 - 0.3;

      // calculate
      if (mirror < 3 &&
          fabs(m_skewing_scopeMiniA2_Angle_Correct[mirror] -
               (m_skewing_scopeMiniA2_Angle[mirror] + offsetAngle +
                m_MirrorVerAngleOffset[mirror])) > 0.001) {
        m_skewing_scopeMiniA2_Angle_Correct[mirror] =
            m_skewing_scopeMiniA2_Angle[mirror] + offsetAngle +
            m_MirrorVerAngleOffset[mirror];
        m_skewing_sin_scopeMiniA2_192[mirror] =
            sin(m_skewing_scopeMiniA2_Angle_Correct[mirror] * m_calRA);
        m_skewing_cos_scopeMiniA2_192[mirror] =
            cos(m_skewing_scopeMiniA2_Angle_Correct[mirror] * m_calRA);
      }

      double hA = 0.5 * (horizontalAngle)*m_calRA;
      double hA_sin = sin(hA);
      double hA_cos = cos(hA);

      x_cal_1 = 2.0 * m_skewing_cos_scopeMiniA2_192[mirror] *
                    m_skewing_cos_scopeMiniA2_192[mirror] * hA_cos * hA_cos -
                1;
      x_cal_2 = 2.0 * m_skewing_sin_scopeMiniA2_192[mirror] *
                m_skewing_cos_scopeMiniA2_192[mirror] * hA_cos;

      y_cal_1 = 2.0 * m_skewing_cos_scopeMiniA2_192[mirror] *
                m_skewing_cos_scopeMiniA2_192[mirror] * hA_sin * hA_cos;
      y_cal_2 = 2.0 * m_skewing_sin_scopeMiniA2_192[mirror] *
                m_skewing_cos_scopeMiniA2_192[mirror] * hA_sin;

      z_cal_1 = 2.0 * m_skewing_sin_scopeMiniA2_192[mirror] *
                m_skewing_cos_scopeMiniA2_192[mirror] * hA_cos;
      z_cal_2 = 2.0 * m_skewing_sin_scopeMiniA2_192[mirror] *
                    m_skewing_sin_scopeMiniA2_192[mirror] -
                1;
    }

    unsigned int blockSecond =
        FourHexToInt(udpData[offset + 132], udpData[offset + 133],
                     udpData[offset + 134], udpData[offset + 135]);
    unsigned char hexBlockMicrosecond = udpData[offset + 137];
    hexBlockMicrosecond = hexBlockMicrosecond & 0x0F;
    unsigned int blockMicrosecond =
        FourHexToInt(0x00, hexBlockMicrosecond, udpData[offset + 138],
                     udpData[offset + 139]);

    // separate index
    unsigned char hexSepIndex = udpData[offset + 136];
    unsigned short sepIndex = hexSepIndex >> 6;

    for (int seq = 0; seq < 16; seq++) {
      double hexToInt1 = TwoHextoInt(udpData[offset + seq * 8 + 0],
                                     udpData[offset + seq * 8 + 1]);
      double hexPulse1 = TwoHextoInt(udpData[offset + seq * 8 + 2],
                                     udpData[offset + seq * 8 + 3]);
      double L_1 = hexToInt1 * m_calSimple;

      double pulse_1 = hexPulse1 * m_calPulseFPGA;

      double intensity1 = pulse_1 / ScopePulseMapValue * 255.0 + 0.5;
      intensity1 = intensity1 > 255 ? 255 : intensity1;

      double hexToInt2 = TwoHextoInt(udpData[offset + seq * 8 + 4],
                                     udpData[offset + seq * 8 + 5]);
      double hexPulse2 = TwoHextoInt(udpData[offset + seq * 8 + 6],
                                     udpData[offset + seq * 8 + 7]);
      double L_2 = hexToInt2 * m_calSimple;
      double pulse_2 = hexPulse2 * m_calPulse;
      int intensity2 = pulse_2 / ScopePulseMapValue * 255.0 + 0.5;
      intensity2 = intensity2 > 255 ? 255 : intensity2;

      int channel =
          65 - (16 * (blocks_num >= 4 ? blocks_num - 4 : blocks_num) + seq + 1);

      double cos_vA_RA =
          m_verticalChannelAngle_ScopeMiniA2_cos_vA_RA[channel - 1];
      double sin_vA_RA =
          m_verticalChannelAngle_ScopeMiniA2_sin_vA_RA[channel - 1];

      TWPoint basic_point;
      basic_point.angle = horizontalAngle;
      basic_point.mirror = mirror;
      basic_point.channel = channel;

      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }

      if (_echoNum & Echo1) {
        basic_point.x = L_1 * (cos_vA_RA * x_cal_1 + sin_vA_RA * x_cal_2);
        basic_point.y = L_1 * (cos_vA_RA * y_cal_1 + sin_vA_RA * y_cal_2);
        basic_point.z = -L_1 * (cos_vA_RA * z_cal_1 + sin_vA_RA * z_cal_2);

        basic_point.echo = 1;
        basic_point.distance = L_1;
        basic_point.pulse = pulse_1;
        basic_point.intensity = intensity1;
        points.push_back(std::move(basic_point));
      }

      if (_echoNum & Echo2) {
        basic_point.x = L_2 * (cos_vA_RA * x_cal_1 + sin_vA_RA * x_cal_2);
        basic_point.y = L_2 * (cos_vA_RA * y_cal_1 + sin_vA_RA * y_cal_2);
        basic_point.z = -L_2 * (cos_vA_RA * z_cal_1 + sin_vA_RA * z_cal_2);

        basic_point.echo = 2;
        basic_point.distance = L_2;
        basic_point.pulse = pulse_2;
        basic_point.intensity = intensity2;
        points.push_back(std::move(basic_point));
      }
    }
  }
}

void LidarDevice::UseDecodeTempoA2(const char *udpData,
                                   TWPointCloud::Points &points,
                                   unsigned int *sec, unsigned int *usec,
                                   float *maxAngle) {
  double horizontalAngle = 0;
  // face id
  unsigned short mirror = 0;
  double x_cal_1 = 0.0;
  double x_cal_2 = 0.0;
  double y_cal_1 = 0.0;
  double y_cal_2 = 0.0;
  double z_cal_1 = 0.0;
  double z_cal_2 = 0.0;

  for (int blocks_num = 0; blocks_num < 8; blocks_num++) {
    int offset = blocks_num * 140;
    if (0 == blocks_num || 4 == blocks_num) {
      // horizontal angle index: 128-131
      int HextoAngle =
          FourHexToInt(udpData[offset + 128], udpData[offset + 129],
                       udpData[offset + 130], udpData[offset + 131]);
      horizontalAngle = HextoAngle * 0.00001;

      unsigned char hexMirror = udpData[offset + 136];
      hexMirror = hexMirror << 2;
      mirror = hexMirror >> 6;

      horizontalAngle += m_MirrorHorAngleOffset[mirror];

      // offset angle m_skewing_scopeMiniA2_angle
      double offsetAngle = 0;
      unsigned char hexACount = udpData[offset + 136];
      hexACount = hexACount << 4;
      unsigned short uACount = hexACount >> 4;
      offsetAngle = uACount * 0.04 - 0.3;

      // calculate
      if (mirror < 3 &&
          fabs(m_skewing_scopeMiniA2_Angle_Correct[mirror] -
               (m_skewing_scopeMiniA2_Angle[mirror] + offsetAngle +
                m_MirrorVerAngleOffset[mirror])) > 0.001) {
        m_skewing_scopeMiniA2_Angle_Correct[mirror] =
            m_skewing_scopeMiniA2_Angle[mirror] + offsetAngle +
            m_MirrorVerAngleOffset[mirror];
        m_skewing_sin_scopeMiniA2_192[mirror] =
            sin(-1 * m_skewing_scopeMiniA2_Angle_Correct[mirror] * m_calRA);
        m_skewing_cos_scopeMiniA2_192[mirror] =
            cos(-1 * m_skewing_scopeMiniA2_Angle_Correct[mirror] * m_calRA);
      }

      double hA = 0.5 * (horizontalAngle)*m_calRA;
      double hA_sin = sin(hA);
      double hA_cos = cos(hA);

      x_cal_1 = 2.0 * m_skewing_cos_scopeMiniA2_192[mirror] *
                    m_skewing_cos_scopeMiniA2_192[mirror] * hA_cos * hA_cos -
                1;
      x_cal_2 = 2.0 * m_skewing_sin_scopeMiniA2_192[mirror] *
                m_skewing_cos_scopeMiniA2_192[mirror] * hA_cos;

      y_cal_1 = 2.0 * m_skewing_cos_scopeMiniA2_192[mirror] *
                m_skewing_cos_scopeMiniA2_192[mirror] * hA_sin * hA_cos;
      y_cal_2 = 2.0 * m_skewing_sin_scopeMiniA2_192[mirror] *
                m_skewing_cos_scopeMiniA2_192[mirror] * hA_sin;

      z_cal_1 = 2.0 * m_skewing_sin_scopeMiniA2_192[mirror] *
                m_skewing_cos_scopeMiniA2_192[mirror] * hA_cos;
      z_cal_2 = 2.0 * m_skewing_sin_scopeMiniA2_192[mirror] *
                    m_skewing_sin_scopeMiniA2_192[mirror] -
                1;
    }

    unsigned int blockSecond =
        FourHexToInt(udpData[offset + 132], udpData[offset + 133],
                     udpData[offset + 134], udpData[offset + 135]);
    unsigned char hexBlockMicrosecond = udpData[offset + 137];
    hexBlockMicrosecond = hexBlockMicrosecond & 0x0F;
    unsigned int blockMicrosecond =
        FourHexToInt(0x00, hexBlockMicrosecond, udpData[offset + 138],
                     udpData[offset + 139]);

    PRE_LOAD_PCAP_FILE_CALCULATE

    // separate index
    unsigned char hexSepIndex = udpData[offset + 136];
    unsigned short sepIndex = hexSepIndex >> 6;

    for (int seq = 0; seq < 16; seq++) {
      double hexToInt1 = TwoHextoInt(udpData[offset + seq * 8 + 0],
                                     udpData[offset + seq * 8 + 1]);
      double hexPulse1 = TwoHextoInt(udpData[offset + seq * 8 + 2],
                                     udpData[offset + seq * 8 + 3]);
      double L_1 = hexToInt1 * m_calSimpleFPGA;
      //      int intensity1 =
      //          (hexPulse1 * m_calPulseFPGA) / ScopePulseMapValue * 255.0
      //          + 0.5;
      //      double pulse_1 = intensity1 > 255 ? 255 : intensity1;
      double pulse_1 = hexPulse1 * m_calPulseFPGA; // 脉宽

      double hexToInt2 = TwoHextoInt(udpData[offset + seq * 8 + 4],
                                     udpData[offset + seq * 8 + 5]);
      double hexPulse2 = TwoHextoInt(udpData[offset + seq * 8 + 6],
                                     udpData[offset + seq * 8 + 7]);
      double L_2 = hexToInt2 * m_calSimpleFPGA;
      //      int intensity2 =
      //          (hexPulse2 * m_calPulseFPGA) / ScopePulseMapValue * 255.0
      //          + 0.5;
      //      double pulse_2 = intensity2 > 255 ? 255 : intensity2;
      double pulse_2 = hexPulse2 * m_calPulseFPGA; // 脉宽

      int channel =
          65 - (16 * (blocks_num >= 4 ? blocks_num - 4 : blocks_num) + seq + 1);

      double cos_vA_RA =
          m_verticalChannelAngle_ScopeMiniA2_cos_vA_RA[channel - 1];
      double sin_vA_RA =
          m_verticalChannelAngle_ScopeMiniA2_sin_vA_RA[channel - 1];

      TWPoint basic_point;
      basic_point.angle = horizontalAngle;
      basic_point.mirror = mirror;
      basic_point.channel = channel;
      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }
      if (_echoNum & Echo1) {
        basic_point.x = L_1 * (cos_vA_RA * x_cal_1 + sin_vA_RA * x_cal_2);
        basic_point.y = L_1 * (cos_vA_RA * y_cal_1 + sin_vA_RA * y_cal_2);
        basic_point.z = -L_1 * (cos_vA_RA * z_cal_1 + sin_vA_RA * z_cal_2);

        basic_point.echo = 1;
        basic_point.distance = L_1;
        basic_point.pulse = pulse_1;
        basic_point.intensity = pulseToIntensity(pulse_1);
        points.push_back(std::move(basic_point));
      }

      if (_echoNum & Echo2) {
        basic_point.x = L_2 * (cos_vA_RA * x_cal_1 + sin_vA_RA * x_cal_2);
        basic_point.y = L_2 * (cos_vA_RA * y_cal_1 + sin_vA_RA * y_cal_2);
        basic_point.z = -L_2 * (cos_vA_RA * z_cal_1 + sin_vA_RA * z_cal_2);

        basic_point.echo = 2;
        basic_point.distance = L_2;
        basic_point.pulse = pulse_2;
        basic_point.intensity = pulseToIntensity(pulse_2);
        points.push_back(std::move(basic_point));
      }
    }
  }
}

void LidarDevice::UseDecodeTempoA3(const char *udpData,
                                   TWPointCloud::Points &points,
                                   unsigned int *sec, unsigned int *usec,
                                   float *maxAngle) {
  int frame_cycle_cout = TwoHextoInt(udpData[4], udpData[5]);
  for (int blocks_num = 0; blocks_num < 12; blocks_num++) {
    int offset_block = blocks_num * 100;

    //    unsigned int blockSecond =
    //        FourHexToInt(udpData[offset_block + 132], udpData[offset_block +
    //        133],
    //                     udpData[offset_block + 134], udpData[offset_block +
    //                     135]);
    //    unsigned char hexBlockMicrosecond = udpData[offset_block + 137];
    //    hexBlockMicrosecond = hexBlockMicrosecond & 0x0F;
    //    unsigned int blockMicrosecond =
    //        FourHexToInt(0x00, hexBlockMicrosecond, udpData[offset_block +
    //        138],
    //                     udpData[offset_block + 139]);

    if (0 == blocks_num || 6 == blocks_num) {
      bool checkValue[6 * 6] = {0};
      for (int check_block_num = 0; check_block_num < 6; check_block_num++) {
        unsigned char hexCheck =
            udpData[offset_block + 34 + check_block_num * 100];
        checkValue[check_block_num * 6 + 0] = (bool)(0x20 & hexCheck);
        checkValue[check_block_num * 6 + 1] = (bool)(0x10 & hexCheck);
        checkValue[check_block_num * 6 + 2] = (bool)(0x08 & hexCheck);
        checkValue[check_block_num * 6 + 3] = (bool)(0x04 & hexCheck);
        checkValue[check_block_num * 6 + 4] = (bool)(0x02 & hexCheck);
        checkValue[check_block_num * 6 + 5] = (bool)(0x01 & hexCheck);
      }
      for (int b = 0; b < 6; b++) {
        for (int P = 1; P <= 3; P++) {
          CheckDataInfo info;
          if (0 == b)
            info =
                CheckDataInfo(CHECKCH(P, 10), checkValue[b * 6 + (P - 1) * 2],
                              CHECKCH(P, 24), checkValue[b * 6 + P * 2 - 1]);
          if (1 == b)
            info =
                CheckDataInfo(CHECKCH(P, 11), checkValue[b * 6 + (P - 1) * 2],
                              CHECKCH(P, 24), checkValue[b * 6 + P * 2 - 1]);
          if (2 == b)
            info = CheckDataInfo(CHECKCH(P, 6), checkValue[b * 6 + (P - 1) * 2],
                                 CHECKCH(P, 18), checkValue[b * 6 + P * 2 - 1]);
          if (3 == b)
            info = CheckDataInfo(CHECKCH(P, 7), checkValue[b * 6 + (P - 1) * 2],
                                 CHECKCH(P, 19), checkValue[b * 6 + P * 2 - 1]);
          if (4 == b)
            info = CheckDataInfo(CHECKCH(P, 1), checkValue[b * 6 + (P - 1) * 2],
                                 CHECKCH(P, 14), checkValue[b * 6 + P * 2 - 1]);
          if (5 == b)
            info = CheckDataInfo(CHECKCH(P, 1), checkValue[b * 6 + (P - 1) * 2],
                                 CHECKCH(P, 15), checkValue[b * 6 + P * 2 - 1]);
          m_tempoA3ChannelToCheckIndex[CHECKCH(P, 1 + b) - 1] = info;
          m_tempoA3ChannelToCheckIndex[CHECKCH(P, 7 + b) - 1] = info;
          m_tempoA3ChannelToCheckIndex[CHECKCH(P, 13 + b) - 1] = info;
          m_tempoA3ChannelToCheckIndex[CHECKCH(P, 19 + b) - 1] = info;
        }
      }
    }

    // 强弱标志
    unsigned char hexStrong = udpData[35 + offset_block];
    int nStrong = (int)((hexStrong & 0x08) >> 3);

    // mirror
    unsigned char hexMirror = udpData[35 + offset_block];
    hexMirror = hexMirror << 5;
    unsigned short mirror = hexMirror >> 6;

    // 转镜俯仰角（δ：参照PDF定义）
    double cos_delta = m_skewing_cos_tempoA3[mirror];
    double sin_delta = m_skewing_sin_tempoA3[mirror];

    for (int seq = 0; seq < 12; seq++) {
      // 水平角度	2Byte 36-37(index)  hexAngle*0.01
      double hexHorAngle = TwoHextoInt(udpData[36 + offset_block + seq * 8],
                                       udpData[37 + offset_block + seq * 8]);
      double horAngle = hexHorAngle * 0.01;
      double horAngleToShow = horAngle;

      // 回波1距离值 2Byte 40-41(index)  hexL1*0.005
      double hexL1 = TwoHextoInt(udpData[38 + offset_block + seq * 8],
                                 udpData[39 + offset_block + seq * 8]);
      double L_1 = hexL1 * m_calSimpleFPGA; // 米

      // 回波1强度值 0-255
      unsigned char hexPulse1 = udpData[40 + offset_block + seq * 8];
      double intensity_1 = (double)hexPulse1;
      double pulse_1 = intensity_1 * 0.128;

      // 回波2距离值 2Byte 41-42(index)  hexL1*0.005
      double hexL2 = TwoHextoInt(udpData[41 + offset_block + seq * 8],
                                 udpData[42 + offset_block + seq * 8]);
      double L_2 = hexL2 * m_calSimpleFPGA; // 米

      // 回波2强度值 0-255
      unsigned char hexPulse2 = udpData[43 + offset_block + seq * 8];
      double intensity_2 = (double)hexPulse2;
      double pulse_2 = intensity_2 * 0.128;

      // 单镜面通道号：1-72
      int signalMirrorChannel = blocks_num % 6 * 12 + seq + 1;

      // 转镜角度 三角函数计算 （θ：参照PDF定义）
      double theta = (horAngle)*0.5;
      if (signalMirrorChannel >= 1 && signalMirrorChannel <= 24) {
        theta = (horAngle + m_tempoA3RecvAmend[2]) * 0.5;
        horAngleToShow = horAngle + m_tempoA3RecvAmend[2];
      } else if (signalMirrorChannel >= 25 && signalMirrorChannel <= 48) {
        theta = (horAngle + m_tempoA3RecvAmend[1]) * 0.5;
        horAngleToShow = horAngle + m_tempoA3RecvAmend[1];
      } else if (signalMirrorChannel >= 49 && signalMirrorChannel <= 72) {
        theta = (horAngle + m_tempoA3RecvAmend[0]) * 0.5;
        horAngleToShow = horAngle + m_tempoA3RecvAmend[0];
      }

      double sin_theta = sin(theta * m_calRA);
      double cos_theta = cos(theta * m_calRA);

      // 每通道对应出射光线与水平方向夹角，仰为正（β：参照PDF定义）
      double cos_beta =
          m_verticalChannelAngle_TempoA3_cos_vA_RA[signalMirrorChannel - 1];
      double sin_beta =
          m_verticalChannelAngle_TempoA3_sin_vA_RA[signalMirrorChannel - 1];

      // 计算过程
      double x_t =
          cos_beta * (2 * cos_delta * cos_delta * cos_theta * cos_theta - 1) -
          sin_beta * (2 * sin_delta * cos_delta * cos_theta);
      double y_t =
          cos_beta * (2 * cos_delta * cos_delta * cos_theta * sin_theta) -
          sin_beta * (2 * cos_delta * sin_delta * sin_theta);
      double z_t = cos_beta * (2 * cos_delta * sin_delta * cos_theta) +
                   sin_beta * (1 - 2 * sin_delta * sin_delta);

      TWPoint basic_point;
      basic_point.angle = horAngleToShow;
      basic_point.mirror = mirror;
      basic_point.cycle_count = frame_cycle_cout;

      // echo1
      if (_echoNum & Echo1) {
        //  basic_point.channel = channel;
        basic_point.x = L_1 * x_t;
        basic_point.y = L_1 * y_t;
        basic_point.z = -L_1 * z_t;

        basic_point.echo = 1;
        basic_point.distance = L_1;
        basic_point.pulse = pulse_1;
        points.push_back(std::move(basic_point));
      }

      // echo2
      if (_echoNum & Echo2) {

        basic_point.x = L_2 * x_t;
        basic_point.y = L_2 * y_t;
        basic_point.z = -L_2 * z_t;

        basic_point.echo = 2;
        basic_point.distance = L_2;
        basic_point.pulse = pulse_2;
        points.push_back(std::move(basic_point));
      }
      //      if (_is_lidar_time) {
      //        basic_point.t_sec = blockSecond;
      //        basic_point.t_usec = blockMicrosecond;
      //      } else {
      //        basic_point.t_sec = *sec;
      //        basic_point.t_usec = *usec;
      //      }
    }
  }
}

void LidarDevice::UseDecodeDuetto(const char *udpData,
                                  TWPointCloud::Points &points,
                                  unsigned int *sec, unsigned int *usec,
                                  float *maxAngle) {
  // ptp
  unsigned int frameSecond =
      FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
  double frameMicrosecond =
      FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;
  int frame_cycle_cout = TwoHextoInt(udpData[4], udpData[5]);

  double sin_gamma1 = m_rotate_duetto_sinL;
  double cos_gamma1 = m_rotate_duetto_cosL;
  double sin_gamma2 = m_rotate_duetto_sinR;
  double cos_gamma2 = m_rotate_duetto_cosR;

  double nx = duettoPivotVector[0];
  double ny = duettoPivotVector[1];
  double nz = duettoPivotVector[2];

  for (int blocks_num = 0; blocks_num < 8; blocks_num++) {
    int offset_block = blocks_num * 164;

    int offsetMicrosecond =
        TwoHextoInt(udpData[32 + offset_block], udpData[33 + offset_block]);
    double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
    unsigned int blockSecond =
        (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
    unsigned int blockMicrosecond =
        (totalMicrosecond >= 1000000)
            ? (unsigned int)(totalMicrosecond - 1000000)
            : (unsigned int)totalMicrosecond;

    // L/R
    unsigned char hexLOrR = udpData[35 + offset_block];
    hexLOrR = hexLOrR << 7;
    unsigned short leftOrRight = hexLOrR >> 7; // 0:右；1:左
    // mirror
    unsigned char hexMirror = udpData[35 + offset_block];
    hexMirror = hexMirror << 5;
    unsigned short mirror = hexMirror >> 6;
    // block
    double hexBlockAngle = TwoHextoInt(udpData[36 + offset_block + 0 * 10],
                                       udpData[37 + offset_block + 0 * 10]);
    double blockAngle = hexBlockAngle * 0.01;
    int block = GetDuettoBlockNumber(blockAngle, mirror, leftOrRight);
    //
    double cos_delta = m_skewing_cos_duetto[mirror];
    double sin_delta = m_skewing_sin_duetto[mirror];

    for (int seq = 0; seq < 16; seq++) {
      double hexHorAngle = TwoHextoInt(udpData[36 + offset_block + seq * 10],
                                       udpData[37 + offset_block + seq * 10]);
      double horAngle = hexHorAngle * 0.01;

      horAngle += m_MirrorHorAngleOffset[mirror];

      // x、y、z
      double x_t = 0, y_t = 0, z_t = 0;
      double x_move = 0, y_move = 0, z_move = 0;
      //
      if (1 == leftOrRight) {
        double mp_angle = (210.0 - horAngle) * 0.5 + 240;
        double sin_theta = sin(mp_angle * m_calRA);
        double cos_theta = cos(mp_angle * m_calRA);

        x_move = m_correction_movement_L[0];
        y_move = m_correction_movement_L[1];
        z_move = m_correction_movement_L[2];

        double cos_beta = m_verticalChannelAngle_Duetto16L_cos_vA_RA[seq];
        double sin_beta = m_verticalChannelAngle_Duetto16L_sin_vA_RA[seq];

        double Tr11 = nx * nx * (1 - cos_theta) + cos_theta;
        double Tr12 = nx * ny * (1 - cos_theta) + nz * sin_theta;
        double Tr13 = nx * nz * (1 - cos_theta) - ny * sin_theta;
        double Tr21 = nx * ny * (1 - cos_theta) - nz * sin_theta;
        double Tr22 = ny * ny * (1 - cos_theta) + cos_theta;
        double Tr23 = ny * nz * (1 - cos_theta) + nx * sin_theta;
        double Tr31 = nx * nz * (1 - cos_theta) + ny * sin_theta;
        double Tr32 = ny * nz * (1 - cos_theta) - nx * sin_theta;
        double Tr33 = nz * nz * (1 - cos_theta) + cos_theta;

        double Nx = cos_delta * cos_gamma1 * Tr11 -
                    cos_delta * sin_gamma1 * Tr12 + sin_delta * Tr13;
        double Ny = cos_delta * cos_gamma1 * Tr21 -
                    cos_delta * sin_gamma1 * Tr22 + sin_delta * Tr23;
        double Nz = cos_delta * cos_gamma1 * Tr31 -
                    cos_delta * sin_gamma1 * Tr32 + sin_delta * Tr33;

        x_t = cos_beta * (-cos_gamma1 + 2 * cos_gamma1 * Nx * Nx -
                          2 * sin_gamma1 * Nx * Ny) -
              sin_beta * (2 * Nx * Nz);
        y_t = cos_beta * (sin_gamma1 + 2 * cos_gamma1 * Nx * Ny -
                          2 * sin_gamma1 * Ny * Ny) -
              sin_beta * (2 * Ny * Nz);
        z_t = cos_beta * (2 * cos_gamma1 * Nx * Nz - 2 * sin_gamma1 * Ny * Nz) +
              sin_beta * (1 - 2 * Nz * Nz);
      } else {
        double mp_angle = (210.0 - horAngle) * 0.5;
        double sin_theta = sin(mp_angle * m_calRA);
        double cos_theta = cos(mp_angle * m_calRA);

        x_move = m_correction_movement_R[0];
        y_move = m_correction_movement_R[1];
        z_move = m_correction_movement_R[2];

        double cos_beta = m_verticalChannelAngle_Duetto16R_cos_vA_RA[seq];
        double sin_beta = m_verticalChannelAngle_Duetto16R_sin_vA_RA[seq];

        double Tr11 = nx * nx * (1 - cos_theta) + cos_theta;
        double Tr12 = nx * ny * (1 - cos_theta) + nz * sin_theta;
        double Tr13 = nx * nz * (1 - cos_theta) - ny * sin_theta;
        double Tr21 = nx * ny * (1 - cos_theta) - nz * sin_theta;
        double Tr22 = ny * ny * (1 - cos_theta) + cos_theta;
        double Tr23 = ny * nz * (1 - cos_theta) + nx * sin_theta;
        double Tr31 = nx * nz * (1 - cos_theta) + ny * sin_theta;
        double Tr32 = ny * nz * (1 - cos_theta) - nx * sin_theta;
        double Tr33 = nz * nz * (1 - cos_theta) + cos_theta;

        double Nx = cos_delta * cos_gamma2 * Tr11 +
                    cos_delta * sin_gamma2 * Tr12 - sin_delta * Tr13;
        double Ny = cos_delta * cos_gamma2 * Tr21 +
                    cos_delta * sin_gamma2 * Tr22 - sin_delta * Tr23;
        double Nz = cos_delta * cos_gamma2 * Tr31 +
                    cos_delta * sin_gamma2 * Tr32 - sin_delta * Tr33;

        x_t = cos_beta * (cos_gamma2 - 2 * cos_gamma2 * Nx * Nx -
                          2 * sin_gamma2 * Nx * Ny) -
              sin_beta * (2 * Nx * Nz);
        y_t = cos_beta * (sin_gamma2 - 2 * cos_gamma2 * Nx * Ny -
                          2 * sin_gamma2 * Ny * Ny) -
              sin_beta * (2 * Ny * Nz);
        z_t =
            -cos_beta * (2 * cos_gamma2 * Nx * Nz + 2 * sin_gamma2 * Ny * Nz) +
            sin_beta * (1 - 2 * Nz * Nz);
      }

      double hexL1 = TwoHextoInt(udpData[40 + offset_block + seq * 10],
                                 udpData[41 + offset_block + seq * 10]);
      double L_1 = hexL1 * 0.005;
      unsigned char hexChar1 = udpData[42 + offset_block + seq * 10];
      unsigned short hexPulse1 = hexChar1;
      double pulse_1 = hexPulse1;

      double hexL2 = TwoHextoInt(udpData[43 + offset_block + seq * 10],
                                 udpData[44 + offset_block + seq * 10]);
      double L_2 = hexL2 * 0.005; //
      unsigned char hexChar2 = udpData[45 + offset_block + seq * 10];
      unsigned short hexPulse2 = hexChar2;
      double pulse_2 = hexPulse2;

      TWPoint basic_point;
      basic_point.angle = horAngle;
      basic_point.mirror = mirror;
      basic_point.left_right = leftOrRight;
      basic_point.channel =
          48 * leftOrRight + (abs(mirror - 2) * 16 + (16 - seq));
      basic_point.block = block;
      basic_point.cycle_count = frame_cycle_cout;

      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }

      if (_echoNum & Echo1) {
        basic_point.x = L_1 * x_t + x_move;
        basic_point.y = L_1 * y_t + y_move;
        basic_point.z = L_1 * z_t + z_move;
        basic_point.distance = L_1;

        double intensityNormalize =
            ((0 == leftOrRight) ? m_intensityNormalizeR
                                : m_intensityNormalizeL);
        basic_point.intensity = pulse_1 * 0.125 / intensityNormalize * 128;
        basic_point.pulse = pulse_1 * 0.125;

        basic_point.echo = 1;
        points.push_back(std::move(basic_point));
      }

      if (_echoNum & Echo2) {
        basic_point.x = L_2 * x_t + x_move;
        basic_point.y = L_2 * y_t + y_move;
        basic_point.z = L_2 * z_t + z_move;

        basic_point.distance = L_2;
        double intensityNormalize =
            ((0 == leftOrRight) ? m_intensityNormalizeR
                                : m_intensityNormalizeL);
        basic_point.intensity = pulse_2 * 0.125 / intensityNormalize * 128;
        basic_point.pulse = pulse_2 * 0.125;
        basic_point.echo = 2;
        points.push_back(std::move(basic_point));
      }
    }
  }
}

void LidarDevice::UseDecodeTensor48(const char *udpData,
                                    TWPointCloud::Points &points,
                                    unsigned int *sec, unsigned int *usec,
                                    float *maxAngle) {
  // ptp
  unsigned int frameSecond =
      FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
  double frameMicrosecond =
      FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;

  for (int blocks_num = 0; blocks_num < 8; blocks_num++) {
    int offset_block = blocks_num * 164;

    // ptp
    int offsetMicrosecond =
        TwoHextoInt(udpData[32 + offset_block], udpData[33 + offset_block]);
    double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
    unsigned int blockSecond =
        (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
    unsigned int blockMicrosecond =
        (totalMicrosecond >= 1000000)
            ? (unsigned int)(totalMicrosecond - 1000000)
            : (unsigned int)totalMicrosecond;

    // mirror
    unsigned char hexMirror = udpData[35 + offset_block];
    hexMirror = hexMirror << 5;
    unsigned short mirror = hexMirror >> 6;

    double cos_delta = m_skewing_cos_tsp48[mirror];
    double sin_delta = m_skewing_sin_tsp48[mirror];

    for (int seq = 0; seq < 16; seq++) {

      // 2Byte 36-37
      double hexHorAngle = TwoHextoInt(udpData[36 + offset_block + seq * 10],
                                       udpData[37 + offset_block + seq * 10]);
      double horAngle = hexHorAngle * 0.01;
      horAngle += m_MirrorHorAngleOffset[mirror];

      // L1 2Byte 40-41
      double hexL1 = TwoHextoInt(udpData[40 + offset_block + seq * 10],
                                 udpData[41 + offset_block + seq * 10]);
      double L_1 = hexL1 * m_calSimpleFPGA;

      // intensity 0-255
      unsigned char hexIntensity1 = udpData[42 + offset_block + seq * 10];
      double intensity_1 = hexIntensity1;

      // L2 2Byte
      double hexL2 = TwoHextoInt(udpData[43 + offset_block + seq * 10],
                                 udpData[44 + offset_block + seq * 10]);
      double L_2 = hexL2 * m_calSimpleFPGA;

      // intensity 0-255
      unsigned char hexIntensity2 = udpData[45 + offset_block + seq * 10];
      double intensity_2 = hexIntensity2;

      // 转镜法线转过的角度 θ =  (720 - φ) / 2
      double theta = (720 - horAngle) * 0.5;
      double sin_theta = sin(theta * m_calRA);
      double cos_theta = cos(theta * m_calRA);

      // 计算各通道竖直角度（β）的正弦和余弦
      double cos_beta = m_verticalChannelAngle_TSP48_cos_vA_RA[seq];
      double sin_beta = m_verticalChannelAngle_TSP48_sin_vA_RA[seq];

      // 机芯发射光线单位向量 li = (-cosβ,0,sinβ)
      double Lx = -cos_beta;
      double Ly = 0;
      double Lz = sin_beta;

      double Nx = cos_delta * cos_theta;
      double Ny = -cos_delta * sin_theta;
      double Nz = sin_delta;

      TWPoint basic_point;
      basic_point.angle = horAngle;
      basic_point.mirror = mirror;
      basic_point.left_right = 1;
      basic_point.channel = mirror * 16 + seq + 1;

      double dot_product = Lx * Nx + Ly * Ny + Lz * Nz;

      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }

      // 计算笛卡尔坐标X,Y,Z
      // echo 1
      if (_echoNum & Echo1) {
        basic_point.x = L_1 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_1 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_1 * (Lz - 2 * dot_product * Nz);

        basic_point.distance = L_1;
        basic_point.intensity = intensity_1;
        basic_point.pulse = intensity_1 * 0.128;
        basic_point.echo = 1;
        points.push_back(std::move(basic_point));
      }

      // echo 2
      if (_echoNum & Echo2) {
        basic_point.x = L_2 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_2 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_2 * (Lz - 2 * dot_product * Nz);

        basic_point.distance = L_2;
        basic_point.intensity = intensity_2;
        basic_point.pulse = intensity_2 * 0.128;
        basic_point.echo = 2;
        points.push_back(std::move(basic_point));
      }
    }
  }
}

void LidarDevice::UseDecodeTensor48_2(const char *udpData,
                                      TWPointCloud::Points &points,
                                      unsigned int *sec, unsigned int *usec,
                                      float *maxAngle) {
  // ptp
  unsigned int frameSecond =
      FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
  double frameMicrosecond =
      FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;

  for (int blocks_num = 0; blocks_num < 8; blocks_num++) {
    int offset_block = blocks_num * 164;

    // ptp
    int offsetMicrosecond =
        TwoHextoInt(udpData[32 + offset_block], udpData[33 + offset_block]);
    double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
    unsigned int blockSecond =
        (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
    unsigned int blockMicrosecond =
        (totalMicrosecond >= 1000000)
            ? (unsigned int)(totalMicrosecond - 1000000)
            : (unsigned int)totalMicrosecond;

    // mirror
    unsigned char hexMirror = udpData[35 + offset_block];
    hexMirror = hexMirror << 5;
    unsigned short mirror = hexMirror >> 6;

    double cos_delta = m_skewing_cos_tsp48[mirror];
    double sin_delta = m_skewing_sin_tsp48[mirror];

    for (int seq = 0; seq < 16; seq++) {
      // 2Byte 36-37
      double hexHorAngle = TwoHextoInt(udpData[36 + offset_block + seq * 10],
                                       udpData[37 + offset_block + seq * 10]);
      double horAngle = hexHorAngle * 0.01;
      horAngle += m_MirrorHorAngleOffset[mirror];

      // L1 2Byte 38-39
      double hexL1 = TwoHextoInt(udpData[38 + offset_block + seq * 10],
                                 udpData[39 + offset_block + seq * 10]);
      double L_1 = hexL1 * m_calSimpleFPGA;

      // intensity 0-255
      unsigned char hexChar1 = udpData[40 + offset_block + seq * 10];
      double intensity_1 = ((double)hexChar1);

      double confidence1 = udpData[41 + offset_block + seq * 10];

      // L2 2Byte
      double hexL2 = TwoHextoInt(udpData[42 + offset_block + seq * 10],
                                 udpData[43 + offset_block + seq * 10]);
      double L_2 = hexL2 * m_calSimpleFPGA;

      // intensity 0-255
      unsigned char hexChar2 = udpData[44 + offset_block + seq * 10];
      double intensity_2 = ((double)hexChar2);

      double confidence2 = udpData[45 + offset_block + seq * 10];

      // 转镜法线转过的角度 θ =  (720 - φ) / 2
      double theta = (720 - horAngle) * 0.5;
      double sin_theta = sin(theta * m_calRA);
      double cos_theta = cos(theta * m_calRA);

      // 计算各通道竖直角度（β）的正弦和余弦
      double cos_beta = m_verticalChannelAngle_TSP48_cos_vA_RA[seq];
      double sin_beta = m_verticalChannelAngle_TSP48_sin_vA_RA[seq];

      // 机芯发射光线单位向量 li = (-cosβ,0,sinβ)
      double Lx = -cos_beta;
      double Ly = 0;
      double Lz = sin_beta;

      double Nx = cos_delta * cos_theta;
      double Ny = -cos_delta * sin_theta;
      double Nz = sin_delta;

      TWPoint basic_point;
      basic_point.angle = horAngle;
      basic_point.mirror = mirror;
      basic_point.left_right = 1;
      basic_point.channel = mirror * 16 + seq + 1;

      double dot_product = Lx * Nx + Ly * Ny + Lz * Nz;
      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }
      // 计算笛卡尔坐标X,Y,Z
      // echo 1
      if (_echoNum & Echo1) {
        basic_point.x = L_1 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_1 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_1 * (Lz - 2 * dot_product * Nz);

        basic_point.distance = L_1;
        basic_point.intensity = intensity_1;
        basic_point.echo = 1;
        basic_point.confidence = confidence1;
        points.push_back(std::move(basic_point));
      }

      // echo 2
      if (_echoNum & Echo2) {
        basic_point.x = L_2 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_2 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_2 * (Lz - 2 * dot_product * Nz);

        basic_point.distance = L_2;
        basic_point.intensity = intensity_2;
        basic_point.echo = 2;
        basic_point.confidence = confidence2;
        points.push_back(std::move(basic_point));
      }
    }
  }
}

void LidarDevice::UseDecodeTensor48Calib(const char *udpData,
                                         TWPointCloud::Points &points,
                                         unsigned int *sec, unsigned int *usec,
                                         float *maxAngle) {
  // ptp
  unsigned int frameSecond =
      FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
  double frameMicrosecond =
      FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;

  for (int blocks_num = 0; blocks_num < 6; blocks_num++) {
    int offset_block = blocks_num * 196;

    // ptp
    int offsetMicrosecond =
        TwoHextoInt(udpData[32 + offset_block], udpData[33 + offset_block]);
    double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
    unsigned int blockSecond =
        (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
    unsigned int blockMicrosecond =
        (totalMicrosecond >= 1000000)
            ? (unsigned int)(totalMicrosecond - 1000000)
            : (unsigned int)totalMicrosecond;

    // mirror
    unsigned char hexMirror = udpData[35 + offset_block];
    hexMirror = hexMirror << 5;
    unsigned short mirror = hexMirror >> 6;

    double cos_delta = m_skewing_cos_tsp48[mirror];
    double sin_delta = m_skewing_sin_tsp48[mirror];

    for (int seq = 0; seq < 16; seq++) {

      // 2Byte 36-37
      double hexHorAngle = TwoHextoInt(udpData[36 + offset_block + seq * 12],
                                       udpData[37 + offset_block + seq * 12]);
      double horAngle = hexHorAngle * 0.01;
      horAngle += m_MirrorHorAngleOffset[mirror];

      // L1 2Byte 40-41
      double hexL1 = TwoHextoInt(udpData[38 + offset_block + seq * 12],
                                 udpData[39 + offset_block + seq * 12]);
      double L_1 = hexL1 * m_calSimpleFPGA;

      // intensity 0-255
      unsigned char hexChar1 = udpData[40 + offset_block + seq * 12];
      double intensity_1 = ((double)hexChar1);
      double pulse_1 =
          OneHexHalfHextoInt(udpData[41 + offset_block + seq * 12],
                             udpData[42 + offset_block + seq * 12]) *
          0.032;

      double confidence1 = udpData[42 + offset_block + seq * 12] & 1;

      double hexL2 = TwoHextoInt(udpData[43 + offset_block + seq * 12],
                                 udpData[44 + offset_block + seq * 12]);
      double L_2 = hexL2 * m_calSimpleFPGA;

      // intensity 0-255
      unsigned char hexChar2 = udpData[45 + offset_block + seq * 12];
      double intensity_2 = ((double)hexChar2);
      double pulse_2 =
          OneHexHalfHextoInt(udpData[46 + offset_block + seq * 12],
                             udpData[47 + offset_block + seq * 12]) *
          0.032;

      double confidence2 = udpData[46 + offset_block + seq * 12] & 1;

      // 转镜法线转过的角度 θ =  (720 - φ) / 2
      double theta = (720 - horAngle) * 0.5;
      double sin_theta = sin(theta * m_calRA);
      double cos_theta = cos(theta * m_calRA);

      // 计算各通道竖直角度（β）的正弦和余弦
      double cos_beta = m_verticalChannelAngle_TSP48_cos_vA_RA[seq];
      double sin_beta = m_verticalChannelAngle_TSP48_sin_vA_RA[seq];

      // 机芯发射光线单位向量 li = (-cosβ,0,sinβ)
      double Lx = -cos_beta;
      double Ly = 0;
      double Lz = sin_beta;

      double Nx = cos_delta * cos_theta;
      double Ny = -cos_delta * sin_theta;
      double Nz = sin_delta;

      TWPoint basic_point;
      basic_point.angle = horAngle;
      basic_point.mirror = mirror;
      basic_point.left_right = 1;
      basic_point.channel = mirror * 16 + seq + 1;

      double dot_product = Lx * Nx + Ly * Ny + Lz * Nz;
      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }
      // 计算笛卡尔坐标X,Y,Z
      if (_echoNum & Echo1) {
        basic_point.x = L_1 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_1 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_1 * (Lz - 2 * dot_product * Nz);

        basic_point.distance = L_1;
        basic_point.intensity = intensity_1;
        basic_point.pulse = pulse_1;
        basic_point.confidence = confidence1;
        basic_point.echo = 1;
        points.push_back(std::move(basic_point));
      }

      if (_echoNum & Echo2) {
        basic_point.x = L_2 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_2 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_2 * (Lz - 2 * dot_product * Nz);

        basic_point.distance = L_2;
        basic_point.intensity = intensity_2;
        basic_point.pulse = pulse_2;
        basic_point.confidence = confidence2;
        basic_point.echo = 2;
        points.push_back(std::move(basic_point));
      }
    }
  }
}

void LidarDevice::UseDecodeTensor48Depth(const char *udpData,
                                         TWPointCloud::Points &points,
                                         unsigned int *sec, unsigned int *usec,
                                         float *maxAngle) {
  // ptp
  unsigned int frameSecond =
      FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
  double frameMicrosecond =
      FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;

  for (int blocks_num = 0; blocks_num < 8; blocks_num++) {
    int offset_block = blocks_num * 116;

    // ptp
    int offsetMicrosecond =
        TwoHextoInt(udpData[32 + offset_block], udpData[33 + offset_block]);
    double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
    unsigned int blockSecond =
        (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
    unsigned int blockMicrosecond =
        (totalMicrosecond >= 1000000)
            ? (unsigned int)(totalMicrosecond - 1000000)
            : (unsigned int)totalMicrosecond;

    // L/R
    unsigned char hexLOrR = udpData[35 + offset_block];
    hexLOrR = hexLOrR << 7;
    unsigned short leftOrRight = hexLOrR >> 7; // 0:right；1:left

    // mirror
    unsigned char hexMirror = udpData[35 + offset_block];
    hexMirror = hexMirror << 5;
    unsigned short mirror = hexMirror >> 6;

    for (int seq = 0; seq < 16; seq++) {
      // U
      double hexU = TwoHextoInt(udpData[36 + offset_block + seq * 7],
                                udpData[37 + offset_block + seq * 7]);
      double dU = hexU /** 0.01*/; //

      // V
      double hexV = TwoHextoInt(udpData[38 + offset_block + seq * 7],
                                udpData[39 + offset_block + seq * 7]);
      double dV = hexV /** 0.01*/; //

      // Z
      double hexZ = TwoHextoInt(udpData[40 + offset_block + seq * 7],
                                udpData[41 + offset_block + seq * 7]);
      double dZ = hexZ * m_calSimpleFPGA;

      // intensity 0-255
      unsigned char hexChar = udpData[42 + offset_block + seq * 7];
      unsigned short hexPulse = hexChar;
      double pulse = hexPulse;

      TWPoint basic_point;
      // basic_point.angle = 150-0.3333*dU;
      // basic_point.angle_show = 150 - 0.3333*dU;
      basic_point.mirror = mirror;
      basic_point.left_right = leftOrRight;
      basic_point.channel = mirror * 16 + seq + 1;

      basic_point.x = dU;
      basic_point.y = dV;
      basic_point.z = 0;
      basic_point.distance = dZ;
      basic_point.pulse = pulse;
      basic_point.echo = 1;
      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }

      points.push_back(std::move(basic_point));
    }
  }
}

bool LidarDevice::UseDecodeScope256(const char *udpData,
                                    TWPointCloud::Points &points,
                                    unsigned int *sec, unsigned int *usec,
                                    float *maxAngle) {
  // ptp
  unsigned int frameSecond =
      FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
  double frameMicrosecond =
      FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;

  int frame_cycle_cout = TwoHextoInt(udpData[4], udpData[5]);

  for (int blocks_num = 0; blocks_num < 8; blocks_num++) {
    int offset_block = blocks_num * 164;

    // ptp
    int offsetMicrosecond =
        TwoHextoInt(udpData[32 + offset_block], udpData[33 + offset_block]);
    double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
    unsigned int blockSecond =
        (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
    unsigned int blockMicrosecond =
        (totalMicrosecond >= 1000000)
            ? (unsigned int)(totalMicrosecond - 1000000)
            : (unsigned int)totalMicrosecond;

    // L/R
    unsigned char hexLOrR = udpData[35 + offset_block];
    hexLOrR = hexLOrR << 7;
    unsigned short leftOrRight = hexLOrR >> 7;

    // mirror
    unsigned char hexMirror = udpData[35 + offset_block];
    hexMirror = hexMirror << 5;
    unsigned short mirror = hexMirror >> 6;

    //
    double cos_delta = m_skewing_cos_scp256[mirror];
    double sin_delta = m_skewing_sin_scp256[mirror];

    for (int seq = 0; seq < 16; seq++) {
      // 2Byte 36-37 解析点云水平角度φ
      double hexHorAngle = TwoHextoInt(udpData[36 + offset_block + seq * 10],
                                       udpData[37 + offset_block + seq * 10]);
      double horAngle = hexHorAngle * 0.01;
      horAngle += m_MirrorHorAngleOffset[mirror];

      // L1 2Byte 40-41
      double hexL1 = TwoHextoInt(udpData[40 + offset_block + seq * 10],
                                 udpData[41 + offset_block + seq * 10]);
      double L_1 = hexL1 * m_calSimpleFPGA;

      // intensity 0-255
      unsigned char hexIntensity1 = udpData[42 + offset_block + seq * 10];
      double intensity_1 = hexIntensity1;

      // L2 2Byte
      double hexL2 = TwoHextoInt(udpData[43 + offset_block + seq * 10],
                                 udpData[44 + offset_block + seq * 10]);
      double L_2 = hexL2 * m_calSimpleFPGA;

      // intensity 0-255
      unsigned char hexIntensity2 = udpData[45 + offset_block + seq * 10];
      double intensity_2 = hexIntensity2;

      // 通道计算
      int channel = -1;
      int new_channel = -1; // new channel for point attr in scope128H
      int signal_mirror_seq = -1;

      if (m_decode_for_scope128H) {
        int block_id = blocks_num % 2;
        signal_mirror_seq = (0 == leftOrRight) ? (32 * block_id + seq * 2 + 1)
                                               : (32 * block_id + seq * 2);

        if (0 == mirror) {
          channel = (0 == leftOrRight) ? (128 + signal_mirror_seq * 2 + 2)
                                       : (signal_mirror_seq * 2 + 2);
          // new channel for point attr in scope128H
          new_channel = (0 == leftOrRight) ? (64 + signal_mirror_seq + 1)
                                           : (signal_mirror_seq + 2);
        } else if (1 == mirror) {
          channel = (0 == leftOrRight) ? (128 + signal_mirror_seq * 2 + 1)
                                       : (signal_mirror_seq * 2 + 1);
          // new channel for point attr in scope128H
          new_channel = (0 == leftOrRight) ? (64 + signal_mirror_seq)
                                           : (signal_mirror_seq + 1);
        }
      } else {
        signal_mirror_seq =
            16 * (blocks_num >= 4 ? blocks_num - 4 : blocks_num) + seq;

        if (0 == mirror) {
          channel = (0 == leftOrRight) ? (128 + signal_mirror_seq * 2 + 2)
                                       : (signal_mirror_seq * 2 + 2);
        } else if (1 == mirror) {
          channel = (0 == leftOrRight) ? (128 + signal_mirror_seq * 2 + 1)
                                       : (signal_mirror_seq * 2 + 1);
        }
      }

      // laser1/laser2
      int laser =
          (channel > 128)
              ? (((channel - 128) >= 1 && (channel - 128) <= 64) ? 2 : 1)
              : ((channel >= 1 && channel <= 64) ? 1 : 2);

      // 转镜角度 三角函数计算 （θ：参照PDF定义）
      double theta = 0;
      if (1 == leftOrRight) // 左机芯
      {
        theta = (1 == laser) ? ((720.0 - horAngle) * 0.5 - 3.0)
                             : ((720.0 - horAngle) * 0.5 - 1.0);
      } else // 右机芯
      {
        theta = (1 == laser) ? ((540.0 - horAngle) * 0.5 + 3.0)
                             : ((540.0 - horAngle) * 0.5 + 1.0);
      }

      // v3
      if (m_scope256LaserVersion == 1 && m_scope256PLAngleCorrectFlag == 0) {
        if (1 == leftOrRight) {
          theta = (720.0 - horAngle) * 0.5;
        } else {
          theta = (540.0 - horAngle) * 0.5;
        }
        int index = 0;
        if (signal_mirror_seq >= 16 && signal_mirror_seq < 32) {
          index = 1;
        } else if (signal_mirror_seq >= 32 && signal_mirror_seq < 48) {
          index = 2;
        } else if (signal_mirror_seq >= 48 && signal_mirror_seq < 64) {
          index = 3;
        }

        theta = (1 == leftOrRight)
                    ? (theta + m_Scope256LeftLaserOffset[index])
                    : (theta + m_Scope256RightLaserOffset[index]);
      }

      double Lx = 0.0;
      double Ly = 0.0;
      double Lz = 0.0;
      // 转镜法线转过的角度（θ）的正弦和余弦
      double sin_theta = sin(theta * m_calRA);
      double cos_theta = cos(theta * m_calRA);

      double Nx = cos_delta * cos_theta;
      double Ny = -cos_delta * sin_theta;
      double Nz = sin_delta;

      if (1 == leftOrRight) {
        // 计算各通道竖直角度（β）的正弦和余弦
        double sin_beta =
            m_verticalChannelAngle_scp256L_sin_vA_RA[signal_mirror_seq];
        double cos_beta =
            m_verticalChannelAngle_scp256L_cos_vA_RA[signal_mirror_seq];

        // 计算出射光线与X轴夹角（γ）的正弦和余弦
        double sin_gamma = m_rotate_scp256L_sin;
        double cos_gamma = m_rotate_scp256L_cos;

        // 机芯发射光线单位向量li = (cosβcosγ,-cosβsinγ,sinβ)
        Lx = cos_beta * cos_gamma;
        Ly = -cos_beta * sin_gamma;
        Lz = sin_beta;
      } else {
        // 计算各通道竖直角度（β）的正弦和余弦
        double sin_beta =
            m_verticalChannelAngle_scp256R_sin_vA_RA[signal_mirror_seq];
        double cos_beta =
            m_verticalChannelAngle_scp256R_cos_vA_RA[signal_mirror_seq];

        // 计算出射光线与X轴夹角（γ）的正弦和余弦
        double sin_gamma = m_rotate_scp256R_sin;
        double cos_gamma = m_rotate_scp256R_cos;

        // 机芯发射光线单位向量li = (cosβcosγ,-cosβsinγ,sinβ)
        Lx = cos_beta * cos_gamma;
        Ly = -cos_beta * sin_gamma;
        Lz = sin_beta;
      }

      double dot_product = Lx * Nx + Ly * Ny + Lz * Nz;

      TWPoint basic_point;
      basic_point.angle = horAngle;
      basic_point.mirror = mirror;
      basic_point.left_right = leftOrRight;
      basic_point.channel =
          (m_decode_for_scope128H == true) ? new_channel : channel;
      basic_point.cycle_count = frame_cycle_cout;

      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }

      if (_echoNum & Echo1) {
        basic_point.x = L_1 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_1 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_1 * (Lz - 2 * dot_product * Nz);

        basic_point.intensity = intensity_1;
        basic_point.distance = L_1;
        basic_point.pulse = intensity_1 * 0.128;
        basic_point.echo = 1;
        points.push_back(std::move(basic_point));
      }

      if (_echoNum & Echo2) {
        basic_point.x = L_2 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_2 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_2 * (Lz - 2 * dot_product * Nz);

        basic_point.intensity = intensity_2;
        basic_point.distance = L_2;
        basic_point.pulse = intensity_2 * 0.128;
        basic_point.echo = 2;
        points.push_back(std::move(basic_point));
      }
    }
  }
  USEDECODE_COMMON_RETURN
}

void LidarDevice::WallThicknessScope256(char *udpData) {
  int offset_block1;
  int offset_block2;
  int offset_block3;
  int offset_block4;
  int pulseyz = 15;
  float startdistance = 1;
  float ds = 0.05;
  float Pulse_threshold = 2;
  double *bcz = nullptr;

  for (int colunm_num = 0; colunm_num < 2; colunm_num++) {
    int blocks_num = 0;
    if (colunm_num == 1) {
      blocks_num = 4 + blocks_num;
    }
    for (int seq = 0; seq < 16; seq++) {
      offset_block1 = blocks_num * 164;
      offset_block2 = (blocks_num + 1) * 164;
      offset_block3 = (blocks_num + 2) * 164;
      offset_block4 = (blocks_num + 3) * 164;
      double hexL11 = TwoHextoInt(udpData[40 + offset_block1 + seq * 10],
                                  udpData[41 + offset_block1 + seq * 10]);
      double L_11 = hexL11 * m_calSimpleFPGA;
      unsigned char hexIntensity11 = udpData[42 + offset_block1 + seq * 10];
      double pulse_11 = hexIntensity11 * 0.128;
      double hexL21 = TwoHextoInt(udpData[43 + offset_block1 + seq * 10],
                                  udpData[44 + offset_block1 + seq * 10]);
      double L_21 = hexL21 * m_calSimpleFPGA;
      unsigned char hexIntensity21 = udpData[45 + offset_block1 + seq * 10];
      double pulse_21 = hexIntensity21 * 0.128;

      double hexL12 = TwoHextoInt(udpData[40 + offset_block2 + seq * 10],
                                  udpData[41 + offset_block2 + seq * 10]);
      double L_12 = hexL12 * m_calSimpleFPGA;
      unsigned char hexIntensity12 = udpData[42 + offset_block2 + seq * 10];
      double pulse_12 = hexIntensity12 * 0.128;
      double hexL22 = TwoHextoInt(udpData[43 + offset_block2 + seq * 10],
                                  udpData[44 + offset_block2 + seq * 10]);
      double L_22 = hexL22 * m_calSimpleFPGA;
      unsigned char hexIntensity22 = udpData[45 + offset_block2 + seq * 10];
      double pulse_22 = hexIntensity22 * 0.128;

      double hexL13 = TwoHextoInt(udpData[40 + offset_block3 + seq * 10],
                                  udpData[41 + offset_block3 + seq * 10]);
      double L_13 = hexL13 * m_calSimpleFPGA;
      unsigned char hexIntensity13 = udpData[42 + offset_block3 + seq * 10];
      double pulse_13 = hexIntensity13 * 0.128;
      double hexL23 = TwoHextoInt(udpData[43 + offset_block3 + seq * 10],
                                  udpData[44 + offset_block3 + seq * 10]);
      double L_23 = hexL23 * m_calSimpleFPGA;
      unsigned char hexIntensity23 = udpData[45 + offset_block3 + seq * 10];
      double pulse_23 = hexIntensity23 * 0.128;

      double hexL14 = TwoHextoInt(udpData[40 + offset_block4 + seq * 10],
                                  udpData[41 + offset_block4 + seq * 10]);
      double L_14 = hexL14 * m_calSimpleFPGA;
      unsigned char hexIntensity14 = udpData[42 + offset_block4 + seq * 10];
      double pulse_14 = hexIntensity14 * 0.128;
      double hexL24 = TwoHextoInt(udpData[43 + offset_block4 + seq * 10],
                                  udpData[44 + offset_block4 + seq * 10]);
      double L_24 = hexL24 * m_calSimpleFPGA;
      unsigned char hexIntensity24 = udpData[45 + offset_block4 + seq * 10];
      double pulse_24 = hexIntensity24 * 0.128;

      if (seq >= 0 && seq <= 1) {
        bcz = m_1;
      } else if (seq >= 2 && seq <= 3) {
        bcz = m_2;
      } else if (seq >= 4 && seq <= 5) {
        bcz = m_3;
      } else if (seq >= 6 && seq <= 7) {
        bcz = m_4;
      } else if (seq >= 8 && seq <= 9) {
        bcz = m_5;
      } else if (seq >= 10 && seq <= 11) {
        bcz = m_6;
      } else if (seq >= 12 && seq <= 13) {
        bcz = m_7;
      } else if (seq >= 14 && seq <= 15) {
        bcz = m_8;
      }
      if (L_11 > 0 && pulse_11 > pulse_12 && pulse_11 > pulse_13 &&
          pulse_11 > pulse_14 && pulse_11 > pulseyz) {
        if (L_12 > L_11 && pulse_12 > pulse_22) {
          float dc = L_12 - L_11;
          int n = round((dc) / ds);
          float m = round((dc) / ds * 100) / 100;
          if (n <= 18) {
            L_12 -= m_0[n] + (m_0[n + 1] - m_0[n]) * (m - n);
            int L = L_12 / m_calSimpleFPGA;
            udpData[40 + offset_block2 + seq * 10] = L / 256;
            udpData[41 + offset_block2 + seq * 10] = L % 256;
            udpData[42 + offset_block2 + seq * 10] = 0;
          }
        }
        if (L_13 > L_11 && pulse_13 > pulse_23) {
          float dc = L_13 - L_11;
          int n = round((dc) / ds);
          float m = round((dc) / ds * 100) / 100;
          if (n <= 18) {
            L_13 -= m_0[n] + (m_0[n + 1] - m_0[n]) * (m - n);
            int L = L_13 / m_calSimpleFPGA;
            udpData[40 + offset_block3 + seq * 10] = L / 256;
            udpData[41 + offset_block3 + seq * 10] = L % 256;
            udpData[42 + offset_block3 + seq * 10] = 0;
          }
        }
        if (L_14 > L_11 && pulse_14 > pulse_24) {
          float dc = L_14 - L_11;
          int n = round((dc) / ds);
          float m = round((dc) / ds * 100) / 100;
          if (n <= 18) {
            L_14 -= m_0[n] + (m_0[n + 1] - m_0[n]) * (m - n);
            int L = L_14 / m_calSimpleFPGA;
            udpData[40 + offset_block4 + seq * 10] = L / 256;
            udpData[41 + offset_block4 + seq * 10] = L % 256;
            udpData[42 + offset_block4 + seq * 10] = 0;
          }
        }
        if (L_22 > L_11 && pulse_22 > pulse_12 * Pulse_threshold) {
          float dc = L_22 - L_11;
          if (dc >= startdistance) {
            int n = round((dc - startdistance) / ds);
            float m = round((dc - startdistance) / ds * 100) / 100;
            if (n <= 58) {
              L_22 -= bcz[n] + (bcz[n + 1] - bcz[n]) * (m - n);
              int L = L_22 / m_calSimpleFPGA;
              udpData[43 + offset_block2 + seq * 10] = L / 256;
              udpData[44 + offset_block2 + seq * 10] = L % 256;
              udpData[45 + offset_block2 + seq * 10] = 0;
            }
          }
        }
        if (L_23 > L_11 && pulse_23 > pulse_13 * Pulse_threshold) {
          float dc = L_23 - L_11;
          if (dc >= startdistance) {
            int n = round((dc - startdistance) / ds);
            float m = round((dc - startdistance) / ds * 100) / 100;
            if (n <= 58) {
              L_23 -= bcz[n] + (bcz[n + 1] - bcz[n]) * (m - n);
              int L = L_23 / m_calSimpleFPGA;
              udpData[43 + offset_block3 + seq * 10] = L / 256;
              udpData[44 + offset_block3 + seq * 10] = L % 256;
              udpData[45 + offset_block3 + seq * 10] = 0;
            }
          }
        }
        if (L_24 > L_11 && pulse_24 > pulse_14 * Pulse_threshold) {
          float dc = L_24 - L_11;
          if (dc >= startdistance) {
            int n = round((dc - startdistance) / ds);
            float m = round((dc - startdistance) / ds * 100) / 100;
            if (n <= 58) {
              L_24 -= bcz[n] + (bcz[n + 1] - bcz[n]) * (m - n);
              int L = L_24 / m_calSimpleFPGA;
              udpData[43 + offset_block4 + seq * 10] = L / 256;
              udpData[44 + offset_block4 + seq * 10] = L % 256;
              udpData[45 + offset_block4 + seq * 10] = 0;
            }
          }
        }
      }
      if (L_12 > 0 && pulse_12 > pulse_11 && pulse_12 > pulse_13 &&
          pulse_12 > pulse_14 && pulse_12 > pulseyz) {
        if (L_11 > L_12 && pulse_11 > pulse_21) {
          float dc = L_11 - L_12;
          int n = round((dc) / ds);
          float m = round((dc) / ds * 100) / 100;
          if (n <= 18) {
            L_11 -= m_0[n] + (m_0[n + 1] - m_0[n]) * (m - n);
            int L = L_11 / m_calSimpleFPGA;
            udpData[40 + offset_block1 + seq * 10] = L / 256;
            udpData[41 + offset_block1 + seq * 10] = L % 256;
            udpData[42 + offset_block1 + seq * 10] = 0;
          }
        }
        if (L_13 > L_12 && pulse_13 > pulse_23) {
          float dc = L_13 - L_12;
          int n = round((dc) / ds);
          float m = round((dc) / ds * 100) / 100;
          if (n <= 18) {
            L_13 -= m_0[n] + (m_0[n + 1] - m_0[n]) * (m - n);
            int L = L_13 / m_calSimpleFPGA;
            udpData[40 + offset_block3 + seq * 10] = L / 256;
            udpData[41 + offset_block3 + seq * 10] = L % 256;
            udpData[42 + offset_block3 + seq * 10] = 0;
          }
        }
        if (L_14 > L_12 && pulse_14 > pulse_24) {
          float dc = L_14 - L_12;
          int n = round((dc) / ds);
          float m = round((dc) / ds * 100) / 100;
          if (n <= 18) {
            L_14 -= m_0[n] + (m_0[n + 1] - m_0[n]) * (m - n);
            int L = L_14 / m_calSimpleFPGA;
            udpData[40 + offset_block4 + seq * 10] = L / 256;
            udpData[41 + offset_block4 + seq * 10] = L % 256;
            udpData[42 + offset_block4 + seq * 10] = 0;
          }
        }
        if (L_21 > L_12 && pulse_21 > pulse_11 * Pulse_threshold) {
          float dc = L_21 - L_12;
          if (dc >= startdistance) {
            int n = round((dc - startdistance) / ds);
            float m = round((dc - startdistance) / ds * 100) / 100;
            if (n <= 58) {
              L_21 -= bcz[n] + (bcz[n + 1] - bcz[n]) * (m - n);
              int L = L_21 / m_calSimpleFPGA;
              udpData[43 + offset_block1 + seq * 10] = L / 256;
              udpData[44 + offset_block1 + seq * 10] = L % 256;
              udpData[45 + offset_block1 + seq * 10] = 0;
            }
          }
        }
        if (L_23 > L_12 && pulse_23 > pulse_13 * Pulse_threshold) {
          float dc = L_23 - L_12;
          if (dc >= startdistance) {
            int n = round((dc - startdistance) / ds);
            float m = round((dc - startdistance) / ds * 100) / 100;
            if (n <= 58) {
              L_23 -= bcz[n] + (bcz[n + 1] - bcz[n]) * (m - n);
              int L = L_23 / m_calSimpleFPGA;
              udpData[43 + offset_block3 + seq * 10] = L / 256;
              udpData[44 + offset_block3 + seq * 10] = L % 256;
              udpData[45 + offset_block3 + seq * 10] = 0;
            }
          }
        }
        if (L_24 > L_12 && pulse_24 > pulse_14 * Pulse_threshold) {
          float dc = L_24 - L_12;
          if (dc >= startdistance) {
            int n = round((dc - startdistance) / ds);
            float m = round((dc - startdistance) / ds * 100) / 100;
            if (n <= 58) {
              L_24 -= bcz[n] + (bcz[n + 1] - bcz[n]) * (m - n);
              int L = L_24 / m_calSimpleFPGA;
              udpData[43 + offset_block4 + seq * 10] = L / 256;
              udpData[44 + offset_block4 + seq * 10] = L % 256;
              udpData[45 + offset_block4 + seq * 10] = 0;
            }
          }
        }
      }
      if (L_13 > 0 && pulse_13 > pulse_11 && pulse_13 > pulse_12 &&
          pulse_13 > pulse_14 && pulse_13 > pulseyz) {
        if (L_11 > L_13 && pulse_11 > pulse_21) {
          float dc = L_11 - L_13;
          int n = round((dc) / ds);
          float m = round((dc) / ds * 100) / 100;
          if (n <= 18) {
            L_11 -= m_0[n] + (m_0[n + 1] - m_0[n]) * (m - n);
            int L = L_11 / m_calSimpleFPGA;
            udpData[40 + offset_block1 + seq * 10] = L / 256;
            udpData[41 + offset_block1 + seq * 10] = L % 256;
            udpData[42 + offset_block1 + seq * 10] = 0;
          }
        }
        if (L_12 > L_13 && pulse_12 > pulse_22) {
          float dc = L_12 - L_13;
          int n = round((dc) / ds);
          float m = round((dc) / ds * 100) / 100;
          if (n <= 18) {
            L_12 -= m_0[n] + (m_0[n + 1] - m_0[n]) * (m - n);
            int L = L_12 / m_calSimpleFPGA;
            udpData[40 + offset_block2 + seq * 10] = L / 256;
            udpData[41 + offset_block2 + seq * 10] = L % 256;
            udpData[42 + offset_block2 + seq * 10] = 0;
          }
        }
        if (L_14 > L_13 && pulse_14 > pulse_24) {
          float dc = L_14 - L_13;
          int n = round((dc) / ds);
          float m = round((dc) / ds * 100) / 100;
          if (n <= 18) {
            L_14 -= m_0[n] + (m_0[n + 1] - m_0[n]) * (m - n);
            int L = L_14 / m_calSimpleFPGA;
            udpData[40 + offset_block4 + seq * 10] = L / 256;
            udpData[41 + offset_block4 + seq * 10] = L % 256;
            udpData[42 + offset_block4 + seq * 10] = 0;
          }
        }
        if (L_21 > L_13 && pulse_21 > pulse_11 * Pulse_threshold) {
          float dc = L_21 - L_13;
          if (dc >= startdistance) {
            int n = round((dc - startdistance) / ds);
            float m = round((dc - startdistance) / ds * 100) / 100;
            if (n <= 58) {
              L_21 -= bcz[n] + (bcz[n + 1] - bcz[n]) * (m - n);
              int L = L_21 / m_calSimpleFPGA;
              udpData[43 + offset_block1 + seq * 10] = L / 256;
              udpData[44 + offset_block1 + seq * 10] = L % 256;
              udpData[45 + offset_block1 + seq * 10] = 0;
            }
          }
        }
        if (L_22 > L_13 && pulse_22 > pulse_12 * Pulse_threshold) {
          float dc = L_22 - L_13;
          if (dc >= startdistance) {
            int n = round((dc - startdistance) / ds);
            float m = round((dc - startdistance) / ds * 100) / 100;
            if (n <= 58) {
              L_22 -= bcz[n] + (bcz[n + 1] - bcz[n]) * (m - n);
              int L = L_22 / m_calSimpleFPGA;
              udpData[43 + offset_block2 + seq * 10] = L / 256;
              udpData[44 + offset_block2 + seq * 10] = L % 256;
              udpData[45 + offset_block2 + seq * 10] = 0;
            }
          }
        }
        if (L_24 > L_13 && pulse_24 > pulse_14 * Pulse_threshold) {
          float dc = L_24 - L_13;
          if (dc >= startdistance) {
            int n = round((dc - startdistance) / ds);
            float m = round((dc - startdistance) / ds * 100) / 100;
            if (n <= 58) {
              L_24 -= bcz[n] + (bcz[n + 1] - bcz[n]) * (m - n);
              int L = L_24 / m_calSimpleFPGA;
              udpData[43 + offset_block4 + seq * 10] = L / 256;
              udpData[44 + offset_block4 + seq * 10] = L % 256;
              udpData[45 + offset_block4 + seq * 10] = 0;
            }
          }
        }
      }
      if (L_14 > 0 && pulse_14 > pulse_11 && pulse_14 > pulse_12 &&
          pulse_14 > pulse_13 && pulse_14 > pulseyz) {
        if (L_11 > L_14 && pulse_11 > pulse_21) {
          float dc = L_11 - L_14;
          int n = round((dc) / ds);
          float m = round((dc) / ds * 100) / 100;
          if (n <= 18) {
            L_11 -= m_0[n] + (m_0[n + 1] - m_0[n]) * (m - n);
            int L = L_11 / m_calSimpleFPGA;
            udpData[40 + offset_block1 + seq * 10] = L / 256;
            udpData[41 + offset_block1 + seq * 10] = L % 256;
            udpData[42 + offset_block1 + seq * 10] = 0;
          }
        }
        if (L_12 > L_14 && pulse_12 > pulse_22) {
          float dc = L_12 - L_14;
          int n = round((dc) / ds);
          float m = round((dc) / ds * 100) / 100;
          if (n <= 18) {
            L_12 -= m_0[n] + (m_0[n + 1] - m_0[n]) * (m - n);
            int L = L_12 / m_calSimpleFPGA;
            udpData[40 + offset_block2 + seq * 10] = L / 256;
            udpData[41 + offset_block2 + seq * 10] = L % 256;
            udpData[42 + offset_block2 + seq * 10] = 0;
          }
        }
        if (L_13 > L_14 && pulse_13 > pulse_23) {
          float dc = L_13 - L_14;
          int n = round((dc) / ds);
          float m = round((dc) / ds * 100) / 100;
          if (n <= 18) {
            L_13 -= m_0[n] + (m_0[n + 1] - m_0[n]) * (m - n);
            int L = L_13 / m_calSimpleFPGA;
            udpData[40 + offset_block3 + seq * 10] = L / 256;
            udpData[41 + offset_block3 + seq * 10] = L % 256;
            udpData[42 + offset_block3 + seq * 10] = 0;
          }
        }
        if (L_21 > L_14 && pulse_21 > pulse_11 * Pulse_threshold) {
          float dc = L_21 - L_14;
          if (dc >= startdistance) {
            int n = round((dc - startdistance) / ds);
            float m = round((dc - startdistance) / ds * 100) / 100;
            if (n <= 58) {
              L_21 -= bcz[n] + (bcz[n + 1] - bcz[n]) * (m - n);
              int L = L_21 / m_calSimpleFPGA;
              udpData[43 + offset_block1 + seq * 10] = L / 256;
              udpData[44 + offset_block1 + seq * 10] = L % 256;
              udpData[45 + offset_block1 + seq * 10] = 0;
            }
          }
        }
        if (L_22 > L_14 && pulse_22 > pulse_12 * Pulse_threshold) {
          float dc = L_22 - L_14;
          if (dc >= startdistance) {
            int n = round((dc - startdistance) / ds);
            float m = round((dc - startdistance) / ds * 100) / 100;
            if (n <= 58) {
              L_22 -= bcz[n] + (bcz[n + 1] - bcz[n]) * (m - n);
              int L = L_22 / m_calSimpleFPGA;
              udpData[43 + offset_block2 + seq * 10] = L / 256;
              udpData[44 + offset_block2 + seq * 10] = L % 256;
              udpData[45 + offset_block2 + seq * 10] = 0;
            }
          }
        }
        if (L_23 > L_14 && pulse_23 > pulse_13 * Pulse_threshold) {
          float dc = L_23 - L_14;
          if (dc >= startdistance) {
            int n = round((dc - startdistance) / ds);
            float m = round((dc - startdistance) / ds * 100) / 100;
            if (n <= 58) {
              L_23 -= bcz[n] + (bcz[n + 1] - bcz[n]) * (m - n);
              int L = L_23 / m_calSimpleFPGA;
              udpData[43 + offset_block3 + seq * 10] = L / 256;
              udpData[44 + offset_block3 + seq * 10] = L % 256;
              udpData[45 + offset_block3 + seq * 10] = 0;
            }
          }
        }
      }
    }
  }
}

void LidarDevice::UseDecodeScope256Depth(const char *udpData,
                                         TWPointCloud::Points &points,
                                         unsigned int *sec, unsigned int *usec,
                                         float *maxAngle) {
  UseDecodeTensor48Depth(udpData, points, sec, usec, maxAngle);
}

void LidarDevice::UseDecodeFocusB1(const char *udpData,
                                   TWPointCloud::Points &points,
                                   unsigned int *sec, unsigned int *usec,
                                   float *maxAngle) {
  double horizontalAngle = 0;
  // face id
  unsigned short mirror = 0;
  double Nx = 0.0, Ny = 0.0, Nz = 0.0;
  double theta;

  for (int blocks_num = 0; blocks_num < 8; blocks_num++) {
    int offset = blocks_num * 140;

    // horizontal angle index: 128-131
    // horizontalAngle表示点云水平角度(φ)
    int HextoAngle = FourHexToInt(udpData[offset + 128], udpData[offset + 129],
                                  udpData[offset + 130], udpData[offset + 131]);
    horizontalAngle = HextoAngle * 0.00001;

    unsigned char hexMirror = udpData[offset + 136];
    hexMirror = hexMirror << 2;
    mirror = hexMirror >> 6;

    horizontalAngle += m_MirrorHorAngleOffset[mirror];

    // offset angle m_skewing_scopeMiniA2_angle
    unsigned char hexACount = udpData[offset + 136];
    hexACount = hexACount << 4;
    unsigned short uACount = hexACount >> 4;

    // calculate
    if (mirror < 3) {
      m_skewing_Focus_Angle_Correct[mirror] =
          uACount * 0.04 - m_skewing_Focus_Angle_Offset[mirror];
      m_skewing_sin_Focus[mirror] = sin((m_skewing_Focus_Angle_Correct[mirror] +
                                         m_MirrorVerAngleOffset[mirror]) *
                                        m_calRA);
      m_skewing_cos_Focus[mirror] = cos((m_skewing_Focus_Angle_Correct[mirror] +
                                         m_MirrorVerAngleOffset[mirror]) *
                                        m_calRA);
    }

    unsigned short faceIndex = mirror;

    // 转镜法线转过的角度 θ = φ / 2
    theta = (720 - horizontalAngle) * m_calRA / 2;

    // Nx = cosδcosθ
    // Ny = -cosδsinθ
    // Nz = sinδ
    Nx = m_skewing_cos_Focus[faceIndex] * cos(theta);
    Ny = -m_skewing_cos_Focus[faceIndex] * sin(theta);
    Nz = m_skewing_sin_Focus[faceIndex];

    unsigned int blockSecond =
        FourHexToInt(udpData[offset + 132], udpData[offset + 133],
                     udpData[offset + 134], udpData[offset + 135]);
    unsigned char hexBlockMicrosecond = udpData[offset + 137];
    hexBlockMicrosecond = hexBlockMicrosecond & 0x0F;
    unsigned int blockMicrosecond =
        FourHexToInt(0x00, hexBlockMicrosecond, udpData[offset + 138],
                     udpData[offset + 139]);

    if (_enable_time_windows) {

      bool isFakeUdp = false; // 从udpData读取证明是否为空包

      if (mirror == 3)
        isFakeUdp = true;

      if (isFakeUdp) // 空包就只要时间戳
      {
        TWPoint basic_point;
        if (_is_lidar_time) {
          basic_point.t_sec = blockSecond;
          basic_point.t_usec = blockMicrosecond;
        } else {
          basic_point.t_sec = *sec;
          basic_point.t_usec = *usec;
        }
        points.push_back(std::move(basic_point));
        continue;
      }
    }

    PRE_LOAD_PCAP_FILE_CALCULATE

    for (int seq = 0; seq < 16; seq++) {
      double hexToInt1 = TwoHextoInt(udpData[offset + seq * 8 + 0],
                                     udpData[offset + seq * 8 + 1]);
      double hexPulse1 = TwoHextoInt(udpData[offset + seq * 8 + 2],
                                     udpData[offset + seq * 8 + 3]);
      double L_1 = hexToInt1 * m_calSimpleFPGA;
      // int intensity1 =
      //    (hexPulse1 * m_calPulseFPGA) / ScopePulseMapValue * 255.0 + 0.5;
      // double pulse_1 = intensity1 > 255 ? 255 : intensity1;

      double pulse_1 = hexPulse1 * m_calPulseFPGA;

      double hexToInt2 = TwoHextoInt(udpData[offset + seq * 8 + 4],
                                     udpData[offset + seq * 8 + 5]);
      double hexPulse2 = TwoHextoInt(udpData[offset + seq * 8 + 6],
                                     udpData[offset + seq * 8 + 7]);
      double L_2 = hexToInt2 * m_calSimpleFPGA;

      double pulse_2 = hexPulse2 * m_calPulseFPGA;

      int channel =
          65 - (16 * (blocks_num >= 4 ? blocks_num - 4 : blocks_num) + seq + 1);

      TWPoint basic_point;
      basic_point.angle = horizontalAngle;
      basic_point.mirror = mirror;
      basic_point.channel = channel;
      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }

      // 计算各通道竖直角度（β）的正弦和余弦
      double cos_beta = m_verticalChannelAngle_Focus_cos_vA_RA[channel - 1];
      double sin_beta = m_verticalChannelAngle_Focus_sin_vA_RA[channel - 1];

      // 机芯发射光线单位向量li = (-cosβ,0,sinβ)
      double Lx = -cos_beta;
      double Ly = 0;
      double Lz = sin_beta;

      // 计算笛卡尔坐标X,Y,Z
      double dot_product = Lx * Nx + Ly * Ny + Lz * Nz;
      if (_echoNum & Echo1) {
        basic_point.x = L_1 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_1 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_1 * (Lz - 2 * dot_product * Nz);

        basic_point.echo = 1;
        basic_point.distance = L_1;
        basic_point.pulse = pulse_1;
        basic_point.intensity = pulseToIntensity(pulse_1);
        points.push_back(std::move(basic_point));
      }

      if (_echoNum & Echo2) {
        basic_point.x = L_2 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_2 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_2 * (Lz - 2 * dot_product * Nz);

        basic_point.echo = 2;
        basic_point.distance = L_2;
        basic_point.pulse = pulse_2;
        basic_point.intensity = pulseToIntensity(pulse_2);
        points.push_back(std::move(basic_point));
      }
    }
  }
}

void LidarDevice::UseDecodeTempoA4(const char *udpData,
                                   TWPointCloud::Points &points,
                                   unsigned int *sec, unsigned int *usec,
                                   float *maxAngle) {
  double horizontalAngle = 0;
  unsigned short mirror = 0;
  double Nx = 0.0, Ny = 0.0, Nz = 0.0;

  unsigned int frameSecond =
      FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
  double frameMicrosecond =
      FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;
  int frame_cycle_cout = TwoHextoInt(udpData[4], udpData[5]);

  for (int blocks_num = 0; blocks_num < 8; blocks_num++) {
    int offset = blocks_num * 164;

    // ptp
    int offsetMicrosecond =
        TwoHextoInt(udpData[32 + offset], udpData[33 + offset]);
    double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
    unsigned int blockSecond =
        (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
    unsigned int blockMicrosecond =
        (totalMicrosecond >= 1000000)
            ? (unsigned int)(totalMicrosecond - 1000000)
            : (unsigned int)totalMicrosecond;

    unsigned char hexMirror = udpData[35 + offset];
    hexMirror = hexMirror << 5;
    mirror = hexMirror >> 6;
    unsigned short faceIndex = mirror;

    for (int seq = 0; seq < 16; seq++) {
      // horizontalAngle表示点云水平角度(φ)
      int HextoAngle = TwoHextoInt(udpData[36 + offset + seq * 10],
                                   udpData[37 + offset + seq * 10]);
      horizontalAngle = HextoAngle * 0.01;
      horizontalAngle += m_MirrorHorAngleOffset[mirror];

      // 点云径向距离L_1
      double hexToInt1 = TwoHextoInt(udpData[38 + offset + seq * 10],
                                     udpData[39 + offset + seq * 10]);
      double L_1 = hexToInt1 * m_calSimpleFPGA;

      unsigned char hexChar1 = udpData[40 + offset + seq * 10];
      double intensity_1 = ((double)hexChar1);

      double confidence1 = udpData[41 + offset + seq * 10];

      // 点云径向距离L_2
      double hexToInt2 = TwoHextoInt(udpData[42 + offset + seq * 10],
                                     udpData[43 + offset + seq * 10]);
      double L_2 = hexToInt2 * m_calSimpleFPGA;

      unsigned char hexChar2 = udpData[44 + offset + seq * 10];
      double intensity_2 = ((double)hexChar2);

      double confidence2 = udpData[45 + offset + seq * 10];

      int channel =
          16 * (blocks_num >= 4 ? blocks_num - 4 : blocks_num) + seq + 1;

      TWPoint basic_point;
      basic_point.angle = horizontalAngle;
      basic_point.mirror = mirror;
      basic_point.channel = channel;
      basic_point.cycle_count = frame_cycle_cout;

      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }

      // 计算各通道竖直角度（β）的正弦和余弦
      double cos_beta =
          m_verticalChannelAngle_ScopeMiniA2_cos_vA_RA[channel - 1];
      double sin_beta =
          m_verticalChannelAngle_ScopeMiniA2_sin_vA_RA[channel - 1];

      // 机芯发射光线单位向量 li = (-cosβ,0,sinβ)
      double Lx = -cos_beta;
      double Ly = 0;
      double Lz = sin_beta;

      // 转镜法线转过的角度 θ = φ / 2
      double theta = horizontalAngle * m_calRA / 2;

      // Nx = cosδcosθ
      // Ny = cosδsinθ
      // Nz = sinδ
      Nx = m_skewing_cos_tempoA4[faceIndex] * cos(theta);
      Ny = m_skewing_cos_tempoA4[faceIndex] * sin(theta);
      Nz = m_skewing_sin_tempoA4[faceIndex];

      double dot_product = Lx * Nx + Ly * Ny + Lz * Nz;

      // 计算笛卡尔坐标X,Y,Z
      if (_echoNum & Echo1) {
        basic_point.x = L_1 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_1 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_1 * (Lz - 2 * dot_product * Nz);

        basic_point.echo = 1;
        basic_point.distance = L_1;
        basic_point.intensity = intensity_1;
        basic_point.confidence = confidence1;
        points.push_back(std::move(basic_point));
      }

      if (_echoNum & Echo2) {
        basic_point.x = L_2 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_2 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_2 * (Lz - 2 * dot_product * Nz);

        basic_point.echo = 2;
        basic_point.distance = L_2;
        basic_point.intensity = intensity_2;
        basic_point.confidence = confidence2;
        points.push_back(std::move(basic_point));
      }
    }
  }
}

void LidarDevice::UseDecodeTempoA4Calib(const char *udpData,
                                        TWPointCloud::Points &points,
                                        unsigned int *sec, unsigned int *usec,
                                        float *maxAngle) {
  double horizontalAngle = 0;
  unsigned short mirror = 0;
  double Nx = 0.0, Ny = 0.0, Nz = 0.0;

  for (int blocks_num = 0; blocks_num < 6; blocks_num++) {
    int offset = blocks_num * 196;

    unsigned int frameSecond =
        FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
    double frameMicrosecond =
        FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;

    // ptp
    int offsetMicrosecond =
        TwoHextoInt(udpData[32 + offset], udpData[33 + offset]);
    double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
    unsigned int blockSecond =
        (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
    unsigned int blockMicrosecond =
        (totalMicrosecond >= 1000000)
            ? (unsigned int)(totalMicrosecond - 1000000)
            : (unsigned int)totalMicrosecond;

    int blockFlag = (udpData[35 + offset] & 0xF0) >> 4;

    unsigned char hexMirror = udpData[35 + offset];
    hexMirror = hexMirror << 5;
    mirror = hexMirror >> 6;
    unsigned short faceIndex = mirror;

    // horizontalAngle表示点云水平角度(φ)
    int HextoAngle = TwoHextoInt(udpData[36 + offset], udpData[37 + offset]);
    horizontalAngle = HextoAngle * 0.01;

    PRE_LOAD_PCAP_FILE_CALCULATE

    for (int seq = 0; seq < 16; seq++) {
      // 重新解析每个点的水平角度(φ)
      int HextoAngle = TwoHextoInt(udpData[36 + offset + seq * 12],
                                   udpData[37 + offset + seq * 12]);
      horizontalAngle = HextoAngle * 0.01;
      horizontalAngle += m_MirrorHorAngleOffset[mirror];

      // 点云径向距离L_1
      double hexToInt1 = TwoHextoInt(udpData[38 + offset + seq * 12],
                                     udpData[39 + offset + seq * 12]);
      double L_1 = hexToInt1 * m_calSimpleFPGA;

      unsigned char hexChar1 = udpData[40 + offset + seq * 12];
      double intensity_1 = ((double)hexChar1);

      double pulse_1 = OneHexHalfHextoInt(udpData[41 + offset + seq * 12],
                                          udpData[42 + offset + seq * 12]) *
                       0.032;

      double confidence1 = udpData[42 + offset + seq * 12] & 0x01;

      // 点云径向距离L_2
      double hexToInt2 = TwoHextoInt(udpData[43 + offset + seq * 12],
                                     udpData[44 + offset + seq * 12]);
      double L_2 = hexToInt2 * m_calSimpleFPGA;

      unsigned char hexChar2 = udpData[45 + offset + seq * 12];
      double intensity_2 = ((double)hexChar2);

      double pulse_2 = OneHexHalfHextoInt(udpData[46 + offset + seq * 12],
                                          udpData[47 + offset + seq * 12]) *
                       0.032;

      double confidence2 = udpData[47 + offset + seq * 12] & 0x01;

      int channel = 16 * blockFlag + seq + 1;

      TWPoint basic_point;
      basic_point.angle = horizontalAngle;
      basic_point.mirror = mirror;
      basic_point.channel = channel;
      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }

      // 计算各通道竖直角度（β）的正弦和余弦
      double cos_beta =
          m_verticalChannelAngle_ScopeMiniA2_cos_vA_RA[channel - 1];
      double sin_beta =
          m_verticalChannelAngle_ScopeMiniA2_sin_vA_RA[channel - 1];

      // 机芯发射光线单位向量 li = (-cosβ,0,sinβ)
      double Lx = -cos_beta;
      double Ly = 0;
      double Lz = sin_beta;

      // 转镜法线转过的角度 θ = φ / 2
      double theta = horizontalAngle * m_calRA / 2;

      // Nx = cosδcosθ
      // Ny = cosδsinθ
      // Nz = sinδ
      Nx = m_skewing_cos_tempoA4[faceIndex] * cos(theta);
      Ny = m_skewing_cos_tempoA4[faceIndex] * sin(theta);
      Nz = m_skewing_sin_tempoA4[faceIndex];

      // 计算笛卡尔坐标X,Y,Z
      double dot_product = Lx * Nx + Ly * Ny + Lz * Nz;

      if (_echoNum & Echo1) {
        basic_point.x = L_1 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_1 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_1 * (Lz - 2 * dot_product * Nz);

        basic_point.echo = 1;
        basic_point.distance = L_1;
        basic_point.pulse = pulse_1;
        basic_point.intensity = intensity_1;
        basic_point.confidence = confidence1;
        points.push_back(std::move(basic_point));
      }

      if (_echoNum & Echo2) {
        basic_point.x = L_2 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_2 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_2 * (Lz - 2 * dot_product * Nz);

        basic_point.echo = 2;
        basic_point.distance = L_2;
        basic_point.pulse = pulse_2;
        basic_point.intensity = intensity_2;
        basic_point.confidence = confidence2;
        points.push_back(std::move(basic_point));
      }
    }
  }
}

void LidarDevice::UseDecodeFocusB2(const char *udpData,
                                   TWPointCloud::Points &points,
                                   unsigned int *sec, unsigned int *usec,
                                   float *maxAngle) {
  double horizontalAngle = 0;
  unsigned short mirror = 0;
  double Nx = 0.0, Ny = 0.0, Nz = 0.0;

  unsigned int frameSecond =
      FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
  double frameMicrosecond =
      FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;

  int frame_cycle_cout = TwoHextoInt(udpData[4], udpData[5]);

  if (_enable_time_windows) {
    const unsigned char *data =
        reinterpret_cast<const unsigned char *>(udpData); // 避免符号扩展风险
    bool isFakeUdp = false; // 从udpData读取证明是否为空包

    if (data[0] == 0xEB && data[1] == 0x90 && data[2] == 0xCB &&
        data[3] == 0x55)
      isFakeUdp = true;

    if (isFakeUdp) // 空包就只要时间戳
    {
      TWPoint basic_point;
      unsigned int blockSecond =
          (frameMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
      unsigned int blockMicrosecond =
          (frameMicrosecond >= 1000000)
              ? (unsigned int)(frameMicrosecond - 1000000)
              : (unsigned int)frameMicrosecond;

      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }
      points.push_back(std::move(basic_point));
      return;
    }
  }

  for (int blocks_num = 0; blocks_num < 8; blocks_num++) {
    int offset = blocks_num * 164;

    int offsetMicrosecond =
        TwoHextoInt(udpData[32 + offset], udpData[33 + offset]);
    // ptp
    double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
    unsigned int blockSecond =
        (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
    unsigned int blockMicrosecond =
        (totalMicrosecond >= 1000000)
            ? (unsigned int)(totalMicrosecond - 1000000)
            : (unsigned int)totalMicrosecond;

    unsigned char hexMirror = udpData[35 + offset];
    hexMirror = hexMirror << 5;
    mirror = hexMirror >> 6;
    unsigned short faceIndex = mirror;

    for (int seq = 0; seq < 16; seq++) {
      // horizontalAngle表示点云水平角度(φ)
      int HextoAngle = TwoHextoInt(udpData[36 + offset + seq * 10],
                                   udpData[37 + offset + seq * 10]);
      horizontalAngle = HextoAngle * 0.01;
      horizontalAngle += m_MirrorHorAngleOffset[mirror];

      // 点云径向距离L_1
      double hexToInt1 = TwoHextoInt(udpData[38 + offset + seq * 10],
                                     udpData[39 + offset + seq * 10]);
      double L_1 = hexToInt1 * m_calSimpleFPGA;

      unsigned char hexChar1 = udpData[40 + offset + seq * 10];
      double intensity_1 = ((double)hexChar1);

      double confidence1 = udpData[41 + offset + seq * 10];

      // 点云径向距离L_2
      double hexToInt2 = TwoHextoInt(udpData[42 + offset + seq * 10],
                                     udpData[43 + offset + seq * 10]);
      double L_2 = hexToInt2 * m_calSimpleFPGA;

      unsigned char hexChar2 = udpData[44 + offset + seq * 10];
      double intensity_2 = ((double)hexChar2);

      double confidence2 = udpData[45 + offset + seq * 10];

      int channel =
          16 * (blocks_num >= 4 ? blocks_num - 4 : blocks_num) + seq + 1;

      TWPoint basic_point;
      basic_point.angle = horizontalAngle;
      basic_point.mirror = mirror;
      basic_point.channel = channel;
      basic_point.cycle_count = frame_cycle_cout;

      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }

      // 计算各通道竖直角度（β）的正弦和余弦
      double cos_beta = m_verticalChannelAngle_FocusB2_cos_vA_RA[channel - 1];
      double sin_beta = m_verticalChannelAngle_FocusB2_sin_vA_RA[channel - 1];

      // 机芯发射光线单位向量 li = (-cosβ,0,sinβ)
      double Lx = -cos_beta;
      double Ly = 0;
      double Lz = sin_beta;

      // 转镜法线转过的角度 θ = φ / 2
      double theta = horizontalAngle * m_calRA / 2;

      // Nx = cosδcosθ
      // Ny = cosδsinθ
      // Nz = sinδ
      Nx = m_skewing_cos_FocusB2[faceIndex] * cos(theta);
      Ny = m_skewing_cos_FocusB2[faceIndex] * sin(theta);
      Nz = m_skewing_sin_FocusB2[faceIndex];

      double dot_product = Lx * Nx + Ly * Ny + Lz * Nz;

      int apd_temp = (udpData[23] & 0x7F);
      if (udpData[23] & 0x80)
        apd_temp = -apd_temp;

      // 计算笛卡尔坐标X,Y,Z
      if (_echoNum & Echo1) {
        basic_point.x = L_1 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_1 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_1 * (Lz - 2 * dot_product * Nz);

        basic_point.echo = 1;
        basic_point.distance = L_1;
        basic_point.intensity = intensity_1;
        basic_point.confidence = confidence1;
        basic_point.encoding_interval = _encoding_interval;
        basic_point.state = 0;
        basic_point.label = 0;
        basic_point.apd_temp = apd_temp;
        basic_point.pulseCodeInterval = udpData[24] * 10;
        points.push_back(std::move(basic_point));
      }

      if (_echoNum & Echo2) {
        basic_point.x = L_2 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_2 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_2 * (Lz - 2 * dot_product * Nz);

        basic_point.echo = 2;
        basic_point.distance = L_2;
        basic_point.intensity = intensity_2;
        basic_point.confidence = confidence2;
        basic_point.encoding_interval = _encoding_interval;
        basic_point.state = 0;
        basic_point.label = 0;
        basic_point.apd_temp = apd_temp;
        basic_point.pulseCodeInterval = udpData[24] * 10;
        points.push_back(std::move(basic_point));
      }
    }
  }
}

void LidarDevice::UseDecodeFocusB2Calib(const char *udpData,
                                        TWPointCloud::Points &points,
                                        unsigned int *sec, unsigned int *usec,
                                        float *maxAngle) {
  double horizontalAngle = 0;
  unsigned short mirror = 0;
  double Nx = 0.0, Ny = 0.0, Nz = 0.0;
  for (int blocks_num = 0; blocks_num < 6; blocks_num++) {
    int offset = blocks_num * 196;

    unsigned int frameSecond =
        FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
    double frameMicrosecond =
        FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;

    // ptp
    int offsetMicrosecond =
        TwoHextoInt(udpData[32 + offset], udpData[33 + offset]);
    double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
    unsigned int blockSecond =
        (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
    unsigned int blockMicrosecond =
        (totalMicrosecond >= 1000000)
            ? (unsigned int)(totalMicrosecond - 1000000)
            : (unsigned int)totalMicrosecond;

    int blockFlag = (udpData[35 + offset] & 0xF0) >> 4;

    unsigned char hexMirror = udpData[35 + offset];
    hexMirror = hexMirror << 5;
    mirror = hexMirror >> 6;
    unsigned short faceIndex = mirror;

    // horizontalAngle表示点云水平角度(φ)
    int HextoAngle = TwoHextoInt(udpData[36 + offset], udpData[37 + offset]);
    horizontalAngle = HextoAngle * 0.01;

    PRE_LOAD_PCAP_FILE_CALCULATE

    for (int seq = 0; seq < 16; seq++) {
      // 重新解析每个点的水平角度(φ)
      int HextoAngle = TwoHextoInt(udpData[36 + offset + seq * 12],
                                   udpData[37 + offset + seq * 12]);
      horizontalAngle = HextoAngle * 0.01;
      horizontalAngle += m_MirrorHorAngleOffset[mirror];

      // 点云径向距离L_1
      double hexToInt1 = TwoHextoInt(udpData[38 + offset + seq * 12],
                                     udpData[39 + offset + seq * 12]);
      double L_1 = hexToInt1 * m_calSimpleFPGA;

      unsigned char hexChar1 = udpData[40 + offset + seq * 12];
      double intensity_1 = ((double)hexChar1);

      double pulse_1 = OneHexHalfHextoInt(udpData[41 + offset + seq * 12],
                                          udpData[42 + offset + seq * 12]) *
                       0.032;

      double confidence1 = udpData[42 + offset + seq * 12] & 0x01;

      // 点云径向距离L_2
      double hexToInt2 = TwoHextoInt(udpData[43 + offset + seq * 12],
                                     udpData[44 + offset + seq * 12]);
      double L_2 = hexToInt2 * m_calSimpleFPGA;

      unsigned char hexChar2 = udpData[45 + offset + seq * 12];
      double intensity_2 = ((double)hexChar2);

      double pulse_2 = OneHexHalfHextoInt(udpData[46 + offset + seq * 12],
                                          udpData[47 + offset + seq * 12]) *
                       0.032;

      double confidence2 = udpData[47 + offset + seq * 12] & 0x01;

      int channel = 16 * blockFlag + seq + 1;

      TWPoint basic_point;
      basic_point.angle = horizontalAngle;
      basic_point.mirror = mirror;
      basic_point.channel = channel;
      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }

      // 计算各通道竖直角度（β）的正弦和余弦
      double cos_beta = m_verticalChannelAngle_FocusB2_cos_vA_RA[channel - 1];
      double sin_beta = m_verticalChannelAngle_FocusB2_sin_vA_RA[channel - 1];

      // 机芯发射光线单位向量 li = (-cosβ,0,sinβ)
      double Lx = -cos_beta;
      double Ly = 0;
      double Lz = sin_beta;

      // 转镜法线转过的角度 θ = φ / 2
      double theta = horizontalAngle * m_calRA / 2;

      // Nx = cosδcosθ
      // Ny = cosδsinθ
      // Nz = sinδ
      Nx = m_skewing_cos_FocusB2[faceIndex] * cos(theta);
      Ny = m_skewing_cos_FocusB2[faceIndex] * sin(theta);
      Nz = m_skewing_sin_FocusB2[faceIndex];

      // 计算笛卡尔坐标X,Y,Z
      double dot_product = Lx * Nx + Ly * Ny + Lz * Nz;

      int apd_temp = (udpData[23] & 0x7F);
      if (udpData[23] & 0x80)
        apd_temp = -apd_temp;

      if (_echoNum & Echo1) {
        basic_point.x = L_1 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_1 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_1 * (Lz - 2 * dot_product * Nz);

        basic_point.echo = 1;
        basic_point.distance = L_1;
        basic_point.pulse = pulse_1;
        basic_point.intensity = intensity_1;
        basic_point.confidence = confidence1;
        basic_point.encoding_interval = _encoding_interval;
        basic_point.state = 0;
        basic_point.label = 0;
        basic_point.apd_temp = apd_temp;
        basic_point.pulseCodeInterval = udpData[24] * 10;
        points.push_back(std::move(basic_point));
      }

      if (_echoNum & Echo2) {
        basic_point.x = L_2 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_2 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_2 * (Lz - 2 * dot_product * Nz);

        basic_point.echo = 2;
        basic_point.distance = L_2;
        basic_point.pulse = pulse_2;
        basic_point.intensity = intensity_2;
        basic_point.confidence = confidence2;
        basic_point.encoding_interval = _encoding_interval;
        basic_point.state = 0;
        basic_point.label = 0;
        basic_point.apd_temp = apd_temp;
        basic_point.pulseCodeInterval = udpData[24] * 10;
        points.push_back(std::move(basic_point));
      }
    }
  }
}

bool LidarDevice::UseDecodeFocusT(const char *udpData,
                                  TWPointCloud::Points &points,
                                  unsigned int *sec, unsigned int *usec,
                                  float *maxAngle) {
  PRE_LOAD_PCAP_FILE_CALCULATE_EX
  double horizontalAngle = 0;
  unsigned short mirror = 0;
  double Nx = 0.0, Ny = 0.0, Nz = 0.0;

  unsigned int frameSecond =
      FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
  double frameMicrosecond =
      FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;

  int frame_cycle_cout = TwoHextoInt(udpData[4], udpData[5]);
  int data_block = 10; // 每个data block 10字节
  for (int blocks_num = 0; blocks_num < 6; blocks_num++) {
    int offset = blocks_num * 164;

    int offsetMicrosecond =
        TwoHextoInt(udpData[32 + offset], udpData[33 + offset]);
    // ptp
    double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
    unsigned int blockSecond =
        (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
    unsigned int blockMicrosecond =
        (totalMicrosecond >= 1000000)
            ? (unsigned int)(totalMicrosecond - 1000000)
            : (unsigned int)totalMicrosecond;

    mirror = (udpData[35 + offset] & 0x06) >> 1;
    // unsigned short faceIndex = mirror;

    for (int seq = 0; seq < 16; seq++) {
      // horizontalAngle表示点云水平角度(φ)
      int HextoAngle = TwoHextoInt(udpData[36 + offset + seq * data_block],
                                   udpData[37 + offset + seq * data_block]);
      horizontalAngle = HextoAngle * 0.01;
      horizontalAngle += m_MirrorHorAngleOffset[mirror];

      // 点云径向距离L_1
      double hexToInt1 = TwoHextoInt(udpData[38 + offset + seq * data_block],
                                     udpData[39 + offset + seq * data_block]);
      double L_1 = hexToInt1 * m_calSimpleFPGA;

      unsigned char hexChar1 = udpData[40 + offset + seq * data_block];
      double intensity_1 = ((double)hexChar1);

      // double confidence1 = udpData[41 + offset + seq * 10];  //预留

      // 点云径向距离L_2
      double hexToInt2 = TwoHextoInt(udpData[42 + offset + seq * data_block],
                                     udpData[43 + offset + seq * data_block]);
      double L_2 = hexToInt2 * m_calSimpleFPGA;

      unsigned char hexChar2 = udpData[44 + offset + seq * data_block];
      double intensity_2 = ((double)hexChar2);

      // double confidence2 = udpData[45 + offset + seq * 10];

      int channel = 16 * blocks_num + seq + 1;
      int laser_column = 0;
      if ((26 <= channel && channel <= 40 && channel % 2 == 0) ||
          (57 <= channel && channel <= 64))
        laser_column = 1;
      else if ((9 <= channel && channel <= 23 && channel % 2 == 1) ||
               (41 <= channel && channel <= 55 && channel % 2 == 1) ||
               (65 <= channel && channel <= 72) ||
               (81 <= channel && channel <= 88))
        laser_column = 2;
      else if ((1 <= channel && channel <= 8) ||
               (25 <= channel && channel <= 39 && channel % 2 == 1) ||
               (73 <= channel && channel <= 80) ||
               (89 <= channel && channel <= 96))
        laser_column = 3;
      else if ((10 <= channel && channel <= 24 && channel % 2 == 0) ||
               (42 <= channel && channel <= 56 && channel % 2 == 0))
        laser_column = 4;
      double theta = 0; // 转镜法线转过的角度
      double alpha = m_alphaMap_FocusT.count(channel)
                         ? m_alphaMap_FocusT.at(channel)
                         : 0.0; // 修正角度

      double horizontalAngle_modify = 0;
      switch (laser_column) {
      case 1:
        theta = (horizontalAngle - 1.63 + alpha) * m_calRA / 2;
        horizontalAngle_modify = horizontalAngle - 1.63;
        break;
      case 2:
        theta = (horizontalAngle - 0.82 + alpha) * m_calRA / 2;
        horizontalAngle_modify = horizontalAngle - 0.82;
        break;
      case 3:
        theta = (horizontalAngle + alpha) * m_calRA / 2;
        horizontalAngle_modify = horizontalAngle;
        break;
      case 4:
        theta = (horizontalAngle + 0.82 + alpha) * m_calRA / 2;
        horizontalAngle_modify = horizontalAngle + 0.82;
        break;
      default:
        break;
      }

      TWPoint basic_point;
      basic_point.angle = horizontalAngle_modify; // 需要修正
      basic_point.mirror = mirror;
      basic_point.channel = channel;
      basic_point.cycle_count = frame_cycle_cout;

      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }

      // 计算各通道竖直角度（β）的正弦和余弦
      double cos_beta = m_verticalChannelAngle_FocusT_cos_vA_RA[channel - 1];
      double sin_beta = m_verticalChannelAngle_FocusT_sin_vA_RA[channel - 1];

      // 机芯发射光线单位向量 li = (-cosβ,0,sinβ)
      double Lx = -cos_beta;
      double Ly = 0;
      double Lz = sin_beta;

      // 转镜法线转过的角度 θ = φ / 2
      // double theta = horizontalAngle * m_calRA / 2;

      // Nx = cosδcosθ
      // Ny = cosδsinθ
      // Nz = sinδ
      Nx = m_skewing_cos_FocusT[mirror] * cos(theta);
      Ny = m_skewing_cos_FocusT[mirror] * sin(theta);
      Nz = m_skewing_sin_FocusT[mirror];

      double dot_product = Lx * Nx + Ly * Ny + Lz * Nz;

      // APD温度，背面点标定使用
      int apd_temp = (udpData[23] & 0x7F);
      if (udpData[23] & 0x80)
        apd_temp = -apd_temp;

      // 计算笛卡尔坐标X,Y,Z
      if (_echoNum & Echo1) {
        basic_point.x = L_1 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_1 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_1 * (Lz - 2 * dot_product * Nz);

        basic_point.echo = 1;
        basic_point.distance = L_1;
        basic_point.intensity = intensity_1;
        // basic_point.confidence = confidence1;
        basic_point.encoding_interval = _encoding_interval;
        basic_point.state = 0;
        basic_point.label = 0;
        basic_point.apd_temp = apd_temp;
        // basic_point.pulseCodeInterval = udpData[24] * 10;
        points.push_back(std::move(basic_point));
      }

      if (_echoNum & Echo2) {
        basic_point.x = L_2 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_2 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_2 * (Lz - 2 * dot_product * Nz);

        basic_point.echo = 2;
        basic_point.distance = L_2;
        basic_point.intensity = intensity_2;
        // basic_point.confidence = confidence2;
        basic_point.encoding_interval = _encoding_interval;
        basic_point.state = 0;
        basic_point.label = 0;
        basic_point.apd_temp = apd_temp;
        // basic_point.pulseCodeInterval = udpData[24] * 10;
        points.push_back(std::move(basic_point));
      }
    }
  }
  USEDECODE_COMMON_RETURN
}

bool LidarDevice::UseDecodeFocusT_2(const char *udpData,
                                    TWPointCloud::Points &points,
                                    unsigned int *sec, unsigned int *usec,
                                    float *maxAngle) {
  PRE_LOAD_PCAP_FILE_CALCULATE_EX
  double horizontalAngle = 0;
  unsigned short mirror = 0;
  double Nx = 0.0, Ny = 0.0, Nz = 0.0;

  unsigned int frameSecond =
      FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);

  double frameMicrosecond =
      FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;

  int frame_cycle_cout = TwoHextoInt(udpData[4], udpData[5]);
  int data_block = 6; // 每个data block 6字节
  for (int blocks_num = 0; blocks_num < 12; blocks_num++) {
    int offset = blocks_num * 100;

    int offsetMicrosecond =
        TwoHextoInt(udpData[32 + offset], udpData[33 + offset]);
    // ptp
    double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
    unsigned int blockSecond =
        (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
    unsigned int blockMicrosecond =
        (totalMicrosecond >= 1000000)
            ? (unsigned int)(totalMicrosecond - 1000000)
            : (unsigned int)totalMicrosecond;

    mirror = (udpData[35 + offset] & 0x06) >> 1;

    for (int seq = 0; seq < 16; seq++) {
      // horizontalAngle表示点云水平角度(φ)
      int HextoAngle = TwoHextoInt(udpData[36 + offset + seq * data_block],
                                   udpData[37 + offset + seq * data_block]);
      horizontalAngle = HextoAngle * 0.01;
      horizontalAngle += m_MirrorHorAngleOffset[mirror];

      // 点云径向距离L_1
      double hexToInt1 = TwoHextoInt(udpData[38 + offset + seq * data_block],
                                     udpData[39 + offset + seq * data_block]);
      double L_1 = hexToInt1 * m_calSimpleFPGA;

      unsigned char hexChar1 = udpData[40 + offset + seq * data_block];
      double intensity_1 = ((double)hexChar1);

      int channel =
          16 * (blocks_num < 6 ? blocks_num : blocks_num - 6) + seq + 1;
      int laser_column = 0;
      if ((26 <= channel && channel <= 40 && channel % 2 == 0) ||
          (57 <= channel && channel <= 64))
        laser_column = 1;
      else if ((9 <= channel && channel <= 23 && channel % 2 == 1) ||
               (41 <= channel && channel <= 55 && channel % 2 == 1) ||
               (65 <= channel && channel <= 72) ||
               (81 <= channel && channel <= 88))
        laser_column = 2;
      else if ((1 <= channel && channel <= 8) ||
               (25 <= channel && channel <= 39 && channel % 2 == 1) ||
               (73 <= channel && channel <= 80) ||
               (89 <= channel && channel <= 96))
        laser_column = 3;
      else if ((10 <= channel && channel <= 24 && channel % 2 == 0) ||
               (42 <= channel && channel <= 56 && channel % 2 == 0))
        laser_column = 4;
      double theta = 0; // 转镜法线转过的角度
      double alpha = m_alphaMap_FocusT.count(channel)
                         ? m_alphaMap_FocusT.at(channel)
                         : 0.0; // 修正角度

      double horizontalAngle_modify = 0;
      // 转镜法线转过的角度 θ
      switch (laser_column) {
      case 1:
        theta = (horizontalAngle - 1.63 + alpha) * m_calRA / 2;
        horizontalAngle_modify = horizontalAngle - 1.63;
        break;
      case 2:
        theta = (horizontalAngle - 0.82 + alpha) * m_calRA / 2;
        horizontalAngle_modify = horizontalAngle - 0.82;
        break;
      case 3:
        theta = (horizontalAngle + alpha) * m_calRA / 2;
        horizontalAngle_modify = horizontalAngle;
        break;
      case 4:
        theta = (horizontalAngle + 0.82 + alpha) * m_calRA / 2;
        horizontalAngle_modify = horizontalAngle + 0.82;
        break;
      default:
        break;
      }

      TWPoint basic_point;
      basic_point.angle = horizontalAngle_modify;
      basic_point.mirror = mirror;
      basic_point.channel = channel;
      basic_point.cycle_count = frame_cycle_cout;

      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }

      // 计算各通道竖直角度（β）的正弦和余弦
      double cos_beta = m_verticalChannelAngle_FocusT_cos_vA_RA[channel - 1];
      double sin_beta = m_verticalChannelAngle_FocusT_sin_vA_RA[channel - 1];

      // 机芯发射光线单位向量 li = (-cosβ,0,sinβ)
      double Lx = -cos_beta;
      double Ly = 0;
      double Lz = sin_beta;

      // Nx = cosδcosθ
      // Ny = cosδsinθ
      // Nz = sinδ
      Nx = m_skewing_cos_FocusT[mirror] * cos(theta);
      Ny = m_skewing_cos_FocusT[mirror] * sin(theta);
      Nz = m_skewing_sin_FocusT[mirror];

      double dot_product = Lx * Nx + Ly * Ny + Lz * Nz;

      int apd_temp = (udpData[23] & 0x7F);
      if (udpData[23] & 0x80)
        apd_temp = -apd_temp;

      // 计算笛卡尔坐标X,Y,Z
      if (_echoNum & Echo1) {
        basic_point.x = L_1 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_1 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_1 * (Lz - 2 * dot_product * Nz);

        basic_point.echo = 1;
        basic_point.distance = L_1;
        basic_point.intensity = intensity_1;
        // basic_point.confidence = confidence1;
        basic_point.encoding_interval = _encoding_interval;
        basic_point.state = 0;
        basic_point.label = 0;
        basic_point.apd_temp = apd_temp;
        // basic_point.pulseCodeInterval = udpData[24] * 10;
        points.push_back(std::move(basic_point));
      }
    }
  }
  USEDECODE_COMMON_RETURN
}

bool LidarDevice::UseDecodeFocusTCalib(const char *udpData,
                                       TWPointCloud::Points &points,
                                       unsigned int *sec, unsigned int *usec,
                                       float *maxAngle) {
  PRE_LOAD_PCAP_FILE_CALCULATE_EX
  double horizontalAngle = 0;
  unsigned short mirror = 0;
  double Nx = 0.0, Ny = 0.0, Nz = 0.0;
  unsigned int frameSecond =
      FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
  double frameMicrosecond =
      FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;
  int data_block = 12;
  for (int blocks_num = 0; blocks_num < 6; blocks_num++) {
    int offset = blocks_num * 196;

    // ptp
    int offsetMicrosecond =
        TwoHextoInt(udpData[32 + offset], udpData[33 + offset]);
    double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
    unsigned int blockSecond =
        (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
    unsigned int blockMicrosecond =
        (totalMicrosecond >= 1000000)
            ? (unsigned int)(totalMicrosecond - 1000000)
            : (unsigned int)totalMicrosecond;

    // int blockFlag = (udpData[35 + offset] & 0xF0) >> 4;

    mirror = (udpData[35 + offset] & 0x06) >> 1;

    for (int seq = 0; seq < 16; seq++) {
      // horizontalAngle表示点云水平角度(φ)
      int HextoAngle = TwoHextoInt(udpData[36 + offset + seq * data_block],
                                   udpData[37 + offset + seq * data_block]);
      horizontalAngle = HextoAngle * 0.01;
      horizontalAngle += m_MirrorHorAngleOffset[mirror];

      // 点云径向距离L_1
      double hexToInt1 = TwoHextoInt(udpData[38 + offset + seq * data_block],
                                     udpData[39 + offset + seq * data_block]);
      double L_1 = hexToInt1 * m_calSimpleFPGA;

      unsigned char hexChar1 = udpData[40 + offset + seq * data_block];
      double intensity_1 = ((double)hexChar1);

      double pulse_1 =
          OneHexHalfHextoInt(udpData[41 + offset + seq * data_block],
                             udpData[42 + offset + seq * data_block]) *
          0.032;

      // double confidence1 = udpData[42 + offset + seq * 12] & 0x01;

      // 点云径向距离L_2
      double hexToInt2 = TwoHextoInt(udpData[43 + offset + seq * data_block],
                                     udpData[44 + offset + seq * data_block]);
      double L_2 = hexToInt2 * m_calSimpleFPGA;

      unsigned char hexChar2 = udpData[45 + offset + seq * data_block];
      double intensity_2 = ((double)hexChar2);

      double pulse_2 =
          OneHexHalfHextoInt(udpData[46 + offset + seq * data_block],
                             udpData[47 + offset + seq * data_block]) *
          0.032;

      // double confidence2 = udpData[47 + offset + seq * 12] & 0x01;

      int channel = 16 * blocks_num + seq + 1;
      int laser_column = 0;
      if ((26 <= channel && channel <= 40 && channel % 2 == 0) ||
          (57 <= channel && channel <= 64))
        laser_column = 1;
      else if ((9 <= channel && channel <= 23 && channel % 2 == 1) ||
               (41 <= channel && channel <= 55 && channel % 2 == 1) ||
               (65 <= channel && channel <= 72) ||
               (81 <= channel && channel <= 88))
        laser_column = 2;
      else if ((1 <= channel && channel <= 8) ||
               (25 <= channel && channel <= 39 && channel % 2 == 1) ||
               (73 <= channel && channel <= 80) ||
               (89 <= channel && channel <= 96))
        laser_column = 3;
      else if ((10 <= channel && channel <= 24 && channel % 2 == 0) ||
               (42 <= channel && channel <= 56 && channel % 2 == 0))
        laser_column = 4;
      double theta = 0; // 转镜法线转过的角度
      double alpha = m_alphaMap_FocusT.count(channel)
                         ? m_alphaMap_FocusT.at(channel)
                         : 0.0; // 修正角度
      double horizontalAngle_modify = 0;
      switch (laser_column) {
      case 1:
        theta = (horizontalAngle - 1.63 + alpha) * m_calRA / 2;
        horizontalAngle_modify = horizontalAngle - 1.63;
        break;
      case 2:
        theta = (horizontalAngle - 0.82 + alpha) * m_calRA / 2;
        horizontalAngle_modify = horizontalAngle - 0.82;
        break;
      case 3:
        theta = (horizontalAngle + alpha) * m_calRA / 2;
        horizontalAngle_modify = horizontalAngle;
        break;
      case 4:
        theta = (horizontalAngle + 0.82 + alpha) * m_calRA / 2;
        horizontalAngle_modify = horizontalAngle + 0.82;
        break;
      default:
        break;
      }

      TWPoint basic_point;
      basic_point.angle = horizontalAngle_modify;
      basic_point.mirror = mirror;
      basic_point.channel = channel;
      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }

      // 计算各通道竖直角度（β）的正弦和余弦
      double cos_beta = m_verticalChannelAngle_FocusT_cos_vA_RA[channel - 1];
      double sin_beta = m_verticalChannelAngle_FocusT_sin_vA_RA[channel - 1];

      // 机芯发射光线单位向量 li = (-cosβ,0,sinβ)
      double Lx = -cos_beta;
      double Ly = 0;
      double Lz = sin_beta;

      // Nx = cosδcosθ
      // Ny = cosδsinθ
      // Nz = sinδ
      Nx = m_skewing_cos_FocusT[mirror] * cos(theta);
      Ny = m_skewing_cos_FocusT[mirror] * sin(theta);
      Nz = m_skewing_sin_FocusT[mirror];

      // 计算笛卡尔坐标X,Y,Z
      double dot_product = Lx * Nx + Ly * Ny + Lz * Nz;

      int apd_temp = (udpData[23] & 0x7F);
      if (udpData[23] & 0x80)
        apd_temp = -apd_temp;

      if (_echoNum & Echo1) {
        basic_point.x = L_1 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_1 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_1 * (Lz - 2 * dot_product * Nz);

        basic_point.echo = 1;
        basic_point.distance = L_1;
        basic_point.pulse = pulse_1;
        basic_point.intensity = intensity_1;
        // basic_point.confidence = confidence1;
        basic_point.encoding_interval = _encoding_interval;
        basic_point.state = 0;
        basic_point.label = 0;
        basic_point.apd_temp = apd_temp;
        // basic_point.pulseCodeInterval = udpData[24] * 10;
        points.push_back(std::move(basic_point));
      }

      if (_echoNum & Echo2) {
        basic_point.x = L_2 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_2 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_2 * (Lz - 2 * dot_product * Nz);

        basic_point.echo = 2;
        basic_point.distance = L_2;
        basic_point.pulse = pulse_2;
        basic_point.intensity = intensity_2;
        // basic_point.confidence = confidence2;
        basic_point.encoding_interval = _encoding_interval;
        basic_point.state = 0;
        basic_point.label = 0;
        basic_point.apd_temp = apd_temp;
        // basic_point.pulseCodeInterval = udpData[24] * 10;
        points.push_back(std::move(basic_point));
      }
    }
  }
  USEDECODE_COMMON_RETURN
}

bool LidarDevice::UseDecodeScope128(const char *udpData,
                                    TWPointCloud::Points &points,
                                    unsigned int *sec, unsigned int *usec,
                                    float *maxAngle) {
  PRE_LOAD_PCAP_FILE_CALCULATE_EX

  // ptp
  unsigned int frameSecond =
      FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
  double frameMicrosecond =
      FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;

  for (int blocks_num = 0; blocks_num < 8; blocks_num++) {
    int offset_block = blocks_num * 164;

    // ptp
    int offsetMicrosecond =
        TwoHextoInt(udpData[32 + offset_block], udpData[33 + offset_block]);
    double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
    unsigned int blockSecond =
        (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
    unsigned int blockMicrosecond =
        (totalMicrosecond >= 1000000)
            ? (unsigned int)(totalMicrosecond - 1000000)
            : (unsigned int)totalMicrosecond;

    // Positive or Negative 0:Neg 1:Pos
    unsigned short posOrNeg = udpData[35 + offset_block] & 0x01;

    // mirror
    unsigned short mirror = (udpData[35 + offset_block] >> 1) & 0x03;
    int blockID = (udpData[35 + offset_block] & 0xF0) >> 4;

    if (blockID != 0 && blockID != 1)
      continue;

    if (mirror != 0 && mirror != 1 && mirror != 2)
      continue;

    for (int seq = 0; seq < 16; seq++) {
      // 2Byte 36-37 解析点云水平角度φ
      double hexHorAngle = TwoHextoInt(udpData[36 + offset_block + seq * 10],
                                       udpData[37 + offset_block + seq * 10]);
      double horAngle = hexHorAngle * 0.01;
      horAngle += m_MirrorHorAngleOffset[mirror];

      // L1 2Byte 40-41
      double hexL1 = TwoHextoInt(udpData[38 + offset_block + seq * 10],
                                 udpData[39 + offset_block + seq * 10]);
      double L_1 = hexL1 * m_calSimpleFPGA;

      // intensity 0-255
      unsigned char hexIntensity1 = udpData[40 + offset_block + seq * 10];
      double intensity_1 = hexIntensity1;

      int confidence1 = udpData[41 + offset_block + seq * 10] & 0x01;

      // L2 2Byte
      double hexL2 = TwoHextoInt(udpData[42 + offset_block + seq * 10],
                                 udpData[43 + offset_block + seq * 10]);
      double L_2 = hexL2 * m_calSimpleFPGA;

      // intensity 0-255
      unsigned char hexIntensity2 = udpData[44 + offset_block + seq * 10];
      double intensity_2 = hexIntensity2;

      int confidence2 = udpData[45 + offset_block + seq * 10] & 0x01;

      // 通道计算
      int channel = -1;
      int signal_mirror_seq = blockID * 16 + seq + 1;

      if (posOrNeg) {
        if (0 == mirror) {
          channel = signal_mirror_seq * 2 - 1;
        } else if (1 == mirror || 2 == mirror) {
          channel = signal_mirror_seq * 2;
        }
      } else {
        if (0 == mirror) {
          channel = 64 + signal_mirror_seq * 2 - 1;
        } else if (2 == mirror || 1 == mirror) {
          channel = 64 + signal_mirror_seq * 2;
        }
      }

      // 转镜角度 三角函数计算 （θ：参照PDF定义）
      double theta = 0;

      if (1 == posOrNeg) // 左机芯 正机芯
      {
        theta = horAngle / 2; // + 30;
        if (channel >= 1 && channel <= 16) {
          theta += (-1.25) + m_Scope128PosLaserOffset[0];
          horAngle += 1.5;
        } else if (channel >= 17 && channel <= 32) {
          theta += (-1.25) + m_Scope128PosLaserOffset[1];
          horAngle += 1.5;
        } else if (channel >= 33 && channel <= 48) {
          theta += (-2.75) + m_Scope128PosLaserOffset[2];
          horAngle -= 1.5;
        } else if (channel >= 49 && channel <= 64) {
          theta += (-2.75) + m_Scope128PosLaserOffset[3];
          horAngle -= 1.5;
        }
      } else // 右机芯 负机芯
      {
        theta = horAngle / 2 + 90; //+ 120;
        if (channel >= 65 && channel <= 80) {
          theta += (2.75) + m_Scope128NegLaserOffset[3];
          horAngle += 1.5;
        } else if (channel >= 81 && channel <= 96) {
          theta += (2.75) + m_Scope128NegLaserOffset[2];
          horAngle += 1.5;
        } else if (channel >= 97 && channel <= 112) {
          theta += (1.25) + m_Scope128NegLaserOffset[1];
          horAngle -= 1.5;
        } else if (channel >= 113 && channel <= 128) {
          theta += (1.25) + m_Scope128NegLaserOffset[0];
          horAngle -= 1.5;
        }
      }

      double Lx = 0.0;
      double Ly = 0.0;
      double Lz = 0.0;

      double Nx = 0.0;
      double Ny = 0.0;
      double Nz = 0.0;

      // Positive
      if (1 == posOrNeg) {
        // 计算各通道竖直角度（β）的正弦和余弦
        double sin_beta =
            m_verticalChannelAngle_scp128P_sin_vA_RA[signal_mirror_seq - 1];
        double cos_beta =
            m_verticalChannelAngle_scp128P_cos_vA_RA[signal_mirror_seq - 1];

        // 计算出射光线与X轴夹角（γ）的正弦和余弦
        double sin_gamma = m_rotate_scp128P_sin;
        double cos_gamma = m_rotate_scp128P_cos;

        // 机芯发射光线单位向量li = (cosβcosγ,-cosβsinγ,sinβ)
        Lx = cos_beta * cos_gamma;
        Ly = -cos_beta * sin_gamma;
        Lz = sin_beta;

        // 转镜法线转过的角度（θ）的正弦和余弦
        double sin_theta = sin(theta * m_calRA);
        double cos_theta = cos(theta * m_calRA);

        Nx = m_skewing_cos_scope128[mirror] * cos_theta;
        Ny = m_skewing_cos_scope128[mirror] * sin_theta;
        Nz = m_skewing_sin_scope128[mirror];

      } else {
        // 计算各通道竖直角度（β）的正弦和余弦
        double sin_beta =
            m_verticalChannelAngle_scp128N_sin_vA_RA[signal_mirror_seq - 1];
        double cos_beta =
            m_verticalChannelAngle_scp128N_cos_vA_RA[signal_mirror_seq - 1];

        // 计算出射光线与X轴夹角（γ）的正弦和余弦
        double sin_gamma = m_rotate_scp128N_sin;
        double cos_gamma = m_rotate_scp128N_cos;

        // 机芯发射光线单位向量li = (cosβcosγ,-cosβsinγ,sinβ)
        Lx = cos_beta * cos_gamma;
        Ly = -cos_beta * sin_gamma;
        Lz = sin_beta;

        double sin_theta = sin(theta * m_calRA);
        double cos_theta = cos(theta * m_calRA);

        Nx = m_skewing_cos_scope128[mirror] * cos_theta;
        Ny = m_skewing_cos_scope128[mirror] * sin_theta;
        Nz = m_skewing_sin_scope128[mirror];
      }

      double dot_product = Lx * Nx + Ly * Ny + Lz * Nz;

      TWPoint basic_point;
      basic_point.angle = horAngle;
      basic_point.mirror = mirror;
      basic_point.left_right = posOrNeg;
      basic_point.channel = channel;

      basic_point.back_point = IsBackPoint(basic_point);

      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }

      if (_echoNum & Echo1) {
        basic_point.x = L_1 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_1 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_1 * (Lz - 2 * dot_product * Nz);

        basic_point.intensity = intensity_1;
        basic_point.distance = L_1;
        basic_point.pulse = intensity_1 * 0.128;
        basic_point.echo = 1;
        basic_point.confidence = confidence1;
        points.push_back(std::move(basic_point));
      }

      if (_echoNum & Echo2) {
        basic_point.x = L_2 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_2 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_2 * (Lz - 2 * dot_product * Nz);

        basic_point.intensity = intensity_2;
        basic_point.distance = L_2;
        basic_point.pulse = intensity_2 * 0.128;
        basic_point.echo = 2;
        basic_point.confidence = confidence2;
        points.push_back(std::move(basic_point));
      }
    }
  }

  USEDECODE_COMMON_RETURN
}

bool LidarDevice::UseDecodeScope128_2(const char *udpData,
                                      TWPointCloud::Points &points,
                                      unsigned int *sec, unsigned int *usec,
                                      float *maxAngle) {
  PRE_LOAD_PCAP_FILE_CALCULATE_EX

  // ptp
  unsigned int frameSecond =
      FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
  double frameMicrosecond =
      FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;

  for (int blocks_num = 0; blocks_num < 12; blocks_num++) {
    int offset_block = blocks_num * 100;

    // ptp
    int offsetMicrosecond =
        TwoHextoInt(udpData[32 + offset_block], udpData[33 + offset_block]);
    double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
    unsigned int blockSecond =
        (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
    unsigned int blockMicrosecond =
        (totalMicrosecond >= 1000000)
            ? (unsigned int)(totalMicrosecond - 1000000)
            : (unsigned int)totalMicrosecond;

    // Positive or Negative 0:Neg 1:Pos
    unsigned short posOrNeg = udpData[35 + offset_block] & 0x01;

    // mirror
    unsigned short mirror = (udpData[35 + offset_block] >> 1) & 0x03;
    int blockID = (udpData[35 + offset_block] & 0xF0) >> 4;

    if (blockID != 0 && blockID != 1)
      continue;

    if (mirror != 0 && mirror != 1 && mirror != 2)
      continue;

    for (int seq = 0; seq < 16; seq++) {
      // 2Byte 36-37 解析点云水平角度φ
      double hexHorAngle = TwoHextoInt(udpData[36 + offset_block + seq * 6],
                                       udpData[37 + offset_block + seq * 6]);
      double horAngle = hexHorAngle * 0.01;
      horAngle += m_MirrorHorAngleOffset[mirror];

      // L1 2Byte 40-41
      double hexL1 = TwoHextoInt(udpData[38 + offset_block + seq * 6],
                                 udpData[39 + offset_block + seq * 6]);
      double L_1 = hexL1 * m_calSimpleFPGA;

      // intensity 0-255
      unsigned char hexIntensity1 = udpData[40 + offset_block + seq * 6];
      double intensity_1 = hexIntensity1;

      int confidence1 = udpData[41 + offset_block + seq * 6] & 0x01;

      // 通道计算
      int channel = -1;
      int signal_mirror_seq = blockID * 16 + seq + 1;

      if (posOrNeg) {
        if (0 == mirror) {
          channel = signal_mirror_seq * 2 - 1;
        } else if (1 == mirror || 2 == mirror) {
          channel = signal_mirror_seq * 2;
        }
      } else {
        if (0 == mirror) {
          channel = 64 + signal_mirror_seq * 2 - 1;
        } else if (2 == mirror || 1 == mirror) {
          channel = 64 + signal_mirror_seq * 2;
        }
      }

      // 转镜角度 三角函数计算 （θ：参照PDF定义）
      double theta = 0;

      if (1 == posOrNeg) // 左机芯 正机芯
      {
        theta = horAngle / 2; // + 30;
        if (channel >= 1 && channel <= 16) {
          theta += (-1.25) + m_Scope128PosLaserOffset[0];
          horAngle += 1.5;
        } else if (channel >= 17 && channel <= 32) {
          theta += (-1.25) + m_Scope128PosLaserOffset[1];
          horAngle += 1.5;
        } else if (channel >= 33 && channel <= 48) {
          theta += (-2.75) + m_Scope128PosLaserOffset[2];
          horAngle -= 1.5;
        } else if (channel >= 49 && channel <= 64) {
          theta += (-2.75) + m_Scope128PosLaserOffset[3];
          horAngle -= 1.5;
        }
      } else // 右机芯 负机芯
      {
        theta = horAngle / 2 + 90; //+ 120;
        if (channel >= 65 && channel <= 80) {
          theta += (2.75) + m_Scope128NegLaserOffset[3];
          horAngle += 1.5;
        } else if (channel >= 81 && channel <= 96) {
          theta += (2.75) + m_Scope128NegLaserOffset[2];
          horAngle += 1.5;
        } else if (channel >= 97 && channel <= 112) {
          theta += (1.25) + m_Scope128NegLaserOffset[1];
          horAngle -= 1.5;
        } else if (channel >= 113 && channel <= 128) {
          theta += (1.25) + m_Scope128NegLaserOffset[0];
          horAngle -= 1.5;
        }
      }

      double Lx = 0.0;
      double Ly = 0.0;
      double Lz = 0.0;

      double Nx = 0.0;
      double Ny = 0.0;
      double Nz = 0.0;

      // Positive
      if (1 == posOrNeg) {
        // 计算各通道竖直角度（β）的正弦和余弦
        double sin_beta =
            m_verticalChannelAngle_scp128P_sin_vA_RA[signal_mirror_seq - 1];
        double cos_beta =
            m_verticalChannelAngle_scp128P_cos_vA_RA[signal_mirror_seq - 1];

        // 计算出射光线与X轴夹角（γ）的正弦和余弦
        double sin_gamma = m_rotate_scp128P_sin;
        double cos_gamma = m_rotate_scp128P_cos;

        // 机芯发射光线单位向量li = (cosβcosγ,-cosβsinγ,sinβ)
        Lx = cos_beta * cos_gamma;
        Ly = -cos_beta * sin_gamma;
        Lz = sin_beta;

        // 转镜法线转过的角度（θ）的正弦和余弦
        double sin_theta = sin(theta * m_calRA);
        double cos_theta = cos(theta * m_calRA);

        Nx = m_skewing_cos_scope128[mirror] * cos_theta;
        Ny = m_skewing_cos_scope128[mirror] * sin_theta;
        Nz = m_skewing_sin_scope128[mirror];

      } else {
        // 计算各通道竖直角度（β）的正弦和余弦
        double sin_beta =
            m_verticalChannelAngle_scp128N_sin_vA_RA[signal_mirror_seq - 1];
        double cos_beta =
            m_verticalChannelAngle_scp128N_cos_vA_RA[signal_mirror_seq - 1];

        // 计算出射光线与X轴夹角（γ）的正弦和余弦
        double sin_gamma = m_rotate_scp128N_sin;
        double cos_gamma = m_rotate_scp128N_cos;

        // 机芯发射光线单位向量li = (cosβcosγ,-cosβsinγ,sinβ)
        Lx = cos_beta * cos_gamma;
        Ly = -cos_beta * sin_gamma;
        Lz = sin_beta;

        double sin_theta = sin(theta * m_calRA);
        double cos_theta = cos(theta * m_calRA);

        Nx = m_skewing_cos_scope128[mirror] * cos_theta;
        Ny = m_skewing_cos_scope128[mirror] * sin_theta;
        Nz = m_skewing_sin_scope128[mirror];
      }

      double dot_product = Lx * Nx + Ly * Ny + Lz * Nz;

      TWPoint basic_point;
      basic_point.angle = horAngle;
      basic_point.mirror = mirror;
      basic_point.left_right = posOrNeg;
      basic_point.channel = channel;

      basic_point.back_point = IsBackPoint(basic_point);

      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }

      if (_echoNum & Echo1) {
        basic_point.x = L_1 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_1 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_1 * (Lz - 2 * dot_product * Nz);

        basic_point.intensity = intensity_1;
        basic_point.distance = L_1;
        basic_point.echo = 1;
        basic_point.confidence = confidence1;
        points.push_back(std::move(basic_point));
      }
    }
  }

  USEDECODE_COMMON_RETURN
}

bool LidarDevice::UseDecodeScope128Calib(const char *udpData,
                                         TWPointCloud::Points &points,
                                         unsigned int *sec, unsigned int *usec,
                                         float *maxAngle) {
  PRE_LOAD_PCAP_FILE_CALCULATE_EX

  // ptp
  unsigned int frameSecond =
      FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
  double frameMicrosecond =
      FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;

  for (int blocks_num = 0; blocks_num < 6; blocks_num++) {
    int offset_block = blocks_num * 196;

    // ptp
    int offsetMicrosecond =
        TwoHextoInt(udpData[32 + offset_block], udpData[33 + offset_block]);
    double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
    unsigned int blockSecond =
        (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
    unsigned int blockMicrosecond =
        (totalMicrosecond >= 1000000)
            ? (unsigned int)(totalMicrosecond - 1000000)
            : (unsigned int)totalMicrosecond;

    // Positive or Negative 0:Neg 1:Pos
    unsigned short posOrNeg = udpData[35 + offset_block] & 0x01;

    // mirror
    unsigned short mirror = (udpData[35 + offset_block] >> 1) & 0x03;
    int blockID = (udpData[35 + offset_block] & 0xF0) >> 4;

    if (blockID != 0 && blockID != 1)
      continue;

    if (mirror != 0 && mirror != 1 && mirror != 2)
      continue;

    for (int seq = 0; seq < 16; seq++) {
      // 2Byte 36-37 解析点云水平角度φ
      double hexHorAngle = TwoHextoInt(udpData[36 + offset_block + seq * 12],
                                       udpData[37 + offset_block + seq * 12]);
      double horAngle = hexHorAngle * 0.01;
      horAngle += m_MirrorHorAngleOffset[mirror];

      // L1 2Byte 40-41
      double hexL1 = TwoHextoInt(udpData[38 + offset_block + seq * 12],
                                 udpData[39 + offset_block + seq * 12]);
      double L_1 = hexL1 * m_calSimpleFPGA;

      // intensity 0-255
      unsigned char hexIntensity1 = udpData[40 + offset_block + seq * 12];
      double intensity_1 = hexIntensity1;

      double pulse_1 =
          OneHexHalfHextoInt(udpData[41 + offset_block + seq * 12],
                             udpData[42 + offset_block + seq * 12]) *
          0.032;

      int confidence1 = udpData[42 + offset_block + seq * 12] & 0x01;

      // L2 2Byte
      double hexL2 = TwoHextoInt(udpData[43 + offset_block + seq * 12],
                                 udpData[44 + offset_block + seq * 12]);
      double L_2 = hexL2 * m_calSimpleFPGA;

      // intensity 0-255
      unsigned char hexIntensity2 = udpData[45 + offset_block + seq * 12];
      double intensity_2 = hexIntensity2;

      double pulse_2 =
          OneHexHalfHextoInt(udpData[46 + offset_block + seq * 12],
                             udpData[47 + offset_block + seq * 12]) *
          0.032;

      int confidence2 = udpData[47 + offset_block + seq * 12] & 0x01;

      // 通道计算
      int channel = -1;
      int signal_mirror_seq = blockID * 16 + seq + 1;

      if (posOrNeg) {
        if (0 == mirror) {
          channel = signal_mirror_seq * 2 - 1;
        } else if (1 == mirror || 2 == mirror) {
          channel = signal_mirror_seq * 2;
        }
      } else {
        if (0 == mirror) {
          channel = 64 + signal_mirror_seq * 2 - 1;
        } else if (2 == mirror || 1 == mirror) {
          channel = 64 + signal_mirror_seq * 2;
        }
      }

      // 转镜角度 三角函数计算 （θ：参照PDF定义）
      double theta = 0;

      if (1 == posOrNeg) // 左机芯 正机芯
      {
        theta = horAngle / 2; // + 30;
        if (channel >= 1 && channel <= 16) {
          theta += (-1.25) + m_Scope128PosLaserOffset[0];
          horAngle += 1.5;
        } else if (channel >= 17 && channel <= 32) {
          theta += (-1.25) + m_Scope128PosLaserOffset[1];
          horAngle += 1.5;
        } else if (channel >= 33 && channel <= 48) {
          theta += (-2.75) + m_Scope128PosLaserOffset[2];
          horAngle -= 1.5;
        } else if (channel >= 49 && channel <= 64) {
          theta += (-2.75) + m_Scope128PosLaserOffset[3];
          horAngle -= 1.5;
        }
      } else // 右机芯 负机芯
      {
        theta = horAngle / 2 + 90; //+ 120;
        if (channel >= 65 && channel <= 80) {
          theta += (2.75) + m_Scope128NegLaserOffset[3];
          horAngle += 1.5;
        } else if (channel >= 81 && channel <= 96) {
          theta += (2.75) + m_Scope128NegLaserOffset[2];
          horAngle += 1.5;
        } else if (channel >= 97 && channel <= 112) {
          theta += (1.25) + m_Scope128NegLaserOffset[1];
          horAngle -= 1.5;
        } else if (channel >= 113 && channel <= 128) {
          theta += (1.25) + m_Scope128NegLaserOffset[0];
          horAngle -= 1.5;
        }
      }

      double Lx = 0.0;
      double Ly = 0.0;
      double Lz = 0.0;

      double Nx = 0.0;
      double Ny = 0.0;
      double Nz = 0.0;

      // Positive
      if (1 == posOrNeg) {
        // 计算各通道竖直角度（β）的正弦和余弦
        double sin_beta =
            m_verticalChannelAngle_scp128P_sin_vA_RA[signal_mirror_seq - 1];
        double cos_beta =
            m_verticalChannelAngle_scp128P_cos_vA_RA[signal_mirror_seq - 1];

        // 计算出射光线与X轴夹角（γ）的正弦和余弦
        double sin_gamma = m_rotate_scp128P_sin;
        double cos_gamma = m_rotate_scp128P_cos;

        // 机芯发射光线单位向量li = (cosβcosγ,-cosβsinγ,sinβ)
        Lx = cos_beta * cos_gamma;
        Ly = -cos_beta * sin_gamma;
        Lz = sin_beta;

        // 转镜法线转过的角度（θ）的正弦和余弦
        double sin_theta = sin(theta * m_calRA);
        double cos_theta = cos(theta * m_calRA);

        Nx = m_skewing_cos_scope128[mirror] * cos_theta;
        Ny = m_skewing_cos_scope128[mirror] * sin_theta;
        Nz = m_skewing_sin_scope128[mirror];

      } else {
        // 计算各通道竖直角度（β）的正弦和余弦
        double sin_beta =
            m_verticalChannelAngle_scp128N_sin_vA_RA[signal_mirror_seq - 1];
        double cos_beta =
            m_verticalChannelAngle_scp128N_cos_vA_RA[signal_mirror_seq - 1];

        // 计算出射光线与X轴夹角（γ）的正弦和余弦
        double sin_gamma = m_rotate_scp128N_sin;
        double cos_gamma = m_rotate_scp128N_cos;

        // 机芯发射光线单位向量li = (cosβcosγ,-cosβsinγ,sinβ)
        Lx = cos_beta * cos_gamma;
        Ly = -cos_beta * sin_gamma;
        Lz = sin_beta;

        double sin_theta = sin(theta * m_calRA);
        double cos_theta = cos(theta * m_calRA);

        Nx = m_skewing_cos_scope128[mirror] * cos_theta;
        Ny = m_skewing_cos_scope128[mirror] * sin_theta;
        Nz = m_skewing_sin_scope128[mirror];
      }

      double dot_product = Lx * Nx + Ly * Ny + Lz * Nz;

      TWPoint basic_point;
      basic_point.angle = horAngle;
      basic_point.mirror = mirror;
      basic_point.left_right = posOrNeg;
      basic_point.channel = channel;

      basic_point.back_point = IsBackPoint(basic_point);

      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }

      if (_echoNum & Echo1) {
        basic_point.x = L_1 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_1 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_1 * (Lz - 2 * dot_product * Nz);

        basic_point.intensity = intensity_1;
        basic_point.distance = L_1;
        basic_point.pulse = pulse_1;
        basic_point.echo = 1;
        basic_point.confidence = confidence1;
        points.push_back(std::move(basic_point));
      }

      if (_echoNum & Echo2) {
        basic_point.x = L_2 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_2 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_2 * (Lz - 2 * dot_product * Nz);

        basic_point.intensity = intensity_2;
        basic_point.distance = L_2;
        basic_point.pulse = pulse_2;
        basic_point.echo = 2;
        basic_point.confidence = confidence2;
        points.push_back(std::move(basic_point));
      }
    }
  }

  USEDECODE_COMMON_RETURN
}

bool LidarDevice::UseDecodeScope128F(const char *udpData,
                                     TWPointCloud::Points &points,
                                     unsigned int *sec, unsigned int *usec,
                                     float *maxAngle) {
  PRE_LOAD_PCAP_FILE_CALCULATE_EX

  // ptp
  unsigned int frameSecond =
      FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
  double frameMicrosecond =
      FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;

  for (int blocks_num = 0; blocks_num < 8; blocks_num++) {
    int offset_block = blocks_num * 164;

    // ptp
    int offsetMicrosecond =
        TwoHextoInt(udpData[32 + offset_block], udpData[33 + offset_block]);
    double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
    unsigned int blockSecond =
        (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
    unsigned int blockMicrosecond =
        (totalMicrosecond >= 1000000)
            ? (unsigned int)(totalMicrosecond - 1000000)
            : (unsigned int)totalMicrosecond;

    // L/R
    unsigned char hexLOrR = udpData[35 + offset_block];
    hexLOrR = hexLOrR << 7;
    unsigned short leftOrRight = hexLOrR >> 7;

    // mirror
    unsigned char hexMirror = udpData[35 + offset_block];
    hexMirror = hexMirror << 5;
    unsigned short mirror = hexMirror >> 6;
    int blockID = (udpData[35 + offset_block] & 0xF0) >> 4;

    //
    double cos_delta = m_skewing_cos_scp256[mirror];
    double sin_delta = m_skewing_sin_scp256[mirror];

    for (int seq = 0; seq < 16; seq++) {
      // 2Byte 36-37 解析点云水平角度φ
      double hexHorAngle = TwoHextoInt(udpData[36 + offset_block + seq * 10],
                                       udpData[37 + offset_block + seq * 10]);
      double horAngle = hexHorAngle * 0.01;
      horAngle += m_MirrorHorAngleOffset[mirror];

      // L1 2Byte 40-41
      double hexL1 = TwoHextoInt(udpData[38 + offset_block + seq * 10],
                                 udpData[39 + offset_block + seq * 10]);
      double L_1 = hexL1 * m_calSimpleFPGA;

      // intensity 0-255
      unsigned char hexIntensity1 = udpData[40 + offset_block + seq * 10];
      double intensity_1 = hexIntensity1;

      int confidence1 = udpData[41 + offset_block + seq * 10];

      // L2 2Byte
      double hexL2 = TwoHextoInt(udpData[42 + offset_block + seq * 10],
                                 udpData[43 + offset_block + seq * 10]);
      double L_2 = hexL2 * m_calSimpleFPGA;

      // intensity 0-255
      unsigned char hexIntensity2 = udpData[44 + offset_block + seq * 10];
      double intensity_2 = hexIntensity2;

      int confidence2 = udpData[45 + offset_block + seq * 10];

      // 通道计算
      int channel = -1;
      int signal_mirror_seq = blockID * 16 + seq + 1;

      if (leftOrRight) {
        if (0 == mirror) {
          channel = signal_mirror_seq * 2 - 1;
        } else if (1 == mirror || 2 == mirror) {
          channel = signal_mirror_seq * 2;
        }
      } else {
        if (0 == mirror) {
          channel = 64 + signal_mirror_seq * 2 - 1;
        } else if (2 == mirror || 1 == mirror) {
          channel = 64 + signal_mirror_seq * 2;
        }
      }

      // laser1/laser2
      int laser = (channel > 64)
                      ? (((channel - 64) >= 1 && (channel - 64) <= 32) ? 1 : 2)
                      : ((channel >= 1 && channel <= 32) ? 1 : 2);

      // 转镜角度 三角函数计算 （θ：参照PDF定义）
      double theta = 0;
      if (1 == leftOrRight) // 左机芯
      {
        theta = (1 == laser) ? ((720.0 - horAngle) * 0.5 - 3.0)
                             : ((720.0 - horAngle) * 0.5 - 1.0);
      } else // 右机芯
      {
        theta = (1 == laser) ? ((540.0 - horAngle) * 0.5 + 1.0)
                             : ((540.0 - horAngle) * 0.5 + 3.0);
      }

      // v3
      if (m_scope256LaserVersion == 1 && m_scope256PLAngleCorrectFlag == 0) {
        if (1 == leftOrRight) // 左机芯 正机芯
        {
          theta = (720.0 - horAngle) * 0.5;
          if (channel >= 1 && channel <= 16)
            theta += m_Scope256LeftLaserOffset[0];
          else if (channel >= 17 && channel <= 32)
            theta += m_Scope256LeftLaserOffset[1];
          else if (channel >= 33 && channel <= 48)
            theta += m_Scope256LeftLaserOffset[2];
          else if (channel >= 49 && channel <= 64)
            theta += m_Scope256LeftLaserOffset[3];
        } else // 右机芯 负机芯
        {
          theta = (540.0 - horAngle) * 0.5;
          if (channel >= 65 && channel <= 80)
            theta += m_Scope256RightLaserOffset[3];
          else if (channel >= 81 && channel <= 96)
            theta += m_Scope256RightLaserOffset[2];
          else if (channel >= 97 && channel <= 112)
            theta += m_Scope256RightLaserOffset[1];
          else if (channel >= 113 && channel <= 128)
            theta += m_Scope256RightLaserOffset[0];
        }
      }

      double Lx = 0.0;
      double Ly = 0.0;
      double Lz = 0.0;
      // 转镜法线转过的角度（θ）的正弦和余弦
      double sin_theta = sin(theta * m_calRA);
      double cos_theta = cos(theta * m_calRA);

      double Nx = cos_delta * cos_theta;
      double Ny = -cos_delta * sin_theta;
      double Nz = sin_delta;

      if (1 == leftOrRight) {
        // 计算各通道竖直角度（β）的正弦和余弦
        double sin_beta =
            m_verticalChannelAngle_scp256L_sin_vA_RA[signal_mirror_seq * 2 - 1];
        double cos_beta =
            m_verticalChannelAngle_scp256L_cos_vA_RA[signal_mirror_seq * 2 - 1];

        // 计算出射光线与X轴夹角（γ）的正弦和余弦
        double sin_gamma = m_rotate_scp256L_sin;
        double cos_gamma = m_rotate_scp256L_cos;

        // 机芯发射光线单位向量li = (cosβcosγ,-cosβsinγ,sinβ)
        Lx = cos_beta * cos_gamma;
        Ly = -cos_beta * sin_gamma;
        Lz = sin_beta;

      } else {
        // 计算各通道竖直角度（β）的正弦和余弦
        double sin_beta =
            m_verticalChannelAngle_scp256R_sin_vA_RA[signal_mirror_seq * 2 - 1];
        double cos_beta =
            m_verticalChannelAngle_scp256R_cos_vA_RA[signal_mirror_seq * 2 - 1];

        // 计算出射光线与X轴夹角（γ）的正弦和余弦
        double sin_gamma = m_rotate_scp256R_sin;
        double cos_gamma = m_rotate_scp256R_cos;

        // 机芯发射光线单位向量li = (cosβcosγ,-cosβsinγ,sinβ)
        Lx = cos_beta * cos_gamma;
        Ly = -cos_beta * sin_gamma;
        Lz = sin_beta;
      }

      double dot_product = Lx * Nx + Ly * Ny + Lz * Nz;

      TWPoint basic_point;
      basic_point.angle = horAngle;
      basic_point.mirror = mirror;
      basic_point.left_right = leftOrRight;
      basic_point.channel = channel;
      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }

      if (_echoNum & Echo1) {
        basic_point.x = L_1 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_1 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_1 * (Lz - 2 * dot_product * Nz);

        basic_point.intensity = intensity_1;
        basic_point.distance = L_1;
        basic_point.pulse = intensity_1 * 0.128;
        basic_point.confidence = confidence1;
        basic_point.echo = 1;
        points.push_back(std::move(basic_point));
      }

      if (_echoNum & Echo2) {
        basic_point.x = L_2 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_2 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_2 * (Lz - 2 * dot_product * Nz);

        basic_point.intensity = intensity_2;
        basic_point.distance = L_2;
        basic_point.pulse = intensity_2 * 0.128;
        basic_point.echo = 2;
        basic_point.confidence = confidence2;
        points.push_back(std::move(basic_point));
      }
    }
  }

  USEDECODE_COMMON_RETURN
}

bool LidarDevice::UseDecodeScope128FCalib(const char *udpData,
                                          TWPointCloud::Points &points,
                                          unsigned int *sec, unsigned int *usec,
                                          float *maxAngle) {
  PRE_LOAD_PCAP_FILE_CALCULATE_EX

  // ptp
  unsigned int frameSecond =
      FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
  double frameMicrosecond =
      FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;

  for (int blocks_num = 0; blocks_num < 6; blocks_num++) {
    int offset_block = blocks_num * 196;

    // ptp
    int offsetMicrosecond =
        TwoHextoInt(udpData[32 + offset_block], udpData[33 + offset_block]);
    double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
    unsigned int blockSecond =
        (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
    unsigned int blockMicrosecond =
        (totalMicrosecond >= 1000000)
            ? (unsigned int)(totalMicrosecond - 1000000)
            : (unsigned int)totalMicrosecond;

    // L/R
    unsigned char hexLOrR = udpData[35 + offset_block];
    hexLOrR = hexLOrR << 7;
    unsigned short leftOrRight = hexLOrR >> 7;

    // mirror
    unsigned char hexMirror = udpData[35 + offset_block];
    hexMirror = hexMirror << 5;
    unsigned short mirror = hexMirror >> 6;
    int blockID = (udpData[35 + offset_block] & 0xF0) >> 4;

    //
    double cos_delta = m_skewing_cos_scp256[mirror];
    double sin_delta = m_skewing_sin_scp256[mirror];

    for (int seq = 0; seq < 16; seq++) {
      // 2Byte 36-37 解析点云水平角度φ
      double hexHorAngle = TwoHextoInt(udpData[36 + offset_block + seq * 12],
                                       udpData[37 + offset_block + seq * 12]);
      double horAngle = hexHorAngle * 0.01;
      horAngle += m_MirrorHorAngleOffset[mirror];

      // L1 2Byte 40-41
      double hexL1 = TwoHextoInt(udpData[38 + offset_block + seq * 12],
                                 udpData[39 + offset_block + seq * 12]);
      double L_1 = hexL1 * m_calSimpleFPGA;

      // intensity 0-255
      unsigned char hexIntensity1 = udpData[40 + offset_block + seq * 12];
      double intensity_1 = hexIntensity1;

      double pulse_1 =
          OneHexHalfHextoInt(udpData[41 + offset_block + seq * 12],
                             udpData[42 + offset_block + seq * 12]) *
          0.032;

      int confidence1 = udpData[42 + offset_block + seq * 12] & 0x01;

      // L2 2Byte
      double hexL2 = TwoHextoInt(udpData[43 + offset_block + seq * 12],
                                 udpData[44 + offset_block + seq * 12]);
      double L_2 = hexL2 * m_calSimpleFPGA;

      // intensity 0-255
      unsigned char hexIntensity2 = udpData[45 + offset_block + seq * 12];
      double intensity_2 = hexIntensity2;

      double pulse_2 =
          OneHexHalfHextoInt(udpData[46 + offset_block + seq * 12],
                             udpData[47 + offset_block + seq * 12]) *
          0.032;

      int confidence2 = udpData[47 + offset_block + seq * 12];

      // 通道计算
      int channel = -1;
      int signal_mirror_seq = blockID * 16 + seq + 1;

      if (leftOrRight) {
        if (0 == mirror) {
          channel = signal_mirror_seq * 2 - 1;
        } else if (1 == mirror || 2 == mirror) {
          channel = signal_mirror_seq * 2;
        }
      } else {
        if (0 == mirror) {
          channel = 64 + signal_mirror_seq * 2 - 1;
        } else if (2 == mirror || 1 == mirror) {
          channel = 64 + signal_mirror_seq * 2;
        }
      }

      // laser1/laser2
      int laser = (channel > 64)
                      ? (((channel - 64) >= 1 && (channel - 64) <= 32) ? 1 : 2)
                      : ((channel >= 1 && channel <= 32) ? 1 : 2);

      // 转镜角度 三角函数计算 （θ：参照PDF定义）
      double theta = 0;
      if (1 == leftOrRight) // 左机芯
      {
        theta = (1 == laser) ? ((720.0 - horAngle) * 0.5 - 3.0)
                             : ((720.0 - horAngle) * 0.5 - 1.0);
      } else // 右机芯
      {
        theta = (1 == laser) ? ((540.0 - horAngle) * 0.5 + 1.0)
                             : ((540.0 - horAngle) * 0.5 + 3.0);
      }

      // v3
      if (m_scope256LaserVersion == 1 && m_scope256PLAngleCorrectFlag == 0) {
        if (1 == leftOrRight) // 左机芯 正机芯
        {
          theta = (720.0 - horAngle) * 0.5;
          if (channel >= 1 && channel <= 16)
            theta += m_Scope256LeftLaserOffset[0];
          else if (channel >= 17 && channel <= 32)
            theta += m_Scope256LeftLaserOffset[1];
          else if (channel >= 33 && channel <= 48)
            theta += m_Scope256LeftLaserOffset[2];
          else if (channel >= 49 && channel <= 64)
            theta += m_Scope256LeftLaserOffset[3];
        } else // 右机芯 负机芯
        {
          theta = (540.0 - horAngle) * 0.5;
          if (channel >= 65 && channel <= 80)
            theta += m_Scope256RightLaserOffset[3];
          else if (channel >= 81 && channel <= 96)
            theta += m_Scope256RightLaserOffset[2];
          else if (channel >= 97 && channel <= 112)
            theta += m_Scope256RightLaserOffset[1];
          else if (channel >= 113 && channel <= 128)
            theta += m_Scope256RightLaserOffset[0];
        }
      }

      double Lx = 0.0;
      double Ly = 0.0;
      double Lz = 0.0;
      // 转镜法线转过的角度（θ）的正弦和余弦
      double sin_theta = sin(theta * m_calRA);
      double cos_theta = cos(theta * m_calRA);

      double Nx = cos_delta * cos_theta;
      double Ny = -cos_delta * sin_theta;
      double Nz = sin_delta;

      if (1 == leftOrRight) {
        // 计算各通道竖直角度（β）的正弦和余弦
        double sin_beta =
            m_verticalChannelAngle_scp256L_sin_vA_RA[signal_mirror_seq * 2 - 1];
        double cos_beta =
            m_verticalChannelAngle_scp256L_cos_vA_RA[signal_mirror_seq * 2 - 1];

        // 计算出射光线与X轴夹角（γ）的正弦和余弦
        double sin_gamma = m_rotate_scp256L_sin;
        double cos_gamma = m_rotate_scp256L_cos;

        // 机芯发射光线单位向量li = (cosβcosγ,-cosβsinγ,sinβ)
        Lx = cos_beta * cos_gamma;
        Ly = -cos_beta * sin_gamma;
        Lz = sin_beta;

      } else {
        // 计算各通道竖直角度（β）的正弦和余弦
        double sin_beta =
            m_verticalChannelAngle_scp256R_sin_vA_RA[signal_mirror_seq * 2 - 1];
        double cos_beta =
            m_verticalChannelAngle_scp256R_cos_vA_RA[signal_mirror_seq * 2 - 1];

        // 计算出射光线与X轴夹角（γ）的正弦和余弦
        double sin_gamma = m_rotate_scp256R_sin;
        double cos_gamma = m_rotate_scp256R_cos;

        // 机芯发射光线单位向量li = (cosβcosγ,-cosβsinγ,sinβ)
        Lx = cos_beta * cos_gamma;
        Ly = -cos_beta * sin_gamma;
        Lz = sin_beta;
      }

      double dot_product = Lx * Nx + Ly * Ny + Lz * Nz;

      TWPoint basic_point;
      basic_point.angle = horAngle;
      basic_point.mirror = mirror;
      basic_point.left_right = leftOrRight;
      basic_point.channel = channel;
      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }

      if (_echoNum & Echo1) {
        basic_point.x = L_1 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_1 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_1 * (Lz - 2 * dot_product * Nz);

        basic_point.intensity = intensity_1;
        basic_point.distance = L_1;
        basic_point.pulse = pulse_1;
        basic_point.confidence = confidence1;
        basic_point.echo = 1;
        points.push_back(std::move(basic_point));
      }

      if (_echoNum & Echo2) {
        basic_point.x = L_2 * (Lx - 2 * dot_product * Nx);
        basic_point.y = L_2 * (Ly - 2 * dot_product * Ny);
        basic_point.z = L_2 * (Lz - 2 * dot_product * Nz);

        basic_point.intensity = intensity_2;
        basic_point.distance = L_2;
        basic_point.pulse = pulse_2;
        basic_point.confidence = confidence2;
        basic_point.echo = 2;
        points.push_back(std::move(basic_point));
      }
    }
  }
  USEDECODE_COMMON_RETURN
}

// 用户协议双回波
bool LidarDevice::UseDecodeTW360(const char *udpData,
                                 TWPointCloud::Points &points,
                                 unsigned int *sec, unsigned int *usec,
                                 float *maxAngle) {
  PRE_LOAD_PCAP_FILE_CALCULATE_EX
  double horizontalAngle = 0;
  unsigned short mirror = 0;
  unsigned int frameSecond =
      FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
  double frameMicrosecond =
      FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;
  int data_block = 10;
  for (int blocks_num = 0; blocks_num < 6; blocks_num++) {
    int offset = blocks_num * 164;

    // ptp
    int offsetMicrosecond =
        TwoHextoInt(udpData[32 + offset], udpData[33 + offset]);
    double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
    unsigned int blockSecond =
        (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
    unsigned int blockMicrosecond =
        (totalMicrosecond >= 1000000)
            ? (unsigned int)(totalMicrosecond - 1000000)
            : (unsigned int)totalMicrosecond;

    mirror = (udpData[35 + offset] & 0x06) >> 1;

    for (int seq = 0; seq < 16; seq++) {
      // horizontalAngle表示点云水平角度(φ)
      int HextoAngle = TwoHextoInt(udpData[36 + offset + seq * data_block],
                                   udpData[37 + offset + seq * data_block]);
      horizontalAngle = HextoAngle * 0.01;

      // 点云径向距离L_1
      double hexToInt1 = TwoHextoInt(udpData[38 + offset + seq * data_block],
                                     udpData[39 + offset + seq * data_block]);
      double L_1 = hexToInt1 * m_calSimpleFPGA_TW360;

      unsigned char hexChar1 = udpData[40 + offset + seq * data_block];
      double intensity_1 = ((double)hexChar1);

      double confidence1 = udpData[41 + offset + seq * data_block] & 0x01;

      // 点云径向距离L_2
      double hexToInt2 = TwoHextoInt(udpData[42 + offset + seq * data_block],
                                     udpData[43 + offset + seq * data_block]);
      double L_2 = hexToInt2 * m_calSimpleFPGA_TW360;

      unsigned char hexChar2 = udpData[44 + offset + seq * data_block];
      double intensity_2 = ((double)hexChar2);

      double confidence2 = udpData[45 + offset + seq * data_block] & 0x01;

      int channel =
          16 * (blocks_num < 3 ? blocks_num : blocks_num - 3) + seq + 1;

      double horizontalAngle_modify = 0;
      // todo
      int laser_blockSize = m_channelNumber / m_laserBlock; // 每块的大小,12
      int laser_blockNum =
          (channel - 1) / laser_blockSize; // 计算当前channel所在的块号(0-based)

      // 判断块号是奇数还是偶数
      if (laser_blockNum % 2 == 0) {
        // 偶数块(0,2,4,6)使用偏移量1
        horizontalAngle_modify = horizontalAngle + m_laserBlockOffset_1;
      } else {
        // 奇数块(1,3,5,7)使用偏移量2
        horizontalAngle_modify = horizontalAngle + m_laserBlockOffset_2;
      }

      TWPoint basic_point;
      basic_point.angle = 360 - horizontalAngle; // 显示时，逆时针为正
      basic_point.mirror = mirror;
      basic_point.channel = channel;

      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }

      double alpha = horizontalAngle_modify * m_calRA; // 水平角度转换为弧度
      if (_echoNum & Echo1) {
        basic_point.x =
            L_1 * m_verticalChannelAngle_TW360_cos_vA_RA[channel - 1] *
                cos(alpha) +
            m_R_TW360 * cos(alpha);
        basic_point.y =
            L_1 * m_verticalChannelAngle_TW360_cos_vA_RA[channel - 1] *
                sin(-alpha) +
            m_R_TW360 * sin(-alpha);
        basic_point.z =
            L_1 * m_verticalChannelAngle_TW360_sin_vA_RA[channel - 1] +
            m_Z_TW360;
        double distanceReal =
            sqrt(basic_point.x * basic_point.x + basic_point.y * basic_point.y +
                 basic_point.z * basic_point.z);
        basic_point.echo = 1;
        basic_point.distance = distanceReal;
        basic_point.intensity = intensity_1;
        basic_point.confidence = confidence1;
        // basic_point.encoding_interval = _encoding_interval;
        basic_point.state = 0;
        basic_point.label = 0;
        // basic_point.apd_temp = apd_temp;
        //  basic_point.pulseCodeInterval = udpData[24] * 10;
        points.push_back(std::move(basic_point));
      }

      if (_echoNum & Echo2) {
        basic_point.x =
            L_2 * m_verticalChannelAngle_TW360_cos_vA_RA[channel - 1] *
                cos(alpha) +
            m_R_TW360 * cos(alpha);
        basic_point.y =
            L_2 * m_verticalChannelAngle_TW360_cos_vA_RA[channel - 1] *
                sin(-alpha) +
            m_R_TW360 * sin(-alpha);
        basic_point.z =
            L_2 * m_verticalChannelAngle_TW360_sin_vA_RA[channel - 1] +
            m_Z_TW360;
        double distanceReal =
            sqrt(basic_point.x * basic_point.x + basic_point.y * basic_point.y +
                 basic_point.z * basic_point.z);
        basic_point.echo = 2;
        basic_point.distance = distanceReal;
        basic_point.intensity = intensity_2;
        basic_point.confidence = confidence2;
        // basic_point.encoding_interval = _encoding_interval;
        basic_point.state = 0;
        basic_point.label = 0;
        // basic_point.apd_temp = apd_temp;
        //  basic_point.pulseCodeInterval = udpData[24] * 10;
        points.push_back(std::move(basic_point));
      }
    }
  }
  USEDECODE_COMMON_RETURN
}

// 用户协议单回波
bool LidarDevice::UseDecodeTW360_2(const char *udpData,
                                   TWPointCloud::Points &points,
                                   unsigned int *sec, unsigned int *usec,
                                   float *maxAngle) {
  PRE_LOAD_PCAP_FILE_CALCULATE_EX
  double horizontalAngle = 0;
  unsigned short mirror = 0;
  unsigned int frameSecond =
      FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
  double frameMicrosecond =
      FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;
  int data_block = 6;
  for (int blocks_num = 0; blocks_num < 12; blocks_num++) {
    int offset = blocks_num * 100;

    // ptp
    int offsetMicrosecond =
        TwoHextoInt(udpData[32 + offset], udpData[33 + offset]);
    double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
    unsigned int blockSecond =
        (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
    unsigned int blockMicrosecond =
        (totalMicrosecond >= 1000000)
            ? (unsigned int)(totalMicrosecond - 1000000)
            : (unsigned int)totalMicrosecond;

    mirror = (udpData[35 + offset] & 0x06) >> 1;

    for (int seq = 0; seq < 16; seq++) {
      // horizontalAngle表示点云水平角度(φ)
      int HextoAngle = TwoHextoInt(udpData[36 + offset + seq * data_block],
                                   udpData[37 + offset + seq * data_block]);
      horizontalAngle = HextoAngle * 0.01;

      // 点云径向距离L_1
      double hexToInt1 = TwoHextoInt(udpData[38 + offset + seq * data_block],
                                     udpData[39 + offset + seq * data_block]);
      double L_1 = hexToInt1 * m_calSimpleFPGA_TW360;

      unsigned char hexChar1 = udpData[40 + offset + seq * data_block];
      double intensity_1 = ((double)hexChar1);

      double confidence1 = udpData[41 + offset + seq * data_block] & 0x01;

      // 计算通道
      int block_offset = 0;

      if (blocks_num < 3) {
        block_offset = blocks_num;
      } else if (blocks_num < 6) {
        block_offset = blocks_num - 3;
      } else if (blocks_num < 9) {
        block_offset = blocks_num - 6;
      } else {
        block_offset = blocks_num - 9;
      }

      int channel = 16 * block_offset + seq + 1;
      double horizontalAngle_modify = 0;
      int laser_blockSize = m_channelNumber / m_laserBlock; // 每块的大小,12
      int laser_blockNum =
          (channel - 1) / laser_blockSize; // 计算当前channel所在的块号(0-based)

      // 判断块号是奇数还是偶数
      if (laser_blockNum % 2 == 0) {
        // 偶数块(0,2,4,6)使用偏移量1
        horizontalAngle_modify = horizontalAngle + m_laserBlockOffset_1;
      } else {
        // 奇数块(1,3,5,7)使用偏移量2
        horizontalAngle_modify = horizontalAngle + m_laserBlockOffset_2;
      }
      TWPoint basic_point;
      basic_point.angle = 360 - horizontalAngle;
      basic_point.mirror = mirror;
      basic_point.channel = channel;

      if (_is_lidar_time) {
        basic_point.t_sec = blockSecond;
        basic_point.t_usec = blockMicrosecond;
      } else {
        basic_point.t_sec = *sec;
        basic_point.t_usec = *usec;
      }

      double alpha = horizontalAngle_modify * m_calRA; // 水平角度转换为弧度
      if (_echoNum & Echo1) {
        basic_point.x =
            L_1 * m_verticalChannelAngle_TW360_cos_vA_RA[channel - 1] *
                cos(alpha) +
            m_R_TW360 * cos(alpha);
        basic_point.y =
            L_1 * m_verticalChannelAngle_TW360_cos_vA_RA[channel - 1] *
                sin(-alpha) +
            m_R_TW360 * sin(-alpha);
        basic_point.z =
            L_1 * m_verticalChannelAngle_TW360_sin_vA_RA[channel - 1] +
            m_Z_TW360;
        double distanceReal =
            sqrt(basic_point.x * basic_point.x + basic_point.y * basic_point.y +
                 basic_point.z * basic_point.z);
        basic_point.echo = 1;
        basic_point.distance = distanceReal;
        basic_point.intensity = intensity_1;
        basic_point.confidence = confidence1;
        // basic_point.encoding_interval = _encoding_interval;
        basic_point.state = 0;
        basic_point.label = 0;
        // basic_point.apd_temp = apd_temp;
        //  basic_point.pulseCodeInterval = udpData[24] * 10;
        points.push_back(std::move(basic_point));
      }
    }
  }
  USEDECODE_COMMON_RETURN
}

bool LidarDevice::UseDecodeTW360Calib(const char *udpData,
                                      TWPointCloud::Points &points,
                                      unsigned int *sec, unsigned int *usec,
                                      float *maxAngle) {

  PRE_LOAD_PCAP_FILE_CALCULATE_EX
  double horizontalAngle = 0;
  unsigned short mirror = 0;
  unsigned int frameSecond =
      FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
  double frameMicrosecond =
      FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;
  int data_block = 12;

  for (int blocks_num = 0; blocks_num < 6; blocks_num++) {
    int offset = blocks_num * 196;

    // ptp
    int offsetMicrosecond =
        TwoHextoInt(udpData[32 + offset], udpData[33 + offset]);
    double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
    unsigned int blockSecond =
        (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
    unsigned int blockMicrosecond =
        (totalMicrosecond >= 1000000)
            ? (unsigned int)(totalMicrosecond - 1000000)
            : (unsigned int)totalMicrosecond;
    mirror = (udpData[35 + offset] & 0x06) >> 1;
    unsigned int blockSecond_tmp =
        _is_lidar_time ? blockSecond : *sec;
    unsigned int blockMicrosecond_tmp =
        _is_lidar_time ? blockMicrosecond : *usec;
    for (int seq = 0; seq < 16; seq++) {
      // horizontalAngle表示点云水平角度(φ)
      int HextoAngle = TwoHextoInt(udpData[36 + offset + seq * data_block],
                                   udpData[37 + offset + seq * data_block]);
      horizontalAngle = HextoAngle * 0.01;

      // 点云径向距离L_1
      double hexToInt1 = TwoHextoInt(udpData[38 + offset + seq * data_block],
                                     udpData[39 + offset + seq * data_block]);
      double L_1 = hexToInt1 * m_calSimpleFPGA_TW360;

      unsigned char hexChar1 = udpData[40 + offset + seq * data_block];
      double intensity_1 = ((double)hexChar1);

      double pulse_1 =
          OneHexHalfHextoInt(udpData[41 + offset + seq * data_block],
                             udpData[42 + offset + seq * data_block]) *
          0.0625;

      double confidence1 = udpData[42 + offset + seq * data_block] & 0x01;

      // 点云径向距离L_2
      double hexToInt2 = TwoHextoInt(udpData[43 + offset + seq * data_block],
                                     udpData[44 + offset + seq * data_block]);
      double L_2 = hexToInt2 * m_calSimpleFPGA_TW360;

      unsigned char hexChar2 = udpData[45 + offset + seq * data_block];
      double intensity_2 = ((double)hexChar2);

      double pulse_2 =
          OneHexHalfHextoInt(udpData[46 + offset + seq * data_block],
                             udpData[47 + offset + seq * data_block]) *
          0.0625;

      double confidence2 = udpData[47 + offset + seq * data_block] & 0x01;

      int channel =
          16 * (blocks_num < 3 ? blocks_num : blocks_num - 3) + seq + 1;
      double offsetTime = 0;
      int offsetChannel = 0;
      if (1 <= channel && channel <= 12) {
        offsetTime = 0;
        offsetChannel = channel;
      } else if (13 <= channel && channel <= 24) {
        offsetTime = -20;
        offsetChannel = channel - 12;
      } else if (25 <= channel && channel <= 36) {
        offsetTime = -40;
        offsetChannel = channel - 24;
      } else if (37 <= channel && channel <= 48) {
        offsetTime = -60;
        offsetChannel = channel - 36;
      }
      // const int base_value = blockMicrosecond_tmp + offsetTime +
      //                        double(20.0 / 12.0) * offsetChannel;
      const int base_value =
          ((_is_lidar_time) ? blockMicrosecond : *usec) + offsetTime;
      const int value = (blocks_num < 3) ? (base_value - 100) : base_value;
      if (value < 0)
      {
        blockSecond_tmp = ((_is_lidar_time) ? blockSecond : *sec) - 1;
        blockMicrosecond_tmp = 1000000 + value;
      }
      else 
        blockMicrosecond_tmp = value;

      double horizontalAngle_modify = 0;
      int laser_blockSize = m_channelNumber / m_laserBlock; // 每块的大小,12
      int laser_blockNum =
          (channel - 1) / laser_blockSize; // 计算当前channel所在的块号(0-based)

      // 判断块号是奇数还是偶数
      if (laser_blockNum % 2 == 0) {
        // 偶数块(0,2,4,6)使用偏移量1
        horizontalAngle_modify = horizontalAngle + m_laserBlockOffset_1;
      } else {
        // 奇数块(1,3,5,7)使用偏移量2
        horizontalAngle_modify = horizontalAngle + m_laserBlockOffset_2;
      }

      TWPoint basic_point;
      basic_point.angle = 360 - horizontalAngle;
      basic_point.mirror = mirror;
      basic_point.channel = channel;


      basic_point.t_sec = blockSecond_tmp;
      basic_point.t_usec = blockMicrosecond_tmp;
      

      double alpha = horizontalAngle_modify * m_calRA; // 水平角度转换为弧度
      if (_echoNum & Echo1) {
        basic_point.x =
            L_1 * m_verticalChannelAngle_TW360_cos_vA_RA[channel - 1] *
                cos(alpha) +
            m_R_TW360 * cos(alpha);
        basic_point.y =
            L_1 * m_verticalChannelAngle_TW360_cos_vA_RA[channel - 1] *
                sin(-alpha) +
            m_R_TW360 * sin(-alpha);
        basic_point.z =
            L_1 * m_verticalChannelAngle_TW360_sin_vA_RA[channel - 1] +
            m_Z_TW360;
        double distanceReal =
            sqrt(basic_point.x * basic_point.x + basic_point.y * basic_point.y +
                 basic_point.z * basic_point.z);
        basic_point.echo = 1;
        basic_point.distance = distanceReal;
        basic_point.pulse = pulse_1;
        basic_point.intensity = std::min<double>(255.0, pulse_1);
        basic_point.confidence = confidence1;
        // basic_point.encoding_interval = _encoding_interval;
        basic_point.state = 0;
        basic_point.label = 0;
        // basic_point.apd_temp = apd_temp;
        //  basic_point.pulseCodeInterval = udpData[24] * 10;
        points.push_back(std::move(basic_point));
      }

      if (_echoNum & Echo2) {
        basic_point.x =
            L_2 * m_verticalChannelAngle_TW360_cos_vA_RA[channel - 1] *
                cos(alpha) +
            m_R_TW360 * cos(alpha);
        basic_point.y =
            L_2 * m_verticalChannelAngle_TW360_cos_vA_RA[channel - 1] *
                sin(-alpha) +
            m_R_TW360 * sin(-alpha);
        basic_point.z =
            L_2 * m_verticalChannelAngle_TW360_sin_vA_RA[channel - 1] +
            m_Z_TW360;
        double distanceReal =
            sqrt(basic_point.x * basic_point.x + basic_point.y * basic_point.y +
                 basic_point.z * basic_point.z);
        basic_point.echo = 2;
        basic_point.distance = distanceReal;
        basic_point.pulse = pulse_2;
        basic_point.intensity = std::min<double>(255.0, pulse_2);
        basic_point.confidence = confidence2;
        // basic_point.encoding_interval = _encoding_interval;
        basic_point.state = 0;
        basic_point.label = 0;
        // basic_point.apd_temp = apd_temp;
        //  basic_point.pulseCodeInterval = udpData[24] * 10;
        points.push_back(std::move(basic_point));
      }
    }
  }
  USEDECODE_COMMON_RETURN
}

bool LidarDevice::DecodeTensor16(const char *udpData, unsigned int *sec,
                                 unsigned int *usec, float *maxAngle,
                                 uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeTensor16(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    UseDecodeTensor16(udpData, points, sec, usec, maxAngle);
    return PointCloudCallback(points, maxAngle);
  }

  return framed;
}

bool LidarDevice::DecodeTensor32(const char *udpData, unsigned int *sec,
                                 unsigned int *usec, float *maxAngle,
                                 uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeTensor32(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    UseDecodeTensor32(udpData, points, sec, usec, maxAngle);
    return PointCloudCallback(points, maxAngle, 0);
  }

  return framed;
}

bool LidarDevice::DecodeScope192(const char *udpData, unsigned int *sec,
                                 unsigned int *usec, float *maxAngle,
                                 uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeScope192(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    UseDecodeScope192(udpData, points, sec, usec, maxAngle);
    return PointCloudCallback(points, maxAngle, 0);
  }

  return framed;
}

bool LidarDevice::DecodeDuetto(const char *udpData, unsigned int *sec,
                               unsigned int *usec, float *maxAngle,
                               uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeDuetto(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    UseDecodeDuetto(udpData, points, sec, usec, maxAngle);
    return PointCloudCallback(points, maxAngle, 1);
  }

  return framed;
}

bool LidarDevice::DecodeTempoA1(const char *udpData, unsigned int *sec,
                                unsigned int *usec, float *maxAngle,
                                uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeTempoA1(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    UseDecodeTempoA1(udpData, points, sec, usec, maxAngle);
    return PointCloudCallback(points, maxAngle, 0);
  }

  return framed;
}

bool LidarDevice::DecodeTempoA2(const char *udpData, unsigned int *sec,
                                unsigned int *usec, float *maxAngle,
                                uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeTempoA2(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    UseDecodeTempoA2(udpData, points, sec, usec, maxAngle);
    return PointCloudCallback(points, maxAngle, 0);
  }

  return framed;
}

bool LidarDevice::DecodeTempoA3(const char *udpData, unsigned int *sec,
                                unsigned int *usec, float *maxAngle,
                                uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeTempoA3(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    UseDecodeTempoA3(udpData, points, sec, usec, maxAngle);
    return PointCloudCallback(points, maxAngle, 0);
  }

  return framed;
}

bool LidarDevice::DecodeTempoA4(const char *udpData, unsigned int *sec,
                                unsigned int *usec, float *maxAngle,
                                uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeTempoA4(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    UseDecodeTempoA4(udpData, points, sec, usec, maxAngle);
    return PointCloudCallback(points, maxAngle, 0);
  }

  return framed;
}

bool LidarDevice::DecodeTempoA4Calib(const char *udpData, unsigned int *sec,
                                     unsigned int *usec, float *maxAngle,
                                     uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeTempoA4Calib(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    UseDecodeTempoA4Calib(udpData, points, sec, usec, maxAngle);
    return PointCloudCallback(points, maxAngle, 0);
  }

  return framed;
}

bool LidarDevice::DecodeTensor48(const char *udpData, unsigned int *sec,
                                 unsigned int *usec, float *maxAngle,
                                 uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    if (PCF_type == 0x00) {
      UseDecodeTensor48(udpData, points, sec, usec, maxAngle);
    } else if (PCF_type == 0x02) {
      UseDecodeTensor48_2(udpData, points, sec, usec, maxAngle);
    }

    PointCloudCallback(points, framed, frameIndex);
  } else {
    if (PCF_type == 0x00) {
      UseDecodeTensor48(udpData, points, sec, usec, maxAngle);
    } else if (PCF_type == 0x02) {
      UseDecodeTensor48_2(udpData, points, sec, usec, maxAngle);
    }

    return PointCloudCallback(points, maxAngle, 1);
  }

  return framed;
}

bool LidarDevice::DecodeTensor48Calib(const char *udpData, unsigned int *sec,
                                      unsigned int *usec, float *maxAngle,
                                      uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeTensor48Calib(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    UseDecodeTensor48Calib(udpData, points, sec, usec, maxAngle);
    return PointCloudCallback(points, maxAngle, 1);
  }

  return framed;
}

bool LidarDevice::DecodeTensor48Depth(const char *udpData, unsigned int *sec,
                                      unsigned int *usec, float *maxAngle,
                                      uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeTensor48Depth(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    UseDecodeTensor48Depth(udpData, points, sec, usec, maxAngle);
    return PointCloudCallback(points, maxAngle, 1, -1, 0);
  }

  return framed;
}

bool LidarDevice::DecodeScope256(const char *udpData, unsigned int *sec,
                                 unsigned int *usec, float *maxAngle,
                                 uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);
  // char udpDatas[1500] = {0};
  // memcpy(udpDatas, udpData, 1348);
  // WallThicknessScope256(udpDatas);

  if (_parsed) {
    UseDecodeScope256(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);

  } else if (!_cycle_count_frame_split) {
    UseDecodeScope256(udpData, points, sec, usec, maxAngle);
    return PointCloudCallback(points, maxAngle, 1, 0);
  } else {
    bool frameSplit = UseDecodeScope256(udpData, points, sec, usec, maxAngle);
    if (!maxAngle) {
      // points[points.size() - 1].framesplit = frameSplit;
      PointCloudCallback(points, frameSplit, frameIndex);
    }
    return frameSplit;
  }

  return framed;
}

bool LidarDevice::DecodeScope256Depth(const char *udpData, unsigned int *sec,
                                      unsigned int *usec, float *maxAngle,
                                      uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeScope256Depth(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    UseDecodeScope256Depth(udpData, points, sec, usec, maxAngle);
    return PointCloudCallback(points, maxAngle, 1, 0, 0);
  }

  return framed;
}

bool LidarDevice::DecodeFocusB1(const char *udpData, unsigned int *sec,
                                unsigned int *usec, float *maxAngle,
                                uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeFocusB1(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    UseDecodeFocusB1(udpData, points, sec, usec, maxAngle);
    return PointCloudCallback(points, maxAngle, 0);
  }

  return framed;
}

bool LidarDevice::DecodeFocusB2(const char *udpData, unsigned int *sec,
                                unsigned int *usec, float *maxAngle,
                                uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);
  if (_parsed) {
    UseDecodeFocusB2(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    UseDecodeFocusB2(udpData, points, sec, usec, maxAngle);
    return PointCloudCallback(points, maxAngle, 0);
  }

  return framed;
}

bool LidarDevice::DecodeFocusB2Calib(const char *udpData, unsigned int *sec,
                                     unsigned int *usec, float *maxAngle,
                                     uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeFocusB2Calib(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    UseDecodeFocusB2Calib(udpData, points, sec, usec, maxAngle);
    return PointCloudCallback(points, maxAngle, 0);
  }

  return framed;
}

bool LidarDevice::DecodeFocusB2_64(const char *udpData, unsigned int *sec,
                                   unsigned int *usec, float *maxAngle,
                                   uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeFocusB2Calib(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    UseDecodeFocusB2Calib(udpData, points, sec, usec, maxAngle);
    // return PointCloudCallback(points, maxAngle, 0);
    return PointCloudCallback(points, maxAngle);
  }

  return framed;
}

bool LidarDevice::DecodeFocusT(const char *udpData, unsigned int *sec,
                               unsigned int *usec, float *maxAngle,
                               uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeFocusT(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    bool frameSplit = UseDecodeFocusT(udpData, points, sec, usec, maxAngle);
    if (!maxAngle) {
      PointCloudCallback(points, frameSplit, frameIndex);
    }
    return frameSplit;
  }

  return framed;
}
bool LidarDevice::DecodeFocusT_2(const char *udpData, unsigned int *sec,
                                 unsigned int *usec, float *maxAngle,
                                 uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeFocusT_2(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    bool frameSplit = UseDecodeFocusT_2(udpData, points, sec, usec, maxAngle);
    if (!maxAngle) {
      PointCloudCallback(points, frameSplit, frameIndex);
    }
    return frameSplit;
  }

  return framed;
}

bool LidarDevice::DecodeFocusTCalib(const char *udpData, unsigned int *sec,
                                    unsigned int *usec, float *maxAngle,
                                    uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {

    UseDecodeFocusTCalib(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    bool frameSplit =
        UseDecodeFocusTCalib(udpData, points, sec, usec, maxAngle);
    if (!maxAngle) {
      PointCloudCallback(points, frameSplit, frameIndex);
    }
    return frameSplit;
  }

  return framed;
}

bool LidarDevice::DecodeScope128(const char *udpData, unsigned int *sec,
                                 unsigned int *usec, float *maxAngle,
                                 uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeScope128(udpData, points, sec, usec, maxAngle);
    points[points.size() - 1].framesplit = framed;
    PointCloudCallback(points, framed, frameIndex);
  } else {
    bool frameSplit = UseDecodeScope128(udpData, points, sec, usec, maxAngle);
    if (!maxAngle) {
      points[points.size() - 1].framesplit = frameSplit;
      PointCloudCallback(points, frameSplit, frameIndex);
    }
    return frameSplit;
  }

  return framed;
}

bool LidarDevice::DecodeScope128_2(const char *udpData, unsigned int *sec,
                                   unsigned int *usec, float *maxAngle,
                                   uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeScope128_2(udpData, points, sec, usec, maxAngle);
    points[points.size() - 1].framesplit = framed;
    PointCloudCallback(points, framed, frameIndex);
  } else {
    bool frameSplit = UseDecodeScope128_2(udpData, points, sec, usec, maxAngle);
    if (!maxAngle) {
      points[points.size() - 1].framesplit = frameSplit;
      PointCloudCallback(points, frameSplit, frameIndex);
    }
    return frameSplit;
  }

  return framed;
}

bool LidarDevice::DecodeScope128Calib(const char *udpData, unsigned int *sec,
                                      unsigned int *usec, float *maxAngle,
                                      uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeScope128Calib(udpData, points, sec, usec, maxAngle);
    points[points.size() - 1].framesplit = framed;
    PointCloudCallback(points, framed, frameIndex);
  } else {
    bool frameSplit =
        UseDecodeScope128Calib(udpData, points, sec, usec, maxAngle);
    if (!maxAngle) {
      points[points.size() - 1].framesplit = frameSplit;
      PointCloudCallback(points, frameSplit, frameIndex);
    }
    return frameSplit;
  }

  return framed;
}

bool LidarDevice::DecodeScope128F(const char *udpData, unsigned int *sec,
                                  unsigned int *usec, float *maxAngle,
                                  uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeScope128F(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    bool frameSplit = UseDecodeScope128F(udpData, points, sec, usec, maxAngle);
    if (!maxAngle)
      PointCloudCallback(points, frameSplit, frameIndex);

    return frameSplit;
  }

  return framed;
}

bool LidarDevice::DecodeScope128FCalib(const char *udpData, unsigned int *sec,
                                       unsigned int *usec, float *maxAngle,
                                       uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  bool frameSplit =
      UseDecodeScope128FCalib(udpData, points, sec, usec, maxAngle);
  if (_parsed) {
    PointCloudCallback(points, framed, frameIndex);
  } else {
    if (!maxAngle)
      PointCloudCallback(points, frameSplit, frameIndex);

    return frameSplit;
  }

  return framed;
}

bool LidarDevice::DecodeTW360(const char *udpData, unsigned int *sec,
                              unsigned int *usec, float *maxAngle,
                              uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeTW360(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    bool frameSplit = UseDecodeTW360(udpData, points, sec, usec, maxAngle);
    if (!maxAngle) {
      PointCloudCallback(points, frameSplit, frameIndex);
    }
    return frameSplit;
  }

  return framed;
}
bool LidarDevice::DecodeTW360_2(const char *udpData, unsigned int *sec,
                                unsigned int *usec, float *maxAngle,
                                uint64_t frameIndex, bool framed) {
  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {
    UseDecodeTW360_2(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    bool frameSplit = UseDecodeTW360_2(udpData, points, sec, usec, maxAngle);
    if (!maxAngle) {
      PointCloudCallback(points, frameSplit, frameIndex);
    }
    return frameSplit;
  }

  return framed;
}

bool LidarDevice::DecodeTW360Calib(const char *udpData, unsigned int *sec,
                                   unsigned int *usec, float *maxAngle,
                                   uint64_t frameIndex, bool framed) {

  TWPointCloud::Points points;
  points.reserve(300);

  if (_parsed) {

    UseDecodeTW360Calib(udpData, points, sec, usec, maxAngle);
    PointCloudCallback(points, framed, frameIndex);
  } else {
    bool frameSplit = UseDecodeTW360Calib(udpData, points, sec, usec, maxAngle);
    if (!maxAngle) {
      PointCloudCallback(points, frameSplit, frameIndex);
    }
    return frameSplit;
  }

  return framed;
}

void LidarDevice::SetDuettoVerticalAngleType(int type, double offsetVerAngleL,
                                             double offsetVerAngleR) {
  // Duetto
  for (int i = 0; i < 16; i++) {
    double vA_L = m_verticalChannelsAngle_Duetto16L[i] + offsetVerAngleL;
    m_verticalChannelAngle_Duetto16L_cos_vA_RA[i] = cos(vA_L * m_calRA);
    m_verticalChannelAngle_Duetto16L_sin_vA_RA[i] = sin(vA_L * m_calRA);

    double vA_R = m_verticalChannelsAngle_Duetto16R[i] + offsetVerAngleR;
    m_verticalChannelAngle_Duetto16R_cos_vA_RA[i] = cos(vA_R * m_calRA);
    m_verticalChannelAngle_Duetto16R_sin_vA_RA[i] = sin(vA_R * m_calRA);
  }
}

void LidarDevice::DecodeDIFImp(const char *udpData,
                               DeviceInfoFrame &deviceInfoFrame) {
  _isLidarReady = true;
  deviceInfoFrame.header =
      FourHexToInt(udpData[0], udpData[1], udpData[2], udpData[3]);
  deviceInfoFrame.deviceType =
      FourHexToInt(udpData[8], udpData[9], udpData[10], udpData[11]);
  deviceInfoFrame.deviceNumber =
      FourHexToInt(udpData[12], udpData[13], udpData[14], udpData[15]);

  char version[16] = {'\0'};
  snprintf(version, sizeof(version), "%d.%d.%d", udpData[17], udpData[18],
           udpData[19]);

  deviceInfoFrame.psVersion = version;

  snprintf(version, sizeof(version), "%d.%d.%d", udpData[21], udpData[22],
           udpData[23]);

  deviceInfoFrame.plVersion = version;

  deviceInfoFrame.workMode = udpData[32];
  deviceInfoFrame.workStatus = udpData[33];
  memcpy(deviceInfoFrame.privateData, udpData, 1024);
}

void LidarDevice::DecodeDIFData_Duetto(const char *udpData) {
  // ABC
  int hex8_mirrorA = (unsigned char)udpData[508 + 4 * 14 + 1];
  int hex8_mirrorB = (unsigned char)udpData[508 + 4 * 14 + 2];
  int hex8_mirrorC = (unsigned char)udpData[508 + 4 * 14 + 3];

  double mirrorA = (hex8_mirrorA - 128) * 0.01 + (-4.5);
  double mirrorB = (hex8_mirrorB - 128) * 0.01 + (0);
  double mirrorC = (hex8_mirrorC - 128) * 0.01 + (4.5);

  unsigned int offsetAngle =
      FourHexToInt(udpData[508 + 4 * 30 + 0], udpData[508 + 4 * 30 + 1],
                   udpData[508 + 4 * 30 + 2], udpData[508 + 4 * 30 + 3]);
  double offsetVerAngleL =
      (((int)((offsetAngle >> 9) & 0x000001FF)) - 256) * 0.01 +
      m_LeftRightMechCoreVerAngleOffset[0];
  double offsetVerAngleR = (((int)(offsetAngle & 0x000001FF)) - 256) * 0.01 +
                           m_LeftRightMechCoreVerAngleOffset[1];

  //
  unsigned short hexMoveAngleL =
      TwoHextoInt(udpData[508 + 4 * 31 + 0], udpData[508 + 4 * 31 + 1]);
  unsigned short hexMoveAngleR =
      TwoHextoInt(udpData[508 + 4 * 31 + 2], udpData[508 + 4 * 31 + 3]);
  double moveAngleL =
      hexMoveAngleL * 0.01 + m_LeftRightMechCoreHorAngleOffset[0];
  double moveAngleR =
      hexMoveAngleR * 0.01 + m_LeftRightMechCoreHorAngleOffset[1];

  // NX NY NZ
  int hexPivotVectorX =
      TwoHextoInt(udpData[508 + 4 * 48 + 0], udpData[508 + 4 * 48 + 1]);
  int hexPivotVectorY =
      TwoHextoInt(udpData[508 + 4 * 48 + 2], udpData[508 + 4 * 48 + 3]);
  int hexPivotVectorZ =
      TwoHextoInt(udpData[508 + 4 * 49 + 2], udpData[508 + 4 * 49 + 3]);
  double pivotVectorX = (hexPivotVectorX - 32768) * 0.0001;
  double pivotVectorY = (hexPivotVectorY - 32768) * 0.0001;
  double pivotVectorZ = (hexPivotVectorZ - 32768) * 0.0001;

  if (!IsEqualityFloat3((mirrorA + m_MirrorVerAngleOffset[0]),
                        m_skewing_duetto_Angle[0])) {
    m_skewing_duetto_Angle[0] = mirrorA + m_MirrorVerAngleOffset[0];
    m_skewing_sin_duetto[0] = sin(m_skewing_duetto_Angle[0] * m_calRA);
    m_skewing_cos_duetto[0] = cos(m_skewing_duetto_Angle[0] * m_calRA);
    // std::cout << "Mirror: A, " << mirrorA << std::endl;
  }
  if (!IsEqualityFloat3((mirrorB + m_MirrorVerAngleOffset[1]),
                        m_skewing_duetto_Angle[1])) {
    m_skewing_duetto_Angle[1] = mirrorB + m_MirrorVerAngleOffset[1];
    m_skewing_sin_duetto[1] = sin(m_skewing_duetto_Angle[1] * m_calRA);
    m_skewing_cos_duetto[1] = cos(m_skewing_duetto_Angle[1] * m_calRA);
    // std::cout << "Mirror: B, " << mirrorB << std::endl;
  }
  if (!IsEqualityFloat3((mirrorC + m_MirrorVerAngleOffset[2]),
                        m_skewing_duetto_Angle[2])) {
    m_skewing_duetto_Angle[2] = mirrorC + m_MirrorVerAngleOffset[2];
    m_skewing_sin_duetto[2] = sin(m_skewing_duetto_Angle[2] * m_calRA);
    m_skewing_cos_duetto[2] = cos(m_skewing_duetto_Angle[2] * m_calRA);
    // std::cout << "Mirror: C, " << mirrorC << std::endl;
  }

  if (!IsEqualityFloat3(moveAngleL, m_leftMoveAngle)) {
    m_leftMoveAngle = moveAngleL;
    m_rotate_duetto_sinL = sin(m_leftMoveAngle * m_calRA); //
    m_rotate_duetto_cosL = cos(m_leftMoveAngle * m_calRA); //
    // std::cout << "MoveAngle: L, " << moveAngleL << std::endl;
  }
  if (!IsEqualityFloat3(moveAngleR, m_rightMoveAngle)) {
    m_rightMoveAngle = moveAngleR;
    m_rotate_duetto_sinR = sin(m_rightMoveAngle * m_calRA); //
    m_rotate_duetto_cosR = cos(m_rightMoveAngle * m_calRA); //
    // std::cout << "MoveAngle: R, " << moveAngleR << std::endl;
  }

  if (!IsEqualityFloat5(pivotVectorX, duettoPivotVector[0])) {
    duettoPivotVector[0] = pivotVectorX;
    // std::cout << "PivotVector: X, " << pivotVectorX << std::endl;
  }
  if (!IsEqualityFloat5(pivotVectorY, duettoPivotVector[1])) {
    duettoPivotVector[1] = pivotVectorY;
    // std::cout << "PivotVector: Y, " << pivotVectorY << std::endl;
  }
  if (!IsEqualityFloat5(pivotVectorZ, duettoPivotVector[2])) {
    duettoPivotVector[2] = pivotVectorZ;
    // std::cout << "PivotVector: Z, " << pivotVectorZ << std::endl;
  }

  if ((!IsEqualityFloat3(offsetVerAngleL, m_offsetVerAngleL)) ||
      (!IsEqualityFloat3(offsetVerAngleR, m_offsetVerAngleR))) {
    m_offsetVerAngleL = offsetVerAngleL;
    m_offsetVerAngleR = offsetVerAngleR;

    SetDuettoVerticalAngleType(0, m_offsetVerAngleL, m_offsetVerAngleR);
  }

  DeviceInfoFrame deviceInfoFrame;
  DecodeDIFImp(udpData, deviceInfoFrame);

  _lidarObserver->OnDeviceInfoFrame(_lidarInfo, deviceInfoFrame);
}

void LidarDevice::DecodeIMUData(const char *udpData) {

  IMUData imu_data;

  // time
  unsigned int t_sec =
      FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
  double t_usec =
      FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;

  imu_data.stamp = (uint64_t)((uint64_t)(t_sec)*1000 * 1000 + t_usec);

  // status calibrate
  unsigned char hex_status = udpData[35];
  hex_status = hex_status << 7;
  unsigned short leftOrRight = hex_status >> 7;
  imu_data.calibrate = (0 == leftOrRight) ? false : true;

  // temperature
  imu_data.temperature =
      (float)((TwoHextoInt(udpData[36], udpData[37]) - 32768) * (1.0 / 256));

  // angular velocity
  imu_data.angular_velocity[0] =
      (float)((TwoHextoInt(udpData[38], udpData[39]) - 32768) * 0.0005);
  imu_data.angular_velocity[1] =
      (float)((TwoHextoInt(udpData[40], udpData[41]) - 32768) * 0.0005);
  imu_data.angular_velocity[2] =
      (float)((TwoHextoInt(udpData[42], udpData[43]) - 32768) * 0.0005);

  // linear acceleration
  imu_data.linear_acceleration[0] =
      (float)((TwoHextoInt(udpData[44], udpData[45]) - 32768) * 0.0002);
  imu_data.linear_acceleration[1] =
      (float)((TwoHextoInt(udpData[46], udpData[47]) - 32768) * 0.0002);
  imu_data.linear_acceleration[2] =
      (float)((TwoHextoInt(udpData[48], udpData[49]) - 32768) * 0.0002);

  imu_data.gyro_noise =
      FourHexToFloat(udpData[50], udpData[51], udpData[52], udpData[53]);
  imu_data.gyro_bias =
      FourHexToFloat(udpData[54], udpData[55], udpData[56], udpData[57]);

  imu_data.accel_noise =
      FourHexToFloat(udpData[58], udpData[59], udpData[60], udpData[61]);
  imu_data.accel_bias =
      FourHexToFloat(udpData[62], udpData[63], udpData[64], udpData[65]);

  _lidarObserver->OnIMU(_lidarInfo, imu_data);
}
void LidarDevice::DecodeIMUData_TW360(const char *udpData, unsigned int *t_sec,
                                      unsigned int *t_usec) {
  IMUData imu_data;

  // time
  unsigned int frameSecond =
      FourHexToInt(udpData[10], udpData[11], udpData[12], udpData[13]);
  double frameMicrosecond =
      FourHexToInt(udpData[14], udpData[15], udpData[16], udpData[17]) * 0.1;
  if (_is_lidar_time) {
    imu_data.stamp =
        (uint64_t)((uint64_t)(frameSecond)*1000 * 1000 + frameMicrosecond);
  } else {
    imu_data.stamp = (uint64_t)((uint64_t)(*t_sec) * 1000 * 1000 + *t_usec);
  }

  // std::cout<<"imu_data.stamp"<<imu_data.stamp<<std::endl;

  // gyro_factor 角速度系数
  int gyro_factor = udpData[18];
  // 根据gyro_factor选择正确的转换系数
  float gyro_scale;
  switch (gyro_factor) {
  case 0:
    gyro_scale = 1.0f / 16.4f;
    break;
  case 1:
    gyro_scale = 1.0f / 32.8f;
    break;
  case 2:
    gyro_scale = 1.0f / 65.5f;
    break;
  case 3:
    gyro_scale = 1.0f / 131.0f;
    break;
  case 4:
    gyro_scale = 1.0f / 262.0f;
    break;
  case 5:
    gyro_scale = 1.0f / 524.3f;
    break;
  case 6:
    gyro_scale = 1.0f / 1048.6f;
    break;
  case 7:
    gyro_scale = 1.0f / 2097.2f;
    break;
  default:
    gyro_scale = 1.0f;
    break; // 默认值
  }
  // angular velocity
  imu_data.angular_velocity[0] =
      (float)((SignedTwoHextoInt(udpData[19], udpData[20])) * gyro_scale *
              m_calRA); // °/s转换为rad/s
  imu_data.angular_velocity[1] =
      (float)((SignedTwoHextoInt(udpData[21], udpData[22])) * gyro_scale *
              m_calRA);
  imu_data.angular_velocity[2] =
      (float)((SignedTwoHextoInt(udpData[23], udpData[24])) * gyro_scale *
              m_calRA);

  // accel_factor加速度系数
  int accel_factor = udpData[25];
  float accel_scale;
  switch (accel_factor) {
  case 0:
    accel_scale = 1.0f / 2048.0f;
    break;
  case 1:
    accel_scale = 1.0f / 4096.0f;
    break;
  case 2:
    accel_scale = 1.0f / 8192.0f;
    break;
  case 3:
    accel_scale = 1.0f / 16384.0f;
    break;
  default:
    accel_scale = 1.0f;
    break; // 默认值
  }
  // linear acceleration
  imu_data.linear_acceleration[0] =
      (float)((SignedTwoHextoInt(udpData[26], udpData[27])) *
              accel_scale); // 单位g
  imu_data.linear_acceleration[1] =
      (float)((SignedTwoHextoInt(udpData[28], udpData[29])) * accel_scale);
  imu_data.linear_acceleration[2] =
      (float)((SignedTwoHextoInt(udpData[30], udpData[31])) * accel_scale);

  _lidarObserver->OnIMU(_lidarInfo, imu_data);
}

void LidarDevice::DecodeDIFData_Tensor48(const char *udpData) {
  _lidarTypeFromDIF = (unsigned int)(udpData[11]);
  // ABC
  int hex8_mirrorA = (unsigned char)udpData[508 + 4 * 14 + 1];
  int hex8_mirrorB = (unsigned char)udpData[508 + 4 * 14 + 2];
  int hex8_mirrorC = (unsigned char)udpData[508 + 4 * 14 + 3];

  float mirrorA = (hex8_mirrorA - 128) * 0.01 + (-4.5);
  float mirrorB = (hex8_mirrorB - 128) * 0.01 + (0);
  float mirrorC = (hex8_mirrorC - 128) * 0.01 + (4.5);

  if (!IsEqualityFloat3(mirrorA + m_MirrorVerAngleOffset[0],
                        m_skewing_tsp48_Angle[0])) {
    m_skewing_tsp48_Angle[0] = mirrorA + m_MirrorVerAngleOffset[0];
    m_skewing_sin_tsp48[0] = sin(m_skewing_tsp48_Angle[0] * m_calRA);
    m_skewing_cos_tsp48[0] = cos(m_skewing_tsp48_Angle[0] * m_calRA);
    // std::cout << "Tensor48-Polar Mirror: A, " << mirrorA << std::endl;
  }
  if (!IsEqualityFloat3(mirrorB + m_MirrorVerAngleOffset[1],
                        m_skewing_tsp48_Angle[1])) {
    m_skewing_tsp48_Angle[1] = mirrorB + m_MirrorVerAngleOffset[1];
    m_skewing_sin_tsp48[1] = sin(m_skewing_tsp48_Angle[1] * m_calRA);
    m_skewing_cos_tsp48[1] = cos(m_skewing_tsp48_Angle[1] * m_calRA);
    // std::cout << "Tensor48-Polar Mirror: B, " << mirrorB << std::endl;
  }
  if (!IsEqualityFloat3(mirrorC + m_MirrorVerAngleOffset[2],
                        m_skewing_tsp48_Angle[2])) {
    m_skewing_tsp48_Angle[2] = mirrorC + m_MirrorVerAngleOffset[2];
    m_skewing_sin_tsp48[2] = sin(m_skewing_tsp48_Angle[2] * m_calRA);
    m_skewing_cos_tsp48[2] = cos(m_skewing_tsp48_Angle[2] * m_calRA);
    // std::cout << "Tensor48-Polar Mirror: C, " << mirrorC << std::endl;
  }
  PCF_type = (udpData[31] & 0xF0) >> 4;

  DeviceInfoFrame deviceInfoFrame;
  DecodeDIFImp(udpData, deviceInfoFrame);

  _lidarObserver->OnDeviceInfoFrame(_lidarInfo, deviceInfoFrame);
}

void LidarDevice::DecodeDIFData_Scope256(const char *udpData) {
  _lidarTypeFromDIF = (unsigned int)(udpData[11]);
  int diftype = udpData[31];

  if (diftype == 0) {
    // beta
    unsigned int offsetAngle =
        FourHexToInt(udpData[508 + 4 * 30 + 0], udpData[508 + 4 * 30 + 1],
                     udpData[508 + 4 * 30 + 2], udpData[508 + 4 * 30 + 3]);
    double offsetVerAngleL =
        (((int)((offsetAngle >> 9) & 0x000001FF)) - 256) * 0.01;
    double offsetVerAngleR = (((int)(offsetAngle & 0x000001FF)) - 256) * 0.01;

    // gamma
    unsigned int moveAngle =
        FourHexToInt(udpData[508 + 4 * 31 + 0], udpData[508 + 4 * 31 + 1],
                     udpData[508 + 4 * 31 + 2], udpData[508 + 4 * 31 + 3]);
    double moveAngleL = 180 -
                        (((int)((moveAngle >> 10) & 0x000003FF))) * 0.020833 +
                        m_LeftRightMechCoreHorAngleOffset[0];
    double moveAngleR = (((int)(moveAngle & 0x000003FF))) * 0.020833 +
                        m_LeftRightMechCoreHorAngleOffset[1];

    // ABC
    double mirrorABCAmend[3] = {
        (((int)((unsigned char)udpData[508 + 4 * 14 + 1])) - 128) * 0.01,
        (((int)((unsigned char)udpData[508 + 4 * 14 + 2])) - 128) * 0.01,
        (((int)((unsigned char)udpData[508 + 4 * 14 + 3])) - 128) * 0.01,
    };

    if (!IsEqualityFloat3(moveAngleL, m_scp256MoveAngleL) ||
        !IsEqualityFloat3(moveAngleR, m_scp256MoveAngleR)) {
      m_scp256MoveAngleL = moveAngleL;
      m_scp256MoveAngleR = moveAngleR;

      // std::cout << "Scope256,MoveAngle: " << m_scp256MoveAngleL << "," <<
      // m_scp256MoveAngleR << std::endl;
      m_rotate_scp256L_sin = sin(m_scp256MoveAngleL * m_calRA);
      m_rotate_scp256L_cos = cos(m_scp256MoveAngleL * m_calRA);
      m_rotate_scp256R_sin = sin(m_scp256MoveAngleR * m_calRA);
      m_rotate_scp256R_cos = cos(m_scp256MoveAngleR * m_calRA);
    }

    for (int i = 0; i < 3; i++) {
      if (!IsEqualityFloat3(mirrorABCAmend[i] + m_MirrorVerAngleOffset[i],
                            m_scp256MirrorABCAmend[i])) {
        m_scp256MirrorABCAmend[i] =
            mirrorABCAmend[i] + m_MirrorVerAngleOffset[i];

        // std::cout << "Scope256,Mirror: " << i << "," <<
        // m_scp256MirrorABCAmend[i] << std::endl;

        m_skewing_sin_scp256[i] = sin(m_scp256MirrorABCAmend[i] * m_calRA);
        m_skewing_cos_scp256[i] = cos(m_scp256MirrorABCAmend[i] * m_calRA);
      }
    }

    // 应用参数：机芯竖直偏移角度
    if ((!IsEqualityFloat3(offsetVerAngleL +
                               m_LeftRightMechCoreVerAngleOffset[0],
                           m_scp256OffsetVerAngleL)) ||
        (!IsEqualityFloat3(offsetVerAngleR +
                               m_LeftRightMechCoreVerAngleOffset[1],
                           m_scp256OffsetVerAngleR))) {
      m_scp256OffsetVerAngleL =
          offsetVerAngleL + m_LeftRightMechCoreVerAngleOffset[0];
      m_scp256OffsetVerAngleR =
          offsetVerAngleR + m_LeftRightMechCoreVerAngleOffset[1];

      // std::cout << "Scope256,OffsetVerticalAngle: " <<
      // m_scp256OffsetVerAngleL << "," << m_scp256OffsetVerAngleR <<
      // std::endl;

      // Scope256
      for (int i = 0; i < 64; i++) // 每通道对应出射光线与水平方向夹角
      {
        double vA_L =
            m_verticalChannelsAngle_SCP256L[i] + m_scp256OffsetVerAngleL;
        m_verticalChannelAngle_scp256L_sin_vA_RA[i] = sin(vA_L * m_calRA);
        m_verticalChannelAngle_scp256L_cos_vA_RA[i] = cos(vA_L * m_calRA);

        double vA_R =
            m_verticalChannelsAngle_SCP256R[i] + m_scp256OffsetVerAngleR;
        m_verticalChannelAngle_scp256R_sin_vA_RA[i] = sin(vA_R * m_calRA);
        m_verticalChannelAngle_scp256R_cos_vA_RA[i] = cos(vA_R * m_calRA);
      }
    }

    m_scope256LaserVersion = udpData[508 + 243] & 0x0f;
    m_scope256PLAngleCorrectFlag = (udpData[508 + 52] & 0x10) >> 4;

    for (int i = 0; i < 4; i++) {
      m_Scope256LeftLaserOffset[3 - i] = udpData[508 + 232 + i] / 6.0 - 3.0;
      m_Scope256RightLaserOffset[3 - i] = udpData[508 + 236 + i] / 6.0 - 3.0;
    }

    DeviceInfoFrame deviceInfoFrame;
    DecodeDIFImp(udpData, deviceInfoFrame);

    _lidarObserver->OnDeviceInfoFrame(_lidarInfo, deviceInfoFrame);
  }
}

void LidarDevice::DecodeDIFData_FocusB1(const char *udpData) {
  DeviceInfoFrame deviceInfoFrame;
  bool _set_time_windows =
      (udpData[508 + 44] >> 1) &
      0x01; // 读取是否网页开启了time window模式,(udpData[508 + 44] & 0x02 )>>1
  //_frame_interval = udpData[];  //读取分帧间隔
  if (_set_time_windows && _use_time_windows)
    _enable_time_windows = true;
  else if (_set_time_windows != _use_time_windows)
    _lidarObserver->OnException(
        _lidarInfo, Exception(ERR_TIME_WINDOW,
                              "Enable time window mode failed!")); // 报故障

  DecodeDIFImp(udpData, deviceInfoFrame);

  _lidarObserver->OnDeviceInfoFrame(_lidarInfo, deviceInfoFrame);
}

void LidarDevice::DecodeDIFData_FocusB2(const char *udpData) {
  // 计算ABC面转镜俯仰角（δ）
  double mirrorABCAmend[3] = {0};
  double mirrorABCOffset[3] = {0.3, 0.2, 0.1};

  bool _set_time_windows =
      (udpData[508 + 44] >> 1) &
      0x01; // 读取是否网页开启了time window模式,(udpData[508 + 44] & 0x02 )>>1
  //_frame_interval = udpData[];  //读取分帧间隔
  if (_set_time_windows && _use_time_windows)
    _enable_time_windows = true;
  else if (_set_time_windows != _use_time_windows)
    _lidarObserver->OnException(
        _lidarInfo, Exception(ERR_TIME_WINDOW,
                              "Enable time window mode failed!")); // 报故障

  for (int i = 0; i < 3; i++) {
    // 提前UDP中的三个面的偏移角度
    unsigned char hexACount = udpData[508 + 4 * 14 + (i + 1)];
    hexACount = hexACount << 4;
    unsigned short uACount = hexACount >> 4;
    // A面：δ = K * 0.04 - 0.3
    // B面：δ = K * 0.04 - 0.2
    // C面：δ = K * 0.04 - 0.1
    mirrorABCAmend[i] = uACount * 0.04 - mirrorABCOffset[i];
  }

  for (int i = 0; i < 3; i++) {
    m_FocusB2MirrorABCAmend[i] = mirrorABCAmend[i] + m_MirrorVerAngleOffset[i];
    // 计算sinδ和cosδ
    m_skewing_sin_FocusB2[i] = sin(m_FocusB2MirrorABCAmend[i] * m_calRA);
    m_skewing_cos_FocusB2[i] = cos(m_FocusB2MirrorABCAmend[i] * m_calRA);
  }

  m_tempoA4UserMode = (udpData[31] >> 4) == 0;

  DeviceInfoFrame deviceInfoFrame;
  DecodeDIFImp(udpData, deviceInfoFrame);

  unsigned char high = udpData[508 + 180 + 1] & 0x7f;

  _encoding_interval = high * 10 * 1.5;

  _lidarObserver->OnDeviceInfoFrame(_lidarInfo, deviceInfoFrame);
}

void LidarDevice::DecodeDIFData_FocusT(const char *udpData) {
  // 计算ABC面转镜俯仰角（δ）

  double mirrorABCAmend[3] = {0};
  // double mirrorABCOffset = 0.2;

  int base = 508 + 4 * 14 + 3; // 倒序计算，先计算C镜面
  for (int i = 0; i < 3; i++) {
    // 每个面占用连续的10位，计算起始位: i=0→0, i=1→10, i=2→20
    int start_bit = i * 10;

    // 计算起始字节偏移和位偏移
    int byte_offset = start_bit / 8; // 确定起始所在的字节位置,0,1,2
    int bit_offset = start_bit % 8;  // 确定位偏移,0,2,4

    // 合并两个连续的字节数据
    unsigned short merged_value =
        ((unsigned char)(udpData[base - byte_offset - 1])
         << 8) |                                    // 高字节左移8位
        (unsigned char)udpData[base - byte_offset]; // // 低字节补在末尾
    // 截取目标10位
    merged_value = (merged_value >> (bit_offset)) & 0x3FF; // 右移后，保留低10位

    // 计算修正量 δ = (K-300)*0.001
    mirrorABCAmend[2 - i] = (merged_value - 300) * 0.001;
  }

  for (int i = 0; i < 3; i++) {
    m_FocusTMirrorABCAmend[i] = mirrorABCAmend[i] + m_MirrorVerAngleOffset[i];
    // 计算sinδ和cosδ
    m_skewing_sin_FocusT[i] = sin(m_FocusTMirrorABCAmend[i] * m_calRA);
    m_skewing_cos_FocusT[i] = cos(m_FocusTMirrorABCAmend[i] * m_calRA);
  }

  // m_tempoA4UserMode = (udpData[31] >> 4) == 0;

  DeviceInfoFrame deviceInfoFrame;
  DecodeDIFImp(udpData, deviceInfoFrame);

  _lidarObserver->OnDeviceInfoFrame(_lidarInfo, deviceInfoFrame);
}

void LidarDevice::DecodeDIFData_Scope128(const char *udpData) {
  _lidarTypeFromDIF = (unsigned int)(udpData[11]);
  // ABC
  int hex8_mirrorA = (unsigned char)udpData[508 + 4 * 14 + 1];
  int hex8_mirrorB = (unsigned char)udpData[508 + 4 * 14 + 2];
  int hex8_mirrorC = (unsigned char)udpData[508 + 4 * 14 + 3];

  // 提取该字节的高4位（第4位到第7位）,将结果右移4位便于读取,β角标志位
  int scp128BetaAnglePS = (udpData[750] & 0xF0) >> 4;

  if (scp128BetaAnglePS != m_scp128BetaAnglePS) {
    for (int i = 0; i < 32; i++) {
      if (scp128BetaAnglePS == 1) {
        m_verticalChannelsAngle_SCP128P[i] =
            m_verticalChannelsAngle_SCP128P_v2[i];
        m_verticalChannelsAngle_SCP128N[i] =
            m_verticalChannelsAngle_SCP128N_v2[i];
      } else if (scp128BetaAnglePS == 0) {
        m_verticalChannelsAngle_SCP128P[i] =
            m_verticalChannelsAngle_SCP128P_v1[i];
        m_verticalChannelsAngle_SCP128N[i] =
            m_verticalChannelsAngle_SCP128N_v1[i];
      }
      double vA_P = m_verticalChannelsAngle_SCP128P[i];
      m_verticalChannelAngle_scp128P_sin_vA_RA[i] = sin(vA_P * m_calRA);
      m_verticalChannelAngle_scp128P_cos_vA_RA[i] = cos(vA_P * m_calRA);
      double vA_N = m_verticalChannelsAngle_SCP128N[i];
      m_verticalChannelAngle_scp128N_sin_vA_RA[i] = sin(vA_N * m_calRA);
      m_verticalChannelAngle_scp128N_cos_vA_RA[i] = cos(vA_N * m_calRA);
    }
    m_scp128BetaAnglePS = scp128BetaAnglePS;
  }
  float mirrorA = (hex8_mirrorA - 128) * 0.01;
  float mirrorB = (hex8_mirrorB - 128) * 0.01;
  float mirrorC = (hex8_mirrorC - 128) * 0.01;

  if (!IsEqualityFloat3(mirrorA + m_MirrorVerAngleOffset[0],
                        m_skewing_scope128_Angle[0])) {
    m_skewing_scope128_Angle[0] = mirrorA + m_MirrorVerAngleOffset[0];
    m_skewing_sin_scope128[0] = sin(m_skewing_scope128_Angle[0] * m_calRA);
    m_skewing_cos_scope128[0] = cos(m_skewing_scope128_Angle[0] * m_calRA);
  }

  if (!IsEqualityFloat3(mirrorB + m_MirrorVerAngleOffset[1],
                        m_skewing_scope128_Angle[1])) {
    m_skewing_scope128_Angle[1] = mirrorB + m_MirrorVerAngleOffset[1];
    m_skewing_sin_scope128[1] = sin(m_skewing_scope128_Angle[1] * m_calRA);
    m_skewing_cos_scope128[1] = cos(m_skewing_scope128_Angle[1] * m_calRA);
  }

  if (!IsEqualityFloat3(mirrorC + m_MirrorVerAngleOffset[2],
                        m_skewing_scope128_Angle[2])) {
    m_skewing_scope128_Angle[2] = mirrorC + m_MirrorVerAngleOffset[2];
    m_skewing_sin_scope128[2] = sin(m_skewing_scope128_Angle[2] * m_calRA);
    m_skewing_cos_scope128[2] = cos(m_skewing_scope128_Angle[2] * m_calRA);
  }

  for (int i = 0; i < 4; i++) {
    m_Scope128PosLaserOffset[3 - i] = udpData[508 + 0xE8 + i] / 6.0 - 3.0;
    m_Scope128NegLaserOffset[3 - i] = udpData[508 + 0xEC + i] / 6.0 - 3.0;
  }

  double scope128NegBetaAngleOffset =
      ((((unsigned char)udpData[508 + 0x7A] << 8) |
        (unsigned char)udpData[508 + 0x7B]) &
       0x01ff) *
          0.01 -
      2.56;

  double scope128PosBetaAngleOffset =
      (((((unsigned char)udpData[508 + 0x79] << 8) |
         (unsigned char)udpData[508 + 0x7A]) >>
        1) &
       0x01ff) *
          0.01 -
      2.56;

  if (scope128NegBetaAngleOffset != m_Scope128NegBetaAngleOffset) {
    for (int i = 0; i < 32; i++) {
      double vA_N =
          m_verticalChannelsAngle_SCP128N[i] + scope128NegBetaAngleOffset;
      m_verticalChannelAngle_scp128N_sin_vA_RA[i] = sin(vA_N * m_calRA);
      m_verticalChannelAngle_scp128N_cos_vA_RA[i] = cos(vA_N * m_calRA);
    }

    m_Scope128NegBetaAngleOffset = scope128NegBetaAngleOffset;
  }

  if (scope128PosBetaAngleOffset != m_Scope128PosBetaAngleOffset) {
    for (int i = 0; i < 32; i++) {
      double vA_N =
          m_verticalChannelsAngle_SCP128P[i] + scope128PosBetaAngleOffset;
      m_verticalChannelAngle_scp128P_sin_vA_RA[i] = sin(vA_N * m_calRA);
      m_verticalChannelAngle_scp128P_cos_vA_RA[i] = cos(vA_N * m_calRA);
    }

    m_Scope128PosBetaAngleOffset = scope128PosBetaAngleOffset;
  }

  m_negtive_back_startAngle =
      TwoHextoInt(udpData[508 + 0xB2], udpData[508 + 0xB3]) * 0.01 -
      1.0; // 由于pl计算误差，左右各扩充1°
  m_negtive_back_endAngle =
      TwoHextoInt(udpData[508 + 0xB6], udpData[508 + 0xB7]) * 0.01 + 1.0;
  m_positive_back_startAngle =
      TwoHextoInt(udpData[508 + 0xBA], udpData[508 + 0xBB]) * 0.01 - 1.0;
  m_positive_back_endAngle =
      TwoHextoInt(udpData[508 + 0xBE], udpData[508 + 0xBF]) * 0.01 + 1.0;

  PCF_type = (udpData[31] & 0xF0) >> 4;

  DeviceInfoFrame deviceInfoFrame;
  DecodeDIFImp(udpData, deviceInfoFrame);

  _lidarObserver->OnDeviceInfoFrame(_lidarInfo, deviceInfoFrame);
}

void LidarDevice::DecodeDIFData_TempoA4(const char *udpData) {
  // 计算ABC面转镜俯仰角（δ）
  double mirrorABCAmend[3] = {0};
  double mirrorABCOffset[3] = {0.3, 0.2, 0.1};

  for (int i = 0; i < 3; i++) {
    // 提前UDP中的三个面的偏移角度
    unsigned char hexACount = udpData[508 + 4 * 14 + (i + 1)];
    hexACount = hexACount << 4;
    unsigned short uACount = hexACount >> 4;
    // A面：δ = K * 0.04 - 0.3
    // B面：δ = K * 0.04 - 0.2
    // C面：δ = K * 0.04 - 0.1
    mirrorABCAmend[i] = uACount * 0.04 - mirrorABCOffset[i];
  }

  for (int i = 0; i < 3; i++) {
    m_tempoA4MirrorABCAmend[i] = mirrorABCAmend[i] + m_MirrorVerAngleOffset[i];
    // 计算sinδ和cosδ
    m_skewing_sin_tempoA4[i] = sin(m_tempoA4MirrorABCAmend[i] * m_calRA);
    m_skewing_cos_tempoA4[i] = cos(m_tempoA4MirrorABCAmend[i] * m_calRA);
  }

  m_tempoA4UserMode = (udpData[31] >> 4) == 0;

  DeviceInfoFrame deviceInfoFrame;
  DecodeDIFImp(udpData, deviceInfoFrame);

  _lidarObserver->OnDeviceInfoFrame(_lidarInfo, deviceInfoFrame);
}
// todo
void LidarDevice::DecodeDIFData_TW360(const char *udpData) {
  // 从excel表中的152和154位置读取R和z
  m_R_TW360 = TwoHextoInt(udpData[98 + 152], udpData[98 + 153]) * 0.001;
  m_Z_TW360 = TwoHextoInt(udpData[98 + 154], udpData[98 + 155]) * 0.001;

  // 从158-168位置读取外参
  double extrinsic_x =
      SignedTwoHextoInt(udpData[98 + 158], udpData[98 + 159]) * 0.01;
  double extrinsic_y =
      SignedTwoHextoInt(udpData[98 + 160], udpData[98 + 161]) * 0.01;
  double extrinsic_z =
      SignedTwoHextoInt(udpData[98 + 162], udpData[98 + 163]) * 0.01;
  double extrinsic_roll =
      TwoHextoInt(udpData[98 + 164], udpData[98 + 165]) * 0.1;
  double extrinsic_pitch =
      TwoHextoInt(udpData[98 + 166], udpData[98 + 167]) * 0.1;
  double extrinsic_yaw =
      TwoHextoInt(udpData[98 + 168], udpData[98 + 169]) * 0.1;
  if (m_transformMoveX != extrinsic_x || m_transformMoveY != extrinsic_y ||
      m_transformMoveZ != extrinsic_z) {
    SetXYZTransVec(extrinsic_x, extrinsic_y, extrinsic_z);
  }
  if (m_transformRotateX != extrinsic_roll ||
      m_transformRotateY != extrinsic_pitch ||
      m_transformRotateZ != extrinsic_yaw) {
    // 设置旋转角度,旋转时应用z-y-x（yaw-pith-roll）
    SetXYZRotateAngle(extrinsic_roll, extrinsic_pitch, extrinsic_yaw);
  }

  const int channelNumber = [](uint8_t raw) {
    switch (raw & 0x0F) { // 保留低4位
    case 1:
      return 96;
    case 2:
      return 196;
    default:
      return 48; // case 0和其他情况
    }
  }(udpData[98 + 266]);

  if (m_channelNumber != channelNumber) {
    m_channelNumber = channelNumber;
  }
  const int verticalType = udpData[98 + 265] & 0x0F; // 保留低4位

  if (m_verticalType != verticalType) {
    m_verticalType = verticalType;
    // 使用指针数组避免重复的switch-case
    const float *angleTables[] = {
        m_verticalChannelAngle_TW360_47deg, // case 0
        m_verticalChannelAngle_TW360_50deg, // case 1
        m_verticalChannelAngle_TW360_90deg  // case 2
    };
    // 目前之支持47和50度的FOV，所以限定了verticalType<2
    if (m_channelNumber == 48 && verticalType < 2) {
      const float *srcTable = angleTables[verticalType];
      if (srcTable && m_verticalChannelAngle_TW360) {
        std::copy(srcTable, srcTable + m_channelNumber,
                  m_verticalChannelAngle_TW360);
        for (int i = 0; i < m_channelNumber; i++) {
          double vA = m_verticalChannelAngle_TW360[i];
          m_verticalChannelAngle_TW360_cos_vA_RA[i] = cos(vA * m_calRA);

          m_verticalChannelAngle_TW360_sin_vA_RA[i] = sin(vA * m_calRA);
        }
      }
    }
  }

  // 从excel表中的261读取vertical_angle_compensation，有负数情况
  double verticalOffset =
      SignedTwoHextoInt(udpData[98 + 261], udpData[98 + 262]) * 0.01;
  if (m_verticalOffset != verticalOffset) {
    m_verticalOffset = verticalOffset;
    for (int i = 0; i < m_channelNumber; i++) {
      double vA = m_verticalChannelAngle_TW360[i] + m_verticalOffset;
      m_verticalChannelAngle_TW360_cos_vA_RA[i] = cos(vA * m_calRA);

      m_verticalChannelAngle_TW360_sin_vA_RA[i] = sin(vA * m_calRA);
    }
  }
  const int laserBlock = [this](uint8_t raw) {
    const uint8_t key = raw & 0x0F; // 保留低4位
    auto it = m_laserBlockMap.find(key);
    return (it != m_laserBlockMap.end()) ? it->second : 4; // 默认返回4
  }(udpData[98 + 267]);

  if (m_laserBlock != laserBlock) {
    m_laserBlock = laserBlock;
  }
  m_laserBlockOffset_1 =
      SignedTwoHextoInt(udpData[98 + 268], udpData[98 + 269]) * 0.01;
  m_laserBlockOffset_2 =
      SignedTwoHextoInt(udpData[98 + 270], udpData[98 + 271]) * 0.01;
  // PCF帧类型，用户双回波、用户单回波、标定协议
  PCF_type = (udpData[31] & 0xF0) >> 4;

  DeviceInfoFrame deviceInfoFrame;
  DecodeDIFImp(udpData, deviceInfoFrame);

  _lidarObserver->OnDeviceInfoFrame(_lidarInfo, deviceInfoFrame);
}

void LidarDevice::CalculateRotateAllPointCloud(TWPoint &point) {
  double x_tmp1 = point.x;
  double y_tmp1 = point.y;
  double z_tmp1 = point.z;
  if (m_x_forward_flag) {
    x_tmp1 = point.y;
    y_tmp1 = -point.x;
  }

  if (m_rotationMatrix[0][0] != 1.0 || m_rotationMatrix[1][1] != 1.0 ||
      m_rotationMatrix[2][2] != 1.0 || m_rotationMatrix[0][1] != 0.0 ||
      m_rotationMatrix[0][2] != 0.0 || m_rotationMatrix[1][0] != 0.0 ||
      m_rotationMatrix[1][2] != 0.0 || m_rotationMatrix[2][0] != 0.0 ||
      m_rotationMatrix[2][1] != 0.0) {
    point.x = m_rotationMatrix[0][0] * x_tmp1 +
              m_rotationMatrix[0][1] * y_tmp1 + m_rotationMatrix[0][2] * z_tmp1;
    point.y = m_rotationMatrix[1][0] * x_tmp1 +
              m_rotationMatrix[1][1] * y_tmp1 + m_rotationMatrix[1][2] * z_tmp1;
    point.z = m_rotationMatrix[2][0] * x_tmp1 +
              m_rotationMatrix[2][1] * y_tmp1 + m_rotationMatrix[2][2] * z_tmp1;
  }

  if (m_transformMoveX != 0 || m_transformMoveY != 0 || m_transformMoveZ != 0) {
    point.x += m_transformMoveX;
    point.y += m_transformMoveY;
    point.z += m_transformMoveZ;
  }
}

int LidarDevice::pulseToIntensity(double pulse) {
  int intensity = (pulse / ScopePulseMapValue * 255.0 + 0.5);
  return intensity > 255 ? 255 : intensity;
}

bool LidarDevice::IsEqualityFloat3(const double value1, const double value2) {
  return fabs(value1 - value2) < 0.001;
}

bool LidarDevice::IsEqualityFloat5(const double value1, const double value2) {
  return fabs(value1 - value2) < 0.00001;
}

int LidarDevice::GetDuettoBlockNumber(double angle, int mirror, int lORr) {
  if (angle < m_startAngle && 1 == mirror) {
    m_blockNumberForDuetto[0] = 1;    // RA 1-1000
    m_blockNumberForDuetto[1] = 2001; // RB
    m_blockNumberForDuetto[2] = 4001; // RC
    m_blockNumberForDuetto[3] = 3001; // LA
    m_blockNumberForDuetto[4] = 5001; // LB
    m_blockNumberForDuetto[5] = 1001; // LC
    return 0;
  } else {
    return m_blockNumberForDuetto[mirror + lORr * 3]++;
  }
}

} // namespace tanway
