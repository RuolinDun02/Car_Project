#pragma once
#include "Exception.h"
#include "Export.h"
#include "user_define.h"
#include <cfloat>

namespace tanway {
class ILidarAlgo;
/**
 * @brief Interface that is used to callback the Tanway Lidar PointCloud data
 * and access basic information of the device.
 */
class ILidarObserver {
public:
  /**
   * OnPointCloud callback the pointcloud data to the user
   */
  virtual void OnPointCloud(const LidarInfo &lidarInfo,
                            const UserPointCloud &pointCloud) = 0;
  /**
   * OnIMU callback the imu data to the user
   */
  virtual void OnIMU(const LidarInfo &lidarInfo, const IMUData &imu) = 0;
  /**
   * OnException callback the error information in the lidar device
   */
  virtual void OnException(const LidarInfo &lidarInfo, const Exception &e) = 0;
  /**
   * OnDeviceInfoFrame callback the lidar device information data to the user
   */
  virtual void OnDeviceInfoFrame(const LidarInfo &lidarInfo,
                                 const DeviceInfoFrame &deviceInfoFrame) = 0;
  /**
   * OnParsePcapProcess callback the pcap analyze progress information, if
   * process equal to 100, it parse done
   */
  virtual void OnParsePcapProcess(const LidarInfo &lidarInfo, int process,
                                  uint64_t frame, uint64_t stamp) = 0;

protected:
  virtual ~ILidarObserver() {}
};

/**
 * @brief Interface that is used to connect the Tanway Lidar
 */
class TANWAY_API_EXPORT ILidarDevice {
public:
  /**
   * @brief Create the lidar device.
   * @param [in] lidarIPOrPcapPath lidar IP address or pcap path.
   * @param [in] hostIPOrLidarIPForFilter host IP or pcap lidar IP for filter.
   * @param [in] pointcloudPort point cloud port number.
   * @param [in] DIFPort device information port number.
   * @param [in] IMUPort device IMU port number.
   * @param [in] lidarObserver the instance of ILidarObserver
   * @param [in] lidarType lidar type.
   * @param [in] repeat play the pcap, only offline mode takes effect.
   * @return the instance of the lidar device.
   */
  static std::unique_ptr<ILidarDevice>
  Create(const std::string &lidarIPOrPcapPath,
         const std::string &hostIPOrLidarIPForFilter, int pointcloudPort,
         int DIFPort, ILidarObserver *lidarObserver, LidarType lidarType,
         bool repeat = false, int lidarID = 0, int IMUPort = 5900);

  /**
   * @brief Get the version of the sdk
   */
  static std::string VersionNumber();
  /**
   * @brief Get the debug string of the sdk
   */
  static std::string DebugString();

  /**
   * @brief Start the lidar device.
   * @param [in] whether play point cloud, if set false, it only start the
   * lidar device and no point cloud data will callback, only offline mode takes
   * effect.
   * @return true if start the lidar device succeed.
   */
  virtual bool Start(bool play = true) = 0;

  /**
   * @brief Set the pcap play rate.
   * @param [in] the play rate of the pcap
   */
  virtual void SetPlayRate(float rate = 1.0) = 0;

  /**
   * @brief Stop the lidar device.
   */
  virtual void Stop() = 0;

  /**
   * @brief Play or pause the pcap, only offline mode takes effect.
   * @param [in] true is play
   */
  virtual void Play(bool play) = 0;

  /**
   * @brief Seek frame num, only offline mode takes effect.
   * @param [in] the frame index of the pcap
   */
  virtual void SeekFrame(int index) = 0;

  virtual void UpdateFrame() = 0;
  /**
   * @brief Online or offline mode
   * @return true if lidar device is in online mode.
   */
  virtual bool GetPlayMode() = 0;

  /**
   * @brief Set the lidar echo num.
   * @param [in] Echo1 Echo2 or Echo1|Echo2
   */
  virtual void SetEchoNum(EchoNum echoNum = Echo1) = 0;

  /**
   * @brief Set the point cloud frame id.
   * @param [in] frame ID
   */
  virtual void SetFrameID(const std::string &frameID) = 0;

  /**
   * @brief Set the lidar mirror abc vertical angle offset.
   * @param [in] mirror a vertical angle offset, degree
   * @param [in] mirror b vertical angle offset, degree
   * @param [in] mirror c vertical angle offset, degree
   */
  virtual void SetMirrorVerAngleOffset(float a = 0.0, float b = 0.0,
                                       float c = 0.0) = 0;

  /**
   * @brief Set the lidar mirror abc horizontal angle offset.
   * @param [in] mirror a horizontal angle offset, degree
   * @param [in] mirror b horizontal angle offset, degree
   * @param [in] mirror c horizontal angle offset, degree
   */
  virtual void SetMirrorHorAngleOffset(float a = 0.0, float b = 0.0,
                                       float c = 0.0) = 0;

  /**
   * @brief Set the lidar core vertical angle offset.
   * @param [in] left core vertical angle offset, degree
   * @param [in] right core vertical angle offset, degree
   */
  virtual void SetCoreVerAngleOffset(float l = 0.0, float r = 0.0) = 0;

  /**
   * @brief Set the lidar core horizontal angle offset.
   * @param [in] left core horizontal angle offset, degree
   * @param [in] right core horizontal angle offset, degree
   */
  virtual void SetCoreHorAngleOffset(float l = 0.0, float r = 0.0) = 0;

  /**
   * @brief Set the point cloud rotate angle.
   * @param [in] rotate around the x-axis angle, degree
   * @param [in] rotate around the y-axis angle, degree
   * @param [in] rotate around the z-axis angle, degree
   */
  virtual void SetXYZRotateAngle(float x = 0.0, float y = 0.0,
                                 float z = 0.0) = 0;

  /**
   * @brief Set the point cloud translation vector.
   * @param [in] translate along the x-axis, meters
   * @param [in] translate along the y-axis, meters
   * @param [in] translate along the z-axis, meters
   */
  virtual void SetXYZTransVec(float x = 0.0, float y = 0.0, float z = 0.0) = 0;

  /**
   * @brief Set point cloud distance filtering range [min, max)
   * @param [in] min distance, meters
   * @param [in] max distance, meters
   */
  virtual void SetDistanceRange(double min = DBL_MIN, double max = DBL_MAX) = 0;

  /**
   * @brief Set point cloud horizontal angle filtering range [min, max]
   * @param [in] min angle, degree
   * @param [in] max angle, degree
   */
  virtual void SetAngleRange(double min = 30, double max = 150) = 0;

  /**
   * @brief Set the x-axis to point forward (the direction the lidar is facing)
   * and the y-axis to point left
   */
  virtual void SetXForwardFlag() = 0;

  /**
   * @brief Start parse the pcap, it is asynchronous operation, only offline
   * mode takes effect.
   * @return true if parse succeed.
   */
  virtual bool StartParsePcap() = 0;

  /**
   * @brief Stop parse the pcap, only offline mode takes effect.
   */
  virtual void StopParsePcap() = 0;

  /**
   * @brief Get total number of point cloud frames of the pcap, only offline
   * mode takes effect.
   */
  virtual int GetPcapFrameNum() = 0;

  /**
   * @brief Set the lidar algorithm
   */
  virtual void SetLidarAlgo(ILidarAlgo *lidarAlgo) = 0;

  virtual void SetTimeStampType(const std::string &timestamp) = 0;

  virtual void SetLidarTime(bool lidartime) = 0;

  virtual void SetFrameSplit(bool frameSplit) = 0;

  virtual void SetTimeWindowMode(bool useTimeWindow) = 0;

  virtual ~ILidarDevice(){};
};

} // namespace tanway
