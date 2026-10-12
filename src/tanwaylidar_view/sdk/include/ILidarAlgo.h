#pragma once
#include "Export.h"
#include "common.h"

namespace tanway {
class TANWAY_API_EXPORT ILidarAlgo {
public:
  static std::unique_ptr<ILidarAlgo>
  Create(LidarType lidarType, const std::string &jsonConfigFile = "");

  static std::string VersionNumber();
  static std::string DebugString();

  virtual void Process(TWPointCloud::Points &points) = 0;
  virtual bool UpdateAlgoParam(const std::string &paramJson) = 0;
  virtual void ClearCache() = 0;
};
} // namespace tanway
