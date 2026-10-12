#pragma once
#include <string>
namespace tanway {

extern const std::string debugString;
extern const std::string versionNumber;
extern const bool debugMode;

extern const std::string kTensor16;
extern const std::string kTensor32;
extern const std::string kScope192;
extern const std::string kDuetto;

extern const std::string kTempoA1;
extern const std::string kTempoA2;
extern const std::string kScopeMini;
extern const std::string kTempoA3;
extern const std::string kTempoA4;

extern const std::string kTensor48;
extern const std::string kTensor48_Depth;

extern const std::string kScope256;
extern const std::string kScope256_Depth;
extern const std::string kScope256_SmallBlind;
extern const std::string kScope128H;

extern const std::string kFocusB1;
extern const std::string kFocusB2_B3_MP;
extern const std::string kFocusB2_64;
extern const std::string kFocusT;

extern const std::string kScope128;
extern const std::string kScope128F;

extern const std::string kTW360;

std::string LidarTypeToStr(int lidartype);

} // namespace tanway
