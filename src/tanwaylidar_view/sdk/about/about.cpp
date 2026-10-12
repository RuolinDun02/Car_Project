#include "about.h"

namespace tanway {
const std::string debugString = DEBUG_STRING;
const bool debugMode = DEBUG_MODE;
const std::string versionNumber = VERSION_NUMBER;

const std::string kTensor16 = "Tensor16";
const std::string kTensor32 = "Tensor32";
const std::string kScope192 = "Scope192";
const std::string kDuetto = "Duetto";
const std::string kTempoA1 = "TempoA1";
const std::string kTempoA2 = "TempoA2";
const std::string kScopeMini = "ScopeMini";
const std::string kTempoA3 = "TempoA3";
const std::string kTempoA4 = "TempoA4";
const std::string kTensor48 = "Tensor48";
const std::string kTensor48_Depth = "Tensor48Depth";
const std::string kScope256 = "Scope256";
const std::string kScope256_Depth = "Scope256Depth";
const std::string kFocusB1 = "FocusB1";
const std::string kFocusB2_B3_MP = "FocusB2/B3/MP";
const std::string kScope256_SmallBlind = "Scope256SmallBlind";
const std::string kScope128H = "Scope128H";
const std::string kScope128 = "Scope128";
const std::string kScope128F = "Scope128F";
const std::string kFocusB2_64 = "FocusB2-64";
const std::string kFocusT = "FocusT";
const std::string kTW360 = "TW360";

std::string LidarTypeToStr(int lidartype) {
  std::string lidarType;
  switch (lidartype) {
  case 0:
    lidarType = kTensor16;
    break;
  case 1:
    lidarType = kTensor32;
    break;
  case 2:
    lidarType = kScope192;
    break;
  case 3:
    lidarType = kDuetto;
    break;
  case 4:
    lidarType = kTempoA1;
    break;
  case 5:
    lidarType = kTempoA2;
    break;
  case 6:
    lidarType = kScopeMini;
    break;
  case 7:
    lidarType = kTempoA3;
    break;
  case 8:
    lidarType = kTempoA4;
    break;
  case 9:
    lidarType = kTensor48;
    break;
  case 10:
    lidarType = kTensor48_Depth;
    break;
  case 11:
    lidarType = kScope256;
    break;
  case 12:
    lidarType = kScope256_Depth;
    break;
  case 13:
    lidarType = kFocusB1;
    break;
  case 14:
    lidarType = kScope256_SmallBlind;
    break;
  case 15:
    lidarType = kFocusB2_B3_MP;
    break;
  case 16:
    lidarType = kScope128H;
    break;
  case 17:
    lidarType = kScope128;
    break;
  case 18:
    lidarType = kScope128F;
    break;
  case 19:
    lidarType = kFocusB2_64;
    break;
  case 20:
    lidarType = kFocusT;
    break;
  case 21:
    lidarType = kTW360;
    break;
  default:
    break;
  }

  return lidarType;
}

} // namespace tanway
