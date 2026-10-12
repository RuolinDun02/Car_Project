#include "plat_utils.h"
static const int kTimeStrLen = 1000;
#include <iostream>
#ifdef _MSC_VER
#define EPOCHFILETIME (116444736000000000UL)
#include <windows.h>
#else
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>
#define gettid() syscall(SYS_gettid)
#endif
namespace tanway {
#ifdef _MSC_VER
void SetThreadPriorityWin(int priority) {
  auto handle = GetCurrentThread();
  printf("set thread %lu, priority %d\n", std::this_thread::get_id(), priority);
  SetThreadPriority(handle, priority);
  int prior = GetThreadPriority(handle);
  printf("get thead %lu, priority %d\n", std::this_thread::get_id(), prior);
}
#else
void SetThreadPriority(int policy, int priority) {
  printf("set thread %lu, tid %ld, policy %d and priority %d\n", pthread_self(),
         gettid(), policy, priority);
  sched_param param;
  param.sched_priority = priority;
  pthread_setschedparam(pthread_self(), policy, &param);

  int ret_policy;
  pthread_getschedparam(pthread_self(), &ret_policy, &param);
  printf("get thead %lu, tid %ld, policy %d and priority %d\n", pthread_self(),
         gettid(), ret_policy, param.sched_priority);
}
#endif

#ifndef _MSC_VER
unsigned int GetTickCount() {
  unsigned int ret = 0;
#ifdef _MSC_VER
  FILETIME time;
  LARGE_INTEGER larger_int;
  GetSystemTimeAsFileTime(&time);
  larger_int.LowPart = time.dwLowDateTime;
  larger_int.HighPart = time.dwHighDateTime;
  ret = (larger_int.QuadPart - EPOCHFILETIME) / 10000;
#else
  timespec time;
  memset(&time, 0, sizeof(time));
  if (clock_gettime(CLOCK_MONOTONIC, &time) == 0) {
    ret = time.tv_nsec / 1000000 + time.tv_sec * 1000;
  }
#endif
  return ret;
}
#endif

unsigned int GetMicroTickCount() {
  unsigned int ret = 0;
#ifdef _MSC_VER
  FILETIME time;
  LARGE_INTEGER larger_int;
  GetSystemTimeAsFileTime(&time);
  larger_int.LowPart = time.dwLowDateTime;
  larger_int.HighPart = time.dwHighDateTime;
  ret = (larger_int.QuadPart - EPOCHFILETIME) / 10;
#else
  timespec time;
  memset(&time, 0, sizeof(time));
  if (clock_gettime(CLOCK_MONOTONIC, &time) == 0) {
    ret = time.tv_nsec / 1000 + time.tv_sec * 1000000;
  }
#endif
  return ret;
}

uint64_t GetMicroTickCountU64() {
  uint64_t ret = 0;
#ifdef _MSC_VER
  FILETIME time;
  LARGE_INTEGER larger_int;
  GetSystemTimeAsFileTime(&time);
  larger_int.LowPart = time.dwLowDateTime;
  larger_int.HighPart = time.dwHighDateTime;
  ret = (larger_int.QuadPart - EPOCHFILETIME) / 10;
#else
  timespec time;
  memset(&time, 0, sizeof(time));
  if (clock_gettime(CLOCK_MONOTONIC, &time) == 0) {
    ret = time.tv_nsec / 1000 + time.tv_sec * 1000000;
  }
#endif
  return ret;
}

int GetAvailableCPUNum() {
#ifdef _MSC_VER
  SYSTEM_INFO sysInfo;
  GetSystemInfo(&sysInfo);
  int numProcessors = sysInfo.dwNumberOfProcessors;
  return numProcessors;
  return 1;
#else
  return sysconf(_SC_NPROCESSORS_ONLN);
#endif
}

// 2004-05-03T17:30:08+08:00
int GetCurrentTimeStamp(std::string &sTime, int nFormat) {
  time_t currentTime = time(NULL);
  struct tm *pLocalTime = localtime(&currentTime);
  char sFormattedTime[kTimeStrLen];

  if (ISO_8601_FORMAT == nFormat) {
    strftime(sFormattedTime, kTimeStrLen, "%FT%T%z", pLocalTime);

    sTime = std::string(sFormattedTime);
    // either ISO 8601 or C language is stupid, so change 0800 to 08:00
    sTime = sTime.insert(sTime.length() - 2, ":");

    return 0;
  } else {
    return -1;
  }
}
} // namespace tanway
