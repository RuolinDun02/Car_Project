#pragma once
#ifdef _MSC_VER
#else
#include <pthread.h>
#include <sched.h>
#endif
#include <map>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <thread>
#include <time.h>
#include <utility>
namespace tanway {

#define SHED_FIFO_PRIORITY_HIGH 99
#define SHED_FIFO_PRIORITY_MEDIUM 70
#define SHED_FIFO_PRIORITY_LOW 1
#define ISO_8601_FORMAT 1

#ifdef _MSC_VER
extern void SetThreadPriorityWin(int priority);
#else
extern void SetThreadPriority(int policy, int priority);
#endif
#ifndef _MSC_VER
extern unsigned int GetTickCount();
#endif

extern unsigned int GetMicroTickCount();

extern uint64_t GetMicroTickCountU64();

extern int GetAvailableCPUNum();

extern int GetCurrentTimeStamp(std::string &sTime,
                               int nFormat = ISO_8601_FORMAT);
} // namespace tanway
