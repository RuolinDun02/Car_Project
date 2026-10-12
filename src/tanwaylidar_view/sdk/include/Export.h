#pragma once

#ifdef _WIN32

#ifndef TANWAY_LIBRARY
#define TANWAY_API_EXPORT __declspec(dllexport)
#else
#define TANWAY_API_EXPORT __declspec(dllimport)
#endif
#else
#if __has_attribute(visibility)
#define TANWAY_API_EXPORT __attribute__((visibility("default")))
#endif
#endif
