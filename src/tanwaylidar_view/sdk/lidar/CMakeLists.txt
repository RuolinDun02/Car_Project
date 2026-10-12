cmake_minimum_required(VERSION 3.0)
add_compile_options(-std=c++14)
project(lidar)

# 添加编译参数
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall")

add_definitions(-DDEBUG_STRING="")
add_definitions(-DDEBUG_MODE="false")
add_definitions(-DVERSION_NUMBER="2.0.6")

# 设置代码构建级别为 Release 方式
set(CMAKE_BUILD_TYPE Release)

add_compile_options(-fPIC)

# Get the current CPU architecture
set(CPU_ARCH ${CMAKE_SYSTEM_PROCESSOR})

# 包含目录
include_directories(${CMAKE_SOURCE_DIR}/../include
                    ${CMAKE_SOURCE_DIR}/../about 
                    ${CMAKE_SOURCE_DIR}/../utils
		    ${CMAKE_SOURCE_DIR}/../log)

set(SOURCES
LidarDevice.cpp
../utils/PackageList.cpp
../about/about.cpp
../log/twlog.cpp
)

if(CMAKE_SYSTEM_NAME MATCHES "Windows")
    add_library(lidar-AMD64 STATIC ${SOURCES})
else()
    if(CPU_ARCH STREQUAL "x86_64")
        add_library(lidar-x86_64 STATIC ${SOURCES})
    elseif(CPU_ARCH MATCHES "aarch64")
        add_library(lidar-aarch64 STATIC ${SOURCES})
    else()
        message(STATUS "Unsupported architecture: ${CPU_ARCH}")
    endif()
endif()


