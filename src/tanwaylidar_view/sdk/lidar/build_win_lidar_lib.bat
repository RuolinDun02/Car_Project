@echo off

REM 设置变量，构建方式为 Release x64
set BUILD_DIR=build
set TARGET_LIB=lidar-AMD64.lib
set CONFIGURATION=Release
set CMAKE_GENERATOR="Visual Studio 16 2019"
set PLATFORM=x64

REM 指定 CMake 的安装路径，如果已经添加到 PATH 中可以省略
set CMAKE_PATH="C:\Program Files\CMake\bin\cmake.exe"

REM 配置 Visual Studio 环境变量，需要根据个人电脑更改路径
call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat" amd64

REM 如果 build 文件夹存在，则删除它
if exist %BUILD_DIR% (
    echo There is already a "build" folder,  deleting it...
    rmdir /s /q %BUILD_DIR%
)

REM 创建新的 build 文件夹
mkdir %BUILD_DIR%

REM 运行 CMake 生成解决方案，并确保是 VS2019 x64
cmake -B %BUILD_DIR% -G %CMAKE_GENERATOR% -A %PLATFORM% -DCMAKE_BUILD_TYPE=%CONFIGURATION%

REM 检查 CMake 是否成功
if %errorlevel% neq 0 (
    echo Fail: CMake failed to generate solution
    pause
    exit /b 1
)

REM 检查 lidar.sln 是否存在
echo Checking for lidar.sln in %BUILD_DIR%
if exist "%BUILD_DIR%\lidar.sln" (
    echo Success: lidar.sln found.
    echo Compiling... 
) else (
    echo Fail: lidar.sln not found.
    pause
    exit /b 1
)

REM 进入 build 文件夹并运行 msbuild 编译 Release 配置，控制只输出“错误”信息到终端中，不显示“警告”

msbuild "%BUILD_DIR%\lidar.sln" /p:Configuration="%CONFIGURATION%" /p:Platform="%PLATFORM%" /p:WarningLevel=0 > NUL 2>&1 


REM 检查 msbuild 是否成功
if %errorlevel% neq 0 (
    echo Fail: VS compilation failed
    pause
    exit /b 1
)

REM 检查目标库文件是否生成
echo Checking for lidar.lib in %BUILD_DIR%\%CONFIGURATION%
if exist %BUILD_DIR%\%CONFIGURATION%\%TARGET_LIB% (
    echo Success: Compilation was successful and generated "%TARGET_LIB%"
) else (
    echo Fail: The target library file is not generated
    pause
    exit /b 1
)

REM 打开 build 目录
start "" "%BUILD_DIR%\%CONFIGURATION%"

REM 显示当前目录，确保已进入目标文件夹
echo Success: Now in the directory: %BUILD_DIR%\%CONFIGURATION%
pause