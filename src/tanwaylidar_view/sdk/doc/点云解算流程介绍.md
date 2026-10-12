# 点云解算流程介绍

本文档详细介绍 SDK 如何解析和发布激光雷达点云数据。下文以 Tensor48 型号雷达为例，通过这一示例，用户可以了解整个数据处理流程，包括数据的接收、解析和回调发布。

# 1 目录

[TOC]

# 2 激光雷达数据处理流程图

![点云解算流程图](./pics/点云解算流程图.png)

上图展示了通用雷达型号的解析及回调过程，整个过程可以分为四个主要部分：雷达初始化、接收数据、解析 UDP 包和单帧点云数据回调。下面将逐一介绍各部分的工作流程。

# 3 激光雷达点云数据处理流程详解

## 3.1 雷达初始化

1. 首先创建激光雷达数据处理类LidarObserver实例（以 Tensor48 型号雷达为例）和点云预处理算法接口类ILidarAlgo实例。
  - LidarObserver 类负责处理来自激光雷达的数据，包括接收数据、解析数据和调用预处理算法。
  - ILidarAlgo 类，定义了一些点云预处理算法的基本方法。注意，算法模块根据用户需要选择是否创建与创建，并非必要。
  - 各参数的详细解释，请参考 [README第5节](../README.md) 。

    ```cpp
    //on-line:using network.
    LidarObserver lidarObserver;
    auto lidar = ILidarDevice::Create("192.168.111.51", "192.168.111.204", 5600, 5700, &lidarObserver, LT_Tensor48);
    
    //off-line:using pcap file to replay.
    //auto lidar = ILidarDevice::Create("test.pcap", "192.168.111.51", 5600, 5700, &lidarObserver, LT_Tensor48);  //pcap包路径使用绝对路径
    
    //use algo:
    auto algo = ILidarAlgo::Create(LT_Tensor48, "algo_table.json");  //json文件路径使用绝对路径
    lidar->SetLidarAlgo(algo.get());
    
    //start lidar
    lidar->Start();
    ```

2. 通过 `Start`函数与 Lidar 设备建立连接，启动Lidar数据接收与处理流程，包括检查回调类对象、验证设备状态、启动工作线程和解析线程等操作。

    ```cpp
    bool LidarDevice::Start(bool play) 
    {      
      // 检查状态、创建_pointCloudPtr点云数据结构、更新_run等参数状态
      // 省略此处代码，详见LidarDevice.cpp
    
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
    ```

    以下为`Start`函数代码块的详解介绍：

    - 检查当前状态并初始化：
      - 检查是否成功创建回调类对象指针 `_lidarObserver`。如果 `_lidarObserver` 为空，将输出错误信息并返回 `false`。
      - 检查当前是否处于停止状态（`LD_STOPED`）。如果处于非停止状态，将输出错误信息并返回 `false`，此时用户需先调用`Stop()`停止后再重新启动。
      - 若运行环境为Windows系统，则需要额外初始化 Winsock 库。如果初始化失败，将输出错误信息并返回 `false`。

    - 创建点云数据结构：
      - 创建一个共享的点云数据结构 `_pointCloudPtr`，并设置其帧 ID 为 `_frameID`，并预留空间。

    - 启动数据接收线程：
      - 基于`isValidIp`函数判断输入参数 `_lidarIPOrPcapPath` 的类型。
      - 若参数为有效IP地址，则启动 `ReadFromNetworkThread` 线程，从网络中读取 Lidar 数据；
      - 若参数非有效IP地址，则启动 `ReadFromLocalThread` 线程，从本地 PCAP 文件中读取 Lidar 数据。
      
    - 启动解析线程：
      - 启动解析线程 `DecodeThread`，负责处理从工作线程接收到的数据。解析过程参考 [3.3节](#3.3 解析UDP包)
      
    - 更新设备状态：
      - 将设备状态更新为 `LD_STARTED`，表示设备已成功启动，并返回 `true`。

## 3.2 数据接收模块

SDK支持两种数据输入方式：以太网数据接收方式与本地PCAP数据读取方式。下文将分别介绍两种方式的处理逻辑。

### 3.2.1 以太网数据接收线程 `ReadFromNetworkThread`

`ReadFromNetworkThread` 基于socket通信，负责从网络中读取UDP数据，并将其存储至 `_packageList` UDP缓存队列中，供解析线程处理。该功能的具体实现位于LidarDevice.cpp，以下为处理流程描述。

1. 创建socket并初始化相关设置

     - 调用 `CreateAndBindSocket`函数，创建数组 `sockets`，分别用于接收点云数据帧、DIF数据帧、IMU数据帧的网络数据。设置对应UDP接收端IP地址、端口参数等网络参数。

     - 若创建socket或bind端口失败，程序将最多尝试5次重新创建和绑定。每次重试之间会有2秒的等待时间，并通过 `_lidarObserver` 报告异常，异常的详细含义参考 [状态码详解](状态码详解.md)。

     - 清除 `_packageList`，确保在新的读取周期中没有遗留数据。

2. 循环接收网络数据并存储至数据队列

     - 设置 `select` 函数的超时时间为1秒。若1秒内未接收到有效UDP数据，则报告超时异常，异常提示信息可参考 [状态码详解](状态码详解.md)。

     - 若 `INPUT_SOCKET_POINT_CLOUD` 套接字有可读的点云数据包, 则检查接收的地址和数据帧的有效性。若数据有效，并将其存入 `_packageList`。

     - 若 `INPUT_SOCKET_DIF` 套接字有可读的DIF数据包, 则检查接收的地址和数据帧的有效性。若数据有效，并将其存入 `_packageList`。

3. 结束时清理资源，并关闭套接字。

### 3.2.2 本地读取线程 `ReadFromLocalThread`

`ReadFromLocalThread` 负责从本地 PCAP文件中读取UDP数据，并将其存储至 `_packageList` UDP缓存队列中，供解析线程处理。该功能的具体实现位于LidarDevice.cpp，以下为处理流程描述。

1. 打开与验证 PCAP 文件
   - 以二进制模式打开指定路径 `_lidarIPOrPcapPath` 的 PCAP 文件。
   - 检查文件头，确保文件有效。若文件无效或无法打开，通过 `_lidarObserver` 报告错误并终止线程，异常提示信息可参考 [状态码详解](状态码详解.md)。
2. Seek 指定帧处理（仅限上位机使用）
   - 若 `_seeking` 为 `true`，执行seek指定帧操作，包括读取目标帧索引、调整文件指针至对应位置、清空 `_packageList` 和点云数据、重置时间戳。
   - 若无需寻址且 `_play` 为 `false`，线程进入短暂休眠状态并重置时间戳。
3. 读取与处理数据包
   - 从 PCAP 文件中读取 UDP 数据包至 `udpData`。
   - 在`ReadUdpPacket()`函数中验证数据包的有效性，接收到无效数据包会中断读取PCAP包。
   - 计算数据包的时间戳，设置帧索引，并将数据包加入 `_packageList`。
4. 文件结束处理
   - 当读取至文件末尾时，通知 `_lidarObserver` 文件读取完毕。
   - 若 `_repeat` 为 `true`，重置文件读取状态，继续循环读取文件。

## 3.3 解析UDP包

以Tensor48型号雷达为例，解析单个UDP包主要调用了5个函数，分别是 `DecodeThread`、`DecodeImp`、`DecodeDIFData_Tensor48`、`DecodeTensor48`、`UseDecodeTensor48_2`。这些函数之间的调用关系如下图所示，下面进行逐一介绍。

![解析UDP包流程图](./pics/解析UDP包流程图.jpg)

### 3.3.1 **`DecodeThread`**

`DecodeThread`是一个解析线程，负责从数据包列表 `_packageList` 中取出数据包，并调用 `DecodeImp` 函数对数据包进行解码。解码完成后，将数据包放回空闲列表中，以便后续复用。

```cpp
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
      continue;
    }

    DecodeImp(packagePtr);
    _packageList.PushFreePackage(packagePtr);
  }
}

```

解析过程：

- 检查数据包列表：解析线程会不断检查数据包列表 `_packageList`，如果列表为空，线程会短暂休眠（1纳秒），然后继续检查。
- 获取数据包：从 `_packageList` 中取出一个数据包 `packagePtr`。
- 过滤数据包：如果数据包的长度为120字节，跳过该数据包。
- 解析数据包：调用 `DecodeImp(packagePtr)` 方法解析数据包。
- 释放数据包：将解析后的数据包放回空闲队列 `_packageList`，以便后续重复使用。

### 3.3.2 **`DecodeImp`**

`DecodeImp`函数通过判断雷达型号和UDP包长度，应用不同的Decode函数进行解析。如果雷达型号或UDP包长不符合预期，则会报提示信息，参考 [状态码详解](状态码详解.md)

```cpp
bool LidarDevice::DecodeImp(const UDPPackage::Ptr &udpData, float *maxAngle,
                            bool *exception) {
  bool ret = false;
  switch (_lidarType) {
  //```````````省略中间代码``````````
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
      PointCloudCallback(udpData->framed, udpData->frameIndex);
    } else
      ON_DECODE_EXCEPTION
    break;
  //```````````省略中间代码``````````
  default:
    ON_DECODE_EXCEPTION
    break;
  }

  return ret;
}
```

**输入参数：**

- `const UDPPackage::Ptr &udpData`：`udpData`是指向 `UDPPackage` 对象的智能指针，存放UDP 数据包的内容、时间戳等信息。
- `float *maxAngle`：`maxAngle`用于存储解析后数据的最大角度值。在线模式下 `maxAngle`为空，离线模式下只有**上位机预加载PCAP包**时，`maxAngle`不为空。
- `bool *exception`：用于指示解析过程中是否发生异常。如果发生异常，`*exception` 会被设置为 `true`。

**返回值：**

- `bool`：解析是否成功。如果解析成功，返回 `true`；否则返回 `false`。该返回值只在**上位机读取PCAP**包时会使用。

**详细解析：**

- 初始化返回值

  - `bool ret = false;`：初始化返回值 `ret` 为 `false`，表示解析未成功。
- 根据 Lidar 类型进行解析

  - 使用 `switch` 语句根据 `_lidarType` 的值选择相应的解析逻辑。
- 处理 `LT_Tensor48` 类型的数据包，根据数据包的长度调用不同的解析函数：

  - `if (udpData->m_length == 1348)`：如果数据包长度为1348字节，接收到的是正常PCF数据包，调用 `DecodeTensor48` 函数进行解析。
  - `else if (udpData->m_length == 1212)`：如果数据包长度为1212字节，接收到的是标定模式下PCF数据包，调用 `DecodeTensor48Calib` 函数进行解析。
  - `else if (udpData->m_length == 1024)`：如果数据包长度为1024字节，接收到的是DIF帧，调用 `DecodeDIFData_Tensor48` 函数进行解析。
  - `else`：如果数据包长度不符合上述任何一种情况，调用 `ON_DECODE_EXCEPTION` 宏处理异常。
- 默认情况

  - `default:`：如果 Lidar 类型不符合任何已知类型，调用 `ON_DECODE_EXCEPTION` 宏处理异常。

### 3.3.3 **`DecodeDIFData_Tensor48`**

此处以 Tensor48 为例，其他型号请参考用户手册的解析协议章节进行解析。

`DecodeDIFData_Tensor48`函数从DIF帧中解算镜面俯仰角，PCF帧数据类型等，其中，`m_skewing_sin_tsp48`、`m_skewing_cos_tsp48`等参数在函数 `InitBasicVariables()`中进行了初始化，该函数在构造 `LidarDevice`对象时被调用，可在 `LidarDevice.cpp`文件中进行查看。

```cpp
void LidarDevice::DecodeDIFData_Tensor48(const char *udpData) {
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
  //```````````省略中间代码``````````
  PCF_type = (udpData[31] & 0xF0) >> 4;

  DeviceInfoFrame deviceInfoFrame;
  DecodeDIFImp(udpData, deviceInfoFrame);

  _lidarObserver->OnDeviceInfoFrame(_lidarInfo, deviceInfoFrame);
}
```

对上述代码进行详解介绍：

- 提取镜面俯仰角信息（参考协议）

  从 UDP 数据包中提取三个镜面的角度信息：

  - `hex8_mirrorA`：从 `udpData[508 + 4 * 14 + 1]` 提取。
  - `hex8_mirrorB`：从 `udpData[508 + 4 * 14 + 2]` 提取。
  - `hex8_mirrorC`：从 `udpData[508 + 4 * 14 + 3]` 提取。
  - 根据协议中规定的偏移量和精度进行处理后，得到转镜ABC面俯仰角
- 更新各镜面俯仰角和正弦/余弦值

  - 检查并更新镜面 A 的俯仰角和正弦/余弦值：如果 `mirrorA + m_MirrorVerAngleOffset[0]` 与 `m_skewing_tsp48_Angle[0]` 不相等，则更新 `m_skewing_tsp48_Angle[0]`，并重新计算 `m_skewing_sin_tsp48[0]` 和 `m_skewing_cos_tsp48[0]`。
  - 检查并更新镜面 B 的倾斜角度和正弦/余弦值：可类比上条。
  - 检查并更新镜面 C 的倾斜角度和正弦/余弦值：可类比上条。
- 提取 PCF 数据类型

  - 从 `udpData[31]` 提取 PCF 类型，并右移4位得到最终的 PCF 类型值，`0x2`表示用户点云帧，其他为预留状态。
- 解析设备信息帧

  - 创建 `DeviceInfoFrame` 对象 `deviceInfoFrame`，`deviceInfoFrame`对象的详细介绍可参考 [参数详解](参数详解.md) 中`DeviceInfoFrame`部分。
  - 调用 `DecodeDIFImp` 函数解析设备信息帧，并将结果存储在 `deviceInfoFrame` 中。
- 触发DIF帧回调

  - 调用 `_lidarObserver->OnDeviceInfoFrame` 方法，发布DIF帧相关数据，只在**ROS环境**下生效

### 3.3.4 **`DecodeTensor48`**

此处以 Tensor48 为例，其他型号请参考用户手册的解析协议章节进行解析。

`DecodeTensor48` 函数用于解析 Tensor48 Lidar 设备的 UDP 数据包，并将解析后的点云数据传递给回调函数`PointCloudCallback()`。

Tensor48 支持两种不同的用户协议，基于DIF帧中用户协议字段 `PCF_type`进行区分：

- 0x00表示用户协议1，使用UsedecodeTensor48解析；

- 0x02表示用户协议2，参考UsedecodeTensor48_2解析；

  在解算前请咨询技术支持确定雷达所用的协议。

```cpp
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
```

**输入参数：**

- `const char *udpData`：指向 UDP 数据包内容的指针。
- `unsigned int *sec`：指向秒数的指针，用于存储时间戳的秒部分，在接收到UDP数据时计算的主机时间。
- `unsigned int *usec`：指向微秒数的指针，用于存储时间戳的微秒部分。
- `float *maxAngle`：指向最大角度的指针，用于存储解析后的最大角度值，详细含义参考前文的解释。
- `uint64_t frameIndex`：当前帧的索引。
- `bool framed`：判断当前PCF包是否应该进行分帧，该参数默认为 `false`，只有当**上位机预解析**到达分帧位置时才会置为 `true`。

**返回参数：**

- `framed`：参考前文的参数含义解释，为 `true`时表示当前包已达分帧位置，应该进行分帧，仅**上位机解析PCAP包**时返回该参数。
- `PointCloudCallback()`函数结果：`PointCloudCallback()`函数中会返回是否组帧成功，`true`为组帧成功，`false`为未收到新一帧。

**功能详解：**

- **初始化点云数据**
  - `TWPointCloud::Points points`：定义一个 `TWPointCloud::Points` 类型的容器 `points`，用于存储解析后的点云数据。`TWPointCloud`是探维的自定义数据类型，支持增加自定义字段，用户可参考 [参数详解第1.6节](参数详解.md)
  - `points.reserve(300);`：预分配300个点的空间，以提高性能。
- **检查解析状态**

  - `_parsed`：该参数表示当前包是否已被上位机预解析过了，仅在**上位机解析PCAP包**时有效，`true`表示完成了预解析过程，`false`表示未完成预解析，当前正在进行预解析。
  - `if (PCF_type == 0x00)`：调用 `UseDecodeTensor48` 函数进行解析。
  - `else if (PCF_type == 0x02)`：调用 `UseDecodeTensor48_2` 函数进行解析。
  - 调用 `PointCloudCallback` 函数处理解析后的点云数据。

### 3.3.5 **`UseDecodeTensor48_2`**

此处以 Tensor48 为例，其他型号请参考用户手册的解析协议章节进行解析。

以下以`UseDecodeTensor48_2`为例介绍解算过程，`UseDecodeTensor48`解析函数可类比。

`UseDecodeTensor48_2`函数用于解析收到的PCF数据包，并将解析后的点云数据存储在一个容器中。该函数处理每个数据块中的点云数据，并计算每个点的笛卡尔坐标、强度、置信度等信息。

```cpp
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

    // 计算blockSecond和blockMicrosecond
    //```````````省略中间代码``````````

    // mirror
    unsigned char hexMirror = udpData[35 + offset_block];
    hexMirror = hexMirror << 5;
    unsigned short mirror = hexMirror >> 6;

    double cos_delta = m_skewing_cos_tsp48[mirror];
    double sin_delta = m_skewing_sin_tsp48[mirror];

    for (int seq = 0; seq < 16; seq++) {
      //解算水平角度、强度、转镜法线转过的角度等
  	  //```````````省略中间代码``````````
      
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
      //```````````省略中间代码``````````
      }
    }
  }
}
```

**输入参数：** 参考前文介绍

**功能详解：**

- 提取时间戳信息
  - `frameSecond`：从 `udpData[13]` 到 `udpData[16]` 提取秒数。具体位数需参考不同型号的雷达协议。
  - `frameMicrosecond`：从 `udpData[17]` 到 `udpData[20]` 提取微秒数，并乘以0.1转换为实际微秒数。
- 处理每个数据块
  - `offset_block`：计算当前数据块的偏移量。
  - `offsetMicrosecond`：从 `udpData[32 + offset_block]` 和 `udpData[33 + offset_block]` 提取当前数据块的微秒偏移量。
  - `totalMicrosecond`：计算当前数据块的总微秒数。
  - `blockSecond` 和 `blockMicrosecond`：根据总微秒数计算当前数据块的秒数和微秒数。
- 提取镜面信息
  - 从 `udpData[35 + offset_block]` 提取镜面信息，并进行位操作转换为镜面编号 `mirror`。
  - 使用 `m_skewing_cos_tsp48` 和 `m_skewing_sin_tsp48` 获取镜面的俯仰角度的余弦和正弦值。
- 处理每个数据块中的16个通道
  - `horAngle`：从 `udpData[36 + offset_block + seq * 10]` 和 `udpData[37 + offset_block + seq * 10]` 提取水平角度，并进行转换和偏移。
  - `L_1` 和 `L_2`：从 `udpData[38 + offset_block + seq * 10]` 到 `udpData[43 + offset_block + seq * 10]` 提取两个回波下的径向距离，并进行转换。
  - `intensity_1` 和 `intensity_2`：从 `udpData[40 + offset_block + seq * 10]` 和 `udpData[44 + offset_block + seq * 10]` 提取两个回波的强度。
  - `confidence1` 和 `confidence2`：从 `udpData[41 + offset_block + seq * 10]` 和 `udpData[45 + offset_block + seq * 10]` 提取两个回波的置信度。
- 计算笛卡尔坐标
  - 计算转镜法线转过的角度 `theta`。
  - 计算各通道竖直角度的正弦和余弦值。
  - 计算机芯发射光线的单位向量。
  - 计算点的笛卡尔坐标 `x`、`y`、`z`。
- 填充点云数据
  - 创建 `TWPoint` 对象 `basic_point`，并填充点的各项属性。
  - 根据 `_echoNum` 的值，决定是否将第一个回波和第二个回波的数据添加到 `points` 容器中。

## 3.4 点云回调

`PointCloudCallback` 函数用于处理解析后的点云数据，并根据特定条件将点云数据组帧，通过 `OnPointCloud()`发布整帧点云数据。

```cpp
bool LidarDevice::PointCloudCallback(TWPointCloud::Points &points,
                                     float *maxAngle, int mirror, int leftRight,
                                     int value) {
  bool onFrame = false;
  if (_isLidarReady) {
    if (_lidarAlgo && !maxAngle)
      _lidarAlgo->Process(points);

    _validPointsTimes++;

    int pointSize = points.size();

    for (int i = 0; i < pointSize; i++) {
      TWPoint &point = points[i];

      // 组帧条件
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
	
      // 判断组帧是否成功
      if (_has_framed_cycle_count) {
        _pointCloudPtr->height = 1;
        _pointCloudPtr->width = _pointCloudPtr->size();

       //```````````省略中间代码``````````

        if (!maxAngle) {
          _lidarObserver->OnPointCloud(_lidarInfo, *_pointCloudPtr);
        }
        _callbackdone.store(true);
        _pointCloudPtr->clear();
        onFrame = true; // 帧检测，是否检测到新的一帧的点云
        _has_framed_cycle_count = false;

        _validPointsTimes = 0;
      }

      if (point.angle < m_startAngle || point.angle > m_endAngle)
        continue;

      // 计算第一个点和最后一个点的时间戳，省略此处代码``````````

      if (point.distance < _dis_min || point.distance >= _dis_max)
        continue;

      CalculateRotateAllPointCloud(point);

      UserPoint basic_point;
      setX(basic_point, static_cast<float>(point.x));
      //```````````省略中间代码``````````

      _pointCloudPtr->push_back(std::move(basic_point));
    }
  }

  return onFrame;
}
```

**输入参数：**

- `TWPointCloud::Points &points`：参考上文解释
- `float *maxAngle`：参考上文解释
- `int mirror`：镜面号，规定当前数据包中的点是镜面 `mirror`扫描得到的才可能分帧。`mirror = 0`为A镜面，`mirror = 1`为B镜面，`mirror = 2`为C镜面。该参数不传值时默认为 `-1`，即分帧时不考虑镜面。
- `int leftRight`：正负机芯，规定当前数据包中的点是机芯 `leftRight`扫描得到的才可能分帧。`leftRight =0`为负机芯， `leftRight =1`为正机芯。该参数不传值时默认为 `-1`，即分帧时不考虑机芯。
- `int value`：特定值，规定当前数据包中的点的 `x` 坐标为0时才可能分帧。该参数不传值时默认为 `-1`，即分帧时不考虑点的 `x`坐标。

**输出参数：**

- `onFrame`：表示是否检测到新的一帧点云数据。如果检测到新的一帧，返回 `true`；否则返回 `false`。

**功能详解：**

- **检查 Lidar 就绪状态**
  - `_isLidarReady`：如果已解析过DIF帧，`_isLidarReady`置为true，可继续处理点云数据，否则认为接收到的UDP数据不准确，不触发回调函数。
- **处理点云数据**
  - 如果 `_lidarAlgo` 存在且 `maxAngle` 为 `nullptr`，调用 `_lidarAlgo->Process(points)` 处理点云数据。
  - 每解析一个UDP包就将 `_validPointsTimes`参数递增1，组帧时需判断 `_validPointsTimes>50`，即一帧中至少有50个UDP包的数据。
- **遍历点云数据**
  - 遍历 `points`中的每个点，判断是否到达分帧位置：组帧条件可以参考 [参数详解第3.3节](参数详解.md)
- **组帧处理**
  - 设置点云数据容器的高度和宽度。
  - 根据 `_stampType` 设置点云数据的时间戳。可参考 [参数详解第3.1节](参数详解.md)
  - 设置点云数据的序列号。
  - 如果 `maxAngle` 为 `nullptr`，调用 `_lidarObserver->OnPointCloud(_lidarInfo, *_pointCloudPtr)` 发布点云数据，用户可在该函数中对点云数据（`UserPointCloud`数据类型）进行操作，如可视化、打印整帧点云数量等。
  - 设置 `_callbackdone` 为 `true`，表示回调已完成。
  - 清空点云数据容器 `_pointCloudPtr`。
  - 设置 `onFrame` 为 `true`，表示检测到新的一帧点云数据。
  - 重置 `_has_framed_cycle_count` 和 `_validPointsTimes`。
- **点云数据处理**
  - 检查点的角度是否在指定FOV范围内，默认为30-150°，具体可参考 [接口说明](接口说明.md) 中 `SetAngleRange`函数。
  - 当点云数据容器为空时，记录当前点（第一个点）的时间戳。
  - 更新最后一个点的时间戳。
  - 检查点的距离是否在指定范围内，具体可参考 [接口说明](接口说明.md) 中 `SetDistanceRange`函数。
  - 调用 `CalculateRotateAllPointCloud` 函数处理点的旋转变换。
  - 创建 `UserPoint` 对象 `basic_point`，并填充点的各项属性，`UserPoint`数据类型的介绍可参考 [参数详解第2节](参数详解.md)。
  - 将 `basic_point` 添加到点云数据容器 `_pointCloudPtr` 中。
