# FW-mid Gazebo URDF 数字孪生（本机适配版）

原厂包路径：`小车底盘资料/.../FW-mid-gazebo-ros1/fw-mid`  
原厂启动（需要 **ROS1 + Gazebo Classic**）：

```bash
roslaunch fw-mid yhs_gazebo.launch
# /cmd_vel 控制；档位在 control.launch 里 gear=6(4T4D)/7(横移)
```

本机是 **ROS2 Jazzy + Gazebo Harmonic**，没有 ROS1，因此用同一套 **Gazebo URDF/网格**，做成：

- CAN → `/joint_states`（实时转向角）
- `robot_state_publisher` + **RViz2**（官方车壳/轮子，姿态正确）
- 可选再开 **gz sim** 生成同模型

## 启动

在有桌面显示的终端（本机显示器 / 远程桌面）执行：

```bash
cd /home/jiayi/Car_Project/fw_mid_gz
./run_gazebo_twin.sh
```

只要 RViz、不要 Gazebo 窗口：

```bash
USE_GZ=0 ./run_gazebo_twin.sh
```

## 说明

- 转向角来自 `can0` 的 `0x18C4DCEF/DDEF`，约 100Hz+
- 驻车时轮角非零是底盘真实反馈
- 旧的 MuJoCo Web（8090）可不再使用
