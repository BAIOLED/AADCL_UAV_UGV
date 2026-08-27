# AADCL UAV / UGV

本仓库用于统一维护 AADCL 的无人机与无人车项目。当前包含两套并列、独立部署的 UGV 自主导航工程：Scout Mini 与 WheelTech 四轮差速机器人。

## 项目简介

| 项目 | 平台与传感器 | 主要功能 | 项目入口 |
|---|---|---|---|
| Scout Mini | AgileX Scout Mini、Jetson、Livox Mid-360、RealSense D435i | 底盘 CAN 控制、FAST-LIO 建图、NDT 重定位、ROS Navigation/TEB 导航及 RGB-D 接入 | [项目 README](Scout_mini/README.md) · [项目文档](Scout_mini/docs/) |
| WheelTech | 轮趣四轮差速底盘、Jetson NX、Livox Mid-360 | 串口底盘控制、FAST-LIO2 建图、NDT 重定位、ROS Navigation/TEB 导航 | [项目 README](WheelTech/README.md) · [项目文档](WheelTech/docs/) |

两套工程采用相近的软件数据流：

```text
底盘里程计 + Livox 点云/IMU
          ↓
       FAST-LIO
          ↓
静态地图生成 → NDT 重定位 → move_base + TEB → 底盘速度控制
```

## 目录

```text
AADCL_UAV_UGV/
├── Scout_mini/   # Scout Mini 源码、配置与文档
└── WheelTech/    # 轮趣四轮差速机器人源码、配置与文档
```

两个项目的底盘驱动、通信接口、车体尺寸、TF 外参和导航参数不同，不得直接混用。安装、编译、启动、测试和故障排查方法以各项目目录中的 README 与 `docs/` 为准。

仓库不提交私钥、密码、Token、地图、PCD、rosbag、运行日志以及 `build/`、`devel/` 等生成文件。
