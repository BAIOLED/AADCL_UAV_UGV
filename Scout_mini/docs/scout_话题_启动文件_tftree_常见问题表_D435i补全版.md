# Scout Mini ROS 系统信息表

> **2026-08-25 一键建图接口更新：** `scout_mapping.launch` 接收
> `map_name`，并自动启动 `scout_pointcloud_mapper` 与
> `scout_mapping_finisher`。mapper 输入 `/cloud_registered`、`/Odometry`，输出
> `/scout/static_scan`、`/scout/static_map_cloud`，可选输出
> `/scout/dynamic_points`；`/finish_mapping` 是正式完成接口，会依次调用底层
> `/scout_pointcloud_mapper/save_map` 和 `finalize_map.py --replace-raw`。两个
> 新节点都不发布 TF，也不向 FAST-LIO 回灌点云。必须等待 `/finish_mapping`
> 返回 `success: True` 后再停止 launch。FAST-LIO 的 `pcd_save_en` 现为
> `false`；下文相反描述属于旧版。

> 适用范围：当前 `livox_fastlio` ROS1 工作区，Mid-360 + FAST-LIO + Scout Mini + PCD/NDT 重定位 + move_base/TEB，并包含独立 `realsense_ws` 中的 Intel RealSense D435i RGB/Depth/IMU 接入。
>
> 本文依据本次提供的两份 `src` 源码整理。表中以**当前主链路实际引用关系**为准；源码中保留的上游示例、早期调试文件、DWA 回退方案和迁移补丁不作为当前正式运行链路。

---

## 1. 当前正式运行链路

### 1.1 建图

```text
Livox Mid-360
    ↓ /livox/lidar + /livox/imu
FAST-LIO
    ↓ camera_init -> body
TF Manager
    ↓ odom -> camera_init
    ↓ body -> base_link
Pose Adapter
    ↓ /fastlio_odom
Scout Base
    ↓ /scout/odom
```

推荐入口：

```bash
roslaunch scout_system_bringup scout_mapping.launch
```

### 1.2 重定位

```text
public_map.pcd → map_loader → /map_cloud ┐
                                         ├→ fast_lio_localization → map -> odom
/cloud_registered_base ------------------┤
/fastlio_odom ---------------------------┘

odom -> camera_init -> body -> base_link
```

推荐入口：

```bash
roslaunch scout_system_bringup scout_localization.launch map_name:=scout_map_01
```

### 1.3 导航

先启动重定位，再启动 TEB：

```bash
roslaunch scout_navigation navigation_teb.launch map_name:=scout_map_01
```

导航位姿主要依赖 TF：

```text
map -> odom -> camera_init -> body -> base_link
```

速度反馈使用：

```text
/scout/odom
```

控制输出使用：

```text
/cmd_vel
```

---


### 1.4 D435i 视觉/深度链

D435i 通过独立 `realsense_ws` 提供 ROS 驱动，统一入口位于：

```bash
roslaunch scout_system_bringup d435i.launch
```

当前链路：

```text
D435i USB3
    ↓ librealsense 2.50.0 / RSUSB
realsense2_camera
    ├─ /camera/color/image_raw
    ├─ /camera/depth/image_rect_raw
    ├─ /camera/aligned_depth_to_color/image_raw
    ├─ /camera/gyro/sample
    ├─ /camera/accel/sample
    └─ /camera/imu

base_link
    ↓ static TF
camera_link
    ↓ RealSense internal TF
camera_color/depth/imu frames
```

当前固定安装外参：

```text
base_link -> camera_link
x=0.27 m, y=0.00 m, z=0.10 m
yaw=0, pitch=0, roll=π
```

当前用途为目标识别、视觉检测、深度测距和深度避障；默认不启用 D435i PointCloud2。


# 2. 话题信息详细表

## 2.1 核心传感器、FAST-LIO、重定位话题

| 话题 | 消息类型 | 主要发布者 | 主要订阅者 | frame / child_frame | 使用阶段 | 说明 |
|---|---|---|---|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | `livox_lidar_publisher2` | `laserMapping` | `livox_frame` | 建图、重定位 | Mid-360 原始点云。FAST-LIO `mid360.yaml` 中 `common/lid_topic` 指向该话题。 |
| `/livox/imu` | `sensor_msgs/Imu` | `livox_lidar_publisher2` | `laserMapping` | 通常为 `livox_frame` | 建图、重定位 | Mid-360 IMU 数据。 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | `laserMapping` | RViz/调试 | `camera_init` | 建图、重定位 | FAST-LIO 输出的世界坐标点云。当前运行链不依赖它做重定位。 |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | `laserMapping` | `scout_cloud_adapter`；导航 local costmap | `body` | 重定位、导航 | 当前实时障碍点云默认也从这里进入 local costmap。 |
| `/Odometry` | `nav_msgs/Odometry` | `laserMapping` | 当前主链路无直接订阅 | header=`camera_init`；child=`body` | 建图、重定位 | FAST-LIO 原生里程计。它描述的是 `body`，不是车辆 `base_link`，因此当前系统不直接拿它给重定位/导航使用。 |
| `/fastlio_odom` | `nav_msgs/Odometry` | `scout_pose_adapter` | `fast_lio_localization` | header=`odom`；child=`base_link` | 建图、重定位 | 由 TF `odom -> base_link` 转成标准车体 Odometry。**当前代码 twist 全部置 0，仅用于位姿/重定位，不适合给 TEB 做速度反馈。** |
| `/cloud_registered_base` | `sensor_msgs/PointCloud2` | `scout_cloud_adapter` | `fast_lio_localization` | `base_link` | 重定位 | `/cloud_registered_body` 经 `body -> base_link` TF 转换后的车体点云，是当前 NDT 重定位输入。 |
| `/map_cloud` | `sensor_msgs/PointCloud2` | `scout_map_loader` | `scout_global_localizer` | `map` | 重定位 | `public_map.pcd` 加载后的全局 PCD。发布器为 latch，一次发布后后启动的 localizer 仍可收到。 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz / 用户程序 | `scout_global_localizer` | 应使用 `map` | 重定位 | RViz “2D Pose Estimate” 输入。localizer 会以该初值执行 NDT。 |
| `/map_2d` | `nav_msgs/OccupancyGrid` | `scout_map_server` | RViz/其他可视化节点 | `map` | 重定位 | `scout_localization.launch` 加载 `map.yaml` 后将默认 `/map` 重映射为 `/map_2d`。它不是 NDT 使用的 PCD。 |
| `/map_metadata` | `nav_msgs/MapMetaData` | `scout_map_server` | 可视化/工具 | - | 重定位 | 当前 localization launch 只重映射了 `map`，没有重映射 `map_metadata`，因此 metadata 仍是 `/map_metadata`。 |
| `/tf` | `tf2_msgs/TFMessage` | FAST-LIO、NDT localizer 等 | 全系统 | 多 frame | 全阶段 | 动态 TF：主要是 `camera_init -> body`，重定位时另有 `map -> odom`。 |
| `/tf_static` | `tf2_msgs/TFMessage` | `scout_tf_manager`、`scout_geometry_tf_publisher` | 全系统 | 多 frame | 全阶段 | 静态 TF：`odom -> camera_init`、`body -> base_link`。 |

## 2.2 导航与底盘核心话题

| 话题 | 消息类型 | 主要发布者 | 主要订阅者 | 使用阶段 | 说明 |
|---|---|---|---|---|---|
| `/nav_static_map` | `nav_msgs/OccupancyGrid` | `scout_navigation_map_server` | move_base global costmap | 导航 | `navigation_teb.launch` 默认加载 `map_raw.yaml`，避免在静态图中提前做导航膨胀。 |
| `/nav_static_map_metadata` | `nav_msgs/MapMetaData` | `scout_navigation_map_server` | 导航工具 | 导航 | `/nav_static_map` 的地图元信息。 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz | `move_base` | 导航 | RViz “2D Nav Goal” 的标准入口。目标通常应在 `map` 坐标系。 |
| `/move_base/goal` | `move_base_msgs/MoveBaseActionGoal` | action client | `move_base` | 导航 | move_base action goal。 |
| `/move_base/status` | `actionlib_msgs/GoalStatusArray` | `move_base` | RViz/日志分析 | 导航 | 判断 goal 是否 ACTIVE/SUCCEEDED/ABORTED。 |
| `/move_base/GlobalPlanner/plan` | `nav_msgs/Path` | `GlobalPlanner` | RViz/日志 | 导航 | 全局规划路径。 |
| `/move_base/TebLocalPlannerROS/global_plan` | `nav_msgs/Path` | TEB | RViz/日志 | 导航 | 送入 TEB 的局部截取/转换后的全局路径，用于诊断。 |
| `/move_base/TebLocalPlannerROS/local_plan` | `nav_msgs/Path` | TEB | RViz/日志 | 导航 | TEB 当前局部轨迹。 |
| `/move_base/TebLocalPlannerROS/teb_feedback` | `teb_local_planner/FeedbackMsg` | TEB | 日志/调试 | 导航 | 当前配置 `publish_feedback: true`，用于分析 TEB 优化结果。 |
| `/move_base/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | global costmap | RViz/日志 | 导航 | 全局代价地图。 |
| `/move_base/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | local costmap | RViz/日志 | 导航 | 6 m × 6 m rolling local costmap。 |
| `/cmd_vel` | `geometry_msgs/Twist` | `move_base` / TEB | `scout_base_node` | 导航 | 最终底盘速度命令。当前主链路没有额外速度仲裁器，调试时必须确认没有其他节点同时发布。 |
| `/scout/odom` | `nav_msgs/Odometry` | `scout_base_node` | move_base / TEB / 日志 | 导航 | Scout 轮速积分里程计。header=`odom`，child=`base_link`。**当前 `pub_tf=false`，因此它不发布 `odom -> base_link` TF。** |
| `/scout_status` | `scout_msgs/ScoutStatus` | `scout_base_node` | 监控程序 | 底盘 | 线速度、角速度、电压、故障码、电机状态等。 |
| `/BMS_status` | `scout_msgs/ScoutBmsStatus` | `scout_base_node` | 监控程序 | 底盘 | 当前源码中 BMS 详细字段多数未实际填充，不能把字段存在等同于数据有效。 |
| `/rs_status` | `scout_msgs/ScoutRsStatus` | `scout_base_node` | 监控程序 | 底盘 | 遥控器/RC 状态。 |
| `/scout_light_control` | `scout_msgs/ScoutLightCmd` | 用户程序 | `scout_base_node` | 底盘 | 灯光控制。 |


## 2.3 D435i RGB、Depth 与 IMU 话题

| 话题 | 消息类型 | 主要发布者 | 主要订阅者/用途 | frame | 说明 |
|---|---|---|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | `realsense2_camera` | 目标识别、视觉检测、RViz/rqt | `camera_color_optical_frame` | D435i RGB 原始图像。 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | `realsense2_camera` | 相机模型、投影/反投影 | `camera_color_optical_frame` | RGB 内参。 |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | `realsense2_camera` | 原始深度处理 | `camera_depth_optical_frame` | Depth rectified 原始深度图。 |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | `realsense2_camera` | 深度相机模型 | `camera_depth_optical_frame` | Depth 内参。 |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | `realsense2_camera` | **目标检测后查询对应深度的推荐输入** | `camera_color_optical_frame`/对齐输出 frame 以实际消息为准 | 当前 `align_depth=true`。 |
| `/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | `realsense2_camera` | 对齐深度图相机模型 | 以实际消息为准 | 与 aligned depth 配套。 |
| `/camera/gyro/sample` | `sensor_msgs/Imu` | `realsense2_camera` | IMU调试/算法 | gyro frame | D435i 陀螺仪原始流。 |
| `/camera/accel/sample` | `sensor_msgs/Imu` | `realsense2_camera` | IMU调试/算法 | accel frame | D435i 加速度计原始流。 |
| `/camera/imu` | `sensor_msgs/Imu` | `realsense2_camera` | 需要统一 IMU 流的算法 | 以实际消息为准 | `unite_imu_method=linear_interpolation` 生成。 |
| `/tf_static` | `tf2_msgs/TFMessage` | `base_to_d435i` + RealSense | 全系统 | 多 frame | `base_link -> camera_link` 以及 D435i 内部静态 TF。 |

### 2.3.1 D435i 常用检查命令

```bash
lsusb -t
```

```bash
rostopic hz /camera/color/image_raw
```

```bash
rostopic hz /camera/depth/image_rect_raw
```

```bash
rostopic hz /camera/aligned_depth_to_color/image_raw
```

```bash
rostopic hz /camera/gyro/sample
```

```bash
rostopic hz /camera/accel/sample
```

```bash
rostopic hz /camera/imu
```

```bash
rosrun tf tf_echo base_link camera_link
```

```bash
rosrun tf tf_echo base_link camera_color_optical_frame
```


## 2.4 当前存在但不是主链路有效数据的话题

| 话题 | 当前状态 | 原因 |
|---|---|---|
| `/path` | Advertise 存在，但当前默认不发布 | `mid360.yaml` 中 `publish/path_en: false`。 |
| `/cloud_effected` | Advertise 存在，但实际发布代码被注释 | `publish_effect_world()` 调用被注释。 |
| `/Laser_map` | Advertise 存在，但实际发布代码被注释 | `publish_map()` 调用被注释。 |
| `/Odometry` | 有效发布，但当前主链路不直接使用 | 其 frame 是 `camera_init -> body`，当前系统统一通过 TF + `scout_pose_adapter` 得到 `odom -> base_link`。 |

### 2.5 常用话题检查命令

以下命令均为单行，可直接复制：

```bash
rostopic hz /livox/lidar
```

```bash
rostopic hz /livox/imu
```

```bash
rostopic hz /cloud_registered_body
```

```bash
rostopic hz /cloud_registered_base
```

```bash
rostopic echo -n 1 /fastlio_odom
```

```bash
rostopic echo -n 1 /scout/odom
```

```bash
rostopic info /cmd_vel
```

```bash
rostopic list | sort
```

---

# 3. Launch 信息表

## 3.1 当前正式使用的 Launch

| Launch 文件 | 级别 | 作用 | 主要启动/包含内容 | 建议 |
|---|---|---|---|---|
| `scout_system_bringup/launch/scout_mapping.launch` | **主入口** | 一键建图 | Mid-360、FAST-LIO、静态点云过滤/累积、地图完成服务、TF Manager、Pose Adapter、Scout Base | 启动时必须传 `map_name`；结束前调用 `/finish_mapping`。 |
| `scout_system_bringup/launch/scout_localization.launch` | **主入口** | 重定位 | Mid-360、FAST-LIO local odom、TF Manager、Pose Adapter、Cloud Adapter、NDT localizer、2D map_server、Scout Base | 当前定位推荐入口。 |
| `scout_system_bringup/launch/d435i.launch` | **D435i入口** | RGB/Depth/IMU + 相机安装 TF | include `realsense2_camera/rs_camera.launch`；发布 `base_link -> camera_link` | 可单独启动，也可由整车主 launch include；若已 include，不要再重复手工启动。 |
| `scout_navigation/launch/navigation_teb.launch` | **主入口** | 导航 | navigation map_server、move_base、GlobalPlanner、TEB、local/global costmap | 当前导航推荐入口；应在 localization 正常后启动。 |
| `scout_system_bringup/launch/fastlio_mapping_scout.launch` | 被包含 | FAST-LIO 建图包装 | 载入 `mid360.yaml`，保持 `pcd_save_en=false`，发布注册点云和 body 点云 | 不建议单独作为整车入口。 |
| `scout_pointcloud_mapper/launch/pointcloud_mapper.launch` | 被包含 | 过滤地图构建 | 订阅注册点云和 FAST-LIO odom，过滤、累积并提供底层保存服务 | 不回灌 FAST-LIO，不发布 TF。 |
| `scout_mapping_finisher` | 被启动 | 完成地图 | `/finish_mapping` 先保存过滤 PCD，再运行地图归档和栅格生成 | 成功后才能停止建图 launch。 |
| `scout_system_bringup/launch/fastlio_local_odom.launch` | 被包含 | FAST-LIO 定位模式本地里程计 | `pcd_save_en=false`，保留 body 点云 | localization 内部使用。 |
| `scout_system_bringup/launch/scout_relocalization.launch` | 被包含 | PCD/NDT 全局重定位 | `map_loader` + `fast_lio_localization`；输入重映射为 `/cloud_registered_base`、`/fastlio_odom` | localization 内部使用。 |
| `scout_tf_manager/launch/tf_manager.launch` | 被包含 | 静态几何 TF | `body -> base_link`；`odom -> camera_init` | 必须保持唯一 TF 发布者。 |
| `scout_pose_adapter/launch/pose_adapter.launch` | 被包含 | TF 转 Odometry | `odom -> base_link` TF → `/fastlio_odom` | 给重定位提供车体位姿。 |
| `scout_cloud_adapter/launch/cloud_adapter.launch` | 被包含 | 点云坐标转换 | `/cloud_registered_body` → `/cloud_registered_base` | 给 NDT 使用 base_link 点云。 |
| `livox_ros_driver2/launch_ROS1/msg_MID360.launch` | 被包含 | Mid-360 驱动 | `/livox/lidar`、`/livox/imu` | 当前硬件驱动。 |
| `scout_bringup/launch/scout_mini_robot_base.launch` | 被包含 | Scout Mini 底盘入口 | 进一步包含 `scout_base.launch`，设置 mini 模式 | 当前整车启动链使用。 |
| `scout_base/launch/scout_base.launch` | 被包含 | 底盘 ROS 驱动 | CAN `can0`、`/cmd_vel`、`/scout/odom`、状态话题 | 当前通过上层 bringup 调用。 |

## 3.2 调试/回退 Launch

| Launch 文件 | 当前定位 | 是否纳入正式操作流程 | 说明 |
|---|---|---|---|
| `scout_navigation/launch/nav_logging.launch` | 导航诊断 | 可选 | 启动 rosbag 记录和自动分析。不是车辆运行必要组件。 |
| `scout_navigation/launch/global_planning_test.launch` | 全局规划专项测试 | 否 | 禁止实际输出到底盘，`cmd_vel` 被重映射到 blocked 话题。 |
| `scout_navigation/launch/navigation.launch` | DWA 回退方案 | 默认否 | 当前主方案已经使用 `navigation_teb.launch`；可保留作为对照/回滚。 |

## 3.3 建议忽略的历史、示例或已废弃 Launch

以下文件不应再写入当前正式操作步骤；可以保留源码，但不要与主链路混用：

| 文件/目录 | 判断 | 原因 |
|---|---|---|
| `scout_system_bringup/launch/scout_system.launch` | **早期整机入口，建议不再作为正式入口** | 直接调用上游 `mapping_mid360.launch`，缺少当前已加入的 Pose Adapter、Cloud Adapter、重定位和导航组织方式。 |
| `fast_lio_localization/launch/fast_lio_localization.launch` | **上游示例** | 使用示例 PCD `IB-4L.pcd`，还带 `body -> velodyne` 静态 TF，不符合当前 Scout TF 设计。 |
| `fast_lio_localization/launch/map_loader.launch` | 上游示例 | 默认地图同样指向示例 `IB-4L.pcd`；当前由 `scout_relocalization.launch` 管理。 |
| `FAST_LIO/launch/mapping_avia.launch` 等非 Mid-360 launch | 上游示例 | 当前硬件是 Mid-360。 |
| `FAST_LIO/launch/mapping_mid360.launch` | 上游原始入口 | 当前正式建图/定位已分别由 `fastlio_mapping_scout.launch` 和 `fastlio_local_odom.launch` 包装，避免参数混淆。 |
| `livox_ros_driver2/launch_ROS1/msg_AVIA2.launch`、`msg_HAP.launch`、`msg_mixed.launch` 等 | 上游示例 | 当前只使用 Mid-360。 |
| `livox_ros_driver2/launch_ROS1/rviz_*.launch` | 上游可视化示例 | 非整车主链路。 |
| `scout_description/launch/*`、Gazebo/display launch | 模型/仿真 | 当前实车 `scout_mini_robot_base.launch` 中 description include 已被注释。 |
| `scout_base/launch/scout_mini_base.launch`、`scout_mini_omni.launch` | 直接底盘示例/旧入口 | 当前通过 `scout_bringup/scout_mini_robot_base.launch -> scout_base.launch` 启动。 |
| `teb_migration_patch_V4.0/` | **迁移补丁/备份目录** | 无 `package.xml`，不是当前 catkin 包；正式配置已经进入 `scout_navigation`。 |

> 注意：`ugv_sdk`、`scout_msgs` 等虽然不在“主入口 launch 表”中，但它们仍是底盘驱动的编译/运行依赖，**不能因为未直接 launch 就删除**。

---

# 4. TF Tree 信息表

## 4.1 当前 TF 主树

### 建图模式

```text
odom
 └── camera_init
      └── body
           └── base_link
```

### 重定位 / 导航模式

```text
map
 └── odom
      └── camera_init
           └── body
                └── base_link
```

## 4.2 TF 详细信息

| Parent | Child | 类型 | 发布者 | 参数来源 | 使用阶段 | 作用 |
|---|---|---|---|---|---|---|
| `map` | `odom` | 动态 TF | `scout_global_localizer` / `fast_lio_localization` | NDT 在线计算；`tf_postdate_sec` 默认 0.25 s | 重定位、导航 | 全局地图对 FAST-LIO 本地里程计的修正。 |
| `odom` | `camera_init` | 静态 TF | `scout_geometry_tf_publisher` | `scout_system_bringup/config/scout_geometry.yaml` | 建图、重定位、导航 | 将 FAST-LIO 的 `camera_init` 世界系与车辆 `odom` 约定对齐。当前参数：x=0.25 m，y=0，z=0.20 m，pitch=45°。 |
| `camera_init` | `body` | 动态 TF | FAST-LIO `laserMapping` | FAST-LIO 状态估计 | 建图、重定位、导航 | FAST-LIO 实时估计的 IMU/body 位姿。 |
| `body` | `base_link` | 静态 TF | `scout_tf_manager` | `scout_tf_manager/config/extrinsics.yaml` | 建图、重定位、导航 | 把 FAST-LIO body 转换到车辆标准 `base_link`。配置文件记录的是 `base_link -> body` 测量值，程序设置 `publish_inverse=true` 后发布反变换。 |
| `base_link` | `camera_link` | 静态 TF | `base_to_d435i` | `scout_system_bringup/launch/d435i.launch` | D435i运行时 | D435i 安装外参，当前写死 x=0.27、y=0、z=0.10、roll=π。 |

## 4.3 当前 body/base_link 外参说明

`extrinsics.yaml` 中写入的是便于测量的：

```text
base_link -> body
x = 0.25 m
y = 0.00 m
z = 0.20 m
pitch = +45°
```

实际发布的是完整刚体反变换：

```text
body -> base_link
```

由于存在 45° 旋转，反变换的平移**不能简单写成** `(-0.25, 0, -0.20)`；程序使用 4×4 齐次变换矩阵求逆，这是正确做法。

## 4.4 哪些节点绝对不能再发布同一条 TF

| TF | 当前唯一发布者 | 禁止的重复发布来源 |
|---|---|---|
| `map -> odom` | `fast_lio_localization` | 其他 AMCL/SLAM/localization 节点同时发布同一边。 |
| `odom -> camera_init` | `scout_geometry_tf_publisher` | 旧 static_transform_publisher、旧脚本、手工写死 launch。 |
| `camera_init -> body` | FAST-LIO | 其他节点不能伪造同名动态 TF。 |
| `body -> base_link` | `scout_tf_manager` | 旧 static_transform_publisher、URDF 中重复固定关节。 |
| `odom -> base_link` | **不应有直接发布者** | Scout 底盘驱动必须保持 `pub_tf=false`；系统通过 `odom -> camera_init -> body -> base_link` 组合得到。 |

## 4.5 `/scout/odom` 与 TF 的关系

这是当前系统中最容易误解的一点：

- `/scout/odom` 消息的 `header.frame_id = odom`、`child_frame_id = base_link`；
- 但底盘 launch 明确设置 `pub_tf=false`；
- 因此 `/scout/odom` **不会**产生 `odom -> base_link` TF；
- 导航位姿应以 TF 主树为准；
- `/scout/odom` 当前主要用于给 TEB 提供实际线速度和角速度反馈。

如果把 `pub_tf` 改成 `true`，会和 FAST-LIO 组合出来的 `odom -> base_link` 形成两套不一致来源，出现 TF 抖动、跳变或导航异常。

## 4.6 `livox_frame` 为什么不在 TF 主树里

Livox 驱动消息头使用 `livox_frame`，但当前系统没有额外发布 `body <-> livox_frame` TF。FAST-LIO 对激光雷达到 IMU/body 的小外参使用 `FAST_LIO/config/mid360.yaml` 中：

```yaml
mapping:
  extrinsic_T: [-0.011, -0.02329, 0.04412]
  extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]
```

这组参数属于 **LiDAR ↔ IMU 内部外参**，与车辆安装关系 `base_link ↔ body` 不是同一组参数，不要混改。

### TF 检查命令

```bash
rosrun tf tf_echo map base_link
```

建图模式没有 `map -> odom`，此时检查：

```bash
rosrun tf tf_echo odom base_link
```

```bash
rosrun tf tf_echo odom camera_init
```

```bash
rosrun tf tf_echo camera_init body
```

```bash
rosrun tf tf_echo body base_link
```

```bash
rosrun tf view_frames
```

---


## 4.7 D435i TF 分支

当前新增：

```text
base_link
 └── camera_link
      ├── camera_color_frame
      │    └── camera_color_optical_frame
      ├── camera_depth_frame
      │    └── camera_depth_optical_frame
      ├── camera_gyro_frame
      └── camera_accel_frame
```

其中：

```text
base_link -> camera_link
```

由：

```text
scout_system_bringup/launch/d435i.launch
```

中的 `tf2_ros/static_transform_publisher` 发布。

固定值：

```text
x=0.27
y=0.00
z=0.10
yaw=0
pitch=0
roll=3.14159265
```

RealSense 内部：

```text
camera_link -> color/depth/optical/IMU frames
```

由 `realsense2_camera` 自己发布，`d435i.launch` 中必须保持：

```text
publish_tf=true
tf_publish_rate=0
```

不要再增加第二套内部 TF publisher。


# 5. 常见问题表

| 现象 | 主要原因 | 检查方法 | 处理方式 |
|---|---|---|---|
| `/livox/lidar` 或 `/livox/imu` 没数据 | Mid-360 网络、配置、驱动未正常启动 | `rostopic hz /livox/lidar`；`rostopic hz /livox/imu` | 先解决 Livox 驱动，不要继续调 FAST-LIO。 |
| FAST-LIO 没有 `/cloud_registered_body` | 激光/IMU 输入缺失，或 FAST-LIO 未正常初始化 | `rosnode list`；检查 `/livox/lidar`、`/livox/imu` | 检查 `mid360.yaml` 的话题、雷达类型及终端报错。 |
| `odom -> base_link` 查不到 | TF Manager 或 FAST-LIO 缺失；树中任一边断开 | 分别 `tf_echo odom camera_init`、`camera_init body`、`body base_link` | 找到缺失的边，不要额外直接发布 `odom -> base_link` 绕过去。 |
| TF 抖动、RViz 车体跳变 | 同一 TF 有多个 publisher | `rosrun tf view_frames`；检查 `rosnode info` | 保证每条 TF 只有一个发布者；底盘必须 `pub_tf=false`。 |
| `/scout/odom` 在变化，但 TF 位姿与它不完全一样 | 当前设计中轮速里程计和 FAST-LIO 是两套位姿来源 | 比较 `/scout/odom` 和 `tf_echo odom base_link` | 正常情况下导航位姿以 TF 为准，`/scout/odom` 主要提供速度。 |
| `/fastlio_odom` 位置正常但速度始终为 0 | `tf_to_odom.py` 明确把 twist 全部置 0 | `rostopic echo -n 1 /fastlio_odom` | 这是当前设计；不要将其作为 TEB 的 odometry velocity source。 |
| RViz 发初始位姿后提示 `No point cloud` | localizer 尚未收到 `/cloud_registered_base` | `rostopic hz /cloud_registered_base` | 等点云链路正常后再发 2D Pose Estimate。 |
| 尚未发送初始位姿时 `map -> odom` 看起来仍然存在 | localizer 初始化时 `_odomMap` 为单位变换，并会持续发布 | 启动后直接 `tf_echo map odom`，同时确认是否已经发过 `/initialpose` | 不要把“有 `map -> odom` TF”误认为已经完成重定位；正式使用前仍应通过 2D Pose Estimate/NDT 确认全局对齐。 |
| NDT 不是每帧都重新匹配 | 当前代码仅在相对运动超过 0.50 m 或约 10° 时再次触发 NDT；中间持续复用并发布现有 `map -> odom` | 查看 `scout_relocalization.launch` 的 `ndt/thresh_shift`、`ndt/thresh_rot` 和 NDT 日志 | 这是当前降低计算量的设计；只有确有漂移修正需求时再调整阈值。 |
| 发初始位姿后 NDT 不收敛或跳到错误位置 | 初始猜测过远、地图与当前环境不一致、点云过少或参数不合适 | 检查 `/map_cloud`、`/cloud_registered_base`；观察 NDT 日志 | 先给更准确初值；确认使用同一张 PCD；再调 voxel、resolution、扫描范围。 |
| 重定位正常，但 move_base 报 TF future extrapolation | `map -> odom` 时间戳落后控制查询时刻 | 查看 move_base/TF 报错 | 当前 localizer 默认将 TF 向未来预发布 0.25 s；不要随意删除该机制。 |
| 修改雷达安装角度/位置后旧地图无法对齐 | 车辆安装几何发生变化，但旧 PCD/map 仍使用旧变换生成 | 对比 `scout_geometry.yaml`、`extrinsics.yaml` 和 `map_metadata.yaml` | 修改外参后应重新生成 `public_map.pcd` 和 2D 地图；必要时重新建图。 |
| 改了 `scout_geometry.yaml`，但 `body -> base_link` 仍是旧值 | 当前 `body -> base_link` 仍从 `extrinsics.yaml` 读取 | `rosparam get /scout_geometry`；`tf_echo body base_link` | **当前仍需同步检查两处配置。** 后续可把车辆安装几何进一步收敛到单一配置源。 |
| 为什么 `scout_geometry.yaml` 和 `extrinsics.yaml` 都出现 0.25 m / 0.20 m / 45° | 当前设计分别负责 `odom -> camera_init` 和 `base_link -> body` 测量关系 | 查看两个 YAML 和 TF tree | 逻辑上不是同一条 TF，但当前安装方式下数值存在耦合；修改传感器安装时必须整体检查。 |
| 直接把 `body -> base_link` 写成 x=-0.25、z=-0.20、pitch=-45° 后位置不对 | 含旋转的刚体逆变换不能只对每个参数取负 | 查看 `tf_manager.py` 的矩阵求逆 | 使用当前 `publish_inverse=true` 的完整矩阵求逆方式。 |
| 2D 地图有 `map.yaml` 和 `map_raw.yaml`，不知道导航该用哪个 | 两张图用途不同 | 查看 `finalize_map.py` 和 navigation launch | localization 显示 `/map_2d` 用 `map.yaml`；导航静态层用 `map_raw.yaml`，再由 costmap inflation 做运行期膨胀。 |
| `map_raw.yaml` 名字叫 raw，但仍不是完全未经处理的点云投影 | raw profile 仍有 `free_dilation_m: 0.10`，只是 `obstacle_inflation_m: 0.00` | 查看 `scout_raw.yaml` | “raw”在这里主要表示**不预膨胀障碍**，不要理解成原始 PCD。 |
| 导航 local costmap 看不到实时障碍 | `/cloud_registered_body` 无数据、TF 不通或高度阈值不匹配 | `rostopic hz /cloud_registered_body`；查看 local costmap | 检查 `local_costmap.yaml` 的 `min_obstacle_height/max_obstacle_height`、topic 和 TF。 |
| 车有规划但不动 | `/cmd_vel` 未发布、底盘未接管、CAN/控制模式异常 | `rostopic echo /cmd_vel`；`rostopic echo /scout_status` | 先确认 move_base 有速度，再检查 Scout 驱动和遥控/控制模式。 |
| 车运动但 TEB 判断速度异常 | `/scout/odom` 实际速度异常或消息频率低 | `rostopic hz /scout/odom`；查看 twist | 底盘 odom 是当前 TEB 速度反馈，应优先检查底盘驱动。 |
| 起点或目标贴近障碍时无法规划 | footprint、静态障碍或 inflation 后起点/终点处于不可行区域 | RViz 查看 global/local costmap | 将目标点放到可通行区域；这种情况不应通过降低安全距离强行规划。 |
| PCD finalize 后地图与建图坐标方向不一致 | `raw_camera_init.pcd -> public_map.pcd` 的几何转换参数错误 | 查看 `map_metadata.yaml` 中 geometry snapshot | 修正 `scout_geometry.yaml` 后重新执行 finalize；不要手工对 PCD 再做第二次变换。 |
| `finalize_map.py` 不小心覆盖原始 PCD | 使用了 `--replace-raw` | 查看终端 `[KEEP]` / `[OK]` 信息 | 默认不会替换已归档 `raw_camera_init.pcd`；只有明确需要时使用 `--replace-raw`。 |
| 日志占磁盘很快 | `nav_logging.launch` 记录 TF、点云、costmap、TEB 等大量话题 | `du -sh ~/livox_fastlio/logs/navigation/*` | 只在测试时启动；测试结束及时 Ctrl+C，并定期归档/清理旧 bag。 |
| D435i 在 `lsusb` 中存在但 `lsusb -t` 只有 `480M` | USB2 线/接口协商 | `lsusb -t` | 使用原装或明确支持 USB3 5Gbps 的线，连接 Jetson USB3，目标 `5000M`。 |
| `rs-enumerate-devices` 显示 `Intel RealSense D4XX Recovery` | 相机处于固件 Recovery 模式 | `rs-enumerate-devices`；`rs-fw-update -l` | 不启动 ROS，恢复当前工程匹配固件后再测试。 |
| `realsense2_camera` 编译提示缺少 `ddynamic_reconfigure` | ROS依赖缺失 | 查看 CMake 报错 | `sudo apt install ros-noetic-ddynamic-reconfigure`。 |
| `rosdep` 提示未初始化 | 本机首次使用 rosdep | `rosdep install ...` 报错 | `sudo rosdep init` 后普通用户执行 `rosdep update`。 |
| RealSense manager 报 `_ZN2cv3MatC1Ev` undefined symbol | Jetson 上 OpenCV 未显式链接 | 查看 roslaunch 终端 | 在 `realsense2_camera/CMakeLists.txt` 显式 `find_package(OpenCV REQUIRED)` 并链接 `${OpenCV_LIBRARIES}`，再干净编译。 |
| 新终端找不到 `scout_system_bringup`，手工 source 主工作区后恢复 | `realsense_ws` 最后 source 覆盖主 catkin overlay | `rospack find scout_system_bringup`；`echo $CMAKE_PREFIX_PATH` | `.bashrc` 使用 `source ~/realsense_ws/devel/setup.bash --extend`。 |
| RGB/Depth topic 有数据，但 RViz 报 `camera_color_optical_frame does not exist` | RealSense 内部 TF 未发布 | `tf_echo camera_link camera_color_optical_frame` | `d435i.launch` 保持 `publish_tf=true`、`tf_publish_rate=0`；不要重复手工发布内部 TF。 |
| D435i 被两个节点重复打开 | 总 launch 已 include `d435i.launch`，同时又手工启动一次 | `rosnode list | grep camera` | 只保留一个 D435i 启动入口。 |

---

# 6. 当前配置中值得保留的设计约束

1. **同一条 TF 只能有一个 publisher。** 特别是底盘 `pub_tf=false` 必须保持。
2. **重定位使用 `/fastlio_odom` 的 pose，不使用其 twist。**
3. **TEB 速度反馈使用 `/scout/odom`。**
4. **NDT 输入使用 base_link 坐标点云 `/cloud_registered_base`。**
5. **导航静态地图使用 `map_raw.yaml`，避免静态图预膨胀与 costmap inflation 重复。**
6. **LiDAR-IMU 内部外参与车辆 body-base_link 安装外参是两类参数，不能混为一谈。**
7. **外参变化后，旧 `public_map.pcd` 和 2D 地图不应继续直接使用。**
8. **D435i 必须优先保持 USB3 `5000M`，不要把 `480M` 当作正式状态。**
9. **D435i 的 `base_link -> camera_link` 只由 `d435i.launch` 发布；RealSense 内部 TF 只由 `realsense2_camera` 发布。**
10. **当前 D435i 默认 `align_depth=true`、`enable_pointcloud=false`，服务于目标识别/深度避障，而不是定位。**
11. **主工作区与 `realsense_ws` 并存时，`~/.bashrc` 使用 `realsense_ws/devel/setup.bash --extend`。**

---

# 7. 本次源码中未纳入正式文档的内容

本次有意不把以下内容写入正式运行步骤：

- 非 Mid-360 的 FAST-LIO/Livox 示例；
- 上游 `fast_lio_localization` 示例 launch；
- Gazebo、模型 display、omni 底盘示例；
- `teb_migration_patch_V4.0` 迁移补丁目录；
- DWA 主流程（保留作回滚/对照）；
- 只用于专项测试的 `global_planning_test.launch`；
- FAST-LIO 中已 advertise 但调用被注释的 `/cloud_effected`、`/Laser_map`。

这些文件可以暂时留在源码树中，但后续维护文档时应避免与当前主链路并列，否则很容易造成“到底该启动哪个 launch / 到底谁发布 TF”的歧义。

---

# 8. 源码依据索引

主要依据文件：

- `scout_system_bringup/launch/scout_mapping.launch`
- `scout_system_bringup/launch/scout_localization.launch`
- `scout_system_bringup/launch/scout_relocalization.launch`
- `scout_system_bringup/launch/fastlio_mapping_scout.launch`
- `scout_system_bringup/launch/fastlio_local_odom.launch`
- `scout_system_bringup/config/scout_geometry.yaml`
- `scout_tf_manager/launch/tf_manager.launch`
- `scout_tf_manager/config/extrinsics.yaml`
- `scout_tf_manager/scripts/tf_manager.py`
- `scout_tf_manager/scripts/geometry_tf_publisher.py`
- `scout_pose_adapter/scripts/tf_to_odom.py`
- `scout_cloud_adapter/src/cloud_frame_adapter.cpp`
- `FAST_LIO/config/mid360.yaml`
- `FAST_LIO/src/laserMapping.cpp`
- `fast_lio_localization/src/fast_lio_localization.cpp`
- `fast_lio_localization/src/map_loader.cpp`
- `scout_map_tools/scripts/finalize_map.py`
- `scout_map_tools/config/scout_raw.yaml`
- `scout_map_tools/config/scout_nav.yaml`
- `scout_navigation/launch/navigation_teb.launch`
- `scout_navigation/config/local_costmap.yaml`
- `scout_navigation/config/global_costmap.yaml`
- `scout_navigation/config/teb_local_planner.yaml`
- `scout_navigation/scripts/nav_log_session.sh`
- `scout_ros/scout_base/src/scout_messenger.cpp`
- `scout_system_bringup/launch/d435i.launch`
- `~/realsense_ws/src/realsense-ros/realsense2_camera/launch/rs_camera.launch`
- `~/realsense_ws/src/realsense-ros/realsense2_camera/CMakeLists.txt`
