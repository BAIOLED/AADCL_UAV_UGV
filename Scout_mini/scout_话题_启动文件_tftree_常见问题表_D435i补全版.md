# Scout Mini 启动文件、节点、话题、TF与常见问题完整表

> 本表按当前源码逐项整理。车端工作空间：`~/livox_fastlio`。
> 正式建图入口：`scout_mapping.launch`；正式定位入口：`scout_localization.launch`；正式导航入口：`navigation_teb.launch`。
> 导航当前工作正常，本表只记录现状，不要求修改navigation参数。

## 1. 工作模式与禁止同时启动项

| 模式 | 必需启动 | 主要输出 | 禁止同时启动 |
|---|---|---|---|
| 底盘/雷达基础检查 | `scout_livox_base.launch` | Livox数据、Scout状态和轮速里程计 | 其他会重复启动雷达或底盘的总launch |
| 建图 | `scout_mapping.launch` | FAST-LIO位姿、贝叶斯静态点云、过滤PCD | `scout_localization.launch`、旧`scout_system.launch` |
| 重定位 | `scout_localization.launch` | `map → odom`、地图点云、`/map_2d` | 建图总launch、其他`map → odom`发布者 |
| 导航 | 先定位，再`navigation_teb.launch` | costmap、路径、`/cmd_vel` | 第二个move_base、第二个底盘驱动 |
| 全局规划安全测试 | 定位后`global_planning_test.launch` | make_plan结果，不向底盘发速度 | 正式navigation launch |
| 相机 | `D435I.launch` | RGB、对齐深度、camera_info、TF | 第二个RealSense驱动实例 |

## 2. Launch文件完整清单

### 2.1 系统级Launch

| 文件 | 参数 | 实际启动内容 | 使用状态 |
|---|---|---|---|
| `scout_system_bringup/launch/scout_mapping.launch` | `map_name`，默认`current_mapping` | Mid-360、FAST-LIO、mapper、TF manager、pose adapter、Scout底盘 | 正式建图唯一入口 |
| `scout_system_bringup/launch/scout_localization.launch` | `map_name`、`map_dir`、`map_pcd`、`map_yaml` | Mid-360、FAST-LIO local odom、TF、pose/cloud adapter、map loader、NDT、map_server、底盘 | 正式重定位入口 |
| `scout_system_bringup/launch/scout_relocalization.launch` | `map_pcd` | `map_loader`和NDT localizer | 被localization包含，不单独启动整机 |
| `scout_system_bringup/launch/fastlio_mapping_scout.launch` | `rviz=false` | 参数加载、`laserMapping`，FAST-LIO PCD保存关闭 | 被mapping包含 |
| `scout_system_bringup/launch/fastlio_local_odom.launch` | `rviz=false` | 与建图相同的FAST-LIO里程计输出，PCD保存关闭 | 被localization包含 |
| `scout_system_bringup/launch/scout_livox_base.launch` | 无 | Mid-360和底盘，不启动FAST-LIO | 硬件基础检查 |
| `scout_system_bringup/launch/D435I.launch` | 由rs_camera提供 | D435i彩色、深度、对齐、内部TF及安装TF | 相机独立启动 |
| `scout_system_bringup/launch/scout_system.launch` | 无 | 上游`mapping_mid360.launch`、TF、底盘 | 旧入口；没有新mapper，正式建图禁用 |

### 2.2 功能包Launch

| 文件 | 参数/默认值 | 节点或行为 |
|---|---|---|
| `scout_pointcloud_mapper/launch/pointcloud_mapper.launch` | `map_name=current_mapping`、`input_cloud=/cloud_registered`、`input_odom=/Odometry`、`output_path=~/livox_fastlio/maps/<name>/filtered_camera_init.pcd` | `scout_pointcloud_mapper`，required=true |
| `scout_tf_manager/launch/tf_manager.launch` | 从两个YAML加载 | `scout_tf_manager`发布`body → base_link`；`scout_geometry_tf_publisher`发布`odom → camera_init` |
| `scout_pose_adapter/launch/pose_adapter.launch` | parent=`odom`、child=`base_link`、topic=`/fastlio_odom`、20 Hz | TF转Odometry |
| `scout_cloud_adapter/launch/cloud_adapter.launch` | input=`/cloud_registered_body`、output=`/cloud_registered_base`、target=`base_link` | 点云frame转换 |
| `scout_navigation/launch/navigation.launch` | `map_name`、`odom_topic=/scout/odom`、`cmd_vel_topic=/cmd_vel`、`obstacle_cloud_topic=/cloud_registered_body` | `map_raw.yaml` + move_base + GlobalPlanner + DWA |
| `scout_navigation/launch/navigation_teb.launch` | 同上 | `map_raw.yaml` + move_base + GlobalPlanner + TEB；当前正式导航入口 |
| `scout_navigation/launch/global_planning_test.launch` | `map_name` | 关闭local obstacle layer，cmd_vel重映射到blocked话题，RViz clicked point请求make_plan |
| `scout_navigation/launch/nav_logging.launch` | `tag=nav_test` | 启动导航rosbag记录脚本 |
| `scout_bringup/launch/scout_mini_robot_base.launch` | `port_name=can0`、`simulated_robot=false`、`odom_topic_name`、`pub_tf=false` | `scout_base_node`；正式系统始终覆盖odom话题为`/scout/odom` |

### 2.3 正式Launch的节点展开

#### 建图

```text
/livox_lidar_publisher2
/laserMapping
/scout_pointcloud_mapper
/scout_tf_manager
/scout_geometry_tf_publisher
/scout_pose_adapter
/scout_base_node
```

检查：

```bash
roslaunch --nodes scout_system_bringup scout_mapping.launch map_name:=check_map
```

#### 重定位

```text
/livox_lidar_publisher2
/laserMapping
/scout_tf_manager
/scout_geometry_tf_publisher
/scout_pose_adapter
/scout_cloud_adapter
/scout_map_loader
/scout_global_localizer
/scout_map_server
/scout_base_node
```

#### 导航TEB

```text
/scout_navigation_map_server
/move_base
```

定位launch必须继续运行，导航launch不重复启动雷达、FAST-LIO、TF、NDT或底盘。

## 3. 节点职责与输入输出

| 节点 | 包 | 订阅/输入 | 发布/输出 | 是否发TF |
|---|---|---|---|---|
| `livox_lidar_publisher2` | `livox_ros_driver2` | Mid-360 UDP | `/livox/lidar`、`/livox/imu` | 否 |
| `laserMapping` | `fast_lio` | Livox点云、IMU | 注册点云、`/Odometry`、FAST-LIO TF | `camera_init → body` |
| `scout_pointcloud_mapper` | `scout_pointcloud_mapper` | `/cloud_registered`、`/Odometry` | 静态扫描、静态地图云、PCD | 否 |
| `scout_tf_manager` | `scout_tf_manager` | `extrinsics.yaml` | `/tf_static` | `body → base_link` |
| `scout_geometry_tf_publisher` | `scout_tf_manager` | `scout_geometry.yaml` | `/tf_static` | `odom → camera_init` |
| `scout_pose_adapter` | `scout_pose_adapter` | TF `odom → base_link` | `/fastlio_odom` | 否 |
| `scout_cloud_adapter` | `scout_cloud_adapter` | body点云和TF | `/cloud_registered_base` | 否 |
| `scout_map_loader` | `fast_lio_localization` | `public_map.pcd` | `/map_cloud` | 否 |
| `scout_global_localizer` | `fast_lio_localization` | 地图、实时base点云、FAST-LIO位姿、初始位姿 | NDT结果 | `map → odom` |
| `scout_map_server` | `map_server` | `map.yaml` | `/map_2d`、地图元数据 | 否 |
| `scout_navigation_map_server` | `map_server` | `map_raw.yaml` | `/nav_static_map`、元数据 | 否 |
| `move_base` | `move_base` | 地图、TF、轮速里程计、实时障碍、目标 | 全局/局部路径、速度 | 否 |
| `scout_base_node` | `scout_base` | CAN、`/cmd_vel`、灯光命令 | `/scout/odom`和状态 | 正式配置为否 |
| `scout_global_plan_tester` | `scout_navigation` | `/clicked_point`、TF、make_plan服务 | `/scout_global_plan_test` | 否 |
| `realsense2_camera` | `realsense2_camera` | D435i USB | RGB、深度、camera_info、相机TF | 相机内部TF |
| `base_to_d435i` | `tf2_ros` | launch参数 | `/tf_static` | `base_link → camera_link` |

## 4. 话题完整表

### 4.1 Livox与FAST-LIO

| 话题 | 类型 | 发布者 | 主要订阅者 | header frame | 阶段/说明 |
|---|---|---|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | Livox驱动 | FAST-LIO | `livox_frame`或驱动配置值 | 原始雷达数据 |
| `/livox/imu` | `sensor_msgs/Imu` | Livox驱动 | FAST-LIO | Livox IMU frame | IMU初始化必需 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | FAST-LIO | mapper、RViz | `camera_init` | 世界系当前注册扫描，不是完整累计图 |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | FAST-LIO | cloud adapter、local costmap | `body` | 车体系当前扫描 |
| `/cloud_registered_base` | `sensor_msgs/PointCloud2` | cloud adapter | NDT localizer | `base_link` | 重定位匹配输入 |
| `/cloud_effected` | `sensor_msgs/PointCloud2` | FAST-LIO | 调试 | `camera_init` | 有效匹配特征 |
| `/Laser_map` | `sensor_msgs/PointCloud2` | FAST-LIO | 调试/RViz | `camera_init` | FAST-LIO内部局部地图输出 |
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO | mapper | frame=`camera_init`，child=`body` | mapper时间同步和传感器位置 |
| `/path` | `nav_msgs/Path` | FAST-LIO | RViz | `camera_init` | 当前`path_en=false`，通常无数据 |

### 4.2 点云预处理与地图累积

| 话题/服务 | 类型 | 发布/服务节点 | 消费者 | frame/行为 |
|---|---|---|---|---|
| `/scout/static_scan` | `sensor_msgs/PointCloud2` | mapper | RViz/诊断 | 与`/cloud_registered`相同，通常`camera_init`；仅当前满足静态条件的点 |
| `/scout/static_map_cloud` | `sensor_msgs/PointCloud2` | mapper | RViz/诊断 | `camera_init`；latched累计有效细地图 |
| `/scout/dynamic_points` | `sensor_msgs/PointCloud2` | mapper | 调试 | 默认不发布，`publish_dynamic_points=false` |
| `/scout_pointcloud_mapper/save_map` | `std_srvs/Trigger` | mapper | 人工诊断 | 立即保存当前有效PCD；正常流程不用 |
| `/scout_pointcloud_mapper/reset_map` | `std_srvs/Empty` | mapper | 人工诊断 | 清空占据与细地图，谨慎调用 |

mapper日志字段：

| 字段 | 含义 |
|---|---|
| `input` | FAST-LIO输入点数 |
| `filtered` | 距离、体素和离群处理后点数 |
| `static_scan` | 当前扫描中已满足静态条件的点数 |
| `map` | 内存中的0.05 m细体素数，包含尚未输出的当前generation候选 |
| `occupancy` | 0.20 m贝叶斯占据状态数量 |
| `bayes_cleared` | 因空闲概率达到阈值而清除的粗体素累计数 |

### 4.3 地图和重定位

| 话题 | 类型 | 发布者 | 订阅者 | frame/说明 |
|---|---|---|---|---|
| `/map_cloud` | `sensor_msgs/PointCloud2` | `scout_map_loader` | NDT、RViz | `map`，latched；来自`public_map.pcd` |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz 2D Pose Estimate | NDT | 应为`map` frame |
| `/fastlio_odom` | `nav_msgs/Odometry` | pose adapter | NDT | frame=`odom`，child=`base_link`；twist全0 |
| `/map_2d` | `nav_msgs/OccupancyGrid` | localization map_server | RViz/其他参考消费者 | `map`；来自`map.yaml` |
| `/map_metadata` | `nav_msgs/MapMetaData` | localization map_server | 地图消费者 | 只remap了`map`时，元数据通常仍为此名；以`rostopic list`实机确认 |
| `/tf` | `tf2_msgs/TFMessage` | FAST-LIO、NDT | 全系统 | 动态TF |
| `/tf_static` | `tf2_msgs/TFMessage` | TF manager、D435i等 | 全系统 | 静态TF |

NDT节点当前没有单独的“定位结果Pose”话题，最终结果通过TF `map → odom`生效。

### 4.4 Scout底盘

| 话题 | 类型 | 发布/订阅 | 说明 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | move_base发布，`scout_base_node`订阅 | `linear.x`和`angular.z`控制底盘 |
| `/scout/odom` | `nav_msgs/Odometry` | `scout_base_node`发布 | frame=`odom`，child=`base_link`；正式系统速度反馈源 |
| `/scout_status` | `scout_msgs/ScoutStatus` | 底盘发布 | 车辆状态、电压等协议字段 |
| `/BMS_status` | `scout_msgs/ScoutBmsStatus` | 底盘发布 | BMS消息；不代表当前CAN一定有可靠SOC |
| `/rs_status` | `scout_msgs/ScoutRsStatus` | 底盘发布 | 遥控/状态相关消息 |
| `/scout_light_control` | `scout_msgs/ScoutLightCmd` | 用户发布，底盘订阅 | 灯光控制 |

正式launch中`pub_tf=false`，所以`/scout/odom`有消息但底盘不广播`odom → base_link`。

### 4.5 Navigation与move_base

| 话题 | 类型 | 发布者/订阅者 | 说明 |
|---|---|---|---|
| `/nav_static_map` | `nav_msgs/OccupancyGrid` | navigation map_server发布，global static layer订阅 | 来自`map_raw.yaml`，固化膨胀为0 |
| `/nav_static_map_metadata` | `nav_msgs/MapMetaData` | navigation map_server发布 | navigation地图元数据 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz发布，move_base订阅 | 2D Nav Goal入口 |
| `/move_base/goal` | `move_base_msgs/MoveBaseActionGoal` | action客户端→move_base | action目标 |
| `/move_base/status` | `actionlib_msgs/GoalStatusArray` | move_base | action状态 |
| `/move_base/result` | `move_base_msgs/MoveBaseActionResult` | move_base | action结果 |
| `/move_base/feedback` | `move_base_msgs/MoveBaseActionFeedback` | move_base | action反馈 |
| `/move_base/GlobalPlanner/plan` | `nav_msgs/Path` | GlobalPlanner | 全局路径 |
| `/move_base/TebLocalPlannerROS/local_plan` | `nav_msgs/Path` | TEB | 局部轨迹 |
| `/move_base/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | global costmap | 全局代价地图 |
| `/move_base/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | local costmap | 局部滚动代价地图 |
| `/clicked_point` | `geometry_msgs/PointStamped` | RViz Publish Point | global plan tester输入 |
| `/scout_global_plan_test` | `nav_msgs/Path` | global plan tester | 安全全局规划测试结果 |
| `/scout_navigation/cmd_vel_blocked` | `geometry_msgs/Twist` | global planning test中的move_base | 安全锁话题，不接底盘 |

move_base常用服务：

| 服务 | 类型 | 用途 |
|---|---|---|
| `/move_base/make_plan` | `nav_msgs/GetPlan` | 给定起终点请求全局路径 |
| `/move_base/clear_costmaps` | `std_srvs/Empty` | 清理可清除costmap层；车辆安全确认后使用 |

### 4.6 D435i

当前launch参数：color=true、depth=true、align_depth=true、accel/gyro=false、pointcloud=false、publish_tf=true。

典型RealSense ROS1话题如下；具体命名受驱动版本影响，最终以`rostopic list | sort`为准。

| 话题 | 类型 | frame/用途 |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 彩色原图，通常`camera_color_optical_frame` |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 彩色内参 |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 原深度图 |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 深度内参 |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 对齐到彩色的深度 |
| `/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | 对齐深度内参，驱动版本可能复用color info |
| `/camera/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | 仅在image_transport插件启用时存在 |

当前不应出现D435i IMU和PointCloud2流；若出现，检查launch参数是否被其他入口覆盖。

## 5. TF树与唯一发布者

### 5.1 建图TF树

```text
odom
└─ camera_init                     static
   └─ body                         dynamic
      └─ base_link                 static
         └─ camera_link            static（仅D435I.launch启动时）
            ├─ camera_color_frame  RealSense static
            │  └─ camera_color_optical_frame
            └─ camera_depth_frame  RealSense static
               └─ camera_depth_optical_frame
```

### 5.2 定位/导航TF树

```text
map
└─ odom                            dynamic
   └─ camera_init                  static
      └─ body                      dynamic
         └─ base_link              static
            └─ camera_link ...
```

### 5.3 TF责任表

| TF边 | 类型 | 唯一发布者 | 参数/源码 |
|---|---|---|---|
| `map → odom` | dynamic | `scout_global_localizer` | `fast_lio_localization.cpp`，`tf_postdate_sec=0.50` |
| `odom → camera_init` | static | `scout_geometry_tf_publisher` | `scout_geometry.yaml` |
| `camera_init → body` | dynamic | FAST-LIO `laserMapping` | FAST-LIO状态估计 |
| `body → base_link` | static | `scout_tf_manager` | `extrinsics.yaml`，完整刚体逆变换 |
| `base_link → camera_link` | static | `base_to_d435i` | `D435I.launch`实际参数`0.27 0 0.10 0 0 0` |
| RealSense内部frame | static/dynamic | `realsense2_camera` | 驱动标定 |

禁止项：

- 底盘`pub_tf=true`；
- 第二个NDT/AMCL发布`map → odom`；
- static_transform_publisher重复`camera_init → body`；
- 在多个launch中手写同一外参；
- 同时启动两个RealSense实例。

### 5.4 TF检查

```bash
rosrun tf tf_echo map odom
rosrun tf tf_echo odom camera_init
rosrun tf tf_echo camera_init body
rosrun tf tf_echo body base_link
rosrun tf tf_echo map base_link
rosrun tf tf_monitor
rosrun rqt_tf_tree rqt_tf_tree
```

FAST-LIO完成IMU初始化前，`camera_init → body`暂时不存在，启动早期一次“two or more unconnected trees”可以是正常等待；初始化后持续出现才是故障。

## 6. 关键参数完整表

### 6.1 mapper：`scout_pointcloud_mapper/config/mapper.yaml`

| 参数 | 当前值 | 作用 | 调大影响 | 调小影响 |
|---|---:|---|---|---|
| `min_range` | 0.50 m | 删除近距离点 | 删除更多近点 | 保留更多车体/近噪声 |
| `max_range` | 50 m | 最大建图距离 | 点数和噪声增加 | 远处结构缺失 |
| `scan_voxel_size` | 0.05 m | 单帧降采样 | 更稀疏、更省CPU | 更密、更耗CPU |
| `radius_filter/radius` | 0.15 m | 离群邻域半径 | 更易保留稀疏群 | 更易删除稀疏点 |
| `radius_filter/min_neighbors` | 2 | 最少邻点 | 更严格 | 更宽松 |
| `self_filter/enable` | false | 自车包围盒 | 开启前必须实测 | 关闭可能保留车体点 |
| `dynamic_filter/voxel_size` | 0.20 m | 贝叶斯粗占据体素 | 动态判断更粗 | 状态数量/CPU增加 |
| `hit_probability` | 0.70 | 命中log-odds | 更快确认，也更难清 | 更慢确认 |
| `miss_probability` | 0.40 | 穿过射线的空闲证据 | 越接近0.5清除越慢 | 越小清除越快 |
| `occupied_probability` | 0.72 | 输出占据阈值 | 更严格 | 更宽松 |
| `clearing_probability` | 0.35 | 删除粗体素阈值 | 更早彻底失效 | 需要更多空闲证据 |
| `min_hit_scans` | 8 | 最少命中扫描 | 动态抑制更强 | 静态确认更快 |
| `min_observation_span` | 2.0 s | 最短稳定时间 | 行人更难进入，建图变慢 | 慢目标更易进入 |
| `ray_stride` | 4 | 每N点取一条清除射线 | CPU下降、清除变慢 | CPU增加、清除更密 |
| `max_clearing_range` | 20 m | 最大清除距离 | CPU和远处清除增加 | 远处旧点不易清除 |
| `ray_endpoint_margin` | 0.30 m | 端点前保护距离 | 表面更安全、清除死区大 | 易误清表面 |
| `candidate_timeout` | 5 s | 非静态候选超时 | 状态保留更久 | 临时候选更快删除 |
| `map/voxel_size` | 0.05 m | 最终PCD细体素 | 地图稀疏 | 地图密、内存高 |
| `map/autosave_period` | 30 s | 自动保存周期 | 数据丢失窗口大、磁盘写少 | 写盘更频繁 |
| `max_odom_age` | 0.20 s | 点云/里程计允许时差 | 容忍延迟但位姿误差增大 | 更易跳过扫描 |

### 6.2 地图生成参数

| 文件/参数 | 当前值 | 输出 |
|---|---:|---|
| `scout_raw.yaml/resolution` | 0.05 m | `map_raw.*` |
| `scout_raw.yaml/floor_min_z..floor_max_z` | -0.30～0.05 m | 自由地面高度 |
| `scout_raw.yaml/obstacle_min_z..max_z` | 0.05～1.20 m | 障碍高度 |
| `scout_raw.yaml/free_dilation_m` | 0.10 m | 地面自由区域扩展 |
| `scout_raw.yaml/obstacle_inflation_m` | 0 | 正式navigation基础地图不固化膨胀 |
| `scout_nav.yaml/obstacle_inflation_m` | 0.15 m | `/map_2d`参考图；正式navigation不读取 |

### 6.3 NDT

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `odom_frame` | `odom` | TF child |
| `tf_postdate_sec` | 0.50 s | 降低future extrapolation |
| `ndt/num_threads` | 4 | NDT-OMP线程 |
| `maximum_iterations` | 30 | 最大迭代 |
| `voxel_leaf_size` | 0.20 m | 实时扫描降采样 |
| `resolution` | 1.0 m | NDT网格 |
| `transformation_epsilon` | 0.01 | 收敛阈值 |
| `step_size` | 0.10 | 优化步长 |
| `thresh_shift` | 0.50 m | 位移阈值 |
| `thresh_rot` | 0.174533 rad | 约10° |
| `min_scan_range/max_scan_range` | 0.50/50 m | 匹配点范围 |

### 6.4 Navigation现状（只记录，不修改）

| 文件/参数 | 当前值 |
|---|---:|
| `costmap_common.yaml/footprint` | 前0.370、后-0.300、左右±0.295 m |
| `footprint_padding` | 0.03 m |
| global/local `inflation_radius` | 0.45 m |
| `cost_scaling_factor` | 4.0 |
| local window | 6×6 m，0.05 m |
| obstacle height | 0.08～1.50 m |
| obstacle/raytrace range | 4.0/5.0 m |
| TEB `max_vel_x` | 0.35 m/s |
| TEB `max_vel_theta` | 1.0 rad/s |
| TEB `min_obstacle_dist` | 0.15 m |
| TEB `inflation_dist` | 0.35 m |
| homotopy | false |

## 7. 地图文件与frame

| 文件 | 生成者 | frame | 消费者 |
|---|---|---|---|
| `filtered_camera_init.pcd` | mapper | `camera_init` | finalize脚本 |
| `raw_camera_init.pcd` | finalize归档 | `camera_init` | 重生成资产 |
| `public_map.pcd` | pcd_transform | `map` | NDT map loader |
| `map_raw.pgm/.yaml` | pcd_to_pgm + raw配置 | `map` | navigation map_server |
| `map.pgm/.yaml` | pcd_to_pgm + reference配置 | `map` | localization `/map_2d` |
| `map_metadata.yaml` | finalize | 不适用 | 参数追溯 |

## 8. 常见问题：症状、检查顺序、修复位置

### 8.1 找不到包或launch

症状：`is neither a launch file in package`。

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_fastlio/devel/setup.bash
rospack profile
rospack find scout_system_bringup
```

若刚执行`catkin_make --pkg`，必须重新source。修复位置不是launch内容，而是环境overlay或编译失败。

### 8.2 can0不存在或底盘无数据

```bash
rosrun scout_bringup bringup_can2usb.bash
ip -details link show can0
candump can0
rostopic hz /scout/odom
rostopic echo -n 1 /scout_status
```

依次检查USB-CAN设备名、bitrate、接口UP状态、急停和协议日志`Detected protocol: AGX_V2`。

### 8.3 Livox无点云/无IMU

```bash
rostopic hz /livox/lidar
rostopic hz /livox/imu
rosnode info /livox_lidar_publisher2
```

检查雷达IP、Jetson网口IP、MID360 JSON、网线和是否重复启动驱动。

### 8.4 FAST-LIO一直不输出Odometry

```bash
rostopic hz /Odometry
rostopic hz /cloud_registered
```

先看是否出现`IMU Initial Done`。检查IMU数据、雷达类型、时间戳、外参和启动日志。初始阶段`No point, skip this scan`偶发一次可继续观察。

### 8.5 mapper一直等待Odometry或跳过扫描

日志：`waiting for FAST-LIO /Odometry`或`odometry differs by ...`。

```bash
rostopic echo -n 1 /Odometry/header
rostopic echo -n 1 /cloud_registered/header
```

检查时间戳和frame。修复位置：FAST-LIO同步配置；不要先盲目扩大`max_odom_age`。

### 8.6 mapper frame mismatch

预期`/Odometry.header.frame_id`和`/cloud_registered.header.frame_id`都为`camera_init`。

```bash
rostopic echo -n 1 /Odometry/header
rostopic echo -n 1 /cloud_registered/header
```

若不同，检查是否订错点云或上游FAST-LIO被修改。

### 8.7 `/scout/static_scan`启动后短时为空

贝叶斯静态条件要求至少8次命中且跨2秒。刚启动短时为空是设计行为。超过5秒仍空时检查输入点数、时间同步、`occupied_probability`和frame。

### 8.8 人或其他机器人留下轨迹

```bash
rostopic hz /scout/static_map_cloud
rosnode info /scout_pointcloud_mapper
```

操作：目标离开后让雷达继续看到原位置，观察日志`bayes_cleared`。完全遮挡位置必须换视角。若重新可见后仍不清，再基于rosbag比较miss证据；不要直接改navigation参数。

### 8.9 地面出现规则孔洞

检查：

```bash
rosparam get /scout_pointcloud_mapper/scan_voxel_size
rosparam get /scout_pointcloud_mapper/map/voxel_size
rosparam get /scout_pointcloud_mapper/dynamic_filter/voxel_size
```

正确值分别0.05、0.05、0.20。粗动态体素绝不能作为最终地图体素。旧PCD必须重新建图，修改参数不会修复既有PCD。

### 8.10 PCD没有自动保存

```bash
rosparam get /scout_pointcloud_mapper/output_path
rosparam get /scout_pointcloud_mapper/map/autosave_period
ls -ld ~/livox_fastlio/maps/<map_name>
```

至少等待30秒并确认日志有静态点。正常Ctrl+C会最终保存；`kill -9`和断电不会执行析构保存。

### 8.11 finalize_map.py失败

```bash
source ~/livox_fastlio/devel/setup.bash
rospack find scout_map_tools
ls -lh ~/livox_fastlio/maps/<map_name>/filtered_camera_init.pcd
rosrun scout_map_tools finalize_map.py <map_name> --replace-raw
```

检查缺失PCD、文件权限、`pcd_transform_node`/`pcd_to_pgm_node`是否已编译、YAML格式和磁盘空间。

### 8.12 public_map方向或高度错误

检查`map_metadata.yaml`保存的geometry快照，并对照：

```bash
rosparam get /scout_geometry/odom_to_camera_init
rosrun tf tf_echo odom camera_init
```

修复唯一位置：`scout_system_bringup/config/scout_geometry.yaml`。修改后重新finalize；不要在命令行另写一套变换。

### 8.13 cloud adapter报TF错误

```bash
rostopic echo -n 1 /cloud_registered_body/header
rosrun tf tf_echo body base_link
rostopic hz /cloud_registered_base
```

启动早期单次错误可能是TF缓存尚未建立；持续错误检查`tf_manager.launch`和`extrinsics.yaml`，不要删除cloud adapter。

### 8.14 pose adapter等待`odom → base_link`

依次检查：

```bash
rosrun tf tf_echo odom camera_init
rosrun tf tf_echo camera_init body
rosrun tf tf_echo body base_link
```

任一边缺失都会导致链断。FAST-LIO未初始化时等待属于正常；初始化后持续等待才修对应发布者。

### 8.15 NDT不收敛

检查顺序：

1. `public_map.pcd`存在且`/map_cloud`有点；
2. `/cloud_registered_base`有点且frame=`base_link`；
3. `/fastlio_odom`时间持续更新；
4. RViz初值位于真实位置附近，朝向合理；
5. 实时点云与地图有足够重叠；
6. 再考虑NDT参数，而不是先乱调分辨率。

### 8.16 `map → odom` future extrapolation

```bash
rosparam get /scout_global_localizer/tf_postdate_sec
rosrun tf tf_echo map odom
```

当前launch为0.50 s。还持续报错时检查系统时间、消息时间戳和是否存在第二个`map → odom`发布者。

### 8.17 TF抖动或树反复跳变

```bash
rosrun tf tf_monitor
rosnode list
rosnode info /scout_global_localizer
rosnode info /scout_base_node
```

确认底盘`pub_tf=false`，只有NDT发`map → odom`，只有FAST-LIO发`camera_init → body`。删除/关闭重复发布者，而不是增加第三条补偿TF。

### 8.18 `/map_2d`和`/nav_static_map`看起来不同

这是当前设计：

- `/map_2d`来自`map.yaml`，参考图固化膨胀0.15 m；
- `/nav_static_map`来自`map_raw.yaml`，固化膨胀0，由现有costmap运行时处理。

导航正常时不要为让两张显示图一样而修改navigation参数。

### 8.19 move_base无全局路径

```bash
rostopic echo -n 1 /nav_static_map/header
rosrun tf tf_echo map base_link
rosservice call /move_base/make_plan "..."
rostopic echo -n 1 /move_base/GlobalPlanner/plan
```

检查目标是否在已知自由区、`allow_unknown=false`、起点/终点是否被footprint或障碍占据。导航当前已验证正常，不因单次目标失败随意改全局参数。

### 8.20 TEB不发速度

```bash
rostopic hz /scout/odom
rostopic hz /cmd_vel
rostopic echo /move_base/status
rostopic echo -n 1 /move_base/local_costmap/costmap
```

确认NDT已收敛、目标有效、局部costmap有数据、底盘急停释放。`/fastlio_odom`不是TEB速度源。

### 8.21 D435i无图像或负载过高

```bash
rosnode list | grep camera
rostopic list | grep '^/camera/'
rostopic hz /camera/color/image_raw
rostopic hz /camera/aligned_depth_to_color/image_raw
```

无图像检查USB3、设备权限和驱动；负载高检查是否误开pointcloud、IMU、高分辨率或第二个相机节点。

## 9. 一组可复制的完整诊断命令

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_fastlio/devel/setup.bash

rosnode list
rostopic list | sort
rosservice list | sort

rostopic hz /livox/lidar
rostopic hz /livox/imu
rostopic hz /Odometry
rostopic hz /cloud_registered
rostopic hz /cloud_registered_body
rostopic hz /scout/static_scan
rostopic hz /scout/static_map_cloud
rostopic hz /scout/odom

rostopic echo -n 1 /cloud_registered/header
rostopic echo -n 1 /cloud_registered_body/header
rostopic echo -n 1 /cloud_registered_base/header
rostopic echo -n 1 /map_cloud/header
rostopic echo -n 1 /map_2d/header
rostopic echo -n 1 /nav_static_map/header

rosrun tf tf_echo map odom
rosrun tf tf_echo odom camera_init
rosrun tf tf_echo camera_init body
rosrun tf tf_echo body base_link
rosrun tf tf_echo map base_link
rosrun tf tf_monitor

rosparam get /scout_pointcloud_mapper
rosparam get /scout_global_localizer
rosparam get /move_base/global_costmap
rosparam get /move_base/local_costmap
```

## 10. 编译和生效规则速查

| 修改对象 | 是否编译 | 何时生效 |
|---|---|---|
| C++源码 | 是，`catkin_make -j1 --pkg <包>` | 重新source并重启节点 |
| Python脚本内容 | 通常否 | 重启节点；新安装规则需编译 |
| launch | 否 | 重启launch |
| mapper YAML | 否 | 重启mapper；旧PCD不追溯修改 |
| map tools YAML | 否 | 重新运行finalize |
| localization参数 | 否 | 重启localization |
| navigation参数 | 否 | 重启move_base；当前导航正常，本轮禁止修改 |

完整编译：

```bash
cd ~/livox_fastlio
source /opt/ros/noetic/setup.bash
catkin_make -j1
source ~/livox_fastlio/devel/setup.bash
```
