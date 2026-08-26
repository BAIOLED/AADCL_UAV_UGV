# Scout Mini 自主导航机器人开发文档

> 适用平台：AgileX Scout Mini、Jetson、Ubuntu 20.04、ROS Noetic、Livox Mid-360、Intel RealSense D435i。
> 车端工作空间：`~/livox_fastlio`。
> 本文按“从源码修改到实车验证”的顺序记录，每项给出实际文件、写入内容、编译目标和验证命令。

## 1. 最终架构与边界

```text
Scout CAN底盘
  ├─ /scout/odom：轮速反馈，供局部规划器读取速度
  └─ /cmd_vel：底盘速度命令

Mid-360 + IMU → FAST-LIO
  ├─ /Odometry：camera_init → body位姿消息
  ├─ TF camera_init → body
  ├─ /cloud_registered：camera_init中的注册扫描
  └─ /cloud_registered_body：body中的当前扫描

/cloud_registered + /Odometry → scout_pointcloud_mapper
  ├─ 几何/离群点预处理
  ├─ 3D贝叶斯占据与动态物体清除
  ├─ 0.05 m细地图累积
  └─ filtered_camera_init.pcd

filtered_camera_init.pcd → finalize_map.py
  ├─ raw_camera_init.pcd
  ├─ public_map.pcd
  ├─ map_raw.pgm / map_raw.yaml
  ├─ map.pgm / map.yaml
  └─ map_metadata.yaml

public_map.pcd + 实时base点云 + /fastlio_odom → NDT-OMP → TF map → odom
map_raw.yaml + TF + /scout/odom + 实时障碍点云 → move_base + TEB → /cmd_vel
```

边界：

- 不修改FAST-LIO的雷达输入，也不把过滤点云回灌FAST-LIO。
- FAST-LIO仍用原始Livox点云计算里程计和内部局部地图。
- mapper处理FAST-LIO去畸变、配准后的输出，负责最终交付PCD。
- FAST-LIO原生PCD保存必须关闭。
- mapper不发布TF；每条TF只能有一个发布者。
- D435i当前未并入激光PCD建图链。

## 2. 源码目录与职责

```text
~/livox_fastlio/
├─ AGENTS.md
├─ maps/<map_name>/
└─ src/
   ├─ FAST_LIO/config/mid360.yaml
   ├─ FAST_LIO/src/laserMapping.cpp
   ├─ scout_system_bringup/
   │  ├─ config/scout_geometry.yaml
   │  └─ launch/
   │     ├─ scout_mapping.launch
   │     ├─ scout_localization.launch
   │     ├─ scout_relocalization.launch
   │     ├─ fastlio_mapping_scout.launch
   │     ├─ fastlio_local_odom.launch
   │     ├─ scout_livox_base.launch
   │     ├─ D435I.launch
   │     └─ scout_system.launch
   ├─ scout_pointcloud_mapper/
   │  ├─ CMakeLists.txt
   │  ├─ package.xml
   │  ├─ launch/pointcloud_mapper.launch
   │  ├─ config/mapper.yaml
   │  └─ src/pointcloud_mapper_node.cpp
   ├─ scout_map_tools/
   │  ├─ config/scout_raw.yaml
   │  ├─ config/scout_nav.yaml
   │  ├─ src/pcd_transform.cpp
   │  ├─ src/pcd_to_pgm.cpp
   │  └─ scripts/finalize_map.py
   ├─ scout_tf_manager/
   │  ├─ config/extrinsics.yaml
   │  ├─ launch/tf_manager.launch
   │  └─ scripts/{tf_manager.py,geometry_tf_publisher.py}
   ├─ scout_pose_adapter/
   │  ├─ launch/pose_adapter.launch
   │  └─ scripts/tf_to_odom.py
   ├─ scout_cloud_adapter/
   │  ├─ launch/cloud_adapter.launch
   │  └─ src/cloud_frame_adapter.cpp
   ├─ fast_lio_localization/src/{map_loader.cpp,fast_lio_localization.cpp}
   └─ scout_navigation/
      ├─ launch/{navigation.launch,navigation_teb.launch,
      │          global_planning_test.launch,nav_logging.launch}
      ├─ config/{costmap_common.yaml,global_costmap.yaml,local_costmap.yaml,
      │          global_planner.yaml,move_base.yaml,move_base_teb.yaml,
      │          dwa_local_planner.yaml,teb_local_planner.yaml}
      └─ scripts/{global_plan_tester.py,nav_log_session.sh,analyze_nav_bag.py}
```

`scout_system.launch`是旧入口，直接包含上游`mapping_mid360.launch`，没有贝叶斯mapper。正式建图只使用`scout_mapping.launch`。

## 3. 开发前检查

```bash
source /opt/ros/noetic/setup.bash
cd ~/livox_fastlio
test -d src
rosversion -d
uname -m

rospack find fast_lio
rospack find livox_ros_driver2
rospack find scout_base
rospack find scout_pointcloud_mapper
rospack find scout_map_tools
rospack find scout_tf_manager
rospack find scout_navigation
rospack find teb_local_planner
rospack find realsense2_camera
```

需要：ROS消息、TF/TF2、PCL、map_server、move_base、GlobalPlanner、TEB、RealSense驱动、can-utils，以及仓库内Livox/FAST-LIO/NDT/Scout驱动。

## 4. 配置FAST-LIO

### 4.1 修改`FAST_LIO/config/mid360.yaml`

```yaml
common:
  lid_topic: /livox/lidar
  imu_topic: /livox/imu

preprocess:
  lidar_type: 1
  scan_line: 4
  blind: 0.5

publish:
  path_en: false
  scan_publish_en: true
  dense_publish_en: true
  scan_bodyframe_pub_en: true

pcd_save:
  pcd_save_en: false
  interval: -1
```

不要打开`pcd_save_en`，否则得到未经过动态清除的PCD。

### 4.2 修改`scout_system_bringup/launch/fastlio_mapping_scout.launch`

```xml
<rosparam command="load" file="$(find fast_lio)/config/mid360.yaml" />
<param name="feature_extract_enable" type="bool" value="0" />
<param name="point_filter_num" type="int" value="3" />
<param name="max_iteration" type="int" value="3" />
<param name="filter_size_surf" type="double" value="0.5" />
<param name="filter_size_map" type="double" value="0.5" />
<param name="pcd_save/pcd_save_en" type="bool" value="false" />
<param name="publish/scan_bodyframe_pub_en" type="bool" value="true" />
<node pkg="fast_lio" type="fastlio_mapping" name="laserMapping" output="screen" />
```

定位用`fastlio_local_odom.launch`保持同样的PCD关闭和body点云发布设置。

### 4.3 FAST-LIO输出契约

源码：`FAST_LIO/src/laserMapping.cpp`。

| 输出 | 类型 | frame |
|---|---|---|
| `/cloud_registered` | `sensor_msgs/PointCloud2` | `camera_init` |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | `body` |
| `/cloud_effected` | `sensor_msgs/PointCloud2` | `camera_init` |
| `/Laser_map` | `sensor_msgs/PointCloud2` | `camera_init` |
| `/Odometry` | `nav_msgs/Odometry` | parent=`camera_init`，child=`body` |
| `/path` | `nav_msgs/Path` | `camera_init`；当前`path_en=false` |
| TF `camera_init → body` | TF | FAST-LIO唯一发布 |

## 5. 建立唯一TF系统

### 5.1 TF目标

```text
建图：      odom → camera_init → body → base_link
定位导航：  map → odom → camera_init → body → base_link → camera_link...
```

### 5.2 `odom → camera_init`

唯一参数文件：`scout_system_bringup/config/scout_geometry.yaml`

```yaml
odom_to_camera_init:
  x: 0.25
  y: 0.00
  z: 0.20
  roll_deg: 0.0
  pitch_deg: 45.0
  yaw_deg: 0.0
```

发布代码：`scout_tf_manager/scripts/geometry_tf_publisher.py`

```python
t.header.frame_id = "odom"
t.child_frame_id = "camera_init"
broadcaster = tf2_ros.StaticTransformBroadcaster()
broadcaster.sendTransform(t)
```

### 5.3 `body → base_link`

参数：`scout_tf_manager/config/extrinsics.yaml`

```yaml
transforms:
  - name: body_to_base_link
    parent: base_link
    child: body
    x: 0.25
    y: 0.00
    z: 0.20
    roll_deg: 0.0
    pitch_deg: 45.0
    yaw_deg: 0.0
    publish_inverse: true
```

配置记录易测量的`base_link → body`，`tf_manager.py`对完整4×4刚体变换求逆后实际发布`body → base_link`。存在旋转时不能仅把xyz取负。

### 5.4 禁止底盘重复发布TF

所有正式launch必须写：

```xml
<include file="$(find scout_bringup)/launch/scout_mini_robot_base.launch">
  <arg name="odom_topic_name" value="/scout/odom" />
  <arg name="pub_tf" value="false" />
</include>
```

## 6. 新增点云预处理和贝叶斯地图包

包：`scout_pointcloud_mapper`

### 6.1 `package.xml`

```xml
<buildtool_depend>catkin</buildtool_depend>
<depend>boost</depend>
<depend>nav_msgs</depend>
<depend>pcl_conversions</depend>
<depend>pcl_ros</depend>
<depend>roscpp</depend>
<depend>sensor_msgs</depend>
<depend>std_srvs</depend>
```

### 6.2 `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(scout_pointcloud_mapper)
add_compile_options(-std=c++14 -O2)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_srvs)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(Boost REQUIRED COMPONENTS filesystem system)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pointcloud_mapper_node src/pointcloud_mapper_node.cpp)
target_link_libraries(pointcloud_mapper_node
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS pointcloud_mapper_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
```

### 6.3 `launch/pointcloud_mapper.launch`

```xml
<launch>
  <arg name="map_name" default="current_mapping" />
  <arg name="input_cloud" default="/cloud_registered" />
  <arg name="input_odom" default="/Odometry" />
  <arg name="output_path"
       default="$(env HOME)/livox_fastlio/maps/$(arg map_name)/filtered_camera_init.pcd" />
  <node pkg="scout_pointcloud_mapper"
        type="pointcloud_mapper_node"
        name="scout_pointcloud_mapper"
        output="screen" required="true">
    <rosparam command="load"
              file="$(find scout_pointcloud_mapper)/config/mapper.yaml" />
    <param name="input_cloud" value="$(arg input_cloud)" />
    <param name="input_odom" value="$(arg input_odom)" />
    <param name="output_path" value="$(arg output_path)" />
  </node>
</launch>
```

### 6.4 `config/mapper.yaml`

```yaml
input_cloud: /cloud_registered
input_odom: /Odometry
static_scan_topic: /scout/static_scan
static_map_topic: /scout/static_map_cloud
dynamic_topic: /scout/dynamic_points
min_range: 0.50
max_range: 50.0
scan_voxel_size: 0.05

radius_filter:
  enable: true
  radius: 0.15
  min_neighbors: 2

self_filter:
  enable: false
  min_x: -0.55
  max_x: 0.55
  min_y: -0.45
  max_y: 0.45
  min_z: -0.40
  max_z: 0.25

dynamic_filter:
  enable: true
  voxel_size: 0.20
  hit_probability: 0.70
  miss_probability: 0.40
  occupied_probability: 0.72
  clearing_probability: 0.35
  min_hit_scans: 8
  min_observation_span: 2.0
  ray_stride: 4
  max_clearing_range: 20.0
  ray_endpoint_margin: 0.30
  candidate_timeout: 5.0
  cleanup_period: 1.0
  max_voxels: 2000000

map:
  voxel_size: 0.05
  max_voxels: 5000000
  autosave_period: 30.0
  save_on_shutdown: true

map_publish_period: 2.0
publish_dynamic_points: false
max_odom_age: 0.20
output_path: /tmp/filtered_camera_init.pcd
```

粗占据体素0.20 m不等于最终地图分辨率；最终地图为0.05 m。禁止再次用粗体素直接导出地图。

### 6.5 `src/pointcloud_mapper_node.cpp`代码路径

完整源码以该文件为唯一真值，必须包含以下实现。

#### 6.5.1 ROS接口

```cpp
static_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(static_scan_topic_, 1);
static_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(static_map_topic_, 1, true);
odom_sub_ = nh_.subscribe(input_odom_, 20, &PointcloudMapper::odomCallback, this);
cloud_sub_ = nh_.subscribe(input_cloud_, 2, &PointcloudMapper::cloudCallback, this);
save_service_ = pnh_.advertiseService("save_map", &PointcloudMapper::saveMap, this);
reset_service_ = pnh_.advertiseService("reset_map", &PointcloudMapper::resetMap, this);
publish_timer_ = nh_.createTimer(ros::Duration(map_publish_period_),
                                 &PointcloudMapper::publishMapTimer, this);
autosave_timer_ = nh_.createTimer(ros::Duration(autosave_period_),
                                  &PointcloudMapper::autosaveTimer, this);
```

#### 6.5.2 预处理

`prefilter()`依次执行：NaN/Inf删除、按`/Odometry`转换到body相对坐标、距离裁剪、可选自车包围盒、PCL 0.05 m VoxelGrid、PCL RadiusOutlierRemoval。

自车包围盒默认关闭，因为最终机械外轮廓尚未实测。

#### 6.5.3 贝叶斯更新

```cpp
static double probabilityToLogOdds(double p) {
  return std::log(p / (1.0 - p));
}
```

命中，每个粗体素每帧最多一次：

```cpp
state.log_odds = std::min(max_log_odds_, state.log_odds + hit_log_odds_);
++state.hit_scans;
if (state.first_hit.isZero()) state.first_hit = stamp;
state.last_hit = stamp;
```

空闲射线穿过：

```cpp
state.log_odds = std::max(min_log_odds_, state.log_odds + miss_log_odds_);
++state.miss_scans;
if (state.log_odds <= clearing_log_odds_) {
  temporal_voxels_.erase(it);
  ++bayesian_cleared_voxels_;
  dirty_ = true;
}
```

静态输出条件：

```cpp
state.hit_scans >= min_hit_scans_
&& state.log_odds >= occupied_log_odds_
&& (state.last_hit - state.first_hit).toSec() >= min_observation_span_
```

射线每4个过滤点取1条，追踪到端点前0.30 m，最远20 m。当前帧所有端点粗体素进入保护集合，避免其他射线误清真实表面。

#### 6.5.4 generation失效机制

每个粗体素创建时获得递增generation。每个0.05 m细体素记录所属粗体素和generation。粗体素清除后，旧generation细点在构图时被删除，避免人的旧点在同一位置重新出现。

#### 6.5.5 保存

`saveMapToDisk()`将当前有效细点保存为binary PCD。dirty时每30秒保存，正常SIGINT退出时再保存。正常流程不调用服务；以下仅用于诊断：

```bash
rosservice call /scout_pointcloud_mapper/save_map
rosservice call /scout_pointcloud_mapper/reset_map
```

旧`scout_map_tools/scripts/finish_mapping.py`已删除，不应恢复。

### 6.6 动态清除边界

- 快速走动目标通常达不到2秒门槛。
- 长时间站立目标可能暂时进入；离开后必须重新看见其原位置。
- 完全遮挡且不再观察的位置无法判断为空闲。
- 多机轨迹重新可见后可清除；当前视野中的其他机器人仍是占据物。
- 真正多机互相屏蔽还需共享机器人位姿并按包围盒删点，当前未实现。

## 7. 将mapper并入一键建图

文件：`scout_system_bringup/launch/scout_mapping.launch`

```xml
<arg name="map_name" default="current_mapping" />
<include file="$(find livox_ros_driver2)/launch_ROS1/msg_MID360.launch" />
<include file="$(find scout_system_bringup)/launch/fastlio_mapping_scout.launch">
  <arg name="rviz" value="false" />
</include>
<include file="$(find scout_pointcloud_mapper)/launch/pointcloud_mapper.launch">
  <arg name="map_name" value="$(arg map_name)" />
</include>
<include file="$(find scout_tf_manager)/launch/tf_manager.launch" />
<include file="$(find scout_pose_adapter)/launch/pose_adapter.launch" />
<include file="$(find scout_bringup)/launch/scout_mini_robot_base.launch">
  <arg name="odom_topic_name" value="/scout/odom" />
  <arg name="pub_tf" value="false" />
</include>
```

```bash
roslaunch scout_system_bringup scout_mapping.launch map_name:=factory_a
```

节点应包括Livox、`laserMapping`、mapper、两个TF节点、pose adapter和底盘。

## 8. 地图资产生成

### 8.1 文件和编译目标

| 文件 | 目标/用途 |
|---|---|
| `scout_map_tools/src/pcd_transform.cpp` | `pcd_transform_node`，camera_init PCD变换到map |
| `scout_map_tools/src/pcd_to_pgm.cpp` | `pcd_to_pgm_node`，按高度生成PGM/YAML |
| `scout_map_tools/scripts/finalize_map.py` | 串联归档、变换、两套PGM和元数据 |
| `scout_map_tools/config/scout_raw.yaml` | `map_raw.*`，障碍固化膨胀0 |
| `scout_map_tools/config/scout_nav.yaml` | `map.*`，参考图固化膨胀0.15 m |

### 8.2 实际流程

```text
filtered_camera_init.pcd
  ├─复制/覆盖→ raw_camera_init.pcd
  ├─读取scout_geometry.yaml
  ├─pcd_transform_node → public_map.pcd
  ├─scout_raw.yaml → map_raw.pgm + map_raw.yaml
  ├─scout_nav.yaml → map.pgm + map.yaml
  └─map_metadata.yaml
```

```bash
rosrun scout_map_tools finalize_map.py factory_a --replace-raw
ls -lh ~/livox_fastlio/maps/factory_a/
```

| 文件 | frame/用途 |
|---|---|
| `filtered_camera_init.pcd` | mapper实时输出，`camera_init` |
| `raw_camera_init.pcd` | 完成时归档，`camera_init` |
| `public_map.pcd` | NDT地图，`map` |
| `map_raw.pgm/.yaml` | 无固化膨胀，navigation默认读取 |
| `map.pgm/.yaml` | 固化0.15 m，localization的`/map_2d`参考图 |
| `map_metadata.yaml` | 外参与生成参数快照 |

当前navigation读取`map_raw.yaml`，所以参考图0.15 m与costmap 0.45 m在正式导航链中不叠加。本轮文档整改不修改任何navigation参数。

## 9. 重定位开发

### 9.1 点云适配

`scout_cloud_adapter/src/cloud_frame_adapter.cpp`订阅`/cloud_registered_body`，按消息时间查询TF并发布`/cloud_registered_base`，frame设为`base_link`。

`scout_cloud_adapter/launch/cloud_adapter.launch`：

```xml
<param name="input_topic" value="/cloud_registered_body" />
<param name="output_topic" value="/cloud_registered_base" />
<param name="target_frame" value="base_link" />
```

### 9.2 位姿适配

`scout_pose_adapter/scripts/tf_to_odom.py`查询`odom → base_link`，发布`/fastlio_odom`。其twist当前为0，不能给TEB作速度反馈；TEB使用`/scout/odom`。

### 9.3 NDT修改

`fast_lio_localization/src/fast_lio_localization.cpp`必须：

- 订阅`/map_cloud`和`/initialpose`；
- 同步订阅重映射后的`/cloud_registered_base`和`/fastlio_odom`；
- 只发布`map → odom`；
- TF时间戳按`tf_postdate_sec`向未来预发布。

```cpp
tfMsg.header.stamp = ros::Time::now() + ros::Duration(_cfg.tfPostdateSec);
tfMsg.header.frame_id = "map";
tfMsg.child_frame_id = _cfg.odomFrame;
_br.sendTransform(tfMsg);
```

`scout_system_bringup/launch/scout_relocalization.launch`：

```xml
<param name="odom_frame" value="odom" />
<param name="tf_postdate_sec" value="0.50" />
<param name="ndt/num_threads" value="4" />
<param name="ndt/maximum_iterations" value="30" />
<param name="ndt/voxel_leaf_size" value="0.20" />
<param name="ndt/resolution" value="1.0" />
<param name="ndt/transformation_epsilon" value="0.01" />
<param name="ndt/step_size" value="0.10" />
<remap from="/velodyne_points" to="/cloud_registered_base" />
<remap from="/odom_lio" to="/fastlio_odom" />
```

`scout_localization.launch`必须同时包含Mid-360、FAST-LIO local odom、TF manager、pose adapter、cloud adapter、map loader、NDT、map_server和底盘。

```bash
roslaunch scout_system_bringup scout_localization.launch map_name:=factory_a
```

## 10. 导航开发

### 10.1 footprint

`scout_navigation/config/costmap_common.yaml`：

```yaml
footprint:
  - [ 0.370,  0.295]
  - [ 0.370, -0.295]
  - [-0.300, -0.295]
  - [-0.300,  0.295]
footprint_padding: 0.03
transform_tolerance: 0.5
```

增加机械结构后必须重测外轮廓。

### 10.2 costmap

`global_costmap.yaml`：frame=`map`，读取`/nav_static_map`，膨胀0.45 m。
`local_costmap.yaml`：frame=`odom`，6×6 m、0.05 m分辨率，实时源`/cloud_registered_body`，障碍高度0.08～1.50 m，marking 4 m、clearing 5 m、膨胀0.45 m。

0.45 m是从`map_raw`障碍栅格向外的总渐变代价范围，不是给车体尺寸额外相加0.45 m；footprint负责车体碰撞几何。

### 10.3 TEB

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `odom_topic` | `/scout/odom` | 真实速度反馈 |
| `max_vel_x` | 0.35 m/s | 最大前进速度 |
| `max_vel_x_backwards` | 0 | 禁止主动倒车 |
| `max_vel_theta` | 1.0 rad/s | 最大角速度 |
| `min_obstacle_dist` | 0.15 m | footprint边界净空 |
| `inflation_dist` | 0.35 m | TEB软代价范围 |
| `enable_homotopy_class_planning` | false | 当前关闭多拓扑 |

启动顺序：

```bash
# 终端1
roslaunch scout_system_bringup scout_localization.launch map_name:=factory_a
# NDT收敛后，终端2
roslaunch scout_navigation navigation_teb.launch map_name:=factory_a
```

## 11. D435i开发

`scout_system_bringup/launch/D435I.launch`当前启用color、depth、align_depth和内部TF；关闭IMU和PointCloud2。

实际安装TF：

```xml
<node pkg="tf2_ros" type="static_transform_publisher"
      name="base_to_d435i"
      args="0.27 0.00 0.10 0 0 0 base_link camera_link"/>
```

launch注释曾写“倒装roll=pi”，但实际参数为0。以代码为准；若确实倒装，必须实测后同时修改代码和注释。

## 12. 修改文件后如何编译

### 12.1 完整编译

```bash
cd ~/livox_fastlio
source /opt/ros/noetic/setup.bash
catkin_make -j1
source ~/livox_fastlio/devel/setup.bash
```

使用`-j1`避免Jetson内存压力和并行依赖竞态。

### 12.2 编译矩阵

| 修改文件 | 命令/动作 |
|---|---|
| `scout_pointcloud_mapper/src/*.cpp`、CMake、package.xml | `catkin_make -j1 --pkg scout_pointcloud_mapper` |
| `scout_cloud_adapter/src/*.cpp` | `catkin_make -j1 --pkg scout_cloud_adapter` |
| `scout_map_tools/src/*.cpp`、CMake、package.xml | `catkin_make -j1 --pkg scout_map_tools` |
| `fast_lio_localization/src/*.cpp` | `catkin_make -j1 --pkg fast_lio_localization` |
| `FAST_LIO/src/*.cpp` | `catkin_make -j1 --pkg fast_lio` |
| 已安装的Python脚本内容 | 通常无需C++编译，重启节点；安装规则/新文件变化时编译该包 |
| `*.launch`、已有`*.yaml` | 无需编译，停止旧节点、重新source、重启launch |
| `scout_map_tools/config/*.yaml` | 无需编译；重新运行`finalize_map.py`才改变已有地图 |
| `scout_navigation/config/*.yaml` | 无需编译；重启move_base |

单包编译后仍须：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_fastlio/devel/setup.bash
```

## 13. 标准修改、部署和验证流程

### 13.1 备份

```bash
stamp=$(date +%Y%m%d_%H%M%S)
mkdir -p ~/livox_fastlio/deploy_backups/$stamp
cp -a ~/livox_fastlio/src/scout_pointcloud_mapper \
      ~/livox_fastlio/deploy_backups/$stamp/
```

按明确包目录备份，不对工作空间根目录递归删除或覆盖。

### 13.2 权限与文本检查

```bash
chmod +x ~/livox_fastlio/src/scout_tf_manager/scripts/*.py
chmod +x ~/livox_fastlio/src/scout_pose_adapter/scripts/*.py
chmod +x ~/livox_fastlio/src/scout_map_tools/scripts/*.py
chmod +x ~/livox_fastlio/src/scout_navigation/scripts/*.py
rg -n "pcd_save_en|pub_tf|dynamic_filter|tf_postdate_sec" ~/livox_fastlio/src
```

### 13.3 静态launch检查

```bash
source ~/livox_fastlio/devel/setup.bash
roslaunch --nodes scout_system_bringup scout_mapping.launch map_name:=test_map
roslaunch --nodes scout_system_bringup scout_localization.launch map_name:=test_map
roslaunch --nodes scout_navigation navigation_teb.launch map_name:=test_map
```

### 13.4 运行验证

```bash
rosnode list
rostopic hz /livox/lidar
rostopic hz /Odometry
rostopic hz /cloud_registered
rostopic hz /scout/static_scan
rostopic echo -n 1 /scout/static_map_cloud/header
rosrun tf tf_echo odom base_link
```

mapper日志应包含：

```text
Mapper: input=... filtered=... static_scan=... map=... occupancy=... bayes_cleared=...
Saved ... fine static-map points to .../filtered_camera_init.pcd
```

### 13.5 动态目标实测

1. 启动正式建图；
2. 让人员横穿；
3. 人员离开后继续观察原位置；
4. 查看`/scout/static_map_cloud`；
5. 观察`bayes_cleared`是否增加；
6. 检查最终PCD；
7. 遮挡区域换视角后再判断。

### 13.6 地图文件检查

```bash
ls -lh ~/livox_fastlio/maps/factory_a/
head -n 11 ~/livox_fastlio/maps/factory_a/filtered_camera_init.pcd
```

二进制PCD头的`POINTS`必须大于0，不能只看文件大小。

## 14. 完整实车流程

```bash
# CAN
rosrun scout_bringup bringup_can2usb.bash
ip -details link show can0

# 建图
roslaunch scout_system_bringup scout_mapping.launch map_name:=factory_a

# Ctrl+C正常结束后生成地图资产
rosrun scout_map_tools finalize_map.py factory_a --replace-raw

# 定位
roslaunch scout_system_bringup scout_localization.launch map_name:=factory_a

# NDT收敛后导航
roslaunch scout_navigation navigation_teb.launch map_name:=factory_a
```

## 15. 已验证结果

- mapper单包编译通过；
- 45秒静态场景自动保存成功；
- PCD约60,645点、948 KiB；
- mapper约21%单核CPU、68 MiB RSS；
- FAST-LIO、底盘和TF节点同时运行正常。

该测试只证明编译、启动、静态保留和保存；人员动态清除必须按13.5节实测，不能用静态测试代替。

## 16. 禁止事项和已知问题

- 禁止FAST-LIO与mapper同时保存交付PCD。
- 禁止重复发布任一TF边。
- 禁止把`/fastlio_odom`作为TEB速度源，其twist为0。
- 禁止用旧`scout_system.launch`正式建图。
- 禁止混淆`map.yaml`与`map_raw.yaml`。
- FAST-LIO完成IMU初始化前，TF树可能短暂未连接；持续报错才是故障。
- D435i倒装注释与实际零旋转参数不一致，待实测。
- 多机器人主动互相屏蔽尚未实现。
- 当前CAN不能确认SOC百分比。

## 17. 文档与版本管理

每次功能修改同步更新：源码、开发文档、接口排错表、使用手册和`AGENTS.md`。

```bash
git status --short
git diff --check
git diff --stat
```

仓库：`https://github.com/BAIOLED/AADCL_UAV_UGV.git`。禁止提交PCD、rosbag、build、devel、日志或密码。
