# Scout Mini 自主导航系统开发文档

> 本文只描述架构、源码职责、参数设计和开发验证。日常操作见《使用手册》，话题、TF和故障索引见《接口与排错表》。

## 1. 平台与目标

- NVIDIA Jetson，Ubuntu 20.04，ROS Noetic
- AgileX Scout Mini，CAN `can0`，协议 AGX V2
- Livox Mid-360 + IMU；Intel RealSense D435i
- FAST-LIO建图，NDT-OMP重定位，move_base + Dijkstra + TEB导航

工作空间为 `~/livox_fastlio`，推荐 `catkin_make -j1`。

## 2. 系统架构

### 2.1 建图

```text
/livox/lidar + /livox/imu
        ↓
FAST-LIO：去畸变、里程计、扫描配准
        ↓ /cloud_registered + /Odometry
scout_pointcloud_mapper
        ├─ 距离、非有限值、离群点过滤
        ├─ 时序动态候选判断
        ├─ 0.05 m细地图累积
        ├─ /scout/static_scan
        ├─ /scout/static_map_cloud
        └─ filtered_camera_init.pcd
```

FAST-LIO输入不被修改，处理点云不回灌FAST-LIO。FAST-LIO原生PCD保存保持关闭，交付点云只由mapper构建。启动`scout_mapping.launch`后，过滤、累积、发布、每30秒保存及正常退出保存全部自动运行，不需要额外服务指令。

### 2.2 地图资产

```text
filtered_camera_init.pcd
  → raw_camera_init.pcd
  → public_map.pcd
  → map_raw.pgm/yaml
  → map.pgm/yaml
  → map_metadata.yaml
```

PCD建图自动完成。首次需要定位/导航资产时运行一次`finalize_map.py`；它是地图格式转换，不是点云预处理。

### 2.3 重定位与导航

```text
public_map.pcd + /cloud_registered_base + /fastlio_odom
        ↓ NDT-OMP
map → odom
        ↓
move_base → GlobalPlanner → TEB → /cmd_vel → Scout
```

## 3. ROS包职责

| 包 | 职责 |
|---|---|
| `livox_ros_driver2` | Mid-360点云与IMU驱动 |
| `FAST_LIO` | 激光惯导、扫描配准、局部里程计 |
| `scout_pointcloud_mapper` | 注册点云过滤、静态地图累积、自动保存 |
| `scout_map_tools` | PCD坐标转换和二维地图生成 |
| `scout_tf_manager` | 唯一静态几何TF管理 |
| `scout_pose_adapter` | TF转标准Odometry |
| `scout_cloud_adapter` | body点云转`base_link` |
| `fast_lio_localization` | PCD加载和NDT重定位 |
| `scout_navigation` | move_base、Dijkstra、TEB、costmap |
| `scout_ros`、`ugv_sdk` | Scout底盘与CAN |
| `scout_system_bringup` | 建图、定位入口编排 |

## 4. 点云处理设计

FAST-LIO已有Livox有效点筛选、盲区过滤、抽点、IMU去畸变和匹配体素化。mapper不重复时间同步或运动补偿。

```text
注册点云
→ NaN/Inf清理
→ 相对车体距离范围
→ 可选车体包围盒
→ 0.05 m帧内体素
→ 半径离群点过滤
→ 0.20 m时序持续性判断
→ 0.05 m独立地图累积
```

动态判定体素和地图存储体素必须分离。粗时序体素只回答“该区域是否跨帧稳定存在”，不能直接作为输出地图分辨率，否则地面和墙面会产生规则孔洞。

```yaml
scan_voxel_size: 0.05
radius_filter:
  radius: 0.15
  min_neighbors: 2
dynamic_filter:
  voxel_size: 0.20
  confirm_hits: 3
map:
  voxel_size: 0.05
  autosave_period: 30.0
  save_on_shutdown: true
```

车体过滤边界尚未实测，默认关闭。当前时序方法适合剔除短时横穿目标；长时间静止的人仍可能成为静态结构，后续应通过射线可见性或语义检测增强，不能简单继续加大体素。

## 5. TF唯一性

| TF | 唯一发布者 |
|---|---|
| `map → odom` | `scout_global_localizer` |
| `odom → camera_init` | `scout_geometry_tf_publisher` |
| `camera_init → body` | FAST-LIO |
| `body → base_link` | `scout_tf_manager` |
| `base_link → camera_link` | D435i安装TF发布者 |

mapper、map loader和cloud adapter不得发布TF。Scout轮速里程计保持`pub_tf=false`。重定位显式使用`tf_postdate_sec=0.50`覆盖导航的未来时刻查询。启动早期FAST-LIO尚未初始化时，TF树短暂未连接属于等待状态；持续报错才是故障。

## 6. 关键源码

- 点云：`scout_pointcloud_mapper/src/pointcloud_mapper_node.cpp`、`config/mapper.yaml`
- 入口：`scout_system_bringup/launch/scout_mapping.launch`、`scout_localization.launch`
- 重定位：`scout_system_bringup/launch/scout_relocalization.launch`
- TF：`scout_tf_manager/launch/tf_manager.launch`、`config/extrinsics.yaml`
- 几何：`scout_system_bringup/config/scout_geometry.yaml`
- 点云坐标转换：`scout_cloud_adapter/launch/cloud_adapter.launch`

## 7. 编译与验证

```bash
cd ~/livox_fastlio
catkin_make -j1
source devel/setup.bash
rospack find scout_pointcloud_mapper
roslaunch --nodes scout_system_bringup scout_mapping.launch map_name:=test_map
roslaunch --nodes scout_system_bringup scout_localization.launch map_name:=test_map
```

参数调整必须用同一场景或rosbag比较：输入/过滤/地图点数、地面墙面连续性、细结构保留、动态目标残留、CPU内存、保存耗时和FAST-LIO轨迹。不能只凭RViz效果加大滤波。

## 8. 开发约束

- 保持ROS Noetic和C++14兼容。
- 不修改FAST-LIO输入格式或点时间字段。
- 不增加重复TF发布者。
- 调试点云默认关闭。
- 地图、bag、日志和编译产物不得提交Git。
- 修改外参后重新生成地图。
- `WheelTech`未经明确要求不得修改。
