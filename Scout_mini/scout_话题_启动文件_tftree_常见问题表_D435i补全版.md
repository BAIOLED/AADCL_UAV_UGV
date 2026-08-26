# Scout Mini ROS接口与排错表

> 本文只用于查询节点、话题、TF、launch、关键参数和诊断路径。

## 1. 正式入口

| 模式 | 命令 | 说明 |
|---|---|---|
| 建图 | `roslaunch scout_system_bringup scout_mapping.launch map_name:=NAME` | FAST-LIO + 过滤地图自动累积/保存 |
| 地图转换 | `rosrun scout_map_tools finalize_map.py NAME --replace-raw` | 生成定位PCD和二维地图 |
| 重定位 | `roslaunch scout_system_bringup scout_localization.launch map_name:=NAME` | FAST-LIO local odom + NDT |
| 导航 | `roslaunch scout_navigation navigation_teb.launch map_name:=NAME` | Dijkstra + TEB |
| D435i | `roslaunch scout_system_bringup D435I.launch` | RGB、Depth、相机TF |

禁止同时启动mapping与localization，禁止重复启动雷达、底盘或相机驱动。

## 2. 建图接口

| 节点 | 输入 | 输出 | frame |
|---|---|---|---|
| `livox_lidar_publisher2` | Mid-360 | `/livox/lidar`、`/livox/imu` | `livox_frame` |
| `laserMapping` | 雷达、IMU | `/cloud_registered`、`/cloud_registered_body`、`/Odometry` | `camera_init`/`body` |
| `scout_pointcloud_mapper` | `/cloud_registered`、`/Odometry` | `/scout/static_scan`、`/scout/static_map_cloud` | 注册点云frame |

mapper不发布TF，也不回灌FAST-LIO。FAST-LIO的`pcd_save_en=false`。

| 调试服务 | 用途 |
|---|---|
| `/scout_pointcloud_mapper/save_map` | 手动立即保存，仅用于诊断 |
| `/scout_pointcloud_mapper/reset_map` | 清空内存地图，谨慎使用 |

正常建图无需调用服务。

## 3. 重定位接口

| 节点 | 输入 | 输出 |
|---|---|---|
| `scout_map_loader` | `public_map.pcd` | `/map_cloud` (`map`) |
| `scout_cloud_adapter` | `/cloud_registered_body` (`body`) | `/cloud_registered_base` (`base_link`) |
| `scout_pose_adapter` | `odom → base_link` TF | `/fastlio_odom` |
| `scout_global_localizer` | 地图、base点云、odom、`/initialpose` | `map → odom` |
| `scout_map_server` | `map.yaml` | `/map_2d` |

NDT输入必须是`/cloud_registered_base`，不能直接使用世界坐标下的`/cloud_registered`。

## 4. 导航与底盘

| 话题 | 类型 | 用途 |
|---|---|---|
| `/scout/odom` | `nav_msgs/Odometry` | TEB真实速度反馈 |
| `/cmd_vel` | `geometry_msgs/Twist` | 底盘速度命令 |
| `/scout_status` | `scout_msgs/ScoutStatus` | 底盘状态 |
| `/move_base/GlobalPlanner/plan` | `nav_msgs/Path` | 全局路径 |
| `/move_base/TebLocalPlannerROS/local_plan` | `nav_msgs/Path` | 局部轨迹 |
| `/nav_static_map` | `nav_msgs/OccupancyGrid` | 导航静态地图 |

`/fastlio_odom`的twist为0，仅用于位姿/重定位；TEB速度源使用`/scout/odom`。

## 5. TF树

```text
map
 └─ odom
     └─ camera_init
         └─ body
             └─ base_link
                 └─ camera_link ...
```

| TF | 类型 | 唯一发布者 |
|---|---|---|
| `map → odom` | 动态 | `scout_global_localizer` |
| `odom → camera_init` | 静态 | `scout_geometry_tf_publisher` |
| `camera_init → body` | 动态 | FAST-LIO |
| `body → base_link` | 静态 | `scout_tf_manager` |
| `base_link → camera_link` | 静态 | D435i安装TF |

重定位显式设置`tf_postdate_sec=0.50`。future extrapolation应检查localizer、TF频率和系统时间，不能增加第二个TF发布者。

## 6. 点云参数

文件：`scout_pointcloud_mapper/config/mapper.yaml`

| 参数 | 值 | 含义 |
|---|---:|---|
| `scan_voxel_size` | 0.05 m | 帧内轻量降采样 |
| `radius_filter/radius` | 0.15 m | 离群邻域半径 |
| `radius_filter/min_neighbors` | 2 | 最少邻点 |
| `dynamic_filter/voxel_size` | 0.20 m | 贝叶斯占据判断，不是地图分辨率 |
| `dynamic_filter/hit_probability` | 0.70 | 端点命中的占据更新概率 |
| `dynamic_filter/miss_probability` | 0.40 | 射线穿过的空闲更新概率 |
| `dynamic_filter/occupied_probability` | 0.72 | 允许输出细地图点的占据阈值 |
| `dynamic_filter/clearing_probability` | 0.35 | 清除体素及其旧细点的阈值 |
| `dynamic_filter/min_hit_scans` | 8 | 最少命中扫描数 |
| `dynamic_filter/min_observation_span` | 2.0 s | 最短稳定观测时长 |
| `dynamic_filter/ray_stride` | 4 | 每4个点取1条清除射线，控制CPU |
| `dynamic_filter/max_clearing_range` | 20 m | 最大空闲清除距离 |
| `map/voxel_size` | 0.05 m | 最终地图分辨率 |
| `map/autosave_period` | 30 s | 自动保存周期 |
| `scout_nav.yaml/obstacle_inflation_m` | 0.15 m | 写入导航PGM的基础膨胀 |
| global/local `inflation_radius` | 0.45 m | move_base运行时渐变代价范围 |

禁止把动态判断体素当作最终地图体素。

## 7. 检查命令

### TF

```bash
rosrun tf tf_echo odom camera_init
rosrun tf tf_echo camera_init body
rosrun tf tf_echo body base_link
rosrun tf tf_echo map base_link
rosrun tf tf_monitor
```

启动前几秒FAST-LIO尚未初始化，`camera_init → body`不存在，可能短暂提示两棵TF树；初始化后必须自动连通。

### 点云

```bash
rostopic hz /livox/lidar
rostopic hz /cloud_registered
rostopic hz /cloud_registered_body
rostopic hz /cloud_registered_base
rostopic hz /scout/static_scan
rostopic echo -n 1 /cloud_registered_base/header
rostopic echo -n 1 /map_cloud/header
```

期望：`cloud_registered_base.frame_id=base_link`，`map_cloud.frame_id=map`。

## 8. 故障顺序

### mapper无输出

1. `/cloud_registered`；2. `/Odometry`时间戳；3. `max_odom_age`告警；4. 过滤后点数。

### 地图规则孔洞

1. 确认不是旧版0.20 m单体素地图；2. 确认`map/voxel_size=0.05`；3. 用新版重新建图；4. 不要继续降低密度。

### 人或其他机器人留下轨迹

1. 确认已启用`dynamic_filter`；2. 目标离开后重新观测其原位置；3. 查看`/scout/static_map_cloud`是否清除；4. 若仍残留，优先降低`miss_probability`或增加清除射线密度，不要直接增大最终地图体素。完全遮挡区域只有再次可见后才能清除。

### 重定位点云frame错误

1. body点云frame；2. `body → base_link`存在且唯一；3. adapter输出`base_link`；4. map loader输出`map`；5. 地图外参与当前一致。

### map到odom future extrapolation

1. localizer是否存活；2. `tf_postdate_sec=0.50`；3. 系统时间；4. navigation transform tolerance；5. 禁止重复TF。

### NDT不收敛

依次检查地图路径、初值、base点云数量、外参；最后才调NDT体素和分辨率。

## 9. 编译检查

```bash
cd ~/livox_fastlio
catkin_make -j1
source devel/setup.bash
roslaunch --nodes scout_system_bringup scout_mapping.launch map_name:=check
roslaunch --nodes scout_system_bringup scout_localization.launch map_name:=check
```
