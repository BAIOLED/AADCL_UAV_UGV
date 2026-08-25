# Scout Mini + Mid-360 + FAST-LIO + NDT + Dijkstra + TEB + D435i 从零复现开发文档（完整施工版）

> **2026-08-25 一键建图正式流程：** 当前源码已关闭 FAST-LIO 原生 PCD 保存。
> 启动 `roslaunch scout_system_bringup scout_mapping.launch map_name:=地图名` 后，
> launch 会同时启动 FAST-LIO、`scout_pointcloud_mapper` 和
> `scout_mapping_finisher`。完成环境扫描后执行
> `rosservice call /finish_mapping`；该服务会先保存过滤后的
> `filtered_camera_init.pcd`，再自动运行 `finalize_map.py --replace-raw`，生成
> `raw_camera_init.pcd`、`public_map.pcd`、`map_raw.yaml/pgm`、
> `map.yaml/pgm` 和 `map_metadata.yaml`。必须看到 `success: True` 后才能
> `Ctrl+C`。所有文件位于 `~/livox_fastlio/maps/<地图名>/`。本文后续出现的
> `FAST_LIO/PCD/scans.pcd`、`pcd_save_en=true` 和退出时保存均为旧流程。


> 目标：不是解释系统架构，而是按开发顺序把当前工程重新做一遍。每一步明确：下载什么、建立什么文件、修改什么文件、文件内容、编译命令、验证方法和易错点。
> 
> 基准环境：Ubuntu 20.04、ROS Noetic、ROS1 主 catkin 工作区 `~/livox_fastlio`；D435i 使用独立工作区 `~/realsense_ws`。当前正式导航只使用 `GlobalPlanner(Dijkstra) + TEB`，DWA 不纳入正式开发流程。
>
> 本版已重新按源码逐项反查：补齐导航规划日志采集/自动分析、`scout_navigation` 的构建文件、调试脚本、工作区 `maps/` 与 `logs/` 目录，并修正 Mid-360 雷达默认 IP 的确定方法。

## 0. 最终结果先看清楚

最终正式运行链：

```text
Mid-360
  ├─ /livox/lidar + /livox/imu
  ↓
FAST-LIO
  ├─ camera_init -> body
  ├─ /cloud_registered_body
  ↓
Scout TF
  ├─ odom -> camera_init   (静态安装几何)
  └─ body -> base_link     (静态安装几何的逆变换)
  ↓
scout_pose_adapter
  └─ /fastlio_odom         (odom -> base_link 位姿消息，仅给重定位使用)
  ↓
scout_cloud_adapter
  └─ /cloud_registered_base
  ↓
NDT relocalization
  └─ map -> odom
  ↓
move_base
  ├─ GlobalPlanner + Dijkstra
  ├─ global costmap: map
  ├─ local costmap: odom
  └─ TEB -> /cmd_vel
  ↓
Scout Mini chassis
```

最终 TF 主链：

```text
map -> odom -> camera_init -> body -> base_link
```

其中 `map -> odom` 只在定位/导航模式由 NDT 节点维护；建图模式没有全局 `map` 修正。

## 1. 建立工作区与安装基础依赖

### 1.1 建立工作区

终端命令不要用反斜杠续行，下面每条命令都单独复制执行：

```bash
mkdir -p ~/livox_fastlio/src
cd ~/livox_fastlio
catkin_make
source devel/setup.bash
```

建议加入 `~/.bashrc`：

```bash
echo 'source ~/livox_fastlio/devel/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### 1.2 安装本工程明确使用的系统/ROS依赖

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3-pip python3-yaml libasio-dev can-utils libpcl-dev libeigen3-dev libomp-dev
sudo apt install -y ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-tf ros-noetic-tf2-ros ros-noetic-tf2-sensor-msgs ros-noetic-tf-conversions ros-noetic-message-filters ros-noetic-map-server ros-noetic-navigation ros-noetic-teb-local-planner
```

**注意：** `fast_lio_localization/package.xml` 当前上游文件没有完整声明所有依赖，因此不能完全依赖 `rosdep install` 自动补齐；本文显式安装其使用的 PCL、OpenMP、message_filters、tf_conversions 等。

## 2. 下载并安装 Livox-SDK2

当前源码基准：`Livox-SDK2` commit `08f523c930b2f0ba1e98a6afaa8d7476bf479908`。SDK 安装到 `/usr/local` 后，源码目录不再是运行必需。

```bash
cd ~/livox_fastlio/src
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
git checkout 08f523c930b2f0ba1e98a6afaa8d7476bf479908
mkdir -p build
cd build
cmake ..
make -j4
sudo make install
sudo ldconfig
```

验证：

```bash
ls /usr/local/lib/liblivox_lidar_sdk*
```

能看到 Livox SDK 的 `.so`/`.a` 后，SDK 安装完成。之后如果需要精简源码，`~/livox_fastlio/src/Livox-SDK2` 可以删除，但这是精简动作，不影响本文后续开发逻辑。

## 3. 下载 Livox ROS Driver 2，并配置 Mid-360 网络

当前源码基准：`livox_ros_driver2` commit `4a1def929e5b59c7a8122d19fce6efba581ce9f7`。

```bash
cd ~/livox_fastlio/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2
git checkout 4a1def929e5b59c7a8122d19fce6efba581ce9f7
```

### 3.1 确定并修改 Mid-360 IP（必须按实物序列号计算）

**这里不能把 `192.168.1.120` 当成所有 Mid-360 的固定 IP。** Mid-360 出厂默认静态地址规则为：

```text
192.168.1.1XX
```

其中 `XX` 是**雷达机身二维码下方序列号（S/N）的最后两位**。也可以把最后一个八位组理解成：

```text
100 + XX
```

例如：

```text
二维码下方序列号末两位 = 20
雷达默认 IP = 192.168.1.120

二维码下方序列号末两位 = 05
雷达默认 IP = 192.168.1.105
```

当前这套源码配置的是：

```text
192.168.1.120
```

因此复现当前这台车时使用 `.120`；**更换另一台 Mid-360 后，必须重新读取那台雷达二维码下方 S/N 的末两位，不能继续机械复制 `.120`。**

官方 Mid-360 Quick Start Guide 给出的默认规则即为 `192.168.1.1XX`，默认子网掩码为 `255.255.255.0`，默认网关为 `192.168.1.1`。官方资料入口：`https://www.livoxtech.com/cn/mid-360/downloads`。

修改文件：

```text
~/livox_fastlio/src/livox_ros_driver2/config/MID360_config.json
```

当前工程关键值如下：

```json
"host_net_info": {
  "cmd_data_ip": "192.168.1.5",
  "push_msg_ip": "192.168.1.5",
  "point_data_ip": "192.168.1.5",
  "imu_data_ip": "192.168.1.5"
},
"lidar_configs": [
  {
    "ip": "192.168.1.120"
  }
]
```

这里需要区分两个地址：

```text
192.168.1.5    = 工控机/Ubuntu 有线网卡 IP
192.168.1.120  = 当前这台 Mid-360 的 IP
```

两者必须在同一个 `/24` 网段，但不能相同。Livox 官方 `livox_ros_driver2` 的 `MID360_config.json` 示例同样把主机地址写成 `192.168.1.5`，雷达地址则由实际设备决定。

查看网卡名：

```bash
ip addr
```

临时配置示例（把 `<有线网卡名>` 换成实际名称）：

```bash
sudo ip addr flush dev <有线网卡名>
sudo ip addr add 192.168.1.5/24 dev <有线网卡名>
sudo ip link set <有线网卡名> up
```

先验证主机地址：

```bash
ip addr show <有线网卡名>
```

再根据实际雷达 S/N 计算出的地址测试，例如当前设备：

```bash
ping 192.168.1.120
```

能 ping 通后再启动 ROS Driver。若 ping 不通，优先检查：雷达供电、网线、有线网卡 IP、子网掩码、S/N 末两位是否读取正确；不要先修改 FAST-LIO。

永久配置建议用 Ubuntu NetworkManager 把该有线连接设为静态 `192.168.1.5/24`。不要照抄网卡名。

### 3.2 第一次只编译 Livox Driver

在工程很早期可以运行：

```bash
cd ~/livox_fastlio/src/livox_ros_driver2
source /opt/ros/noetic/setup.bash
./build.sh ROS1
```

**非常重要：** 这个 `build.sh` 会删除 `~/livox_fastlio/build`、`devel`、`install`。因此只在工作区还没有放入其他包时使用。工程完成后统一在工作区根目录执行 `catkin_make`，不要再运行该脚本。

### 3.3 驱动验证

```bash
source ~/livox_fastlio/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

另开终端：

```bash
rostopic hz /livox/lidar
rostopic hz /livox/imu
rostopic echo -n 1 /livox/lidar
```

两路数据都稳定后再进入 FAST-LIO；不要在雷达驱动都不稳定时调建图。

## 4. 下载 FAST-LIO，并把旧 Livox Driver API 改成 Driver2

当前 FAST-LIO 基准 commit：`7cc4175de6f8ba2edf34bab02a42195b141027e9`。

```bash
cd ~/livox_fastlio/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git checkout 7cc4175de6f8ba2edf34bab02a42195b141027e9
git submodule update --init --recursive
```

### 4.1 修改 `FAST_LIO/CMakeLists.txt`

把：

```cmake
livox_ros_driver
```

改成：

```cmake
livox_ros_driver2
```

位置在 `find_package(catkin REQUIRED COMPONENTS ...)` 中。

### 4.2 修改 `FAST_LIO/package.xml`

把 build/run dependency 中两处：

```xml
<build_depend>livox_ros_driver</build_depend>
<run_depend>livox_ros_driver</run_depend>
```

改为：

```xml
<build_depend>livox_ros_driver2</build_depend>
<run_depend>livox_ros_driver2</run_depend>
```

### 4.3 修改 FAST-LIO 源码的 CustomMsg 类型

在 `src/laserMapping.cpp`：

```cpp
#include <livox_ros_driver2/CustomMsg.h>
```

并把回调参数改为：

```cpp
void livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg)
```

在 `src/preprocess.h`：

```cpp
#include <livox_ros_driver2/CustomMsg.h>
void process(const livox_ros_driver2::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
void avia_handler(const livox_ros_driver2::CustomMsg::ConstPtr &msg);
```

在 `src/preprocess.cpp` 把两个旧命名空间参数类型同步替换为 `livox_ros_driver2::CustomMsg::ConstPtr`。

这一步只换消息包命名空间，不改 FAST-LIO 算法。

### 4.4 配置 Mid-360 参数


**文件：`~/livox_fastlio/src/FAST_LIO/config/mid360.yaml`**

```yaml
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/livox/imu"
    time_sync_en: false         # ONLY turn on when external time synchronization is really not possible
    time_offset_lidar_to_imu: 0.0 # Time offset between lidar and IMU calibrated by other algorithms, e.g. LI-Init (can be found in README).
                                  # This param will take effect no matter what time_sync_en is. So if the time offset is not known exactly, please set as 0.0

preprocess:
    lidar_type: 1                # 1 for Livox serials LiDAR, 2 for Velodyne LiDAR, 3 for ouster LiDAR, 
    scan_line: 4
    blind: 0.5

mapping:
    acc_cov: 0.1
    gyr_cov: 0.1
    b_acc_cov: 0.0001
    b_gyr_cov: 0.0001
    fov_degree:    360
    det_range:     100.0
    extrinsic_est_en:  false      # true: enable the online estimation of IMU-LiDAR extrinsic
    extrinsic_T: [ -0.011, -0.02329, 0.04412 ]
    extrinsic_R: [ 1, 0, 0,
                   0, 1, 0,
                   0, 0, 1]

publish:
    path_en:  false
    scan_publish_en:  true       # false: close all the point cloud output
    dense_publish_en: true       # false: low down the points number in a global-frame point clouds scan.
    scan_bodyframe_pub_en: true  # true: output the point cloud scans in IMU-body-frame

pcd_save:
    pcd_save_en: true
    interval: -1                 # how many LiDAR frames saved in each pcd file; 
                                 # -1 : all frames will be saved in ONE pcd file, may lead to memory crash when having too much frames.
```

这里的 `extrinsic_T/R` 是**雷达与其 IMU 内部外参**，不是雷达整机相对 Scout `base_link` 的安装外参。后面 45° 安装角不要写进这里。当前 `extrinsic_est_en=false`。

### 4.5 第一次完整编译

```bash
cd ~/livox_fastlio
rm -rf build devel
catkin_make -j1
source devel/setup.bash
```

第一次强烈建议 `-j1`。这个工程曾经出现过并行编译依赖竞态；先用单线程把依赖和源码问题分开，成功后日常增量编译再提高并行度。

## 5. 下载 Scout Mini 底盘 ROS 驱动和 ugv_sdk

当前源码基准：

- `scout_ros`: `01e07881cdc566c3a657e288c59a75577992d13e`
- `ugv_sdk`: `58436e9c1732474566e249ce7f726e12e26304d6`

```bash
cd ~/livox_fastlio/src
git clone https://github.com/agilexrobotics/scout_ros.git
cd scout_ros
git checkout 01e07881cdc566c3a657e288c59a75577992d13e
cd ~/livox_fastlio/src
git clone --recursive https://github.com/westonrobot/ugv_sdk.git
cd ugv_sdk
git checkout 58436e9c1732474566e249ce7f726e12e26304d6
git submodule update --init --recursive
```

编译：

```bash
cd ~/livox_fastlio
catkin_make -j1
source devel/setup.bash
```

### 5.1 CAN 初始化

第一次使用 USB-CAN：

```bash
sudo modprobe gs_usb
rosrun scout_bringup setup_can2usb.bash
```

每次重新上电后：

```bash
rosrun scout_bringup bringup_can2usb.bash
```

验证 CAN：

```bash
candump can0
```

### 5.2 底盘测试

```bash
roslaunch scout_bringup scout_mini_robot_base.launch odom_topic_name:=/scout/odom pub_tf:=false
```

验证：

```bash
rostopic hz /scout/odom
rostopic echo -n 1 /scout/odom
```

**关键设计：** 当前系统把 Scout 轮速里程计保留为 `/scout/odom`，但 `pub_tf=false`。也就是说它提供速度/里程计消息，**不允许它再发布 odom->base_link TF**，否则会和 FAST-LIO 构造出来的 TF 链冲突。

## 6. 新建 `scout_tf_manager`：建立统一 TF 链

### 6.1 创建包和目录

```bash
cd ~/livox_fastlio/src
catkin_create_pkg scout_tf_manager rospy geometry_msgs tf tf2_ros
mkdir -p scout_tf_manager/config scout_tf_manager/launch scout_tf_manager/scripts
```

建立以下文件，完整内容见本节。


**文件：`~/livox_fastlio/src/scout_tf_manager/config/extrinsics.yaml`**

```yaml
# ============================================================
# Scout Mini TF configuration
# ============================================================
#
# 坐标约定：
#
#   +X : 前
#   +Y : 左
#   +Z : 上
#
# 距离单位：m
# 角度单位：degree
#
# 本文件当前只负责：
#
#   body -> base_link
#
# odom -> camera_init 已迁移到：
#
#   scout_system_bringup/config/scout_geometry.yaml
#
# 并由 geometry_tf_publisher.py 单独发布。
#
# ============================================================

transforms:

  # ==========================================================
  # body -> base_link
  #
  # 这里记录容易测量的物理关系：
  #
  #   base_link -> body
  #
  # Mid-360 / FAST-LIO body：
  #
  #   位于 base_link 前方 0.25 m
  #   位于 base_link 上方 0.20 m
  #   Pitch = +45 deg
  #
  # FAST-LIO 已发布：
  #
  #   camera_init -> body
  #
  # 因此本节点不能发布：
  #
  #   base_link -> body
  #
  # publish_inverse=true 后，程序对完整刚体变换求逆，
  # 最终实际发布：
  #
  #   body -> base_link
  # ==========================================================

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


**文件：`~/livox_fastlio/src/scout_tf_manager/scripts/tf_manager.py`**

```python
#!/usr/bin/env python3

import math
import rospy
import tf2_ros
import tf.transformations as tft

from geometry_msgs.msg import TransformStamped


def create_matrix(x, y, z, roll, pitch, yaw):
    """
    xyz + RPY -> 4x4 齐次变换矩阵
    """

    translation = tft.translation_matrix(
        [x, y, z]
    )

    quaternion = tft.quaternion_from_euler(
        roll,
        pitch,
        yaw
    )

    rotation = tft.quaternion_matrix(
        quaternion
    )

    return tft.concatenate_matrices(
        translation,
        rotation
    )


def matrix_to_tf(matrix, parent, child):
    """
    4x4 齐次变换矩阵 -> TransformStamped
    """

    translation = tft.translation_from_matrix(
        matrix
    )

    quaternion = tft.quaternion_from_matrix(
        matrix
    )

    msg = TransformStamped()

    msg.header.stamp = rospy.Time.now()

    msg.header.frame_id = parent
    msg.child_frame_id = child

    msg.transform.translation.x = float(
        translation[0]
    )

    msg.transform.translation.y = float(
        translation[1]
    )

    msg.transform.translation.z = float(
        translation[2]
    )

    msg.transform.rotation.x = float(
        quaternion[0]
    )

    msg.transform.rotation.y = float(
        quaternion[1]
    )

    msg.transform.rotation.z = float(
        quaternion[2]
    )

    msg.transform.rotation.w = float(
        quaternion[3]
    )

    return msg


def main():

    rospy.init_node(
        "scout_tf_manager"
    )

    transforms = rospy.get_param(
        "~transforms",
        []
    )

    if not transforms:
        rospy.logfatal(
            "No static transforms configured."
        )
        return

    broadcaster = (
        tf2_ros.StaticTransformBroadcaster()
    )

    messages = []

    for item in transforms:

        name = item.get(
            "name",
            "unnamed"
        )

        parent = item["parent"]
        child = item["child"]

        x = float(
            item.get("x", 0.0)
        )

        y = float(
            item.get("y", 0.0)
        )

        z = float(
            item.get("z", 0.0)
        )

        roll = math.radians(
            float(
                item.get(
                    "roll_deg",
                    0.0
                )
            )
        )

        pitch = math.radians(
            float(
                item.get(
                    "pitch_deg",
                    0.0
                )
            )
        )

        yaw = math.radians(
            float(
                item.get(
                    "yaw_deg",
                    0.0
                )
            )
        )

        matrix = create_matrix(
            x,
            y,
            z,
            roll,
            pitch,
            yaw
        )

        if item.get(
            "publish_inverse",
            False
        ):

            # 对完整刚体变换求逆。
            matrix = tft.inverse_matrix(
                matrix
            )

            tf_parent = child
            tf_child = parent

        else:

            tf_parent = parent
            tf_child = child

        messages.append(
            matrix_to_tf(
                matrix,
                tf_parent,
                tf_child
            )
        )

        rospy.loginfo(
            "[%s] static TF: %s -> %s",
            name,
            tf_parent,
            tf_child
        )

    broadcaster.sendTransform(
        messages
    )

    rospy.loginfo(
        "Published %d static transform(s).",
        len(messages)
    )

    rospy.spin()


if __name__ == "__main__":
    main()
```


**文件：`~/livox_fastlio/src/scout_tf_manager/scripts/geometry_tf_publisher.py`**

```python
#!/usr/bin/env python3
import math
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler


def deg2rad(v):
    return v * math.pi / 180.0


def main():
    rospy.init_node("scout_geometry_tf_publisher")

    base = "/scout_geometry/odom_to_camera_init"

    x = rospy.get_param(base + "/x")
    y = rospy.get_param(base + "/y")
    z = rospy.get_param(base + "/z")

    roll = deg2rad(rospy.get_param(base + "/roll_deg"))
    pitch = deg2rad(rospy.get_param(base + "/pitch_deg"))
    yaw = deg2rad(rospy.get_param(base + "/yaw_deg"))

    q = quaternion_from_euler(roll, pitch, yaw)

    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "camera_init"

    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z

    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(t)

    rospy.loginfo(
        "odom -> camera_init: xyz=(%.4f, %.4f, %.4f), "
        "rpy_deg=(%.3f, %.3f, %.3f)",
        x, y, z,
        rospy.get_param(base + "/roll_deg"),
        rospy.get_param(base + "/pitch_deg"),
        rospy.get_param(base + "/yaw_deg"),
    )

    rospy.spin()


if __name__ == "__main__":
    main()
```


**文件：`~/livox_fastlio/src/scout_tf_manager/launch/tf_manager.launch`**

```xml
<launch>

  <!--
    Scout Mini TF Manager

    TF 职责：

      odom -> camera_init
          geometry_tf_publisher.py
          参数来自 scout_geometry.yaml

      camera_init -> body
          FAST-LIO 动态发布

      body -> base_link
          tf_manager.py
          参数来自 extrinsics.yaml
  -->

  <!-- body -> base_link -->
  <node
      pkg="scout_tf_manager"
      type="tf_manager.py"
      name="scout_tf_manager"
      output="screen">

    <rosparam
        command="load"
        file="$(find scout_tf_manager)/config/extrinsics.yaml"/>

  </node>

  <!-- odom -> camera_init 参数 -->
  <rosparam
      command="load"
      file="$(find scout_system_bringup)/config/scout_geometry.yaml"
      ns="scout_geometry" />

  <!-- odom -> camera_init -->
  <node
      pkg="scout_tf_manager"
      type="geometry_tf_publisher.py"
      name="scout_geometry_tf_publisher"
      output="screen" />

</launch>
```

脚本权限：

```bash
chmod +x ~/livox_fastlio/src/scout_tf_manager/scripts/tf_manager.py
chmod +x ~/livox_fastlio/src/scout_tf_manager/scripts/geometry_tf_publisher.py
```

### 6.2 建立安装几何唯一配置入口

几何配置放在系统 bringup 包中。先创建包：

```bash
cd ~/livox_fastlio/src
catkin_create_pkg scout_system_bringup
mkdir -p scout_system_bringup/config scout_system_bringup/launch
```


**文件：`~/livox_fastlio/src/scout_system_bringup/config/scout_geometry.yaml`**

```yaml
# Scout Mini 机器人/地图坐标配置
# 本文件是 odom -> camera_init 的唯一真值源。
# 不要在其他 launch、脚本或 pcl_transform 命令中再次手写这些数值。

odom_to_camera_init:
  x: 0.25
  y: 0.00
  z: 0.20

  roll_deg: 0.0
  pitch_deg: 45.0
  yaw_deg: 0.0
```

### 6.3 为什么这里两个变换都出现 0.25/0.20/45°

物理上容易测量的是 `base_link -> body`：雷达/IMU body 在车体中心前方约 0.25 m、上方约 0.20 m、Pitch +45°。FAST-LIO 自己动态发布 `camera_init -> body`。为了让系统启动时的 `odom` 与车体坐标对齐，工程做了：

```text
odom -> camera_init = T(base_link -> body)
body -> base_link    = inverse(T(base_link -> body))
```

这样 FAST-LIO 初始 `camera_init -> body ≈ I` 时，两段安装外参互相抵消，得到 `odom -> base_link ≈ I`。

**特别注意：** 当前实现仍然在 `scout_geometry.yaml` 与 `extrinsics.yaml` 两处各保存一份同样的物理安装数值；修改雷达安装位置/角度时必须同步检查两处。不要只改一处。

编译并验证 TF：

```bash
cd ~/livox_fastlio
catkin_make -j1
source devel/setup.bash
roslaunch scout_tf_manager tf_manager.launch
```

单独启动 TF manager 时只能看到静态段；与 FAST-LIO 同时启动后检查：

```bash
rosrun tf tf_echo odom camera_init
rosrun tf tf_echo camera_init body
rosrun tf tf_echo body base_link
rosrun tf tf_echo odom base_link
```

## 7. 新建 `scout_pose_adapter`：把 TF 变成 `/fastlio_odom`

NDT 原项目要求一条 Odometry 输入；当前真正可信的局部位姿来自 TF 链 `odom -> base_link`，因此做一个适配节点。

```bash
cd ~/livox_fastlio/src
catkin_create_pkg scout_pose_adapter rospy tf2_ros nav_msgs geometry_msgs
mkdir -p scout_pose_adapter/scripts scout_pose_adapter/launch
```


**文件：`~/livox_fastlio/src/scout_pose_adapter/scripts/tf_to_odom.py`**

```python
#!/usr/bin/env python3

import rospy
import tf2_ros

from nav_msgs.msg import Odometry


def main():
    rospy.init_node("scout_pose_adapter")

    # --------------------------------------------------------
    # 参数
    # --------------------------------------------------------

    parent_frame = rospy.get_param(
        "~parent_frame",
        "odom"
    )

    child_frame = rospy.get_param(
        "~child_frame",
        "base_link"
    )

    output_topic = rospy.get_param(
        "~output_topic",
        "/fastlio_odom"
    )

    publish_rate = rospy.get_param(
        "~publish_rate",
        20.0
    )

    # --------------------------------------------------------
    # TF Listener
    # --------------------------------------------------------

    tf_buffer = tf2_ros.Buffer(
        cache_time=rospy.Duration(10.0)
    )

    tf_listener = tf2_ros.TransformListener(
        tf_buffer
    )

    # --------------------------------------------------------
    # Odometry Publisher
    # --------------------------------------------------------

    odom_pub = rospy.Publisher(
        output_topic,
        Odometry,
        queue_size=20
    )

    rate = rospy.Rate(
        publish_rate
    )

    rospy.loginfo(
        "Scout Pose Adapter started."
    )

    rospy.loginfo(
        "TF: %s -> %s",
        parent_frame,
        child_frame
    )

    rospy.loginfo(
        "Output: %s",
        output_topic
    )

    # 给 TF Listener 一点时间建立缓存
    rospy.sleep(1.0)

    # --------------------------------------------------------
    # 主循环
    # --------------------------------------------------------

    while not rospy.is_shutdown():

        try:

            # 获取最新的：
            #
            # odom -> base_link
            #
            # 这里返回的是：
            # base_link 在 odom 坐标系中的位姿。
            transform = tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                rospy.Time(0),
                rospy.Duration(0.1)
            )

            odom = Odometry()

            # 使用 TF 自己的时间戳
            if transform.header.stamp != rospy.Time(0):
                odom.header.stamp = transform.header.stamp
            else:
                odom.header.stamp = rospy.Time.now()

            odom.header.frame_id = parent_frame
            odom.child_frame_id = child_frame

            # ------------------------------------------------
            # Position
            # ------------------------------------------------

            odom.pose.pose.position.x = (
                transform.transform.translation.x
            )

            odom.pose.pose.position.y = (
                transform.transform.translation.y
            )

            odom.pose.pose.position.z = (
                transform.transform.translation.z
            )

            # ------------------------------------------------
            # Orientation
            # ------------------------------------------------

            odom.pose.pose.orientation.x = (
                transform.transform.rotation.x
            )

            odom.pose.pose.orientation.y = (
                transform.transform.rotation.y
            )

            odom.pose.pose.orientation.z = (
                transform.transform.rotation.z
            )

            odom.pose.pose.orientation.w = (
                transform.transform.rotation.w
            )

            # ------------------------------------------------
            # 当前节点只用于：
            #
            #   FAST-LIO 位姿转换
            #   PlotJuggler 数据比较
            #
            # 暂时不计算 twist。
            # ------------------------------------------------

            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.linear.z = 0.0

            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = 0.0

            # 发布
            odom_pub.publish(
                odom
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ) as error:

            rospy.logwarn_throttle(
                2.0,
                "Waiting for TF %s -> %s: %s",
                parent_frame,
                child_frame,
                str(error)
            )

        rate.sleep()


if __name__ == "__main__":
    main()
```


**文件：`~/livox_fastlio/src/scout_pose_adapter/launch/pose_adapter.launch`**

```xml
<launch>

  <!-- ====================================================== -->
  <!-- FAST-LIO 标准化车体里程计                              -->
  <!--                                                       -->
  <!-- 输入：TF                                               -->
  <!--       odom -> base_link                                -->
  <!--                                                       -->
  <!-- 输出：                                                 -->
  <!--       /fastlio_odom                                    -->
  <!--                                                       -->
  <!-- message：nav_msgs/Odometry                             -->
  <!-- ====================================================== -->

  <node
      pkg="scout_pose_adapter"
      type="tf_to_odom.py"
      name="scout_pose_adapter"
      output="screen">

    <param
        name="parent_frame"
        value="odom" />

    <param
        name="child_frame"
        value="base_link" />

    <param
        name="output_topic"
        value="/fastlio_odom" />

    <param
        name="publish_rate"
        value="20.0" />

  </node>

</launch>
```


**文件：`~/livox_fastlio/src/scout_pose_adapter/CMakeLists.txt`**

```text
cmake_minimum_required(VERSION 3.0.2)

project(scout_pose_adapter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2_ros
  nav_msgs
  geometry_msgs
)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/tf_to_odom.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

```bash
chmod +x ~/livox_fastlio/src/scout_pose_adapter/scripts/tf_to_odom.py
```

**关键限制：** 这个节点只复制 pose，`twist` 全部填 0。因此 `/fastlio_odom` 只给 NDT 做位姿同步，不给 TEB 做速度反馈；TEB 使用 `/scout/odom`。

## 8. 新建 `scout_cloud_adapter`：把 body 点云转到 base_link

NDT 初始位姿由 `base_link` 表示，因此用于匹配的 scan 也统一转到 `base_link`，避免拿 body 点云直接套 base_link 初值。

```bash
cd ~/livox_fastlio/src
catkin_create_pkg scout_cloud_adapter roscpp sensor_msgs geometry_msgs tf2_ros tf2_sensor_msgs
mkdir -p scout_cloud_adapter/src scout_cloud_adapter/launch
```


**文件：`~/livox_fastlio/src/scout_cloud_adapter/src/cloud_frame_adapter.cpp`**

```cpp
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>

class CloudFrameAdapter
{
public:
    CloudFrameAdapter()
        : tf_listener_(tf_buffer_)
    {
        ros::NodeHandle pnh("~");

        pnh.param<std::string>(
            "input_topic",
            input_topic_,
            "/cloud_registered_body");

        pnh.param<std::string>(
            "output_topic",
            output_topic_,
            "/cloud_registered_base");

        pnh.param<std::string>(
            "target_frame",
            target_frame_,
            "base_link");

        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            output_topic_,
            2);

        sub_ = nh_.subscribe(
            input_topic_,
            2,
            &CloudFrameAdapter::cloudCallback,
            this);

        ROS_INFO(
            "Cloud adapter: %s -> %s, frame=%s",
            input_topic_.c_str(),
            output_topic_.c_str(),
            target_frame_.c_str());
    }

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        try
        {
            geometry_msgs::TransformStamped transform =
                tf_buffer_.lookupTransform(
                    target_frame_,
                    msg->header.frame_id,
                    msg->header.stamp,
                    ros::Duration(0.1));

            sensor_msgs::PointCloud2 output;

            tf2::doTransform(
                *msg,
                output,
                transform);

            output.header.stamp = msg->header.stamp;
            output.header.frame_id = target_frame_;

            pub_.publish(output);
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_WARN_THROTTLE(
                2.0,
                "Cloud TF failed: %s",
                ex.what());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string input_topic_;
    std::string output_topic_;
    std::string target_frame_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scout_cloud_adapter");

    CloudFrameAdapter node;

    ros::spin();

    return 0;
}
```


**文件：`~/livox_fastlio/src/scout_cloud_adapter/launch/cloud_adapter.launch`**

```xml
<launch>

  <node
      pkg="scout_cloud_adapter"
      type="cloud_frame_adapter_node"
      name="scout_cloud_adapter"
      output="screen">

    <param name="input_topic" value="/cloud_registered_body" />
    <param name="output_topic" value="/cloud_registered_base" />
    <param name="target_frame" value="base_link" />

  </node>

</launch>
```


**文件：`~/livox_fastlio/src/scout_cloud_adapter/CMakeLists.txt`**

```text
cmake_minimum_required(VERSION 3.0.2)
project(scout_cloud_adapter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  cloud_frame_adapter_node
  src/cloud_frame_adapter.cpp
)

target_link_libraries(
  cloud_frame_adapter_node
  ${catkin_LIBRARIES}
)
```

输出关系：

```text
/cloud_registered_body (frame=body)
        ↓ TF body -> base_link
/cloud_registered_base (frame=base_link)
```

## 9. 建立建图/局部里程计统一启动文件

在 `scout_system_bringup/launch` 建立以下文件。


**文件：`~/livox_fastlio/src/scout_system_bringup/launch/fastlio_mapping_scout.launch`**

```xml
<launch>

  <arg name="rviz" default="false" />
  <arg name="pcd_interval" default="-1" />

  <rosparam
      command="load"
      file="$(find fast_lio)/config/mid360.yaml" />

  <param name="feature_extract_enable" type="bool" value="0" />
  <param name="point_filter_num" type="int" value="3" />
  <param name="max_iteration" type="int" value="3" />
  <param name="filter_size_surf" type="double" value="0.5" />
  <param name="filter_size_map" type="double" value="0.5" />
  <param name="cube_side_length" type="double" value="1000" />
  <param name="runtime_pos_log_enable" type="bool" value="0" />

  <param
      name="pcd_save/pcd_save_en"
      type="bool"
      value="true" />

  <param
      name="pcd_save/interval"
      type="int"
      value="$(arg pcd_interval)" />

  <param
      name="publish/scan_bodyframe_pub_en"
      type="bool"
      value="true" />

  <node
      pkg="fast_lio"
      type="fastlio_mapping"
      name="laserMapping"
      output="screen" />

  <group if="$(arg rviz)">
    <node
        launch-prefix="nice"
        pkg="rviz"
        type="rviz"
        name="fastlio_rviz"
        args="-d $(find fast_lio)/rviz_cfg/loam_livox.rviz" />
  </group>

</launch>
```


**文件：`~/livox_fastlio/src/scout_system_bringup/launch/fastlio_local_odom.launch`**

```xml
<launch>

  <arg name="rviz" default="false" />

  <rosparam
      command="load"
      file="$(find fast_lio)/config/mid360.yaml" />

  <param name="feature_extract_enable" type="bool" value="0" />
  <param name="point_filter_num" type="int" value="3" />
  <param name="max_iteration" type="int" value="3" />
  <param name="filter_size_surf" type="double" value="0.5" />
  <param name="filter_size_map" type="double" value="0.5" />
  <param name="cube_side_length" type="double" value="1000" />
  <param name="runtime_pos_log_enable" type="bool" value="0" />

  <param
      name="pcd_save/pcd_save_en"
      type="bool"
      value="false" />

  <param
      name="publish/scan_bodyframe_pub_en"
      type="bool"
      value="true" />

  <node
      pkg="fast_lio"
      type="fastlio_mapping"
      name="laserMapping"
      output="screen" />

  <group if="$(arg rviz)">
    <node
        pkg="rviz"
        type="rviz"
        name="fastlio_rviz"
        args="-d $(find fast_lio)/rviz_cfg/loam_livox.rviz" />
  </group>

</launch>
```


**文件：`~/livox_fastlio/src/scout_system_bringup/launch/scout_mapping.launch`**

```xml
<launch>

  <!-- 1. Mid-360 -->
  <include file="$(find livox_ros_driver2)/launch_ROS1/msg_MID360.launch" />

  <!-- 2. FAST-LIO + PCD SAVE -->
  <include file="$(find scout_system_bringup)/launch/fastlio_mapping_scout.launch">
    <arg name="rviz" value="false" />
    <arg name="pcd_interval" value="-1" />
  </include>

  <!-- 3. TF -->
  <include file="$(find scout_tf_manager)/launch/tf_manager.launch" />

  <!-- 4. odom -> base_link 发布为 /fastlio_odom -->
  <include file="$(find scout_pose_adapter)/launch/pose_adapter.launch" />

  <!-- 5. Scout Mini 普通轮 -->
  <include file="$(find scout_bringup)/launch/scout_mini_robot_base.launch">
    <arg name="odom_topic_name" value="/scout/odom" />
    <arg name="pub_tf" value="false" />
  </include>

</launch>
```

`fastlio_mapping_scout.launch` 与 `fastlio_local_odom.launch` 的核心区别只有地图保存：建图时 `pcd_save_en=true`，定位运行时 `false`。两者都打开 `scan_bodyframe_pub_en=true`，因为后续避障和 NDT 都需要 body 点云。

### 9.1 建图模式验证

```bash
roslaunch scout_system_bringup scout_mapping.launch
```

检查：

```bash
rostopic hz /livox/lidar
rostopic hz /livox/imu
rostopic hz /cloud_registered_body
rostopic hz /scout/odom
rostopic hz /fastlio_odom
rosrun tf tf_echo odom base_link
```

小车直行时，`odom -> base_link` 应主要沿 +X 变化；Y/Z 不应随前进出现明显线性漂移。若出现明显 X-Z 同步变化，优先检查 45° 安装外参方向/符号，而不是先调导航参数。

## 10. 新建 `scout_map_tools`：保存 PCD、转换公共地图、生成 2D 栅格

### 10.1 创建包

```bash
cd ~/livox_fastlio/src
catkin_create_pkg scout_map_tools roscpp pcl_ros pcl_conversions
mkdir -p scout_map_tools/src scout_map_tools/scripts scout_map_tools/config
```


**文件：`~/livox_fastlio/src/scout_map_tools/CMakeLists.txt`**

```text
cmake_minimum_required(VERSION 3.0.2)
project(scout_map_tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

add_executable(
  pcd_to_pgm_node
  src/pcd_to_pgm.cpp
)

target_link_libraries(
  pcd_to_pgm_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(
  pcd_transform_node
  src/pcd_transform.cpp
)

target_link_libraries(
  pcd_transform_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

catkin_install_python(
  PROGRAMS
    scripts/finalize_map.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```


**文件：`~/livox_fastlio/src/scout_map_tools/src/pcd_transform.cpp`**

```cpp
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>
#include <string>

static double deg2rad(double deg)
{
    constexpr double kPi = 3.14159265358979323846;
    return deg * kPi / 180.0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scout_pcd_transform");
    ros::NodeHandle pnh("~");

    std::string input_pcd;
    std::string output_pcd;

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;

    pnh.param<std::string>("input_pcd", input_pcd, "");
    pnh.param<std::string>("output_pcd", output_pcd, "");

    pnh.param<double>("x", x, 0.0);
    pnh.param<double>("y", y, 0.0);
    pnh.param<double>("z", z, 0.0);

    pnh.param<double>("roll_deg", roll_deg, 0.0);
    pnh.param<double>("pitch_deg", pitch_deg, 0.0);
    pnh.param<double>("yaw_deg", yaw_deg, 0.0);

    if (input_pcd.empty() || output_pcd.empty())
    {
        ROS_FATAL("input_pcd/output_pcd is empty");
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(input_pcd, *cloud) < 0)
    {
        ROS_FATAL("Failed to load PCD: %s", input_pcd.c_str());
        return 1;
    }

    const float roll = static_cast<float>(deg2rad(roll_deg));
    const float pitch = static_cast<float>(deg2rad(pitch_deg));
    const float yaw = static_cast<float>(deg2rad(yaw_deg));

    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    Eigen::Affine3f tf = Eigen::Affine3f::Identity();
    tf.translation() << static_cast<float>(x),
                        static_cast<float>(y),
                        static_cast<float>(z);

    tf.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    tf.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
    tf.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));

    pcl::PointCloud<pcl::PointXYZI> output;
    pcl::transformPointCloud(*cloud, output, tf);

    if (pcl::io::savePCDFileBinary(output_pcd, output) < 0)
    {
        ROS_FATAL("Failed to save PCD: %s", output_pcd.c_str());
        return 1;
    }

    ROS_INFO("Input : %s", input_pcd.c_str());
    ROS_INFO("Output: %s", output_pcd.c_str());
    ROS_INFO("xyz=(%.4f, %.4f, %.4f)", x, y, z);
    ROS_INFO("rpy_deg=(%.3f, %.3f, %.3f)",
             roll_deg, pitch_deg, yaw_deg);
    ROS_INFO("points=%zu", output.size());

    return 0;
}
```


**文件：`~/livox_fastlio/src/scout_map_tools/src/pcd_to_pgm.cpp`**

```cpp
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

class PcdToPgm
{
public:
    PcdToPgm() : pnh_("~")
    {
        pnh_.param<std::string>("input_pcd", input_pcd_, "");
        pnh_.param<std::string>("output_pgm", output_pgm_, "");
        pnh_.param<std::string>("output_yaml", output_yaml_, "");

        pnh_.param<double>("resolution", resolution_, 0.05);
        pnh_.param<double>("padding_m", padding_m_, 0.50);
        pnh_.param<double>("floor_min_z", floor_min_z_, -0.30);
        pnh_.param<double>("floor_max_z", floor_max_z_, 0.05);
        pnh_.param<double>("obstacle_min_z", obstacle_min_z_, 0.05);
        pnh_.param<double>("obstacle_max_z", obstacle_max_z_, 1.20);
        pnh_.param<double>("free_dilation_m", free_dilation_m_, 0.10);
        pnh_.param<double>("obstacle_inflation_m", obstacle_inflation_m_, 0.30);

        generate();
    }

private:
    int index(int x, int y) const
    {
        return y * width_ + x;
    }

    void dilate(
        const std::vector<uint8_t>& input,
        std::vector<uint8_t>& output,
        int radius)
    {
        output.assign(input.size(), 0);

        for (int y = 0; y < height_; ++y)
        {
            for (int x = 0; x < width_; ++x)
            {
                if (!input[index(x, y)])
                    continue;

                for (int dy = -radius; dy <= radius; ++dy)
                {
                    for (int dx = -radius; dx <= radius; ++dx)
                    {
                        if (dx * dx + dy * dy > radius * radius)
                            continue;

                        const int nx = x + dx;
                        const int ny = y + dy;

                        if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_)
                            continue;

                        output[index(nx, ny)] = 1;
                    }
                }
            }
        }
    }

    std::string basename(const std::string& path) const
    {
        const std::size_t pos = path.find_last_of("/\\");
        if (pos == std::string::npos)
            return path;
        return path.substr(pos + 1);
    }

    void writePgm(const std::vector<uint8_t>& image)
    {
        std::ofstream file(output_pgm_, std::ios::binary);
        if (!file)
            throw std::runtime_error("Cannot open output PGM.");

        file << "P5\n";
        file << width_ << " " << height_ << "\n";
        file << "255\n";

        for (int y = height_ - 1; y >= 0; --y)
        {
            for (int x = 0; x < width_; ++x)
            {
                const uint8_t value = image[index(x, y)];
                file.write(reinterpret_cast<const char*>(&value), 1);
            }
        }
    }

    void writeYaml()
    {
        std::ofstream file(output_yaml_);
        if (!file)
            throw std::runtime_error("Cannot open output YAML.");

        file << "image: " << basename(output_pgm_) << "\n";
        file << "resolution: " << resolution_ << "\n";
        file << "origin: [" << min_x_ << ", " << min_y_ << ", 0.0]\n";
        file << "negate: 0\n";
        file << "occupied_thresh: 0.65\n";
        file << "free_thresh: 0.196\n";
    }

    void generate()
    {
        if (input_pcd_.empty() || output_pgm_.empty() || output_yaml_.empty())
            throw std::runtime_error("PCD/PGM/YAML path is empty.");

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZI>);

        if (pcl::io::loadPCDFile<pcl::PointXYZI>(input_pcd_, *cloud) < 0)
            throw std::runtime_error("Failed to load PCD.");

        if (cloud->empty())
            throw std::runtime_error("PCD is empty.");

        double max_x = -std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();

        min_x_ = std::numeric_limits<double>::infinity();
        min_y_ = std::numeric_limits<double>::infinity();

        for (const auto& p : cloud->points)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                continue;

            min_x_ = std::min(min_x_, static_cast<double>(p.x));
            min_y_ = std::min(min_y_, static_cast<double>(p.y));
            max_x = std::max(max_x, static_cast<double>(p.x));
            max_y = std::max(max_y, static_cast<double>(p.y));
        }

        min_x_ -= padding_m_;
        min_y_ -= padding_m_;
        max_x += padding_m_;
        max_y += padding_m_;

        width_ = static_cast<int>(std::ceil((max_x - min_x_) / resolution_));
        height_ = static_cast<int>(std::ceil((max_y - min_y_) / resolution_));

        const std::size_t cell_count =
            static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);

        std::vector<uint32_t> floor_count(cell_count, 0);
        std::vector<uint32_t> obstacle_count(cell_count, 0);

        for (const auto& p : cloud->points)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                continue;

            const int gx = static_cast<int>(
                std::floor((p.x - min_x_) / resolution_));

            const int gy = static_cast<int>(
                std::floor((p.y - min_y_) / resolution_));

            if (gx < 0 || gy < 0 || gx >= width_ || gy >= height_)
                continue;

            const int id = index(gx, gy);

            if (p.z >= floor_min_z_ && p.z <= floor_max_z_)
                ++floor_count[id];

            if (p.z >= obstacle_min_z_ && p.z <= obstacle_max_z_)
                ++obstacle_count[id];
        }

        std::vector<uint8_t> floor_mask(cell_count, 0);
        std::vector<uint8_t> obstacle_mask(cell_count, 0);

        for (std::size_t i = 0; i < cell_count; ++i)
        {
            if (floor_count[i] > 0)
                floor_mask[i] = 1;

            if (obstacle_count[i] > 0)
                obstacle_mask[i] = 1;
        }

        const int free_radius = static_cast<int>(
            std::round(free_dilation_m_ / resolution_));

        const int obstacle_radius = static_cast<int>(
            std::round(obstacle_inflation_m_ / resolution_));

        std::vector<uint8_t> free_mask;
        std::vector<uint8_t> inflated_obstacle;

        dilate(floor_mask, free_mask, free_radius);
        dilate(obstacle_mask, inflated_obstacle, obstacle_radius);

        std::vector<uint8_t> image(cell_count, 205);

        for (std::size_t i = 0; i < cell_count; ++i)
        {
            if (free_mask[i])
                image[i] = 254;
        }

        for (std::size_t i = 0; i < cell_count; ++i)
        {
            if (inflated_obstacle[i])
                image[i] = 0;
        }

        writePgm(image);
        writeYaml();

        ROS_INFO("PCD points: %zu", cloud->size());
        ROS_INFO("Map: %d x %d, resolution %.3f", width_, height_, resolution_);
        ROS_INFO("PGM: %s", output_pgm_.c_str());
        ROS_INFO("YAML: %s", output_yaml_.c_str());
    }

private:
    ros::NodeHandle pnh_;

    std::string input_pcd_;
    std::string output_pgm_;
    std::string output_yaml_;

    double resolution_;
    double padding_m_;
    double floor_min_z_;
    double floor_max_z_;
    double obstacle_min_z_;
    double obstacle_max_z_;
    double free_dilation_m_;
    double obstacle_inflation_m_;

    int width_{0};
    int height_{0};

    double min_x_{0.0};
    double min_y_{0.0};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scout_pcd_to_pgm");

    try
    {
        PcdToPgm converter;
    }
    catch (const std::exception& e)
    {
        ROS_FATAL("%s", e.what());
        return 1;
    }

    return 0;
}
```


**文件：`~/livox_fastlio/src/scout_map_tools/config/scout_raw.yaml`**

```yaml
resolution: 0.05
padding_m: 0.50

floor_min_z: -0.30
floor_max_z: 0.05

obstacle_min_z: 0.05
obstacle_max_z: 1.20

free_dilation_m: 0.10
obstacle_inflation_m: 0.00
```


**文件：`~/livox_fastlio/src/scout_map_tools/config/scout_nav.yaml`**

```yaml
resolution: 0.05
padding_m: 0.50

floor_min_z: -0.30
floor_max_z: 0.05

obstacle_min_z: 0.05
obstacle_max_z: 1.20

free_dilation_m: 0.10
obstacle_inflation_m: 0.30
```


**文件：`~/livox_fastlio/src/scout_map_tools/scripts/finalize_map.py`**

```python
#!/usr/bin/env python3
import argparse
import copy
import datetime
import os
import shutil
import subprocess
import sys
import time

import yaml


def run(cmd):
    print("[RUN] " + " ".join(cmd))
    subprocess.run(cmd, check=True)


def rospack_find(pkg):
    return subprocess.check_output(
        ["rospack", "find", pkg],
        text=True
    ).strip()


def master_online():
    try:
        import rosgraph
        return rosgraph.is_master_online()
    except Exception:
        return False


def wait_master(timeout_sec=8.0):
    start = time.time()
    while time.time() - start < timeout_sec:
        if master_online():
            return True
        time.sleep(0.2)
    return False


def private_args(params):
    out = []
    for key, value in params.items():
        if isinstance(value, bool):
            value = "true" if value else "false"
        out.append("_{}:={}".format(key, value))
    return out


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data if data is not None else {}


def main():
    parser = argparse.ArgumentParser(
        description="Archive FAST-LIO PCD and build 3D/2D maps"
    )
    parser.add_argument("map_name")
    parser.add_argument(
        "--source",
        default=None,
        help="FAST-LIO scans.pcd path"
    )
    parser.add_argument(
        "--replace-raw",
        action="store_true",
        help="replace existing raw_camera_init.pcd from --source"
    )
    args = parser.parse_args()

    home = os.path.expanduser("~")
    workspace = os.path.join(home, "livox_fastlio")
    map_dir = os.path.join(workspace, "maps", args.map_name)
    os.makedirs(map_dir, exist_ok=True)

    fast_lio_dir = rospack_find("fast_lio")
    bringup_dir = rospack_find("scout_system_bringup")
    tools_dir = rospack_find("scout_map_tools")

    source_pcd = args.source or os.path.join(
        fast_lio_dir, "PCD", "scans.pcd"
    )

    geometry_yaml = os.path.join(
        bringup_dir, "config", "scout_geometry.yaml"
    )

    raw_profile_path = os.path.join(
        tools_dir, "config", "scout_raw.yaml"
    )

    nav_profile_path = os.path.join(
        tools_dir, "config", "scout_nav.yaml"
    )

    raw_pcd = os.path.join(map_dir, "raw_camera_init.pcd")
    public_pcd = os.path.join(map_dir, "public_map.pcd")

    if not os.path.isfile(raw_pcd) or args.replace_raw:
        if not os.path.isfile(source_pcd):
            raise FileNotFoundError(
                "FAST-LIO source PCD not found: " + source_pcd
            )
        shutil.copy2(source_pcd, raw_pcd)
        print("[OK] archived raw PCD -> " + raw_pcd)
    else:
        print("[KEEP] existing raw PCD -> " + raw_pcd)
        print("       use --replace-raw only when intentionally replacing it")

    geometry = load_yaml(geometry_yaml)
    tf_cfg = geometry["odom_to_camera_init"]

    started_master = None
    if not master_online():
        print("[INFO] ROS master is offline; starting temporary roscore")
        started_master = subprocess.Popen(
            ["roscore"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        if not wait_master():
            started_master.terminate()
            raise RuntimeError("temporary roscore failed to start")

    try:
        transform_params = {
            "input_pcd": raw_pcd,
            "output_pcd": public_pcd,
            "x": tf_cfg["x"],
            "y": tf_cfg["y"],
            "z": tf_cfg["z"],
            "roll_deg": tf_cfg["roll_deg"],
            "pitch_deg": tf_cfg["pitch_deg"],
            "yaw_deg": tf_cfg["yaw_deg"],
        }

        run(
            ["rosrun", "scout_map_tools", "pcd_transform_node"]
            + private_args(transform_params)
        )

        profiles = [
            (
                "raw",
                raw_profile_path,
                os.path.join(map_dir, "map_raw.pgm"),
                os.path.join(map_dir, "map_raw.yaml"),
            ),
            (
                "nav",
                nav_profile_path,
                os.path.join(map_dir, "map.pgm"),
                os.path.join(map_dir, "map.yaml"),
            ),
        ]

        profile_snapshot = {}

        for name, profile_path, output_pgm, output_yaml in profiles:
            cfg = load_yaml(profile_path)
            profile_snapshot[name] = copy.deepcopy(cfg)

            params = copy.deepcopy(cfg)
            params["input_pcd"] = public_pcd
            params["output_pgm"] = output_pgm
            params["output_yaml"] = output_yaml

            run(
                ["rosrun", "scout_map_tools", "pcd_to_pgm_node"]
                + private_args(params)
            )

        metadata = {
            "map_name": args.map_name,
            "created_at": datetime.datetime.now().isoformat(timespec="seconds"),
            "frames": {
                "raw_pcd": "camera_init",
                "public_map": "map",
            },
            "files": {
                "raw_pcd": "raw_camera_init.pcd",
                "public_pcd": "public_map.pcd",
                "raw_map_yaml": "map_raw.yaml",
                "nav_map_yaml": "map.yaml",
            },
            "geometry_snapshot": geometry,
            "map_generation": profile_snapshot,
        }

        metadata_path = os.path.join(map_dir, "map_metadata.yaml")
        with open(metadata_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(
                metadata,
                f,
                allow_unicode=True,
                sort_keys=False
            )

        print("\n[DONE] map finalized")
        print("  map dir    : " + map_dir)
        print("  raw PCD    : " + raw_pcd)
        print("  public PCD : " + public_pcd)
        print("  nav map    : " + os.path.join(map_dir, "map.yaml"))
        print("  metadata   : " + metadata_path)

    finally:
        if started_master is not None:
            started_master.terminate()
            started_master.wait(timeout=5)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("[ERROR] {}".format(e), file=sys.stderr)
        sys.exit(1)
```

```bash
chmod +x ~/livox_fastlio/src/scout_map_tools/scripts/finalize_map.py
cd ~/livox_fastlio
catkin_make -j1
source devel/setup.bash
```

### 10.2 实际建图与地图归档

1. 启动建图：

```bash
roslaunch scout_system_bringup scout_mapping.launch map_name:=scout_map_01
```

2. 遥控小车完成环境扫描。

3. 保持 launch 运行，在另一终端完成保存和地图生成：

```bash
rosservice call /finish_mapping
```

`scout_mapping_finisher` 会先调用
`/scout_pointcloud_mapper/save_map`，成功后自动执行：

```bash
rosrun scout_map_tools finalize_map.py scout_map_01 --replace-raw
```

正常返回必须包含：

```text
success: True
```

4. 确认成功后再 `Ctrl+C` 停止建图。不要依赖节点退出保存地图。

生成：

```text
~/livox_fastlio/maps/scout_map_01/
├── raw_camera_init.pcd   # FAST-LIO 原始 camera_init 坐标点云
├── public_map.pcd        # 经过 odom->camera_init 安装变换后的 map 坐标点云
├── map_raw.pgm
├── map_raw.yaml          # 导航 global costmap 使用，不预膨胀
├── map.pgm
├── map.yaml              # 人看/定位辅助的预膨胀二维图
└── map_metadata.yaml     # 保存当时外参与制图参数快照
```

为什么同时有 `map_raw.yaml` 和 `map.yaml`：导航 global costmap 自己还有 `InflationLayer`，所以使用 `map_raw.yaml` 避免静态图预膨胀后再被 costmap 二次膨胀。

## 11. 下载并改造 `fast_lio_localization`，让它发布 `map -> odom`

当前基准 commit：`247aa88e1a17c4f42a8a0036c2144606f0d9714a`。

```bash
cd ~/livox_fastlio/src
git clone https://github.com/BruceXSK/fast_lio_localization.git
cd fast_lio_localization
git checkout 247aa88e1a17c4f42a8a0036c2144606f0d9714a
```

### 11.1 替换 `src/fast_lio_localization.cpp`

当前工程不是小改，而是把上游定位器改成 Scout 的 `map -> odom` 修正结构，并增加：

- `odom_frame` 默认 `odom`；
- `tf_postdate_sec=0.25`，向未来预发布 TF，避免 move_base/TEB 偶发 future extrapolation；
- `/initialpose` 触发 NDT；
- 按位移 0.50 m 或转角 10° 再做一次 NDT；
- 无论是否触发新 NDT，都持续发布当前 `map -> odom`；
- NDT 后按 `T_map_odom = T_map_base × inverse(T_odom_base)` 计算修正。

请把该文件**整体替换**为附录中的当前最终代码。


**文件：`~/livox_fastlio/src/fast_lio_localization/src/fast_lio_localization.cpp（完整替换）`**

```cpp
//
// Created by bruce on 2022/3/29.
//

#include <chrono>
#include <cmath>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>

#include "pclomp/ndt_omp.h"

using namespace std;

typedef pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> NDT;
typedef pcl::PointCloud<pcl::PointXYZI> Cloud;


class Config
{
public:
    // 当前 Scout 系统要求 fast_lio_localization 发布：
    //
    // map -> odom
    //
    // 不再使用原始项目默认的 map -> camera_init。
    string odomFrame = "odom";

    // map -> odom TF 向未来预发布的时间。
    // 用于避免 TEB / move_base 控制周期与 localization TF
    // 发布周期之间几十毫秒的相位差导致 future extrapolation。
    double tfPostdateSec = 0.25;

    struct
    {
        bool debug = false;
        int numThreads = 4;
        int maximumIterations = 20;
        float voxelLeafSize = 0.1;
        float resolution = 1.0;
        double transformationEpsilon = 0.01;
        double stepSize = 0.1;
        double threshShift = 2;
        double threshRot = M_PI / 12;
        double minScanRange = 1.0;
        double maxScanRange = 100;
    } ndt;

    explicit Config(ros::NodeHandle &nh) : _nh(nh)
    {
        // 必须读取 odom_frame。
        // 如果 launch 中没有设置，则默认使用 "odom"。
        _nh.param<string>("odom_frame", odomFrame, string("odom"));

        // map -> odom TF 向未来预发布时间。
        _nh.param("tf_postdate_sec", tfPostdateSec, 0.25);

        _nh.getParam("ndt/debug", ndt.debug);
        _nh.getParam("ndt/num_threads", ndt.numThreads);
        _nh.getParam("ndt/maximum_iterations", ndt.maximumIterations);
        _nh.getParam("ndt/voxel_leaf_size", ndt.voxelLeafSize);
        _nh.getParam("ndt/transformation_epsilon", ndt.transformationEpsilon);
        _nh.getParam("ndt/step_size", ndt.stepSize);
        _nh.getParam("ndt/resolution", ndt.resolution);
        _nh.getParam("ndt/thresh_shift", ndt.threshShift);
        _nh.getParam("ndt/thresh_rot", ndt.threshRot);
        _nh.getParam("ndt/min_scan_range", ndt.minScanRange);
        _nh.getParam("ndt/max_scan_range", ndt.maxScanRange);

        ROS_INFO("fast_lio_localization config:");
        ROS_INFO("  odom_frame      = %s", odomFrame.c_str());
        ROS_INFO("  tf_postdate_sec = %.3f", tfPostdateSec);
    }

private:
    ros::NodeHandle &_nh;
};


class Localizer
{
public:
    explicit Localizer(ros::NodeHandle &nh) :
            _nh(nh),
            _cfg(nh),
            _mapPtr(new Cloud),
            _mapFilteredPtr(new Cloud)
    {
        _mapSub = _nh.subscribe(
                "/map_cloud",
                10,
                &Localizer::mapCallback,
                this
        );

        _initPoseSub = _nh.subscribe(
                "/initialpose",
                10,
                &Localizer::initPoseWithNDTCallback,
                this
        );

        _pcSubPtr =
                new message_filters::Subscriber<sensor_msgs::PointCloud2>(
                        nh,
                        "/velodyne_points",
                        1
                );

        _odomSubPtr =
                new message_filters::Subscriber<nav_msgs::Odometry>(
                        nh,
                        "/odom_lio",
                        1
                );

        _syncPtr =
                new message_filters::Synchronizer<syncPolicy>(
                        syncPolicy(10),
                        *_pcSubPtr,
                        *_odomSubPtr
                );

        _syncPtr->registerCallback(
                boost::bind(
                        &Localizer::syncCallback,
                        this,
                        _1,
                        _2
                )
        );

        _voxelGridFilter.setLeafSize(
                _cfg.ndt.voxelLeafSize,
                _cfg.ndt.voxelLeafSize,
                _cfg.ndt.voxelLeafSize
        );

        _ndt.setNumThreads(_cfg.ndt.numThreads);
        _ndt.setTransformationEpsilon(_cfg.ndt.transformationEpsilon);
        _ndt.setStepSize(_cfg.ndt.stepSize);
        _ndt.setResolution(_cfg.ndt.resolution);
        _ndt.setMaximumIterations(_cfg.ndt.maximumIterations);

        _odomMap.setIdentity();
    }

private:
    ros::NodeHandle &_nh;

    ros::Subscriber _mapSub;
    ros::Subscriber _initPoseSub;

    tf2_ros::TransformBroadcaster _br;

    message_filters::Subscriber<sensor_msgs::PointCloud2> *_pcSubPtr;
    message_filters::Subscriber<nav_msgs::Odometry> *_odomSubPtr;

    typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::PointCloud2,
            nav_msgs::Odometry
    > syncPolicy;

    message_filters::Synchronizer<syncPolicy> *_syncPtr;

    NDT _ndt;
    pcl::VoxelGrid<pcl::PointXYZI> _voxelGridFilter;

    Config _cfg;

    Cloud::Ptr _mapPtr;
    Cloud::Ptr _mapFilteredPtr;

    tf::Pose _baseOdom;
    tf::Pose _odomMap;

    sensor_msgs::PointCloud2::ConstPtr _pcPtr = nullptr;


    void mapCallback(
            const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        ROS_INFO("Get map");

        pcl::fromROSMsg<pcl::PointXYZI>(
                *msg,
                *_mapPtr
        );

        _ndt.setInputTarget(_mapPtr);
    }


    void initPoseCallback(
            const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        auto &q = msg->pose.pose.orientation;
        auto &p = msg->pose.pose.position;

        tf::Pose baseMap(
                tf::Quaternion(
                        q.x,
                        q.y,
                        q.z,
                        q.w
                ),
                tf::Vector3(
                        p.x,
                        p.y,
                        p.z
                )
        );

        _odomMap =
                baseMap *
                _baseOdom.inverse();

        ROS_INFO("Initial pose set");
    }


    void initPoseWithNDTCallback(
            const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        if (_pcPtr == nullptr)
        {
            ROS_WARN("No point cloud");
            return;
        }

        ROS_INFO("Initial pose set");

        auto &q = msg->pose.pose.orientation;
        auto &p = msg->pose.pose.position;

        tf::Pose baseMap(
                tf::Quaternion(
                        q.x,
                        q.y,
                        q.z,
                        q.w
                ),
                tf::Vector3(
                        p.x,
                        p.y,
                        p.z
                )
        );

        match(
                _pcPtr,
                baseMap
        );

        publishTF();
    }


    void syncCallback(
            const sensor_msgs::PointCloud2::ConstPtr &pcMsg,
            const nav_msgs::Odometry::ConstPtr &odomMsg)
    {
        _pcPtr = pcMsg;

        tf::poseMsgToTF(
                odomMsg->pose.pose,
                _baseOdom
        );

        static tf::Pose lastNDTPose = _baseOdom;

        auto T =
                lastNDTPose.inverseTimes(
                        _baseOdom
                );

        const double shift =
                hypot(
                        T.getOrigin().x(),
                        T.getOrigin().y()
                );

        const double rotation =
                std::fabs(
                        tf::getYaw(
                                T.getRotation()
                        )
                );

        if (shift > _cfg.ndt.threshShift ||
            rotation > _cfg.ndt.threshRot)
        {
            match(
                    pcMsg,
                    _odomMap * _baseOdom
            );

            lastNDTPose = _baseOdom;
        }

        // 即使没有触发新的 NDT，
        // 也持续重新发布当前 map -> odom 修正值。
        publishTF();
    }


    /**
     * Matching the point cloud with map to calculate `_odomMap`.
     *
     * @param pcPtr  The point cloud for matching.
     * @param baseMap The guess matrix.
     */
    void match(
            const sensor_msgs::PointCloud2::ConstPtr &pcPtr,
            const tf::Transform &baseMap)
    {
        static chrono::steady_clock::time_point t0;
        static chrono::steady_clock::time_point t1;

        Cloud::Ptr tmpCloudPtr(
                new Cloud
        );

        pcl::fromROSMsg(
                *pcPtr,
                *tmpCloudPtr
        );

        Cloud::Ptr filteredCloudPtr(
                new Cloud
        );

        _voxelGridFilter.setInputCloud(
                tmpCloudPtr
        );

        _voxelGridFilter.filter(
                *filteredCloudPtr
        );

        Cloud::Ptr scanCloudPtr(
                new Cloud
        );

        for (const auto &p : *filteredCloudPtr)
        {
            const auto r =
                    hypot(
                            p.x,
                            p.y
                    );

            if (r > _cfg.ndt.minScanRange &&
                r < _cfg.ndt.maxScanRange)
            {
                scanCloudPtr->push_back(p);
            }
        }

        _ndt.setInputSource(
                scanCloudPtr
        );

        Eigen::Affine3d baseMapMat;

        tf::poseTFToEigen(
                baseMap,
                baseMapMat
        );

        Cloud::Ptr outputCloudPtr(
                new Cloud
        );

        if (_cfg.ndt.debug)
        {
            t0 = chrono::steady_clock::now();
        }

        _ndt.align(
                *outputCloudPtr,
                baseMapMat.matrix().cast<float>()
        );

        if (_cfg.ndt.debug)
        {
            t1 = chrono::steady_clock::now();
        }

        auto tNDT =
                _ndt.getFinalTransformation();

        tf::Transform baseMapNDT;

        tf::poseEigenToTF(
                Eigen::Affine3d(
                        tNDT.cast<double>()
                ),
                baseMapNDT
        );

        // 计算：
        //
        // T_map_odom =
        // T_map_base *
        // inverse(T_odom_base)
        //
        // 当前 Scout 系统中，
        // 这个 correction 最终作为：
        //
        // map -> odom
        //
        // 发布。
        _odomMap =
                baseMapNDT *
                _baseOdom.inverse();

        if (_cfg.ndt.debug)
        {
            ROS_INFO(
                    "NDT: %ldms",
                    chrono::duration_cast<chrono::milliseconds>(
                            t1 - t0
                    ).count()
            );
        }

        ROS_INFO("NDT Relocated");
    }


    void publishTF()
    {
        geometry_msgs::TransformStamped tfMsg;

        // 关键修改：
        //
        // map -> odom 向未来预发布 0.25 秒，
        // 避免 TEB 查询当前时刻 TF 时，
        // 最新 map -> odom 尚落后几十毫秒而出现：
        //
        // Lookup would require extrapolation into the future
        //
        tfMsg.header.stamp =
                ros::Time::now() +
                ros::Duration(
                        _cfg.tfPostdateSec
                );

        tfMsg.header.frame_id = "map";

        // 必须是 odom。
        tfMsg.child_frame_id =
                _cfg.odomFrame;

        tfMsg.transform.translation.x =
                _odomMap.getOrigin().x();

        tfMsg.transform.translation.y =
                _odomMap.getOrigin().y();

        tfMsg.transform.translation.z =
                _odomMap.getOrigin().z();

        tfMsg.transform.rotation.x =
                _odomMap.getRotation().x();

        tfMsg.transform.rotation.y =
                _odomMap.getRotation().y();

        tfMsg.transform.rotation.z =
                _odomMap.getRotation().z();

        tfMsg.transform.rotation.w =
                _odomMap.getRotation().w();

        _br.sendTransform(
                tfMsg
        );
    }
};


int main(
        int argc,
        char **argv)
{
    ros::init(
            argc,
            argv,
            "fast_lio_localization"
    );

    ros::NodeHandle nh("~");

    Localizer localizer(
            nh
    );

    ros::spin();

    return 0;
}
```

**当前已知技术债：** 该代码调用 `_ndt.align()` 后直接采用 `getFinalTransformation()`，目前没有 `hasConverged()`、fitness score 或异常位姿门控。因此 RViz 给的初始位姿必须足够接近真实位置；后续如果要增强可靠性，应优先给 NDT 结果增加质量门控。

## 12. 建立重定位与完整定位启动文件


**文件：`~/livox_fastlio/src/scout_system_bringup/launch/scout_relocalization.launch`**

```xml
<launch>

  <arg
      name="map_pcd"
      default="$(env HOME)/livox_fastlio/maps/scout_map_01/public_map.pcd" />

  <node
      pkg="fast_lio_localization"
      type="map_loader"
      name="scout_map_loader"
      output="screen">

    <param name="map_path" value="$(arg map_pcd)" />

  </node>

  <node
      pkg="fast_lio_localization"
      type="fast_lio_localization"
      name="scout_global_localizer"
      output="screen">

    <param name="odom_frame" value="odom" />

    <param name="ndt/debug" value="true" />
    <param name="ndt/num_threads" value="4" />
    <param name="ndt/maximum_iterations" value="30" />
    <param name="ndt/voxel_leaf_size" value="0.20" />
    <param name="ndt/resolution" value="1.0" />
    <param name="ndt/transformation_epsilon" value="0.01" />
    <param name="ndt/step_size" value="0.10" />
    <param name="ndt/thresh_shift" value="0.50" />
    <param name="ndt/thresh_rot" value="0.174533" />
    <param name="ndt/min_scan_range" value="0.50" />
    <param name="ndt/max_scan_range" value="50.0" />

    <remap from="/velodyne_points" to="/cloud_registered_base" />
    <remap from="/odom_lio" to="/fastlio_odom" />

  </node>

</launch>
```


**文件：`~/livox_fastlio/src/scout_system_bringup/launch/scout_localization.launch`**

```xml
<launch>

  <arg name="map_name" default="scout_map_01" />

  <arg
      name="map_dir"
      default="$(env HOME)/livox_fastlio/maps/$(arg map_name)" />

  <arg
      name="map_pcd"
      default="$(arg map_dir)/public_map.pcd" />

  <arg
      name="map_yaml"
      default="$(arg map_dir)/map.yaml" />

  <!-- 1. Mid-360 -->
  <include file="$(find livox_ros_driver2)/launch_ROS1/msg_MID360.launch" />

  <!-- 2. FAST-LIO local odometry -->
  <include file="$(find scout_system_bringup)/launch/fastlio_local_odom.launch" />

  <!-- 3. 固定 TF：内部加载 scout_geometry.yaml -->
  <include file="$(find scout_tf_manager)/launch/tf_manager.launch" />

  <!-- 4. odom -> base_link Odometry -->
  <include file="$(find scout_pose_adapter)/launch/pose_adapter.launch" />

  <!-- 5. body 点云 -> base_link 点云 -->
  <include file="$(find scout_cloud_adapter)/launch/cloud_adapter.launch" />

  <!-- 6. PCD 全局重定位 -->
  <include file="$(find scout_system_bringup)/launch/scout_relocalization.launch">
    <arg name="map_pcd" value="$(arg map_pcd)" />
  </include>

  <!-- 7. 2D OccupancyGrid -->
  <node
      pkg="map_server"
      type="map_server"
      name="scout_map_server"
      args="$(arg map_yaml)"
      output="screen">

    <remap from="map" to="/map_2d" />
  </node>

  <!-- 8. Scout Mini 普通轮 -->
  <include file="$(find scout_bringup)/launch/scout_mini_robot_base.launch">
    <arg name="odom_topic_name" value="/scout/odom" />
    <arg name="pub_tf" value="false" />
  </include>

</launch>
```

这里的两个关键 remap：

```text
/velodyne_points <- /cloud_registered_base
/odom_lio        <- /fastlio_odom
```

即 NDT 使用 base_link 坐标的当前扫描，并使用 FAST-LIO TF 转出来的局部位姿。

### 12.1 编译

```bash
cd ~/livox_fastlio
rm -rf build devel
catkin_make -j1
source devel/setup.bash
```

### 12.2 定位验证

```bash
roslaunch scout_system_bringup scout_localization.launch map_name:=scout_map_01
```

检查：

```bash
rostopic hz /map_cloud
rostopic hz /cloud_registered_base
rostopic hz /fastlio_odom
rosrun tf tf_echo odom base_link
rosrun tf tf_echo map odom
```

启动后 `map -> odom` 可能先是单位变换，因为 `_odomMap` 初始化为 Identity，**这不代表已经完成全局重定位**。必须在 RViz 使用 **2D Pose Estimate** 发布 `/initialpose`。看到控制台 `NDT Relocated` 后，再检查点云与地图是否对齐。

建议用 RViz：Fixed Frame=`map`，显示 `/map_cloud`、`/map_2d`、`/cloud_registered_base`。初始位姿先保证 XY 和朝向基本正确，再让 NDT 收敛。

## 13. 新建 `scout_navigation`：Dijkstra 全局规划 + TEB 局部规划

### 13.1 创建包

```bash
cd ~/livox_fastlio/src
catkin_create_pkg scout_navigation rospy nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs move_base_msgs
mkdir -p scout_navigation/config scout_navigation/launch scout_navigation/scripts
```

当前正式导航为 Dijkstra + TEB。除了导航 YAML/launch，本包还承担**导航规划日志采集与自动分析**，所以 `scripts/` 目录必须同时建立。

`catkin_create_pkg` 生成的构建文件需要改成下面内容，否则后面 `analyze_nav_bag.py`、`nav_log_session.sh`、`global_plan_tester.py` 不会按当前工程方式安装/执行。

**文件：`~/livox_fastlio/src/scout_navigation/CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(scout_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  move_base_msgs
)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/global_plan_tester.py
    scripts/analyze_nav_bag.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS
    scripts/nav_log_session.sh
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

**文件：`~/livox_fastlio/src/scout_navigation/package.xml`**

```xml
<?xml version="1.0"?>
<package format="2">
  <name>scout_navigation</name>
  <version>0.0.0</version>
  <description>The scout_navigation package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <maintainer email="nvidia@todo.todo">nvidia</maintainer>


  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>TODO</license>


  <!-- Url tags are optional, but multiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://wiki.ros.org/scout_navigation</url> -->


  <!-- Author tags are optional, multiple are allowed, one per tag -->
  <!-- Authors do not have to be maintainers, but could be -->
  <!-- Example: -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->


  <!-- The *depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use depend as a shortcut for packages that are both build and exec dependencies -->
  <!--   <depend>roscpp</depend> -->
  <!--   Note that this is equivalent to the following: -->
  <!--   <build_depend>roscpp</build_depend> -->
  <!--   <exec_depend>roscpp</exec_depend> -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use build_export_depend for packages you need in order to build against this package: -->
  <!--   <build_export_depend>message_generation</build_export_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use exec_depend for packages you need at runtime: -->
  <!--   <exec_depend>message_runtime</exec_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <!-- Use doc_depend for packages you need only for building documentation: -->
  <!--   <doc_depend>doxygen</doc_depend> -->
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>move_base_msgs</build_depend>
  <build_depend>nav_msgs</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>tf2_geometry_msgs</build_depend>
  <build_depend>tf2_ros</build_depend>
  <build_export_depend>geometry_msgs</build_export_depend>
  <build_export_depend>move_base_msgs</build_export_depend>
  <build_export_depend>nav_msgs</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>tf2_geometry_msgs</build_export_depend>
  <build_export_depend>tf2_ros</build_export_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>move_base_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>rosbag</exec_depend>
  <exec_depend>teb_local_planner</exec_depend>
  <exec_depend>tf2_geometry_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>


  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->

  </export>
</package>
```

其中 `rosbag` 是导航日志功能的运行依赖，`teb_local_planner` 是当前正式局部规划器运行依赖。DWA 不作为正式方案建立或配置。


**文件：`~/livox_fastlio/src/scout_navigation/config/costmap_common.yaml`**

```yaml
# Scout Mini footprint
# 官方裸底盘尺寸：612 mm x 580 mm
# 下面先按 base_link 位于车体几何中心配置。
# 实车导航前必须重新测量包含防撞杆、支架、计算平台等在内的最大外轮廓。

footprint:
  - [ 0.370,  0.295]
  - [ 0.370, -0.295]
  - [-0.300, -0.295]
  - [-0.300,  0.295]

footprint_padding: 0.03

transform_tolerance: 0.5
```


**文件：`~/livox_fastlio/src/scout_navigation/config/global_costmap.yaml`**

```yaml
global_frame: map
robot_base_frame: base_link

update_frequency: 2.0
publish_frequency: 1.0
transform_tolerance: 0.5

rolling_window: false

plugins:
  - {name: static_layer,    type: "costmap_2d::StaticLayer"}
  - {name: inflation_layer, type: "costmap_2d::InflationLayer"}

static_layer:
  map_topic: /nav_static_map
  subscribe_to_updates: false

inflation_layer:
  inflation_radius: 0.45
  cost_scaling_factor: 4.0
```


**文件：`~/livox_fastlio/src/scout_navigation/config/local_costmap.yaml`**

```yaml
global_frame: odom
robot_base_frame: base_link

update_frequency: 8.0
publish_frequency: 4.0
transform_tolerance: 0.5

rolling_window: true
width: 6.0
height: 6.0
resolution: 0.05

plugins:
  - {name: obstacle_layer,  type: "costmap_2d::ObstacleLayer"}
  - {name: inflation_layer, type: "costmap_2d::InflationLayer"}

obstacle_layer:
  enabled: true
  footprint_clearing_enabled: true

  observation_sources: fastlio_cloud

  fastlio_cloud:
    topic: /cloud_registered_body
    sensor_frame: body
    data_type: PointCloud2

    marking: true
    clearing: true

    # 高度区间是第一轮测试值。
    # 目标：去掉地面噪声，同时保留真正会碰撞车体的障碍物。
    min_obstacle_height: 0.08
    max_obstacle_height: 1.50

    obstacle_range: 4.0
    raytrace_range: 5.0

    observation_persistence: 0.0
    expected_update_rate: 0.0

inflation_layer:
  inflation_radius: 0.45
  cost_scaling_factor: 4.0
```


**文件：`~/livox_fastlio/src/scout_navigation/config/global_planner.yaml`**

```yaml
GlobalPlanner:
  allow_unknown: false
  default_tolerance: 0.15

  use_dijkstra: true
  use_quadratic: true
  use_grid_path: false
  old_navfn_behavior: false

  visualize_potential: false
```


**文件：`~/livox_fastlio/src/scout_navigation/config/move_base_teb.yaml`**

```yaml
base_global_planner: global_planner/GlobalPlanner
base_local_planner: teb_local_planner/TebLocalPlannerROS

# 有 goal 时每秒允许重新生成一次全局路径。
planner_frequency: 1.0
planner_patience: 5.0

controller_frequency: 10.0
controller_patience: 5.0

# 初期仍关闭 move_base 外部 recovery，避免系统自动旋转影响 TEB 基线判断。
# TEB 自身的 shrink_horizon / oscillation recovery 在 teb_local_planner.yaml 中单独控制。
recovery_behavior_enabled: false
clearing_rotation_allowed: false

oscillation_timeout: 5.0
oscillation_distance: 0.20

shutdown_costmaps: false
```


**文件：`~/livox_fastlio/src/scout_navigation/config/teb_local_planner.yaml`**

```yaml
TebLocalPlannerROS:
  # ============================================================
  # Scout Mini + FAST-LIO / ROS1 Noetic TEB 第一阶段基线
  # 目标：先稳定替换 DWA；保留 0.35 m/s 线速度上限；不允许主动倒车。
  # 第一阶段先关闭 homotopy，确认基础 TEB 正常后再打开双拓扑。
  # ============================================================

  # -------------------------
  # Frames / Odometry
  # -------------------------
  odom_topic: /scout/odom
  map_frame: map

  # -------------------------
  # Trajectory
  # -------------------------
  teb_autosize: true
  dt_ref: 0.30
  dt_hysteresis: 0.10
  min_samples: 5
  max_samples: 50

  global_plan_overwrite_orientation: true
  allow_init_with_backwards_motion: false

  # 当前 local costmap 为 6 m x 6 m rolling window。
  # 第一版看前方 3 m，避免像 DWA 一样只有很短的 rollout。
  max_global_plan_lookahead_dist: 3.0
  global_plan_prune_distance: 1.0

  # -1 表示不强制把 global plan 采成 via points。
  # 这样 TEB 在遇到障碍时有空间明显偏离全局路径做绕行。
  global_plan_viapoint_sep: -1.0
  via_points_ordered: false

  exact_arc_length: false
  feasibility_check_no_poses: 10

  # 打开反馈，方便 rosbag 记录和后续诊断。
  publish_feedback: true

  # -------------------------
  # Robot / Differential Drive
  # -------------------------
  max_vel_x: 0.35
  max_vel_x_backwards: 0.00
  max_vel_y: 0.0
  max_vel_theta: 1.00

  acc_lim_x: 0.50
  acc_lim_theta: 2.50

  # Scout Mini 在当前控制模型下按差速/滑移转向处理，可原地转向。
  min_turning_radius: 0.0

  # TEB 使用真实物理外轮廓；额外安全距离由 min_obstacle_dist 给出。
  footprint_model:
    type: "polygon"
    vertices:
      - [ 0.370,  0.295]
      - [ 0.370, -0.295]
      - [-0.300, -0.295]
      - [-0.300,  0.295]

  is_footprint_dynamic: false

  # -------------------------
  # Goal tolerance
  # -------------------------
  xy_goal_tolerance: 0.15
  yaw_goal_tolerance: 0.15
  free_goal_vel: false
  complete_global_plan: true

  # -------------------------
  # Obstacles
  # -------------------------
  include_costmap_obstacles: true

  # polygon footprint 已经包含车体尺寸，所以这里是车体边界到障碍物的期望净空。
  # 初版给 15 cm，优先解决之前“蹭障碍”的问题。
  min_obstacle_dist: 0.15

  # 在最小净空之外继续给障碍一个软代价区，让轨迹更早开始绕。
  inflation_dist: 0.35

  costmap_obstacles_behind_robot_dist: 1.0
  obstacle_poses_affected: 20

  # 当前 PointCloud2 只是写入 costmap，没有障碍物速度估计，初版不要假装它是动态轨迹。
  include_dynamic_obstacles: false

  # 第一阶段不启用 costmap_converter，先减少变量和 CPU 开销。
  costmap_converter_plugin: ""
  costmap_converter_spin_thread: true
  costmap_converter_rate: 5

  # -------------------------
  # Optimization
  # -------------------------
  no_inner_iterations: 5
  no_outer_iterations: 4
  optimization_activate: true
  optimization_verbose: false
  penalty_epsilon: 0.05

  weight_max_vel_x: 2.0
  weight_max_vel_theta: 1.0
  weight_acc_lim_x: 1.0
  weight_acc_lim_theta: 1.0

  # 非完整约束；差速底盘必须保持较高。
  weight_kinematics_nh: 1000.0

  # 强烈偏好前进；同时 max_vel_x_backwards=0，第一阶段不让 TEB 主动倒车。
  weight_kinematics_forward_drive: 1000.0
  weight_kinematics_turning_radius: 1.0

  # TEB 与 DWA 的一个关键区别：它显式优化时间。
  # 先用 2.0，不一开始过度追求速度。
  weight_optimaltime: 2.0
  weight_shortest_path: 0.0

  # 比默认更重视障碍安全距离，针对之前出现过的“蹭障碍”。
  weight_obstacle: 80.0
  weight_inflation: 0.50
  weight_dynamic_obstacle: 10.0

  # 第一阶段关闭 via point 约束，避免把 TEB 又强行拉回原 global plan。
  weight_viapoint: 0.0
  weight_adapt_factor: 2.0

  # -------------------------
  # Homotopy Class Planner
  # -------------------------
  # Phase 1：先 false，验证 TEB 基础链路、速度和碰撞距离。
  # Phase 2：改为 true，同时保持 max_number_classes=2，形成“左绕/右绕”候选。
  enable_homotopy_class_planning: false
  enable_multithreading: true
  simple_exploration: false
  max_number_classes: 2

  selection_cost_hysteresis: 1.0
  selection_obst_cost_scale: 1.0
  selection_alternative_time_cost: false

  # 控制探索计算量；当前 local costmap 为 6 m x 6 m。
  roadmap_graph_no_samples: 10
  roadmap_graph_area_width: 3.0
  h_signature_prescaler: 0.5
  h_signature_threshold: 0.1
  obstacle_keypoint_offset: 0.1
  obstacle_heading_threshold: 0.45
  visualize_hc_graph: true

  # -------------------------
  # TEB internal recovery
  # -------------------------
  shrink_horizon_backup: true
  shrink_horizon_min_duration: 10.0

  oscillation_recovery: true
  oscillation_v_eps: 0.10
  oscillation_omega_eps: 0.10
  oscillation_recovery_min_duration: 10.0
  oscillation_filter_duration: 10.0
```


**文件：`~/livox_fastlio/src/scout_navigation/launch/navigation_teb.launch`**

```xml
<launch>

  <arg name="map_name" default="scout_map_01" />
  <arg name="odom_topic" default="/scout/odom" />
  <arg name="cmd_vel_topic" default="/cmd_vel" />
  <arg name="obstacle_cloud_topic" default="/cloud_registered_body" />

  <arg
      name="map_dir"
      default="$(env HOME)/livox_fastlio/maps/$(arg map_name)" />

  <arg
      name="nav_map_yaml"
      default="$(arg map_dir)/map_raw.yaml" />

  <!-- 1. 导航专用 raw 静态地图 -->
  <node
      pkg="map_server"
      type="map_server"
      name="scout_navigation_map_server"
      args="$(arg nav_map_yaml)"
      output="screen">

    <remap from="map" to="/nav_static_map" />
    <remap from="map_metadata" to="/nav_static_map_metadata" />

  </node>

  <!-- 2. move_base + TEB -->
  <node
      pkg="move_base"
      type="move_base"
      name="move_base"
      output="screen">

    <rosparam
        command="load"
        file="$(find scout_navigation)/config/move_base_teb.yaml" />

    <rosparam
        command="load"
        ns="global_costmap"
        file="$(find scout_navigation)/config/costmap_common.yaml" />

    <rosparam
        command="load"
        ns="global_costmap"
        file="$(find scout_navigation)/config/global_costmap.yaml" />

    <rosparam
        command="load"
        ns="local_costmap"
        file="$(find scout_navigation)/config/costmap_common.yaml" />

    <rosparam
        command="load"
        ns="local_costmap"
        file="$(find scout_navigation)/config/local_costmap.yaml" />

    <rosparam
        command="load"
        file="$(find scout_navigation)/config/global_planner.yaml" />

    <rosparam
        command="load"
        file="$(find scout_navigation)/config/teb_local_planner.yaml" />

    <!-- 可通过 launch 参数覆盖 PointCloud2 障碍源 -->
    <param
        name="local_costmap/obstacle_layer/fastlio_cloud/topic"
        value="$(arg obstacle_cloud_topic)" />

    <remap from="odom" to="$(arg odom_topic)" />
    <remap from="cmd_vel" to="$(arg cmd_vel_topic)" />

  </node>

</launch>
```

### 13.2 导航算法关系

`move_base_teb.yaml` 明确：

```yaml
base_global_planner: global_planner/GlobalPlanner
base_local_planner: teb_local_planner/TebLocalPlannerROS
```

而 `global_planner.yaml`：

```yaml
GlobalPlanner:
  use_dijkstra: true
```

因此当前全局算法是 **Dijkstra**，局部轨迹优化与动态避障是 **TEB**。

### 13.3 为什么 TEB 使用 `/scout/odom`

`/fastlio_odom` 的 twist 是 0；TEB 需要真实速度反馈，所以：

```yaml
TebLocalPlannerROS:
  odom_topic: /scout/odom
```

位姿 TF 仍来自 FAST-LIO 链；轮速 odom 只提供控制层速度反馈。这两个职责不能混。

### 13.4 主导航文件完成后先不要编译

此时日志脚本和调试脚本还没有建立，而 `CMakeLists.txt` 已经声明要安装它们，因此先继续完成 13.5、13.6；本章所有文件齐全后统一编译一次。

### 13.5 建立导航规划日志采集与自动分析功能（开发调试必须保留）

这部分就是之前用于生成 `20260822_171130_teb_single_baseline` 这类规划调试日志的功能。它**不是另一个独立 ROS package**，而是集成在 `scout_navigation` 包中：

```text
scout_navigation/
├── launch/
│   └── nav_logging.launch
└── scripts/
    ├── nav_log_session.sh
    └── analyze_nav_bag.py
```

工作逻辑：

```text
启动 nav_logging.launch
        ↓
自动创建带时间戳的日志目录
        ↓
快照当前导航 config / launch / ROS 参数 / 节点与话题
        ↓
rosbag 录制 TF、目标、全局路径、TEB、costmap、cmd_vel、轮速 odom、点云
        ↓
测试结束后 Ctrl+C
        ↓
自动停止 rosbag
        ↓
自动运行 analyze_nav_bag.py
        ↓
生成 CSV + summary.txt
```

#### 13.5.1 建立 `nav_logging.launch`

**文件：`~/livox_fastlio/src/scout_navigation/launch/nav_logging.launch`**

```xml
<launch>

  <arg name="tag" default="nav_test" />

  <node
      pkg="scout_navigation"
      type="nav_log_session.sh"
      name="nav_log_session"
      output="screen"
      args="$(arg tag)" />

</launch>
```

#### 13.5.2 建立 `nav_log_session.sh`

**文件：`~/livox_fastlio/src/scout_navigation/scripts/nav_log_session.sh`**

下面给出当前工程已验证脚本的完整内容。脚本中还保留少量**历史日志兼容字段**，因此 topic 列表里仍可看到旧 planner 名称；在当前 TEB 运行时这些 topic 没有数据，不影响 rosbag。因为当前决定是不为了清理单文件而改动已验证脚本，本文保留它们，但**正式导航算法仍只有 TEB**。

```bash
#!/usr/bin/env bash
set -uo pipefail

TAG="${1:-nav_test}"
TAG="$(printf '%s' "${TAG}" | tr -cs 'A-Za-z0-9_.-' '_')"
PKG_DIR="$(rospack find scout_navigation)"
STAMP="$(date +%Y%m%d_%H%M%S)"
ROOT_DIR="${HOME}/livox_fastlio/logs/navigation"
RUN_DIR="${ROOT_DIR}/${STAMP}_${TAG}"
BAG_PID=""
FINISHED=0

if ! rosnode list >/dev/null 2>&1; then
  echo "[NAV_LOG][ERROR] ROS master is unavailable. Start localization/navigation first."
  exit 1
fi

mkdir -p "${RUN_DIR}/config_snapshot" "${RUN_DIR}/launch_snapshot"
printf '%s\n' "${RUN_DIR}" > "${ROOT_DIR}/LAST_RUN"

cp -a "${PKG_DIR}/config/." "${RUN_DIR}/config_snapshot/" 2>/dev/null || true
cp -a "${PKG_DIR}/launch/." "${RUN_DIR}/launch_snapshot/" 2>/dev/null || true
rosparam dump "${RUN_DIR}/move_base_params.yaml" /move_base 2>/dev/null || true
rosparam get /move_base/base_local_planner > "${RUN_DIR}/local_planner.txt" 2>/dev/null || true
rosparam get /scout_geometry > "${RUN_DIR}/scout_geometry.yaml" 2>/dev/null || true
rosnode list > "${RUN_DIR}/rosnode_list.txt" 2>/dev/null || true
rostopic list -v > "${RUN_DIR}/rostopic_list_verbose.txt" 2>/dev/null || true
rosservice list > "${RUN_DIR}/rosservice_list.txt" 2>/dev/null || true
date --iso-8601=seconds > "${RUN_DIR}/started_at.txt"
df -h "${HOME}" > "${RUN_DIR}/disk_before.txt" 2>/dev/null || true

if ! rosnode list 2>/dev/null | grep -qx '/move_base'; then
  echo "[NAV_LOG][WARN] /move_base is not currently visible. The bag will still start, but navigation data may be incomplete."
fi

# 同时保留 DWA 与 TEB 相关 topic。
# 当前使用哪一个 planner，就会有哪一组 topic 实际产生消息；另一组为空不影响 rosbag。
TOPICS=(
  /tf
  /tf_static
  /rosout_agg
  /cmd_vel
  /scout/odom
  /fastlio_odom
  /move_base_simple/goal
  /move_base/status
  /move_base/goal
  /move_base/cancel
  /move_base/feedback
  /move_base/result
  /move_base/GlobalPlanner/plan

  /move_base/DWAPlannerROS/global_plan
  /move_base/DWAPlannerROS/local_plan
  /move_base/DWAPlannerROS/trajectory_cloud
  /move_base/DWAPlannerROS/cost_cloud
  /move_base/DWAPlannerROS/parameter_updates

  /move_base/TebLocalPlannerROS/global_plan
  /move_base/TebLocalPlannerROS/local_plan
  /move_base/TebLocalPlannerROS/teb_poses
  /move_base/TebLocalPlannerROS/teb_markers
  /move_base/TebLocalPlannerROS/teb_feedback
  /move_base/TebLocalPlannerROS/obstacles
  /move_base/TebLocalPlannerROS/via_points
  /move_base/TebLocalPlannerROS/parameter_updates

  /move_base/local_costmap/obstacle_layer/parameter_updates
  /move_base/local_costmap/inflation_layer/parameter_updates
  /move_base/global_costmap/inflation_layer/parameter_updates
  /move_base/parameter_updates
  /move_base/local_costmap/costmap
  /move_base/local_costmap/costmap_updates
  /move_base/local_costmap/footprint
  /move_base/global_costmap/costmap
  /move_base/global_costmap/costmap_updates
  /nav_static_map
  /cloud_registered_body
)

finish_session() {
  if [[ "${FINISHED}" -eq 1 ]]; then
    return
  fi
  FINISHED=1
  trap - INT TERM EXIT

  echo
  echo "[NAV_LOG] stopping rosbag..."
  if [[ -n "${BAG_PID}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
    kill -INT "${BAG_PID}" 2>/dev/null || true
    wait "${BAG_PID}" 2>/dev/null || true
  fi

  date --iso-8601=seconds > "${RUN_DIR}/ended_at.txt"
  df -h "${HOME}" > "${RUN_DIR}/disk_after.txt" 2>/dev/null || true
  du -sh "${RUN_DIR}" > "${RUN_DIR}/run_size.txt" 2>/dev/null || true

  : > "${RUN_DIR}/rosbag_info.txt"
  shopt -s nullglob
  BAGS=("${RUN_DIR}"/*.bag)
  shopt -u nullglob
  if [[ "${#BAGS[@]}" -eq 0 ]]; then
    echo "[NAV_LOG][ERROR] no finalized .bag file found in ${RUN_DIR}" | tee -a "${RUN_DIR}/analysis_console.txt"
  else
    for bag in "${BAGS[@]}"; do
      echo "===== ${bag} =====" >> "${RUN_DIR}/rosbag_info.txt"
      rosbag info "${bag}" >> "${RUN_DIR}/rosbag_info.txt" 2>&1 || true
    done

    echo "[NAV_LOG] recording stopped"
    echo "[NAV_LOG] analyzing..."
    python3 "${PKG_DIR}/scripts/analyze_nav_bag.py" "${RUN_DIR}" 2>&1 | tee "${RUN_DIR}/analysis_console.txt" || true
  fi

  echo
  echo "[NAV_LOG] DONE"
  echo "[NAV_LOG] run_dir=${RUN_DIR}"
  echo "[NAV_LOG] summary=${RUN_DIR}/summary.txt"
}

trap finish_session INT TERM EXIT

echo "[NAV_LOG] run_dir=${RUN_DIR}"
echo "[NAV_LOG] DWA/TEB compatible navigation log enabled (including /cloud_registered_body)"
echo "[NAV_LOG] perform the test now; Ctrl+C this launch when finished"

rosbag record --lz4 --split --size=2048 -O "${RUN_DIR}/navigation" "${TOPICS[@]}" &
BAG_PID=$!
wait "${BAG_PID}" 2>/dev/null || true
finish_session
```

#### 13.5.3 建立 `analyze_nav_bag.py`

**文件：`~/livox_fastlio/src/scout_navigation/scripts/analyze_nav_bag.py`**

```python
#!/usr/bin/env python3
import argparse
import bisect
import csv
import glob
import math
import os
import statistics
import sys

import rosbag


DWA_LOCAL = "/move_base/DWAPlannerROS/local_plan"
DWA_TRAJ = "/move_base/DWAPlannerROS/trajectory_cloud"
TEB_LOCAL = "/move_base/TebLocalPlannerROS/local_plan"
TEB_POSES = "/move_base/TebLocalPlannerROS/teb_poses"


def bag_paths(path):
    path = os.path.abspath(os.path.expanduser(path))
    if os.path.isdir(path):
        files = sorted(glob.glob(os.path.join(path, "*.bag")))
    else:
        files = [path]
    files = [p for p in files if os.path.isfile(p)]
    if not files:
        raise FileNotFoundError("No .bag file found: " + path)
    return files


def write_csv(path, header, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def nearest_value(samples, ts, max_dt=0.25):
    if not samples:
        return None
    times = [x[0] for x in samples]
    i = bisect.bisect_left(times, ts)
    candidates = []
    if i < len(samples):
        candidates.append(samples[i])
    if i > 0:
        candidates.append(samples[i - 1])
    if not candidates:
        return None
    best = min(candidates, key=lambda x: abs(x[0] - ts))
    if abs(best[0] - ts) > max_dt:
        return None
    return best


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def path_length(poses):
    total = 0.0
    for i in range(1, len(poses)):
        p0 = poses[i - 1].pose.position
        p1 = poses[i].pose.position
        total += math.hypot(p1.x - p0.x, p1.y - p0.y)
    return total


def max_abs(rows, idx):
    return max((abs(x[idx]) for x in rows), default=0.0)


def max_gap(rows):
    if len(rows) < 2:
        return 0.0
    return max(rows[i][0] - rows[i - 1][0] for i in range(1, len(rows)))


def percentile(values, p):
    if not values:
        return 0.0
    values = sorted(values)
    if len(values) == 1:
        return values[0]
    k = (len(values) - 1) * p
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return values[int(k)]
    return values[f] * (c - k) + values[c] * (k - f)


def main():
    parser = argparse.ArgumentParser(description="Summarize Scout navigation rosbag (DWA/TEB compatible)")
    parser.add_argument("path", help="navigation run directory or .bag file")
    args = parser.parse_args()

    bags = bag_paths(args.path)
    if os.path.isdir(os.path.abspath(os.path.expanduser(args.path))):
        out_dir = os.path.abspath(os.path.expanduser(args.path))
    else:
        out_dir = os.path.dirname(bags[0])
    os.makedirs(out_dir, exist_ok=True)

    cmd = []
    odom = []
    local_plan = []
    dwa_traj_cloud = []
    teb_poses = []
    status_rows = []
    goals = []
    planner_fail_logs = []
    topic_counts = {}
    t_min = None
    t_max = None

    topics = [
        "/cmd_vel",
        "/scout/odom",
        DWA_LOCAL,
        DWA_TRAJ,
        TEB_LOCAL,
        TEB_POSES,
        "/move_base/status",
        "/move_base/goal",
        "/rosout_agg",
    ]

    fail_patterns = (
        "failed to find a valid plan",
        "cost functions discarded all candidates",
        "DWA planner failed to produce path",
        "failed to produce path",
        "no valid trajectories",
        "trajectory is not feasible",
        "infeasible trajectory",
        "timed-elastic-band detected an infeasible pose",
        "teblocalplannerros",
        "optimization failed",
        "diverged",
        "failed to obtain a local plan",
    )

    for path in bags:
        with rosbag.Bag(path, "r") as bag:
            for topic, msg, t in bag.read_messages(topics=topics):
                ts = t.to_sec()
                t_min = ts if t_min is None else min(t_min, ts)
                t_max = ts if t_max is None else max(t_max, ts)
                topic_counts[topic] = topic_counts.get(topic, 0) + 1

                if topic == "/cmd_vel":
                    cmd.append((ts, float(msg.linear.x), float(msg.angular.z)))
                elif topic == "/scout/odom":
                    odom.append((ts, float(msg.twist.twist.linear.x), float(msg.twist.twist.angular.z)))
                elif topic in (DWA_LOCAL, TEB_LOCAL):
                    local_plan.append((ts, topic, len(msg.poses), path_length(msg.poses)))
                elif topic == DWA_TRAJ:
                    dwa_traj_cloud.append((ts, int(msg.width) * int(msg.height)))
                elif topic == TEB_POSES:
                    teb_poses.append((ts, len(getattr(msg, "poses", []))))
                elif topic == "/move_base/status":
                    active = any(s.status == 1 for s in msg.status_list)
                    pending = any(s.status == 0 for s in msg.status_list)
                    status_rows.append((ts, int(active), int(pending), len(msg.status_list)))
                elif topic == "/move_base/goal":
                    pose = msg.goal.target_pose.pose
                    goals.append((
                        ts,
                        getattr(msg.goal_id, "id", ""),
                        float(pose.position.x),
                        float(pose.position.y),
                        float(yaw_from_quaternion(pose.orientation)),
                    ))
                elif topic == "/rosout_agg":
                    text = str(getattr(msg, "msg", ""))
                    lower = text.lower()
                    if any(pattern.lower() in lower for pattern in fail_patterns):
                        planner_fail_logs.append((ts, text))

    cmd.sort()
    odom.sort()
    local_plan.sort()
    dwa_traj_cloud.sort()
    teb_poses.sort()
    status_rows.sort()
    goals.sort()
    planner_fail_logs.sort()

    write_csv(os.path.join(out_dir, "cmd_vel.csv"), ["t", "linear_x", "angular_z"], cmd)
    write_csv(os.path.join(out_dir, "scout_odom_twist.csv"), ["t", "linear_x", "angular_z"], odom)
    write_csv(os.path.join(out_dir, "local_plan.csv"), ["t", "topic", "pose_count", "path_length_m"], local_plan)
    write_csv(os.path.join(out_dir, "trajectory_cloud.csv"), ["t", "point_count"], dwa_traj_cloud)
    write_csv(os.path.join(out_dir, "teb_poses.csv"), ["t", "pose_count"], teb_poses)
    write_csv(os.path.join(out_dir, "move_base_status.csv"), ["t", "active", "pending", "status_count"], status_rows)
    write_csv(os.path.join(out_dir, "goals.csv"), ["t", "goal_id", "x", "y", "yaw_rad"], goals)
    write_csv(os.path.join(out_dir, "planner_fail_logs.csv"), ["t", "message"], planner_fail_logs)
    # 兼容旧分析流程/文件名。
    write_csv(os.path.join(out_dir, "dwa_fail_logs.csv"), ["t", "message"], planner_fail_logs)

    lin_mismatch = 0
    lin_test = 0
    ang_mismatch = 0
    ang_test = 0
    for ts, vx, wz in cmd:
        nearest = nearest_value(odom, ts)
        if nearest is None:
            continue
        _, ovx, owz = nearest
        if abs(vx) >= 0.20:
            lin_test += 1
            if abs(ovx) < 0.05:
                lin_mismatch += 1
        if abs(wz) >= 0.20:
            ang_test += 1
            if abs(owz) < 0.05:
                ang_mismatch += 1

    empty_local = sum(1 for _, _, n, _ in local_plan if n == 0)
    local_lengths = [length for _, _, _, length in local_plan]
    local_pose_counts = [n for _, _, n, _ in local_plan]

    zero_dwa_traj = sum(1 for _, n in dwa_traj_cloud if n == 0)
    dwa_traj_counts = [n for _, n in dwa_traj_cloud]
    zero_teb_poses = sum(1 for _, n in teb_poses if n == 0)
    teb_pose_counts = [n for _, n in teb_poses]

    status_times = [x[0] for x in status_rows]

    def active_at(ts):
        if not status_rows:
            return False
        i = bisect.bisect_right(status_times, ts) - 1
        return i >= 0 and bool(status_rows[i][1])

    max_active_zero_cmd_sec = 0.0
    zero_start = None
    prev_t = None
    for ts, vx, wz in cmd:
        zero = abs(vx) < 0.01 and abs(wz) < 0.01 and active_at(ts)
        if zero:
            if zero_start is None or (prev_t is not None and ts - prev_t > 0.30):
                zero_start = ts
            max_active_zero_cmd_sec = max(max_active_zero_cmd_sec, ts - zero_start)
        else:
            zero_start = None
        prev_t = ts

    max_empty_local_plan_sec = 0.0
    empty_start = None
    prev_t = None
    for ts, _, n, _ in local_plan:
        if n == 0:
            if empty_start is None or (prev_t is not None and ts - prev_t > 0.30):
                empty_start = ts
            max_empty_local_plan_sec = max(max_empty_local_plan_sec, ts - empty_start)
        else:
            empty_start = None
        prev_t = ts

    duration = 0.0 if t_min is None or t_max is None else t_max - t_min

    nonzero_vx = [abs(vx) for _, vx, _ in cmd if abs(vx) >= 0.01]
    share_ge_030 = 0.0
    if nonzero_vx:
        share_ge_030 = sum(1 for x in nonzero_vx if x >= 0.30) / float(len(nonzero_vx))

    has_dwa = topic_counts.get(DWA_LOCAL, 0) > 0 or topic_counts.get(DWA_TRAJ, 0) > 0
    has_teb = topic_counts.get(TEB_LOCAL, 0) > 0 or topic_counts.get(TEB_POSES, 0) > 0
    if has_teb and not has_dwa:
        planner = "TEB"
    elif has_dwa and not has_teb:
        planner = "DWA"
    elif has_teb and has_dwa:
        planner = "MIXED/UNKNOWN"
    else:
        planner = "UNKNOWN"

    lines = []
    lines.append("Scout navigation log summary (DWA/TEB compatible)")
    lines.append("planner_detected: {}".format(planner))
    lines.append("bags: {}".format(len(bags)))
    lines.append("duration_sec: {:.3f}".format(duration))
    lines.append("goal_count: {}".format(len(goals)))
    for ts, goal_id, x, y, yaw in goals:
        rel = 0.0 if t_min is None else ts - t_min
        lines.append("  goal +{:.3f}s x={:.3f} y={:.3f} yaw_rad={:.3f} id={}".format(rel, x, y, yaw, goal_id))

    lines.append("cmd_vel_samples: {}".format(len(cmd)))
    lines.append("cmd_vel_rate_hz_overall: {:.3f}".format(len(cmd) / duration if duration > 0 else 0.0))
    lines.append("odom_samples: {}".format(len(odom)))
    lines.append("odom_rate_hz_overall: {:.3f}".format(len(odom) / duration if duration > 0 else 0.0))
    lines.append("max_abs_cmd_linear_x: {:.3f}".format(max_abs(cmd, 1)))
    lines.append("max_abs_cmd_angular_z: {:.3f}".format(max_abs(cmd, 2)))
    lines.append("max_abs_odom_linear_x: {:.3f}".format(max_abs(odom, 1)))
    lines.append("max_abs_odom_angular_z: {:.3f}".format(max_abs(odom, 2)))

    if nonzero_vx:
        lines.append("nonzero_cmd_linear_median: {:.3f}".format(statistics.median(nonzero_vx)))
        lines.append("nonzero_cmd_linear_p90: {:.3f}".format(percentile(nonzero_vx, 0.90)))
        lines.append("nonzero_cmd_linear_p95: {:.3f}".format(percentile(nonzero_vx, 0.95)))
        lines.append("nonzero_cmd_linear_share_ge_0.30: {:.3f}".format(share_ge_030))

    lines.append("local_plan_messages: {}".format(len(local_plan)))
    lines.append("empty_local_plan_messages: {}".format(empty_local))
    lines.append("max_local_plan_gap_sec: {:.3f}".format(max_gap([(r[0],) for r in local_plan])))
    lines.append("max_active_zero_cmd_sec: {:.3f}".format(max_active_zero_cmd_sec))
    lines.append("max_empty_local_plan_sec: {:.3f}".format(max_empty_local_plan_sec))
    if local_pose_counts:
        lines.append("local_plan_pose_count_median: {:.1f}".format(statistics.median(local_pose_counts)))
        lines.append("local_plan_pose_count_max: {}".format(max(local_pose_counts)))
    if local_lengths:
        lines.append("local_plan_length_m_median: {:.3f}".format(statistics.median(local_lengths)))
        lines.append("local_plan_length_m_p90: {:.3f}".format(percentile(local_lengths, 0.90)))
        lines.append("local_plan_length_m_max: {:.3f}".format(max(local_lengths)))

    lines.append("dwa_trajectory_cloud_messages: {}".format(len(dwa_traj_cloud)))
    lines.append("dwa_zero_point_trajectory_cloud_messages: {}".format(zero_dwa_traj))
    if dwa_traj_counts:
        lines.append("dwa_trajectory_cloud_points_median: {:.1f}".format(statistics.median(dwa_traj_counts)))
        lines.append("dwa_trajectory_cloud_points_max: {}".format(max(dwa_traj_counts)))

    lines.append("teb_poses_messages: {}".format(len(teb_poses)))
    lines.append("teb_zero_pose_messages: {}".format(zero_teb_poses))
    if teb_pose_counts:
        lines.append("teb_pose_count_median: {:.1f}".format(statistics.median(teb_pose_counts)))
        lines.append("teb_pose_count_max: {}".format(max(teb_pose_counts)))

    lines.append("planner_failed_logs: {}".format(len(planner_fail_logs)))
    lines.append("linear_cmd_without_odom_response: {}/{}".format(lin_mismatch, lin_test))
    lines.append("angular_cmd_without_odom_response: {}/{}".format(ang_mismatch, ang_test))

    lines.append("topic_counts:")
    for topic in sorted(topic_counts):
        lines.append("  {}: {}".format(topic, topic_counts[topic]))

    lines.append("")
    lines.append("Interpretation hints:")
    lines.append("- TEB: local_plan_length_m and teb_poses are the primary trajectory-shape indicators.")
    lines.append("- DWA: trajectory_cloud zero counts remain useful for legacy comparisons.")
    lines.append("- long max_active_zero_cmd_sec + planner failure logs/local-plan gaps: local planner/costmap feasibility issue is likely.")
    lines.append("- nonzero cmd_vel but high *_cmd_without_odom_response: chassis execution/dead-zone issue is likely.")
    lines.append("- low open-space linear command with TEB: inspect weight_optimaltime and velocity/acceleration limits before raising max_vel_x.")
    lines.append("- use goals.csv timestamps to split one session into individual goal tests.")

    summary_path = os.path.join(out_dir, "summary.txt")
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    print("\n".join(lines))
    print("[OK] summary: " + summary_path)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("[ERROR] {}".format(e), file=sys.stderr)
        sys.exit(1)
```

脚本至少会生成这些分析结果：

```text
cmd_vel.csv
scout_odom_twist.csv
local_plan.csv
trajectory_cloud.csv
teb_poses.csv
move_base_status.csv
goals.csv
planner_fail_logs.csv
summary.txt
analysis_console.txt
rosbag_info.txt
```

其中最常看的文件是：

```text
summary.txt              # 一次测试的总体统计
planner_fail_logs.csv    # TEB/move_base 失败信息
cmd_vel.csv              # 控制输出
scout_odom_twist.csv     # 底盘实际速度反馈
local_plan.csv           # 局部路径长度/点数
teb_poses.csv            # TEB 轨迹点数量
```

#### 13.5.4 设置日志脚本权限

```bash
chmod +x ~/livox_fastlio/src/scout_navigation/scripts/nav_log_session.sh
chmod +x ~/livox_fastlio/src/scout_navigation/scripts/analyze_nav_bag.py
```

此时先不要编译，因为 `CMakeLists.txt` 还同时安装下一节的 `global_plan_tester.py`。把 13.6 的脚本也建立并设置权限后，再统一编译。

#### 13.5.5 日志目录是自动生成的，不要手工建每次测试目录

脚本固定使用：

```text
~/livox_fastlio/logs/navigation/
```

每次启动自动生成：

```text
YYYYMMDD_HHMMSS_<tag>/
```

例如：

```text
~/livox_fastlio/logs/navigation/20260822_171130_teb_single_baseline/
```

目录内部典型结构：

```text
20260822_171130_teb_single_baseline/
├── navigation.bag                    # 小于 2 GiB 时通常只有一个
├── navigation_1.bag                  # 超过分片大小后继续产生
├── config_snapshot/                  # 当次 scout_navigation/config 快照
├── launch_snapshot/                  # 当次 scout_navigation/launch 快照
├── move_base_params.yaml             # /move_base 参数快照
├── local_planner.txt                 # 当前 base_local_planner
├── scout_geometry.yaml               # 当次安装几何参数
├── rosnode_list.txt
├── rostopic_list_verbose.txt
├── rosservice_list.txt
├── started_at.txt
├── ended_at.txt
├── disk_before.txt
├── disk_after.txt
├── run_size.txt
├── rosbag_info.txt
├── cmd_vel.csv
├── scout_odom_twist.csv
├── local_plan.csv
├── trajectory_cloud.csv
├── teb_poses.csv
├── move_base_status.csv
├── goals.csv
├── planner_fail_logs.csv
├── analysis_console.txt
└── summary.txt
```

同时维护：

```text
~/livox_fastlio/logs/navigation/LAST_RUN
```

它的内容就是最近一次测试目录的完整路径。查看最近一次测试：

```bash
cat ~/livox_fastlio/logs/navigation/LAST_RUN
```

直接查看最近一次分析：

```bash
cat "$(cat ~/livox_fastlio/logs/navigation/LAST_RUN)/summary.txt"
```

#### 13.5.6 正确的导航日志测试方法

先完成定位并启动导航，然后**在发送目标前**启动日志：

```bash
roslaunch scout_navigation nav_logging.launch tag:=teb_single_baseline
```

此时终端会显示：

```text
[NAV_LOG] run_dir=...
[NAV_LOG] perform the test now; Ctrl+C this launch when finished
```

然后再去 RViz 发送 2D Nav Goal。测试完成后，在**日志终端**按：

```text
Ctrl+C
```

不要直接杀进程。脚本需要收到退出信号，才能完成：

```text
停止 rosbag → rosbag info → 自动分析 → summary.txt
```

看到：

```text
[NAV_LOG] DONE
```

才表示这一组日志完整结束。

**磁盘注意：** 当前日志包含 `/cloud_registered_body` 和 costmap，数据量较大；rosbag 使用 LZ4 压缩并按 2048 MiB 自动分片。长时间测试前先执行：

```bash
df -h ~
```

脚本本身**不会自动生成 zip**。需要上传给别人分析时，可以对最近一次日志手工压缩：

```bash
RUN_DIR=$(cat ~/livox_fastlio/logs/navigation/LAST_RUN)
cd "$(dirname "$RUN_DIR")"
zip -r "$(basename "$RUN_DIR").zip" "$(basename "$RUN_DIR")"
```

这样得到的就是类似：

```text
20260822_171130_teb_single_baseline.zip
```

### 13.6 保留全局路径独立检查脚本（可选调试工具）

源码里还有 `global_plan_tester.py`。这个脚本本身不依赖 DWA，它只是调用 `/move_base/make_plan`，用 RViz 的 Publish Point 检查 Dijkstra 是否能生成全局路径。

**文件：`~/livox_fastlio/src/scout_navigation/scripts/global_plan_tester.py`**

```python
#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest


class GlobalPlanTester:
    def __init__(self):
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.tolerance = rospy.get_param("~tolerance", 0.15)
        self.service_name = rospy.get_param(
            "~make_plan_service",
            "/move_base/make_plan"
        )

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.plan_pub = rospy.Publisher(
            "/scout_global_plan_test",
            Path,
            queue_size=1,
            latch=True
        )

        rospy.loginfo("Waiting for %s ...", self.service_name)
        rospy.wait_for_service(self.service_name)
        self.make_plan = rospy.ServiceProxy(self.service_name, GetPlan)

        self.sub = rospy.Subscriber(
            "/clicked_point",
            PointStamped,
            self.point_callback,
            queue_size=1
        )

        rospy.loginfo(
            "Global plan tester ready. RViz -> Publish Point to request a plan."
        )

    def current_start_pose(self):
        tf_msg = self.tf_buffer.lookup_transform(
            self.global_frame,
            self.base_frame,
            rospy.Time(0),
            rospy.Duration(0.5)
        )

        start = PoseStamped()
        start.header.stamp = rospy.Time.now()
        start.header.frame_id = self.global_frame

        start.pose.position.x = tf_msg.transform.translation.x
        start.pose.position.y = tf_msg.transform.translation.y
        start.pose.position.z = tf_msg.transform.translation.z
        start.pose.orientation = tf_msg.transform.rotation

        return start

    def point_callback(self, msg):
        try:
            point_map = self.tf_buffer.transform(
                msg,
                self.global_frame,
                rospy.Duration(0.5)
            )

            start = self.current_start_pose()

            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = self.global_frame
            goal.pose.position.x = point_map.point.x
            goal.pose.position.y = point_map.point.y
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0

            req = GetPlanRequest()
            req.start = start
            req.goal = goal
            req.tolerance = self.tolerance

            result = self.make_plan(req)

            if not result.plan.poses:
                rospy.logwarn(
                    "No global plan: start=(%.2f, %.2f), goal=(%.2f, %.2f)",
                    start.pose.position.x,
                    start.pose.position.y,
                    goal.pose.position.x,
                    goal.pose.position.y
                )
                return

            # move_base/make_plan 在 ROS Noetic 中可能返回
            # plan.poses 有效，但 nav_msgs/Path 顶层 header 为空。
            # RViz 的 Path Display 需要合法的 frame_id。
            plan = result.plan
            plan.header.stamp = rospy.Time.now()
            plan.header.frame_id = self.global_frame

            # 保险处理：确保每个 PoseStamped 也有合法 frame_id。
            for pose in plan.poses:
                if not pose.header.frame_id:
                    pose.header.frame_id = self.global_frame

            self.plan_pub.publish(plan)

            rospy.loginfo(
                "Global plan OK: %d poses, start=(%.2f, %.2f), goal=(%.2f, %.2f)",
                len(plan.poses),
                start.pose.position.x,
                start.pose.position.y,
                goal.pose.position.x,
                goal.pose.position.y
            )

        except Exception as e:
            rospy.logerr("Global plan test failed: %s", str(e))


def main():
    rospy.init_node("scout_global_plan_tester")
    GlobalPlanTester()
    rospy.spin()


if __name__ == "__main__":
    main()
```

设置权限：

```bash
chmod +x ~/livox_fastlio/src/scout_navigation/scripts/global_plan_tester.py
```

到这里 `scout_navigation/CMakeLists.txt` 中列出的三个脚本都已经存在，现在统一编译：

```bash
cd ~/livox_fastlio
catkin_make -j1
source devel/setup.bash
```

在当前 `navigation_teb.launch` 已经启动、但还没有发送导航目标时，可以另开终端：

```bash
rosrun scout_navigation global_plan_tester.py
```

RViz 中使用 **Publish Point** 点击目标点，脚本会调用 `/move_base/make_plan`，并把结果发布到：

```text
/scout_global_plan_test
```

RViz 添加 `Path` 显示这个话题即可检查 Dijkstra 全局路径。

**不再复现旧的 `global_planning_test.launch`。** 源码中的旧版本 launch 仍引用历史 DWA 配置，既然本工程已经确定跳过 DWA，它只属于早期阶段调试残留，不应进入新的开发步骤。

## 14. 最终正式运行顺序

### 14.1 建图时

```bash
roslaunch scout_system_bringup scout_mapping.launch
```

完成扫描后 `Ctrl+C`，然后：

```bash
rosrun scout_map_tools finalize_map.py scout_map_01 --replace-raw
```

这里**不需要提前手工建立地图文件夹**。`finalize_map.py` 内部会自动执行等价于 `mkdir -p` 的操作，创建：

```text
~/livox_fastlio/maps/scout_map_01/
```

并生成/归档：

```text
raw_camera_init.pcd
public_map.pcd
map_raw.pgm
map_raw.yaml
map.pgm
map.yaml
map_metadata.yaml
```

验证：

```bash
ls -lh ~/livox_fastlio/maps/scout_map_01
```

### 14.2 定位 + 导航时

终端 1，启动定位链：

```bash
roslaunch scout_system_bringup scout_localization.launch map_name:=scout_map_01
```

RViz 发送 `/initialpose`，等待 NDT 成功并确认当前点云贴合历史地图。

终端 2，启动导航：

```bash
roslaunch scout_navigation navigation_teb.launch map_name:=scout_map_01
```

开发调试阶段建议终端 3 在**发目标之前**启动规划日志：

```bash
roslaunch scout_navigation nav_logging.launch tag:=teb_test
```

确认三个终端都正常后，再在 RViz 发 2D Nav Goal。

测试完成后先在日志终端 `Ctrl+C`，等待 `summary.txt` 生成。定位和导航进程是否继续运行取决于是否还要做下一组测试；下一组测试重新启动一次 `nav_logging.launch` 并换一个有意义的 `tag`，例如：

```bash
roslaunch scout_navigation nav_logging.launch tag:=teb_open_space
roslaunch scout_navigation nav_logging.launch tag:=teb_narrow_passage
roslaunch scout_navigation nav_logging.launch tag:=teb_obstacle_avoidance
```

**不要在全局定位还没确认正确时直接发导航目标；也不要发完目标后才开始录日志，否则会丢失规划建立阶段的数据。**

## 15. 每次改动时应该改哪里

### 15.1 雷达整机安装位置/45°安装角变化

至少检查两处：

```text
scout_system_bringup/config/scout_geometry.yaml
scout_tf_manager/config/extrinsics.yaml
```

两者描述同一个物理安装关系的不同用途，当前必须保持一致。然后：

1. 清理并重新编译不是必须的（YAML 运行时加载），但要重启相关 launch；
2. 旧 `public_map.pcd`、PGM/YAML 与旧安装几何绑定，应该重新执行 `finalize_map.py`；
3. 如果只是几何值改正、原始 FAST-LIO `raw_camera_init.pcd` 仍有效，可以保留 raw PCD，再重新生成 public/2D 地图；
4. 如果雷达物理位置发生较大变化并重新采集环境，重新建图最稳妥。

### 15.2 Mid-360 自身 LiDAR-IMU 外参变化

改 `FAST_LIO/config/mid360.yaml` 中 `mapping/extrinsic_T`、`extrinsic_R`。这不是 Scout 车体安装外参。改错这里会直接破坏 FAST-LIO 本身的运动估计。

### 15.3 更换雷达或主机 IP

更换 Mid-360 时先读取**新雷达二维码下方 S/N 的末两位 `XX`**，按 `192.168.1.1XX` 得到该雷达默认 IP，再改 `livox_ros_driver2/config/MID360_config.json`。例如末两位 `20` 对应 `192.168.1.120`。

主机 IP 与雷达 IP 是两回事；当前主机仍使用 `192.168.1.5/24`。先 `ping <雷达IP>`，再验证 `/livox/lidar`、`/livox/imu`，最后才启动 FAST-LIO。

### 15.4 修改车体外轮廓

同步修改：

```text
scout_navigation/config/costmap_common.yaml -> footprint
scout_navigation/config/teb_local_planner.yaml -> footprint_model.vertices
```

不要只改一个，否则 costmap 判定和 TEB 自己的碰撞模型会不一致。

### 15.5 修改障碍安全距离

优先理解三个量：

- `costmap inflation_radius=0.45`：代价地图的膨胀影响范围；
- `TEB min_obstacle_dist=0.15`：车体多边形边界到障碍的期望最小净空；
- `TEB inflation_dist=0.35`：在最小净空外继续施加软障碍代价。

不要把三个值都机械设成同一个数。

## 16. 分层调试顺序（必须从底到顶）

发生“车不走/路径不对/碰障碍”时，按下面顺序，不要一上来调 TEB：

```text
1. CAN / Scout base
   ↓
2. Mid-360 /livox/lidar + /livox/imu
   ↓
3. FAST-LIO camera_init -> body + cloud_registered_body
   ↓
4. 静态安装 TF：odom->camera_init、body->base_link
   ↓
5. odom->base_link 是否合理
   ↓
6. /fastlio_odom 与 /cloud_registered_base
   ↓
7. NDT：map->odom 是否正确、点云是否贴地图
   ↓
8. global costmap / Dijkstra 全局路径
   ↓
9. local costmap 是否正确标障碍
   ↓
10. TEB 轨迹与 /cmd_vel
   ↓
11. 用 nav_logging 保存完整测试证据并查看 summary.txt
```

常用检查命令：

```bash
rostopic hz /scout/odom
rostopic hz /livox/lidar
rostopic hz /livox/imu
rostopic hz /cloud_registered_body
rostopic hz /cloud_registered_base
rostopic hz /fastlio_odom
rostopic hz /cmd_vel
rosrun tf tf_echo odom base_link
rosrun tf tf_echo map odom
rosrun tf view_frames
```

## 17. 当前工程最重要的注意事项

1. **同一个 TF 只能有一个发布者。** Scout 底盘必须 `pub_tf=false`；不要恢复旧的 odom->base_link publisher。
2. **不要把 `/fastlio_odom` 给 TEB 当速度反馈。** 它当前 twist=0。
3. **`map->odom` 存在不等于已经重定位。** 启动时可能先发布 Identity，必须 `/initialpose` + NDT 成功。
4. **NDT 当前没有质量门控。** 初始位姿离真实位置太远时可能给出错误修正。
5. **建图 PCD 和导航地图都与安装几何绑定。** 45°、x、z 改动后不能只改 TF 而继续无条件沿用所有旧地图。
6. **`map_raw.yaml` 才是当前 navigation_teb.launch 的静态地图。** `map.yaml` 是预膨胀版本，不要替换后又叠加 costmap inflation。
7. **第一次/干净编译优先 `catkin_make -j1`。** 成功后才考虑并行。
8. **工程完成后不要再运行 `livox_ros_driver2/build.sh ROS1`。** 它会清空整个工作区的 build/devel/install。
9. **大角度掉头、目标贴障碍物导致局部规划困难是当前已知边界条件。** 现阶段不作为基础链路故障处理。
10. **DWA 已不属于正式方案。** 当前开发、参数说明和启动流程全部以 TEB 为准。导航日志脚本中若看到少量 DWA 字样，只是历史日志兼容代码，不代表系统仍使用 DWA。
11. **导航问题尽量先录日志再改参数。** 每次参数基线测试用不同 `tag`，日志会自动保存当时的 config/launch/ROS 参数，避免事后无法还原实验条件。
12. **地图目录和日志目录都在工作区根目录，不在 `src/` 内。** 地图由 `finalize_map.py` 自动创建，日志由 `nav_log_session.sh` 自动创建。

## 18. 当前建议保留的最终工程结构

不要只看 `src/`。当前工程还有两个非常重要的运行数据目录：`maps/` 和 `logs/`。

```text
~/livox_fastlio/
├── src/
│   ├── FAST_LIO/
│   ├── fast_lio_localization/
│   ├── livox_ros_driver2/
│   ├── scout_ros/
│   ├── ugv_sdk/
│   ├── scout_tf_manager/
│   ├── scout_pose_adapter/
│   ├── scout_cloud_adapter/
│   ├── scout_map_tools/
│   ├── scout_navigation/
│   │   ├── config/
│   │   ├── launch/
│   │   │   ├── navigation_teb.launch
│   │   │   └── nav_logging.launch
│   │   └── scripts/
│   │       ├── nav_log_session.sh
│   │       ├── analyze_nav_bag.py
│   │       └── global_plan_tester.py
│   └── scout_system_bringup/
│       └── launch/
│           └── d435i.launch
│
├── maps/
│   └── scout_map_01/
│       ├── raw_camera_init.pcd
│       ├── public_map.pcd
│       ├── map_raw.pgm
│       ├── map_raw.yaml
│       ├── map.pgm
│       ├── map.yaml
│       └── map_metadata.yaml
│
└── logs/
    └── navigation/
        ├── LAST_RUN
        └── YYYYMMDD_HHMMSS_<tag>/
            ├── navigation*.bag
            ├── config_snapshot/
            ├── launch_snapshot/
            ├── summary.txt
            └── *.csv
```

`Livox-SDK2` 安装到 `/usr/local` 后可不保留源码。是否保留 `scout_description` 取决于你是否还保留上游 `scout_mini_robot_base.launch` 中的 `$(find scout_description)` 参数；本文不要求为了精简而改这个单文件引用。

以下源码中的早期文件不进入本开发流程：旧 DWA 配置/launch、旧 `scout_system.launch`、以及仍与旧 DWA 配置耦合的 `global_planning_test.launch`。它们即使暂时留在磁盘上，也不属于当前正式系统。

## 19. 最终验收清单

按顺序全部通过才算工程复现成功：

- [ ] `candump can0` 有底盘数据。
- [ ] `/scout/odom` 稳定，底盘 `pub_tf=false`。
- [ ] 按 Mid-360 二维码下方 S/N 末两位确认雷达 IP，当前设备为 `192.168.1.120`。
- [ ] 工控机有线网卡为 `192.168.1.5/24`，能够 `ping` 当前雷达 IP。
- [ ] `/livox/lidar`、`/livox/imu` 稳定。
- [ ] FAST-LIO 正常发布 `camera_init -> body`。
- [ ] `/cloud_registered_body` 正常。
- [ ] `odom -> camera_init -> body -> base_link` 连通。
- [ ] 静止启动时 `odom -> base_link` 近似合理，直行主要沿 +X。
- [ ] `/fastlio_odom` 正常但明确只用于定位。
- [ ] `/cloud_registered_base` frame 为 `base_link`。
- [ ] `finalize_map.py` 自动创建 `~/livox_fastlio/maps/<map_name>/` 并生成完整地图文件。
- [ ] 定位模式加载 `/map_cloud` 和 `/map_2d`。
- [ ] RViz 发布 `/initialpose` 后出现 `NDT Relocated`。
- [ ] `map -> odom` 修正后当前点云与历史地图对齐。
- [ ] move_base 使用 `global_planner/GlobalPlanner`。
- [ ] `GlobalPlanner/use_dijkstra=true`。
- [ ] move_base 使用 `teb_local_planner/TebLocalPlannerROS`。
- [ ] local costmap 能用 `/cloud_registered_body` 标记/清除障碍。
- [ ] TEB 输出 `/cmd_vel`，Scout 能按目标行驶并绕开障碍。
- [ ] `roslaunch scout_navigation nav_logging.launch tag:=teb_test` 能自动创建日志目录。
- [ ] 日志停止后能生成 `summary.txt`、`planner_fail_logs.csv`、`cmd_vel.csv` 等分析结果。
- [ ] `~/livox_fastlio/logs/navigation/LAST_RUN` 能定位最近一次测试目录。


## 20. 集成 Intel RealSense D435i：Jetson 驱动、ROS、TF 与一键启动

本节记录当前已经实际完成并验证的 D435i 开发过程。D435i 当前用途为：

```text
目标识别
视觉检测
深度测距
深度避障
```

它不参与 FAST-LIO/NDT 主定位，因此不做高精度 LiDAR-Camera 联合标定。当前采用机械测量近似外参，直接发布 `base_link -> camera_link`。

### 20.1 当前硬件/系统基线

本机环境：

```text
Ubuntu 20.04.6 LTS
ROS Noetic
aarch64
NVIDIA Jetson
kernel 5.10.216-tegra
```

当前已验证 RealSense 组合：

```text
librealsense 2.50.0
realsense-ros 2.3.2
D400 firmware 05.13.00.50
```

Jetson 使用 `RSUSB backend`，不依赖给 `5.10.216-tegra` 打 RealSense UVC kernel patch。

### 20.2 先确认 USB3，不要把驱动问题和线材问题混在一起

D435i 即使没有安装 librealsense，Linux USB 层也应能枚举设备。

检查：

```bash
lsusb
```

```bash
lsusb -t
```

正式运行目标：

```text
D435i -> 5000M
```

如果显示：

```text
480M
```

说明只协商到 USB2。当前实测最终使用 D435i 原装线后恢复到 `5000M`。如果长期停留在 USB2，RGB + Depth + IMU 同时运行可能受到带宽限制。

### 20.3 源码安装 librealsense 2.50.0（Jetson RSUSB）

安装依赖：

```bash
sudo apt update
sudo apt install -y git cmake build-essential libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev libglfw3-dev
```

建立源码目录：

```bash
mkdir -p ~/software
cd ~/software
```

固定下载 2.50.0：

```bash
git clone --depth 1 --branch v2.50.0 https://github.com/IntelRealSense/librealsense.git
```

安装 udev rule：

```bash
cd ~/software/librealsense
sudo ./scripts/setup_udev_rules.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

建立 build：

```bash
mkdir -p ~/software/librealsense/build
cd ~/software/librealsense/build
```

Jetson 关键配置：

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=ON -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
```

编译：

```bash
make -j4
```

安装：

```bash
sudo make install
sudo ldconfig
```

验证：

```bash
pkg-config --modversion realsense2
```

当前应为：

```text
2.50.0
```

### 20.4 如果相机进入 `D4XX Recovery`

本次开发中实际遇到过：

```text
Name       : Intel RealSense D4XX Recovery
Product Id : 0ADB
```

此时相机不是正常 D435i 工作模式。不要继续调 ROS。

先列设备：

```bash
sudo rs-fw-update -l
```

当前工程恢复到匹配的：

```text
Signed_Image_UVC_5_13_0_50.bin
```

Recovery 模式刷写：

```bash
sudo rs-fw-update -r -f Signed_Image_UVC_5_13_0_50.bin
```

刷写期间不要拔线、断电或关闭终端。完成后重新拔插 D435i，再执行：

```bash
rs-enumerate-devices
```

正常目标：

```text
Name                : Intel RealSense D435I
Firmware Version    : 05.13.00.50
Product Id          : 0B3A
Usb Type Descriptor : 3.x
```

> 当前工程已经固定旧版 ROS1/SDK 组合，不要因为 `realsense-viewer` 右上角提示 “Firmware Update Recommended” 就直接升级到最新固件。

### 20.5 librealsense 层验证

执行：

```bash
realsense-viewer
```

依次验证：

```text
Stereo Module  -> Depth
RGB Camera     -> Color
Motion Module  -> Gyro / Accel
```

IMU 不会像 RGB/Depth 一样显示普通“图像”，判断标准是 Gyro/Accel 数据持续变化。

最终可以同时打开 RGB、Depth、Motion Module 运行一段时间，确认没有持续出现：

```text
Frame didn't arrive
RS2_USB_STATUS_IO
USB transfer error
```

### 20.6 建立独立 `realsense_ws`，安装 ROS1 wrapper 2.3.2

不要把 RealSense wrapper 强行塞入 `livox_fastlio/src`。当前使用独立工作区：

```bash
mkdir -p ~/realsense_ws/src
cd ~/realsense_ws/src
```

下载固定 tag：

```bash
git clone --depth 1 --branch 2.3.2 https://github.com/IntelRealSense/realsense-ros.git
```

出现 detached HEAD 是正常的，因为 `2.3.2` 是 tag。

安装缺失依赖：

```bash
sudo apt install -y ros-noetic-ddynamic-reconfigure
```

如果本机从未初始化 rosdep：

```bash
sudo rosdep init
rosdep update
```

再安装其余 ROS 依赖，但跳过系统版 librealsense，避免覆盖源码安装的 2.50.0：

```bash
cd ~/realsense_ws
rosdep install --from-paths src --ignore-src -r -y --skip-keys=librealsense2
```

编译：

```bash
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
```

### 20.7 Jetson/Noetic 的 OpenCV `undefined symbol` 修复

本次实际遇到：

```text
undefined symbol: _ZN2cv3MatC1Ev
```

即 `realsense2_camera_manager` 启动后因 OpenCV 没有正确显式链接而退出。

修改：

```text
~/realsense_ws/src/realsense-ros/realsense2_camera/CMakeLists.txt
```

在 `find_package(catkin REQUIRED COMPONENTS ...)` 前加入：

```cmake
find_package(OpenCV REQUIRED)
```

在 `include_directories(...)` 中加入：

```cmake
${OpenCV_INCLUDE_DIRS}
```

在 `target_link_libraries(${PROJECT_NAME} ...)` 中加入：

```cmake
${OpenCV_LIBRARIES}
```

修改后建议干净编译：

```bash
source /opt/ros/noetic/setup.bash
cd ~/realsense_ws
rm -rf build devel
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
```

### 20.8 两个 catkin 工作区的 `.bashrc` 叠加方式

当前存在：

```text
~/livox_fastlio
~/realsense_ws
```

如果简单写：

```bash
source ~/livox_fastlio/devel/setup.bash
source ~/realsense_ws/devel/setup.bash
```

本次实际出现过：新终端中 `realsense_ws` 最后一次 source 后，`rospack` 找不到 `scout_system_bringup`；手工再 source `livox_fastlio` 又恢复。

最终建议 `~/.bashrc`：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_fastlio/devel/setup.bash
source ~/realsense_ws/devel/setup.bash --extend
```

立即生效：

```bash
source ~/.bashrc
```

验证：

```bash
rospack find scout_system_bringup
```

```bash
rospack find realsense2_camera
```

两者必须同时找到。

### 20.9 D435i 安装外参

当前机械关系采用简单近似：

```text
原雷达安装位置相对 base_link：
x ≈ +0.25 m
y ≈  0.00 m
z ≈ +0.20 m

D435i 相对雷达安装点：
再向前约 0.02 m
再向下约 0.10 m
左右基本一致
```

因此直接写死：

```text
base_link -> camera_link

x = +0.27 m
y =  0.00 m
z = +0.10 m
```

D435i 相对车体倒装、镜头仍朝车头，因此：

```text
yaw   = 0
pitch = 0
roll  = π
```

即：

```text
roll = 3.14159265 rad
```

当前用途不要求毫米级联合标定。如果后续机械支架变化，直接重新测量并修改这组固定数值。

### 20.10 新建 `scout_system_bringup/launch/d435i.launch`

文件：

```text
~/livox_fastlio/src/scout_system_bringup/launch/d435i.launch
```

完整内容：

```xml
<launch>

    <!-- ====================================================== -->
    <!-- Intel RealSense D435i                                  -->
    <!-- ====================================================== -->

    <include file="$(find realsense2_camera)/launch/rs_camera.launch">

        <!-- RGB -->
        <arg name="enable_color" value="true"/>

        <!-- Depth -->
        <arg name="enable_depth" value="true"/>

        <!-- IMU -->
        <arg name="enable_accel" value="true"/>
        <arg name="enable_gyro" value="true"/>

        <!-- Gyro + Accel -> /camera/imu -->
        <arg name="unite_imu_method" value="linear_interpolation"/>

        <!-- 深度对齐到 RGB -->
        <arg name="align_depth" value="true"/>

        <!-- 当前不生成 RealSense PointCloud2，降低 Jetson 负载 -->
        <arg name="enable_pointcloud" value="false"/>

        <!-- RealSense 自己发布 camera_link 以下内部 TF -->
        <arg name="publish_tf" value="true"/>
        <arg name="tf_publish_rate" value="0"/>

    </include>

    <!--
      base_link -> camera_link

      参数顺序：
      x y z yaw pitch roll parent child
    -->
    <node pkg="tf2_ros"
          type="static_transform_publisher"
          name="base_to_d435i"
          args="0.27 0.00 0.10 0 0 3.14159265 base_link camera_link"/>

</launch>
```

这里 TF 职责必须保持：

```text
scout_system_bringup/d435i.launch：
base_link -> camera_link

realsense2_camera：
camera_link -> camera_depth_frame / camera_color_frame / optical frames / IMU frames
```

**不要再手工发布 RealSense 内部 optical TF。**

### 20.11 是否 include 到总 launch

`d435i.launch` 本身已经是 D435i 统一入口。

单独测试：

```bash
roslaunch scout_system_bringup d435i.launch
```

如果希望建图或定位时自动带起 D435i，可以在对应主 launch 的 `<launch>...</launch>` 内加入：

```xml
<include file="$(find scout_system_bringup)/launch/d435i.launch"/>
```

例如按实际需求加入：

```text
scout_mapping.launch
scout_localization.launch
```

但必须遵守：

```text
如果主 launch 已经 include d435i.launch
→ 不要再手工启动第二个 d435i.launch
```

否则同一台 USB 相机会被重复打开。

### 20.12 ROS 话题验收

启动：

```bash
roslaunch scout_system_bringup d435i.launch
```

检查 RGB：

```bash
rostopic hz /camera/color/image_raw
```

检查原始深度：

```bash
rostopic hz /camera/depth/image_rect_raw
```

检查 RGB 对齐深度：

```bash
rostopic hz /camera/aligned_depth_to_color/image_raw
```

检查 IMU：

```bash
rostopic hz /camera/gyro/sample
rostopic hz /camera/accel/sample
rostopic hz /camera/imu
```

图像查看：

```bash
rqt_image_view
```

目标识别 + 深度测距推荐组合：

```text
RGB   : /camera/color/image_raw
Depth : /camera/aligned_depth_to_color/image_raw
```

### 20.13 TF 验收

首先：

```bash
rosrun tf tf_echo base_link camera_link
```

应接近：

```text
Translation = [0.27, 0.00, 0.10]
RPY         = [3.14159, 0, 0]
```

再检查内部 RGB optical frame：

```bash
rosrun tf tf_echo camera_link camera_color_optical_frame
```

最后检查整条链：

```bash
rosrun tf tf_echo base_link camera_color_optical_frame
```

当前 TF 树新增分支：

```text
base_link
 └── camera_link
      ├── camera_depth_frame
      │    └── camera_depth_optical_frame
      ├── camera_color_frame
      │    └── camera_color_optical_frame
      ├── camera_gyro_frame
      └── camera_accel_frame
```

### 20.14 `camera_color_optical_frame does not exist` 的处理

RViz 如果报：

```text
For frame [camera_color_optical_frame]:
Frame[camera_color_optical_frame] does not exist
```

不要修改 `base_link -> camera_link` 平移参数。

先确认 `d435i.launch` 内：

```xml
<arg name="publish_tf" value="true"/>
<arg name="tf_publish_rate" value="0"/>
```

然后：

```bash
rosrun tf tf_echo camera_link camera_color_optical_frame
```

再：

```bash
rosrun tf tf_echo base_link camera_color_optical_frame
```

只有这条内部 TF 链完整，RViz 才能把 RGB/Depth 消息放入整车 TF tree。

### 20.15 为什么默认 `enable_pointcloud=false`

当前已有 Mid-360 + FAST-LIO 负责主要三维点云。D435i 当前只承担 RGB + 深度感知，所以默认：

```text
enable_pointcloud=false
```

可以减少：

```text
Jetson CPU
内存带宽
ROS PointCloud2 数据流量
```

目标识别后直接在 `/camera/aligned_depth_to_color/image_raw` 中查询深度即可。

### 20.16 D435i 分层调试顺序

D435i 出问题时固定按：

```text
1. lsusb / lsusb -t
   ↓
2. 是否 5000M
   ↓
3. rs-enumerate-devices 是否为 D435I，而不是 D4XX Recovery
   ↓
4. realsense-viewer：Depth / RGB / Gyro / Accel
   ↓
5. realsense2_camera ROS 节点是否存活
   ↓
6. RGB / Depth / IMU topic 是否有频率
   ↓
7. camera_link 以下内部 TF 是否存在
   ↓
8. base_link -> camera_link 是否存在
   ↓
9. 最后才检查 RViz / 视觉算法
```

不要一看到 RViz 没图就先改外参。

### 20.17 D435i 当前常见问题速查

| 现象 | 根因优先级 | 处理 |
|---|---|---|
| `lsusb` 都看不到 D435i | 线、接口、供电/物理连接 | 先换原装 USB3 线和接口，不是 ROS 驱动问题。 |
| `lsusb -t` 只有 `480M` | USB2 协商 | 换原装 USB3 线/USB3 口，目标 `5000M`。 |
| `rs-enumerate-devices` 为 `D4XX Recovery` | 固件恢复模式 | 先恢复匹配固件，不启动 ROS。 |
| `RS2_USB_STATUS_IO` | USB控制传输、Recovery/固件状态、占用等 | 先确认不是 Recovery、USB=5000M，再排查 Viewer/进程。 |
| `ddynamic_reconfigure` 找不到 | ROS依赖缺失 | `sudo apt install ros-noetic-ddynamic-reconfigure`。 |
| `rosdep installation has not been initialized` | rosdep 未初始化 | `sudo rosdep init`，然后普通用户执行 `rosdep update`。 |
| `_ZN2cv3MatC1Ev` undefined symbol | Jetson 上 OpenCV 未显式链接 | 按 20.7 修改 RealSense CMakeLists 并干净编译。 |
| `scout_system_bringup` package not found，但手工 source 后恢复 | `realsense_ws` 覆盖主工作区环境 | `.bashrc` 使用 `realsense_ws/devel/setup.bash --extend`。 |
| RGB topic 有数据但 RViz 报 optical frame 不存在 | RealSense 内部 TF 未发布 | `publish_tf=true`，检查 `camera_link -> camera_color_optical_frame`。 |

### 20.18 工程结构新增内容

主工作区新增：

```text
~/livox_fastlio/src/scout_system_bringup/
└── launch/
    └── d435i.launch
```

RealSense 保持独立工作区：

```text
~/realsense_ws/
├── src/
│   └── realsense-ros/
├── build/
└── devel/
```

librealsense 源码/构建目录：

```text
~/software/librealsense/
```

其运行库安装到 `/usr/local` 后，ROS 运行时不要求一直停留在源码目录。


## 21. 一句话总结

这套工程的复现主线就是：**先按雷达 S/N 正确配置 Mid-360 网络并让 FAST-LIO 稳定，再用安装几何构造 `odom -> base_link`，把位姿/点云适配给 NDT 得到 `map -> odom`，生成独立地图目录，最后叠加 `Dijkstra + TEB` 导航；每次导航调试用 `nav_logging` 保存完整证据，再按层级自底向上排查。**
