# Scout Mini 自主导航机器人使用手册

> **2026-08-25 一键建图正式操作：** 先执行
> `roslaunch scout_system_bringup scout_mapping.launch map_name:=地图名`。低速完成
> 环境扫描后执行 `rosservice call /finish_mapping`。该服务自动保存过滤点云并
> 生成定位 PCD、原始栅格地图、导航栅格地图和元数据；看到 `success: True` 后
> 才能停止建图 launch。结果位于 `~/livox_fastlio/maps/<地图名>/`。无需再手动
> 执行 `finalize_map.py`。FAST-LIO 原生 PCD 保存已关闭，本文后续所有
> `FAST_LIO/PCD/scans.pcd` 及退出时自动保存的描述均为旧流程。

> 适用工程：Scout Mini + Livox Mid-360 + FAST-LIO + NDT + Dijkstra + TEB + Intel RealSense D435i  
> 系统环境：Ubuntu 20.04 + ROS Noetic  
> 工作区：`~/livox_fastlio`  
> 本文面向机器人使用者，不涉及源码开发和参数设计。

---

## 1. 机器人当前可用功能

当前机器人正式支持以下功能：

| 功能 | 用途 | 主要入口 |
|---|---|---|
| 建图 | 使用 Mid-360 + FAST-LIO 建立环境点云地图 | `scout_mapping.launch` |
| 地图归档 | 把 FAST-LIO 点云转换为定位/导航需要的地图文件 | `finalize_map.py` |
| 全局重定位 | 在已有地图中确定机器人当前位置 | `scout_localization.launch` |
| 自主导航 | Dijkstra 全局规划 + TEB 局部规划/避障 | `navigation_teb.launch` |
| 导航日志 | 保存一次完整规划测试并自动生成分析结果 | `nav_logging.launch` |
| D435i视觉/深度 | RGB目标识别、深度测距、深度避障、视觉检测 | `scout_system_bringup/d435i.launch` |

当前正式导航算法：

```text
全局路径规划：GlobalPlanner + Dijkstra
局部路径规划：TEB
实时障碍物：FAST-LIO 当前点云
```

---

# 2. 使用前必须知道的几个原则

## 2.1 建图和定位模式不能同时启动

正常情况下只选择一种：

```text
需要建立新地图
    ↓
启动 scout_mapping.launch
```

或者：

```text
使用已有地图
    ↓
启动 scout_localization.launch
    ↓
完成 NDT 重定位
    ↓
再启动 navigation_teb.launch
```

不要同时启动 `scout_mapping.launch` 和 `scout_localization.launch`，因为二者都会启动 Mid-360、FAST-LIO 和 Scout 底盘相关节点。

## 2.2 导航前必须确认重定位成功

启动定位程序以后，即使已经能看到：

```text
map -> odom
```

也**不代表机器人已经完成全局定位**。

必须在 RViz 中使用：

```text
2D Pose Estimate
```

给出机器人当前大概位置和朝向，并看到定位终端出现类似：

```text
NDT Relocated
```

然后检查实时点云与历史地图正确重合。

**只有点云与地图对齐后，才能启动自主导航。**

## 2.3 定位和导航必须使用同一张地图

例如定位使用：

```bash
roslaunch scout_system_bringup scout_localization.launch map_name:=scout_map_01
```

导航也必须使用：

```bash
roslaunch scout_navigation navigation_teb.launch map_name:=scout_map_01
```

不能定位使用 `scout_map_01`，导航却使用 `scout_map_02`。

## 2.4 不要重复启动底盘或雷达驱动

`scout_mapping.launch` 和 `scout_localization.launch` 已经自动启动：

- Livox Mid-360 驱动；
- FAST-LIO；
- TF；
- Scout Mini 底盘驱动；
- 相关位姿/点云适配节点。

因此使用这两个总启动文件时，**不要再另外启动 Mid-360 或 Scout 底盘 launch**，否则可能产生重复节点或设备占用冲突。

---

# 3. 每次开机后的准备工作

## 3.1 打开机器人设备

确认：

1. Scout Mini 底盘已上电；
2. Mid-360 已上电；
3. Ubuntu 工控机已启动；
4. 雷达网线和 USB-CAN 已连接；
5. D435i 已使用原装/确认支持 USB3 SuperSpeed 的数据线连接；
6. 机器人周围有足够安全空间。

如果发生失控、错误运动或存在碰撞危险，优先使用实车的物理急停或遥控器安全机制，不要依赖 ROS 命令作为紧急停止手段。

---

## 3.2 打开终端并加载 ROS 工作区

正常情况下已经写入 `~/.bashrc`。当前同时存在主工作区和独立的 RealSense 工作区，建议 `~/.bashrc` 末尾保持：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_fastlio/devel/setup.bash
source ~/realsense_ws/devel/setup.bash --extend
```

这里 `--extend` 很重要：它用于把 `realsense_ws` 叠加到已经加载的 `livox_fastlio` 环境上，避免最后一次 source 后 `scout_system_bringup` 反而找不到。

修改后执行：

```bash
source ~/.bashrc
```

同时验证：

```bash
rospack find scout_system_bringup
```

```bash
rospack find realsense2_camera
```

如果仍出现：

```text
package not found
```

先手工执行：

```bash
source ~/livox_fastlio/devel/setup.bash
```

再检查 `~/.bashrc` 是否缺少 `--extend` 或 source 顺序是否被其他脚本覆盖。

---

## 3.3 启动 CAN

机器人每次重新上电以后执行：

```bash
rosrun scout_bringup bringup_can2usb.bash
```

检查：

```bash
candump can0
```

如果能持续看到 CAN 数据，说明底盘通信正常。

如果提示没有 `can0`，首次配置时可以执行：

```bash
sudo modprobe gs_usb
```

然后：

```bash
rosrun scout_bringup setup_can2usb.bash
```

之后再执行：

```bash
rosrun scout_bringup bringup_can2usb.bash
```

---

# 4. Mid-360 网络检查

正常使用时不需要每天修改配置，但雷达没有数据时应首先检查网络。

## 4.1 工控机地址

当前工程使用：

```text
工控机有线网卡：192.168.1.5/24
```

检查：

```bash
ip addr
```

## 4.2 雷达 IP

Mid-360 默认 IP 规则：

```text
192.168.1.1XX
```

其中：

```text
XX = 雷达机身二维码下方 S/N 的最后两位
```

例如：

```text
S/N 最后两位 = 20
雷达 IP = 192.168.1.120
```

当前这台工程雷达配置为：

```text
192.168.1.120
```

检查：

```bash
ping 192.168.1.120
```

如果更换了 Mid-360，必须根据新雷达 S/N 最后两位重新确定 IP，不能默认继续使用 `.120`。

---

# 5. 最常用功能：使用已有地图进行自主导航

这是机器人日常使用时最常见的流程。

假设使用地图：

```text
scout_map_01
```

---

## 5.1 第一步：确认地图存在

执行：

```bash
ls ~/livox_fastlio/maps
```

应该能看到：

```text
scout_map_01
```

继续检查：

```bash
ls -lh ~/livox_fastlio/maps/scout_map_01
```

正常情况下至少应包含：

```text
raw_camera_init.pcd
public_map.pcd
map_raw.pgm
map_raw.yaml
map.pgm
map.yaml
map_metadata.yaml
```

其中：

```text
public_map.pcd
```

供 NDT 全局定位使用；

```text
map.yaml
```

供定位时显示二维地图；

```text
map_raw.yaml
```

供正式导航的全局代价地图使用。

使用者不需要手工切换这几个文件，launch 已经自动选择正确文件。

---

## 5.2 第二步：启动定位系统

新开终端：

```bash
roslaunch scout_system_bringup scout_localization.launch map_name:=scout_map_01
```

这个命令会同时启动：

```text
Mid-360
FAST-LIO
Scout Mini 底盘
TF
位姿适配
点云适配
PCD 地图加载
NDT 重定位
二维地图
```

启动后暂时**不要发送导航目标**。

---

## 5.3 第三步：打开 RViz

新开终端：

```bash
rviz
```

把：

```text
Fixed Frame
```

设置为：

```text
map
```

建议至少添加以下显示项目：

| RViz Display | Topic | 用途 |
|---|---|---|
| Map | `/map_2d` | 查看已有二维地图 |
| PointCloud2 | `/map_cloud` | 查看历史 PCD 地图 |
| PointCloud2 | `/cloud_registered_base` | 查看机器人当前实时点云 |
| TF | — | 检查坐标系 |

定位成功后，历史地图和当前扫描应在相同位置正确重合。

---

## 5.4 第四步：设置机器人初始位置

在 RViz 顶部工具栏选择：

```text
2D Pose Estimate
```

操作方法：

1. 在地图上机器人实际所在位置按下鼠标；
2. 按住并拖动箭头；
3. 箭头方向指向机器人车头实际朝向；
4. 松开鼠标。

初始位置不要求厘米级准确，但位置和方向必须大致正确。

如果初始猜测距离真实位置过远，NDT 可能匹配到错误位置。

---

## 5.5 第五步：确认 NDT 重定位成功

观察定位终端。

应该看到类似：

```text
NDT Relocated
```

然后在 RViz 中重点看：

```text
/cloud_registered_base
```

是否与：

```text
/map_cloud
```

正确重合。

### 正确状态

```text
墙面 ↔ 墙面
柱子 ↔ 柱子
走廊边缘 ↔ 地图边缘
机器人移动后点云仍稳定贴合地图
```

### 错误状态

如果出现：

```text
点云整体偏移
点云方向旋转错误
墙面出现明显双层
机器人实际向前，但地图中的运动方向明显异常
```

不要启动导航。

重新使用：

```text
2D Pose Estimate
```

给一个更准确的初始位姿。

> 注意：看到 `map -> odom` TF 存在并不足以证明定位成功，最终必须以 NDT 成功和点云实际对齐为准。

---

## 5.6 第六步：启动导航系统

确认定位正确后，新开终端：

```bash
roslaunch scout_navigation navigation_teb.launch map_name:=scout_map_01
```

当前导航使用：

```text
Dijkstra
    ↓
生成全局路径
    ↓
TEB
    ↓
结合实时点云避障
    ↓
/cmd_vel
    ↓
Scout Mini
```

---

## 5.7 第七步：在 RViz 中显示规划结果

建议再添加：

### 全局路径

```text
Display：Path
Topic：/move_base/GlobalPlanner/plan
```

### TEB 局部路径

```text
Display：Path
Topic：/move_base/TebLocalPlannerROS/local_plan
```

### 导航使用的原始静态地图

可增加：

```text
Display：Map
Topic：/nav_static_map
```

正常情况下 `/map_2d` 与 `/nav_static_map` 表示同一环境，但导航实际使用的是 `map_raw.yaml` 产生的 `/nav_static_map`。

---

## 5.8 第八步：发送导航目标

在 RViz 顶部选择：

```text
2D Nav Goal
```

在地图中：

1. 点击目标位置；
2. 按住鼠标拖出箭头；
3. 箭头表示机器人到达目标后的期望朝向；
4. 松开鼠标。

正常流程：

```text
目标
 ↓
Dijkstra 生成全局路径
 ↓
TEB 生成局部运动轨迹
 ↓
机器人开始运动
 ↓
实时点云进入 local costmap
 ↓
TEB 根据障碍物调整轨迹
 ↓
到达目标
```

---

# 6. 导航目标的使用注意事项

## 6.1 不要把目标点贴在障碍物上

目标不要放在：

```text
墙里面
墙边极近位置
桌腿位置
障碍物膨胀区域内部
```

否则即使全局路径看起来接近目标，局部规划也可能无法找到可执行终点。

建议目标点周围留出足够车体空间。

## 6.2 尽量避免一次要求原地大角度掉头

当前系统对大角度掉头属于已知较困难场景。

如果机器人当前朝向和目标方向差异很大，并出现长时间规划不动，可以先发送一个中间目标，让机器人：

```text
先向前移动/调整方向
    ↓
再发送最终目标
```

## 6.3 不要在定位错误时继续导航

如果机器人运行中发现：

```text
点云突然和地图错开
机器人在 RViz 中的位置明显跳变
全局路径突然出现在错误区域
```

应停止导航测试，重新检查定位，而不是继续调整 TEB。

---

# 7. 保存一次导航调试日志

当出现以下问题时建议录日志：

```text
有路径但车不走
TEB 反复抖动
靠近障碍后停住
局部路径消失
cmd_vel 异常
导航突然失败
某个场景需要对比参数效果
```

日志功能已经集成在 `scout_navigation` 中。

---

## 7.1 正确启动顺序

必须先：

```text
定位成功
 ↓
启动 navigation_teb.launch
 ↓
启动 nav_logging.launch
 ↓
最后发送 2D Nav Goal
```

**日志一定要在发送目标之前启动。**

---

## 7.2 启动日志

新开终端：

```bash
roslaunch scout_navigation nav_logging.launch tag:=teb_test
```

`tag` 建议写测试内容，例如：

```bash
roslaunch scout_navigation nav_logging.launch tag:=open_space
```

```bash
roslaunch scout_navigation nav_logging.launch tag:=narrow_passage
```

```bash
roslaunch scout_navigation nav_logging.launch tag:=obstacle_avoidance
```

日志启动后会显示类似：

```text
[NAV_LOG] run_dir=...
[NAV_LOG] perform the test now; Ctrl+C this launch when finished
```

此时再去 RViz 发送导航目标。

---

## 7.3 正确结束日志

测试完成后，在**日志终端**按：

```text
Ctrl+C
```

不要直接关闭终端窗口。

程序会自动执行：

```text
停止 rosbag
 ↓
整理 bag 信息
 ↓
分析导航数据
 ↓
生成 CSV
 ↓
生成 summary.txt
```

直到看到：

```text
[NAV_LOG] DONE
```

这一组日志才算保存完整。

---

## 7.4 日志保存位置

日志自动保存在：

```text
~/livox_fastlio/logs/navigation/
```

每次会建立一个带时间戳的独立目录，例如：

```text
20260823_103000_narrow_passage
```

查看最近一次日志：

```bash
cat ~/livox_fastlio/logs/navigation/LAST_RUN
```

查看最近一次自动分析：

```bash
cat "$(cat ~/livox_fastlio/logs/navigation/LAST_RUN)/summary.txt"
```

日志包含：

```text
rosbag
当时的导航参数
当时的 launch
TF
全局路径
TEB 局部路径
cmd_vel
Scout 轮速里程计
costmap
实时点云
导航状态
自动生成的 CSV
summary.txt
```

因此以后参数已经修改，也可以恢复某一次测试当时的配置。

---

## 7.5 日志磁盘空间

日志包含点云和 costmap，数据量较大。

测试前建议：

```bash
df -h ~
```

rosbag 会使用 LZ4 压缩，并按约 2048 MiB 自动分片。

长时间录制时必须关注剩余磁盘空间。

---

## 7.6 压缩日志供分析

最近一次日志可以这样压缩：

```bash
RUN_DIR=$(cat ~/livox_fastlio/logs/navigation/LAST_RUN)
```

然后：

```bash
cd "$(dirname "$RUN_DIR")"
```

最后：

```bash
zip -r "$(basename "$RUN_DIR").zip" "$(basename "$RUN_DIR")"
```

生成类似：

```text
20260823_103000_narrow_passage.zip
```

后续排查导航问题时直接提供这个 ZIP 即可。

---

# 8. 建立一张新地图

只有环境发生较大变化、首次部署或需要新的作业区域时才需要重新建图。

---

## 8.1 建图前准备

先完成：

```text
底盘上电
Mid-360 上电
CAN 正常
雷达网络正常
```

建议：

- 环境尽量保持静态；
- 建图过程中低速行驶；
- 避免猛烈急转；
- 尽量完整覆盖所有需要导航的区域；
- 对走廊、拐角、门口等关键区域尽量从多个方向经过；
- 尽量形成闭合或重复经过部分区域，以便检查地图一致性。

---

## 8.2 启动建图

新开终端：

```bash
roslaunch scout_system_bringup scout_mapping.launch map_name:=factory_a
```

检查 FAST-LIO 是否正常工作。

可选检查：

```bash
rostopic hz /cloud_registered_body
```

以及：

```bash
rosrun tf tf_echo odom base_link
```

---

## 8.3 遥控机器人完成环境扫描

使用底盘原有安全遥控方式操纵小车。

建图过程中注意：

```text
慢速
平稳
尽量少急停急转
覆盖墙角和结构特征
不要发生碰撞
```

---

## 8.4 正常结束建图

完成扫描后，保持建图 launch 运行，在另一个终端执行：

```bash
rosservice call /finish_mapping
```

该服务会保存过滤 PCD 并自动生成全部地图文件。必须确认返回：

```text
success: True
```

然后再在建图终端按 `Ctrl+C`。如果返回失败，不要停止 launch；先查看
`scout_mapping_finisher` 的错误日志并重试。

---

# 9. 生成地图文件夹

假设新地图名称为：

```text
factory_a
```

第一次创建：

```bash
rosrun scout_map_tools finalize_map.py factory_a
```

程序会自动建立：

```text
~/livox_fastlio/maps/factory_a/
```

不需要提前手动创建文件夹。

生成：

```text
raw_camera_init.pcd
public_map.pcd
map_raw.pgm
map_raw.yaml
map.pgm
map.yaml
map_metadata.yaml
```

检查：

```bash
ls -lh ~/livox_fastlio/maps/factory_a
```

---

## 9.1 地图命名建议

建议使用：

```text
factory_a
lab_01
office_floor1
warehouse_20260823
```

尽量只使用：

```text
英文字母
数字
下划线
短横线
```

不建议地图名中使用空格。

---

## 9.2 覆盖已有地图

如果明确要使用最新的：

```text
scans.pcd
```

覆盖已有同名地图的原始 PCD，例如：

```text
scout_map_01
```

执行：

```bash
rosrun scout_map_tools finalize_map.py scout_map_01 --replace-raw
```

这是覆盖操作。

如果不确定是否需要覆盖，**优先建立一个新地图名**，例如：

```bash
rosrun scout_map_tools finalize_map.py scout_map_02
```

这样更安全，也便于回退到旧地图。

---

# 10. 使用新建地图进行导航

例如刚建立：

```text
factory_a
```

定位：

```bash
roslaunch scout_system_bringup scout_localization.launch map_name:=factory_a
```

在 RViz 使用：

```text
2D Pose Estimate
```

完成 NDT 重定位并确认点云对齐。

然后启动导航：

```bash
roslaunch scout_navigation navigation_teb.launch map_name:=factory_a
```

最后使用：

```text
2D Nav Goal
```

发送目标。

---

# 11. 切换地图

查看现有地图：

```bash
ls ~/livox_fastlio/maps
```

假设有：

```text
scout_map_01
factory_a
factory_b
```

需要切换到 `factory_b` 时，应停止当前定位和导航，再重新启动：

```bash
roslaunch scout_system_bringup scout_localization.launch map_name:=factory_b
```

重定位成功以后：

```bash
roslaunch scout_navigation navigation_teb.launch map_name:=factory_b
```

**不要在旧定位进程仍运行时直接启动另一张地图。**

---

# 12. 正常停止机器人软件

如果正在录日志：

### 第一步

先在日志终端：

```text
Ctrl+C
```

等待：

```text
[NAV_LOG] DONE
```

### 第二步

停止导航：

```text
navigation_teb.launch 终端 → Ctrl+C
```

### 第三步

停止定位：

```text
scout_localization.launch 终端 → Ctrl+C
```

如果当前是建图模式：

```text
scout_mapping.launch 终端 → Ctrl+C
```

并等待 `scans.pcd` 保存完成。

### 第四步

确认机器人已经停止运动后再关闭硬件电源。

需要关闭 CAN 接口时可以执行：

```bash
sudo ip link set can0 down
```

---

# 13. 常用检查命令

## 13.1 底盘

```bash
candump can0
```

```bash
rostopic hz /scout/odom
```

## 13.2 雷达

```bash
rostopic hz /livox/lidar
```

```bash
rostopic hz /livox/imu
```

## 13.3 FAST-LIO

```bash
rostopic hz /cloud_registered_body
```

## 13.4 定位输入

```bash
rostopic hz /cloud_registered_base
```

```bash
rostopic hz /fastlio_odom
```

## 13.5 TF

```bash
rosrun tf tf_echo odom base_link
```

定位模式下：

```bash
rosrun tf tf_echo map odom
```

## 13.6 导航控制

```bash
rostopic hz /cmd_vel
```

## 13.7 全局路径

```bash
rostopic hz /move_base/GlobalPlanner/plan
```

---

# 14. 常见问题处理

## 14.1 `can0` 不存在

执行：

```bash
sudo modprobe gs_usb
```

然后：

```bash
rosrun scout_bringup bringup_can2usb.bash
```

仍然失败时检查 USB-CAN 硬件连接。

---

## 14.2 Mid-360 没有数据

先检查：

```bash
ip addr
```

工控机有线网卡应处于：

```text
192.168.1.5/24
```

根据雷达二维码 S/N 最后两位确定雷达 IP，例如：

```bash
ping 192.168.1.120
```

如果 ping 不通，先处理网络，不要先调整 FAST-LIO 或导航参数。

---

## 14.3 定位启动了，但点云没有贴合地图

重新使用：

```text
2D Pose Estimate
```

给出更准确的位置和朝向。

同时确认：

- 使用的是正确地图；
- 雷达安装位置没有发生变化；
- 地图对应的是当前环境；
- 当前环境没有发生大范围结构变化。

---

## 14.4 已经看到 `map -> odom`，但机器人位置明显错误

这是可能出现的。

`map -> odom` 存在本身不代表 NDT 已成功。

必须以：

```text
NDT Relocated
```

和点云实际贴合地图为判断标准。

---

## 14.5 有全局路径，但机器人不走

先不要立即改参数。

检查：

```bash
rostopic hz /cmd_vel
```

再检查 RViz：

```text
全局路径
TEB local_plan
实时点云
局部障碍区域
```

如果问题可以重复出现，先启动：

```bash
roslaunch scout_navigation nav_logging.launch tag:=problem_case
```

重新复现一次，然后保存日志分析。

---

## 14.6 目标靠墙时无法到达

这是当前系统应避免的使用方式。

不要把 `2D Nav Goal` 放得过于靠近墙、桌腿或其他障碍物。

机器人不是一个点，导航必须为完整车体轮廓和安全距离保留空间。

---

## 14.7 大角度转向时机器人长时间不动

当前系统在某些大角度掉头场景存在局部规划困难。

实际使用时可以：

```text
先发送一个中间目标
 ↓
让机器人调整朝向
 ↓
再发送最终目标
```

---

## 14.8 地图名称找不到

执行：

```bash
ls ~/livox_fastlio/maps
```

确认实际地图目录名。

定位和导航的：

```text
map_name
```

必须完全一致。

---

## 14.9 日志太大

检查：

```bash
df -h ~
```

一次只录需要分析的测试过程，测试结束后及时在日志终端按：

```text
Ctrl+C
```

并等待自动分析完成。

---


## 14.10 D435i 能识别但只有 `480M`

执行：

```bash
lsusb -t
```

D435i 正式运行应尽量看到：

```text
5000M
```

如果只有：

```text
480M
```

说明实际只工作在 USB2。优先更换为 D435i 原装线或明确支持 USB3/5Gbps 的数据线，并换到 Jetson 的 USB3 接口。RGB + Depth + IMU 同时工作时不建议长期使用 USB2。

## 14.11 D435i 显示 `Intel RealSense D4XX Recovery`

执行：

```bash
rs-enumerate-devices
```

如果设备名称是：

```text
Intel RealSense D4XX Recovery
```

说明相机处于固件恢复模式，不是正常工作状态。普通使用者不要继续启动 ROS、不要在 Viewer 中随意升级固件，应交由开发人员恢复当前工程匹配的固件。

当前已验证的软件组合为：

```text
librealsense 2.50.0
realsense-ros 2.3.2
D400 firmware 05.13.00.50
```

## 14.12 D435i 图像正常，但 RViz 报 `camera_color_optical_frame does not exist`

先检查：

```bash
rosrun tf tf_echo camera_link camera_color_optical_frame
```

再检查：

```bash
rosrun tf tf_echo base_link camera_color_optical_frame
```

当前 `d435i.launch` 必须让 RealSense 自己发布内部 TF：

```text
publish_tf = true
tf_publish_rate = 0
```

工程自己只发布：

```text
base_link -> camera_link
```

不要手工重复发布 `camera_link -> camera_color_optical_frame`。

## 14.13 `scout_system_bringup` 突然找不到

如果：

```bash
rospack find scout_system_bringup
```

报 package not found，但手工：

```bash
source ~/livox_fastlio/devel/setup.bash
```

之后马上恢复，通常是 `realsense_ws` 的 source 覆盖了主工作区。

检查 `~/.bashrc`，推荐保持：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_fastlio/devel/setup.bash
source ~/realsense_ws/devel/setup.bash --extend
```

# 15. 推荐的日常导航标准流程

正常使用已有地图时，只需要记住下面这条流程。

```text
1. 底盘、雷达、工控机上电
       ↓
2. rosrun scout_bringup bringup_can2usb.bash
       ↓
3. roslaunch scout_system_bringup scout_localization.launch map_name:=地图名
       ↓
4. 打开 RViz，Fixed Frame = map
       ↓
5. 2D Pose Estimate
       ↓
6. 等待 NDT Relocated
       ↓
7. 确认实时点云正确贴合历史地图
       ↓
8. roslaunch scout_navigation navigation_teb.launch map_name:=同一地图名
       ↓
9. （需要调试时）启动 nav_logging.launch
       ↓
10. RViz 使用 2D Nav Goal
       ↓
11. 机器人自主导航
```

例如使用 `scout_map_01`：

### 终端 1

```bash
rosrun scout_bringup bringup_can2usb.bash
```

### 终端 2

```bash
roslaunch scout_system_bringup scout_localization.launch map_name:=scout_map_01
```

### 终端 3

```bash
rviz
```

在 RViz：

```text
Fixed Frame = map
2D Pose Estimate
确认 NDT 定位
```

### 终端 4

```bash
roslaunch scout_navigation navigation_teb.launch map_name:=scout_map_01
```

### 可选终端 5

```bash
roslaunch scout_navigation nav_logging.launch tag:=normal_test
```

然后在 RViz：

```text
2D Nav Goal
```

---

# 16. 推荐的建图标准流程

```text
1. 底盘、雷达、工控机上电
       ↓
2. 启动 CAN
       ↓
3. roslaunch scout_system_bringup scout_mapping.launch map_name:=新地图名
       ↓
4. 遥控小车低速完整扫描环境
       ↓
5. rosservice call /finish_mapping
       ↓
6. 确认服务返回 success: True
       ↓
7. 建图终端 Ctrl+C
       ↓
8. 检查 ~/livox_fastlio/maps/新地图名/
       ↓
9. 使用新地图启动 localization
       ↓
10. 2D Pose Estimate + NDT 验证
       ↓
11. 启动 navigation_teb.launch
```

---

# 17. 使用者禁止或不建议进行的操作

普通使用过程中不要做以下操作：

1. 不要同时启动建图模式和定位模式。
2. 不要重复启动 Mid-360 驱动。
3. 不要重复启动 Scout 底盘驱动。
4. 不要在 NDT 尚未成功时发送导航目标。
5. 不要让定位和导航使用不同 `map_name`。
6. 不要随意修改 TF、外参、TEB、costmap 参数。
7. 不要直接覆盖正式地图，除非明确知道为什么需要 `--replace-raw`。
8. 不要把导航目标放入障碍物或紧贴障碍物。
9. 不要在日志仍在写入时直接断电。
10. 不要在机器人运动异常时继续测试；优先使用硬件安全停止措施。

如果雷达安装角度、安装位置、底盘几何尺寸等机械结构发生变化，应交由开发人员重新检查外参和地图，不应把它当成普通操作问题处理。

---


# 18. D435i 视觉、深度与 IMU 使用

当前 D435i 用途限定为：

```text
目标识别
视觉检测
深度测距
深度避障
```

当前不把 D435i 作为 FAST-LIO/NDT 的定位传感器，也不要求进行高精度 LiDAR-Camera 联合标定。

## 18.1 当前固定安装外参

当前按机械安装近似值直接写死：

```text
base_link -> camera_link

x = +0.27 m
y =  0.00 m
z = +0.10 m

yaw   = 0
pitch = 0
roll  = π = 3.14159265
```

含义：

- 相机位于车体中心前方约 27 cm；
- 左右基本居中；
- 相机中心高于 `base_link` 约 10 cm；
- 相机相对车体倒装，但镜头仍朝车头。

该外参由：

```text
scout_system_bringup/launch/d435i.launch
```

发布。

## 18.2 启动 D435i

如果当前总 launch 尚未 include D435i，则单独启动：

```bash
roslaunch scout_system_bringup d435i.launch
```

如果以后已经把 `d435i.launch` include 到 `scout_mapping.launch` 或 `scout_localization.launch`，则不要再手工启动第二次，否则同一台相机会被两个节点重复占用。

## 18.3 启动后快速检查

USB：

```bash
lsusb -t
```

D435i 应为：

```text
5000M
```

RGB：

```bash
rostopic hz /camera/color/image_raw
```

Depth：

```bash
rostopic hz /camera/depth/image_rect_raw
```

RGB 对齐后的 Depth：

```bash
rostopic hz /camera/aligned_depth_to_color/image_raw
```

Gyro：

```bash
rostopic hz /camera/gyro/sample
```

Accel：

```bash
rostopic hz /camera/accel/sample
```

融合 IMU：

```bash
rostopic hz /camera/imu
```

TF：

```bash
rosrun tf tf_echo base_link camera_link
```

完整 RGB optical TF：

```bash
rosrun tf tf_echo base_link camera_color_optical_frame
```

## 18.4 查看 RGB / Depth 图像

推荐：

```bash
rqt_image_view
```

RGB：

```text
/camera/color/image_raw
```

原始深度：

```text
/camera/depth/image_rect_raw
```

目标识别后查询目标距离时，优先使用：

```text
/camera/aligned_depth_to_color/image_raw
```

因为当前 launch 已开启：

```text
align_depth = true
```

这样 RGB 像素位置与对齐深度图更容易直接对应。

## 18.5 RViz 显示

RViz 的：

```text
Fixed Frame
```

可以根据当前模式使用：

```text
base_link
odom
map
```

添加 `Image` 显示 RGB 时选择：

```text
/camera/color/image_raw
```

如果出现：

```text
For frame [camera_color_optical_frame]:
Frame[camera_color_optical_frame] does not exist
```

说明 RealSense 内部 TF 没有正常发布，应按 14.12 检查，不要修改 `base_link -> camera_link` 的 xyz 来解决。

## 18.6 当前为什么默认不开 D435i PointCloud2

当前 launch 保持：

```text
enable_pointcloud = false
```

因为目标识别和深度避障只需要 RGB + Depth；另外系统已有 Mid-360/FAST-LIO 点云。默认再生成 D435i PointCloud2 会额外增加 Jetson CPU、内存和 ROS 数据带宽开销。


# 19. 一句话速记

**日常使用就是：上电 → CAN → 确认 Mid-360/D435i 连接正常 → 定位 → RViz 设初始位姿 → 确认 NDT 点云贴图 → 启动 Dijkstra + TEB → 发导航目标；视觉任务使用 D435i 的 RGB + aligned depth，出现规划问题时在发目标前启动 `nav_logging` 保存完整日志。**
