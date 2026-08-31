# Orbbec Gemini 336L 相机说明

设备 USB ID 为 `2bc5:0807`。NX 使用官方 OrbbecSDK_ROS1 `v2-main`（提交 `a2838b3`），ROS Wrapper / SDK 版本 `2.9.3`。实测固件为 `1.4.60`，连接为 USB 3.2。

## 一键启动

```bash
cd ~/livox_fastlio
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch wheeltec_system_bringup wheeltec_orbbec336l.launch
```

默认输出 `/camera/color/image_raw`、`/camera/depth/image_raw` 和 `/camera/depth/points`，实测约 30 Hz。RGB 为 1280×720 MJPG，深度为 848×480 Y16。

TF 链为：

```text
base_link -> camera_link -> camera_*_optical_frame
```

`camera_link -> camera_*_optical_frame` 由驱动发布。正式 `base_link -> camera_link` 外参为前 `0.16 m`、左 `0 m`、上 `0.08 m`，相机与车体同姿态，因此 `roll=pitch=yaw=0`。

```bash
roslaunch wheeltec_system_bringup wheeltec_orbbec336l.launch \
  camera_x:=0.16 camera_y:=0.0 camera_z:=0.08 \
  camera_roll:=R camera_pitch:=P camera_yaw:=W
```

车体坐标约定为前 `+X`、左 `+Y`、上 `+Z`。只测试驱动时可加 `publish_mount_tf:=false`。

官方驱动在该 NX 上需要 `third_party_patches/orbbec_ros1_aarch64_link_order.patch`，OpenCV 开发头文件也必须与 Ubuntu 20.04/ROS Noetic 的 4.2.0 运行库一致。
