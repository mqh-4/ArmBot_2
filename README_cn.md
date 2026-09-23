# ArmBot

[English](https://github.com/Hiwonder/ArmPi-FPV/blob/main/README.md) | 中文

<p align="center">
  <img src="./sources/images/armpi-fpv.png" alt="Hiwonder ArmPi FPV hardware logo" width="600"/>
</p>

ArmBot 是一个使用机械臂和 RGB-D 深度相机实现水果识别、定位、抓取与放置的 ROS Noetic 项目。当前相机配置为 Orbbec Astra Pro Plus，机械臂硬件面向 Hiwonder ArmPi FPV 兼容设备。为保持兼容性，ROS 包名和 launch 名称沿用上游命名。

## 产品介绍

ArmBot 使用 YOLO 识别配置的水果类别，通过彩色图像和深度图像估计目标的三维位置，再由机械臂完成抓取和放置。手眼标定结果用于将相机坐标转换到机械臂基坐标。当前项目流程聚焦水果抓放，不包含码垛功能。

## 官方资源

### Hiwonder官方
- **官方网站**: [https://www.hiwonder.com/](https://www.hiwonder.com/)
- **产品页面**: [https://www.hiwonder.com/products/armpi-fpv](https://www.hiwonder.com/products/armpi-fpv)
- **官方文档**: [https://docs.hiwonder.com/projects/ArmPi_FPV/en/latest/](https://docs.hiwonder.com/projects/ArmPi_FPV/en/latest/)
- **技术支持**: support@hiwonder.com

## 主要功能

### RGB-D水果识别与定位
- **水果检测** - 使用 YOLO 识别配置的水果类别
- **三维定位** - 融合彩色图像和深度图像估计目标位置
- **手眼标定** - 将相机坐标转换到机械臂基坐标

### 机械臂抓取与放置
- **水果抓放流程** - 执行配置的抓取和放置动作
- **机械臂控制** - 使用现有 ROS 控制器、舵机驱动和运动学
- **ROS Noetic** - 运行相机、感知和操作流程

## 硬件配置
- **处理器**: 树莓派5或树莓派4B
- **操作系统**: ROS Noetic兼容Linux系统
- **深度相机**: Orbbec Astra Pro Plus RGB-D 相机
- **舵机**: 高精度总线舵机
- **自由度**: 多自由度机械臂
- **通信**: WiFi、蓝牙

## 项目结构

```
ArmBot/
├── Dockerfile
├── docker-compose.yml
├── config/                        # 相机配置
├── models/                        # 检测模型
├── src/
│   ├── desk_cleaning/             # RGB-D 水果识别与抓放
│   ├── orbbec_camera/             # 深度相机驱动
│   ├── ros_robot_controller/      # 机械臂控制接口
│   ├── hiwonder_servo_*/          # 舵机控制和消息
│   └── armpi_fpv_*/               # 机器人描述和运动学
└── sources/                      # 资源和文档
```

`src` 中还保留其他上游 ROS 示例包，但它们不属于水果抓放主流程。

## 水果抓取与放置流程

仓库中的 `desk_cleaning` ROS 包实现 YOLO ONNX 水果检测、RGB-D 三维定位和机械臂抓放。详细的相机接入、标定、训练与运行说明见
[`src/desk_cleaning/README_cn.md`](src/desk_cleaning/README_cn.md)。

项目已附带一份可直接打印的 ChArUco 眼在手上标定板，位于
[`sources/calibration/`](sources/calibration/)。

## 版本信息
- **当前版本**: ArmBot v1.0.0（兼容 ArmPi FPV）
- **支持平台**: 树莓派5、树莓派4B
- **ROS版本**: ROS1 (Noetic)

### 相关技术
- [ROS](http://www.ros.org/) - 机器人操作系统
- [MoveIt](https://moveit.ros.org/) - 运动规划框架
- [OpenCV](https://opencv.org/) - 计算机视觉库
- [Python](https://www.python.org/) - 编程语言

---

**注**: ArmBot 面向 Hiwonder ArmPi FPV 硬件运行。硬件使用说明请参考[官方文档](https://docs.hiwonder.com/projects/ArmPi_FPV/en/latest/)。
