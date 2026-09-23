# ArmBot

English | [中文](https://github.com/Hiwonder/ArmPi-FPV/blob/main/README_cn.md)

<p align="center">
  <img src="./sources/images/armpi-fpv.png" alt="Hiwonder ArmPi FPV hardware logo" width="600"/>
</p>

## Product Overview

ArmBot is a ROS Noetic fruit pick-and-place project for a robotic arm paired with an RGB-D depth camera. It detects fruit, estimates 3D positions from color and depth images, and uses the arm to pick and place targets. The current hardware profile uses an Orbbec Astra Pro Plus and targets Hiwonder ArmPi FPV-compatible arm hardware. Upstream ROS package and launch names remain unchanged for compatibility.

## Official Resources

### Official Hiwonder
- **Official Website**: [https://www.hiwonder.com/](https://www.hiwonder.com/)
- **Product Page**: [https://www.hiwonder.com/products/armpi-fpv](https://www.hiwonder.com/products/armpi-fpv)
- **Official Documentation**: [https://docs.hiwonder.com/projects/ArmPi_FPV/en/latest/](https://docs.hiwonder.com/projects/ArmPi_FPV/en/latest/)
- **Technical Support**: support@hiwonder.com

## Key Features

### RGB-D Fruit Perception
- **Fruit Detection** - YOLO-based detection for supported fruit classes
- **3D Localization** - Combines color and depth images to estimate target positions
- **Camera Calibration** - Eye-to-hand transform maps camera coordinates to the arm base

### Pick and Place
- **Fruit Handling** - Executes the configured grasp and placement workflow
- **Arm Control** - Uses the existing ROS controller, servo drivers, and kinematics
- **ROS Noetic** - Provides the camera, perception, and manipulation runtime

## Hardware Configuration
- **Processor**: Raspberry Pi 5 or Raspberry Pi 4B
- **Operating System**: ROS Noetic compatible Linux
- **Depth Camera**: Orbbec Astra Pro Plus RGB-D camera
- **Servos**: High-precision bus servos
- **DOF**: Multi-degree-of-freedom robotic arm
- **Communication**: WiFi, Bluetooth

## Project Structure

```
ArmBot/
├── Dockerfile
├── docker-compose.yml
├── config/                        # Camera configuration
├── models/                        # Detection models
├── src/
│   ├── desk_cleaning/             # RGB-D fruit detection and pick-and-place
│   ├── orbbec_camera/             # Depth camera driver
│   ├── ros_robot_controller/      # Arm controller interface
│   ├── hiwonder_servo_*/          # Servo control and messages
│   └── armpi_fpv_*/               # Robot description and kinematics
└── sources/                      # Resources and documentation
```

Other upstream ROS examples remain in `src`, but are not part of the fruit pick-and-place workflow.

## Version Information
- **Current Version**: ArmBot v1.0.0 (ArmPi FPV-compatible)
- **Supported Platform**: Raspberry Pi 5, Raspberry Pi 4B
- **ROS Version**: ROS1 (Noetic)

### Related Technologies
- [ROS](http://www.ros.org/) - Robot Operating System
- [MoveIt](https://moveit.ros.org/) - Motion Planning Framework
- [OpenCV](https://opencv.org/) - Computer Vision Library
- [Python](https://www.python.org/) - Programming Language

---

**Note**: ArmBot is designed to run with Hiwonder ArmPi FPV hardware. For hardware tutorials, refer to the [Official Documentation](https://docs.hiwonder.com/projects/ArmPi_FPV/en/latest/).
