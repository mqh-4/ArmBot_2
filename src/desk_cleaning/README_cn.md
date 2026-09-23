# ArmBot 水果识别与抓放

ROS 包名 `desk_cleaning` 为兼容上游接口而保留。ArmBot 当前主流程使用 YOLO、RGB-D 深度和手眼标定，完成苹果、橙子、香蕉的识别、定位、抓取与放置。

## 当前支持范围

`config/objects.yaml` 只启用苹果、橙子和香蕉，其余类别均不可抓取。Orbbec Astra Pro Plus 固定在机械臂外部，使用 eye-to-hand 标定。所有运动默认是 `dry_run`；完成相机标定、工作空间测量和单物体验证前，不会向实体机械臂发送动作。

语音/文本任务规划是可选入口；无论使用哪种入口，执行器都只接受配置中启用的水果类别。

## 本地语言模型

默认 `llm_backend=http`，适合把模型放在另一台电脑上通过 OpenAI 兼容接口提供服务。
如果希望语言模型直接加载在 ROS 节点进程内，可使用 `llama.cpp` 的 GGUF 模型：

```bash
roslaunch desk_cleaning desk_cleaning.launch \
  llm_enabled:=true llm_backend:=llama_cpp \
  llm_model_path:=/home/ubuntu/models/qwen2.5-1.5b-instruct-q4_k_m.gguf \
  dry_run:=true
```

目标设备需要自行安装 `llama-cpp-python` 及其编译依赖；仓库不把它写成强制 ROS 依赖，
因为树莓派 5 上是否启用 NEON/CPU 优化取决于你的 Python 和编译环境。内置模式只负责
把自然语言解析成受白名单约束的任务 JSON，规则解析仍优先执行，舵机控制、坐标和逆运动学
不会交给语言模型。

## 节点和接口

```text
/desk_cleaning/voice_text (std_msgs/String)
          |
          v
voice_task_planner --可选本地 LLM--> /desk_cleaning/clean_desk (Action)
                                            |
Orbbec Astra Pro Plus RGB + 对齐深度 + CameraInfo v
          |                         desk_cleaning_executor
          v                                 |
desk_perception --DetectedObjectArray--> 自定义 IK -> 总线舵机
                                            |
                                            v
                                  /desk_cleaning/speech
                                            |
                                            v
                                      TTS 语音模块
```

主要接口：

- `/desk_perception/objects`：带 `base_link` 三维坐标的检测结果。
- `/desk_cleaning/clean_desk`：支持进度、取消和结果的清理 Action。
- `/desk_cleaning/start`、`/desk_cleaning/stop`：不接语音时用于快速测试。
- `/desk_cleaning/speech`：所有需要播报的文本。
- `/desk_cleaning/parsed_command`：经过校验的语言模型/规则解析结果。

桌面清理执行器会在调用旧版 ArmPi IK 服务前，将 `base_link` 的 XY 坐标转换为 IK
库的坐标：`(x_ik, y_ik) = (y_base, -x_base)`。这是因为该库的 FK 端点相对 URDF
的 `base_link` 绕 Z 轴旋转了 90 度；感知、工作空间和复检仍统一使用 TF 的
`base_link` 坐标。末端工具长度已包含在 IK 的 `tool_link=0.1126 m` 中，不在此处
重复补偿。

## Orbbec Astra Pro Plus 接入约定

相机驱动不与本包耦合。Astra Pro Plus 应使用 ROS1/OpenNI 驱动，驱动节点只要发布三个标准
ROS 话题：

```text
/camera/rgb/image_raw                         sensor_msgs/Image, bgr8/rgb8
/camera/depth_registered/image_raw            sensor_msgs/Image, 16UC1 或 32FC1
/camera/rgb/camera_info                       sensor_msgs/CameraInfo
```

深度图必须已经对齐到彩色图。`16UC1` 通常以毫米为单位，对应 `depth_scale=0.001`；
如果厂商输出 `32FC1` 米制深度，应将 `depth_scale` 改为 `1.0`。实际话题名可以通过
launch 参数覆盖。

采购和部署相机前必须确认：Ubuntu ARM64 驱动或源码编译能力、ROS1 Noetic 驱动、RGB/深度同步、深度对齐、
CameraInfo 内参、近距离最小深度以及 USB 供电要求。若只有厂商 Python/C++ SDK，应单独
创建 `astra_camera_driver` 包完成上述标准话题适配。Astra Pro Plus 属于较老的 ROS1 v1
维护路线；在树莓派 5 上建议先验证驱动能否稳定发布 ARM64 下的彩色、注册深度和 CameraInfo，
再进行手眼标定。

## 目标检测模型

仓库没有提交完整的水果专用模型权重。当前默认优先尝试 `yolov8s.onnx`，如果部署目录中还没有
该文件，感知节点会自动回退到同目录已有的 `yolov8n.onnx` 并打印警告。把同样类别顺序的
YOLOv8s ONNX 文件放到 `models/yolov8s.onnx` 后即可直接启用，不需要改代码。

如需训练专用模型，请使用真实相机和实际水果场景采集数据，标注以下类别：

```text
apple orange banana
```

建议至少覆盖：不同背景和光照、目标旋转、部分遮挡、空桌面和机械臂进入
画面的负样本。训练完成后导出 YOLOv8 风格 ONNX，输入大小默认 `640x640`。当前配置使用
COCO 80 类顺序；若使用三分类模型，需把 `perception.yaml` 中的 `labels` 改为模型输出顺序。

模型部署示例：

```bash
roslaunch desk_cleaning desk_cleaning.launch \
  model_path:=/home/ubuntu/models/fruit.onnx \
  start_arm:=false dry_run:=true
```

## 工作区域 ROI 裁剪

感知节点默认只在彩色图的桌面区域运行检测，减少背景和机械臂对 YOLO 的干扰。ROI 在
`config/perception.yaml` 中用 0 到 1 的归一化比例配置：

```yaml
camera:
  roi_enabled: true
  desktop_roi:
    x_min: 0.05
    y_min: 0.20
    x_max: 0.95
    y_max: 0.98
```

检测框会自动加回 ROI 左上角偏移后再取深度、计算三维坐标和发布，因此不会改变机械臂坐标。
调试图会显示橙色 ROI 边框和绿色检测框。若相机视野或桌面位置不同，只需调整四个比例；需要
临时关闭时将 `roi_enabled` 设为 `false`。

感知节点使用 OpenCV DNN，不要求运行时安装完整训练框架。

## 眼在手外的相机外参标定

Orbbec Astra Pro Plus 固定在机械臂外部，采用 eye-to-hand。标定板固定在拆除夹爪后的
`link5` 法兰上，标定结果是固定的 `base_link -> astra_color_optical_frame` 变换。

当前使用 `6 x 4` ChArUco 标定板，单格 `20 mm`、标记 `15 mm`、字典
`DICT_4X4_50`。棋盘区域尺寸为 `120 x 80 mm`，包含 12 个 ArUco 标记和 15 个
ChArUco 内角点。

采集与验证流程：

1. 固定相机、机械臂底座和标定板，采集过程中不得移动固定关系。
2. 将标定板安装到法兰并使用 `base_link -> link5` 作为机器人位姿。
3. 采集 15 至 25 个覆盖远近、左右、高低和多个倾角的姿态。
4. 每次机械臂停止后再保存样本，标定板必须完整可见。
5. 剔除机器人相对旋转与视觉相对旋转不一致的异常样本。
6. 使用未参与求解的新姿态验证，建议平移误差不超过 `10 mm`、旋转误差不超过 `5 deg`。

正式结果保存在：

```text
output/eye_to_hand_result.json
```

统一入口 `armpi_fpv_project.launch` 将外部相机命名为 `astra_*`，从而避开旧 URDF 中
随机械臂运动的 `camera_link`。`eye_to_hand_tf_publisher.py` 会读取结果文件，并结合相机
驱动的内部 TF 自动发布：

```text
base_link -> astra_link -> astra_depth_frame -> astra_color_frame
          -> astra_color_optical_frame
```

可用以下命令检查标定外参：

```bash
rosrun tf tf_echo base_link astra_color_optical_frame
```

外部相机固定时，该变换在机械臂运动后必须保持不变。相机、相机支架或机械臂底座移动后，
必须重新标定。完成目标坐标和空载动作验证前保持 `dry_run:=true`。

对于苹果等近似球体，中心像素的深度是朝向相机一侧的表面深度，不是几何中心。
感知节点对 `apple` 使用检测框表观半径沿相机光轴补偿深度，再发布几何中心的
`base_link` 坐标；执行器不再额外添加表面高度偏移。

## 工作空间和抓取参数

[workspace.yaml](config/workspace.yaml) 包含：

- 允许抓取的 `x/y/z` 范围。
- 默认、左侧和右侧收纳盒的位置。
- 接近高度、抓取偏移、抬升高度。
- 夹爪开合脉宽和安全观察姿态。
- 检测置信度、最大物体尺寸和重试次数。

配置值目前是保守起点，不是完成标定的生产参数。先用软物体、低速和急停按钮进行单物体
测试。桌面、收纳盒、相机支架或相机安装角度变化后需要重新标定或验证。

## 构建和测试

ROS Noetic 环境中：

```bash
cd /home/ubuntu/armpi_fpv
catkin_make
source devel/setup.bash
```

### 旧版桌面 Gazebo 示例

以下场景使用文具模型，不属于当前水果抓放主流程，也不作为当前功能验收方式。

安装 ROS Noetic 对应组件：

```bash
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
  ros-noetic-ros-controllers
```

此 Gazebo 场景是保留的旧桌面示例，不代表当前水果抓放主流程；当前主流程以外置 Astra RGB-D 相机和实体机械臂为准。
它使用 Gazebo 真值输出代替检测网络，因此不需要 ONNX 权重就能先验证语音、任务状态机、
逆运动学、舵机指令桥接、抓放和逐件播报：

```bash
roslaunch desk_cleaning gazebo_desk_cleaning.launch
rostopic pub -1 /desk_cleaning/voice_text std_msgs/String "data: '清理桌面'"
```

仿真中的播报会出现在终端和 `/desk_cleaning/speech`，不会调用实体语音硬件。暂停启动和无界面
运行分别使用 `paused:=true`、`gui:=false`。场景物品定义在
[desk_cleaning.world](worlds/desk_cleaning.world)，检测类别和尺寸定义在
[gazebo.yaml](config/gazebo.yaml)。新增物体时，两处的 Gazebo 模型名必须一致。

确认真值模式完整运行后，可让真实检测代码处理仿真相机图像：

```bash
roslaunch desk_cleaning gazebo_desk_cleaning.launch \
  use_truth_perception:=false \
  model_path:=/home/ubuntu/models/desk_objects.onnx
```

仿真相机发布与 Astra Pro Plus 接入约定相同的三个话题。它用于软件联调，不代表 Astra Pro Plus
真实噪声、畸变和深度空洞；训练与最终精度验证仍必须使用实体相机数据。仿真抓取使用逻辑吸附，
只有末端到物体距离小于 `max_attach_distance` 才成功，这比依赖未调好的接触摩擦更适合先验证
项目流程。

完全不连接相机和机械臂时，可以先运行端到端模拟。模拟节点会发布一支签字笔和一块橡皮，
执行器只打印计划动作：

```bash
roslaunch desk_cleaning mock_demo.launch
rostopic pub -1 /desk_cleaning/voice_text std_msgs/String "data: '清理桌面'"
```

不连接硬件，先启动相机驱动，再启动规划模式：

```bash
roslaunch desk_cleaning desk_cleaning.launch \
  start_arm:=false dry_run:=true \
  model_path:=/home/ubuntu/models/desk_objects.onnx
```

发送测试语言命令：

```bash
rostopic pub -1 /desk_cleaning/voice_text std_msgs/String "data: '清理桌面'"
rostopic pub -1 /desk_cleaning/voice_text std_msgs/String "data: '停止清理'"
```

不经过语音入口：

```bash
rosservice call /desk_cleaning/start "{}"
rosservice call /desk_cleaning/stop "{}"
```

查看感知结果和任务状态：

```bash
rostopic echo /desk_perception/objects
rostopic echo /desk_cleaning/clean_desk/feedback
rostopic echo /desk_cleaning/speech
```

确认所有检查通过后，才允许实体运动：

```bash
roslaunch desk_cleaning desk_cleaning.launch \
  start_arm:=true dry_run:=false \
  use_hardware_tts:=true use_usb_asr:=true use_hardware_asr:=false \
  asr_model_path:=/home/ubuntu/whisper.cpp/models/ggml-base.bin \
  asr_whisper_executable:=/home/ubuntu/whisper.cpp/build/bin/whisper-cli \
  asr_audio_device:=default \
  model_path:=/home/ubuntu/models/desk_objects.onnx \
  publish_camera_mount_tf:=true \
  camera_mount_frame:=camera_link \
  camera_mount_x:=... camera_mount_y:=... camera_mount_z:=... \
  camera_mount_yaw:=... camera_mount_pitch:=... camera_mount_roll:=...
```

## 本地语言模型

模型不是启动“清理”命令的必要条件。只有需要更自由的表达时才启用。启动一个提供
OpenAI 兼容 `/v1/chat/completions` 接口的本地推理服务，然后：

```bash
roslaunch desk_cleaning desk_cleaning.launch \
  llm_enabled:=true \
  llm_url:=http://127.0.0.1:8080/v1/chat/completions
```

建议选择量化的小型中文指令模型。提示词强制模型只返回 JSON，代码还会再次验证 intent、
类别和收纳位置白名单；任何额外字段都会被拒绝。

## USB 麦克风和 whisper.cpp

`usb_whisper_asr_node.py` 已将普通 USB 麦克风接入现有语音命令链路：

```text
USB 麦克风 -> arecord -> 能量 VAD -> whisper.cpp -> /desk_cleaning/voice_text
```

节点只在本机处理音频，不上传录音；临时 WAV 文件在每次识别后自动删除。它监听
`/desk_cleaning/speech`，机器人播报时会暂时丢弃麦克风音频，避免把自己的 TTS 当成新指令。

在树莓派上安装编译工具和 ALSA：

```bash
sudo apt update
sudo apt install -y alsa-utils git cmake build-essential
cd /home/ubuntu
git clone https://github.com/ggml-org/whisper.cpp.git
cd whisper.cpp
cmake -B build -DWHISPER_BUILD_TESTS=OFF
cmake --build build --config Release -j2
bash ./models/download-ggml-model.sh base
```

这里必须下载多语言 `base` 模型，不要下载只支持英文的 `base.en`。编译完成后的典型路径为：

```text
/home/ubuntu/whisper.cpp/build/bin/whisper-cli
/home/ubuntu/whisper.cpp/models/ggml-base.bin
```

插入 USB 麦克风后先查看 ALSA 设备：

```bash
arecord -l
arecord -L
```

先用 `default` 录制五秒并回放：

```bash
arecord -D default -f S16_LE -r 16000 -c 1 -d 5 /tmp/usb-mic.wav
aplay /tmp/usb-mic.wav
```

如果 `default` 不是 USB 麦克风，根据 `arecord -l` 的卡号和设备号改用例如
`plughw:2,0`。卡号在重启或插拔后可能变化，长期使用时建议在 ALSA 中为 USB 麦克风设置
固定名称。

编译工作空间后，可以先只测试语音转文字：

```bash
cd /home/ubuntu/armpi_fpv
catkin_make
source devel/setup.bash
roslaunch desk_cleaning usb_whisper_asr.launch \
  model_path:=/home/ubuntu/whisper.cpp/models/ggml-base.bin \
  whisper_executable:=/home/ubuntu/whisper.cpp/build/bin/whisper-cli \
  audio_device:=default
```

另一个终端查看识别文字：

```bash
source /home/ubuntu/armpi_fpv/devel/setup.bash
rostopic echo /desk_cleaning/voice_text
```

启动完整实体流程时使用：

```bash
roslaunch desk_cleaning desk_cleaning.launch \
  use_usb_asr:=true use_hardware_asr:=false \
  asr_model_path:=/home/ubuntu/whisper.cpp/models/ggml-base.bin \
  asr_whisper_executable:=/home/ubuntu/whisper.cpp/build/bin/whisper-cli \
  asr_audio_device:=default \
  start_arm:=true dry_run:=true \
  model_path:=/home/ubuntu/models/desk_objects.onnx
```

首次测试必须保持 `dry_run:=true`。确认识别、视觉坐标和目标选择都正确后，才改成
`dry_run:=false`。不要同时开启 `use_usb_asr` 和 `use_hardware_asr`，否则同一句命令可能被
发布两次。

语音切分参数位于 [whisper_asr.yaml](config/whisper_asr.yaml)。如果环境噪声导致自动触发，
提高 `vad/rms_threshold`；如果正常说话一直不触发，降低它。建议从默认值 `550` 开始，
麦克风距离保持在 0.5 到 1 米。`base` 在树莓派 5 上优先保证准确率；需要更低延迟时可把
模型换成 `tiny`。

## 硬件 ASR 和 TTS

当前 ArmPi I2C 语音识别模块是关键词识别，不是连续语音转写。
[asr.yaml](config/asr.yaml) 为它配置“清理桌面、停止清理、继续清理”等固定词条。
首次烧录词表时将 `configure_vocabulary` 改为 `true`，成功后再改回 `false`。

USB 麦克风的连续语音识别已经由上述 `usb_whisper_asr_node.py` 提供。TTS 节点会将执行器
的结果文本切分后发送给原有 I2C 语音合成模块。

## 已知边界

- 当前使用检测框内最大轮廓的长轴估计 `grasp_yaw`；复杂背景下建议升级为实例分割模型，
  以提高笔、尺子等长条物体的方向精度。
- 当前没有夹爪力传感器，动作执行成功不等于物理抓取一定成功。建议后续加入抓取后复检、
  舵机负载或夹爪传感器。
- 透明、反光、纯黑和很薄的物品可能导致深度缺失。
- 禁止默认抓取剪刀、手机、水杯；只有完成专用抓取测试后才能在 `objects.yaml` 中启用。
- 不应在有人手进入工作区、桌面边缘没有挡板或机械臂未固定时运行自动清理。
