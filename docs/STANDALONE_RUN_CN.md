# ArmBot 独立运行检查与启动说明

> 本文件保留此前独立部署与联调的历史记录。当前启动步骤、语音参数和模型获取方式以根目录 [中文 README](../README.md) 为准。水果主链路模型已随 Git 提供，可选 Whisper 可用 `tools/prepare_models.py --with-ai` 恢复；静态检查默认检查水果链路，`--with-ai` 才额外检查 Whisper。历史记录中的容器状态不代表当前运行状态。

## 结论与目录关系

本机实际项目目录是 `/home/pi/docker/tmp/ArmBot`（注意大小写）。首次检查时，同级路径的关系为：

```text
/home/pi/docker/tmp/
├── ArmBot/                 实际文件、源码、模型与配置
└── ArmPi-FPV -> ArmBot     旧名称的软链接
```

两个入口访问同一份文件；`ArmPi-FPV` 本身没有第二份项目代码。ArmBot 的 Docker 构建与项目数据挂载直接使用 `ArmBot`，因此主流程的源码和部署配置不依赖这个旧入口。删除旧软链接不会释放一份项目所占用的磁盘空间。

用户现已使用 `unlink` 删除旧软链接，删除后独立路径检查全部通过。补充核实确认宿主机旧入口已不存在，ArmBot 目录正常，新容器继续为 Up；容器内 `desk_cleaning` 和 `AI_pkg` 均可正常定位到 ArmBot 工作区。

已完成源码、启动资源、数据文件和 Compose 路径检查。用户随后从 ArmBot 成功构建 `armbot:noetic`，并启动新容器 `armbot-armpi_fpv-1`；补充的容器内检查确认编译产物和实际挂载全部使用 ArmBot，容器内不存在 `/home/ubuntu/share/tmp/ArmPi-FPV`。**现已验证独立构建、容器启动、运行依赖及自动测试，并确认宿主机旧入口删除后工作区仍正常；真实相机、ROS 应用启动和实体机械臂仍未在本次验收。** 额外的 ROS 启动检查命令受会话权限限制未能执行。实体机械臂未操作。

## 必须保留的项目内容

| 内容 | 作用 | 是否位于 ArmBot 内 |
| --- | --- | --- |
| `src/desk_cleaning` | RGB-D 水果识别、定位、抓放与标定 | 是 |
| `src/orbbec_camera` | Orbbec Astra 相机驱动及 SDK | 是 |
| `src/ros_robot_controller`、`src/hiwonder_servo_*` | 控制板、总线舵机与消息 | 是 |
| `src/armpi_fpv_*` | 机器人描述、运动学与保留的 ROS 包 | 是 |
| `src/AI_pkg` | FastMCP、模型接口、录音与动作工具 | 是 |
| `models/yolov8s.onnx` | 主流程默认视觉模型 | 是 |
| `models/whisper-small/` | AI_pkg 的本地语音识别模型 | 是 |
| `output/eye_to_hand_result.json` | 主流程默认手眼标定文件 | 是 |
| `src/AI_pkg/config/actions/*.bag` | 本地动作示教文件 | 是 |
| `Dockerfile`、`requirements.txt`、ROS keyring | 构建 Noetic 与 Python 依赖 | 是 |

ROS 包名、launch 名和 Compose 服务名中的 `armpi_fpv` 是现有接口名称，继续保留即可。尤其不要把项目内的 `src/armpi_fpv_*` 当作同级旧软链接一起删除。

宿主机 `/dev` 和 `/tmp/.X11-unix` 分别用于设备与图形显示，属于系统资源挂载。镜像中的 `/opt/ros/noetic`、`/opt/opencv-4.9`、`/opt/ai-python` 由 Docker 构建步骤提供，不来自同级项目。

`src/CMakeLists.txt` 指向 `/opt/ros/noetic/share/catkin/cmake/toplevel.cmake`。本机宿主环境没有 ROS，因此这个链接在宿主机看起来不存在；目标由 `ros:noetic` 镜像提供。这不是对另一个项目目录的依赖。

## 本次修正

- `.zshrc` 使用 `ROS_WS` 或 ArmBot 路径查找 `.robotrc`，移除旧名称入口的依赖。
- `.robotrc` 的 Bash 分支从错误的 `/opt/ros_ws/noetic/setup.bash` 改为 `/opt/ros/noetic/setup.bash`。
- `command` 中四条旧工作区绝对路径改用 ArmBot。
- 中英文 README 的语言切换链接改为本地文件，并增加本文入口。
- 新增 `tools/check_standalone.py`，可重复检查本地部署输入。

## 旧 build/devel 的处理

宿主机项目中的 `build/CMakeCache.txt`、`devel/.catkin` 等仍记录旧路径 `/home/ubuntu/share/tmp/ArmPi-FPV`。它们是历史编译产物，不能直接迁移到新路径使用，也不应通过批量替换编译产物中的字符串来修复。

当前 `.dockerignore` 排除了宿主机的 `build/`、`devel/`，Dockerfile 仅复制 `src/` 等构建输入，并在 `/home/ubuntu/share/tmp/ArmBot` 执行 `catkin_make`。Compose 也没有把宿主机 `build/`、`devel/` 挂载到容器。因此推荐从 ArmBot 构建镜像，使用镜像内的新产物。宿主机旧产物在本次检查中保留原样。

不要在宿主机直接 `source devel/setup.bash` 作为启动方式。本机宿主环境是 Debian/Python 3.11，项目使用 Docker 中的 ROS Noetic/Python 3.8，并为 AI 节点另外安装 Python 3.11。

## 本次验证范围

| 检查 | 结果 |
| --- | --- |
| 项目中的 ROS 包 | 19 个包；主流程所需包齐全 |
| desk_cleaning、AI_pkg 的 launch 及递归包含的本地资源 | 路径检查通过 |
| 源码与运行配置中的旧工作区路径 | 已修正，无旧入口路径引用 |
| Compose 配置解析、构建上下文和项目挂载来源 | 通过，全部直接来自 ArmBot |
| 不存在同级 ArmPi-FPV 的临时副本 | 复制源码、模型和配置，不带 build/devel；独立路径检查通过，随后清除临时副本 |
| 项目内部源码软链接 | 通过；仅 catkin 顶层链接依赖 ROS 安装 |
| `desk_cleaning` 自动测试 | 宿主机及新容器内均为 40 项通过 |
| AI 自动测试 | 新容器 Python 3.11 中完整 `test_*.py` 测试 38 项通过，包含动作示教测试 |
| 主视觉模型 | 宿主机及新容器内 OpenCV 4.9.0 均成功加载 `yolov8s.onnx` |
| Whisper 模型 | `model.bin` SHA-256 与项目校验文件一致 |
| 主流程标定 JSON | 可解析，含 4 行变换矩阵与 `base_link` |
| `src` 中 XML、launch、URDF | 105 个文件解析通过 |
| Docker 镜像重建及新容器启动 | 用户执行成功；`docker compose ps` 确认新容器为 Up |
| 新容器内 `devel/.catkin`、ROS 包定位 | 均指向 `/home/ubuntu/share/tmp/ArmBot/src` |
| 新容器内项目挂载 | `/proc/self/mountinfo` 确认 src/config/models/output/tmp 均直接来自宿主机 ArmBot |
| 新容器内旧入口 | `/home/ubuntu/share/tmp/ArmPi-FPV` 不存在 |
| 宿主机旧入口删除后复查 | ArmPi-FPV 已不存在；独立路径检查通过，容器继续 Up，desk_cleaning/AI_pkg 定位正常 |
| 新容器内 AI 依赖 | FastMCP、OpenAI SDK、faster-whisper 导入成功 |
| ROS 应用启动、相机与硬件 | 本次未验收；额外 ROS 启动检查命令受会话权限限制 |

宿主机没有 `fastmcp` 和 `openai`，它们由镜像构建时的 `src/AI_pkg/scripts/install_ai_runtime.sh` 安装。新容器中的完整 AI 测试已通过，涵盖本地 FastMCP 工具链与模型接口替身，不代表真实云端 API、麦克风或实体动作已验收。

这里的独立运行范围是 `desk_cleaning` 主流程和 `AI_pkg` 的 Docker 部署。保留的上游教程并非全部可直接运行：例如 `command` 中课程软件路径、旧示例中的 `/home/ubuntu/armpi_fpv` 路径仍需要各自的环境；相机驱动中也存在未随附模型文件的 Gemini 335Lg 演示入口。这些不属于 Astra 主流程，也不是对同级 ArmPi-FPV 的依赖。

旧的 `use_usb_asr:=true` 路线依赖单独安装的 `whisper.cpp` 和 GGML 模型；默认关闭，也没有由当前 Dockerfile 安装。`AI_pkg` 使用项目内的 faster-whisper 模型，是另一条语音路线。云端问答仍需要运行时提供有效的 API key 和网络。

## 从 ArmBot 构建与验证

在有 Docker 服务访问权限的宿主机终端执行：

```bash
cd /home/pi/docker/tmp/ArmBot
python3 tools/check_standalone.py
docker compose build armpi_fpv
docker compose up -d armpi_fpv
```

镜像首次构建需要联网安装系统和 Python 依赖。`up` 启动工作区容器；当前 Compose 命令只保持 Bash 会话，不会自动启动完整相机、机械臂和 AI 节点。若已有业务节点正在容器中运行，应安排好重建时机，容器更新会结束旧容器中的进程。

先做容器内只读检查，无需启动实体机械臂：

```bash
docker compose exec -T armpi_fpv bash -lc '
  set -e
  source /opt/ros/noetic/setup.bash
  source devel/setup.bash
  python3 -c "from pathlib import Path; p=Path(\"devel/.catkin\").read_text(); assert \"ArmPi-FPV\" not in p, p; print(p)"
  rospack find desk_cleaning
  rospack find AI_pkg
  rospack find armpi_fpv_kinematics
  PYTHONPATH=/opt/opencv-4.9:$PYTHONPATH python3 -c "import cv2; net=cv2.dnn.readNetFromONNX(\"models/yolov8s.onnx\"); assert not net.empty(); print(cv2.__version__)"
  /opt/ai-python/bin/python -c "import fastmcp, openai, faster_whisper; print(\"AI imports OK\")"
'
```

输出的包目录应位于 `/home/ubuntu/share/tmp/ArmBot/src`。如需核实实际容器挂载，可运行 `docker inspect armbot-armpi_fpv-1 --format '{{json .Mounts}}'`；如果容器使用了自定义项目名，先用 `docker compose ps` 确认容器名。

连接好 Astra 后，在前台启动无实体机械臂动作的主流程：

```bash
docker compose exec armpi_fpv bash -lc '
  source /opt/ros/noetic/setup.bash
  source devel/setup.bash
  roslaunch desk_cleaning armpi_fpv_project.launch \
    start_arm:=false dry_run:=true visualizer:=false
'
```

另一个终端可检查 `/desk_perception/ready`，或在相同工作区中启动 `roslaunch AI_pkg ai.launch`。实体机械臂动作和装配后的标定精度需要另行验收。

## 旧入口删除记录

用户已确认链接目标，并执行以下 `unlink` 命令删除旧入口。保留命令作为操作记录，无需再次执行：

```bash
ls -ld /home/pi/docker/tmp/ArmPi-FPV
readlink /home/pi/docker/tmp/ArmPi-FPV
# 确认仍是软链接后，只移除这个入口；路径末尾不要加斜杠。
test -L /home/pi/docker/tmp/ArmPi-FPV && unlink /home/pi/docker/tmp/ArmPi-FPV
```

使用 `unlink` 只删除软链接。不要进入旧路径后删除其中的文件，因为那会修改实际 ArmBot 项目。旧 ZIP 备份和历史文档不会因本次源码修正自动更新，它们可能仍包含旧路径。

## 首次真实启动后的相机断连排查

删除旧入口后，用户已启动真实主流程，控制板、舵机管理器、运动学和感知节点启动，相机也识别为 Astra Pro Plus，但在开始图像流后反复断连。用户随后按 Ctrl+C 正常关闭主流程。

宿主机内核在相同时段连续报告 `over-current change #1` 至 `#4`，相机内置 USB Hub、彩色相机、深度设备、USB 声卡、键盘和鼠标均发生断开重枚举。这是 USB 供电/过流状态异常的直接证据，需要检查电源、设备负载、线缆和接口；尚未确定具体故障部件，不能用软件重连或提高 USB 电流上限代替硬件排查。

设备型号确认为 Raspberry Pi 5 Model B Rev 1.0。启动设备树记录 `chosen/power/max_current=3000`（mA）、`usb_max_current_enable=0`，表示当前启动记录没有启用高 USB 电流额度；这支持优先检查树莓派供电能力，但不能单凭这些启动属性确定实际故障部件。当前负载包含 Astra 相机、USB 声卡、键盘和鼠标。

另外，新容器未保留 `/root/.ros/camera_info` 中的临时文件，造成标定文件缺失警告。主入口 `armpi_fpv_project.launch` 现已显式通过 `color_info_uri`、`ir_info_uri` 读取项目 `config/camera_info/` 中原有的 YAML；没有修改标定数值。这一修改通过源码挂载生效，无需重建镜像。用户可通过同名 launch 参数覆盖路径。

应先停止抓取任务，检查树莓派电源和相机 USB 线，移除暂时不需要的 USB 外设；必要时使用合规的带独立电源 USB Hub。供电接线处理后，先用 `start_arm:=false dry_run:=true` 验证相机持续取流，再恢复实体抓取。当前 USB 问题修复后的持续取流尚未验收。

查看最近 USB 断连与过流事件：

```bash
dmesg --ctime | tail -120
```

再次测试时相机路径由 `3-1.1` 变为 `1-1.1`，深度流先报 `OB_SENSOR_DEPTH config video mode failed`，后续重试报 `Firmware stream Depth processor claimed failed`。同一时段内核新增 `over-current change #5` 至 `#7`，相机、声卡和键鼠再次同时重枚举，说明更换 USB 接口后 USB 电气/供电状态异常仍在。用户停止程序后，当前容器和可见进程列表中未发现残留相机进程；这不能证明报错发生时完全没有占用。

驱动 `ob_camera_node.cpp` 的 `startStreams()` 在首次 pipeline 启动异常后固定输出“try to disable ir stream try again”，然后立即再次启动。日志中的 IR 本已关闭，因此这条提示不表示配置实际启用了 IR；后续 claimed 错误也不足以单独证明有第二个程序占用。先处理 USB 供电、线缆和故障设备，并在退出驱动后让相机真正断电再接入；供电稳定后若仍报深度模式错误，再独立排查流配置和固件状态。

## 可视化 tkinter 依赖修复

单独启动 `desk_visualizer.launch` 曾因 `ModuleNotFoundError: No module named 'tkinter'` 退出。脚本使用 Tk 窗口及 `PhotoImage` 显示 OpenCV 绘制的图像，需要 Ubuntu 的 `python3-tk`。

已在当前运行容器中安装 `python3-tk`，同时将它加入 Dockerfile 和 `desk_cleaning/package.xml` 的运行依赖。容器内已验证可视化脚本所有导入、Tk 8.6、X11 显示连接及 PPM 图像创建成功，测试窗口已关闭；没有启动机械臂或相机取流。

当前容器可直接重新运行可视化，无需先重建。原 `armbot:noetic` 镜像尚未因本次依赖修改重新构建；将来删除或重建容器前，先执行 `docker compose build armpi_fpv`，让新镜像包含依赖。此前的 USB 断连是独立问题，窗口能够打开不代表相机取流已经恢复。
