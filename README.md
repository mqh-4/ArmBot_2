# ArmBot 水果识别、语音输入与机械臂夹取

ArmBot 是一个 ROS Noetic + Docker 项目。它使用 Orbbec Astra Pro Plus 彩色/深度相机识别苹果、橙子和香蕉，利用手眼标定把相机坐标转换到机械臂坐标，再由 ArmPi FPV 兼容机械臂完成夹取和放置。

本文是当前项目的完整运行手册，覆盖实际链路：

```text
ArmPi ASR -> “开始作业” -> 任务规划 -> RGB-D/YOLO 识别
          -> 工作空间/尺寸/IK 安全检查 -> 机械臂夹取与放置
          -> ArmPi TTS 播报 + Tk 可视化窗口
```

## 1. 硬件和目录

需要准备：

- Raspberry Pi 4B/5 或兼容 Linux 主机；
- Docker 和 Docker Compose；
- ArmPi FPV 兼容机械臂、舵机控制板和稳定电源；
- Orbbec Astra Pro Plus RGB-D 相机；
- ArmPi I2C 语音识别模块；
- ArmPi I2C 语音合成模块（需要机器人播报时）；
- 可以显示 X11 窗口的桌面会话。

项目目录固定为：

```text
/home/pi/docker/tmp/ArmBot
```

同级的 `ArmPi-FPV` 不属于运行依赖。Docker 容器内的工作区是 `/home/ubuntu/share/tmp/ArmBot`。不要在宿主机直接 source 旧的 `devel/setup.*`。本文使用的是本机已部署的 ARM64 环境；仓库包含架构相关的运动学二进制库，不代表任意主机架构均可直接运行。

本手册的语音输入使用 ArmPi I2C ASR 模块。普通 USB 麦克风不能直接替代该模块；USB Whisper 是另一条可选链路，需要另行安装 `whisper-cli` 和 GGML 模型，本文不启用它。

从 Git 新克隆项目后，运行资源的提供方式如下：

| 资源 | 用途和提交方式 |
|---|---|
| `models/yolov8s.onnx` | 默认水果检测权重，约 43 MiB，已随仓库提供 |
| `models/whisper-small/model.bin` | 可选 AI_pkg 语音模型，约 461 MiB；固定 ASR 关键词链路不用它，可通过下载工具恢复 |
| `models/whisper-small/config.json`、`tokenizer.json`、`vocabulary.txt` | 与 Whisper 权重匹配的模型配置，随仓库提供 |
| `output/eye_to_hand_result.json` | 随仓库提供的默认手眼标定结果；仅适用于对应的相机和机械臂布局 |
| `config/camera_info/astra_color.yaml`、`astra_ir.yaml` | 相机标定配置，保留在仓库内 |

本文的水果夹取主链路使用仓库内的 YOLO 权重和硬件 ASR，不需要下载 Whisper。获取项目后，可先校验主链路权重：

```bash
cd /home/pi/docker/tmp/ArmBot
python3 tools/prepare_models.py --check
```

如果需要单独运行可选的 AI_pkg / faster-whisper 语音功能，再执行：

```bash
python3 tools/prepare_models.py --with-ai
python3 tools/check_standalone.py --with-ai
```

下载工具使用 `models/manifest.json` 中的文件大小和 SHA-256 校验模型，只在校验通过后安装，不覆盖已有的错误文件。可以用 `--endpoint https://hf-mirror.com` 选择 Hugging Face 镜像。模型来源和使用边界见 [模型说明](models/README.md)。替换检测模型时必须保持 COCO 80 类输出顺序和加载器兼容性。

## 2. 运行前检查

固定机械臂底座，把水果和红色边界苹果盒放到相机能看到、机械臂能够到达的桌面区域。运行静态检查：

```bash
cd /home/pi/docker/tmp/ArmBot
python3 tools/check_standalone.py
```

必须看到：

```text
RESULT: PASS (static inputs only)
```

该检查只检查文件、路径、Docker 配置和 ROS 包引用，不会连接相机或移动机械臂。

## 3. 构建镜像

首次部署，或者修改了 `package.xml`、消息、服务、Action、C++ 节点时执行：

```bash
cd /home/pi/docker/tmp/ArmBot
docker compose build armpi_fpv
docker compose up -d armpi_fpv
```

也可以直接启动并构建：

```bash
docker compose up -d --build armpi_fpv
```

项目的 `src/`、`config/`、`models/`、`output/` 和 `tmp/` 会挂载到容器。launch、YAML 和 Python 脚本修改通常在下一次启动时生效；消息定义修改必须重新构建。

使用可视化时，应从树莓派本地桌面终端执行启动命令。纯 SSH 会话没有可用显示服务时，不能仅通过设置 `DISPLAY=:0` 保证窗口显示。

## 4. 首次写入 ASR 关键词

硬件 ASR 是关键词识别模块，不是连续语音转写。当前词表为：

| 编号 | 说法 | 模块拼音 |
|---|---|---|
| 1 | 开始作业 | `kai shi zuo ye` |
| 2 | 停止清理 | `ting zhi qing li` |
| 3 | 继续清理 | `ji xu qing li` |

第一次部署时写入一次，词表会保存在模块中：

```bash
cd /home/pi/docker/tmp/ArmBot

docker compose exec -T armpi_fpv bash -lc '
  source /opt/ros/noetic/setup.bash
  source devel/setup.bash
  python3 - <<"PY"
from armpi_fpv_common.asr import ASR

asr = ASR(bus=1)
asr.eraseWords()
asr.setMode(1)
asr.addWords(1, "kai shi zuo ye")
asr.addWords(2, "ting zhi qing li")
asr.addWords(3, "ji xu qing li")
print("ASR vocabulary configured")
PY
'
```

该操作会覆盖模块原有关键词。底层 ASR 驱动会忽略部分 I2C 写入错误，因此上述打印只表示配置调用结束；实际识别是否生效，需要在后续启动日志中确认 `ASR command 1: 开始作业`。容器必须能访问并读写 `/dev/i2c-1`；仅挂载 `/dev` 不等于已经具备设备访问权限，遇到 `Operation not permitted` 时应检查主机 I2C 和 Docker 设备权限配置。

正常运行时保持 `src/desk_cleaning/config/asr.yaml` 中的：

```yaml
configure_vocabulary: false
```

不要在每次启动时重复擦写词表。

## 5. 干运行预检

干运行会启动相机、识别、可视化、语音链路和任务规划。`dry_run:=true` 仅禁止水果执行器发送夹取运动；下面的 `start_arm:=true` 仍会启动底层控制器，不能把它视为整个系统的硬件隔离。先用它检查视觉和 TF：

```bash
cd /home/pi/docker/tmp/ArmBot
xhost +SI:localuser:root
docker compose up -d armpi_fpv

docker compose exec \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  armpi_fpv bash -lc '
    source /opt/ros/noetic/setup.bash
    source devel/setup.bash
    roslaunch desk_cleaning armpi_fpv_project.launch \
      start_arm:=true \
      dry_run:=true \
      visualizer:=true \
      use_hardware_asr:=true \
      use_hardware_tts:=true \
      use_usb_asr:=false \
      llm_enabled:=false
  '
```

窗口标题为 `ArmBot Camera`。绿色水果表示深度和坐标通过初步工作空间检查，紫色框表示苹果盒边界，`OUT` 表示坐标超出工作空间。执行器还会额外检查置信度、尺寸、稳定帧数和 IK，因此绿色不等于最终一定允许抓取。

如果出现 `No module named tkinter`，重新构建镜像：

```bash
docker compose build --no-cache armpi_fpv
docker compose up -d --force-recreate armpi_fpv
```

## 6. 真实语音夹取

确认干运行中彩色图、深度图、红色盒子边界和水果位置稳定后，按 `Ctrl-C` 停止干运行，再执行真实启动：

```bash
cd /home/pi/docker/tmp/ArmBot
xhost +SI:localuser:root
docker compose up -d armpi_fpv

docker compose exec \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  armpi_fpv bash -lc '
    source /opt/ros/noetic/setup.bash
    source devel/setup.bash
    roslaunch desk_cleaning armpi_fpv_project.launch \
      start_arm:=true \
      dry_run:=false \
      visualizer:=true \
      use_hardware_asr:=true \
      use_hardware_tts:=true \
      use_usb_asr:=false \
      llm_enabled:=false
  '
```

真实启动应看到类似日志：

```text
desk cleaning executor ready (dry_run=False, simulation=False)
voice task planner ready
desk visualizer image topic: /astra/color/image_raw
```

然后对 ASR 模块说：

```text
开始作业
```

完整流程如下：

1. ASR 识别编号 1，并发布 `/desk_cleaning/voice_text`；
2. 任务规划器把命令解析为苹果、橙子、香蕉，目标位置为 `apple_box`；
3. 执行器通过 TTS 播报“开始清理桌面”；
4. 感知节点同步读取 `/astra/color/image_raw`、`/astra/depth/image_raw` 和 CameraInfo；
5. 执行器等待稳定的红色盒子边界和水果检测；
6. 执行开爪、接近、下降、闭爪、抬升、移到盒子、松爪和回到安全姿态；
7. 通过视觉复检确认目标离开原位置，并播报结果。

“开始作业”表示处理所有启用的水果类别。当前 `objects.yaml` 只允许 `apple`、`orange`、`banana` 自动抓取。

程序不会在启动时主动播报“请说开始作业”；“开始清理桌面”是在接收到任务后发布的，执行器也不会等待 TTS 播报完毕才扫描。不要在任务进行中反复说“开始作业”，新任务会触发 Action 的替换/取消流程。当前“继续清理”会重新提交上一次目标，不是从某个已暂停的动作步骤原地恢复。

## 7. 停止和关闭

说“停止清理”会取消当前 ROS Action 并播报停止状态。它不是物理急停；出现碰撞、异常姿态、供电问题或人员进入工作区时，应立即使用机械臂控制板或外部急停方式切断运动。

关闭当前 ROS 启动：

```text
Ctrl-C
```

停止容器但保留镜像和项目数据：

```bash
cd /home/pi/docker/tmp/ArmBot
docker compose stop armpi_fpv
```

## 8. 关键文件和话题

| 用途 | 文件或话题 |
|---|---|
| 总启动入口 | `src/desk_cleaning/launch/armpi_fpv_project.launch` |
| 水果白名单 | `src/desk_cleaning/config/objects.yaml` |
| 工作空间、抓取和安全参数 | `src/desk_cleaning/config/workspace.yaml` |
| 相机和 YOLO 参数 | `src/desk_cleaning/config/perception.yaml` |
| ASR 关键词 | `src/desk_cleaning/config/asr.yaml` |
| 手眼标定 | `output/eye_to_hand_result.json` |
| 彩色图 | `/astra/color/image_raw` |
| 深度图 | `/astra/depth/image_raw` |
| CameraInfo | `/astra/color/camera_info` |
| 识别结果 | `/desk_perception/objects` |
| 语音输入 | `/desk_cleaning/voice_text` |
| TTS 文本 | `/desk_cleaning/speech` |
| 清理 Action | `/desk_cleaning/clean_desk` |

## 9. 故障排查

### 说“开始作业”没有反应

确认启动命令含有 `use_hardware_asr:=true`，词表已经写入，并检查日志是否出现：

```text
ASR command 1: 开始作业
```

还要确认 I2C 总线为 bus 1，且没有其他程序占用 ASR 模块。

### 机器人没有播报

启动命令必须含有 `use_hardware_tts:=true`。只看到 `robot says:` 日志而没有声音时，检查 TTS 模块、电源、I2C 接线和音量。ASR 地址是 `0x79`，TTS 地址是 `0x40`。

### 相机出现 Depth processor claimed failed

该错误出现在深度流启动阶段，可能与流配置、设备占用、固件状态或 USB 连接/供电有关；仅凭这条报错不能确定根因。停止重复的相机进程，检查端口、供电和流配置，再确认以下话题持续发布：

```text
/astra/color/image_raw
/astra/depth/image_raw
/astra/color/camera_info
```

### 能识别但不夹取

执行器会拒绝深度无效、置信度低、尺寸过大、坐标超出工作空间、红色盒子边界不稳定或 IK 无解的目标。应先根据可视化和日志调整相机安装、手眼标定、桌面位置、盒子位置和工作空间，不要关闭安全检查强行运动。

### 出现 Cannot load message class

必须在容器内 source ROS 和项目工作区：

```bash
docker compose exec armpi_fpv bash -lc '
  source /opt/ros/noetic/setup.bash
  source devel/setup.bash
  rospack find desk_cleaning
'
```

修改消息、服务或 Action 后重新构建：

```bash
docker compose build armpi_fpv
docker compose up -d --force-recreate armpi_fpv
```

## 10. 运行边界

当前使用 COCO YOLO 模型中的水果类别，不是针对每个实际水果和桌面场景训练的专用三分类模型。透明、反光、遮挡、纯黑物体和深度孔洞可能导致识别或定位失败。机械臂没有夹爪力传感器，视觉复检只能确认目标是否离开原位置，不能保证物理抓取质量。

每次修改相机安装位置、机械臂底座、桌面高度、盒子位置或相机内参后，都必须重新检查手眼标定和工作空间，再使用 `dry_run:=false`。
