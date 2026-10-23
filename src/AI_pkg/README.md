# AI_pkg：语音、FastMCP 与 ROS

参考本目录中用户提供的 `voice_assistant.py`、`voice_mcp_assistant.py`、`mcp_server.py`、`mcp_client.py` 实现：

```text
按键录音（a 开始、s 停止）→ arecord → faster-whisper → 转写话题
                                          ↓
文本输入话题 ─────────────────────→ DeepSeek/兼容模型接口
                                          ↓ 工具发现与调用
                                   FastMCP Client
                                          ↓ HTTP /mcp
                                   FastMCP Server
                                          ↓
                              add / ROS 话题与感知状态查询
                                          ↓
                                   最终回答 output 话题
```

## 实现文件

- `scripts/mcp_server_node.py`、`scripts/llm_agent_node.py`：ROS 可执行脚本。
- `src/ai_pkg/nodes.py`：两个节点的实际实现，同时供 ROS2 console entry points 调用。
- `src/ai_pkg/ros_runtime.py`：ROS1 `rospy` / ROS2 `rclpy` 的参数、话题、Trigger 服务和生命周期适配。
- `src/ai_pkg/tools.py`：真正的 `fastmcp.FastMCP` 服务及工具注册；移植参考文件的 `add(a, b)`。
- `src/ai_pkg/agent.py`：真正的 `fastmcp.Client`、模型 SDK、工具 schema 转换与有限轮工具调用循环。
- `src/ai_pkg/speech.py`：16 kHz 单声道 PCM16 录音、临时 WAV 清理、Whisper CPU int8、模型复用。
- `scripts/teach_motion_bag.py`、`src/ai_pkg/motion_teach.py`：拖拽示教录制，以及裁剪/重采样/生成舵机指令的纯函数。

用户提供的参考文件保留原样，构建时不复制进运行包；新节点不会 import 其中含全局客户端初始化和交互式 input() 的示例。新实现只从环境变量读取 API key。

## 运行条件与当前环境

**可以直接在现有 `armpi_fpv` 容器运行，已完成真实 ROS1 与 FastMCP 联调。** 系统 `/usr/bin/python3` 仍是 Python 3.8，供 Noetic 和原有节点使用；两个 AI 节点通过独立的 `/opt/ai-python/bin/python`（Python 3.11）运行。`rospy`、`std_msgs`、`std_srvs` 的纯 Python 模块沿用 Noetic 安装，AI 运行时单独安装其 `rospkg`、`catkin-pkg`、PyYAML 依赖。

Noetic 的旧日志调用栈查找在 Python 3.11 下可能无限循环，`ros_runtime.py` 仅在 AI 进程内应用兼容处理，不修改系统 ROS 文件。这一方案已验证本包所用的话题和 Trigger 接口；不要直接用此解释器运行依赖 `cv_bridge`、`tf2_py` 等 Python 3.8 二进制扩展的其他节点。

## 在当前 Docker 中运行（推荐）

本次已将运行时安装到正在使用的容器，并启动两个 AI 节点。查看日志和接口：

```bash
cd /home/pi/docker/tmp/ArmBot
docker compose exec armpi_fpv tail -f /tmp/ai-pkg-nodes.log
# 另开终端：
docker compose exec armpi_fpv bash -lc '
  source /opt/ros/noetic/setup.bash
  source devel/setup.bash
  rosnode list
  rosservice list | grep llm_agent
'
```

旧镜像创建的新容器需要先安装一次；普通源码修改不需要重新安装：

```bash
docker compose exec armpi_fpv bash -lc 'bash src/AI_pkg/scripts/install_ai_runtime.sh'
```

安装脚本使用清华 Miniconda ARM64 镜像及清华 PyPI，校验固定安装器的 SHA256，不替换系统 Python。Dockerfile 已加入同一安装步骤；下次 `docker compose build armpi_fpv` 构建的镜像会包含运行时。**本轮在现有容器完成安装和运行验证，尚未重新构建整张镜像；若现在删除容器，需用上述脚本重新安装，或先构建新镜像。**

容器重启后，先启动 roscore，再启动 AI（已有 roscore 时不要重复启动）：

```bash
docker compose exec armpi_fpv bash -lc 'source /opt/ros/noetic/setup.bash && roscore'
# 另一个终端：
docker compose exec armpi_fpv bash -lc '
  source /opt/ros/noetic/setup.bash
  source devel/setup.bash
  roslaunch AI_pkg ai.launch
'
```

云端问答需要把 API key 设置到启动节点的同一个容器终端。当前后台节点未配置密钥；要切换到云端，在容器内停止这两个 AI 节点后重新启动，不影响相机节点：

```bash
docker compose exec armpi_fpv bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosnode kill /llm_agent_node /mcp_server_node
read -r -s -p 'DeepSeek API key: ' DEEPSEEK_API_KEY; echo
export DEEPSEEK_API_KEY
roslaunch AI_pkg ai.launch llm_model:=deepseek-flash
```

模型名来自参考示例，需改成账号实际可用的名称；未验证该名称的在线可用性。密钥只从环境变量读取，不存入 ROS 参数或代码。启动本身不录音，也不请求云端模型。

查看回答和错误，分别在其他终端运行：

```bash
docker compose exec armpi_fpv bash -lc 'source /opt/ros/noetic/setup.bash && rostopic echo /llm_agent_node/output'
docker compose exec armpi_fpv bash -lc 'source /opt/ros/noetic/setup.bash && rostopic echo /llm_agent_node/error'
```

发送文本或触发一次固定时长录音：

```bash
docker compose exec armpi_fpv bash -lc 'source /opt/ros/noetic/setup.bash && rostopic pub -1 /llm_agent_node/input std_msgs/String "data: 请调用工具计算100加50"'
docker compose exec armpi_fpv bash -lc 'source /opt/ros/noetic/setup.bash && rosservice call /llm_agent_node/record "{}"'
```

交互式语音控制使用单独的键盘节点。它通过 ROS 服务控制 `llm_agent_node`，因此 AI 节点可以继续由
`roslaunch` 在后台运行。先确认 `llm_agent_node` 已启动，再在交互式容器终端运行：

```bash
docker compose exec -it armpi_fpv bash -lc '
  source /opt/ros/noetic/setup.bash
  source devel/setup.bash
  python3 src/AI_pkg/scripts/voice_keyboard.py
'
```

键盘节点的操作流程：

```text
a  开始录音
s  停止录音，执行 Whisper 转写并发送给 LLM/MCP
q  退出；如果正在录音，会先请求停止
```

交互式服务为 `/llm_agent_node/start_recording` 和 `/llm_agent_node/stop_recording`。录音最长 60 秒，
按 `s` 后会先在 `/llm_agent_node/transcript` 发布识别文本，再在 `/llm_agent_node/output` 发布最终回答。
说“点头”时模型应调用 `play_nod`，说“摇头”时应调用 `play_shake`。首次进行实体动作测试前必须确认
机械臂周围空间和急停可用。

实际录音前需要 `arecord -l` 列出输入设备。本轮已在重建后的容器中用 USB 声卡完成 3 秒录音和 `SpeechInput` 1 秒录音验证（16 kHz、单声道、PCM16）。Whisper 实际模型转写仍需首次下载或准备模型。Compose 已补充 ALSA 字符设备主号 116 的权限，与原 USB 189 权限并存；设备编号变化时优先使用卡名 `plughw:Device,0`。

ROS2 建议使用 Ubuntu 22.04 + ROS2 Humble / Python 3.10，或 Ubuntu 24.04 + ROS2 Jazzy / Python 3.12。需要 `rclpy`、`std_msgs`、`std_srvs`、`launch_ros`、`ament_index_python`、`python3-yaml`、`python3-venv`、`python3-pip` 及 ALSA `arecord`（`alsa-utils`）。Python AI 依赖见 `requirements.txt`。

Python 安装 `rclpy` 不能替代完整 ROS2 安装；应使用 ROS2 系统提供的匹配版本。ROS1 和 ROS2 的同名话题不会自动互通。若 ROS2 要读取当前 Noetic 的感知数据，需要另外部署 ROS1/ROS2 bridge 或将感知节点迁移到 ROS2。本改动不迁移机械臂、相机与其余 ROS1 包。

## 在独立 ROS2 环境构建（可选）

**以下命令在已安装 ROS2 Humble 的环境执行，不能直接在当前 Noetic 容器执行。** Jazzy 用户应替换 ROS 安装路径。使用独立工作区，避免让 colcon 构建整个 ROS1 工程。

```bash
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages /tmp/armbot-ai-ros2-venv
source /tmp/armbot-ai-ros2-venv/bin/activate
export PYTHONPATH=/tmp/armbot-ai-ros2-venv/lib/python3.10/site-packages:${PYTHONPATH:-}
python -m pip install --index-url https://pypi.tuna.tsinghua.edu.cn/simple \
  -r /home/pi/docker/tmp/ArmBot/src/AI_pkg/requirements.txt
mkdir -p /tmp/armbot-ai-ros2-ws/src
ln -s /home/pi/docker/tmp/ArmBot/src/AI_pkg /tmp/armbot-ai-ros2-ws/src/AI_pkg
cd /tmp/armbot-ai-ros2-ws
# 显式使用虚拟环境 Python，让 console scripts 使用安装了 AI 依赖的解释器。
python /usr/bin/colcon build --symlink-install --packages-select AI_pkg
source install/setup.bash
```

上面 site-packages 路径适用于 Python 3.10；Jazzy/Python 3.12 应改成 `python3.12`。已有工作区软链接时无需再次执行 `ln -s`。

包名沿用用户指定的 `AI_pkg`，ROS 工具可能提示推荐使用小写包名。

## 配置并启动

```bash
# 在上述 ROS2 与虚拟环境已 source 的终端中设置，密钥不放入 ROS 参数服务器。
# 输入密钥并回车，输入内容不会显示。
read -r -s DEEPSEEK_API_KEY
export DEEPSEEK_API_KEY
ros2 launch AI_pkg ai.launch.py \
  llm_base_url:=https://api.deepseek.com/v1 \
  llm_model:=deepseek-flash \
  audio_device:=default
```

模型名来自参考示例，需要替换为账号实际可用的模型。`base_url` 是 SDK 基础地址，不是完整的 `/chat/completions` URL。本地兼容服务可设 `http://127.0.0.1:8080/v1`。云端接口必须配置密钥；默认不自动发起网络模型请求。

`config/agent.yaml` 还可设置录音设备、录音秒数（1–60）、Whisper 模型名或本地路径、语言、下载镜像、超时、工具调用轮数、模型 `reasoning_effort` 和 `extra_body_json`。参考示例的 thinking 参数按需配置，默认不强制附加，以兼容不同模型。

`arecord -l` 可查看录音设备。当前配置默认使用 C-Media USB PnP 声卡的稳定名称 `plughw:Device,0`；如果设备名称不同，再改成实际设备，例如 `plughw:2,0`。启动不会自动录音或下载 Whisper；第一次触发录音转写时才加载模型。离线运行可设置本地模型路径及 `whisper.local_files_only=true`。

Whisper 模型必须在第一次语音转写前可访问。当前 AI 容器使用 `faster-whisper` 的 CTranslate2 模型目录，
不是 `whisper.cpp` 的 `ggml-*.bin` 文件。如果容器无法访问 Hugging Face 镜像，可在有网络的机器上下载
`Systran/faster-whisper-small` 完整目录，将目录复制到项目的 `models/whisper-small/`，然后在
`src/AI_pkg/config/agent.yaml` 设置：

```yaml
whisper:
  model: /home/ubuntu/share/tmp/ArmBot/models/whisper-small
  local_files_only: true
```

当前 `docker-compose.yml` 已将宿主机 `./models` 挂载到容器的 `/home/ubuntu/share/tmp/ArmBot/models`。
模型目录中至少应包含 `model.bin`、`config.json`、`tokenizer.json` 等 faster-whisper 文件。只复制一个
`ggml` 文件不能被 `faster-whisper` 直接加载。

2026-10-18 已重新下载并验证当前 `Systran/faster-whisper-small` 模型。`model.bin` 大小为
`483546902` 字节，SHA-256 为
`3e305921506d8872816023e4c273e75d2419fb89b24da97b4fe7bce14170d671`。
宿主机可在模型目录运行 `sha256sum -c model.bin.sha256`；更新为其他模型版本时应重新取得该版本的仓库校验值。
容器中的 CTranslate2 为 `4.5.0`，已通过本地模型加载验证。启动前可再次检查：

```bash
docker compose exec -T armpi_fpv /opt/ai-python/bin/python -c '
from faster_whisper import WhisperModel
WhisperModel("/home/ubuntu/share/tmp/ArmBot/models/whisper-small", device="cpu", compute_type="int8", local_files_only=True)
print("WHISPER_LOCAL_MODEL_OK")
'
```

## ROS2 接口与测试

| 接口 | 类型 | 说明 |
| --- | --- | --- |
| `/llm_agent_node/input` | `std_msgs/msg/String` | 直接文本问答 |
| `/llm_agent_node/record` | `std_srvs/srv/Trigger` | 提交一次定长录音、转写及问答任务 |
| `/llm_agent_node/transcript` | `std_msgs/msg/String` | 语音识别出的文本 |
| `/llm_agent_node/output` | `std_msgs/msg/String` | 模型最终回答 |
| `/llm_agent_node/error` | `std_msgs/msg/String` | 录音、转写、模型或 MCP 错误 |
| `/desk_perception/ready` | `std_msgs/msg/Bool` | MCP 服务端监听的感知状态 |

分别在已 source 相同 ROS2 工作区的终端中订阅转写、回答和错误：

```bash
ros2 topic echo /llm_agent_node/transcript
ros2 topic echo /llm_agent_node/output
ros2 topic echo /llm_agent_node/error
```

触发录音或文本提问：

```bash
ros2 service call /llm_agent_node/record std_srvs/srv/Trigger '{}'
ros2 topic pub --once /llm_agent_node/input std_msgs/msg/String "{data: '请计算100加50，必须调用工具'}"
```

Trigger 返回表示任务被队列接受，不表示录音、模型回答已经完成。工作线程一次处理一条任务，最多保留一条排队任务；过量请求返回失败。语音转写先发布，即使后续模型调用失败，也能从 transcript 查看识别结果。Ctrl+C 停止节点时会终止正在执行的 arecord 并清理临时录音。

## FastMCP 接口

服务默认监听 `http://127.0.0.1:8765/mcp`，使用独立 HTTP 服务便于两个 ROS 节点分别启动。与参考代码的 stdio 相比，客户端不再拉起第二份服务端子进程。服务目前只允许绑定回环地址。

工具为 `add(a: int, b: int)`、`list_ros_topics()`、`get_perception_status()`、
`list_motion_actions()`、`play_motion(action, repeat, speed)`、`play_nod()`、`play_shake()`
和 `stop_motion()`。

动作名不再硬编码：`list_motion_actions` 会扫描 `config/actions/*.bag`（不含子目录），
`play_motion` 只接受该列表里的名称，并校验名称只含小写字母、数字、`_`、`-`。
把新的 `<name>.bag` 放进 `config/actions/` 就能立即播放，**不需要重启**。
重复次数限制 1–3 次、速度 0.25–2.0 倍，同一时间只允许一个动作播放；模型不能传入任意路径或
shell 命令。备份请放在 `config/actions/backups/`，否则会被当成可播放动作列出来。
状态中的消息年龄是最近接收时间，不能单独当作硬件在线证明。ROS2 查询的是 ROS2 图；没有跨版本桥接时不会显示 Noetic 的节点和状态。

`llm.timeout` 只控制模型接口请求，默认 30 秒；`mcp_timeout` 独立控制 MCP 请求，默认 900 秒。
动作工具会等待 rosbag 播放结束后才返回，不能使用同一个 30 秒超时：当前 `nod.bag` 约 69 秒，
以最低允许速度 0.25 倍播放 3 次约需 828 秒。实际等待时间随动作时长、速度和重复次数变化。
客户端超时不代表服务端动作已经停止，不应直接重复发送动作口令；先查询播放状态，必要时调用
`stop_motion`。修改配置后需要重启 AI 助手节点。

## 人工示教录制 rosbag

动作由人示教录制，再作为 MCP 的固定动作工具资源。有两种方式：**拖拽示教**（推荐，动作连贯、包短）和**键盘示教**。

两种方式都只记录舵机指令，不会把相机、深度和 TF 写进动作包；录制前必须启动真实机械臂控制器。

### 方式一：拖拽示教（推荐）

舵机在力矩开启时是刚性的、手推不动，所以拖拽示教必须先卸载力矩。卸载后机械臂会变软下坠，
**必须先用一只手托住机械臂**，并一直托到脚本提示力矩已恢复。

#### 两个实测要点（2026-10-18）

1. **本机的卸载指令是有效的**：`bus_servo_enable_torque(False)`（`0x0C`）之后，在不发送任何
   读写的情况下，机械臂可以被动移动（实测 6 秒内肘部被推动 55 刻度）。
2. **但每次读取位置都会让该舵机重新使能**。所以"一边高频读位置一边拖拽"会把机械臂重新锁死，
   表现就是"状态显示已卸载、却完全推不动"。

因此录制器采用**交错采样**：每读一个舵机，立刻重新卸载该舵机；每轮结束再卸载全部。
6 个舵机一轮约 50 ms（约 20 Hz），每个舵机只有约 10% 时间被使能，其余时间保持松软。

一个容易踩的坑：`/servo_controllers/port_id_1/servo_states` 在本项目中是**假反馈**
（`hiwonder_servo_controller.yaml` 里 `fake_read: true`，内容只是上次下发的目标值），
拖拽期间不反映真实位置。录制器直接调用
`/ros_robot_controller/bus_servo/get_state` 读取真实编码器位置。

力矩操作会**读回校验**：加载状态 `0=已加载（刚性）`、`1=已卸载（可拖动）`。
逐个舵机发送卸载、读回确认，最多重试 3 次；只要有一个没卸载成功就中止，不会写包。

先做只读预检（不改力矩、不写包）：

```bash
docker compose exec -it armpi_fpv bash -lc '
  source /opt/ros/noetic/setup.bash
  source devel/setup.bash
  export PYTHONPATH=src/AI_pkg/src:$PYTHONPATH
  python3 src/AI_pkg/scripts/teach_motion_bag.py --check
'
```

预检打印电池电压、各舵机真实位置、力矩状态和采样周期，并指出舵机无响应时的排查顺序
（电源开关、总线排线、过载保护需断电 10 秒）。**控制板电池有电压不代表舵机供电已开。**

正式录制（以点头为例）：

```bash
docker compose exec -it armpi_fpv bash -lc '
  source /opt/ros/noetic/setup.bash
  source devel/setup.bash
  export PYTHONPATH=src/AI_pkg/src:$PYTHONPATH
  python3 src/AI_pkg/scripts/teach_motion_bag.py --name nod --force
'
```

操作顺序：输入 `yes` 确认已托住 → 看到"已确认全部卸载"后先轻推腕部确认变软 →
按 `a` 开始 → 用手摆出动作 → 按 `s` 结束（`q` 取消且不写包）→ 看到"力矩已恢复并确认"再松手。

| 参数 | 默认 | 说明 |
|---|---|---|
| `--rate` | 15 | 写包采样率（Hz） |
| `--entry-duration` | 1.0 | 播放时从当前位置缓入示教起点的秒数 |
| `--min-delta` | 2 | 位置变化达到多少编码器刻度才发一条指令 |
| `--smooth` | 0.35 | 平滑系数，1 表示不平滑 |
| `--ids` | 1 2 3 4 5 6 | 参与示教的舵机 |
| `--limp-ids` | 同 `--ids` | **只卸载这些舵机**，其余继续托住手臂（强烈建议只卸要动的那个） |
| `--force` | 关 | 覆盖同名包，并保留带时间戳的备份 |
| `--no-torque` | 关 | 不卸载力矩，仅配合 `--allow-static` 验证流程 |
| `--allow-static` | 关 | 没检测到动作也写包，仅用于测试链路 |

#### 录制其它动作（例如转圈）

把 `--name` 换成新名字即可，例如：

```bash
docker compose exec -it armpi_fpv bash -lc '
  source /opt/ros/noetic/setup.bash; source devel/setup.bash
  export PYTHONPATH=src/AI_pkg/src:$PYTHONPATH
  python3 src/AI_pkg/scripts/teach_motion_bag.py --name circle --force
'
```

录完立即生效，不需要重启：

1. `list_motion_actions` 会列出 `circle`；
2. 语音说“转圈”走直达路径（`config/agent.yaml` 的 `llm.motion_commands_json` 已含
   `"转圈": "circle"`）；
3. 其它说法由模型通过 `play_motion("circle")` 调用；
4. 若用别的名字（如 `turn`），在 `motion_commands_json` 里补 `"转圈": "turn"` 并重启 AI 节点。

注意总线舵机的行程约为 0–1000（约 240°），"转圈"只能是来回转动或部分旋转，
无法像轮式底盘那样连续 360° 旋转。

**为什么建议只卸一个关节**：舵机卸载后齿轮组仍有明显阻力，六个关节全卸等于你要扛着整条手臂
的重量去推，非常费力。只卸要动的那个关节（例如转底座用 `--limp-ids 6`、转腕部用
`--limp-ids 2`），其余关节会继续支撑手臂，手感轻得多，也不会突然下坠。

拖动时偶尔会感到轻微阻力，这是读取瞬间的短暂使能，属正常现象；动作尽量平滑连续。
没有检测到动作（时长小于 0.3 秒）时脚本拒绝写包。

### 方式二：从已有动作包裁剪

旧动作包里通常包含示教时的等待和基线调整，可以用 `--from-bag` 只截取有效片段：

```bash
docker compose exec -it armpi_fpv bash -lc '
  source /opt/ros/noetic/setup.bash
  source devel/setup.bash
  export PYTHONPATH=src/AI_pkg/src:$PYTHONPATH
  python3 src/AI_pkg/scripts/teach_motion_bag.py \
    --from-bag src/AI_pkg/config/actions/backups/nod_backup_20261018_073115.bag \
    --start 35.9 --end 53.0 --target-duration 3.5 --amplitude-scale 0.4 \
    --rate 20 --name nod
'
```

- `--start/--end`：截取的时间区间（秒）；省略时自动裁掉首尾静止段
- `--target-duration`：把区间缩放到指定秒数
- `--amplitude-scale`：围绕各自中心缩放幅度（0.4 = 压到原来的 40%）
- 只保留区间内**真正发生位移**的舵机，避免把常量舵机写成入场指令
- 同名旧包会自动备份

### 纯正弦生成（最平滑，可完全参数化）

如果手拖录制的轨迹仍有抖动，可以直接生成解析正弦，速度在极值处自然归零、绝无折点：

```bash
docker compose exec -it armpi_fpv bash -lc '
  source /opt/ros/noetic/setup.bash; source devel/setup.bash
  export PYTHONPATH=src/AI_pkg/src:$PYTHONPATH
  python3 src/AI_pkg/scripts/teach_motion_bag.py --sine \
    --servo 3 --center 227 --amplitude 80 --period 2.4 --cycles 2.5 \
    --rate 30 --entry-duration 1.0 --name nod
'
```

| 参数 | 含义 |
|---|---|
| `--servo` | 摆动的舵机号（点头用 3，腕部俯仰） |
| `--center` / `--amplitude` | 摆动中心与幅度（编码器刻度，0–1000） |
| `--period` / `--cycles` | 每次点头的秒数与总次数 |
| `--rate` | 指令频率，默认每帧必发，间隔恒定 |
| `--invert` | 反向（默认先向下点头） |

代价是完全脱离手拖轨迹的形状，只保留幅度、次数和节奏。

### 方式三：键盘示教

键盘示教不卸载力矩，由 `calibration_arm_teleop.py` 发布目标位置，`record_motion_bag.py`
录制该话题。它只能记录**示教器、遥控器或其他节点发布的目标位置**，直接用手推机械臂不会在
该话题上产生消息；步进较大，动作通常不如拖拽连贯。

先在第一个终端启动键盘示教器：

```bash
docker compose exec -it armpi_fpv bash -lc '
  source /opt/ros/noetic/setup.bash
  source devel/setup.bash
  rosrun desk_cleaning calibration_arm_teleop.py     _allow_motion:=true     _step_pulse:=10     _duration:=0.5
'
```

`a/d` 底座，`w/s` 肩部，`r/f` 肘部，`t/g` 腕部俯仰，`y/h` 腕部旋转，`q` 退出。
`_allow_motion:=true` 会真实驱动舵机，首次务必把步进值设小并确认急停可用。

再开第二个终端录制，按 `a` 开始、`s` 停止、`q` 退出：

```bash
docker compose exec -it armpi_fpv bash -lc '
  source /opt/ros/noetic/setup.bash
  source devel/setup.bash
  python3 src/AI_pkg/scripts/record_motion_bag.py --name nod
'
```

摇头用 `--name shake`。文件保存为 `src/AI_pkg/config/actions/<name>.bag`；若示教设备发布了
其他话题，可重复传 `--topic`。

### 验证动作可播放

MCP 服务从 `action_dir` 参数读动作目录，当前 launch 默认 `$(find AI_pkg)/config/actions`。
下面只查询动作资源，不会驱动机械臂：

```bash
docker compose exec -T armpi_fpv /opt/ai-python/bin/python -c   'import asyncio; from fastmcp import Client; exec("async def main():\n async with Client(\"http://127.0.0.1:8765/mcp\") as c:\n  r=await c.call_tool(\"list_motion_actions\", {}); print(r.content[0].text)"); asyncio.run(main())'
```

模型收到“点头”应调用 `play_nod`，收到“摇头”应调用 `play_shake`。`play_nod` 启动后立即
返回 `started=true`，完成状态用 `list_motion_actions` 查询，播放中可用 `stop_motion` 打断。
真正调用这些工具会驱动实体机械臂，必须先确认急停、周围空间和动作范围。

## ROS1 兼容范围

`package.xml` 的条件依赖、catkin 配置及 `launch/ai.launch` 支持现有 Noetic。launch 通过 `ai_python` 参数指定独立解释器，默认 `/opt/ai-python/bin/python`；直接 rosrun 时，脚本检测旧 Python 后也会自动切换。AI 依赖见 `requirements-noetic.txt`，不要安装到系统 Python 3.8。

## 自动验证

在 Python >=3.10 的 AI 虚拟环境中：

```bash
PYTHONPATH=src/AI_pkg/src python -m unittest discover \
  -s src/AI_pkg/test -p 'test_*.py' -v
```

当前共 **38 项**测试通过（2026-10-18）：18 项覆盖 FastMCP 工具协议、模型调用循环、录音与
Whisper 复用；20 项覆盖拖拽示教的静止段裁剪、重采样、平滑与舵机指令生成（纯函数，不需要 ROS
或机械臂）。

测试使用真实 FastMCP 服务/客户端验证工具协议，用模拟模型验证工具调用和结果回传；录音、Whisper、ROS 话题发布的测试使用替身，不会录制真实声音、下载模型或调用付费接口。这不能代替真实麦克风、Whisper 模型、云端 API 和 ROS2 环境验收。

## 本次验证记录（2026-10-17）

- ARM64 / Python 3.11 独立环境安装了 FastMCP 2.14.7、模型 SDK 2.54.0、faster-whisper 1.2.1。
- 12 项测试通过：真实 FastMCP 内存协议和 Streamable HTTP、移植 add 工具、工具参数检查、模型调用循环、录音格式/清理、Whisper 模型复用和语音任务发布流程。
- setuptools 的 ROS2 分支打包构建通过。现有 Noetic 工作区 `catkin_make --pkg AI_pkg -j2 -l2` 通过；ROS_VERSION=2 的 manifest 能选择 ament_python 和 rclpy 条件依赖。
- 当前宿主机与 Noetic 容器没有 ROS2/rclpy，尚未进行真实 ROS2 colcon 构建及节点启动。USB 声卡的 3 秒 `arecord` 和 `SpeechInput` 1 秒录音已在重建后的 Noetic 容器中通过；Whisper 实际模型转写和 DeepSeek 在线接口仍未验收，自动测试使用替身，不调用云端费用。
- 同一 Noetic 容器内 Python 3.11 导入 rospy、std_msgs、std_srvs、FastMCP、模型 SDK 和 faster-whisper 成功，`pip check` 通过，12 项测试在该容器中再次通过。
- `/mcp_server_node`、`/llm_agent_node` 注册成功，录音 Trigger 服务可见；真实 HTTP MCP 工具发现、`add(100,50)=150`、ROS 话题查询及感知 `ready=true` 查询通过。
- `test/smoke_noetic.py` 启动独立测试节点，以本地 HTTP 模型替身验证 ROS 文本 → 模型 SDK → 真实 MCP add → 工具结果回传 → ROS 回答 `150`。测试节点和模型端口随后清理，不调用云端或麦克风。复现命令：

```bash
docker compose exec armpi_fpv bash -lc '
  source /opt/ros/noetic/setup.bash
  source devel/setup.bash
  /opt/ai-python/bin/python src/AI_pkg/test/smoke_noetic.py
'
```

此测试需要已运行 roscore 和默认 8765 端口的 MCP 服务。
