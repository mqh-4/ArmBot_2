# 模型资源

水果夹取主链路使用随 Git 提交的 `yolov8s.onnx`，大小 44,842,052 字节。
该文件为当前部署使用的 YOLOv8s COCO ONNX 权重，类别顺序见
`src/desk_cleaning/config/perception.yaml`。来源记录为 Ultralytics 的 YOLOv8s COCO
预训练模型；仓库没有保存当时的完整导出命令，因此不承诺任意重新导出的权重与此文件逐字节一致。

所有部署文件的精确大小和 SHA-256 位于 [manifest.json](manifest.json)。
已有其他本地 YOLO 模型仍可保留，但不会自动提交到 Git。

## 校验默认水果模型

在项目根目录运行：

```bash
python3 tools/prepare_models.py --check
```

## 恢复可选 AI 语音模型

ArmPi 硬件 ASR 关键词链路不使用 Whisper。需要 AI_pkg 的 faster-whisper 功能时运行：

```bash
python3 tools/prepare_models.py --with-ai
```

下载来源为 [Systran/faster-whisper-small](https://huggingface.co/Systran/faster-whisper-small)。
可使用镜像：

```bash
python3 tools/prepare_models.py --with-ai --endpoint https://hf-mirror.com
```

配置、tokenizer 和词表随 Git 提交；约 461 MiB 的 `model.bin` 在本地下载，不写入 Git。
工具把下载数据写入临时文件，严格检查大小和 SHA-256 后才安装到模型路径。
虽然远程 URL 使用 `main`，摘要固定了部署文件；上游变更将导致校验失败，不能静默换成新权重。
已有文件损坏时工具会停止并报错，不覆盖原文件。
本文对应的权重摘要是：

```text
3e305921506d8872816023e4c273e75d2419fb89b24da97b4fe7bce14170d671
```

只验证所有本地模型、不访问网络：

```bash
python3 tools/prepare_models.py --with-ai --check
```

## 模型来源与许可证

YOLOv8 模型遵循 Ultralytics 对应版本的许可条款，参见
[Ultralytics 许可说明](https://www.ultralytics.com/license)。
Whisper 的 CTranslate2 转换仓库模型卡标注为 MIT，来源见
[Whisper 模型说明](whisper-small/README.md)。
第三方模型的许可证与本项目源码各自适用，仓库不把它们重新授权为统一许可证。
