#!/usr/bin/env python3
"""校验随仓库提供的 YOLO；按需下载并校验可选 Whisper，不启动 ROS。"""
import argparse
import hashlib
import json
from pathlib import Path
import tempfile
import urllib.parse
import urllib.request

ROOT = Path(__file__).resolve().parents[1]


def verify(path, entry):
    if not path.is_file():
        raise ValueError("缺少文件: " + str(path))
    if path.stat().st_size != entry["size"]:
        raise ValueError("文件大小不匹配，请先移走损坏文件: " + str(path))
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    if digest.hexdigest() != entry["sha256"]:
        raise ValueError("SHA-256 不匹配，请先移走损坏文件: " + str(path))


def download(path, entry, endpoint):
    """只安装缺失文件，临时文件通过大小和摘要校验后才进入正式路径。"""
    if path.exists():
        verify(path, entry)
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    url = endpoint.rstrip("/") + "/" + entry["source"]
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            dir=path.parent, prefix=path.name + ".", suffix=".part", delete=False,
        ) as target:
            temporary = Path(target.name)
            print("下载 " + url, flush=True)
            with urllib.request.urlopen(url, timeout=60) as response:
                received = 0
                while True:
                    chunk = response.read(1024 * 1024)
                    if not chunk:
                        break
                    received += len(chunk)
                    if received > entry["size"]:
                        raise ValueError("下载数据超过预期大小")
                    target.write(chunk)
        verify(temporary, entry)
        if path.exists():
            raise ValueError("目标已由其他进程创建，请重新校验: " + str(path))
        temporary.replace(path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--with-ai", action="store_true", help="同时准备可选 Whisper 模型")
    parser.add_argument("--check", action="store_true", help="只校验本地文件，不访问网络")
    parser.add_argument("--endpoint", default="https://huggingface.co",
                        help="Hugging Face HTTPS 镜像根地址")
    args = parser.parse_args()
    url = urllib.parse.urlsplit(args.endpoint)
    if (url.scheme != "https" or not url.hostname or url.username or url.password
            or url.query or url.fragment):
        parser.error("--endpoint 必须是无凭据、查询参数和片段的 HTTPS 地址")
    manifest = json.loads((ROOT / "models/manifest.json").read_text())
    for entry in manifest["models"]:
        if entry["profile"] == "ai" and not args.with_ai:
            continue
        path = (ROOT / entry["path"]).resolve()
        try:
            path.relative_to(ROOT)
        except ValueError:
            raise ValueError("清单路径超出项目: " + entry["path"])
        if not path.exists() and not args.check and entry.get("source"):
            download(path, entry, args.endpoint)
        verify(path, entry)
        print("PASS: " + entry["path"])
    print("模型校验完成")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError) as error:
        raise SystemExit("ERROR: " + str(error))
