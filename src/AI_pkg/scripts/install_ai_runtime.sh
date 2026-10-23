#!/usr/bin/env bash
# Keep Noetic's system Python unchanged; install only the AI runtime here.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ai_prefix="${AI_PYTHON_PREFIX:-/opt/ai-python}"
ai_index="${AI_PIP_INDEX_URL:-https://pypi.tuna.tsinghua.edu.cn/simple}"
requirements="${1:-${script_dir}/../requirements-noetic.txt}"

if [[ ! -x "${ai_prefix}/bin/python" ]]; then
    case "$(uname -m)" in
        aarch64|arm64)
            installer_name=Miniconda3-py311_24.11.1-0-Linux-aarch64.sh
            installer_sha=a8846ade7a5ddd9b6a6546590054d70d1c2cbe4fbe8c79fb70227e8fd93ef9f8
            ;;
        *) echo 'This installer is pinned for the ARM64 ArmBot container.' >&2; exit 1 ;;
    esac
    installer="$(mktemp /tmp/armbot-ai-python.XXXXXXXX.sh)"
    trap 'rm -f -- "$installer"' EXIT
    # The checksum was verified against the official Anaconda download index.
    env -u PYTHONPATH python3 - "$installer" "$installer_name" <<'PY'
import sys
import urllib.request
url = 'https://mirrors.tuna.tsinghua.edu.cn/anaconda/miniconda/' + sys.argv[2]
print('Downloading independent AI Python from ' + url, flush=True)
with urllib.request.urlopen(url, timeout=120) as response, open(sys.argv[1], 'wb') as target:
    while True:
        block = response.read(1024 * 1024)
        if not block:
            break
        target.write(block)
PY
    printf '%s  %s\n' "$installer_sha" "$installer" | sha256sum --check --status
    env -u PYTHONPATH bash "$installer" -b -p "$ai_prefix"
fi

env -u PYTHONPATH "${ai_prefix}/bin/python" -c \
    'import sys; assert sys.version_info >= (3, 10), "AI Python must be >=3.10"; print(sys.version)'
env -u PYTHONPATH PIP_CONFIG_FILE=/dev/null "${ai_prefix}/bin/python" -m pip --isolated install \
    --no-cache-dir --index-url "$ai_index" --timeout 60 --retries 2 -r "$requirements"
echo "AI runtime ready: ${ai_prefix}/bin/python (system Python unchanged)"
