# 语音助手：录音 -> Whisper 转文本 -> DeepSeek 回答 -> 语音播报
# 依赖安装：pip3 install --user soundfile faster-whisper edge-tts pygame openai
# 录音使用系统自带 arecord 命令，无需额外系统依赖
import os
# 国内网络使用 hf-mirror 镜像下载 Whisper 模型
os.environ.setdefault("HF_ENDPOINT", "https://hf-mirror.com")
import signal
import subprocess
import sys
import tempfile

import soundfile as sf
from faster_whisper import WhisperModel
from openai import OpenAI

# 全局录音状态
SAMPLE_RATE = 16000  # Whisper 需要 16kHz
CHANNELS = 1
TTS_VOICE = "zh-CN-XiaoxiaoNeural"  # 微软中文女声
WHISPER_MODEL = "small"  # 首次运行会自动下载模型

# 大语言模型（DeepSeek）；密钥只从环境变量读取，禁止写入源码或提交到仓库。
llm = OpenAI(
    api_key=os.environ.get("DEEPSEEK_API_KEY"),
    base_url="https://api.deepseek.com",
)


def record_audio() -> str:
    """按回车开始录音，再按回车结束，返回 wav 文件路径"""
    wav_path = os.path.join(tempfile.gettempdir(), "voice_input.wav")
    input("按回车开始录音...")
    # arecord 录制 16kHz 单声道 16bit PCM（Whisper 要求 16kHz）
    proc = subprocess.Popen(
        ["arecord", "-f", "S16_LE", "-r", str(SAMPLE_RATE),
         "-c", str(CHANNELS), wav_path],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    input("录音中... 按回车结束\n")
    # SIGINT 让 arecord 正常收尾并写入 wav 头
    proc.send_signal(signal.SIGINT)
    proc.wait()

    data, sr = sf.read(wav_path, dtype="float32")
    if len(data) == 0:
        print("未录到任何声音")
        sys.exit(1)
    print(f"录音完成，时长 {len(data) / sr:.1f} 秒")
    return wav_path


def transcribe(wav_path: str) -> str:
    """用 Whisper 把语音转成文本"""
    print("正在加载 Whisper 模型（首次运行需下载）...")
    model = WhisperModel(WHISPER_MODEL, device="cpu", compute_type="int8")
    segments, _ = model.transcribe(wav_path, language="zh", vad_filter=True)
    text = "".join(seg.text.strip() for seg in segments).strip()
    print(f"识别结果：{text}")
    return text


def ask_llm(prompt: str) -> str:
    """把文本交给大语言模型回答"""
    response = llm.chat.completions.create(
        model="deepseek-flash",
        messages=[
            {"role": "system", "content": "You are a helpful assistant"},
            {"role": "user", "content": prompt},
        ],
        stream=False,
        reasoning_effort="high",
        extra_body={"thinking": {"type": "enabled"}},
    )
    answer = response.choices[0].message.content.strip()
    print(f"模型回答：{answer}")
    return answer


def speak(text: str):
    """用 edge-tts 合成语音并播放"""
    import edge_tts
    import asyncio
    import pygame

    async def synth():
        mp3_path = os.path.join(tempfile.gettempdir(), "voice_answer.mp3")
        tts = edge_tts.Communicate(text, TTS_VOICE)
        await tts.save(mp3_path)
        return mp3_path

    mp3_path = asyncio.run(synth())
    pygame.mixer.init()
    try:
        pygame.mixer.music.load(mp3_path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)
    finally:
        pygame.mixer.quit()


def main():
    while True:
        wav_path = record_audio()
        text = transcribe(wav_path)
        if not text:
            print("没有识别到语音内容，请重试")
            continue
        answer = ask_llm(text)
        speak(answer)
        again = input("按回车继续对话，输入 q 退出：").strip().lower()
        if again == "q":
            break


if __name__ == "__main__":
    main()
