# 语音指令助手（MCP 工具调用版）
# 流程：录音 -> Whisper 转文本 -> DeepSeek 工具调用 -> 通过 MCP Server 执行 add -> 最终回答 -> TTS 播报
# DeepSeek 不直接算答案，而是通过 MCP 协议回调 mcp_server.py 的 add 工具
# 依赖安装：pip3 install --user soundfile faster-whisper edge-tts pygame openai fastmcp
import asyncio
import json
import os

# 国内网络使用 hf-mirror 镜像下载 Whisper 模型
os.environ.setdefault("HF_ENDPOINT", "https://hf-mirror.com")

from fastmcp import Client
from fastmcp.client.transports import StdioTransport
from faster_whisper import WhisperModel

# 复用 voice_assistant.py 的录音和 DeepSeek 客户端
# （speak 不能直接复用：其内部 asyncio.run() 无法在已有事件循环中运行）
from voice_assistant import record_audio, llm, WHISPER_MODEL, TTS_VOICE

import edge_tts
import pygame
import tempfile

SERVER_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mcp_server.py")


def _play_mp3(mp3_path: str):
    """阻塞式播放 mp3（在线程中运行，不阻塞事件循环）"""
    pygame.mixer.init()
    try:
        pygame.mixer.music.load(mp3_path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)
    finally:
        pygame.mixer.quit()


async def speak_async(text: str):
    """async 版语音播报：edge-tts 合成后播放"""
    mp3_path = os.path.join(tempfile.gettempdir(), "voice_answer.mp3")
    tts = edge_tts.Communicate(text, TTS_VOICE)
    await tts.save(mp3_path)
    await asyncio.to_thread(_play_mp3, mp3_path)

# Whisper 模型只加载一次，避免每轮对话重复加载
_whisper_model = None


def get_whisper_model():
    global _whisper_model
    if _whisper_model is None:
        print("正在加载 Whisper 模型（首次运行需下载）...")
        _whisper_model = WhisperModel(WHISPER_MODEL, device="cpu", compute_type="int8")
    return _whisper_model


def transcribe(wav_path: str) -> str:
    """用 Whisper 把语音转成文本"""
    model = get_whisper_model()
    segments, _ = model.transcribe(wav_path, language="zh", vad_filter=True)
    text = "".join(seg.text.strip() for seg in segments).strip()
    print(f"识别结果：{text}")
    return text


def mcp_tools_to_openai(tools) -> list:
    """把 MCP 工具列表转换成 OpenAI function calling 格式"""
    return [
        {
            "type": "function",
            "function": {
                "name": tool.name,
                "description": tool.description or "",
                "parameters": tool.input_schema or {"type": "object", "properties": {}},
            },
        }
        for tool in tools
    ]


async def chat_with_mcp(client: Client, user_text: str) -> str:
    """把用户文本交给 DeepSeek，工具调用通过 MCP Server 执行"""
    tools = await client.list_tools()
    openai_tools = mcp_tools_to_openai(tools)

    messages = [
        {"role": "system", "content": "你是语音助手，计算类问题必须调用工具完成，不要自己心算。"},
        {"role": "user", "content": user_text},
    ]

    # 工具调用循环：模型可能连续发起多次工具调用
    for _ in range(10):
        response = llm.chat.completions.create(
            model="deepseek-flash",
            messages=messages,
            tools=openai_tools,
            stream=False,
            reasoning_effort="high",
            extra_body={"thinking": {"type": "enabled"}},
        )
        msg = response.choices[0].message

        # 模型决定调用工具：通过 MCP Server 执行并把结果回传
        if msg.tool_calls:
            messages.append(msg)
            for tool_call in msg.tool_calls:
                name = tool_call.function.name
                args = json.loads(tool_call.function.arguments)
                print(f"[工具调用] {name}({args})")
                result = await client.call_tool(name, args)
                result_text = result.content[0].text
                print(f"[工具返回] {result_text}")
                messages.append({
                    "role": "tool",
                    "tool_call_id": tool_call.id,
                    "content": result_text,
                })
            continue

        # 没有工具调用，说明模型已给出最终回答
        return msg.content.strip()

    return "工具调用次数超限，请重试"


async def process_text(client: Client, text: str) -> str:
    """文本 -> DeepSeek + MCP 工具调用 -> 最终回答"""
    answer = await chat_with_mcp(client, text)
    print(f"模型回答：{answer}")
    return answer


async def main():
    async with Client(StdioTransport("python3", [SERVER_PATH])) as client:
        print("已连接 MCP Server，语音指令助手启动")
        while True:
            wav_path = record_audio()
            text = transcribe(wav_path)
            if not text:
                print("没有识别到语音内容，请重试")
                continue
            answer = await process_text(client, text)
            # 去掉 markdown 符号再播报，避免读出星号
            await speak_async(answer.replace("**", "").replace("`", ""))
            again = input("按回车继续对话，输入 q 退出：").strip().lower()
            if again == "q":
                break


if __name__ == "__main__":
    asyncio.run(main())
