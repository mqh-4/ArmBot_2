"""Reference voice agent: FastMCP Client + asynchronous model tool calls."""
import json
import os
import re
from urllib.parse import urlsplit

from fastmcp import Client
from openai import AsyncOpenAI


DEFAULT_MOTION_COMMANDS = {
    '点头': 'nod', '點頭': 'nod', '请点头': 'nod', '請點頭': 'nod',
    '点个头': 'nod', '點個頭': 'nod', '请点个头': 'nod',
    '摇头': 'shake', '搖頭': 'shake', '请摇头': 'shake', '請搖頭': 'shake',
    '摇个头': 'shake', '摇個頭': 'shake', '请摇个头': 'shake',
    '转圈': 'circle', '轉圈': 'circle', '请转圈': 'circle', '請轉圈': 'circle',
    '转个圈': 'circle', '转一圈': 'circle', '绕一圈': 'circle', '繞一圈': 'circle',
}

STOP_COMMANDS = {
    '停止动作', '停止動作', '停止播放', '停止点头', '停止摇头', '停止點頭', '停止搖頭',
}


def command_tool(text, motion_commands=None):
    """Route only complete, explicit commands; never match a substring.

    Returns ``{'tool': 'stop_motion'}`` or ``{'tool': 'play_motion', 'action': ...}``.
    """
    commands = dict(DEFAULT_MOTION_COMMANDS)
    if motion_commands:
        commands.update(motion_commands)
    normalized = re.sub(r'[\s，。！？,.!?]', '', text)
    if normalized in STOP_COMMANDS:
        return {'tool': 'stop_motion'}
    action = commands.get(normalized)
    if not action:
        return None
    return {'tool': 'play_motion', 'action': action}


def model_tools(tools):
    """FastMCP returns MCP Tool models with the inputSchema field."""
    return [{'type': 'function', 'function': {
        'name': tool.name, 'description': tool.description or '',
        'parameters': tool.inputSchema or {'type': 'object', 'properties': {}},
    }} for tool in tools]


def tool_result_text(result):
    return json.dumps({
        'isError': result.is_error,
        'content': [block.model_dump(mode='json', exclude_none=True) for block in result.content],
    }, ensure_ascii=False)


class LLMAgent:
    def __init__(self, base_url, model, mcp_url, system_prompt,
                 api_key_env='DEEPSEEK_API_KEY', timeout=30.0,
                 max_tool_rounds=3, max_tokens=512, reasoning_effort='', extra_body=None,
                 mcp_timeout=900.0, motion_commands=None):
        if (not 0 <= max_tool_rounds <= 10 or timeout <= 0 or max_tokens <= 0
                or mcp_timeout <= 0):
            raise ValueError('Invalid tool round limit, timeout or token limit')
        self.base_url = base_url
        self.model = model
        self.mcp_url = mcp_url
        self.system_prompt = system_prompt
        self.api_key_env = api_key_env
        self.timeout = timeout
        self.mcp_timeout = mcp_timeout
        self.max_tool_rounds = max_tool_rounds
        self.max_tokens = max_tokens
        self.reasoning_effort = reasoning_effort
        self.extra_body = extra_body or {}
        self.motion_commands = dict(DEFAULT_MOTION_COMMANDS)
        if motion_commands:
            self.motion_commands.update(motion_commands)

    async def answer(self, text):
        command = command_tool(text, self.motion_commands)
        if command is not None:
            async with Client(self.mcp_url, timeout=self.mcp_timeout) as mcp:
                if command['tool'] == 'stop_motion':
                    result = await mcp.call_tool('stop_motion', {}, raise_on_error=False)
                else:
                    result = await mcp.call_tool(
                        'play_motion', {'action': command['action']}, raise_on_error=False)
                if result.is_error:
                    raise RuntimeError('动作工具调用失败：' + tool_result_text(result))
                payload = json.loads(result.content[0].text)
                return payload['message']
        key = os.environ.get(self.api_key_env, '')
        if not key and urlsplit(self.base_url).hostname not in ('localhost', '127.0.0.1', '::1'):
            raise ValueError('Model API key environment variable is not set')
        # Motion tools return after rosbag finishes. Their HTTP read timeout
        # must cover the action duration independently of the model API timeout.
        async with Client(self.mcp_url, timeout=self.mcp_timeout) as mcp:
            specs = await mcp.list_tools()
            tools = model_tools(specs)
            allowed = {tool.name for tool in specs}
            messages = [{'role': 'system', 'content': self.system_prompt},
                        {'role': 'user', 'content': text}]
            async with AsyncOpenAI(base_url=self.base_url, api_key=key or 'local-no-key',
                                   timeout=self.timeout, max_retries=0) as llm:
                for turn in range(self.max_tool_rounds + 1):
                    options = {'model': self.model, 'messages': messages,
                               'max_tokens': self.max_tokens, 'stream': False}
                    if tools:
                        options.update(tools=tools, tool_choice=(
                            'auto' if turn < self.max_tool_rounds else 'none'))
                    if self.reasoning_effort:
                        options['reasoning_effort'] = self.reasoning_effort
                    if self.extra_body:
                        options['extra_body'] = self.extra_body
                    response = await llm.chat.completions.create(**options)
                    message = response.choices[0].message
                    calls = message.tool_calls or []
                    if not calls:
                        if not message.content or not message.content.strip():
                            raise ValueError('Model returned an empty answer')
                        return message.content.strip()
                    if turn == self.max_tool_rounds or len(calls) > 8:
                        raise RuntimeError('Model exceeded the tool call limit')
                    assistant = {'role': 'assistant', 'content': message.content,
                                 'tool_calls': [call.model_dump(exclude_none=True) for call in calls]}
                    reasoning = getattr(message, 'reasoning_content', None)
                    if reasoning is not None:
                        assistant['reasoning_content'] = reasoning
                    messages.append(assistant)
                    for call in calls:
                        try:
                            if call.function.name not in allowed:
                                raise ValueError('Tool was not advertised by the MCP server')
                            args = json.loads(call.function.arguments)
                            if not isinstance(args, dict):
                                raise ValueError('Tool arguments must be an object')
                            result = await mcp.call_tool(call.function.name, args, raise_on_error=False)
                            content = tool_result_text(result)
                        except (ValueError, RuntimeError) as error:
                            content = json.dumps({'isError': True, 'message': str(error)})
                        messages.append({'role': 'tool', 'tool_call_id': call.id,
                                         'content': content})
        raise RuntimeError('Model did not produce an answer')
