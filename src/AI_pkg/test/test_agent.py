import asyncio
import json
from pathlib import Path
import socket
import tempfile
import threading
import time
import io
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, MagicMock, patch
import wave

from fastmcp import Client
from fastmcp.client.transports import StreamableHttpTransport
import httpx
import uvicorn
from openai.types.chat import ChatCompletionMessage

from ai_pkg.agent import LLMAgent, command_tool
from ai_pkg.nodes import AgentNode
from ai_pkg.speech import SpeechInput, trim_encoder_padding
from ai_pkg.tools import create_server, MotionPlayer


class FakeRos:
    version = 2

    def __init__(self):
        self.messages = {}

    def list_topics(self):
        return [{'name': '/camera', 'types': ['sensor_msgs/msg/Image']}]

    def perception_status(self):
        return {'ready': True, 'ros_version': 2}

    def param(self, _name, default):
        return default

    def publisher(self, topic):
        self.messages[topic] = []
        return self.messages[topic].append

    def subscribe_text(self, _topic, callback):
        self.text_callback = callback

    def record_service(self, _topic, callback):
        self.record_callback = callback

    def trigger_service(self, topic, callback):
        if topic.endswith('/start_recording'):
            self.start_recording_callback = callback
        elif topic.endswith('/stop_recording'):
            self.stop_recording_callback = callback
        else:
            self.record_callback = callback

    def log(self, _text, error=False):
        pass

    def ok(self):
        return True


class FastMCPTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.server = create_server(FakeRos())

    async def test_streamable_http_transport(self):
        app = self.server.http_app(path='/mcp', stateless_http=True, json_response=True)

        def factory(**kwargs):
            return httpx.AsyncClient(transport=httpx.ASGITransport(app=app), **kwargs)

        transport = StreamableHttpTransport('http://localhost/mcp', httpx_client_factory=factory)
        async with app.router.lifespan_context(app):
            async with Client(transport) as client:
                result = await client.call_tool('add', {'a': 100, 'b': 50})
                self.assertEqual('150', result.content[0].text)
                self.assertFalse(result.is_error)

    async def test_tool_discovery_and_ported_add(self):
        async with Client(self.server) as client:
            tools = {tool.name: tool for tool in await client.list_tools()}
            self.assertEqual({
                'add', 'list_ros_topics', 'get_perception_status',
                'list_motion_actions', 'play_motion', 'play_nod', 'play_shake', 'stop_motion',
            }, set(tools))
            self.assertEqual('integer', tools['add'].inputSchema['properties']['a']['type'])
            result = await client.call_tool('add', {'a': 100, 'b': 50})
            self.assertEqual('150', result.content[0].text)
            self.assertFalse(result.is_error)

    async def test_ros_queries_and_argument_validation(self):
        async with Client(self.server) as client:
            status = await client.call_tool('get_perception_status', {})
            self.assertTrue(json.loads(status.content[0].text)['ready'])
            topics = await client.call_tool('list_ros_topics', {})
            self.assertEqual('/camera', json.loads(topics.content[0].text)['topics'][0]['name'])
            invalid = await client.call_tool('add', {'a': 'wrong', 'b': 1}, raise_on_error=False)
            self.assertTrue(invalid.is_error)

    async def run_agent(self, responses, rounds=3):
        mock_llm = MagicMock()
        mock_llm.chat.completions.create = AsyncMock(side_effect=[
            SimpleNamespace(choices=[SimpleNamespace(message=ChatCompletionMessage(**message))])
            for message in responses
        ])
        manager = MagicMock()
        manager.__aenter__ = AsyncMock(return_value=mock_llm)
        manager.__aexit__ = AsyncMock(return_value=False)
        agent = LLMAgent('http://127.0.0.1:8080/v1', 'fixture', 'fixture-mcp', 'Use tools.',
                         max_tool_rounds=rounds)
        with patch('ai_pkg.agent.AsyncOpenAI', return_value=manager), patch(
                'ai_pkg.agent.Client', side_effect=lambda *_a, **_kw: Client(self.server)):
            answer = await agent.answer('100+50')
        return answer, mock_llm

    @staticmethod
    def tool_call(name='add', arguments='{"a":100,"b":50}'):
        return {'role': 'assistant', 'content': None, 'tool_calls': [
            {'id': 'call-1', 'type': 'function', 'function': {'name': name, 'arguments': arguments}}
        ]}

    async def test_real_fastmcp_tool_result_reaches_model(self):
        answer, model = await self.run_agent([self.tool_call(), {'role': 'assistant', 'content': '150'}])
        self.assertEqual('150', answer)
        messages = model.chat.completions.create.call_args.kwargs['messages']
        tool_result = json.loads(messages[-1]['content'])
        self.assertEqual('150', tool_result['content'][0]['text'])
        self.assertFalse(tool_result['isError'])

    async def test_unknown_tool_rejected(self):
        _, model = await self.run_agent([self.tool_call('move_arm'), {'role': 'assistant', 'content': '拒绝'}])
        messages = model.chat.completions.create.call_args.kwargs['messages']
        self.assertTrue(json.loads(messages[-1]['content'])['isError'])

    async def test_tool_loop_bounded(self):
        with self.assertRaisesRegex(RuntimeError, 'limit'):
            await self.run_agent([self.tool_call()], rounds=0)

    async def test_bad_json_arguments_rejected(self):
        _, model = await self.run_agent([
            self.tool_call(arguments='not-json'), {'role': 'assistant', 'content': '参数错误'}])
        messages = model.chat.completions.create.call_args.kwargs['messages']
        self.assertTrue(json.loads(messages[-1]['content'])['isError'])

    async def test_explicit_motion_command_does_not_wait_for_cloud_model(self):
        with tempfile.TemporaryDirectory() as directory:
            Path(directory, 'nod.bag').touch()
            agent = LLMAgent('https://api.deepseek.com/v1', 'fixture',
                             'fixture-mcp', 'Use tools.')
            response = {'ok': True, 'started': True, 'message': '已启动点头动作'}
            with patch('ai_pkg.tools.MotionPlayer.play', return_value=response) as play, patch(
                    'ai_pkg.agent.AsyncOpenAI') as cloud, patch(
                    'ai_pkg.agent.Client', side_effect=lambda *_a, **_kw: Client(self.server)):
                self.assertEqual('已启动点头动作', await agent.answer('點頭。'))
            play.assert_called_once_with('nod', 1, 1.0)
            cloud.assert_not_called()

    def test_fast_command_requires_complete_explicit_match(self):
        for text in ('不要点头', '如果我说点头你会做什么', '点头三次', '点头然后摇头'):
            self.assertIsNone(command_tool(text))
        self.assertEqual({'tool': 'stop_motion'}, command_tool('停止动作！'))
        self.assertEqual({'tool': 'play_motion', 'action': 'circle'}, command_tool('转圈！'))
        self.assertEqual({'tool': 'play_motion', 'action': 'nod'}, command_tool('请点头。'))

    async def test_slow_http_tool_uses_separate_timeout_from_model(self):
        @self.server.tool
        async def slow_add(a: int, b: int) -> int:
            await asyncio.sleep(0.4)
            return a + b

        mock_llm = MagicMock()
        mock_llm.chat.completions.create = AsyncMock(side_effect=[
            SimpleNamespace(choices=[SimpleNamespace(message=ChatCompletionMessage(**message))])
            for message in [self.tool_call('slow_add'), {'role': 'assistant', 'content': '150'}]
        ])
        manager = MagicMock()
        manager.__aenter__ = AsyncMock(return_value=mock_llm)
        manager.__aexit__ = AsyncMock(return_value=False)
        app = self.server.http_app(path='/mcp', stateless_http=True, json_response=True)
        server = uvicorn.Server(uvicorn.Config(app, log_level='critical', lifespan='on'))
        with socket.socket() as listener:
            listener.bind(('127.0.0.1', 0))
            port = listener.getsockname()[1]
            task = asyncio.create_task(server.serve(sockets=[listener]))
            try:
                async def wait_started():
                    while not server.started:
                        if task.done():
                            await task
                            self.fail('HTTP fixture exited before startup')
                        await asyncio.sleep(0.01)
                await asyncio.wait_for(wait_started(), 5)
                agent = LLMAgent(
                    'http://127.0.0.1:8080/v1', 'fixture',
                    'http://127.0.0.1:%d/mcp' % port, 'Use tools.',
                    timeout=0.15, mcp_timeout=3.0,
                )
                with patch('ai_pkg.agent.AsyncOpenAI', return_value=manager) as constructor:
                    self.assertEqual('150', await agent.answer('100+50'))
                self.assertEqual(0.15, constructor.call_args.kwargs['timeout'])
                messages = mock_llm.chat.completions.create.call_args.kwargs['messages']
                result = json.loads(messages[-1]['content'])
                self.assertFalse(result['isError'])
                self.assertEqual('150', result['content'][0]['text'])
            finally:
                server.should_exit = True
                await asyncio.wait_for(task, 5)


class SpeechTests(unittest.TestCase):
    def test_compact_encoder_keeps_short_audio_and_context(self):
        import numpy as np
        features = np.zeros((80, 3000), dtype=np.float32)
        features[:, :250] = 1
        compact = trim_encoder_padding(features)
        self.assertEqual((80, 600), compact.shape)
        np.testing.assert_array_equal(features[:, :600], compact)
        features[:, :1100] = 1
        self.assertIs(features, trim_encoder_padding(features))

    @staticmethod
    def recorder_process(command, **_kwargs):
        with wave.open(command[-1], 'wb') as audio:
            audio.setnchannels(1)
            audio.setsampwidth(2)
            audio.setframerate(16000)
            audio.writeframes(bytes(3200))
        return SimpleNamespace(communicate=lambda **_kw: (None, b''), returncode=0)

    def test_recording_format_and_temporary_file_cleanup(self):
        speech = SpeechInput(seconds=1)
        with patch('ai_pkg.speech.subprocess.Popen', side_effect=self.recorder_process) as command:
            with speech.recording() as path:
                self.assertTrue(Path(path).is_file())
                self.assertIn('16000', command.call_args.args[0])
            self.assertFalse(Path(path).exists())

    def test_recording_failure_is_reported(self):
        process = SimpleNamespace(communicate=lambda **_kw: (None, b'no audio device'), returncode=1)
        with patch('ai_pkg.speech.subprocess.Popen', return_value=process):
            with self.assertRaisesRegex(RuntimeError, 'arecord failed'):
                with SpeechInput().recording():
                    self.fail('Invalid recording accepted')

    def test_whisper_model_is_reused_and_segments_joined(self):
        model = MagicMock()
        model.transcribe.side_effect = [
            ([SimpleNamespace(text=' 你好 '), SimpleNamespace(text='世界 ')], None),
            ([SimpleNamespace(text='第二次')], None),
        ]
        with patch('faster_whisper.WhisperModel', return_value=model) as constructor:
            speech = SpeechInput(local_files_only=True)
            self.assertEqual('你好世界', speech.transcribe('/fixture.wav'))
            self.assertEqual('第二次', speech.transcribe('/fixture.wav'))
            self.assertEqual(1, constructor.call_count)
            self.assertTrue(constructor.call_args.kwargs['local_files_only'])
            self.assertEqual(1, model.transcribe.call_args.kwargs['beam_size'])
            self.assertEqual(0.0, model.transcribe.call_args.kwargs['temperature'])
            self.assertIsNone(model.transcribe.call_args.kwargs['chunk_length'])

    def test_invalid_duration_and_cancellation(self):
        with self.assertRaises(ValueError):
            SpeechInput(seconds=0)
        speech = SpeechInput()
        speech.stop()
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            with speech.recording():
                self.fail('Stopped recorder started')


class MotionPlayerTests(unittest.TestCase):
    class Process:
        def __init__(self):
            self.finished = threading.Event()
            self.stderr = io.StringIO('')
            self.returncode = None

        def poll(self):
            return 0 if self.finished.is_set() else None

        def wait(self, timeout=None):
            if not self.finished.wait(timeout):
                raise TimeoutError('Fixture did not finish')
            return 0

    def test_play_acknowledges_before_completion_and_rejects_overlap(self):
        with tempfile.TemporaryDirectory() as directory:
            Path(directory, 'nod.bag').touch()
            player = MotionPlayer(directory)
            process = self.Process()
            with patch('ai_pkg.tools.subprocess.Popen', return_value=process):
                started = time.monotonic()
                result = player.play('nod')
                try:
                    self.assertTrue(result['started'])
                    self.assertLess(time.monotonic() - started, 0.25)
                    self.assertTrue(player.status()['active'])
                    self.assertFalse(player.play('nod')['ok'])
                finally:
                    process.finished.set()
                    player.worker.join(timeout=2)
            self.assertFalse(player.status()['active'])
            self.assertEqual(1, player.status()['last_result']['completed'])

    def test_stop_remains_available_while_action_runs(self):
        with tempfile.TemporaryDirectory() as directory:
            Path(directory, 'nod.bag').touch()
            player = MotionPlayer(directory)
            process = self.Process()
            with patch('ai_pkg.tools.subprocess.Popen', return_value=process), patch.object(
                    player, '_interrupt', side_effect=lambda _p: process.finished.set()):
                player.play('nod')
                self.assertTrue(player.stop()['stopped'])
                player.worker.join(timeout=2)
            self.assertFalse(player.status()['active'])
            self.assertTrue(player.status()['last_result']['stopped'])


class VoicePipelineTests(unittest.TestCase):
    def test_record_transcribe_agent_publish(self):
        ros = FakeRos()
        with patch('ai_pkg.speech.SpeechInput') as speech_type, patch('ai_pkg.agent.LLMAgent') as agent_type:
            speech = speech_type.return_value
            speech.recording.return_value.__enter__.return_value = '/fixture.wav'
            speech.transcribe.return_value = '一百加五十'
            agent_type.return_value.answer = AsyncMock(return_value='150')
            node = AgentNode(ros)
            try:
                success, _ = ros.record_callback()
                self.assertTrue(success)
                node.pending.join()
                self.assertEqual(['一百加五十'], ros.messages['~/transcript'])
                self.assertEqual(['150'], ros.messages['~/output'])
                self.assertEqual([], ros.messages['~/error'])
            finally:
                node.close()


if __name__ == '__main__':
    unittest.main()
