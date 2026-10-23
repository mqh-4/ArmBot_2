"""Node implementations shared by ROS1 scripts and ROS2 console entry points."""
import asyncio
import json
import queue
import signal
import sys
import threading
import time


def check_python():
    if sys.version_info < (3, 10):
        raise SystemExit('FastMCP requires Python >=3.10. In Noetic, use the independent '
                         '/opt/ai-python/bin/python via roslaunch AI_pkg ai.launch.')


def mcp_main():
    check_python()
    import uvicorn
    from ai_pkg.ros_runtime import RosRuntime
    from ai_pkg.tools import create_server
    ros = RosRuntime('mcp_server_node')
    thread = None
    try:
        host = ros.param('host', '127.0.0.1')
        if host not in ('127.0.0.1', 'localhost'):
            raise ValueError('The unauthenticated MCP server must bind to loopback')
        port = int(ros.param('port', 8765))
        ros.watch_perception(ros.param('ready_topic', '/desk_perception/ready'))
        app = create_server(ros).http_app(path='/mcp', stateless_http=True, json_response=True)
        server = uvicorn.Server(uvicorn.Config(app, host=host, port=port, log_level='info'))
        # Uvicorn handles process signals in the main thread; ROS callbacks run separately.
        def spin_ros():
            try:
                ros.spin()
            finally:
                # rosnode kill / ROS shutdown must also release the HTTP port.
                server.should_exit = True

        thread = threading.Thread(target=spin_ros, daemon=True)
        thread.start()
        ros.log('FastMCP server: http://%s:%d/mcp (ROS%d)' % (host, port, ros.version))
        server.run()
    finally:
        ros.close()
        if thread is not None:
            thread.join(timeout=2)


class AgentNode:
    def __init__(self, ros):
        from ai_pkg.agent import LLMAgent
        from ai_pkg.speech import SpeechInput
        self.ros = ros
        self.stopped = threading.Event()
        self.pending = queue.Queue(maxsize=1)
        self.processing_lock = threading.Lock()
        self.record_lock = threading.Lock()
        self.record_stop = None
        self.record_thread = None
        self.record_stopped_at = None
        p = ros.param
        extra = json.loads(p('llm.extra_body_json', '{}'))
        if not isinstance(extra, dict):
            raise ValueError('llm.extra_body_json must decode to an object')
        commands = json.loads(p('llm.motion_commands_json', '{}') or '{}')
        if not isinstance(commands, dict):
            raise ValueError('llm.motion_commands_json must decode to an object')
        self.agent = LLMAgent(
            base_url=p('llm.base_url', 'https://api.deepseek.com/v1'),
            model=p('llm.model', 'deepseek-flash'),
            mcp_url=p('mcp_url', 'http://127.0.0.1:8765/mcp'),
            system_prompt=p('system_prompt', '你是语音助手。计算必须调用 add，ROS 状态必须查询工具。'),
            api_key_env=p('llm.api_key_env', 'DEEPSEEK_API_KEY'),
            timeout=float(p('llm.timeout', 30.0)),
            mcp_timeout=float(p('mcp_timeout', 900.0)),
            max_tool_rounds=int(p('llm.max_tool_rounds', 3)),
            max_tokens=int(p('llm.max_tokens', 512)),
            reasoning_effort=p('llm.reasoning_effort', ''), extra_body=extra,
            motion_commands=commands,
        )
        self.speech = SpeechInput(
            device=p('audio.device', 'default'), seconds=int(p('audio.seconds', 5)),
            model=p('whisper.model', 'small'), language=p('whisper.language', 'zh'),
            download_root=p('whisper.download_root', ''),
            local_files_only=bool(p('whisper.local_files_only', False)),
            hf_endpoint=p('whisper.hf_endpoint', 'https://hf-mirror.com'),
            max_seconds=int(p('audio.max_seconds', 60)),
            beam_size=int(p('whisper.beam_size', 1)),
            cpu_threads=int(p('whisper.cpu_threads', 4)),
            compact_encoder=bool(p('whisper.compact_encoder', True)),
        )
        if bool(p('whisper.preload', True)):
            self.speech.load_model()
        self.publish = ros.publisher('~/output')
        self.error = ros.publisher('~/error')
        self.transcript = ros.publisher('~/transcript')
        self.timing = ros.publisher('~/timing')
        ros.subscribe_text('~/input', lambda text: self.enqueue('text', text))
        ros.record_service('~/record', lambda: self.enqueue('record', ''))
        ros.trigger_service('~/start_recording', self.start_recording)
        ros.trigger_service('~/stop_recording', self.stop_recording)
        self.worker = threading.Thread(target=self.work, daemon=True)
        self.worker.start()
        ros.log('LLM agent ready: text ~/input; recording services ~/start_recording ~/stop_recording; output ~/output')

    def enqueue(self, kind, text):
        text = text.strip()
        if self.stopped.is_set():
            return False, '节点已停止。'
        if kind == 'text' and not 1 <= len(text) <= 8000:
            message = '文本长度无效（应为 1 至 8000 个字符）。'
            self.error(message)
            return False, message
        try:
            self.pending.put_nowait((kind, text))
        except queue.Full:
            self.error('请求队列已满。')
            return False, '请求队列已满。'
        return True, '任务已加入队列；录音转写通过 transcript 话题输出，回答通过 output 话题输出。'

    def start_recording(self):
        with self.record_lock:
            if self.record_thread is not None and self.record_thread.is_alive():
                return False, '已经在录音。'
            if not self.processing_lock.acquire(blocking=False):
                return False, '当前正在处理上一条请求。'
            self.record_stop = threading.Event()
            self.record_stopped_at = None
            self.record_thread = threading.Thread(
                target=self.interactive_record, args=(self.record_stop,), daemon=True,
            )
            self.record_thread.start()
        self.ros.log('开始录音；按 stop_recording 结束')
        return True, '已开始录音。'

    def stop_recording(self):
        with self.record_lock:
            if self.record_stop is None or self.record_thread is None or not self.record_thread.is_alive():
                return False, '当前没有正在进行的录音。'
            self.record_stopped_at = time.monotonic()
            self.record_stop.set()
        return True, '已请求停止录音，正在转写并处理。'

    def interactive_record(self, stop_event):
        stage = '录音'
        try:
            self.ros.log('交互录音进行中，等待停止信号')
            with self.speech.recording_until_stopped(stop_event) as path:
                stage = 'Whisper 转写'
                transcribe_started = time.monotonic()
                stopped_at = self.record_stopped_at or transcribe_started
                text = self.speech.transcribe(path)
                transcribe_seconds = time.monotonic() - transcribe_started
            if not text:
                raise ValueError('No speech was recognized')
            self.transcript(text)
            self.timing(json.dumps({'stage': 'transcript',
                                    'transcribe_seconds': round(transcribe_seconds, 3),
                                    'seconds_after_stop': round(time.monotonic() - stopped_at, 3)}))
            stage = '模型/MCP'
            answer = asyncio.run(self.agent.answer(text))
            if not self.stopped.is_set() and self.ros.ok():
                self.publish(answer)
                self.timing(json.dumps({'stage': 'response',
                                        'seconds_after_stop': round(time.monotonic() - stopped_at, 3)}))
        except Exception as error:
            detail = str(error).strip().replace('\n', ' ')[-300:]
            message = '%s任务失败：%s%s' % (
                stage, type(error).__name__, (': ' + detail) if detail else '',
            )
            if not self.stopped.is_set():
                self.error(message)
                self.ros.log(message, error=True)
        finally:
            with self.record_lock:
                self.record_stop = None
                self.record_thread = None
            self.processing_lock.release()

    def work(self):
        while not self.stopped.is_set() and self.ros.ok():
            try:
                kind, text = self.pending.get(timeout=0.2)
            except queue.Empty:
                continue
            stage = '模型/MCP'
            with self.processing_lock:
                try:
                    if kind == 'record':
                        stage = '录音'
                        self.ros.log('开始录音（%d 秒）' % self.speech.seconds)
                        with self.speech.recording() as path:
                            stage = 'Whisper 转写'
                            text = self.speech.transcribe(path)
                        if not text:
                            raise ValueError('No speech was recognized')
                        self.transcript(text)
                    if not self.stopped.is_set():
                        stage = '模型/MCP'
                        answer = asyncio.run(self.agent.answer(text))
                        if not self.stopped.is_set() and self.ros.ok():
                            self.publish(answer)
                except Exception as error:
                    # Never log model payloads or API credentials.
                    detail = str(error).strip().replace('\n', ' ')[-300:]
                    message = '%s任务失败：%s%s' % (
                        stage, type(error).__name__, (': ' + detail) if detail else '',
                    )
                    if not self.stopped.is_set():
                        self.error(message)
                        self.ros.log(message, error=True)
                finally:
                    self.pending.task_done()

    def close(self):
        self.stopped.set()
        with self.record_lock:
            if self.record_stop is not None:
                self.record_stop.set()
        self.speech.stop()
        if self.record_thread is not None:
            self.record_thread.join(timeout=2)
        self.worker.join(timeout=2)


def agent_main():
    check_python()
    from ai_pkg.ros_runtime import RosRuntime
    ros = RosRuntime('llm_agent_node')
    node = None
    try:
        node = AgentNode(ros)
        if ros.version == 1:
            # rospy signals are disabled so the main thread owns cleanup.
            def stop(_signum, _frame):
                raise KeyboardInterrupt
            signal.signal(signal.SIGINT, stop)
            signal.signal(signal.SIGTERM, stop)
        ros.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
        ros.close()
