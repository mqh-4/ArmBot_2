"""FastMCP tools ported from the supplied mcp_server.py plus ROS queries."""
import os
import re
import signal
import subprocess
import threading
from pathlib import Path

# Action names become file names under action_dir; keep them filesystem-safe.
ACTION_PATTERN = re.compile(r'^[a-z][a-z0-9_-]{0,31}$')
ACTION_LABELS = {'nod': '点头', 'shake': '摇头', 'circle': '转圈'}

from fastmcp import FastMCP


class MotionPlayer:
    """Play only named, operator-created action bags.

    The player owns one rosbag process at a time so two LLM requests cannot
    command conflicting trajectories concurrently. The stop event is shared
    with the MCP stop tool, which can run while a play tool is waiting.
    """

    def __init__(self, action_dir):
        self.action_dir = Path(action_dir).expanduser().resolve()
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.process = None
        self.action = None
        self.completed = 0
        self.last_result = None
        self.worker = None

    def available(self):
        """Every <name>.bag in action_dir whose name is filesystem-safe."""
        if not self.action_dir.is_dir():
            return []
        return sorted(path.stem for path in self.action_dir.glob('*.bag')
                      if ACTION_PATTERN.match(path.stem))

    def _bag_for(self, action):
        if not isinstance(action, str) or not ACTION_PATTERN.match(action):
            raise ValueError('动作名称无效：只能是小写字母、数字、_ 和 -')
        path = self.action_dir / (action + '.bag')
        if not path.is_file():
            raise FileNotFoundError('动作包不存在：%s' % path)
        return path

    def play(self, action, repeat=1, speed=1.0):
        try:
            path = self._bag_for(action)
        except (ValueError, FileNotFoundError) as error:
            return {'ok': False, 'message': str(error)}
        try:
            repeat = int(repeat)
            speed = float(speed)
        except (TypeError, ValueError):
            raise ValueError('repeat 和 speed 必须是数字')
        if not 1 <= repeat <= 3:
            raise ValueError('repeat 必须在 1 到 3 之间')
        if not 0.25 <= speed <= 2.0:
            raise ValueError('speed 必须在 0.25 到 2.0 之间')

        with self.lock:
            if self.action is not None:
                return {'ok': False, 'message': '已有动作正在播放：%s' % self.action}
            self.stop_event.clear()
            self.action = action
            self.completed = 0
            self.last_result = None
            try:
                self.process = self._start_process(path, speed)
            except (OSError, ValueError) as error:
                self.action = None
                self.process = None
                return {'ok': False, 'message': '无法启动动作播放：%s' % error}
            self.worker = threading.Thread(
                target=self._monitor, args=(action, path, repeat, speed, self.process),
                daemon=True,
            )
            self.worker.start()
        # Acknowledgement is independent of the duration of the taught action.
        # Completion and any playback error are available through status().
        return {'ok': True, 'started': True, 'completed': 0,
                'message': '已启动%s动作，计划播放 %d 次。' % (
                    ACTION_LABELS.get(action, action), repeat)}

    def _start_process(self, path, speed):
        return subprocess.Popen(
            ['rosbag', 'play', '--quiet', '--delay', '0.1',
             '--wait-for-subscribers', '--rate', str(speed), str(path)],
            start_new_session=True, stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE, text=True,
        )

    def _monitor(self, action, path, repeat, speed, process):
        completed = 0
        result = None
        try:
            for index in range(repeat):
                while process.poll() is None:
                    if self.stop_event.wait(0.02):
                        self._interrupt(process)
                        break
                return_code = process.wait()
                details = (process.stderr.read() if process.stderr else '').strip()
                if process.stderr is not None:
                    process.stderr.close()
                if self.stop_event.is_set():
                    result = {'ok': True, 'stopped': True, 'completed': completed,
                              'message': '动作已停止，已完成 %d/%d 次' % (completed, repeat)}
                    break
                if return_code != 0:
                    result = {'ok': False, 'completed': completed,
                              'message': 'rosbag 播放失败：%s' % (details[-300:] or return_code)}
                    break
                completed += 1
                with self.lock:
                    self.completed = completed
                    if index + 1 < repeat:
                        if self.stop_event.is_set():
                            result = {'ok': True, 'stopped': True, 'completed': completed,
                                      'message': '动作已停止'}
                            break
                        process = self._start_process(path, speed)
                        self.process = process
            if result is None:
                result = {'ok': True, 'stopped': False, 'completed': completed,
                          'message': '动作 %s 播放完成 %d 次' % (action, completed)}
        except Exception as error:
            result = {'ok': False, 'completed': completed,
                      'message': '动作播放失败：%s' % str(error)[-300:]}
        finally:
            with self.lock:
                self.last_result = result
                self.process = None
                self.action = None

    def _interrupt(self, process):
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            if process.poll() is None:
                process.terminate()

    def stop(self):
        with self.lock:
            action = self.action
            process = self.process
            if action is None:
                return {'ok': True, 'stopped': False, 'message': '当前没有正在播放的动作'}
            self.stop_event.set()
        if process is not None:
            self._interrupt(process)
        return {'ok': True, 'stopped': True, 'message': '已请求停止动作：%s' % action}

    def status(self):
        with self.lock:
            return {'active': self.action is not None, 'action': self.action,
                    'action_dir': str(self.action_dir), 'completed': self.completed,
                    'last_result': self.last_result}


def create_server(ros):
    default_action_dir = Path(__file__).resolve().parents[2] / 'config' / 'actions'
    action_dir = ros.param('action_dir', str(default_action_dir))
    player = MotionPlayer(action_dir)
    mcp = FastMCP(
        name='ArmBot AI tools',
        instructions='加法、ROS 状态查询和已录制机械臂动作播放工具。'
                     '动作名必须来自 list_motion_actions 的 available 列表。',
    )

    @mcp.tool
    def add(a: int, b: int) -> int:
        """计算两个整数的和。移植自提供的 mcp_server.py。"""
        return a + b

    @mcp.tool
    def list_ros_topics() -> dict:
        """列出当前 ROS 图中的话题名称及消息类型。"""
        return {'ros_version': ros.version, 'topics': ros.list_topics()}

    @mcp.tool
    def get_perception_status() -> dict:
        """查询最近一次感知 ready 状态；此状态不代表实体抓取已经验收。"""
        return ros.perception_status()

    @mcp.tool
    def list_motion_actions() -> dict:
        """列出动作目录里所有可播放的动作名及当前播放状态。"""
        return {'available': player.available(), 'player': player.status()}

    @mcp.tool
    def play_motion(action: str, repeat: int = 1, speed: float = 1.0) -> dict:
        """播放指定动作：action 必须是 list_motion_actions 返回的名称，例如 nod、circle。"""
        return player.play(action, repeat, speed)

    @mcp.tool
    def play_nod(repeat: int = 1, speed: float = 1.0) -> dict:
        """播放"点头"（兼容旧接口，等价于 play_motion("nod")）。"""
        return player.play('nod', repeat, speed)

    @mcp.tool
    def play_shake(repeat: int = 1, speed: float = 1.0) -> dict:
        """播放"摇头"（兼容旧接口，等价于 play_motion("shake")）。"""
        return player.play('shake', repeat, speed)

    @mcp.tool
    def stop_motion() -> dict:
        """停止当前动作播放。"""
        return player.stop()

    return mcp
