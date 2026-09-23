#!/usr/bin/env python3
import json
import re
import urllib.request

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String

from desk_cleaning.core import CleanupCommand, parse_command
from desk_cleaning.msg import CleanDeskAction, CleanDeskGoal


class VoiceTaskPlanner:
    def __init__(self):
        objects = rospy.get_param('~objects', {})
        self.aliases = {
            name: config.get('aliases', [name])
            for name, config in objects.items()
        }
        self.enabled_classes = {
            name for name, config in objects.items() if config.get('enabled', False)
        }
        self.dest_aliases = rospy.get_param('~destination_aliases', {})
        self.destinations = rospy.get_param('~destinations', {})
        self.llm_enabled = bool(rospy.get_param('~llm/enabled', False))
        self.llm_url = rospy.get_param(
            '~llm/url', 'http://127.0.0.1:8080/v1/chat/completions',
        )
        self.llm_model = rospy.get_param('~llm/model', 'local-model')
        self.llm_timeout = float(rospy.get_param('~llm/timeout', 20.0))
        self.llm_backend = rospy.get_param('~llm/backend', 'http').strip().lower()
        self.llm_model_path = rospy.get_param('~llm/model_path', '')
        self.llm_context = int(rospy.get_param('~llm/context', 2048))
        self.llm_threads = int(rospy.get_param('~llm/threads', 4))
        self.local_llm = None
        self.speech_pub = rospy.Publisher('/desk_cleaning/speech', String, queue_size=10)
        self.command_pub = rospy.Publisher('/desk_cleaning/parsed_command', String, queue_size=10)
        self.client = actionlib.SimpleActionClient('/desk_cleaning/clean_desk', CleanDeskAction)
        self.server_available = self.client.wait_for_server(rospy.Duration(5.0))
        if not self.server_available:
            rospy.logwarn('desk cleaning action server is not available yet')
        self.last_goal = None
        rospy.Subscriber('~text_input', String, self.text_callback, queue_size=10)
        rospy.loginfo(
            'voice task planner ready (llm_enabled=%s, backend=%s)',
            self.llm_enabled, self.llm_backend,
        )

    def speak(self, text):
        self.speech_pub.publish(String(data=text))

    def llm_system_prompt(self):
        class_names = sorted(self.aliases)
        destination_names = sorted(self.destinations)
        return (
            '你是桌面清理机器人的指令解析器。只输出一个JSON对象，不要解释。'
            '字段只能是intent, allowed_classes, excluded_classes, destination, '
            'announce_each, max_objects。intent只能是cleanup, stop, pause, resume, status。'
            '类别只能来自%s。位置只能来自%s。缺省位置是default_tray。'
            '不要生成舵机、坐标、ROS话题或其他动作。'
        ) % (class_names, destination_names)
    def parse_llm_content(self, content):
        match = re.search(r'\{.*\}', content, flags=re.DOTALL)
        if not match:
            raise ValueError('LLM response did not contain JSON')
        command = CleanupCommand.from_json(match.group(0))
        return command.validate(self.aliases, self.destinations)

    def query_local_llama(self, text):
        if not self.llm_model_path:
            raise RuntimeError('llm_model_path is required for the llama_cpp backend')
        if self.local_llm is None:
            try:
                from llama_cpp import Llama
            except ImportError as exc:
                raise RuntimeError(
                    'llama-cpp-python is not installed; install it on the target device'
                ) from exc
            self.local_llm = Llama(
                model_path=self.llm_model_path,
                n_ctx=self.llm_context,
                n_threads=self.llm_threads,
                verbose=False,
            )
        response = self.local_llm.create_chat_completion(
            messages=[
                {'role': 'system', 'content': self.llm_system_prompt()},
                {'role': 'user', 'content': text},
            ],
            temperature=0.0,
            max_tokens=220,
        )
        return self.parse_llm_content(response['choices'][0]['message']['content'])

    def query_llm(self, text):
        if self.llm_backend in ('llama_cpp', 'embedded', 'local'):
            return self.query_local_llama(text)
        body = json.dumps({
            'model': self.llm_model,
            'temperature': 0,
            'max_tokens': 220,
            'messages': [
                {'role': 'system', 'content': self.llm_system_prompt()},
                {'role': 'user', 'content': text},
            ],
        }).encode('utf-8')
        request = urllib.request.Request(
            self.llm_url,
            data=body,
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        with urllib.request.urlopen(request, timeout=self.llm_timeout) as response:
            payload = json.loads(response.read().decode('utf-8'))
        content = payload['choices'][0]['message']['content']
        return self.parse_llm_content(content)

    def parse(self, text):
        command = parse_command(text, self.aliases, self.dest_aliases)
        if command is not None:
            return command.validate(self.aliases, self.destinations)
        if not self.llm_enabled:
            return None
        return self.query_llm(text)

    def text_callback(self, msg):
        try:
            command = self.parse(msg.data)
            if command is None:
                self.speak('没有听懂，请说开始作业')
                return
            rospy.loginfo(
                'voice command accepted: intent=%s allowed=%s destination=%s',
                command.intent, command.allowed_classes, command.destination,
            )
            self.command_pub.publish(String(data=command.to_json()))
            self.dispatch(command)
        except Exception as exc:
            rospy.logwarn('voice command rejected: %s', exc)
            self.speak('指令无法执行，请重新描述')

    def dispatch(self, command):
        if not self.server_available:
            self.server_available = self.client.wait_for_server(rospy.Duration(1.0))
        if not self.server_available:
            self.speak('清理执行器尚未就绪')
            return
        if command.intent in ('stop', 'pause'):
            self.client.cancel_all_goals()
            self.speak('已停止清理' if command.intent == 'stop' else '已暂停清理')
            return
        if command.intent == 'resume':
            if self.last_goal is None:
                self.speak('当前没有可以继续的清理任务')
            else:
                self.client.send_goal(self.last_goal)
                self.speak('继续清理')
            return
        if command.intent == 'status':
            state = self.client.get_state()
            if state in (GoalStatus.PENDING, GoalStatus.ACTIVE):
                self.speak('桌面清理正在进行')
            else:
                self.speak('当前没有正在执行的清理任务')
            return

        forbidden = set(command.allowed_classes) - self.enabled_classes
        if forbidden:
            self.speak('这些物品当前禁止自动抓取：%s' % '、'.join(sorted(forbidden)))
            return

        goal = CleanDeskGoal()
        goal.allowed_classes = command.allowed_classes
        goal.excluded_classes = command.excluded_classes
        goal.destination = command.destination
        goal.max_objects = command.max_objects
        goal.announce_each = command.announce_each
        self.last_goal = goal
        self.client.send_goal(goal)


if __name__ == '__main__':
    rospy.init_node('voice_task_planner')
    VoiceTaskPlanner()
    rospy.spin()
