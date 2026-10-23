"""Live ROS1/MCP smoke test; requires roscore and mcp_server_node.

Run with the isolated AI Python after sourcing Noetic and devel/setup.bash.
The model endpoint is a local fixture; no microphone or cloud API is used.
"""
import asyncio
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from fastmcp import Client
import rospy
from std_msgs.msg import String

from ai_pkg.ros_runtime import _modern_find_caller
from rosgraph.roslogging import RospyLogger


class ModelFixture(BaseHTTPRequestHandler):
    verified_result = False

    def log_message(self, *_args):
        pass

    def do_POST(self):
        request = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        messages = request['messages']
        if messages[-1]['role'] == 'tool':
            result = json.loads(messages[-1]['content'])
            assert result['content'][0]['text'] == '150' and not result['isError']
            type(self).verified_result = True
            message = {'role': 'assistant', 'content': '150'}
            reason = 'stop'
        else:
            assert any(tool['function']['name'] == 'add' for tool in request['tools'])
            message = {'role': 'assistant', 'content': None, 'tool_calls': [{
                'id': 'smoke-add', 'type': 'function',
                'function': {'name': 'add', 'arguments': '{"a":100,"b":50}'},
            }]}
            reason = 'tool_calls'
        body = json.dumps({'id': 'smoke', 'object': 'chat.completion', 'created': 0,
                           'model': 'local-fixture', 'choices': [
                               {'index': 0, 'message': message, 'finish_reason': reason}
                           ]}).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)


async def check_mcp():
    async with Client('http://127.0.0.1:8765/mcp', timeout=10) as client:
        names = [tool.name for tool in await client.list_tools()]
        assert set(names) == {
            'add', 'list_ros_topics', 'get_perception_status',
            'list_motion_actions', 'play_motion', 'play_nod', 'play_shake', 'stop_motion',
        }, names
        result = await client.call_tool('add', {'a': 100, 'b': 50})
        assert result.content[0].text == '150'
        state = await client.call_tool('get_perception_status', {})
        print('Live MCP perception:', state.content[0].text, flush=True)
        topics = await client.call_tool('list_ros_topics', {})
        assert any(topic['name'] == '/rosout' for topic in
                   json.loads(topics.content[0].text)['topics'])
        actions = await client.call_tool('list_motion_actions', {})
        print('Live MCP motion actions:', actions.content[0].text, flush=True)
        print('Live MCP tools/add/ROS topic queries: OK', flush=True)


def main():
    RospyLogger.findCaller = _modern_find_caller
    rospy.init_node('ai_pkg_smoke', anonymous=True, disable_signals=True)
    asyncio.run(check_mcp())
    namespace = '/llm_agent_smoke_%d' % os.getpid()
    model = ThreadingHTTPServer(('127.0.0.1', 0), ModelFixture)
    thread = threading.Thread(target=model.serve_forever, daemon=True)
    thread.start()
    done = threading.Event()
    answers, errors = [], []

    def receive(message):
        answers.append(message.data)
        done.set()

    output = rospy.Subscriber(namespace + '/output', String, receive)
    error = rospy.Subscriber(namespace + '/error', String, lambda msg: errors.append(msg.data))
    publisher = rospy.Publisher(namespace + '/input', String, queue_size=1)
    rospy.set_param(namespace + '/llm/base_url', 'http://127.0.0.1:%d/v1' % model.server_port)
    rospy.set_param(namespace + '/llm/model', 'local-fixture')
    script = Path(__file__).resolve().parents[1] / 'scripts/llm_agent_node.py'
    process = None
    try:
        process = subprocess.Popen([sys.executable, str(script), '__name:=' + namespace.lstrip('/')])
        deadline = time.monotonic() + 30
        while not publisher.get_num_connections():
            assert process.poll() is None, 'Smoke agent exited during startup'
            assert time.monotonic() < deadline, 'ROS input subscriber did not connect'
            time.sleep(0.1)
        publisher.publish(String(data='请调用工具计算100加50'))
        assert done.wait(30), 'No ROS answer; errors=%r' % errors
        assert answers == ['150'] and not errors, (answers, errors)
        assert ModelFixture.verified_result, 'MCP result did not reach model endpoint'
        print('ROS text -> local model -> live MCP add -> model -> ROS output: 150 OK', flush=True)
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        publisher.unregister()
        output.unregister()
        error.unregister()
        rospy.delete_param(namespace)
        model.shutdown()
        model.server_close()
        thread.join(timeout=2)
        rospy.signal_shutdown('Smoke test complete')


if __name__ == '__main__':
    main()
