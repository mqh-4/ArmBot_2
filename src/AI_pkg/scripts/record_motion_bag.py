#!/usr/bin/env python3
"""Record one demonstrated ROS motion into a rosbag.

Press ``a`` to start recording and ``s`` to stop. The default topic is a
command input to the servo controller. A teaching source (for example
``calibration_arm_teleop.py`` or a joystick node) must publish it while the
recording is active. Moving a powered arm by hand does not publish this topic.
Only the configured command topics are recorded; camera and state topics are
intentionally excluded from the action artifact.
"""
import argparse
import select
import signal
import subprocess
import sys
import termios
import time
import tty
from pathlib import Path

import yaml


DEFAULT_TOPIC = '/servo_controllers/port_id_1/multi_id_pos_dur'


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name', required=True,
                        help='action name, for example nod or shake')
    parser.add_argument('--directory', default='src/AI_pkg/config/actions',
                        help='directory for bag files (default: src/AI_pkg/config/actions)')
    parser.add_argument('--topic', action='append', dest='topics',
                        help='ROS command topic; may be repeated')
    parser.add_argument('--force', action='store_true',
                        help='replace an existing bag with the same name')
    return parser.parse_args()


def read_key():
    ready, _, _ = select.select([sys.stdin], [], [], 0.25)
    if not ready:
        return None
    return sys.stdin.read(1).lower()


def stop_recording(process):
    if process is None or process.poll() is not None:
        return process.returncode if process is not None else 0
    process.send_signal(signal.SIGINT)
    try:
        return process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            return process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            return process.wait()


def validate_bag(output, topics):
    result = subprocess.run(
        ['rosbag', 'info', '--yaml', str(output)],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    if result.returncode:
        raise SystemExit('rosbag info failed: %s' % result.stderr.strip()[-300:])
    try:
        info = yaml.safe_load(result.stdout) or {}
        recorded = {entry.get('topic'): int(entry.get('messages', 0))
                    for entry in info.get('topics', [])}
    except (TypeError, ValueError, yaml.YAMLError) as error:
        raise SystemExit('Could not inspect recorded bag: %s' % error)
    missing = [topic for topic in topics if recorded.get(topic, 0) <= 0]
    if missing:
        raise SystemExit(
            'Bag has no messages for: %s. Start the arm controller and check that '
            'the teaching source publishes the selected topic.' % ', '.join(missing),
        )
    print('Recorded messages: %s' % ', '.join(
        '%s=%d' % (topic, recorded[topic]) for topic in topics))


def main():
    args = parse_args()
    if not args.name or any(char not in 'abcdefghijklmnopqrstuvwxyz0123456789_-'
                             for char in args.name.lower()):
        raise SystemExit('--name may contain only letters, digits, _ and -')
    if not sys.stdin.isatty():
        raise SystemExit('Run this recorder from an interactive terminal.')

    output_dir = Path(args.directory).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    output = output_dir / (args.name + '.bag')
    if output.exists() and not args.force:
        raise SystemExit('%s already exists; use --force to replace it.' % output)
    topics = args.topics or [DEFAULT_TOPIC]
    command = ['rosbag', 'record', '--quiet', '--output-name', str(output)] + topics

    old_settings = termios.tcgetattr(sys.stdin)
    process = None
    started = False
    try:
        tty.setcbreak(sys.stdin.fileno())
        print('Action: %s' % args.name)
        print('Topics: %s' % ', '.join(topics))
        print('Press a to start recording, s to stop, q to quit.')
        while True:
            key = read_key()
            if key == 'q':
                if process is not None and process.poll() is None:
                    print('\nStop the active recording with s first.')
                    continue
                print('\nCancelled.')
                return 0
            if key == 'a' and process is None:
                print('\nRecording... move the arm, then press s.')
                process = subprocess.Popen(command)
                started = True
                continue
            if key == 's' and process is not None:
                return_code = stop_recording(process)
                process = None
                print('\nRecording stopped (exit=%s).' % return_code)
                if return_code not in (0, 130):
                    raise SystemExit('rosbag record failed with exit code %s' % return_code)
                if output.exists() and output.stat().st_size > 0:
                    validate_bag(output, topics)
                    print('Saved: %s (%d bytes)' % (output, output.stat().st_size))
                    return 0
                raise SystemExit('rosbag did not create a non-empty bag.')
            if process is not None and process.poll() is not None:
                raise SystemExit('rosbag record exited before s was pressed.')
            if started:
                # Keep the terminal responsive while rosbag writes its index.
                time.sleep(0.02)
    except KeyboardInterrupt:
        if process is not None:
            stop_recording(process)
        raise
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == '__main__':
    main()
