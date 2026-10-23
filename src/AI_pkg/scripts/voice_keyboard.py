#!/usr/bin/env python3
"""Interactive a/s keyboard controller for the LLM voice pipeline."""
import select
import sys
import termios
import tty

import rospy
from std_srvs.srv import Trigger


START_SERVICE = '/llm_agent_node/start_recording'
STOP_SERVICE = '/llm_agent_node/stop_recording'


def main():
    rospy.init_node('voice_keyboard')
    rospy.loginfo('waiting for %s and %s', START_SERVICE, STOP_SERVICE)
    rospy.wait_for_service(START_SERVICE)
    rospy.wait_for_service(STOP_SERVICE)
    start = rospy.ServiceProxy(START_SERVICE, Trigger)
    stop = rospy.ServiceProxy(STOP_SERVICE, Trigger)
    if not sys.stdin.isatty():
        raise SystemExit('voice_keyboard requires an interactive terminal (docker exec -it)')

    print('Press a to start recording, s to stop and send to LLM, q to quit.')
    active = False
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while not rospy.is_shutdown():
            readable, _, _ = select.select([sys.stdin], [], [], 0.2)
            if not readable:
                continue
            key = sys.stdin.read(1).lower()
            if key == 'a':
                response = start()
                active = response.success
                print('\n' + response.message)
            elif key == 's':
                response = stop()
                if response.success:
                    active = False
                print('\n' + response.message)
            elif key == 'q':
                if active:
                    response = stop()
                    print('\n' + response.message)
                break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == '__main__':
    main()
