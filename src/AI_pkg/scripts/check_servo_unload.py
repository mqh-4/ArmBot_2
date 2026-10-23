#!/usr/bin/env python3
"""Diagnose whether the servo bus can truly be unloaded.

The recorder reads positions continuously, which may itself re-enable the
servos. This test removes that variable: it unloads the arm, stays completely
silent for several seconds so the operator can push a joint by hand, then reads
the positions once and compares.

Run it while supporting the arm:

    rosrun AI_pkg check_servo_unload.py

Interpretation:
  * a joint moved by more than ``--threshold`` ticks while no command was sent
    -> unloading works, and the continuous reads in the recorder were the
       reason drag teaching felt locked;
  * nothing moved
    -> this board/firmware does not release the motors, so hand-guided
       teaching is not possible and a different input method is required.
"""
import argparse
import time

import rospy

from ros_robot_controller.msg import BusServoState, GetBusServoCmd, SetBusServoState
from ros_robot_controller.srv import GetBusServoState

GET_STATE = '/ros_robot_controller/bus_servo/get_state'
SET_STATE = '/ros_robot_controller/bus_servo/set_state'


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--ids', type=int, nargs='+', default=[1, 2, 3, 4, 5, 6])
    parser.add_argument('--hold-seconds', type=float, default=6.0)
    parser.add_argument('--threshold', type=int, default=15,
                        help='tick change that counts as a real movement')
    return parser.parse_args()


def main():
    args = parse_args()
    rospy.init_node('check_servo_unload', anonymous=True)
    get_state = rospy.ServiceProxy(GET_STATE, GetBusServoState)
    get_state.wait_for_service(timeout=10)
    publisher = rospy.Publisher(SET_STATE, SetBusServoState, queue_size=10)
    rospy.sleep(0.5)

    ids = sorted(set(args.ids))

    def read_positions():
        values = {}
        for servo_id in ids:
            request = GetBusServoCmd()
            request.id = servo_id
            request.get_position = 1
            response = get_state([request]).state
            if response and response[0].position:
                values[servo_id] = int(response[0].position[0])
        return values

    def set_torque(enable):
        for servo_id in ids:
            message = SetBusServoState()
            message.duration = 0.0
            state = BusServoState()
            state.present_id = [1, servo_id]
            state.enable_torque = [1, 1 if enable else 0]
            message.state = [state]
            publisher.publish(message)
            rospy.sleep(0.03)

    print('读取初始位置...')
    before = read_positions()
    print('初始位置: %s' % before)

    print('\n卸载力矩（机械臂会变软，请托住）...')
    set_torque(False)
    rospy.sleep(0.4)

    print('现在有 %.0f 秒时间：请用手推一下腕部/肩部，幅度尽量大一些。' % args.hold_seconds)
    print('这期间脚本不会发送或读取任何舵机数据。')
    for remaining in range(int(args.hold_seconds), 0, -1):
        print('  剩余 %d 秒...' % remaining, flush=True)
        time.sleep(1.0)

    print('\n读取一次位置（不发送任何命令）...')
    after = read_positions()
    print('结束位置: %s' % after)

    deltas = {servo_id: after.get(servo_id, 0) - before.get(servo_id, 0)
              for servo_id in ids if servo_id in after and servo_id in before}
    print('位置变化: %s' % deltas)
    biggest = max((abs(value) for value in deltas.values()), default=0)
    print('最大变化: %d 刻度（判定阈值 %d）' % (biggest, args.threshold))

    # Restore torque at the current pose so the arm is not left limp.
    for servo_id, position in after.items():
        message = SetBusServoState()
        message.duration = 0.2
        state = BusServoState()
        state.present_id = [1, servo_id]
        state.position = [1, position]
        message.state.append(state)
        publisher.publish(message)
    rospy.sleep(0.3)
    set_torque(True)
    print('力矩已恢复。')

    if biggest > args.threshold:
        print('\n结论: 卸载有效 —— 静止期间机械臂可以被动移动。')
        print('      因此录制器"边读边拖"会被重新使能，需改为低占空比采样。')
    else:
        print('\n结论: 卸载无效 —— 该控制板/固件没有真正断开舵机驱动力。')
        print('      手拖示教在本机不可行，需要换输入方式。')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
