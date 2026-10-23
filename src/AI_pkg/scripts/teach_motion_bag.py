#!/usr/bin/env python3
"""Drag-teach one arm motion into a bag the MCP play tools can replay.

Unlike ``record_motion_bag.py`` this does not record a command topic: while the
servos are unloaded nothing publishes there when the arm is moved by hand, so
the recorder reads real positions from
``/ros_robot_controller/bus_servo/get_state`` and writes
``/servo_controllers/port_id_1/multi_id_pos_dur`` messages itself.

Every torque change is verified by reading the servo's load/unload state back.
If the servos refuse to unload, the script aborts instead of letting the
operator push against a rigid arm.

Safety: unloading the servos makes the arm go limp. Support the arm with one
hand before confirming, keep supporting it while moving, and keep supporting it
until the script reports that torque is back on.

Usage:
    rosrun AI_pkg teach_motion_bag.py --name nod --force
    rosrun AI_pkg teach_motion_bag.py --check     # read-only preflight
"""
import argparse
import datetime
import math
import os
import select
import sys
import tempfile
import termios
import time
import tty
from pathlib import Path

import rosbag
import rospy

from ai_pkg.motion_teach import (clamp_position, command_plan, cosine_ease,
                                 plan_summary, resample, smooth, trim_idle)
from hiwonder_servo_msgs.msg import MultiRawIdPosDur, RawIdPosDur
from ros_robot_controller.msg import BusServoState, GetBusServoCmd, SetBusServoState
from ros_robot_controller.srv import GetBusServoState

TOPIC = '/servo_controllers/port_id_1/multi_id_pos_dur'
BATTERY_TOPIC = '/ros_robot_controller/battery'
MIN_BATTERY_MV = 6500
DEFAULT_IDS = (1, 2, 3, 4, 5, 6)
# Bus servo report convention: 0 means loaded (rigid), 1 means unloaded.
TORQUE_LOADED = 0
TORQUE_RELEASED = 1
TORQUE_RETRIES = 3
TORQUE_CHECK_INTERVAL = 2.0


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--name', help='action name written to <directory>/<name>.bag')
    parser.add_argument('--directory', default='src/AI_pkg/config/actions',
                        help='bag directory (default: src/AI_pkg/config/actions)')
    parser.add_argument('--ids', type=int, nargs='+', default=list(DEFAULT_IDS),
                        help='servo ids to teach (default: 1 2 3 4 5 6)')
    parser.add_argument('--limp-ids', type=int, nargs='+',
                        help='only unload these servos; the rest keep holding the arm '
                             '(e.g. --limp-ids 6 to hand-turn the base)')
    parser.add_argument('--rate', type=float, default=15.0,
                        help='playback sample rate in Hz (default: 15)')
    parser.add_argument('--entry-duration', type=float, default=1.0,
                        help='seconds used to ease into the taught start pose')
    parser.add_argument('--min-delta', type=int, default=2,
                        help='minimum tick change before a servo command is emitted')
    parser.add_argument('--smooth', type=float, default=0.35,
                        help='smoothing factor 0<a<=1; 1 disables smoothing')
    parser.add_argument('--check', action='store_true',
                        help='only print battery, positions, torque and sample rate')
    parser.add_argument('--from-bag', help='reprocess an existing action bag instead of teaching')
    parser.add_argument('--start', type=float, help='segment start in seconds (with --from-bag)')
    parser.add_argument('--end', type=float, help='segment end in seconds (with --from-bag)')
    parser.add_argument('--target-duration', type=float,
                        help='time-scale the segment to this many seconds')
    parser.add_argument('--amplitude-scale', type=float, default=1.0,
                        help='scale motion amplitude around the segment centre (from-bag mode)')
    parser.add_argument('--continuous', action='store_true',
                        help='emit one command per sample so playback timing is uniform')
    parser.add_argument('--ease', action='store_true',
                        help='rebuild the trajectory with cosine segments between turning points')
    parser.add_argument('--sine', action='store_true',
                        help='generate an analytically smooth sine nod instead of processing a bag')
    parser.add_argument('--servo', type=int, default=3, help='servo id used by --sine (default 3)')
    parser.add_argument('--center', type=float, default=227.0, help='sine centre in encoder ticks')
    parser.add_argument('--amplitude', type=float, default=80.0, help='sine amplitude in encoder ticks')
    parser.add_argument('--period', type=float, default=2.4, help='seconds per full nod cycle')
    parser.add_argument('--cycles', type=float, default=2.5, help='number of nod cycles')
    parser.add_argument('--invert', action='store_true', help='start the sine upward instead of downward')
    parser.add_argument('--no-torque', action='store_true',
                        help='do not unload the servos; only useful with --allow-static')
    parser.add_argument('--allow-static', action='store_true',
                        help='write the bag even when no motion was detected')
    parser.add_argument('--force', action='store_true',
                        help='replace an existing bag, keeping a timestamped backup')
    parser.add_argument('--yes', action='store_true',
                        help='skip the interactive safety confirmation')
    return parser.parse_args()


def read_positions(get_state, ids, timeout_retries=2):
    """Read real positions; returns only servos that answered."""
    positions = {}
    for servo_id in ids:
        request = GetBusServoCmd()
        request.id = servo_id
        request.get_position = 1
        for _ in range(timeout_retries):
            response = get_state([request]).state
            if response and response[0].position:
                positions[servo_id] = int(response[0].position[0])
                break
    return positions


def read_torque(get_state, ids):
    """Read the load/unload state; 0 = loaded, 1 = unloaded."""
    states = {}
    for servo_id in ids:
        request = GetBusServoCmd()
        request.id = servo_id
        request.get_torque_state = 1
        try:
            response = get_state([request]).state
        except Exception:
            continue
        if response and response[0].enable_torque:
            states[servo_id] = int(response[0].enable_torque[0])
    return states


def battery_mv():
    from std_msgs.msg import UInt16
    try:
        message = rospy.wait_for_message(BATTERY_TOPIC, UInt16, timeout=5)
    except rospy.ROSException:
        return None
    return int(message.data)


def send_torque(publisher, ids, enable, settle=0.03):
    """Send one single-servo message per id; a lost frame only costs one retry."""
    for servo_id in ids:
        message = SetBusServoState()
        message.duration = 0.0
        state = BusServoState()
        state.present_id = [1, servo_id]
        state.enable_torque = [1, 1 if enable else 0]
        message.state = [state]
        publisher.publish(message)
        if settle:
            rospy.sleep(settle)


def read_positions_keep_limp(get_state, publisher, ids):
    """Read positions and unload each servo right after reading it.

    Reading a position re-enables that servo, so a plain continuous read loop
    keeps the whole arm rigid. Interleaving the unload restores the limp state
    after every read and leaves the arm back-drivable most of the time.
    """
    positions = {}
    for servo_id in ids:
        request = GetBusServoCmd()
        request.id = servo_id
        request.get_position = 1
        try:
            response = get_state([request]).state
        except Exception:
            response = None
        if response and response[0].position:
            positions[servo_id] = int(response[0].position[0])
        send_torque(publisher, [servo_id], False, settle=0)
    # If a read re-enables the whole bus, make sure every servo is limp again.
    send_torque(publisher, ids, False, settle=0)
    return positions


def send_hold(publisher, positions, duration=0.2):
    """Command the pose the arm is already in, so enabling torque cannot jerk it."""
    message = SetBusServoState()
    message.duration = duration
    for servo_id, position in positions.items():
        state = BusServoState()
        state.present_id = [1, servo_id]
        state.position = [1, int(position)]
        message.state.append(state)
    publisher.publish(message)


def release_torque(publisher, get_state, ids):
    """Unload the servos and verify it. Returns the readback dict or ``None``."""
    for attempt in range(1, TORQUE_RETRIES + 1):
        send_torque(publisher, ids, False)
        rospy.sleep(0.4)
        states = read_torque(get_state, ids)
        verified = states and all(value == TORQUE_RELEASED for value in states.values())
        # The verification read itself re-enables the servos; unload again.
        send_torque(publisher, ids, False, settle=0.005)
        if verified:
            return states
        print('第 %d 次卸载未完全生效，读回 %s，重试...' % (attempt, states))
    return None


def restore_torque(publisher, get_state, positions):
    """Hold the current pose, re-load the servos and verify it."""
    hold = dict(positions)
    try:
        hold.update(read_positions(get_state, list(hold)))
    except Exception as error:
        print('回读位置失败，按最后采样姿态上力矩: %s' % error)
    send_hold(publisher, hold, 0.2)
    rospy.sleep(0.25)
    ids = list(hold)
    for attempt in range(1, TORQUE_RETRIES + 1):
        send_torque(publisher, ids, True)
        rospy.sleep(0.4)
        states = read_torque(get_state, ids)
        if states and all(value == TORQUE_LOADED for value in states.values()):
            return states
        print('第 %d 次恢复力矩未完全生效，读回 %s，重试...' % (attempt, states))
    return None


def read_key(timeout=0.0):
    """Non-blocking key poll; waiting here would throttle the sample rate."""
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return None
    return sys.stdin.read(1).lower()


def wait_for_key(accepted, prompt):
    print(prompt, end='', flush=True)
    while not rospy.is_shutdown():
        key = read_key(0.2)
        if key is None:
            continue
        if key == 'q':
            print('\n已取消')
            return None
        if key in accepted:
            print()
            return key
    return None


def confirm(text):
    print(text)
    print('确认请完整输入 yes: ', end='', flush=True)
    return sys.stdin.readline().strip().lower() == 'yes'


def write_bag(path, plan, topic=TOPIC):
    temporary = tempfile.NamedTemporaryFile(
        prefix='teach-', suffix='.bag', dir=str(path.parent), delete=False,
    )
    temporary.close()
    # Wall-clock stamps keep the writer usable even without a ROS clock source.
    stamp = rospy.Time.from_sec(time.time())
    try:
        with rosbag.Bag(temporary.name, 'w') as bag:
            for offset, entries in plan:
                message = MultiRawIdPosDur()
                message.id_pos_dur_list = [
                    RawIdPosDur(id=servo_id, position=position, duration=duration)
                    for servo_id, position, duration in entries
                ]
                bag.write(topic, message, stamp + rospy.Duration.from_sec(offset))
        os.replace(temporary.name, str(path))
    except BaseException:
        try:
            os.unlink(temporary.name)
        except OSError:
            pass
        raise


def load_bag_samples(path, start=None, end=None):
    """Rebuild a position timeline from a recorded command bag."""
    current = {}
    samples = []
    with rosbag.Bag(str(path)) as bag:
        origin = bag.get_start_time()
        for _, message, stamp in bag.read_messages():
            for entry in message.id_pos_dur_list:
                current[entry.id] = int(entry.position)
            if not current:
                continue
            offset = stamp.to_sec() - origin
            if start is not None and offset < start:
                continue
            if end is not None and offset > end:
                continue
            samples.append((offset, dict(current)))
    return samples


def fill_missing(samples):
    """Forward/backward fill so a servo that starts later is not driven to 0."""
    if not samples:
        return samples
    ids = sorted({servo_id for _, positions in samples for servo_id in positions})
    first = {}
    for _, positions in samples:
        for servo_id in ids:
            if servo_id in positions and servo_id not in first:
                first[servo_id] = positions[servo_id]
    last = dict(first)
    filled = []
    for time, positions in samples:
        for servo_id in ids:
            if servo_id in positions:
                last[servo_id] = positions[servo_id]
            else:
                positions[servo_id] = last.get(servo_id, first.get(servo_id, 500))
        filled.append((time, dict(positions)))
    return filled


def reprocess(args):
    samples = fill_missing(load_bag_samples(args.from_bag, args.start, args.end))
    if len(samples) < 5:
        raise SystemExit('该时间段内没有足够的舵机指令')
    trimmed = trim_idle(samples)
    # Keep only servos that actually move: a constant servo in the window would
    # otherwise be commanded to that old pose in the entry message.
    ids = {servo_id for _, positions in trimmed for servo_id in positions}
    moving = {servo_id for servo_id in ids
              if max(p[servo_id] for _, p in trimmed if servo_id in p)
              - min(p[servo_id] for _, p in trimmed if servo_id in p) > 10}
    if not moving:
        raise SystemExit('该时间段内没有舵机位移，无法生成动作')
    trimmed = [(time, {servo_id: value for servo_id, value in positions.items()
                       if servo_id in moving}) for time, positions in trimmed]
    print('参与动作的舵机: %s' % sorted(moving))
    if args.amplitude_scale != 1.0:
        if args.amplitude_scale <= 0:
            raise SystemExit('--amplitude-scale 必须大于 0')
        for servo_id in moving:
            values = [positions[servo_id] for _, positions in trimmed
                      if servo_id in positions]
            centre = (max(values) + min(values)) / 2.0
            for _, positions in trimmed:
                if servo_id in positions:
                    positions[servo_id] = centre + (positions[servo_id] - centre) * args.amplitude_scale
        print('幅度缩放: %.2f（围绕各自中心）' % args.amplitude_scale)
    if args.smooth < 1.0:
        trimmed = smooth(trimmed, max(0.05, args.smooth))
    span = trimmed[-1][0] - trimmed[0][0]
    source_span = span
    if args.target_duration:
        # Scale first, then resample, so the bag carries about rate Hz.
        factor = args.target_duration / span
        origin = trimmed[0][0]
        trimmed = [((time - origin) * factor, positions) for time, positions in trimmed]
        span = args.target_duration
    uniform = resample(trimmed, args.rate)
    if args.ease:
        uniform = cosine_ease(uniform)
        print('已用余弦插值重建关键点之间的轨迹')
    if args.continuous:
        # One command per sample: variable gaps make the servos look jerky.
        servo_ids = sorted({servo_id for _, positions in uniform for servo_id in positions})
        plan = [(0.0, [(servo_id, clamp_position(uniform[0][1][servo_id]),
                        args.entry_duration) for servo_id in servo_ids])]
        for index in range(1, len(uniform)):
            interval = uniform[index][0] - uniform[index - 1][0]
            interval = max(0.02, min(0.2, interval))
            plan.append((uniform[index][0] - uniform[0][0],
                         [(servo_id, clamp_position(uniform[index][1][servo_id]), interval)
                          for servo_id in servo_ids]))
    else:
        plan = command_plan(uniform, entry_duration=args.entry_duration,
                            min_delta=args.min_delta)
    summary = plan_summary(plan)
    target = Path(args.directory).expanduser() / (args.name + '.bag')
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        backup_dir = target.parent / 'backups'
        backup_dir.mkdir(parents=True, exist_ok=True)
        backup = backup_dir / ('%s_backup_%s.bag' % (
            args.name, datetime.datetime.now().strftime('%Y%m%d_%H%M%S')))
        os.replace(str(target), str(backup))
        print('已备份旧动作包: %s' % backup.name)
    write_bag(target, plan)
    print('\n=== 重写完成 ===')
    print('文件      : %s' % target)
    print('示教区间  : %.2f 秒（原 %.2f 秒）' % (span, source_span))
    print('播放时长  : %.2f 秒（含 %.2f 秒入场）' % (summary['duration'], args.entry_duration))
    print('消息/指令 : %d / %d' % (summary['messages'], summary['commands']))
    print('各舵机指令: %s' % summary['per_servo'])


def generate_sine(args):
    """Build a perfectly smooth sine trajectory, one command per sample."""
    if args.period <= 0 or args.cycles <= 0:
        raise SystemExit('--period 和 --cycles 必须大于 0')
    if args.rate <= 0:
        raise SystemExit('--rate 必须大于 0')
    duration = args.period * args.cycles
    count = max(1, int(round(duration * args.rate)))
    samples = []
    for index in range(count + 1):
        time = index / float(args.rate)
        sign = -1.0 if args.invert else 1.0
        value = args.center - sign * args.amplitude * math.sin(2.0 * math.pi * time / args.period)
        samples.append((time, {args.servo: value}))
    interval = 1.0 / args.rate
    plan = [(0.0, [(args.servo, clamp_position(samples[0][1][args.servo]),
                    args.entry_duration)])]
    for index in range(1, len(samples)):
        plan.append((samples[index][0],
                     [(args.servo, clamp_position(samples[index][1][args.servo]), interval)]))
    target = Path(args.directory).expanduser() / (args.name + '.bag')
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        backup_dir = target.parent / 'backups'
        backup_dir.mkdir(parents=True, exist_ok=True)
        backup = backup_dir / ('%s_backup_%s.bag' % (
            args.name, datetime.datetime.now().strftime('%Y%m%d_%H%M%S')))
        os.replace(str(target), str(backup))
        print('已备份旧动作包: %s' % backup.name)
    write_bag(target, plan)
    summary = plan_summary(plan)
    lowest = args.center - args.amplitude
    highest = args.center + args.amplitude
    print('\n=== 正弦动作生成完成 ===')
    print('文件      : %s' % target)
    print('舵机      : %s' % args.servo)
    print('中心/幅度 : %.0f ± %.0f（%d - %d）' % (args.center, args.amplitude, lowest, highest))
    print('周期/次数 : %.2f 秒 × %.2f 次 = %.2f 秒' % (args.period, args.cycles, duration))
    print('采样      : %d 条，间隔 %.3f 秒' % (summary['messages'], interval))
    print('播放时长  : %.2f 秒（含 %.2f 秒入场）' % (summary['duration'], args.entry_duration))
    print('提示: 余弦项由正弦保证，速度在极值处自然归零，不存在折点。')


def main():
    args = parse_args()
    if args.sine:
        if not args.name:
            raise SystemExit('--sine 需要 --name 指定输出动作名')
        generate_sine(args)
        return
    if args.from_bag:
        if not args.name:
            raise SystemExit('--from-bag 需要 --name 指定输出动作名')
        reprocess(args)
        return
    rospy.init_node('teach_motion_bag', anonymous=True)
    # Unloading only the joint you move keeps the arm's weight on the other
    # servos, which makes hand guidance far easier (and safer).
    ids = sorted(set(args.limp_ids or args.ids))

    get_state = rospy.ServiceProxy(
        '/ros_robot_controller/bus_servo/get_state', GetBusServoState,
    )
    rospy.loginfo('waiting for the servo state service ...')
    get_state.wait_for_service(timeout=10)
    torque_publisher = rospy.Publisher(
        '/ros_robot_controller/bus_servo/set_state', SetBusServoState, queue_size=10,
    )
    rospy.sleep(0.5)

    volts = battery_mv()
    print('电池: %s mV' % ('unknown' if volts is None else volts))
    if volts is not None and volts < MIN_BATTERY_MV:
        raise SystemExit('电池电压过低（%d mV），请充电后再示教。' % volts)

    started = time.time()
    positions = read_positions(get_state, ids)
    if not positions:
        raise SystemExit(
            '所有舵机都没有响应，无法示教。请依次检查：\n'
            '  1) 机械臂电源开关是否打开（控制板电池有电压不代表舵机供电已开）；\n'
            '  2) 舵机总线/排线是否插紧；\n'
            '  3) 是否刚播放过长动作导致舵机进入过载保护，需要断电 10 秒后重启。',
        )
    missing = [servo_id for servo_id in ids if servo_id not in positions]
    print('参与示教的舵机: %s' % ids)
    print('舵机位置: %s' % positions)
    if missing:
        print('未响应舵机: %s（这些将被跳过）' % missing)
    torque = read_torque(get_state, list(positions))
    print('力矩状态: %s（0=已加载/刚性，1=已卸载）' % torque)

    if args.check:
        samples = []
        for _ in range(10):
            samples.append((time.time(), read_positions(get_state, list(positions))))
        cycle = (samples[-1][0] - samples[0][0]) / max(1, len(samples) - 1)
        print('采样周期约 %.0f ms（约 %.1f Hz）' % (cycle * 1000, 1.0 / cycle))
        print('读取耗时: %.0f ms' % ((time.time() - started) * 1000))
        return

    if not args.name:
        raise SystemExit('缺少 --name（例如 --name nod）')
    if any(char not in 'abcdefghijklmnopqrstuvwxyz0123456789_-' for char in args.name.lower()):
        raise SystemExit('--name 只能包含字母、数字、_ 和 -')
    if not sys.stdin.isatty():
        raise SystemExit('需要交互终端来按 a/s 控制录制（docker exec -it）')

    directory = Path(args.directory).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    target = directory / (args.name + '.bag')
    if target.exists() and not args.force:
        raise SystemExit('%s 已存在；确认要覆盖请加 --force（会保留时间戳备份）' % target)

    if not args.no_torque:
        print('\n即将卸载舵机力矩（机械臂会变软下坠）。')
        print('请先用一只手托住机械臂，另一只手准备操作键盘。')
        if not args.yes and not confirm('卸载前请确认已托住机械臂。'):
            raise SystemExit('已取消，未改变力矩状态。')

    old_settings = termios.tcgetattr(sys.stdin)
    samples = []
    torque_released = False
    try:
        tty.setcbreak(sys.stdin.fileno())
        if not args.no_torque:
            verified = release_torque(torque_publisher, get_state, list(positions))
            if verified is None:
                raise SystemExit(
                    '舵机拒绝卸载力矩，已中止（未写包）。\n'
                    '请确认机械臂供电正常、未被其他程序占用；必要时断电 10 秒后重试。',
                )
            torque_released = True
            print('已确认全部卸载: %s' % verified)
            print('现在可以手动摆姿态了（托住机械臂）。')

        key = wait_for_key({'a'}, '托住机械臂，按 a 开始记录，q 取消: ')
        if key is None:
            return
        print('记录中... 完成动作后按 s 结束（q 取消）')
        last = dict(positions)
        while not rospy.is_shutdown():
            if torque_released:
                current = read_positions_keep_limp(
                    get_state, torque_publisher, list(positions))
            else:
                current = read_positions(get_state, list(positions))
            filled = dict(last)
            filled.update(current)
            samples.append((time.time(), filled))
            last = filled

            key = read_key()
            if key == 's':
                break
            if key == 'q':
                print('\n已取消，不写包')
                return
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        if torque_released:
            hold = dict(positions)
            if samples:
                hold.update(samples[-1][1])
            verified = restore_torque(torque_publisher, get_state, hold)
            if verified is None:
                print('!!! 恢复力矩失败，请立即托住机械臂或断电！')
            else:
                print('力矩已恢复并确认: %s，可以松手了。' % verified)

    if len(samples) < 5:
        raise SystemExit('采样过少（%d 个），未写包。' % len(samples))

    trimmed = trim_idle(samples)
    if len(trimmed) < 2:
        raise SystemExit('未能识别动作区间，未写包。')
    if args.smooth < 1.0:
        trimmed = smooth(trimmed, max(0.05, args.smooth))
    uniform = resample(trimmed, args.rate)
    if args.ease:
        uniform = cosine_ease(uniform)
        print('已用余弦插值重建关键点之间的轨迹')
    plan = command_plan(uniform, entry_duration=args.entry_duration,
                        min_delta=args.min_delta)
    summary = plan_summary(plan)

    span = uniform[-1][0] - uniform[0][0]
    if span < 0.3 and not args.allow_static:
        raise SystemExit(
            '动作时长只有 %.2f 秒，看起来机械臂没有移动。未写包。\n'
            '如果只是想验证流程，加 --allow-static。' % span,
        )

    if target.exists():
        backup_dir = target.parent / 'backups'
        backup_dir.mkdir(parents=True, exist_ok=True)
        backup = backup_dir / ('%s_backup_%s.bag' % (
            args.name, datetime.datetime.now().strftime('%Y%m%d_%H%M%S')))
        os.replace(str(target), str(backup))
        print('已备份旧动作包: %s' % backup.name)

    write_bag(target, plan)
    print('\n=== 写入完成 ===')
    print('文件      : %s' % target)
    print('采样      : %d 个（原始 %d 个）' % (len(uniform), len(samples)))
    print('示教区间  : %.2f 秒' % span)
    print('播放时长  : %.2f 秒（含 %.2f 秒入场）' % (summary['duration'], args.entry_duration))
    print('消息/指令 : %d / %d' % (summary['messages'], summary['commands']))
    print('各舵机指令: %s' % summary['per_servo'])
    print('提示: 旧包时长 68.9 秒，新包应明显更短；用 list_motion_actions 可确认动作已可播放。')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
