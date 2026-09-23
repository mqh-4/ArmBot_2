import math


ARM_SERVO_IDS = (6, 5, 4, 3, 2)
ARM_SERVO_FLIPPED = (False, False, True, False, False)
RADIANS_PER_TICK = math.radians(240.0) / 1000.0


def arm_pulses_to_radians(pulses):
    if len(pulses) != len(ARM_SERVO_IDS):
        raise ValueError('five arm pulses are required')
    angles = []
    for pulse, flipped in zip(pulses, ARM_SERVO_FLIPPED):
        ticks = 500.0 - float(pulse) if flipped else float(pulse) - 500.0
        angles.append(ticks * RADIANS_PER_TICK)
    return tuple(angles)


def gripper_pulse_to_radians(pulse):
    # Matches r_joint_controller: init=700, 240 degrees per 1000 ticks.
    return math.radians((float(pulse) - 700.0) * 240.0 / 1000.0)


def interpolate(start, goal, progress):
    amount = min(1.0, max(0.0, float(progress)))
    return float(start) + (float(goal) - float(start)) * amount


def rotate_vector(vector, quaternion):
    """Rotate xyz by an xyzw quaternion without external dependencies."""
    vx, vy, vz = (float(value) for value in vector)
    qx, qy, qz, qw = (float(value) for value in quaternion)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )
