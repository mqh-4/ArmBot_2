#!/usr/bin/env python3
"""ROS-independent command, projection, and target-selection logic."""
import json
import math
import re
from dataclasses import asdict, dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


SUPPORTED_INTENTS = {'cleanup', 'stop', 'pause', 'resume', 'status'}


def base_to_ik_position(position: Sequence[float]) -> Tuple[float, float, float]:
    """Convert a ``base_link`` point to the legacy ArmPi IK frame.

    The custom ArmPi FK library uses an XY frame rotated -90 degrees from the
    URDF/TF ``base_link`` frame.  Its endpoint is therefore reported by TF as
    ``(-y_ik, x_ik)``.  Keep this conversion at the desk-cleaning boundary so
    perception and workspace checks remain expressed in the documented TF
    frame, while the old IK service receives the coordinates it expects.
    """
    if len(position) != 3:
        raise ValueError('position must contain exactly three values')
    x_base, y_base, z_base = (float(value) for value in position)
    return y_base, -x_base, z_base


def sphere_center_depth(
    surface_depth: float,
    bbox: Sequence[float],
    fx: float,
    fy: float,
) -> float:
    """Estimate a round object's center depth from its visible bbox."""
    if len(bbox) != 4 or fx <= 0 or fy <= 0:
        raise ValueError('invalid bbox or camera focal length')
    x1, y1, x2, y2 = (float(value) for value in bbox)
    width = (x2 - x1) * float(surface_depth) / float(fx)
    height = (y2 - y1) * float(surface_depth) / float(fy)
    return float(surface_depth) + 0.5 * max(width, height)


def adaptive_gripper_open_pulse(
    object_size: Sequence[float],
    base_pulse: float,
    reference_width: float,
    width_margin: float,
    pulse_per_meter: float,
    min_pulse: float,
    max_pulse: float,
    extra_open_pulse: float = 0.0,
) -> int:
    """Choose a safe opening pulse from an object's measured width.

    ArmPi's gripper opens farther as the servo pulse decreases.  ``base_pulse``
    is the calibrated opening for ``reference_width``; a larger measured
    object therefore produces a smaller pulse.  The extra physical margin is
    applied before conversion and the result is clamped away from the servo's
    mechanical end stops.
    """
    dimensions = []
    for value in object_size or ():
        try:
            value = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(value) and value > 0.0:
            dimensions.append(value)
    if not dimensions:
        desired_width = float(reference_width)
    else:
        desired_width = max(dimensions) + max(0.0, float(width_margin))
    pulse = float(base_pulse) - max(
        0.0, desired_width - float(reference_width),
    ) * max(0.0, float(pulse_per_meter)) - float(extra_open_pulse)
    lower = min(float(min_pulse), float(max_pulse))
    upper = max(float(min_pulse), float(max_pulse))
    return int(round(min(upper, max(lower, pulse))))


@dataclass
class CleanupCommand:
    intent: str
    allowed_classes: List[str] = field(default_factory=list)
    excluded_classes: List[str] = field(default_factory=list)
    destination: str = 'default_tray'
    announce_each: bool = True
    max_objects: int = 0

    def validate(self, known_classes: Iterable[str], destinations: Iterable[str]):
        if not isinstance(self.intent, str):
            raise ValueError('intent must be a string')
        if not isinstance(self.allowed_classes, list) or not all(
            isinstance(item, str) for item in self.allowed_classes
        ):
            raise ValueError('allowed_classes must be a list of strings')
        if not isinstance(self.excluded_classes, list) or not all(
            isinstance(item, str) for item in self.excluded_classes
        ):
            raise ValueError('excluded_classes must be a list of strings')
        if not isinstance(self.destination, str):
            raise ValueError('destination must be a string')
        if not isinstance(self.announce_each, bool):
            raise ValueError('announce_each must be a boolean')
        if isinstance(self.max_objects, bool) or not isinstance(self.max_objects, int):
            raise ValueError('max_objects must be an integer')
        known = set(known_classes)
        available_destinations = set(destinations)
        if self.intent not in SUPPORTED_INTENTS:
            raise ValueError('unsupported intent: %s' % self.intent)
        unknown = (set(self.allowed_classes) | set(self.excluded_classes)) - known
        if unknown:
            raise ValueError('unknown classes: %s' % ', '.join(sorted(unknown)))
        if self.destination not in available_destinations:
            raise ValueError('unknown destination: %s' % self.destination)
        if self.max_objects < 0 or self.max_objects > 100:
            raise ValueError('max_objects must be between 0 and 100')
        return self

    def to_json(self) -> str:
        return json.dumps(asdict(self), ensure_ascii=False, separators=(',', ':'))

    @classmethod
    def from_json(cls, raw: str):
        data = json.loads(raw)
        if not isinstance(data, dict):
            raise ValueError('command must be a JSON object')
        allowed_keys = {
            'intent', 'allowed_classes', 'excluded_classes', 'destination',
            'announce_each', 'max_objects',
        }
        unknown_keys = set(data) - allowed_keys
        if unknown_keys:
            raise ValueError('unknown command fields: %s' % ', '.join(sorted(unknown_keys)))
        return cls(**data)


def _mentioned_classes(text: str, aliases: Dict[str, Sequence[str]]) -> List[str]:
    candidates = []
    for class_name, words in aliases.items():
        for word in words:
            if not word:
                continue
            start = text.find(word)
            while start >= 0:
                candidates.append((start, start + len(word), -len(word), class_name, word))
                start = text.find(word, start + 1)
    # Prefer the most specific alias when aliases overlap (e.g. "铅笔" and "笔").
    selected = []
    occupied = set()
    for start, end, _negative_length, class_name, word in sorted(
        candidates, key=lambda item: (item[2], item[0]),
    ):
        span = set(range(start, end))
        if span & occupied:
            continue
        occupied |= span
        selected.append((start, class_name, word))
    selected.sort()
    return selected


def parse_command(
    text: str,
    aliases: Dict[str, Sequence[str]],
    destination_aliases: Optional[Dict[str, Sequence[str]]] = None,
) -> Optional[CleanupCommand]:
    """Parse common commands locally before falling back to an LLM."""
    normalized = re.sub(r'[，。！？,.!?\s]+', '', text.lower())
    if not normalized:
        return None
    if any(word in normalized for word in ('停止', '终止', '停下', 'stop')):
        return CleanupCommand(intent='stop')
    if any(word in normalized for word in ('暂停', '等一下', 'pause')):
        return CleanupCommand(intent='pause')
    if any(word in normalized for word in ('继续', '接着', 'resume')):
        return CleanupCommand(intent='resume')
    if any(word in normalized for word in ('状态', '进度', '怎么样了', 'status')):
        return CleanupCommand(intent='status')
    # "开始作业" is the dedicated fruit-to-box command for this deployment. It
    # deliberately limits this intent to the supported fruit classes.
    if '开始作业' in normalized:
        fruit_classes = [
            class_name for class_name in ('apple', 'orange', 'banana')
            if class_name in aliases
        ]
        destination = (
            'apple_box' if 'apple_box' in (destination_aliases or {})
            else 'default_tray'
        )
        return CleanupCommand(
            intent='cleanup',
            allowed_classes=fruit_classes,
            destination=destination,
        )
    if not any(word in normalized for word in (
        '清理', '收拾', '整理', '打扫', '夹取', '加取', '抓取', '拿取', '拾取',
        'cleanup', 'clean',
    )):
        return None

    mentioned = _mentioned_classes(normalized, aliases)
    excluded = []
    allowed = []
    for _start, class_name, matched_word in mentioned:
        exclusion_pattern = r'(不要|别动|别清理|不清理|跳过)(碰|清理|整理|移动|夹取)?' + re.escape(matched_word)
        is_excluded = re.search(exclusion_pattern, normalized) is not None
        (excluded if is_excluded else allowed).append(class_name)

    # This deployment has one spoken command and one physical destination:
    # the recorded apple box. Fall back to the legacy tray only when an older
    # configuration does not define apple_box.
    destination = (
        'apple_box' if 'apple_box' in (destination_aliases or {})
        else 'default_tray'
    )
    # Resolve the specific apple-box phrase before generic aliases such as
    # "盒子" on default_tray, otherwise "放入盒子" silently used the tray.
    apple_box_words = (destination_aliases or {}).get(
        'apple_box', ('苹果盒', '放入盒子', '放到盒子', 'applebox', 'box'),
    )
    if any(word in normalized for word in apple_box_words):
        destination = 'apple_box'
    else:
        for name, words in (destination_aliases or {}).items():
            if name == 'apple_box':
                continue
            if any(word in normalized for word in words):
                destination = name
                break

    max_objects = 0
    count_match = re.search(r'(\d+)个', normalized)
    if count_match:
        max_objects = min(int(count_match.group(1)), 100)
    else:
        chinese_count = re.search(r'([一二两三四五六七八九十])个', normalized)
        if chinese_count:
            max_objects = {
                '一': 1, '二': 2, '两': 2, '三': 3, '四': 4, '五': 5,
                '六': 6, '七': 7, '八': 8, '九': 9, '十': 10,
            }[chinese_count.group(1)]

    # The current operating mode has one spoken command: clean the desk.
    # If the user says a grasping phrase without a reliable object/destination
    # phrase, keep the same broad cleanup behavior instead of rejecting it.
    return CleanupCommand(
        intent='cleanup',
        allowed_classes=allowed,
        excluded_classes=excluded,
        destination=destination,
        max_objects=max_objects,
    )


@dataclass
class Detection:
    object_id: str
    class_name: str
    display_name: str
    confidence: float
    position: Tuple[float, float, float]
    bbox: Tuple[int, int, int, int] = (0, 0, 0, 0)
    grasp_yaw: float = 0.0
    size: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class Workspace:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float
    # Optional measured horizontal reach envelope.  Each entry contains a
    # height ``z`` and the largest safe horizontal radius at that height.
    # Keeping this optional preserves the simple rectangular workspace used
    # by simulation and older configurations.
    radial_reach: Sequence[Dict[str, float]] = field(default_factory=tuple)

    def __post_init__(self):
        profile = []
        for entry in self.radial_reach or ():
            if not isinstance(entry, dict) or 'z' not in entry or 'max' not in entry:
                raise ValueError('radial_reach entries require z and max')
            z = float(entry['z'])
            radius_max = float(entry['max'])
            radius_min = float(entry.get('min', 0.0))
            if not all(math.isfinite(value) for value in (z, radius_min, radius_max)):
                raise ValueError('radial_reach values must be finite')
            if radius_min < 0.0 or radius_max < radius_min:
                raise ValueError('radial_reach radius limits are invalid')
            profile.append((z, radius_min, radius_max))
        self.radial_reach = tuple(sorted(profile))

    def radial_limits(self, z: float) -> Tuple[float, float]:
        """Return the interpolated (minimum, maximum) XY radius at ``z``."""
        if not self.radial_reach:
            return 0.0, float('inf')
        if z <= self.radial_reach[0][0]:
            _, radius_min, radius_max = self.radial_reach[0]
            return radius_min, radius_max
        if z >= self.radial_reach[-1][0]:
            _, radius_min, radius_max = self.radial_reach[-1]
            return radius_min, radius_max
        for lower, upper in zip(self.radial_reach, self.radial_reach[1:]):
            if lower[0] <= z <= upper[0]:
                span = upper[0] - lower[0]
                ratio = (z - lower[0]) / span if span else 0.0
                radius_min = lower[1] + ratio * (upper[1] - lower[1])
                radius_max = lower[2] + ratio * (upper[2] - lower[2])
                return radius_min, radius_max
        # The sorted profile and bounds above make this unreachable, but a
        # conservative result is preferable if a malformed profile slips in.
        return 0.0, 0.0

    def contains(self, position: Sequence[float]) -> bool:
        if len(position) != 3:
            return False
        x, y, z = (float(value) for value in position)
        if not all(math.isfinite(value) for value in (x, y, z)):
            return False
        radius = math.hypot(x, y)
        radius_min, radius_max = self.radial_limits(z)
        return (
            self.x_min <= x <= self.x_max
            and self.y_min <= y <= self.y_max
            and self.z_min <= z <= self.z_max
            and radius_min <= radius <= radius_max
        )


class PinholeProjector:
    def __init__(self, fx: float, fy: float, cx: float, cy: float):
        if fx <= 0 or fy <= 0:
            raise ValueError('camera focal lengths must be positive')
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)

    def project(self, u: float, v: float, depth_m: float) -> Tuple[float, float, float]:
        if not math.isfinite(depth_m) or depth_m <= 0:
            raise ValueError('depth must be a positive finite value')
        return (
            (float(u) - self.cx) * depth_m / self.fx,
            (float(v) - self.cy) * depth_m / self.fy,
            depth_m,
        )


def select_target(
    detections: Sequence[Detection],
    workspace: Workspace,
    allowed_classes: Sequence[str],
    excluded_classes: Sequence[str],
    failed_ids: Iterable[str] = (),
) -> Optional[Detection]:
    allowed = set(allowed_classes)
    excluded = set(excluded_classes)
    failed = set(failed_ids)
    candidates = [
        item for item in detections
        if item.object_id not in failed
        and item.class_name not in excluded
        and (not allowed or item.class_name in allowed)
        and workspace.contains(item.position)
    ]
    if not candidates:
        return None

    # Prefer reliable, nearby objects. Confidence dominates small distance changes.
    def score(item: Detection):
        x, y, z = item.position
        distance = math.sqrt(x * x + y * y + z * z)
        return item.confidence * 2.0 - distance

    return max(candidates, key=score)
