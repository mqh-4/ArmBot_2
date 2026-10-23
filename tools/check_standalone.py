#!/usr/bin/env python3
"""Check ArmBot's local deployment inputs without starting ROS or hardware.

This checks the supplied Compose configuration, not an existing container or
image. Run the container checks in docs/STANDALONE_RUN_CN.md for that layer.
"""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
CONTAINER_WS = '/home/ubuntu/share/tmp/ArmBot'
CORE_PACKAGES = (
    'desk_cleaning', 'AI_pkg', 'orbbec_camera', 'ros_robot_controller',
    'hiwonder_servo_controllers', 'hiwonder_servo_msgs', 'hiwonder_servo_driver',
    'armpi_fpv_kinematics', 'armpi_fpv_description', 'armpi_fpv_common',
)
OLD_PATH = re.compile(r'(?:/home/|\$(?:HOME|\{HOME\})/)[^\s\'\"]*ArmPi-FPV')
FIND_RESOURCE = re.compile(r'\$\(find ([^)]+)\)([^\s\'\"<>]*)')


def inside_project(path):
    try:
        path.resolve().relative_to(ROOT)
        return True
    except ValueError:
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--with-ai', action='store_true',
                        help='also require the optional local Whisper model')
    args = parser.parse_args()
    errors = []
    print('PROJECT:', ROOT)

    def check(ok, message):
        print(('PASS: ' if ok else 'FAIL: ') + message)
        if not ok:
            errors.append(message)

    packages = {}
    for path in sorted((ROOT / 'src').glob('*/package.xml')):
        packages[ET.parse(path).findtext('name')] = path.parent
    check(all(name in packages for name in CORE_PACKAGES),
          'main ROS packages are present in src (%d local packages)' % len(packages))

    missing = []
    pending = [path for name in ('desk_cleaning', 'AI_pkg') if name in packages
               for path in packages[name].rglob('*.launch')]
    visited = set()
    while pending:
        path = pending.pop()
        if path in visited:
            continue
        visited.add(path)
        ET.parse(path)
        for pkg, suffix in FIND_RESOURCE.findall(path.read_text()):
            if pkg in packages and '$' not in suffix:
                resource = packages[pkg] / suffix.lstrip('/')
                if not resource.exists():
                    missing.append('%s: %s%s' % (path.relative_to(ROOT), pkg, suffix))
                elif resource.suffix == '.launch':
                    pending.append(resource)
    check(not missing, 'desk_cleaning/AI launch resource references resolve' +
          (': ' + '; '.join(missing) if missing else ''))

    old_refs = []
    sources = [ROOT / '.zshrc', ROOT / '.robotrc', ROOT / 'command']
    sources.extend(path for path in (ROOT / 'src').rglob('*')
                   if path.suffix in {'.py', '.sh', '.launch', '.yaml', '.xml'}
                   and path.is_file() and '__pycache__' not in path.parts)
    for path in sources:
        if OLD_PATH.search(path.read_text(encoding='utf-8')):
            old_refs.append(str(path.relative_to(ROOT)))
    check(not old_refs, 'source/configuration has no old workspace path references' +
          (': ' + ', '.join(old_refs) if old_refs else ''))

    bad_links = []
    catkin_top = ROOT / 'src/CMakeLists.txt'
    for path in (ROOT / 'src').rglob('*'):
        if path.is_symlink():
            # Provided by ros:noetic inside the image, absent on this host.
            if (path == catkin_top and str(path.readlink()) ==
                    '/opt/ros/noetic/share/catkin/cmake/toplevel.cmake'):
                continue
            if not path.exists() or not inside_project(path):
                bad_links.append(str(path.relative_to(ROOT)))
    check(not bad_links, 'source symlinks stay within ArmBot (except ROS catkin)' +
          (': ' + ', '.join(bad_links) if bad_links else ''))

    required = (
        'Dockerfile', 'docker-compose.yml', 'requirements.txt',
        'ros1-latest-archive-keyring.gpg',
        'src/AI_pkg/scripts/install_ai_runtime.sh', 'src/AI_pkg/requirements-noetic.txt',
        'config/camera_info/astra_color.yaml', 'config/camera_info/astra_ir.yaml',
        'models/yolov8s.onnx', 'output/eye_to_hand_result.json',
    )
    if args.with_ai:
        required += (
            'models/whisper-small/model.bin', 'models/whisper-small/config.json',
            'models/whisper-small/tokenizer.json', 'models/whisper-small/vocabulary.txt',
        )
    missing = [name for name in required if not (ROOT / name).is_file()
               or (ROOT / name).stat().st_size == 0
               or not inside_project(ROOT / name)]
    check(not missing, 'build inputs and default vision/calibration files exist locally' +
          (': ' + ', '.join(missing) if missing else ''))
    if not args.with_ai:
        print('NOTE: optional Whisper files are checked with --with-ai.')

    ignored = set((ROOT / '.dockerignore').read_text().splitlines())
    check({'build/', 'devel/'} <= ignored,
          'Docker excludes the host build/devel artifacts')
    dockerfile = (ROOT / 'Dockerfile').read_text()
    check('ROS_WS=' + CONTAINER_WS in dockerfile and 'catkin_make' in dockerfile,
          'Dockerfile builds a workspace under the ArmBot container path')

    try:
        result = subprocess.run(
            ['docker', 'compose', 'config', '--format', 'json'], cwd=ROOT,
            text=True, capture_output=True, check=True, timeout=30,
        )
        service = json.loads(result.stdout)['services']['armpi_fpv']
        check(inside_project(Path(service['build']['context'])),
              'Compose build context resolves inside ArmBot')
        check(service['working_dir'] == CONTAINER_WS,
              'Compose working directory uses ArmBot')
        binds = [item for item in service['volumes'] if item['type'] == 'bind']
        bad_mounts = [item['source'] for item in binds
                      if item['source'] not in {'/dev', '/tmp/.X11-unix'}
                      and not inside_project(Path(item['source']))]
        check(not bad_mounts, 'Compose project mounts resolve inside ArmBot' +
              (': ' + ', '.join(bad_mounts) if bad_mounts else ''))
        mounts = {item['target']: Path(item['source']).resolve() for item in binds}
        check(all(mounts.get(CONTAINER_WS + '/' + name) == ROOT / name
                  for name in ('src', 'config', 'models', 'output', 'tmp')),
              'Compose mounts the expected source and data directories')
    except (OSError, subprocess.SubprocessError, ValueError, KeyError) as error:
        check(False, 'Compose configuration check: %s' % error)

    marker = ROOT / 'devel/.catkin'
    if marker.exists() and 'ArmPi-FPV' in marker.read_text():
        print('NOTE: host devel/ uses the old path; use the image build, not host devel/setup.*.')
    print('NOTE: Docker daemon, existing images/containers, ROS and hardware are not tested here.')
    print('RESULT:', 'FAIL (%d checks)' % len(errors) if errors else 'PASS (static inputs only)')
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
