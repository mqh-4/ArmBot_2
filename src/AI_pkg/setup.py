#!/usr/bin/env python3
import os
from glob import glob
from setuptools import setup
from setuptools.command.build_py import build_py


class RuntimeBuild(build_py):
    """Package runtime modules without copying user-supplied credentialed examples."""
    def find_package_modules(self, package, package_dir):
        runtime = {'__init__', 'agent', 'nodes', 'ros_runtime', 'speech', 'tools',
                   'motion_teach'}
        return [entry for entry in super().find_package_modules(package, package_dir)
                if entry[1] in runtime]

if os.environ.get('ROS_VERSION') == '1':
    from catkin_pkg.python_setup import generate_distutils_setup
    setup(cmdclass={'build_py': RuntimeBuild}, **generate_distutils_setup(
        packages=['ai_pkg'], package_dir={'': 'src'}))
else:
    setup(
        name='AI_pkg', version='0.1.0', packages=['ai_pkg'], package_dir={'': 'src'},
        data_files=[
            ('share/ament_index/resource_index/packages', ['resource/AI_pkg']),
            ('share/AI_pkg', ['package.xml', 'README.md', 'requirements.txt']),
            ('share/AI_pkg/launch', glob('launch/*.launch.py')),
            ('share/AI_pkg/config', glob('config/*.yaml')),
        ],
        install_requires=['setuptools'], zip_safe=False,
        cmdclass={'build_py': RuntimeBuild},
        maintainer='ArmBot Maintainer', maintainer_email='maintainer@example.com',
        description='FastMCP tools and a voice/Whisper/LLM ROS agent',
        license='BSD-3-Clause',
        entry_points={'console_scripts': [
            'mcp_server_node = ai_pkg.nodes:mcp_main',
            'llm_agent_node = ai_pkg.nodes:agent_main',
        ]},
    )
