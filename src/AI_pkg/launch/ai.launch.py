"""ROS2 launch, deliberately independent from the existing ROS1 camera launch."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def flatten(mapping, prefix=''):
    values = {}
    for key, value in mapping.items():
        name = prefix + key
        if isinstance(value, dict):
            values.update(flatten(value, name + '.'))
        else:
            values[name] = value
    return values


def generate_launch_description():
    path = Path(get_package_share_directory('AI_pkg')) / 'config' / 'agent.yaml'
    config = flatten(yaml.safe_load(path.read_text()))
    # Preserve empty strings and the JSON string '{}' as ROS string parameters.
    parameters = {key: ParameterValue(value, value_type=str) if isinstance(value, str) else value
                  for key, value in config.items()}
    return LaunchDescription([
        DeclareLaunchArgument('llm_base_url', default_value=config['llm.base_url']),
        DeclareLaunchArgument('llm_model', default_value=config['llm.model']),
        DeclareLaunchArgument('audio_device', default_value=config['audio.device']),
        DeclareLaunchArgument('mcp_port', default_value='8765'),
        DeclareLaunchArgument('ready_topic', default_value='/desk_perception/ready'),
        Node(package='AI_pkg', executable='mcp_server_node', name='mcp_server_node',
             output='screen', parameters=[{
                 'port': ParameterValue(LaunchConfiguration('mcp_port'), value_type=int),
                 'ready_topic': ParameterValue(LaunchConfiguration('ready_topic'), value_type=str),
             }]),
        Node(package='AI_pkg', executable='llm_agent_node', name='llm_agent_node',
             output='screen', parameters=[parameters, {
                 'llm.base_url': ParameterValue(LaunchConfiguration('llm_base_url'), value_type=str),
                 'llm.model': ParameterValue(LaunchConfiguration('llm_model'), value_type=str),
                 'audio.device': ParameterValue(LaunchConfiguration('audio_device'), value_type=str),
                 'mcp_url': ParameterValue(
                     ['http://127.0.0.1:', LaunchConfiguration('mcp_port'), '/mcp'], value_type=str),
             }]),
    ])
