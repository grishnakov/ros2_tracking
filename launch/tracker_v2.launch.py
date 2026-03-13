import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from launch_ros.actions import Node


def _load_yaml(path: str) -> dict:
    with open(path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle) or {}


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('tracker_v2')
    node_config_path = os.path.join(pkg_share, 'config', 'node_config.yaml')
    tracker_params = os.path.join(pkg_share, 'config', 'tracker_params.yaml')
    vesc_params = os.path.join(pkg_share, 'config', 'vesc_params.yaml')
    nav_params = os.path.join(pkg_share, 'config', 'nav_params.yaml')

    cfg = _load_yaml(node_config_path)
    nav_cfg = _load_yaml(nav_params)
    gps_bridge_params = nav_cfg.get('gps_runner_bridge_node', {}).get('ros__parameters', {})
    gps_runner_command = str(
        os.environ.get(
            'TRACKER_V2_GPS_RUNNER_COMMAND',
            gps_bridge_params.get('runner_command', ''),
        )
    ).strip()
    gps_udp_params = nav_cfg.get('gps_udp_bridge_node', {}).get('ros__parameters', {})
    gps_udp_port = int(gps_udp_params.get('listen_port', 0))
    camera_enabled = cfg.get('camera_enabled', True)
    pid_enabled = cfg.get('pid_enabled', True)
    vesc_enabled = cfg.get('vesc_enabled', True)
    gps_enabled = cfg.get('gps_enabled', True)
    search_enabled = cfg.get('search_enabled', True)
    mode_manager_enabled = cfg.get('mode_manager_enabled', True)
    cmd_mux_enabled = cfg.get('cmd_mux_enabled', True)

    nodes = []
    if camera_enabled:
        nodes.append(
            Node(
                package='tracker_v2',
                executable='tracker_camera_node',
                name='tracker_camera_node',
                parameters=[tracker_params],
                output='screen',
                additional_env={'DISPLAY': os.environ.get('DISPLAY', ':0')},
            )
        )
    else:
        nodes.append(LogInfo(msg='[node_config] camera_enabled=false - skipping tracker camera node'))

    if pid_enabled:
        nodes.append(
            Node(
                package='tracker_v2',
                executable='tracker_pid_node',
                name='tracker_pid_node',
                parameters=[tracker_params],
                output='screen',
            )
        )
    else:
        nodes.append(LogInfo(msg='[node_config] pid_enabled=false - skipping PID node'))

    if mode_manager_enabled:
        nodes.append(
            Node(
                package='tracker_v2',
                executable='control_mode_node',
                name='control_mode_node',
                parameters=[nav_params],
                output='screen',
            )
        )
    else:
        nodes.append(LogInfo(msg='[node_config] mode_manager_enabled=false - skipping mode manager'))

    if gps_enabled:
        nodes.append(
            Node(
                package='tracker_v2',
                executable='gps_waypoint_node',
                name='gps_waypoint_node',
                parameters=[nav_params],
                output='screen',
            )
        )
    else:
        nodes.append(LogInfo(msg='[node_config] gps_enabled=false - skipping GPS waypoint node'))

    if gps_runner_command:
        nodes.append(
            Node(
                package='tracker_v2',
                executable='gps_runner_bridge_node',
                name='gps_runner_bridge_node',
                parameters=[nav_params],
                output='screen',
            )
        )
    else:
        nodes.append(
            LogInfo(
                msg='[nav_params] gps runner command empty - skipping GPS runner bridge '
                '(set gps_runner_bridge_node.runner_command or TRACKER_V2_GPS_RUNNER_COMMAND)'
            )
        )

    if gps_udp_port > 0:
        nodes.append(
            Node(
                package='tracker_v2',
                executable='gps_udp_bridge_node',
                name='gps_udp_bridge_node',
                parameters=[nav_params],
                output='screen',
            )
        )
    else:
        nodes.append(LogInfo(msg='[nav_params] gps UDP bridge port <= 0 - skipping GPS UDP bridge'))

    if search_enabled:
        nodes.append(
            Node(
                package='tracker_v2',
                executable='search_nav_node',
                name='search_nav_node',
                parameters=[nav_params],
                output='screen',
            )
        )
    else:
        nodes.append(LogInfo(msg='[node_config] search_enabled=false - skipping search navigation node'))

    if cmd_mux_enabled:
        nodes.append(
            Node(
                package='tracker_v2',
                executable='cmd_vel_mux_node',
                name='cmd_vel_mux_node',
                parameters=[nav_params],
                output='screen',
            )
        )
    else:
        nodes.append(LogInfo(msg='[node_config] cmd_mux_enabled=false - skipping cmd_vel mux node'))

    if vesc_enabled:
        nodes.append(
            Node(
                package='tracker_v2',
                executable='vesc_twist_node',
                name='vesc_twist_node',
                parameters=[vesc_params],
                output='screen',
            )
        )
    else:
        nodes.append(LogInfo(msg='[node_config] vesc_enabled=false - skipping VESC command node'))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
