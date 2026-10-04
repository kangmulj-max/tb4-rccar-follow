"""One capture round: scan_mapper + patrol, in a robot namespace, on a fixed reference map.

ros2 launch layout_delta round.launch.py namespace:=/robot1 ref_map:=/path/map.yaml \
    waypoints:=/path/waypoints.yaml out_dir:=/path/site01/round01 initial_pose:='[x, y, yaw_deg]'
Requires: localization (AMCL) + Nav2 running in the same namespace (e.g. turtlebot4_navigation
localization.launch.py and nav2.launch.py with the same map).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node


def generate_launch_description():
    args = [DeclareLaunchArgument(n, default_value=d) for n, d in (
        ('namespace', '/robot1'), ('ref_map', ''), ('waypoints', ''), ('out_dir', 'round'),
        ('initial_pose', '[0.0]'), ('use_sim_time', 'false'), ('dwell_s', '4.0'))]
    remap = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    common = {'use_sim_time': LC('use_sim_time')}
    mapper = Node(package='layout_delta', executable='scan_mapper', namespace=LC('namespace'), output='screen',
                  parameters=[common, {'ref_map': LC('ref_map'), 'out_dir': LC('out_dir')}], remappings=remap)
    patrol = Node(package='layout_delta', executable='patrol', namespace=LC('namespace'), output='screen',
                  parameters=[common, {'waypoints': LC('waypoints'), 'out_dir': LC('out_dir'),
                                       'initial_pose': LC('initial_pose'), 'dwell_s': LC('dwell_s')}],
                  remappings=remap)
    return LaunchDescription(args + [mapper, patrol])
