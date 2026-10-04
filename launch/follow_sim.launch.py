"""
TurtleBot4(robot1, 스냅샷 카메라 사양) + 검은 RC카 타깃 가즈보 시뮬.
  ros2 launch ~/tb4_follow_sim/launch/follow_sim.launch.py [gui:=false] [x:= y:= yaw:=]
overlay(카메라 704x704) 를 쓰려면 run_sim.sh 로 실행 (AMENT_PREFIX_PATH 설정 포함).
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parent.parent
WORLD = 'follow_arena'
NS = 'robot1'


def generate_launch_description():
    tb4_gz = get_package_share_directory('turtlebot4_gz_bringup')
    tb4_desc = get_package_share_directory('turtlebot4_description')   # overlay 우선
    create_desc = get_package_share_directory('irobot_create_description')
    create_gz = get_package_share_directory('irobot_create_gz_bringup')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    args = [
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('world_file', default_value=WORLD,
                              description='worlds/ 안의 파일 이름(확장자 제외). 내부 world name 은 follow_arena 여야 함'),
        DeclareLaunchArgument('pub_qos', default_value='reliable', choices=['reliable', 'best_effort'],
                              description='어댑터 영상 발행 QoS (실기 best-effort 재현용)'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]
    env = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join([
            str(ROOT / 'worlds'),
            os.path.join(tb4_gz, 'worlds'), os.path.join(create_gz, 'worlds'),
            str(Path(tb4_desc).parent.resolve()), str(Path(create_desc).parent.resolve())])),
        SetEnvironmentVariable('GZ_GUI_PLUGIN_PATH', ':'.join([
            os.path.join(get_package_share_directory('turtlebot4_gz_gui_plugins'), 'lib'),
            os.path.join(get_package_share_directory('irobot_create_gz_plugins'), 'lib')])),
    ]
    gz_launch = PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gui_cfg = os.path.join(tb4_gz, 'gui', 'standard', 'gui.config')
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_launch]),
        launch_arguments=[('gz_args', [LaunchConfiguration('world_file'), f'.sdf -r -v 3 --gui-config {gui_cfg}'])],
        condition=IfCondition(LaunchConfiguration('gui')))
    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_launch]),
        launch_arguments=[('gz_args', [LaunchConfiguration('world_file'), '.sdf -r -s -v 3 --headless-rendering'])],
        condition=UnlessCondition(LaunchConfiguration('gui')))

    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
                        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen')

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(tb4_gz, 'launch', 'turtlebot4_spawn.launch.py')]),
        launch_arguments=[('namespace', NS), ('world', WORLD), ('model', 'standard'),
                          ('x', LaunchConfiguration('x')), ('y', LaunchConfiguration('y')),
                          ('yaw', LaunchConfiguration('yaw'))])

    # RC카: cmd_vel(ROS→gz), 모델 pose(gz→ROS, 실험 로그용 ground truth)
    car_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='rc_car_bridge', output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['/model/rc_car/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                   '/model/rc_car/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/model/dummy_box/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/model/pillar/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/model/pole/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/overhead/image@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/webcam/image@sensor_msgs/msg/Image[gz.msgs.Image'],
        remappings=[('/model/rc_car/cmd_vel', '/rc_car/cmd_vel'),
                    ('/model/rc_car/pose', '/rc_car/pose'),
                    ('/model/dummy_box/pose', '/dummy_box/pose'),
                    ('/model/pillar/pose', '/pillar/pose'),
                    ('/model/pole/pose', '/pole/pose'),
                    ('/overhead/image', '/sim/overhead'),
                    ('/webcam/image', '/sim/webcam')])

    # 실행권한(+x)에 의존하지 않도록 python3 로 직접 실행 (Windows 쪽 편집 시 +x 가 빠짐)
    lidar = ExecuteProcess(output='screen', cmd=[
        'python3', str(ROOT / 'scripts' / 'sim_rplidar.py'), '--ros-args',
        '-p', f"world:={ROOT / 'worlds' / f'{WORLD}.sdf'}", '-p', 'use_sim_time:=true'])

    adapter = ExecuteProcess(output='screen', cmd=[
        'python3', str(ROOT / 'scripts' / 'tb4_real_iface_adapter.py'), '--ros-args',
        '-r', '__node:=tb4_real_iface_adapter', '-p', f'ns:=/{NS}', '-p', 'use_sim_time:=true',
        '-p', ['pub_qos:=', LaunchConfiguration('pub_qos')]])

    return LaunchDescription(args + env + [gz_gui, gz_headless, clock_bridge, robot, car_bridge, lidar, adapter])
