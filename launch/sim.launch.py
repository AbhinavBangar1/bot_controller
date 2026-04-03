import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('bot_controls')
    urdf_path = os.path.join(pkg_share, 'urdf' , 'robot.urdf.xacro')
    controller_path = os.path.join (pkg_share , 'config' , 'controllers.yaml')
    ekf_path = os.path.join(pkg_share , 'config' , 'ekf.yaml')
    launch_file_dir = os.path.join(pkg_share , 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    robot_description = xacro.process_file(urdf_path).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros , 'launch' , 'gazebo.launch.py')
        )
    )

    robot_state_publisher_node = Node(
            package = 'robot_state_publisher',
            executable = 'robot_state_publisher',
            parameters = [
                {"robot_description": robot_description} 
            ]
        )

    spawn_robot = Node(
            package = 'gazebo_ros',
            executable = 'spawn_entity.py',
            arguments = ['-topic', 'robot_description', '-entity', 'diff_bot']
        )

    controller_loader = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                parameters=[controller_path]
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['left_wheel_velocity_controller'],
                parameters=[controller_path]
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['right_wheel_velocity_controller'],
                parameters=[controller_path]
            )
        ]
    )

    robot_localization_node = Node(
        package = 'robot_localization' ,
        executable = 'ekf_node' ,
        name = 'ekf_filter_node' ,
        parameters = [ekf_path]
    )

    bot_controller_node = Node(
        package='bot_controls',
        executable='controls.py',
        name='bot_controller',
        output='screen'
    )

    return LaunchDescription([
        gazebo ,
        robot_state_publisher_node ,
        spawn_robot ,
        controller_loader ,
        robot_localization_node ,
        bot_controller_node
    ])