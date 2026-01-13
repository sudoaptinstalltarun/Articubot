import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('bot')

    urdf = os.path.join(pkg, 'description', 'my_bot.urdf.xacro')
    world = os.path.join(pkg, 'worlds', 'empty.world')

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': os.popen(f"xacro {urdf}").read()
        }],
        output='screen'
    )

    # Start Gazebo Sim
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen'
    )

    # Spawn robot
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'bot', '-topic', 'robot_description'],
        output='screen'
    )

    # 🔥 ROS ↔ Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn,
        bridge
    ])
