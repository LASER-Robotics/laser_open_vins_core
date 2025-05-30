from launch import LaunchDescription

from launch.substitutions import PathJoinSubstitution

from launch.actions import ExecuteProcess

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare commands
    refactor_rviz_config_cmd = ExecuteProcess(
        cmd=[PathJoinSubstitution([FindPackageShare('laser_open_vins_core'),
                                  'scripts', 'refactor_rviz_config.sh'])],
        output='screen')

    # Declare nodes
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d /tmp/default.rviz'],
        prefix=["bash -c 'sleep 2; $0 $@'"])

    return LaunchDescription([refactor_rviz_config_cmd, rviz_node])
