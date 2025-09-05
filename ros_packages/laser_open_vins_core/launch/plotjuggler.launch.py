import launch

from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = launch.LaunchDescription()

    # #{ refactor script

    refactor_plotjuggler_config_cmd = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([
                FindPackageShare('laser_open_vins_core'),
                'scripts',
                'refactor_plotjuggler_config.sh'
            ])
        ],
        output='screen'
    )

    ld.add_action(refactor_plotjuggler_config_cmd)

    # #}

    # #{ plotjuggler node

    plotjuggler_node = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler',
        output='screen',
        arguments=['-l /tmp/imu_layout.xml'],
        prefix=["bash -c 'sleep 2; $0 $@'"])

    ld.add_action(plotjuggler_node)

    # #}

    return ld
