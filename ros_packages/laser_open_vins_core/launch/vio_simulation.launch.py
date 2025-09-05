import os

import launch

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = launch.LaunchDescription()

    # #{ uav_name

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', 'uav1'),
        description='Top-level namespace.'
    ))

    # #}

    # #{ standalone

    standalone = LaunchConfiguration('standalone')

    ld.add_action(DeclareLaunchArgument(
        'standalone',
        default_value='false',
        description='Whether to start as a container or load into an existing container.'
    ))

    # #}

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=PythonExpression(['"', os.getenv('REAL_UAV', 'true'), '" == "false"']),
        description='Whether use the simulation time.'
    ))

    # #}

    # #{ log_level

    ld.add_action(DeclareLaunchArgument(
        name='log_level',
        default_value='info',
        description='Log level.'
    ))

    # #}

    # #{ ov_msckf_config

    ov_msckf_config = LaunchConfiguration('ov_msckf_config')

    ld.add_action(DeclareLaunchArgument(
        'ov_msckf_config',
        default_value='bluefox_simulation.yaml',
        description='Name of the OpenVINS MSCKF configuration file.'
    ))

    # #}

    # #{ openvins vio container

    container_name = ['/', uav_name, '/ov_vio_container']

    open_vins_core_container = ComposableNodeContainer(
        namespace=uav_name,
        name='ov_vio_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            {'use_intra_process_comms': True},
            {'thread_num': os.cpu_count()},
            {'use_sim_time': use_sim_time},
        ],
        condition=UnlessCondition(standalone)
    )

    ld.add_action(open_vins_core_container)

    # #}

    # #{ openvins msckf launcher

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('ov_msckf'), '/launch/run_subscriber_msckf.launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'ov_msckf_config': PathJoinSubstitution([
                    FindPackageShare('laser_open_vins_core'),
                    'config',
                    ov_msckf_config
                ]),
                'standalone': standalone,
                'container_name': container_name,
            }.items()
        )
    )

    # #}

    return ld
