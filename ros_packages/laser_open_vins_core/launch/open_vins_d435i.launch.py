from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument

from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'uav_name',
            default_value=EnvironmentVariable('UAV_NAME'),
            description='Name of the folder containing the file with the parameters.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'params_folder',
            default_value='rs_243522071667',
            description='Name of the folder containing the file with the parameters.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'verbosity',
            default_value='DEBUG',
            description='Verbosity level.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_stereo',
            default_value='true',
            description='If you are using stereo cameras or not.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'max_cameras',
            default_value='2',
            description='Maximum number of cameras.'))

    # Initialize arguments
    uav_name = LaunchConfiguration('uav_name')
    params_folder = LaunchConfiguration('params_folder')
    verbosity = LaunchConfiguration('verbosity')
    use_stereo = LaunchConfiguration('use_stereo')
    max_cameras = LaunchConfiguration('max_cameras')

    # Declare nodes
    msckf_node = Node(
        package='ov_msckf',
        executable='run_subscribe_msckf',
        namespace=uav_name,
        output='screen',
        parameters=[{'uav_name': uav_name,
                     'verbosity': verbosity,
                     'use_stereo': use_stereo,
                     'max_cameras': max_cameras,
                     'config_path': PathJoinSubstitution([FindPackageShare('laser_open_vins_core'),
                                                          'params', params_folder, 'estimator_config.yaml'])}])

    return LaunchDescription(declared_arguments + [msckf_node])
