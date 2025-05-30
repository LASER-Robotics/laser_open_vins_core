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
            'namespace',
            default_value=EnvironmentVariable('UAV_NAME'),
            description='Top-level namespace.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'open_vins_params_folder',
            default_value='rs_243522071667',
            description='Name of the folder containing the camera OpenVINS files.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'verbosity',
            default_value='DEBUG',
            description='OpenVINS verbosity level.',
            choices=['ALL', 'DEBUG', 'INFO', 'WARNING', 'ERROR', 'SILENT']))

    declared_arguments.append(
        DeclareLaunchArgument(
            'topic_imu',
            default_value=['/', EnvironmentVariable('UAV_NAME'), '/rgbd/imu'],
            description='Name of the IMU topic.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'topic_camera0',
            default_value=['/', EnvironmentVariable('UAV_NAME'), '/rgbd/infra1/image_raw'],
            description='Name of the camera 0 topic.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'topic_camera1',
            default_value=['/', EnvironmentVariable('UAV_NAME'), '/rgbd/infra2/image_raw'],
            description='Name of the camera 1 topic.'))

    # Initialize arguments
    namespace = LaunchConfiguration('namespace')
    open_vins_params_folder = LaunchConfiguration('open_vins_params_folder')
    verbosity = LaunchConfiguration('verbosity')
    topic_imu = LaunchConfiguration('topic_imu')
    topic_camera0 = LaunchConfiguration('topic_camera0')
    topic_camera1 = LaunchConfiguration('topic_camera1')

    # Declare nodes
    msckf_node = Node(
        package='ov_msckf',
        executable='run_subscribe_msckf',
        namespace=namespace,
        name='ov_msckf',
        output='screen',
        parameters=[{'namespace': namespace,
                     'verbosity': verbosity,
                     'topic_imu': topic_imu,
                     'topic_camera0': topic_camera0,
                     'topic_camera1': topic_camera1,
                     'config_path': PathJoinSubstitution([FindPackageShare('laser_open_vins_core'),
                                                          'params', open_vins_params_folder, 'estimator_config.yaml'])}])

    return LaunchDescription(declared_arguments + [msckf_node])
