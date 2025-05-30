from launch import LaunchContext, LaunchDescription

from launch.actions import DeclareLaunchArgument, OpaqueFunction

from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context: LaunchContext):
    # Initialize arguments
    uav_name = LaunchConfiguration('uav_name')
    camera_name = LaunchConfiguration('camera_name')
    camera_params_file = LaunchConfiguration('camera_params_file')

    fcu_frame = uav_name.perform(context) + '/fcu'
    fcu_frame_slashless = 'fcu_' + uav_name.perform(context)

    realsense_frame = uav_name.perform(context) + '/' + camera_name.perform(context) + '/link'
    realsense_frame_slashless = uav_name.perform(context) + '_' + camera_name.perform(context) + '_link'

    # Declare nodes
    realsense_camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name=camera_name,
        namespace=uav_name,
        output='screen',
        parameters=[ParameterFile(camera_params_file, allow_substs=True)])

    # TODO(anyone): how can I get the pose of the camera for each frame?
    fcu_to_realsense_tf_static_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=TextSubstitution(text=fcu_frame_slashless + '_to_' + realsense_frame_slashless),
        namespace=uav_name,
        output='screen',
        arguments=['--x', '0.07',
                   '--y', '0.0',
                   '--z', '0.09',
                   '--yaw', '0.0',
                   '--pitch', '0.0',
                   '--roll', '0.0',
                   '--frame-id', fcu_frame,
                   '--child-frame-id', realsense_frame]) # NOTE: Z250 frame

    return [realsense_camera_node, fcu_to_realsense_tf_static_publisher_node]


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'uav_name',
            default_value=EnvironmentVariable('UAV_NAME'),
            description='Top-level namespace.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_name',
            default_value='rgbd',
            description='Camera name.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_params_file',
            default_value=PathJoinSubstitution([FindPackageShare('laser_open_vins_core'),
                                                'params', 'rs_d435i.yaml']),
            description='Full path to the file with the camera parameters.'))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
