from launch import LaunchContext, LaunchDescription

from launch.actions import DeclareLaunchArgument, OpaqueFunction

from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context: LaunchContext):
    # Initialize arguments
    uav_name = LaunchConfiguration('uav_name')
    params_file = LaunchConfiguration('params_file') # TODO(thulioguilherme): check if the namespace matches with the uav name
    camera_name = LaunchConfiguration('camera_name')

    uav_name_env = EnvironmentVariable('UAV_NAME').perform(context)

    uav_name_str = uav_name_env if uav_name_env else uav_name.perform(context)
    camera_name_str = camera_name.perform(context)

    fcu_frame = uav_name_str + '/fcu'
    fcu_frame_slashless = 'fcu_' + uav_name_str

    realsense_frame = uav_name_str + '/' + camera_name_str + '/link'
    realsense_frame_slashless = uav_name_str + '_rgbd_link'

    # Declare nodes
    realsense_camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name=camera_name,
        namespace=TextSubstitution(text=uav_name_str),
        output='screen',
        parameters=[params_file])
    
    # TODO(thulioguilherme): how can I get the pose of the camera for each UAV frame?
    fcu_to_realsense_tf_static_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=TextSubstitution(text=fcu_frame_slashless + '_to_' + realsense_frame_slashless),
        namespace=TextSubstitution(text=uav_name_str),
        output='screen',
        arguments=['--x', '0.07',
                   '--y', '0.0',
                   '--z', '0.09',
                   '--yaw', '0.0',
                   '--pitch', '0.0',
                   '--roll', '0.0',
                   '--frame-id', fcu_frame,
                   '--child-frame-id', realsense_frame]) # Z250 frame

    return [realsense_camera_node, fcu_to_realsense_tf_static_publisher_node]


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'uav_name',
            default_value='uav1',
            description='Top-level namespace.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_name',
            default_value='rgbd',
            description='Camera name.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([FindPackageShare('laser_open_vins_core'),
                                                'params', 'rs_d435i.yaml']),
            description='Full path to the file with all the parameters.'))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
