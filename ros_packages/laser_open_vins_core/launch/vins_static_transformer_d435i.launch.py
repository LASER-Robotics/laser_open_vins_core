from launch import LaunchContext, LaunchDescription

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def launch_setup(context: LaunchContext):
    # Initialize arguments
    uav_name = LaunchConfiguration('uav_name')

    uav_name_env = EnvironmentVariable('UAV_NAME').perform(context)

    uav_name_str = uav_name_env if uav_name_env else uav_name.perform(context)

    fcu_frame = uav_name_str + '/fcu'
    fcu_frame_slashless = 'fcu_' + uav_name_str

    open_vins_imu_frame = uav_name_str + '/ov_imu'
    open_vins_imu_frame_slashless = uav_name_str + '_ov_imu'

    fcu_to_open_vins_imu_tf_static_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=TextSubstitution(text=fcu_frame_slashless + '_to_' + open_vins_imu_frame_slashless),
        namespace=TextSubstitution(text=uav_name_str),
        output='screen',
        arguments=['--x', '0.0',
                   '--y', '0.0',
                   '--z', '0.0',
                   '--yaw', '-1.508',
                   '--pitch', '0.0',
                   '--roll', '-1.508',
                   '--frame-id', fcu_frame,
                   '--child-frame-id', open_vins_imu_frame]) # Z250 frame

    return [fcu_to_open_vins_imu_tf_static_publisher_node]


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'uav_name',
            default_value=EnvironmentVariable('UAV_NAME')))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
