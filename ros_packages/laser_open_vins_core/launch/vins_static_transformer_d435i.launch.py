from launch import LaunchContext, LaunchDescription

from launch.actions import OpaqueFunction
from launch.substitutions import EnvironmentVariable, TextSubstitution

from launch_ros.actions import Node


def launch_setup(context: LaunchContext):
    uav_name = EnvironmentVariable('UAV_NAME').perform(context)

    fcu_frame = uav_name + '/fcu'
    fcu_frame_slashless = 'fcu_' + uav_name

    open_vins_imu_frame = uav_name + '/ov_imu'
    open_vins_imu_frame_slashless = uav_name + '_ov_imu'

    # Declare nodes
    fcu_to_open_vins_imu_tf_static_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=TextSubstitution(text=fcu_frame_slashless + '_to_' + open_vins_imu_frame_slashless),
        namespace=TextSubstitution(text=uav_name),
        output='screen',
        arguments=['--x', '0.0',
                   '--y', '0.0',
                   '--z', '0.0',
                   '--yaw', '-1.570796',
                   '--pitch', '0.0',
                   '--roll', '-1.570796',
                   '--frame-id', fcu_frame,
                   '--child-frame-id', open_vins_imu_frame]) # Z250 frame

    return [fcu_to_open_vins_imu_tf_static_publisher_node]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
