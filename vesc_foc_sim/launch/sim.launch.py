import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('vesc_foc_sim')
    default_params = os.path.join(pkg_share, 'config', 'motor_params.yaml')

    params_arg = DeclareLaunchArgument(
        'params',
        default_value=default_params,
        description='Path to motor_params.yaml'
    )
    params_file = LaunchConfiguration('params')

    pmsm_plant = Node(
        package='vesc_foc_sim',
        executable='pmsm_plant',
        name='pmsm_plant',
        output='screen',
        parameters=[params_file],
    )

    vesc_controller = Node(
        package='vesc_foc_sim',
        executable='vesc_controller',
        name='vesc_controller',
        output='screen',
        parameters=[params_file],
    )

    ecu_sim = Node(
        package='vesc_foc_sim',
        executable='ecu_sim',
        name='ecu_sim',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        params_arg,
        pmsm_plant,
        vesc_controller,
        ecu_sim,
    ])
