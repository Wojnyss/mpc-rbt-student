import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('mpc_rbt_student')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'config.rviz')

    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
	parameters=[{'use_sim_time': True}]
    )
    
    scan_relay_node = Node(
       package='mpc_rbt_student',
       executable='scan_relay_node',
       name='scan_relay_node',
       output='screen',
	parameters=[{'use_sim_time': True}]
    )

    localization_node = Node(
        package='mpc_rbt_student',
        executable='localization_node',
        name='localization_node',
        output='screen',
 	parameters=[{'use_sim_time': True}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
	parameters=[{'use_sim_time': True}]
    )

    motion_control_node = Node(
        package='mpc_rbt_student',
        executable='motion_control_node',
        name='motion_control_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    planning_node = Node(
        package='mpc_rbt_student',
        executable='planning_node',
        name='planning_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    warehouse_manager = Node(
        package='mpc_rbt_solution',
        executable='warehouse_manager',
        name='warehouse_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    bt_server = Node(
        package='mpc_rbt_solution',
        executable='bt_server',
        name='bt_server',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            os.path.join(package_dir, 'config', 'bt_server.yaml')
        ]
    )

    return LaunchDescription([
        map_to_odom_tf,
        localization_node,
	    scan_relay_node,
        rviz_node,
        motion_control_node,
        planning_node,
        bt_server,
        warehouse_manager
    ])
