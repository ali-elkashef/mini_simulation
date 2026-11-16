import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('my_package')
    robot_description_path = os.path.join(package_dir, 'resource', 'rosbot.urdf')
    
    # --- Static transforms ---

    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt')
    )
    my_robot_driver = WebotsController(
        robot_name='rosbot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(robot_description_path).read()}]
    )



    # static_tf_laser = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_laser',
    #     arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
    # )

    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera',
        arguments=['-0.027', '0', '0.165', '0', '0', '0', 'base_link', 'camera_depth']
    )

    obstacle_avoider = Node(
        package='my_package',
        executable='obstacle_avoider',
    )

    local_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        parameters=[os.path.join(package_dir, 'config', 'nav2_costmap.yaml')]
    )

    lidar_costmap = Node(
        package='my_package',
        executable='lidar_costmap_node',
        name='lidar_costmap_node',
        output='screen',
    )

    camera_costmap = Node(
        package='my_package',
        executable='camera_costmap_node',
        name='camera_costmap_node',
        output='screen',
    )

    merger = Node(
        package='my_package',
        executable='merger_node',
        name='merger_node',
        output='screen',
    )

    return LaunchDescription([
        static_tf_map_odom,
        # static_tf_laser,
        static_tf_camera,
        webots,
        my_robot_driver,
        obstacle_avoider,
        local_costmap,
        lidar_costmap,
        camera_costmap,
        merger,
        robot_state_publisher,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
        
    ])