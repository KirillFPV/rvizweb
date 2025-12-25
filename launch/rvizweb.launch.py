from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Declare launch arguments
    websocket_port_arg = DeclareLaunchArgument(
        'websocket_port',
        default_value='9090',
        description='Port for the websocket bridge'
    )
    
    packages_port_arg = DeclareLaunchArgument(
        'packages_port',
        default_value='8001',
        description='Port where site and other package resources will be served'
    )
    
    packages_path_arg = DeclareLaunchArgument(
        'packages_path',
        default_value='/',
        description='Path within each installed ROS package to serve'
    )
    
    tf_arg = DeclareLaunchArgument(
        'tf',
        default_value='true',
        description='Set to false to prevent republishing TF'
    )
    
    interactive_markers_arg = DeclareLaunchArgument(
        'interactive_markers',
        default_value='true',
        description='Set to false if you don\'t want to use interactive markers'
    )
    
    interactive_markers_target_frame_arg = DeclareLaunchArgument(
        'interactive_markers_target_frame',
        default_value='/base_link',
        description='Target frame for interactive markers'
    )
    
    interactive_markers_topic_arg = DeclareLaunchArgument(
        'interactive_markers_topic',
        default_value='/basic_controls',
        description='Topic for interactive markers'
    )
    
    depth_cloud_arg = DeclareLaunchArgument(
        'depth_cloud',
        default_value='false',
        description='Set to true if you want depth cloud support'
    )
    
    video_port_arg = DeclareLaunchArgument(
        'video_port',
        default_value='9999',
        description='Port for visualizing video streams'
    )
    
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/depth/image_raw',
        description='Depth image topic for depthcloud_encoder'
    )
    
    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/camera/rgb/image_raw',
        description='RGB image topic for depthcloud_encoder'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=[PathJoinSubstitution([FindPackageShare('rvizweb'), 'config', 'configuration.json'])],
        description='Configuration file path'
    )

    # Get launch configurations
    websocket_port = LaunchConfiguration('websocket_port')
    packages_port = LaunchConfiguration('packages_port')
    packages_path = LaunchConfiguration('packages_path')
    tf_enabled = LaunchConfiguration('tf')
    interactive_markers_enabled = LaunchConfiguration('interactive_markers')
    interactive_markers_target_frame = LaunchConfiguration('interactive_markers_target_frame')
    interactive_markers_topic = LaunchConfiguration('interactive_markers_topic')
    depth_cloud_enabled = LaunchConfiguration('depth_cloud')
    video_port = LaunchConfiguration('video_port')
    depth_topic = LaunchConfiguration('depth_topic')
    rgb_topic = LaunchConfiguration('rgb_topic')
    config_file = LaunchConfiguration('config_file')

    # Set global parameter for configuration
    set_config_param = SetParameter(
        name='/rvizweb/global_config',
        value=config_file
    )

    # Rosbridge server node (in ROS2, we typically run the websocket server directly)
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',  # This executable name might differ
        name='rosbridge_websocket',
        parameters=[
            {'port': websocket_port}
        ]
    )

    # Note: roswww doesn't exist in ROS2, so we're not including it
    # In ROS2, static web content is typically served differently
    # This may require a custom web server node or using external web server

    # TF2 web republisher node
    tf2_web_republisher_node = Node(
        package='tf2_web_republisher',
        executable='tf2_web_republisher',
        name='tf2_web_republisher',
        condition=IfCondition(tf_enabled),
        respawn=False,
        output='screen'
    )

    # Interactive marker proxy node
    interactive_marker_proxy_node = GroupAction(
        condition=IfCondition(interactive_markers_enabled),
        actions=[
            Node(
                package='interactive_marker_proxy',
                executable='proxy',  # Note: executable name might differ in ROS2
                name='interactive_marker_proxy',
                parameters=[
                    {'target_frame': interactive_markers_target_frame},
                    {'topic_ns': interactive_markers_topic}
                ]
            )
        ]
    )

    # Depth cloud components
    depth_cloud_group = GroupAction(
        condition=IfCondition(depth_cloud_enabled),
        actions=[
            Node(
                package='web_video_server',
                executable='web_video_server',
                name='web_video_server',
                parameters=[
                    {'type': 'vp8'},
                    {'port': video_port}
                ]
            ),
            Node(
                package='depthcloud_encoder',
                executable='depthcloud_encoder_node',  # Note: executable name might differ in ROS2
                name='depthcloud_encoder',
                parameters=[
                    {'depth': depth_topic},
                    {'rgb': rgb_topic}
                ]
            )
        ]
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(websocket_port_arg)
    ld.add_action(packages_port_arg)
    ld.add_action(packages_path_arg)
    ld.add_action(tf_arg)
    ld.add_action(interactive_markers_arg)
    ld.add_action(interactive_markers_target_frame_arg)
    ld.add_action(interactive_markers_topic_arg)
    ld.add_action(depth_cloud_arg)
    ld.add_action(video_port_arg)
    ld.add_action(depth_topic_arg)
    ld.add_action(rgb_topic_arg)
    ld.add_action(config_file_arg)

    # Add actions
    ld.add_action(set_config_param)
    ld.add_action(rosbridge_node)
    ld.add_action(tf2_web_republisher_node)
    ld.add_action(interactive_marker_proxy_node)
    ld.add_action(depth_cloud_group)

    return ld