# RVizWeb - RViz, but on your browser (ROS 2 Humble Hawksbill Version)

RVizWeb provides a convenient way of building and launching a web application
with features similar to [RViz](https://github.com/ros-visualization/rviz).

This project makes use of the following:

* @jstnhuang's [ros-rviz](https://github.com/jstnhuang/ros-rviz) web component
* @RobotWebTools's [rosbridge_server](https://github.com/RobotWebTools/rosbridge_suite)
  and [tf2_web_republisher](https://github.com/RobotWebTools/tf2_web_republisher)

## Quickstart

1. Create a directory for your ROS 2 workspace:

        mkdir -p ~/ws/src
        cd ~/ws/src
        git clone https://github.com/osrf/rvizweb/

1. You will need the LTS version of Node.js:

        curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
        sudo apt-get install -y nodejs

1. Install ROS 2 Humble dependencies:

        cd ~/ws
        rosdep install --from-paths src --ignore-src -r -y

1. Build and install your workspace; this will run `bower` and `polymer-cli`
   to generate and install the site:

        cd ~/ws
        colcon build --packages-select rvizweb

1. Source your workspace:

        . ~/ws/install/setup.bash

1. Launch RVizWeb:

        ros2 launch rvizweb rvizweb.launch.py

1. Open the site on the browser

    http://localhost:8001/rvizweb/www/index.html  # Note: URL may vary based on how web content is served

1. Let's try an example display to check everything is working. On the UI, click on the `+` and choose "Markers".

1. Now open a new terminal and publish the following marker:

        ros2 topic pub /visualization_marker visualization_msgs/msg/Marker '{header: {frame_id: "base_link"}, id: 1, type: 1, action: 0, pose: {position: {x: 0., y: 0.2, z: 0.}, orientation: {x: 0.3, y: 0.2, z: 0.52, w: 0.85}}, scale: {x: 0.2, y: 0.3, z: 0.1}, color: {r: 1., g: 0., b: 1., a: 0.3}, lifetime: {sec: 5, nanosec: 0}}'

1. You should see a pink box show up on the site.

## Viewing URDF

In ROS 2, web content serving may work differently than in ROS 1.
The web interface will connect to ROS 2 topics via rosbridge.

Let's try an example using a simulated robot:

1. Launch your robot simulation (e.g., using Gazebo)

1. Launch RVizWeb:

        ros2 launch rvizweb rvizweb.launch.py

1. Open the site on the browser

1. On the UI, click on the `+` and choose "Robot model".

## Launch Parameters

The RVizWeb launch file accepts the following parameters:

* `websocket_port` - Port for the websocket bridge (default: 9090)
* `packages_port` - Port where site and other package resources will be served (default: 8001)
* `tf` - Set to false to prevent republishing TF (default: true)
* `interactive_markers` - Set to false if you don't want to use interactive markers (default: true)
* `interactive_markers_target_frame` - Target frame for interactive markers (default: /base_link)
* `interactive_markers_topic` - Topic for interactive markers (default: /basic_controls)
* `depth_cloud` - Set to true if you want depth cloud support (default: false)
* `video_port` - Port for visualizing video streams (default: 9999)
* `depth_topic` - Depth image topic for depthcloud_encoder (default: /camera/depth/image_raw)
* `rgb_topic` - RGB image topic for depthcloud_encoder (default: /camera/rgb/image_raw)
* `config_file` - Configuration file path (default: package config/configuration.json)

Example with custom parameters:

    ros2 launch rvizweb rvizweb.launch.py interactive_markers_target_frame:=/base_footprint interactive_markers_topic:=/advanced_controls

## RVizWeb in a Docker container

To run `RVizWeb` inside a container with ROS 2, you'll need to adapt the Docker scripts for ROS 2.

## Notes about ROS 2 Migration

* Launch files have been converted from XML to Python format
* Dependencies have been updated for ROS 2 compatibility
* The roswww component is not available in ROS 2, so web content serving may need to be handled differently
* Topic and service interfaces follow ROS 2 conventions
