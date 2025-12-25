# Building RVizWeb for ROS 2 Humble

## Prerequisites

Before building RVizWeb, ensure you have:

1. ROS 2 Humble Hawksbill installed
2. Node.js (LTS version recommended)
3. Git

## Dependencies

RVizWeb requires several ROS 2 packages that may not be installed by default:

```bash
# Install required ROS 2 packages
sudo apt update
sudo apt install -y ros-humble-rosbridge-server ros-humble-tf2-web-republisher ros-humble-web-video-server ros-humble-interactive-marker-proxy ros-humble-depthcloud-encoder
```

## Building the Package

1. Create a ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone the RVizWeb package:

```bash
git clone <repository-url> rvizweb
```

3. Navigate to the workspace root and build:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select rvizweb
```

4. Source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

## Running RVizWeb

Launch RVizWeb with:

```bash
ros2 launch rvizweb rvizweb.launch.py
```

## Notes

- The web interface files (HTML, CSS, JS) will be built during the CMake configuration step using Node.js, bower, and polymer-cli
- The web interface connects to ROS 2 via rosbridge
- Since roswww is not available in ROS 2, static web content serving may need to be handled differently
- Some executable names in the launch file may need to be adjusted based on the actual ROS 2 package installations