# Distance Warning

## Overview

This package contains:
- **Distance Publisher**
- **Distance Listener**
- **Threshold Service**
- **Action Server/Client**

## Prerequisites

- ROS2 Humble
- C++ compiler (if using C++ nodes) and Python interpreter
- Custom message/service interfaces: `distance_warning_interfaces`


## Build

```bash
cd ~/ros2_ws
colcon build --packages-select distance_warning_interfaces distance_warning
source ~/ros2_ws/install/setup.bash
```

## Run

### Python Implementation

```bash
ros2 run distance_warning set_threshold_service.py
ros2 run distance_warning distance_listener.py
ros2 run distance_warning distance_publisher.py
ros2 run distance_warning distance_action_server.py
ros2 run distance_warning distance_action_client.py
```

### C++ Implementation

```bash
ros2 run distance_warning set_threshold_service
ros2 run distance_warning distance_listener
ros2 run distance_warning distance_publisher
ros2 run distance_warning distance_action_server
ros2 run distance_warning distance_action_client
```

### Calling services 

```bash
ros2 service call /set_threshold distance_warning_interfaces/srv/SetThreshold "{data: true}"
```

### Launch all

```bash
ros2 launch distance_warning.launch.py
```