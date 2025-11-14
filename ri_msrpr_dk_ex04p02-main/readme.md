# Serial Manipulator Direct Kinematic Node

## Overview

`ri_msrpr_dk_ex04p02` is a ROS 2 package designed for serial manipulator that calculates forward kinematic model based on C-Space information

## Features

- Computes robot pose using forward kinematics.
- Publishes end-effector pose in `geometry_msgs/Pose` format.

## Requirements

- ROS 2 Humble
- Python 3
- `rclpy`
- `geometry_msgs`
- `numpy`

## Directory Structure

```bash
ri_msrpr_dk_ex04p02/
├── ri_msrpr_ex04p02/
│   ├── __init__.py
│   └── ri_msrpr_dk_node_ex04p02.py
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── setup.py
└── setup.cfg
```

## Clone to a ROS2 existing Workspace

Clone the package into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone <your-repo-url>
cd ~/ros2_ws
colcon build --packages-select ri_msrpr_dk_ex04p02
source install/setup.bash
```

Or

## Workspace and Package Creation

```bash
mkdir -p ~/ros2_ws/src
echo ''source ~/ros2_ws/install/setup.bash'' >> ~/.bashrc
source ~/.bashrc
```
In the `src` folder inside of the Workspace
```bash
ros2 pkg create <ros_package_name> --build-type ament_python  --dependencies <package_dependencies>
```

## Building

From the root of the workspace:

```bash
colcon build --packages-select ri_msrpr_dk_ex04p02
source install/setup.bash
```

## Usage

### Hard-code Inputs

To run the node:

```bash
ros2 run ri_msrpr_dk_ex04p02 ri_msrpr_dk_node_ex04p02
```

## License

Apache 2.0

## Detail Description `ri_msrpr_dk_node_ex04p02` 

**Initial Conditions in the construct**


```python
# initial conditions
self.th = 0.0 
```

**Subscriber and Publishers**

- Subscribre to `cspace_cmd` topic.
- Publishes pose to `ef_pose` topic.

```python
# publishers and subscribers
qos = QoSProfile(depth=10)

# subscriber
self.cspace_sub = self.create_subscription(
    msg_type=Twist,
    topic="cspace_cmd",
    callback=self.cspace_callback,
    qos_profile=qos
)

# publisher
self.ef_pose_pub = self.create_publisher(
    msg_type=Pose,
    topic="ef_pose",
    qos_profile=qos
)
```

**Main Update Logic**

Each timer tick:
- Redefine DH Parameter Matrix
- Calculated T_DH
- Calculared ${}^{0}_{n}T$
- Publishes updated Pose.


**Pose Message Publisher**

Publishes a `geometry_msgs/Pose` message with current position, orientation (as quaternion).

```python
ef_pose_msg = Pose()
ef_pose_msg.position.x 
ef_pose_msg.position.y 
ef_pose_msg.position.z

ef_pose_msg.orientation.x 
ef_pose_msg.orientation.y
ef_pose_msg.orientation.z 
ef_pose_msg.orientation.w 

# publish the message
self.ef_pose_pub.publish(ef_pose_msg)
```

## Author

*Professor*: David Rozo-Osorio, I.M. M.Sc. email: david.rozo31@eia.edu.co

**EIA University**, Mechatronical Eng. - Industrial Robotics