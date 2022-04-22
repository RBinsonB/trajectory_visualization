# trajectory_visualization

## Overview
A small ROS package for visualizing trajectory from stream of pose or odometry messages. The node will agglomerate a stream of pose into a path that can be visualized on RViz or other visualization tool.

### License
MIT

## Installation

### Dependencies
- [Robot Operating System (ROS)](http://wiki.ros.org)

- **A few basic ROS message packages**

  Install using (replace kinetic by your ROS distro) `sudo apt install ros-kinetic-geometry-msgs ros-kinetic-nav-msgs`

### Building
To build the package, clone the current repository in your catkin workspace using either SSH or HTTP and build it.
```
cd catkin_ws/src
git clone https://github.com/RBinsonB/trajectory_visualization.git
```
Build your workspace with either *catkin_make* or *catkin build*
```
cd ...
catkin_make
```

## Usage
Remap the input pose topic to the pose topic to be tracked either through a launch file or manually. The published path will agglomerate all of the pose into a path. The threshold parameter can be used to specify how far a pose needs to be from the previous one to be added to the path while the max number of pose parameter is used to discard older poses from the path if the limit is reached.

The path can simply be visualized in RViz.

### Example
Below is the example using the Husky gazebo simulation. Install the simulation using the following (replacing kinetic by your ROS distro):
```
sudo apt install ros-kinetic-husky-simulator
```

Run the simulation:

```
roslaunch husky_gazebo husky_empty_world.launch
```

Finally run the trajectory visualization example:

```
roslaunch tracking_visualization odom_example.launch
```

The drone can be driven using the the keyboard teleop package or publishing a command directly on the topic `/cmd_vel`. On RViz the robot can be seen moving with its trajectory tracked.

<img src="/documentation/pictures/husky_example.png" align="center" width="800"/>

Another example launch file used for tracking drone both from its estimator and external Mocap can be found at [/launch/uav_tracking.launch](/launch/uav_tracking.launch) but requires a Mocap specific package.

## Launch files
- **odom_example.launch**: Example of implementation of tracking a robot odometry pose.

- **uav_tracking.launch**: Example of implementation of tracking a UAV to compare the position estimation with a motion capture system.

## Nodes
### trajectory_visualization.py
Track pose from a topic and agglomerate the poses into a path message.

#### Subscribed Topics
- `pose_topic` ([geometry_msgs/Pose](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html))

    Pose topic to be tracked of type `geometry_msgs/Pose`.
    
- `pose_stamped_topic` ([geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))

    Pose topic to be tracked of type `geometry_msgs/PoseStamped`.
    
- `pose_cov_topic` ([geometry_msgs/PoseWithCovariance](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovariance.html))

    Pose topic to be tracked of type `geometry_msgs/PoseWithCovariance`.

- `pose_cov_stamped_topic` ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))

    Pose topic to be tracked of type `geometry_msgs/PoseWithCovarianceStamped`.
    
- `odom_topic` ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/nav_msgs/Odometry.html))

    Pose topic to be tracked from an odometry message `nav_msgs/Odometry`.

#### Published Topics
- `~trajectory_path` ([nav_msgs/Path](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/nav_msgs/Path.html), latched)

    Path from the aggregated tracked pose.

#### Parameters
- `~max_poses` (int, default: 1000)

    Max number of poses in the path. Once reached the first poses will be discarded.  

- `~movement_threshold` (float, default: 0.001)

    Threshold in meters for a new pose to be added to the path. The delta between the last pose in the path and the received one is compared on x, y, and z axis. The pose is added if this delta is higher than the threshold on any of these axis.
    
- `~frame_id` (string, default: map)

    Frame of the path message. Needs to be the same as the one received in the stamped pose messages.  




