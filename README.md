# Robot-Glove-Repo
# Set-up
I have not tested the nuances of transfering code to a different computer and setting it up, but the source file working on my computer is in this repository and can be built with
```
colcon build
```
---
Use this in a terminal to make sure you have ID 93
```
echo $ROS_DOMAIN_ID
```

If the ID is not set, you can change the .bashrc file to set this when opening a new terminal automatically. This is also useful for automatically sourcing ROS.
```
nano ~/.bashrc
```
Add this to your .bashrc file
```
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=93

# ROS2 GUI + WSL2 settings
export DISPLAY=$(grep -oP "(?<=nameserver ).*" /etc/resolv.conf):0
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1
export LIBGL_ALWAYS_INDIRECT=0
```
# How to start a simulation
First, in a new terminal, start a robot_state_publisher using ONE the following:
- One joint
```
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat ~/teleop_arm_ws/src/teleop_arm_sim/urdf/one_joint.urdf)"
```
- One finger
```
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat ~/teleop_arm_ws/src/teleop_arm_sim/urdf/one_finger.urdf)"
```
- Three fingers
```
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/teleop_arm_ws/src/teleop_arm_sim/urdf/three_finger.urdf.xacro)"
```

---
Then, in a new terminal, open RViz using the following
```
rviz2
```
A few settings need to be changed:
- In "Global Options", select "base_link"
- Click "Add" and select "RobotModel"
  - In "Description Source", select "Topic"
  - In "Description Topic", select "/robot_description"
It might be glitched out, but that's because it does not have any initial joint positions

---
Then, in a new terminal, bring up the built-in joint controller
```
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```
Alternatively, open MATLAB and run the joint controller app. This currently only works for the three-finger hand model
