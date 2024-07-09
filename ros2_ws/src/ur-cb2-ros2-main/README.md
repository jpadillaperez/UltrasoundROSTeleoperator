# Universal Robot CB2 ROS2 Driver

## 
ROS 2 node for Universal Robot CB2 Controller based on this repo: (https://github.com/jhu-bigss/Universal_Robots_CB2_ROS2_Driver).

Tested on Ubuntu 22.04, ROS2 Humble Hawksbill

## Build
Clone everything to your ROS2 workspace under the `src` directory,
Make sure all the dependencies are met by running:

```bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys "reflexxes_type2"
```

* Make sure fortran is installed in order for `cisstNetlib` to be build. See cisstNetlib [Issue#5](https://github.com/jhu-cisst/cisstNetlib/issues/5#issuecomment-1169452231)
```bash
sudo apt install gfortran-9
```
* If building failed due to `CMake Error: Could NOT find SWIG (missing: SWIG_EXECUTABLE SWIG_DIR)`, then do:
```bash
sudo apt install swig
```

At the root of your ROS 2 workspace, build using:
```bash
colcon build
```

## Instruction

Once the code is compiled. Source the setup file:
```bash
source install/local_setup.bash
```

You can launch the driver using:
```bash
ros2 launch ur_cb2_bringup ur_control.launch.py ur_type:=<UR_TYPE> robot_ip:=<IP_OF_THE_ROBOT> use_fake_hardware:=<true/false> launch_rviz:=<true/false>
```
Example:
```bash
ros2 launch ur_cb2_bringup ur_control.launch.py ur_type:=ur5 robot_ip:=127.0.0.1 use_fake_hardware:=true launch_rviz:=true
```
- <UR_TYPE> can be 'ur5' or 'ur10'. For e-Series robots, please use official Universal Robot driver.
- You can see all the launch arguments by `--show-args`

Once the robot driver is running, you can launch the Moveit2 to plan the robot's motion in Rviz2:
```bash
ros2 launch ur_cb2_moveit_config ur_moveit.launch.py ur_type:=<UR_TYPE>
```
Example:
```bash
ros2 launch ur_cb2_moveit_config ur_moveit.launch.py ur_type:=ur5
```