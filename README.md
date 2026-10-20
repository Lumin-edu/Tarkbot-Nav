# Tarkbot-Nav

ROS 2 Humble navigation for the Tarkbot mobile base. The `airobot_action`
package integrates Livox MID-360, FAST-LIO, the base driver, Nav2 and RViz. It
captures the startup base pose as a fixed `task_frame` and accepts planar goals.

## Requirements

ROS 2 Humble, `colcon`, `livox_ros_driver2`, `fast_lio`, `tarkbot_robot`,
`nav2_bringup` and `rviz2`.

## Build

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/Lumin-edu/Tarkbot-Nav.git
cd ~/robot_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to airobot_action
source install/setup.bash
```

## Run

```bash
ros2 launch airobot_action task_frame_nav.launch.py
ros2 launch airobot_action task_frame_nav.launch.py use_rviz:=false
ros2 action send_goal /task_frame_nav \
  airobot_action/action/NavigateTaskPoint "{x: 1.0, y: 0.0}"
ros2 launch airobot_action task_frame_nav.launch.py \
  auto_goal:=true goal_x:=2.0 goal_y:=0.5
```

The launch starts physical drivers. Check sensor data, transform calibration
and command direction before moving the robot. Run
`ros2 launch airobot_action task_frame_nav.launch.py --show-args` to list
options; defaults are defined in `launch/task_frame_nav.launch.py`.

## Frames and interfaces

FAST-LIO provides `/Odometry` (`camera_init` to `body`) and point clouds. The
launch publishes the calibrated `body` to `base_link` transform. After three
seconds, the action server anchors `task_frame` to the captured base pose on
`/tf_static`. Nav2 transforms clouds through TF; `/odom` provides velocity
feedback and does not publish a competing TF.

Goals are metres in `task_frame`: `+x` forward, `+y` left. The task action is
`/task_frame_nav` (`airobot_action/action/NavigateTaskPoint`). Inputs are
`/Odometry`, `/cloud_registered_body`, `/Laser_map` and `/odom`; Nav2 publishes
base commands on `/cmd_vel`. See [`action/README.md`](action/README.md) for
action results and feedback. Tune the mount and footprint in the launch file
and [`config/nav2_task_frame.yaml`](config/nav2_task_frame.yaml).

## Tests

```bash
colcon test --packages-select airobot_action
colcon test-result --verbose
```

## Credits and license

Uses [FAST-LIO](https://github.com/hku-mars/FAST_LIO),
[Nav2](https://github.com/ros-navigation/navigation2), and
[Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2). The Nav2
configuration adapts [`nav2_bringup` defaults](https://github.com/ros-navigation/navigation2/blob/main/nav2_bringup/params/nav2_params.yaml).
Tarkbot hardware and `tarkbot_robot` are by [XTARK](https://www.xtark.cn).
Licensed under [Apache-2.0](LICENSE).
