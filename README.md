# Tarkbot-Nav

ROS 2 Humble navigation for the Tarkbot mobile base, using a Livox MID-360,
FAST-LIO and Nav2. This repository builds the `airobot_action` package.

The launch file starts the sensor, odometry, base driver, Nav2, task action
server and RViz. After startup, the action server captures the robot pose as a
fixed `task_frame`; clients can then send planar `(x, y)` goals in metres.

## Requirements

- ROS 2 Humble and `colcon`
- Livox MID-360 and `livox_ros_driver2`
- FAST-LIO (`fast_lio`)
- Tarkbot base driver (`tarkbot_robot`)
- Nav2 and RViz (`nav2_bringup`, `rviz2`)

Install missing ROS dependencies with `rosdep` after cloning.

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
# Start the full hardware stack with RViz
ros2 launch airobot_action task_frame_nav.launch.py

# Run without RViz
ros2 launch airobot_action task_frame_nav.launch.py use_rviz:=false

# Send a goal in task_frame coordinates
ros2 action send_goal /task_frame_nav \
  airobot_action/action/NavigateTaskPoint "{x: 1.0, y: 0.0}"

# Optionally send one goal during launch
ros2 launch airobot_action task_frame_nav.launch.py \
  auto_goal:=true goal_x:=2.0 goal_y:=0.5
```

The launch starts physical hardware drivers. Confirm the sensor, odometry,
frames and base command direction before enabling motion.

## Configuration

| Launch argument | Default | Purpose |
| --- | --- | --- |
| `use_rviz` | `true` | Start RViz. |
| `task_frame` | `task_frame` | Fixed frame used for goal coordinates and Nav2 planning. |
| `body_to_base_x/y/z` | `0.145194959 / 0.0 / -0.137544261` m | Sensor-body to chassis-base translation. |
| `body_to_base_roll/pitch/yaw` | `0 / -0.81245 / 0` rad | Sensor mount compensation. |
| `robot_type` | `r20_fwd` | Tarkbot base model. |
| `robot_port` | `/dev/tarkbot_base` | Base serial device. |
| `auto_goal`, `goal_x`, `goal_y` | `false`, `0`, `0` | Optional startup goal in metres. |
| `fast_lio_config`, `nav2_params` | Package defaults | FAST-LIO and Nav2 parameter files. |

Use `ros2 launch airobot_action task_frame_nav.launch.py --show-args` for the
complete launch argument list. The base pose is captured after a three-second
delay by default.

## Frames and topics

FAST-LIO provides `/Odometry` (`camera_init` to `body`) and
`/cloud_registered_body`. The launch publishes the calibrated `body` to
`base_link` transform. The action server captures the composed base pose once
and publishes `camera_init` to `task_frame` on `/tf_static`. Nav2 transforms
the registered cloud through TF; wheel odometry on `/odom` supplies velocity
feedback, with its competing TF publication disabled.

Goals use the captured base axes: positive `x` is forward and positive `y` is
left. The Nav2 footprint and obstacle height limits are in
[`config/nav2_task_frame.yaml`](config/nav2_task_frame.yaml). Measure the mount
transform and chassis footprint on the actual robot before relying on the
default values.

| Topic | Type | Use |
| --- | --- | --- |
| `/task_frame_nav` | `NavigateTaskPoint` action | Task goal interface. |
| `/navigate_to_pose` | `NavigateToPose` action | Nav2 action used internally. |
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO pose input. |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | Local costmap observations. |
| `/Laser_map` | `sensor_msgs/PointCloud2` | Global costmap observations. |
| `/odom` | `nav_msgs/Odometry` | Base velocity feedback. |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 velocity commands to the base. |

See [`action/README.md`](action/README.md) for the action fields and result
codes.

## Tests

```bash
colcon test --packages-select airobot_action
colcon test-result --verbose
```

Tests cover pose composition, frame capture, launch configuration and the
automatic-goal launch path.

## Branches

`main` is the canonical branch. Changes are reviewed through pull requests.

## Acknowledgements

- [FAST-LIO](https://github.com/hku-mars/FAST_LIO), HKU MaRS Lab, for LiDAR-inertial odometry.
- [Nav2](https://github.com/ros-navigation/navigation2) for navigation; [`nav2_task_frame.yaml`](https://github.com/ros-navigation/navigation2/blob/main/nav2_bringup/params/nav2_params.yaml) adapts the `nav2_bringup` defaults for this robot and task frame.
- [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2) for the MID-360 driver.
- Tarkbot hardware and `tarkbot_robot` base driver by [XTARK](https://www.xtark.cn).

## License

This package is released under [Apache-2.0](LICENSE).
