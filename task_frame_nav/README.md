# Task-frame navigation

This package starts the FAST-LIO, Tarkbot, Nav2, task action, and RViz
components as one navigation chain.

```bash
source /opt/ros/humble/setup.bash
source /home/hyq/robot/install/setup.bash
ros2 launch airobot_action task_frame_nav.launch.py
```

RViz uses `task_frame` as its fixed frame and displays the raw registered
cloud, TF, local/global costmaps, plans, and the published robot footprint.
Set `use_rviz:=false` to run headless or `rviz_config:=...` to select another
configuration.

## Frame and point-cloud flow

The node accepts only the native FAST-LIO odometry frame pair
`camera_init`/`body`. The native point cloud `/cloud_registered_body` is not
rotated by a separate node. Instead, the launch publishes the calibrated
static transform:

```text
camera_init -> body -> base_link
```

`body -> base_link` defaults to a `-0.812450766803360 rad` pitch
(`-46.55°`) and a translation of `(+0.145194959, 0, -0.137544261) m` in
`body` coordinates. The translation comes from the measured 0.20 m lidar
height: `x = 0.20 sin(46.55°)`, `z = -0.20 cos(46.55°)`. It places
`base_link` at the ground projection of the lidar. A stationary MID360 IMU
reading gave `+46.47°`; a visible plane fit gave `+46.55°`. The plane fit
did not establish ground height; the earlier 0.725 m height estimate was
incorrect. Measure any horizontal offset to the actual chassis center and
adjust x/y before driving close to obstacles.

Both costmaps reject points below 0.10 m above this captured ground plane.
They accept points up to 1.2 m. This provisional upper bound stops ceiling
and other high returns from filling the 2D maps; set it to the measured robot
clearance height before relying on the map near overhead obstacles. The local
DWB controller uses `ObstacleFootprint` to check the configured footprint
along each candidate trajectory. The launch arguments
`body_to_base_roll`, `body_to_base_pitch`, `body_to_base_yaw`, and
`body_to_base_x/y/z` must match the measured mount transform. In particular,
reverse the pitch sign if the live FAST-LIO odometry uses the opposite
convention.

At the three-second capture point, the node composes the current
`camera_init -> body` pose with that static transform and publishes the complete
result once on `/tf_static` as `camera_init -> task_frame`. Therefore
`task_frame` is the captured `base_link` plane, including its calibrated
orientation and height; it is not a separately flattened yaw frame. The live
`base_link` remains the Nav2 robot frame and continues to move beneath the
fixed task anchor. Publishing the anchor as static TF keeps it valid for every
FAST-LIO point-cloud timestamp.

Nav2 uses `task_frame` for planner/costmap global frames and `base_link` for
`robot_base_frame`. Its costmap observation source remains
`/cloud_registered_body`; Nav2 applies TF using each cloud timestamp and
projects the points in the captured base plane. This is why no additional
cloud conversion or 45-degree rotation node is needed.

## Goals

Goals are task-frame FLU coordinates. A goal of `(1.0, 0.0)` is one metre
forward along the captured base x-axis, with zero height in that planning
frame:

```bash
ros2 action send_goal /task_frame_nav \
  airobot_action/action/NavigateTaskPoint "{x: -5.0, y: 0.0}"
```

Or send one automatically during launch:

```bash
ros2 launch airobot_action task_frame_nav.launch.py \
  auto_goal:=true goal_x:=2.0 goal_y:=0.5
```

The default chassis footprint is 20 cm by 16 cm. Tune it in
`config/nav2_task_frame.yaml` if the physical base differs. Wheel odometry is
used for velocity feedback; its TF publication is disabled to avoid a second
pose chain. This four-wheel differential chassis is configured as `r20_fwd` in the
integrated launch and driver defaults. Nav2 does not request lateral motion
(`linear.y = 0`). With `r20_fwd`, a low-speed test confirmed that forward commands move the
robot forward, while `invert_angular_z:=true` makes a positive-Z turn go
right. The driver and integrated launch now default to
`invert_angular_z:=false`, so positive-Z commands are sent directly. Verify
that positive-Z turns left before using automatic navigation. The same
optional inversion applies to outgoing commands and wheel odometry.
