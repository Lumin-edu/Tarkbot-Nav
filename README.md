# Task-frame navigation

`airobot_action` contains the `task_frame_nav` action server and launch file
for the MID360, FAST-LIO, Tarkbot, Nav2, and RViz stack.

Build and start it with:

```bash
cd /home/hyq/robot
source /opt/ros/humble/setup.bash
colcon build --packages-up-to airobot_action
source install/setup.bash
ros2 launch airobot_action task_frame_nav.launch.py
```

RViz is enabled by default; use `use_rviz:=false` to disable it. A one-shot
goal can be sent from launch:

```bash
ros2 launch airobot_action task_frame_nav.launch.py \
  auto_goal:=true goal_x:=4.0 goal_y:=1.0
```

## Coordinate frames

FAST-LIO's native `camera_init -> body` odometry and
`/cloud_registered_body` point cloud are kept unchanged. The radar/IMU is
mounted with a 45-degree pitch, so the launch publishes a fixed
`body -> base_link` transform (default pitch `-pi/4`). The same transform is
used when composing each FAST-LIO pose:

```text
camera_init -> body -> base_link
```

The sign of `body_to_base_pitch` must match the measured FAST-LIO convention:
with a native body pitch of `+45°` (forward axis pointing downward), the
default `-45°` compensation makes `base_link` parallel to the ground. If the
live odometry reports the opposite sign, set the launch argument accordingly.

After three seconds the node captures the composed `camera_init -> base_link`
pose and publishes it once as a latched static `camera_init -> task_frame`
anchor. The anchor preserves the complete compensated base orientation and
position, including any residual roll/pitch. Thus `task_frame` and `base_link`
represent the same chassis plane at capture, while `base_link` continues to
follow the live robot. The static publication is important: replaying the
captured transform with old FAST-LIO timestamps makes Nav2's message filter
discard otherwise valid point clouds.

Nav2 uses `task_frame` as its global frame and `base_link` as its robot frame.
It transforms `/cloud_registered_body` through TF at each message timestamp
into that plane, so no extra point-cloud rotation node is required. Goals are
expressed directly in `task_frame`; `(1, 0)` means one metre along the captured
base x-axis on the planning plane.

The `body_to_base_x/y/z` arguments are the measured lever arm from FAST-LIO's
`body` origin to the vehicle `base_link` origin. Pass them to the launch file
once calibrated. The Tarkbot driver's wheel odometry is retained for velocity
feedback, but its TF publication is disabled so FAST-LIO plus the static mount
transform remain the only pose TF path.

See [task_frame_nav/README.md](task_frame_nav/README.md) for all launch
arguments and action examples.
