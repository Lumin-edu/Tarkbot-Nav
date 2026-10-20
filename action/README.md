# NavigateTaskPoint

`airobot_action/action/NavigateTaskPoint` sends an `(x, y)` goal in the captured
`task_frame` to Nav2. Coordinates are metres; `+x` is forward and `+y` is left.

- Goal: `x`, `y` (`float64`).
- Result: `success`, `error_code`, `message`, `target_pose`.
- Feedback: Nav2 status, current pose, distance and timing estimates, recoveries.
- Result codes: `0` success, `1` aborted, `2` canceled, `3` Nav2 unavailable
  or rejects the goal, `4` internal error.

Only one goal runs at a time. Goals are rejected until the task frame is captured
and Nav2 is ready; cancellation is forwarded to Nav2.

```bash
ros2 action send_goal /task_frame_nav \
  airobot_action/action/NavigateTaskPoint "{x: 1.0, y: 0.0}" --feedback
```
