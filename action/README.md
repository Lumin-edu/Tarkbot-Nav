# NavigateTaskPoint

`airobot_action/action/NavigateTaskPoint` accepts a planar goal in the captured
`task_frame` and forwards it to Nav2's `NavigateToPose` action.

| Section | Fields | Meaning |
| --- | --- | --- |
| Goal | `x`, `y` (`float64`) | Position in metres; `+x` is forward and `+y` is left. |
| Result | `success`, `error_code`, `message`, `target_pose` | Completion status and resolved Nav2 target. |
| Feedback | `status`, `status_text`, `current_pose`, `distance_remaining`, `navigation_time`, `estimated_time_remaining`, `number_of_recoveries` | Current Nav2 progress. |

Result codes are `0` success, `1` Nav2 aborted, `2` canceled, `3` Nav2
unavailable or rejected the goal, and `4` internal error. The server accepts one
active goal and rejects requests until the task frame is captured and Nav2 is
ready. Cancellation is forwarded to Nav2.

```bash
ros2 action send_goal /task_frame_nav \
  airobot_action/action/NavigateTaskPoint "{x: 1.0, y: 0.0}" --feedback
```
