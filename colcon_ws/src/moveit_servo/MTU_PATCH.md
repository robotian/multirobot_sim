# moveit_servo 2.12.4, patched for simulation time

Upstream: https://github.com/moveit/moveit2/tree/2.12.4/moveit_ros/moveit_servo (the version apt installs for
Jazzy). This copy overlays the apt package in every workspace that builds `colcon_ws/src`.

## Change (`src/servo_node.cpp`, `ServoNode::servoLoop`)

Upstream paces the servo loop with `rclcpp::WallRate(1 / publish_period)` and every iteration integrates
`publish_period` seconds of motion. With `use_sim_time` and a simulator slower than real time (several robots: ~0.5x),
that is more iterations per simulated second, so a servo command moved the arm ~2x too far per simulated second.

With `use_sim_time` (`ros_time_is_active()`) the loop now steps in fixed `publish_period` steps of the node clock:
it waits until the next step time has passed on the clock, uses that step time as `cur_time` (trajectory point
stamps), and runs several steps back to back when `/clock` advanced by more than one period (it ticks once per sim
frame, slower than the 100 Hz loop). A lag of more than 0.25 s (sim paused, time jump) skips ahead instead of
replaying the backlog. On wall time (real robots) the loop is unchanged.

## Other changes

- Upstream `tests/` and the `BUILD_TESTING` block removed (need `ros_testing` and
  `moveit_resources_panda_moveit_config`, which aren't installed here).

## Updating

Copy the new upstream `moveit_ros/moveit_servo`, then reapply the `servoLoop` change (search `MTU patch`).
