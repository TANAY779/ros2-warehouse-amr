# warehouse_bringup

Top-level launch files and configuration. This is the entry point for running the system: every
demo starts with `ros2 launch warehouse_bringup ...`.

## Files

| File | What it does |
|---|---|
| `launch/sim.launch.py` | Everything for one simulated robot: Gazebo (world scope, `hardware:=gz` only), the robot (`robot.launch.py`) and RViz. Sets `use_sim_time` for every node with `SetParameter`. RViz runs in the robot's absolute namespace with the same `/tf` remaps as the robot |
| `launch/robot.launch.py` | The robot scope (plan §2.3), inside `PushROSNamespace(/<namespace>)` with `/tf` and `/tf_static` remapped to the namespace: `robot_state_publisher`, the controller manager (in Gazebo, or `ros2_control_node` for mock hardware), the three controller spawners and the EKF. Sets `use_sim_time` for its own nodes, so it also runs on its own. The namespace is absolute because the gz topic names in the URDF are: an enclosing namespace must not move the ROS side away from them. Phase 9 includes it once per robot |
| `rviz/robot.rviz` | RViz view of one robot: model, TF, scan (best effort), EKF and wheel odometry. Topic names are relative, so they resolve in the namespace RViz runs in |

## Running

From the workspace root, after `colcon build` and `source install/setup.bash`:

```bash
# Gazebo window + RViz, robot in /robot1 (the defaults)
ros2 launch warehouse_bringup sim.launch.py world:=empty namespace:=robot1

# No windows at all (SSH, tests); the scanner still renders through EGL
ros2 launch warehouse_bringup sim.launch.py headless:=true

# No simulator: mock hardware, wall clock. What CI tests
ros2 launch warehouse_bringup sim.launch.py hardware:=mock

# World and robot separately, as Phase 9 will do once per robot (two terminals)
ros2 launch warehouse_gazebo gazebo.launch.py headless:=true
ros2 launch warehouse_bringup robot.launch.py namespace:=robot1
```

`robot.launch.py` takes the same `namespace`, `hardware`, `use_sim_time`, `x`, `y` and `yaw`
arguments as `sim.launch.py` (table below); it does not start Gazebo or RViz.

| Argument | Default | Meaning |
|---|---|---|
| `world` | `empty` | World file in `warehouse_gazebo/worlds` |
| `namespace` | `robot1` | Robot namespace: nodes, topics, TF and gz topics live under it. `namespace:=/` runs the robot without one (`ros2 launch` refuses an empty value) |
| `hardware` | `gz` | `gz` (Gazebo through `gz_ros2_control`) or `mock` (`mock_components`). `real` is reserved and refused |
| `use_sim_time` | `true` for `gz`, `false` for `mock` | Use Gazebo's `/clock` |
| `headless` | `false` | No Gazebo window and no RViz |
| `rviz` | `true` | Start RViz (never when headless) |
| `x`, `y`, `yaw` | `0.0` | Spawn pose in the world (`gz` only) |

## Driving with teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r __ns:=/robot1 -p stamped:=true -p use_sim_time:=true
```

- `-r __ns:=/robot1` puts teleop in the robot's namespace, so it publishes `/robot1/cmd_vel`.
- `stamped:=true`: Jazzy's `diff_drive_controller` 4.42 accepts only `TwistStamped`.
- `use_sim_time:=true` (Gazebo only; leave it out for `hardware:=mock`): the controller computes
  the age of a command from its header stamp and brakes after `cmd_vel_timeout` (0.5 s). A
  wall-clock stamp is decades ahead of sim time, never looks old, and the robot keeps driving
  after you release the key (plan R1).

Hold a key to drive; release it and the robot stops within 0.5 s plus braking. The hard limits
(1.2 m/s forward, 0.3 m/s reverse, 2 rad/s) apply whatever teleop asks for.

## Checking the robot by hand

```bash
ros2 control list_controllers -c /robot1/controller_manager    # three controllers, all active
ros2 topic info -v /robot1/cmd_vel     # TwistStamped, diff_drive_controller the only subscriber
ros2 topic info -v /robot1/tf          # publishers: robot_state_publisher and ekf_filter_node
ros2 topic echo /robot1/odom/wheel --field child_frame_id      # base_footprint, no prefix
ros2 topic hz /robot1/scan             # 10 Hz (when Gazebo runs at real time)
# TF tools listen on /tf, so remap them to the robot's tree:
ros2 run tf2_tools view_frames --ros-args -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static
ros2 run rqt_tf_tree rqt_tf_tree --ros-args -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static
```

## Things that look wrong but are not

- `diff_drive_controller` logs `Velocity command timed out. Braking.` once per second whenever
  no command is arriving, including at rest. That is `cmd_vel_timeout` at work.
- `ros2 topic info /tf` shows one publisher, `diff_drive_controller`, which never sends anything:
  the controller always creates its `/tf` publisher and only `enable_odom_tf: false` keeps it
  quiet. The controller manager runs inside Gazebo (or, for mock hardware, outside the `/tf`
  remaps, which it would otherwise pass on to every controller), so that publisher stays on the
  global `/tf`.
- `Could not enable FIFO RT scheduling policy` from the controller manager: a normal desktop
  user may not use real-time scheduling. Simulation does not need it.

## Prerequisites

`rosdep install --from-paths src --ignore-src -y` installs everything this package names,
including the tools above (`teleop_twist_keyboard`, `rqt_tf_tree`, `tf2_tools`, `ros2controlcli`).
