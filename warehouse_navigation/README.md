# warehouse_navigation

Localization and navigation of the warehouse AMR. Phase 1 adds the odometry EKF; AMCL and the
map follow in Phase 2, Nav2 in Phase 3, docking in Phase 5.

## Files

| File | What it does |
|---|---|
| `config/ekf.yaml` | `robot_localization` EKF parameters (key `/**/ekf_filter_node`, so it works in any namespace) |
| `launch/ekf.launch.py` | Starts `ekf_node` as `ekf_filter_node` and remaps its output `odometry/filtered` to `odom`. Include it inside the robot's namespace together with the `/tf` remaps, as `warehouse_bringup/launch/robot.launch.py` does |

## The EKF (plan D3)

| | |
|---|---|
| Inputs | `odom/wheel` (from `diff_drive_controller`): forward speed vx, sideways speed vy and yaw rate. `imu` (from `imu_sensor_broadcaster`): yaw rate |
| Why vy | A diff drive cannot slide sideways, so the controller always reports vy = 0, with a small variance. That zero is a real measurement. Without it nothing observes vy: standing still in Gazebo, the y variance of the output reached about 4e4 m² after 30 s and 7e5 m² after a minute (with it: 2-3 m², like x). Such a covariance is useless to RViz and to later fusion (AMCL, Phase 2). robot_localization's configuration guide recommends fusing it |
| Not fused | The IMU orientation: Gazebo reports the true orientation without noise, which would make localization look better than a real robot could manage. Accelerations: not needed for a planar robot at warehouse speeds |
| Mode | `two_d_mode: true`: z, roll and pitch stay zero |
| Outputs | `odom` (`nav_msgs/Odometry`, 50 Hz) and the transform `odom -> base_footprint` on the robot's `tf`. Nothing else publishes this transform: the controller runs with `enable_odom_tf: false` (plan R5) |

The filter weighs its inputs by their covariances: wheel yaw rate variance 0.01 (from
`controllers.yaml`) against IMU yaw rate variance 4e-6 (σ² of the simulated gyro), so the IMU
dominates the heading while the wheels give the speed.

Measured in Gazebo (Phase 1 acceptance test, 15 runs): over 10.4 m straight the EKF's end point
is within 0.1-0.4 % of ground truth, and over a 370° turn in place its heading is within 0.05 %.
The wheel odometry alone does better in simulation (0.01 % and 0.00 %). The EKF integrates the
controller's reported twist, and in Gazebo that twist reads about 0.2 % high (1.002 m/s at a
true 1.000 m/s): the controller manager's update stamps jitter between 9 and 11 ms, and the
controller computes speed as position change / stamp interval. Each update covers 10 ms of
physics, so a 9 ms interval reads 11 % high and an 11 ms one only 9 % low: the jitter pushes
the average up. The controller's own pose integrates the wheel positions and is unaffected.
Removing the IMU does not change the error. On mock hardware the EKF ends close to the wheel
odometry once the robot stops: 0.01-15 mm apart after eight short drives of 0.3-0.7 m.

**Mock hardware caveat.** With `hardware:=mock` the IMU reports its initial values (zero yaw
rate), and the EKF trusts the IMU's yaw rate far more than the wheels'. In mock mode the EKF
follows straight motion but not turns: mock hardware tests wiring, not localization.

## Running

The EKF starts with every robot (`ros2 launch warehouse_bringup sim.launch.py`). To watch it:

```bash
ros2 topic echo /robot1/odom --field pose.pose
ros2 topic echo /diagnostics     # the EKF's status; diagnostic topics are global (plan §2.3)
```
