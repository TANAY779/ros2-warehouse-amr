# Phase reports

One learning report per finished phase, written for someone learning ROS 2: what was built,
the concepts behind it, what went wrong and how it was fixed, and hands-on exercises. Each
finished phase adds `phase-NN-<name>.md` here before its `phase-N` tag; the current state of
every feature is in [PROJECT_STATUS.md](../PROJECT_STATUS.md).

| Phase | Report | Summary |
|---|---|---|
| 0 | [Repository foundation](phase-00-foundation.md) | Workspace layout, ament lint gates, pre-commit, CI, dev containers, ADRs, git workflow, and the bugs verification found |
| 1 | [The robot, its controllers and the first drive in Gazebo](phase-01-robot-description.md) | URDF/xacro from one `robot.yaml`, TF, ros2_control (Gazebo and mock hardware), gz sensors and the ros_gz bridge, QoS, diff-drive odometry, the EKF, namespaces, sim time, launch files and launch_testing; the problems found and the measured results; exercises including recording the demo GIF |
