# AGENTS.md

## Project Direction

This repository is ROS1 Noetic only. Do not keep, restore, or add ROS2 functionality.

Work in small, staged changes. Complete only one small task per iteration, then stop and report what changed and what was verified. Do not modify across phases in the same pass.

## Hard Constraints

- Use ROS1 Noetic only.
- Do not use `ros1_bridge`.
- Do not add ROS2 dependencies, APIs, generated interfaces, build systems, launch systems, or bag tooling.
- Forbidden dependency/API tokens include:
  - `rclcpp`
  - `ament`
  - `rosidl`
  - `rosbag2`
  - `builtin_interfaces`
  - `livox_interfaces2`

If one of these tokens already exists, treat it as migration debt to remove in a focused task. Do not add new usage.

## ROS1 Inputs

- `livox_lidar_topic`: `/livox/lidar`
  - Rate: 10 Hz
  - Preferred message type: ROS1 `livox_ros_driver2` `CustomMsg`
- `imu_topic`: `/livox/imu`
  - Rate: 200 Hz
  - Message type: `sensor_msgs/Imu`
- `chassis_feedback`: `/chassis_feedback`
  - Rate: 50 Hz
  - Message type: `yhs_msgs/chassis_feedback`
  - Required fields:
    - `rear_right_wheel_speed_mps`
    - `rear_left_wheel_speed_mps`
    - `vehicle_speed_mps`

## Build Command

Run builds from a ROS1 Noetic environment:

```bash
source /opt/ros/noetic/setup.bash
catkin build lightning
```

## Required Verification After Every Change

After each change, run both commands:

```bash
rg -n "rclcpp|ament|rosidl|rosbag2|builtin_interfaces|livox_interfaces2" .
source /opt/ros/noetic/setup.bash
catkin build lightning
```

The `rg` command must not show newly introduced ROS2 usage. If existing matches remain, document them explicitly and do not expand them.

## Regression Test Coverage Required

Each implementation task must preserve or add regression coverage for the affected behavior. The regression set for this project must include:

- Mapping regression test: verifies ROS1 Livox lidar plus IMU input can run the mapping flow without ROS2 dependencies.
- Localization regression test: verifies ROS1 localization input and output behavior remains stable.
- Save-map regression test: verifies generated maps are saved through the ROS1 code path.
- Evaluation-script regression test: verifies evaluation scripts still run against expected map/localization outputs.
- Wheel-speed fusion regression test: verifies `/chassis_feedback` data is consumed with `rear_right_wheel_speed_mps`, `rear_left_wheel_speed_mps`, and `vehicle_speed_mps` and fused without ROS2 interfaces.

When a task changes behavior in one of these areas, update the narrowest relevant regression first instead of making broad unrelated changes.

## Development Rules

- Prefer existing ROS1 package patterns in this repository.
- Do not introduce ROS2 compatibility shims.
- Do not rename topics, message fields, or build targets unless the current task explicitly requires it.
- Keep edits narrowly scoped to the requested task.
- Report verification results, including the ROS2-token scan and `catkin build lightning` result, before handing work back.
