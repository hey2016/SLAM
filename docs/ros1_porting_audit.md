# ROS1 Porting Audit

Audit date: 2026-05-08

Scope: full repository scan for ROS2 build dependencies, ROS2 runtime APIs, ROS2 message/service interfaces, ROS2 bag tooling, and nested shared pointer patterns requested by the porting rules.

Primary scan command:

```bash
rg -n "rclcpp|ament|rosidl|rosbag2|builtin_interfaces|livox_interfaces2|std::shared_ptr<[^\n]*::SharedPtr" .
```

Additional interface scan:

```bash
rg -n "rclpy|[a-zA-Z0-9_]+_msgs::msg|[a-zA-Z0-9_]+_msgs/msg|livox_ros_driver2::msg|livox_ros_driver2/msg|lightning/srv|::srv::|srv::|std_srvs::srv|tf2_ros::TransformBroadcaster|spin\(node_|create_subscription|create_service" .
```

## Summary

- The repository is currently structured as a ROS2 package using `ament_cmake`, `rosidl_generate_interfaces`, `rclcpp`, ROS2 message include paths, ROS2 service headers, and `rosbag2`.
- Root build files and `thirdparty/livox_ros_driver` are ROS2 build-system blockers for `catkin build lightning`.
- Online SLAM/localization wrappers use `rclcpp::Node`, `create_subscription`, `create_service`, ROS2 `SharedPtr` message types, and ROS2 tf broadcaster construction.
- Offline tools are tied to `rosbag2_cpp` and ROS2 serialized-message deserialization.
- Several files under `src/core` expose ROS2 message types directly in algorithm-facing APIs. These are high risk because ROS2 types have leaked past the wrapper boundary.
- No exact `std::shared_ptr<...::SharedPtr>` nested pointer pattern was found. A related ROS2 bag pointer exists in `src/wrapper/bag_io.h`: `std::shared_ptr<rosbag2_storage::SerializedBagMessage>`.

## A. Build System

| File | ROS2 findings | ROS1 replacement recommendation | Risk |
| --- | --- | --- | --- |
| `CMakeLists.txt` | Uses `rosidl_generate_interfaces`, generated ROS2 include dir `rosidl_generator_cpp`, adds ROS2 `thirdparty/livox_ros_driver`, ends with `ament_package()`. | Convert root build to catkin: `find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs std_srvs message_generation ...)`; use `add_service_files(FILES SaveMap.srv LocCmd.srv)`, `generate_messages(...)`, `catkin_package(...)`; remove `ament_package()` and ROS2 generated include paths. | High |
| `package.xml` | Uses `ament_cmake`, `rosidl_default_generators`, `rosidl_default_runtime`, `rclcpp`, `rosidl_interface_packages`, `ament_lint_*`, and `<build_type>ament_cmake</build_type>`. | Convert to ROS1 package format dependencies: `catkin`, `roscpp`, `message_generation`, `message_runtime`, `std_msgs`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `std_srvs`, `tf2`, `tf2_ros`, `pcl_ros`, `pcl_conversions`, `message_filters`, and the ROS1 Livox/yhs message packages. | High |
| `cmake/packages.cmake` | Finds `ament_cmake`, `rclcpp`, `rosbag2_cpp`, `rosidl_default_generators`; exports ROS2 include/library variables; includes ROS2 Livox generated path. | Replace with catkin components and ROS1 libraries. Remove `rosbag2_cpp`; use ROS1 `rosbag` for offline bags. Remove ROS2 generated interface include path; rely on catkin-generated headers and installed ROS1 message packages. | High |
| `src/CMakeLists.txt` | Uses `ament_target_dependencies`, links `livox_interfaces2__rosidl_typesupport_cpp` and `${PROJECT_NAME}__rosidl_typesupport_cpp`, adds ROS2 type support dependencies. | Replace with `target_link_libraries(... ${catkin_LIBRARIES} ...)` and `add_dependencies(... ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})`. Remove `livox_interfaces2` and ROS2 type support targets. | High |
| `thirdparty/livox_ros_driver/CMakeLists.txt` | ROS2 Livox driver build: `-DBUILDING_ROS2`, `ament_cmake_auto`, `builtin_interfaces`, `rosidl_default_generators`, `livox_interfaces2`, `rosidl_generate_interfaces`, commented `rclcpp_components`, commented `ament_auto_package`. | Replace this subtree with the ROS1 mode/package of `livox_ros_driver2`, or vendor only the ROS1 `msg/CustomMsg.msg` and `msg/CustomPoint.msg` through catkin. Use ROS1 `std_msgs/Header` timestamps, not `builtin_interfaces`. | High |
| `thirdparty/livox_ros_driver/package.xml` | Described as ROS2 driver; depends on `ament_cmake_auto`, `rosidl_default_generators`, `rclcpp`, `rclcpp_components`, `rosbag2`, `rosidl_default_runtime`, `ament_lint_*`, `<build_type>ament_cmake</build_type>`. | Replace with ROS1 catkin metadata for `livox_ros_driver2` or remove vendored ROS2 package and depend on installed ROS1 `livox_ros_driver2`. | High |

## B. ROS Wrapper/App, Runtime Interfaces, TF, Services, Bags

| File | ROS2 findings | ROS1 replacement recommendation | Risk |
| --- | --- | --- | --- |
| `src/wrapper/bag_io.h` | Includes ROS2 message headers (`nav_msgs/msg`, `sensor_msgs/msg`, `livox_ros_driver2/msg`), `rclcpp` serialization, `rosbag2_cpp`; uses `rosbag2_storage::SerializedBagMessage`, ROS2 `::msg::...::SharedPtr`, and ROS2 serializers. | Replace with ROS1 `rosbag::Bag`/`rosbag::View`, ROS1 headers (`sensor_msgs/Imu.h`, `sensor_msgs/PointCloud2.h`, `livox_ros_driver2/CustomMsg.h`, `nav_msgs/Odometry.h`), and callback signatures using `ConstPtr` or `boost::shared_ptr` as appropriate. | High |
| `src/wrapper/bag_io.cc` | Uses `rosbag2_cpp::Reader`, `SequentialReader`, `ConverterOptions`, sqlite3 ROS2 bags. | Replace with ROS1 `.bag` traversal using `rosbag::View`, topic filters, and `instantiate<T>()`. Offline mapping/localization should accept ROS1 bags directly. | High |
| `src/wrapper/ros_utils.h` | Includes `rclcpp/rclcpp.hpp`; utility functions accept `builtin_interfaces::msg::Time`. | Replace with ROS1 `ros/time.h` or `std_msgs/Header` stamp handling. `ToSec` should accept `ros::Time`; `ToNanoSec` should use `time.toNSec()`. | Medium |
| `src/app/run_slam_online.cc` | Uses `rclcpp::init`/`shutdown`; comments reference `rclcpp`. | Replace with `ros::init(argc, argv, "lightning_slam")`, construct ROS1 node handles inside `SlamSystem`, and use `ros::shutdown()` only where needed. | Medium |
| `src/app/run_loc_online.cc` | Uses `rclcpp::init`/`shutdown`. | Replace with `ros::init(argc, argv, "lightning_loc")`; use ROS1 spin in `LocSystem`. | Medium |
| `src/app/run_slam_offline.cc` | Offline callbacks use ROS2 message types from `RosbagIO`: `sensor_msgs::msg::PointCloud2::SharedPtr`, `livox_ros_driver2::msg::CustomMsg::SharedPtr`. | After `RosbagIO` conversion, switch callback signatures to ROS1 `sensor_msgs::PointCloud2ConstPtr` and `livox_ros_driver2::CustomMsgConstPtr` or internal converted cloud structs. | Medium |
| `src/app/run_loc_offline.cc` | Same ROS2 callback types as offline SLAM. | Same as above; keep offline localization independent of ROS2 bag serialization. | Medium |
| `src/app/run_loop_offline.cc` | Uses ROS2 `sensor_msgs::msg::PointCloud2::SharedPtr` via `RosbagIO`. | Convert to ROS1 `sensor_msgs::PointCloud2ConstPtr` or internal cloud input after bag reader migration. | Medium |
| `src/app/run_frontend_offline.cc` | Uses ROS2 `sensor_msgs::msg::PointCloud2::SharedPtr` via `RosbagIO`. | Convert to ROS1 `sensor_msgs::PointCloud2ConstPtr` or internal cloud input after bag reader migration. | Medium |
| `src/core/system/slam.h` | Includes `rclcpp`, ROS2 `sensor_msgs/msg`, ROS2 generated service `lightning/srv/save_map.hpp`, `livox_ros_driver2/msg/custom_msg.hpp`; stores `rclcpp::Node::SharedPtr`, `rclcpp::Subscription`, `rclcpp::Service`; method signatures expose ROS2 `SharedPtr`. | Treat this as wrapper code and convert to ROS1: include `ros/ros.h`, `sensor_msgs/Imu.h`, `sensor_msgs/PointCloud2.h`, `livox_ros_driver2/CustomMsg.h`, generated `lightning/SaveMap.h`; store `ros::NodeHandle`, `ros::Subscriber`, `ros::ServiceServer`; use `ConstPtr` callbacks and keep algorithm API behind internal types where possible. | High |
| `src/core/system/slam.cc` | Creates `rclcpp::Node`; uses `rclcpp::QoS`, `create_subscription`, `create_service`, ROS2 callback signatures, `spin(node_)`; logs "ros2 node". | Replace with ROS1 subscribers: `nh.subscribe<sensor_msgs::Imu>(imu_topic, queue, cb)`, `nh.subscribe<livox_ros_driver2::CustomMsg>(livox_topic, queue, cb)`, `nh.advertiseService("lightning/save_map", ...)`, and `ros::spin()`/`ros::AsyncSpinner`. Map save service callback should use ROS1 request/response references. | High |
| `src/core/system/loc_system.h` | Includes `rclcpp`, ROS2 message headers, ROS2 Livox header; stores `rclcpp::Node::SharedPtr`, ROS2 subscriptions, and `std::shared_ptr<tf2_ros::TransformBroadcaster>` constructed from ROS2 node. | Convert to ROS1 `ros::NodeHandle`, `ros::Subscriber`, ROS1 `tf2_ros::TransformBroadcaster` or `tf::TransformBroadcaster`; use ROS1 message headers and `ConstPtr` callbacks. | High |
| `src/core/system/loc_system.cc` | Creates `rclcpp::Node`, `rclcpp::QoS`, subscriptions, ROS2 `tf2_ros::TransformBroadcaster(node_)`, callback with `geometry_msgs::msg::TransformStamped`, `spin(node_)`; logs "ros2 node". | Replace with ROS1 subscriptions and broadcaster construction without node argument. Callback should accept/send ROS1 `geometry_msgs::TransformStamped`. Use `ros::spin()` or controlled spinner. | High |
| `scripts/merge_bags.py` | Imports `rclpy` and `rosbag2_py`; merges/decompresses ROS2 `.db3` bags and includes conversion flow from ROS1 to ROS2. | Replace with ROS1 `rosbag` Python API for `.bag` merging/filtering, or delete if not needed. Do not require `rosbags-convert` to ROS2. | High |
| `scripts/save_default_map.sh` | Calls `ros2 service call /lightning/save_map lightning/srv/SaveMap`. | Replace with ROS1 `rosservice call /lightning/save_map "map_id: 'new_map'"` after service definition is generated by catkin. | Medium |
| `README.md` | Documents ROS2 Humble, `colcon build`, `ros2 run`, `ros2 service call`, ROS2 db3 datasets, and ROS1-to-ROS2 conversion. | Rewrite usage to ROS1 Noetic: `catkin build lightning`, `source devel/setup.bash`, `rosrun lightning ...`, ROS1 bag playback/recording, and `rosservice call`. | Low |
| `README_CN.md` | Same ROS2 documentation patterns as English README. | Rewrite to ROS1 Noetic equivalents. | Low |
| `docker/README.md` | Mentions mounting `nclt.db3`. | Update examples to ROS1 `.bag` inputs. | Low |

## C. Algorithm Core ROS2 Coupling

Files below are in algorithm/core directories but expose ROS2 messages directly. These should be treated as high-risk porting points because they make the algorithm layer depend on middleware-specific message types.

| File | ROS2 findings | ROS1 replacement recommendation | Risk |
| --- | --- | --- | --- |
| `src/core/lightning_math.hpp` | Includes `rclcpp/time.hpp`; returns `builtin_interfaces::msg::Time` from `FromSec`. | Remove ROS2 time from math utilities. Use `ros::Time FromSec(double)` only in ROS wrapper code, or keep core pure by returning numeric seconds/nanoseconds and converting in ROS adapters. | High |
| `src/common/options.h` | Includes `rclcpp/rclcpp.hpp`; signal handler calls `rclcpp::shutdown()`. | Remove middleware shutdown from common options. Use a core-only exit flag; have ROS1 app/wrapper call `ros::shutdown()` from its own signal handling. | High |
| `src/core/lio/laser_mapping.h` | Includes ROS2 `sensor_msgs/msg/point_cloud2.hpp` and `livox_ros_driver2/msg/custom_msg.hpp`; public algorithm API accepts ROS2 `SharedPtr` messages. | Move ROS message conversion to wrapper/preprocess adapter. Prefer core API `ProcessPointCloud2(CloudPtr)` plus ROS1 adapter overloads in a ROS wrapper file. If overloads remain, use ROS1 message types only. | High |
| `src/core/lio/laser_mapping.cc` | Processes ROS2 `sensor_msgs::msg::PointCloud2` and ROS2 Livox `CustomMsg`; reads ROS2 stamp fields `.sec`/`.nanosec` through `ToSec`. | Convert stamps through ROS1 `header.stamp.toSec()` or push only internal `CloudPtr` with timestamp into core. | High |
| `src/core/lio/pointcloud_preprocess.h` | Includes ROS2 Livox header; public methods accept ROS2 `sensor_msgs::msg::PointCloud2::SharedPtr` and `livox_ros_driver2::msg::CustomMsg::SharedPtr`. | Convert preprocessing signatures to ROS1 `sensor_msgs::PointCloud2ConstPtr` and ROS1 `livox_ros_driver2::CustomMsgConstPtr`, or split ROS message decoding into adapter functions and keep core preprocessing on PCL/internal structures. | High |
| `src/core/lio/pointcloud_preprocess.cc` | Uses ROS2 message pointer types and ROS2 timestamp fields (`sec`, `nanosec`). | Use ROS1 message includes and `header.stamp.toSec()` for standard clouds; ROS1 Livox `CustomMsg` should use `std_msgs/Header` stamp. | High |
| `src/core/localization/localization.h` | Includes ROS2 `geometry_msgs/msg/transform_stamped.hpp`, `std_msgs/msg/int32.hpp`; public methods/callbacks use ROS2 message types; commented ROS2 `nav_msgs::msg::Path`/`Odometry`. | Decouple localization core from ROS messages. Prefer callbacks with `NavState`, status enum, and `CloudPtr`; convert to ROS1 `geometry_msgs::TransformStamped`/`std_msgs::Int32` in wrapper. If direct ROS callbacks remain, use ROS1 headers/types. | High |
| `src/core/localization/localization.cpp` | Processes ROS2 cloud/Livox messages; creates `std_msgs::msg::Int32`; commented ROS2 odometry handling. | Move ROS message handling to ROS1 adapter or convert direct uses to ROS1 `sensor_msgs::PointCloud2ConstPtr`, `livox_ros_driver2::CustomMsgConstPtr`, `std_msgs::Int32`. | High |
| `src/core/localization/localization_result.h` | Includes ROS2 `geometry_msgs/msg/transform_stamped.hpp`; `ToGeoMsg()` returns ROS2 type. | Keep core result as `NavState`/SE3 only, or provide a ROS1-specific conversion returning `geometry_msgs::TransformStamped` in wrapper code. | High |
| `src/core/localization/localization_result.cc` | Constructs ROS2 `geometry_msgs::msg::TransformStamped`; uses `math::FromSec` returning `builtin_interfaces::msg::Time`. | Convert to ROS1 `geometry_msgs::TransformStamped` and `ros::Time`, or move conversion outside algorithm core. | High |
| `src/core/localization/lidar_loc/lidar_loc.h` | Includes ROS2 `sensor_msgs/msg/point_cloud2.hpp`, although inspected declarations do not appear to need it directly. | Remove the ROS message include if unused; keep lidar localization on `CloudPtr`/internal types. | Medium |
| `src/core/g2p5/g2p5_map.h` | Includes ROS2 `nav_msgs/msg/occupancy_grid.hpp`; method returns `nav_msgs::msg::OccupancyGrid`. | Keep grid map core independent and expose `cv::Mat`/internal grid metadata. Add ROS1 conversion in wrapper returning `nav_msgs::OccupancyGrid` if needed. | High |
| `src/core/g2p5/g2p5_map.cc` | Constructs ROS2 `nav_msgs::msg::OccupancyGrid` and uses ROS2 generated field typedefs. | Convert to ROS1 `nav_msgs::OccupancyGrid` in adapter or remove ROS dependency from core by returning an internal occupancy grid struct. | High |

## Exact Forbidden Token Matches

These files contain at least one of `rclcpp`, `ament`, `rosidl`, `rosbag2`, `builtin_interfaces`, or `livox_interfaces2`:

- `AGENTS.md`: policy text only; keep as constraints, not a source migration target.
- `CMakeLists.txt`: build-system blocker.
- `package.xml`: build-system blocker.
- `cmake/packages.cmake`: build-system blocker.
- `src/CMakeLists.txt`: build-system blocker.
- `thirdparty/livox_ros_driver/CMakeLists.txt`: build-system blocker.
- `thirdparty/livox_ros_driver/package.xml`: build-system blocker.
- `scripts/merge_bags.py`: ROS2 bag tooling blocker.
- `src/wrapper/bag_io.h`: ROS2 bag/message serialization blocker.
- `src/wrapper/bag_io.cc`: ROS2 bag reader blocker.
- `src/wrapper/ros_utils.h`: ROS2 time utility.
- `src/common/options.h`: ROS2 shutdown in common layer.
- `src/app/run_slam_online.cc`: ROS2 init/shutdown.
- `src/app/run_loc_online.cc`: ROS2 init/shutdown.
- `src/core/system/slam.h`: ROS2 node/subscription/service and message APIs.
- `src/core/system/slam.cc`: ROS2 node/subscription/service runtime code.
- `src/core/system/loc_system.h`: ROS2 node/subscription/tf APIs.
- `src/core/system/loc_system.cc`: ROS2 node/subscription/tf runtime code.
- `src/core/lightning_math.hpp`: ROS2 time type.

## Additional ROS2 Interface Matches Without Forbidden Tokens

These files use ROS2-style `::msg::` or `/msg/` interfaces and should be included in the migration plan even when they do not contain the primary forbidden tokens:

- `src/app/run_slam_offline.cc`
- `src/app/run_loc_offline.cc`
- `src/app/run_loop_offline.cc`
- `src/app/run_frontend_offline.cc`
- `src/core/localization/localization.h`
- `src/core/localization/localization.cpp`
- `src/core/localization/localization_result.h`
- `src/core/localization/localization_result.cc`
- `src/core/localization/lidar_loc/lidar_loc.h`
- `src/core/g2p5/g2p5_map.h`
- `src/core/g2p5/g2p5_map.cc`
- `src/core/lio/laser_mapping.h`
- `src/core/lio/laser_mapping.cc`
- `src/core/lio/pointcloud_preprocess.h`
- `src/core/lio/pointcloud_preprocess.cc`
- `README.md`
- `README_CN.md`
- `scripts/save_default_map.sh`
- `docker/README.md`

## SharedPtr Pattern Check

- Exact requested pattern `std::shared_ptr<...::SharedPtr>`: no matches found.
- Related ROS2 bag pointer found: `src/wrapper/bag_io.h` uses `std::shared_ptr<rosbag2_storage::SerializedBagMessage>`. Replace through the ROS1 `rosbag::MessageInstance` flow.
- Many ROS2 generated message aliases use `...::msg::Type::SharedPtr`; these are listed in the wrapper/app and algorithm-core sections above.

## Recommended Porting Order

1. Convert build system first: root `package.xml`, root `CMakeLists.txt`, `cmake/packages.cmake`, `src/CMakeLists.txt`, and the Livox message package strategy.
2. Replace ROS2 bag IO with ROS1 `rosbag` so offline mapping/localization can be tested without ROS2.
3. Move ROS message handling out of algorithm core where practical, especially `LaserMapping`, `PointCloudPreprocess`, `Localization`, `LocalizationResult`, and `G2P5Map`.
4. Convert online ROS wrappers to ROS1 subscribers, service server, and tf broadcaster.
5. Update scripts and README usage from ROS2 commands/db3 bags to ROS1 commands/bags.
6. Add or update regression tests for mapping, localization, save-map service, evaluation scripts, and wheel-speed fusion.

## Wheel-Speed Fusion Gap

The scan found wheel-speed internals in `src/core/lio/eskf.*`, but no ROS wrapper for `/chassis_feedback`, `yhs_msgs/chassis_feedback`, `rear_right_wheel_speed_mps`, `rear_left_wheel_speed_mps`, or `vehicle_speed_mps` outside `AGENTS.md`.

Porting recommendation:

- Add the ROS1 chassis subscriber in a focused future task, not during broad ROS2 removal.
- Use `yhs_msgs/chassis_feedback` on `/chassis_feedback` at 50 Hz.
- Convert the three required fields into the existing wheel-speed observation path in `ESKF`.
- Add a narrow regression test for the conversion/fusion path.
