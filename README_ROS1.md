# Lightning ROS1 Noetic MID360

This workspace uses ROS1 Noetic and catkin.

## Build

```bash
cd /home/gitee/lightning_ws
source /opt/ros/noetic/setup.bash
catkin build lightning
source devel/setup.bash
```

The default MID360 config used by the scripts is:

```bash
/home/gitee/lightning_ws/src/lightning/config/default_livox.yaml
```

Online runs subscribe to:

- Livox lidar: `/livox/lidar`
- IMU: `/livox/imu`
- Reserved chassis feedback: `/chassis_feedback`

## Online Mapping

```bash
cd /home/gitee/lightning_ws/src/lightning
bash scripts/run_slam_online_mid360.sh
```

Override the config if needed:

```bash
bash scripts/run_slam_online_mid360.sh --config /path/to/config.yaml
```

Save the map while online SLAM is running:

```bash
source /home/gitee/lightning_ws/devel/setup.bash
rosservice call /lightning/save_map "map_id: 'new_map'"
```

## Offline Mapping

```bash
cd /home/gitee/lightning_ws/src/lightning
bash scripts/run_slam_offline_mid360.sh --input_bag /path/to/input.bag
```

or:

```bash
bash scripts/run_slam_offline_mid360.sh /path/to/input.bag
```

## Online Localization

```bash
cd /home/gitee/lightning_ws/src/lightning
bash scripts/run_loc_online_mid360.sh
```

Override the config if needed:

```bash
bash scripts/run_loc_online_mid360.sh --config /path/to/config.yaml
```

## Offline Localization

```bash
cd /home/gitee/lightning_ws/src/lightning
bash scripts/run_loc_offline_mid360.sh --input_bag /path/to/input.bag --map_path ./data/new_map/
```

or:

```bash
bash scripts/run_loc_offline_mid360.sh /path/to/input.bag
```

## Logs

Each script saves command-line output to:

```bash
/home/gitee/lightning_ws/logs/runtime/<program>_<YYYYmmdd_HHMMSS>.log
```

The same output is also printed to the terminal.
