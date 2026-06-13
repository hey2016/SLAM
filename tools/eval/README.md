# Evaluation Outputs

Lightning writes evaluation files when `system.evaluation` is `true` and `system.evaluation_output_dir` is set in the active YAML config.

Default:

```yaml
system:
  evaluation: false
  evaluation_output_dir: ./data/evaluation
```

Enable output for an evaluation run by setting `system.evaluation: true`.

## Build

```bash
cd /home/gitee/lightning_ws
source /opt/ros/noetic/setup.bash
catkin build lightning
source devel/setup.bash
```

## Generate Mapping Outputs

Online MID360 mapping:

```bash
cd /home/gitee/lightning_ws/src/lightning
bash scripts/run_slam_online_mid360.sh
```

Save the map while online mapping is running:

```bash
source /home/gitee/lightning_ws/devel/setup.bash
rosservice call /lightning/save_map "map_id: 'new_map'"
```

Offline MID360 mapping:

```bash
cd /home/gitee/lightning_ws/src/lightning
bash scripts/run_slam_offline_mid360.sh --input_bag /path/to/input.bag
```

Dry-run output generation without reading a bag:

```bash
cd /home/gitee/lightning_ws/src/lightning
bash scripts/run_slam_offline_mid360.sh --dry_run
```

## Generate Localization Outputs

Online MID360 localization:

```bash
cd /home/gitee/lightning_ws/src/lightning
bash scripts/run_loc_online_mid360.sh
```

Offline MID360 localization:

```bash
cd /home/gitee/lightning_ws/src/lightning
bash scripts/run_loc_offline_mid360.sh --input_bag /path/to/input.bag --map_path ./data/new_map/
```

## Files

The output directory contains:

- `slam_evaluation.csv` or `loc_evaluation.csv`
- `slam_trajectory.tum` or `loc_trajectory.tum`
- `slam_map_quality_input.yaml` or `loc_map_quality_input.yaml`
- `slam_evaluation_report.md` or `loc_evaluation_report.md`

CSV columns:

```text
timestamp,pose_x,pose_y,pose_z,roll,pitch,yaw,lidar_odom_score,keyframe_id,loop_candidate_id,loop_accepted,loop_score,pgo_before_error,pgo_after_error
```

TUM trajectory format:

```text
timestamp tx ty tz qx qy qz qw
```

Map-quality input format:

```yaml
global_pcd: ./data/new_map/global.pcd
map_pgm: ./data/new_map/map.pgm
trajectory_tum: ./data/evaluation/slam_trajectory.tum
```

The report explicitly records that RTK or external ground truth is unavailable, so ATE and RPE are not computed.
