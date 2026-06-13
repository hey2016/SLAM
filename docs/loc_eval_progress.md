# Localization Evaluation Progress

## Current Status

- ROS1 evaluation output is available only when `system.evaluation: true`.
- Localization writes `loc_evaluation.csv`, `loc_trajectory.tum`, `loc_map_quality_input.yaml`, and `loc_evaluation_report.md`.
- The report records that RTK or external ground truth is unavailable, so ATE and RPE are not computed.
- PGO error columns are left empty until a real optimization metric is available.

## Dry Run

Use the offline SLAM MID360 script to validate the evaluation output file shape without processing a bag:

```bash
bash scripts/run_slam_offline_mid360.sh --dry_run
```
