你是 Executor。当前阶段只增加结果导出，不改变建图/定位算法。

目标：
为后续无真值评估提供统一数据。

要求：
1. 定位都输出：
   odom_baselink_tf_internal_trajectory_tum.txt
   odom_baselink_tf_external_trajectory_tum.txt
   odom_baselink_topic_external_trajectory_tum.txt
   map_odom_trajectory_tf_internal_trajectory_tum.txt
   map_odom_trajectory_tf_external_trajectory_tum.txt
   map_odom_trajectory_topic_external_trajectory_tum.txt
   map_odom_trajectory_tum.txt
   runtime_metrics.csv
   map_save_info.json
2   建图输出

3. odom_baselink_trajectory_tum.txt  map_odom_trajectory_tum.txt 使用 TUM 格式：
   timestamp tx ty tz qx qy qz qw

   odom_baselink_trajectory_tum.txt 是读取到的
5. runtime_metrics.csv 字段：
   stamp,mode,scan_points,used_points,lio_residual,match_score,update_ms,total_ms,is_degenerate
6. map_save_info.json 字段：
   map_path,global_pcd_path,grid_map_path,keyframe_count,total_distance,total_time
7. 输出目录来自：
   - CLI --output_dir
   - 或 YAML eval.output_dir
8. online/offline 都生效。
9. 如果某些指标当前代码没有，写 NaN，并在注释中说明来源缺失。
10. 不依赖 RTK 或反光板真值。
11. 不改变原有 SLAM 输出结果。

完成后运行：
source /opt/ros/noetic/setup.bash
rm -rf /tmp/lightning_lm_catkin_ws
mkdir -p /tmp/lightning_lm_catkin_ws/src
ln -sfn /home/gitee/lightning_ws /tmp/lightning_lm_catkin_ws/src/lightning_lm
cd /tmp/lightning_lm_catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -j2

输出：
- 新增输出文件说明。
- 每个字段的数据来源。
- 哪些字段可能为 NaN。
- git diff --stat。