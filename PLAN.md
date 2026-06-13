# 回环调试日志与参数评估能力 Planner

## 1. 已读代码与当前链路

### 回环检测主链路

- `src/core/loop_closing/loop_closing.h`
  - `LoopClosing::Options` 当前包含 `loop_kf_gap_`, `min_id_interval_`, `closest_id_th_`, `max_range_`, `ndt_score_th_`, `rk_loop_th_`, `with_height_`。
  - `rk_loop_th_` 默认值为 `5.2 / 5`，但当前未从 yaml 读取。
  - `EvaluationInfo` 当前仅包含候选 id、accepted、NDT score、PGO before/after error、status、reject reason、chi2、robust delta。

- `src/core/loop_closing/loop_closing.cc`
  - `Init()` 只读取：
    - `loop_kf_gap`
    - `min_id_interval`
    - `closest_id_th`
    - `max_range`
    - `ndt_score_th`
    - `with_height`
  - 未读取 `rk_loop_th`、调试日志开关、输出目录或 flush 参数。
  - `HandleKF()` 顺序：
    1. `DetectLoopCandidates()`
    2. `ComputeLoopCandidates()`
    3. `PoseOptimization()`
  - `DetectLoopCandidates()` 候选生成逻辑：
    - 第一个关键帧只设置 `last_loop_kf_`，不生成候选。
    - 若 `cur_kf_id - last_loop_kf_id <= loop_kf_gap`，整帧跳过。
    - 遍历历史关键帧：
      - 若距上一个已检查候选 `check_first` 的 id 间隔 `<= min_id_interval`，跳过。
      - 若历史帧与当前帧 id 间隔 `< closest_id_th`，直接 `break`。
      - 计算优化位姿 XY 距离 `t2d`，仅 `t2d < max_range` 时生成 `LoopCandidate`。
    - 只有至少生成一个候选时才更新 `last_loop_kf_ = cur_kf_`。
  - `ComputeForCandidate()`：
    - 使用候选历史帧附近约 `[-40, 40)`、步长 4 的子图作为 target，当前帧点云作为 source。
    - 依次用 NDT 分辨率 `10.0, 5.0, 2.0, 1.0` 进行粗到细配准。
    - 仅保留最后一级 `ndt.getTransformationProbability()` 到 `c.ndt_score_`。
    - 子图为空或当前点云为空时 `ndt_score_=0`，但当前没有单独 reject reason。
  - `ComputeLoopCandidates()`：
    - `ndt_score > ndt_score_th` 进入 PGO。
    - 否则通过 evaluation callback 记录一条 `rejected_ndt / ndt_score_below_threshold`。
  - `PoseOptimization()`：
    - 给 NDT 通过的候选添加 `EdgeSE3` 回环边。
    - 给回环边设置 Cauchy robust kernel，`delta = options_.rk_loop_th_`。
    - 优化后若 `e->Chi2() > e->GetRobustKernel()->Delta()`，设置 `Level(1)`，作为 outlier。
    - evaluation callback 记录 accepted 或 `rejected_outlier / robust_chi2_above_delta`。

### 现有 evaluation 输出

- `src/utils/evaluation_writer.h/.cc`
  - 输出 `slam_evaluation.csv`, `slam_trajectory.tum`, `slam_map_quality_input.yaml`, `slam_evaluation_report.md`。
  - 当前 CSV 表头：
    - `timestamp`
    - `pose_x,pose_y,pose_z,roll,pitch,yaw`
    - `lidar_odom_score`
    - `keyframe_id`
    - `loop_candidate_id`
    - `loop_accepted`
    - `loop_score`
    - `pgo_before_error`
    - `pgo_after_error`
    - `loop_status`
    - `loop_reject_reason`
    - `loop_chi2`
    - `loop_robust_delta`
  - 只能看到关键帧轨迹和部分 loop 事件，看不到候选生成阶段被 `loop_kf_gap / min_id_interval / closest_id_th / max_range` 排除的对象，也看不到每一级 NDT 分数、子图点数、距离、相对姿态和 gate 明细。

- `src/core/system/slam.cc`
  - `system.evaluation=true` 且 `evaluation_output_dir` 存在时创建 `EvaluationWriter`。
  - 建图每个关键帧通过 `WriteEvaluationKeyframe()` 输出轨迹行。
  - 回环 callback 将 `LoopClosing::EvaluationInfo` 转成 `EvaluationRecord`。

- `scripts/run_slam_offline_mid360.sh`
  - 默认配置是 `config/default_livox.yaml`，不是 `default_mid360_slam.yaml`。
  - 运行时生成 `result_dir/config/effective_config.yaml`，只强制：
    - `system.evaluation = True`
    - `system.evaluation_output_dir = eval_dir`
  - dry-run 也只生成当前旧表头。

### default_mid360_slam.yaml 与 rk_loop_th

- `config/default_mid360_slam.yaml`
  - 当前 `system.evaluation: false`，并非 true。
  - `loop_closing` 下只有：
    - `loop_kf_gap`
    - `min_id_interval`
    - `closest_id_th`
    - `max_range`
    - `ndt_score_th`
    - `with_height`
  - 没有 `rk_loop_th`。

- `rk_loop_th` 未进入当前配置的原因：
  - `LoopClosing::Options` 有默认值，但 `LoopClosing::Init()` 没有读取 yaml 中的 `loop_closing.rk_loop_th`。
  - 所有现有默认 yaml 都没有显式写 `rk_loop_th`。
  - `run_slam_offline_mid360.sh` 的 effective config 生成逻辑只改 evaluation，不补充 loop 参数。
  - 因此实际运行时只能隐式使用 C++ 默认值 `5.2 / 5`，后处理无法从配置文件追溯。

## 2. 最小侵入式改造目标

新增可审计回环调试日志，不改变 `debug_log_enable=false` 时的建图结果、候选选择、NDT、PGO、robust kernel、地图输出和 evaluation 原有语义。

需要记录：

- 每个当前关键帧是否触发候选检测。
- 候选生成阶段所有 gate 的通过或拒绝原因。
- 所有进入 NDT 的 candidate，包括被 NDT score 拒绝的 candidate。
- 所有进入 PGO 的 loop edge，包括 robust chi2 outlier 和 accepted。
- 本次运行使用的 loop 参数快照，包括 `rk_loop_th`。
- 可用于后处理评估 `max_range`, `ndt_score_th`, `closest_id_th`, `rk_loop_th`, NMS/候选抑制策略的完整字段。

## 3. 需要修改的文件

### `config/default_mid360_slam.yaml`

- 在 `loop_closing` 下显式补充：
  - `rk_loop_th: 1.04`
  - `debug_log_enable: false`
  - `debug_csv_enable: true`
  - `debug_jsonl_enable: true`
  - `debug_summary_enable: true`
  - `debug_output_dir: ./data/loop_debug`
  - `debug_flush_every_n: 1`
  - `debug_log_rejected_candidates: true`
  - `debug_save_candidate_clouds: false`
  - `debug_cloud_output_dir: ./data/loop_debug/clouds`
  - `debug_keep_recent_window_sec: 30.0`
  - `debug_write_fault_window_on_event: true`
  - `debug_fault_window_pre_sec: 10.0`
  - `debug_fault_window_post_sec: 10.0`

### `config/default_livox.yaml`

- 同步补充相同 `loop_closing` 配置，因为 `scripts/run_slam_offline_mid360.sh` 当前默认使用该文件。
- 不改变现有阈值默认行为；`debug_log_enable` 默认 false。

### `src/core/loop_closing/loop_closing.h`

- 扩展 `LoopClosing::Options`：
  - 读取并保存 debug 开关、输出目录、flush 策略、cloud 保存开关。
  - 显式读取 `rk_loop_th_`。
- 扩展或新增内部结构：
  - `LoopDebugRecord`
  - `LoopGateSnapshot`
  - `LoopMatchSnapshot`
- 增加私有只读日志成员：
  - `std::unique_ptr<LoopDebugLogger> loop_debug_logger_`
- 不改变 `LoopCandidate` 的核心使用语义；如需增加字段，仅作为诊断字段。

### `src/core/loop_closing/loop_closing.cc`

- `Init()`：
  - 用兼容方式读取 `loop_closing.rk_loop_th`，缺失则保持现有默认 `5.2/5`。
  - 读取 debug 配置。
  - debug 开启时初始化 logger，并写入参数快照。
- `DetectLoopCandidates()`：
  - 记录 frame-level 事件：
    - `FIRST_KEYFRAME`
    - `SKIP_LOOP_KF_GAP`
    - `NO_CANDIDATE`
    - `HAS_CANDIDATE`
  - 对每个历史关键帧记录 gate：
    - `MIN_ID_INTERVAL_REJECT`
    - `CLOSEST_ID_BREAK`
    - `RANGE_REJECT`
    - `CANDIDATE_GENERATED`
  - 记录 `current_kf_id`, `history_kf_id`, `id_delta`, `xy_distance`, `z_delta`, `range_th`, `closest_id_th`, `min_id_interval`。
- `ComputeForCandidate()`：
  - 记录子图点数、当前帧点数。
  - 记录初始相对位姿、每级 NDT resolution、是否收敛、score、final transform。
  - 如果 PCL NDT 无法提供某些字段，写 `NaN` 并在 `extra_json` 标注。
- `ComputeLoopCandidates()`：
  - 对 NDT 拒绝的候选写一条 `REJECTED_NDT_SCORE`。
  - 对进入 PGO 的候选写 `PASSED_NDT_SCORE`。
- `PoseOptimization()`：
  - 对每条 loop edge 写：
    - `PGO_EDGE_ADDED`
    - `ACCEPTED`
    - `REJECTED_ROBUST_CHI2`
  - 记录 `loop_chi2`, `rk_loop_th`, `pgo_before_error`, `pgo_after_error`, `edge_level`。

### 新增 `src/utils/loop_debug_logger.h`

- 定义轻量 logger 接口：
  - `Init(yaml_path, output_dir, run_prefix, options_snapshot)`
  - `Enabled()`
  - `WriteCandidateRecord(record)`
  - `WriteEvent(record)`
  - `WriteSummary()`
  - `Finish()`
- 只做文件 IO 和统计，不参与算法判断。

### 新增 `src/utils/loop_debug_logger.cc`

- 自动创建目录。
- 打开 CSV / JSONL / summary。
- 写文件失败只 `LOG(WARNING)`，不抛异常、不退出。
- 支持 flush_every_n。
- 维护 summary 统计：
  - 总关键帧数
  - 检测触发次数
  - range reject 数
  - closest id reject/break 数
  - NDT reject 数
  - PGO robust reject 数
  - accepted loop 数
  - score 分布、chi2 分布、距离分布

### `src/CMakeLists.txt`

- 将 `utils/loop_debug_logger.cc` 加入 `lightning.libs`。

### `scripts/run_slam_offline_mid360.sh`

- effective config 生成时不强制开启 debug。
- dry-run 表头可补充新 debug 文件占位检查，但不改变原 evaluation 文件名。
- 运行结束后输出 debug 文件路径，如果存在：
  - `loop_candidates_csv=...`
  - `loop_events_jsonl=...`
  - `loop_summary=...`

### 新增 `scripts/check_loop_debug_logs.sh`

- 用于最小验证：
  - 生成临时 effective config，将 `loop_closing.debug_log_enable=true`。
  - 运行短 bag 或 dry-run。
  - 检查 CSV / JSONL / summary 文件存在。
  - 检查 CSV 表头关键字段。
  - 检查 summary 包含 `rk_loop_th`, `ndt_score_th`, `max_range`, accepted/rejected 统计。

## 4. 新增配置项

建议放在 `loop_closing` 下：

```yaml
loop_closing:
  rk_loop_th: 1.04
  debug_log_enable: false
  debug_csv_enable: true
  debug_jsonl_enable: true
  debug_summary_enable: true
  debug_output_dir: ./data/loop_debug
  debug_flush_every_n: 1
  debug_log_rejected_candidates: true
  debug_save_candidate_clouds: false
  debug_cloud_output_dir: ./data/loop_debug/clouds
  debug_keep_recent_window_sec: 30.0
  debug_write_fault_window_on_event: true
  debug_fault_window_pre_sec: 10.0
  debug_fault_window_post_sec: 10.0
```

兼容策略：

- 缺失 `rk_loop_th` 时继续使用 C++ 默认值 `5.2 / 5`。
- 缺失 debug 配置时全部按默认值处理。
- `debug_log_enable=false` 时不创建文件、不保存点云、不改变任何候选或优化结果。

## 5. 新增 CSV 格式

文件：

```text
<debug_output_dir>/loop_candidates_<YYYYMMDD-HHMMSS>-<pid>-slam.csv
```

表头建议：

```text
run_id,
wall_time,
ros_time,
current_kf_id,
history_kf_id,
stage,
event,
candidate_generated,
entered_ndt,
entered_pgo,
accepted,
reject_reason,
id_delta,
last_loop_kf_id,
loop_kf_gap,
min_id_interval,
closest_id_th,
max_range,
xy_distance,
z_delta,
with_height,
ndt_score,
ndt_score_th,
ndt_converged,
ndt_resolution,
ndt_iter,
submap_points,
source_points,
initial_rel_x,
initial_rel_y,
initial_rel_z,
initial_rel_roll_deg,
initial_rel_pitch_deg,
initial_rel_yaw_deg,
matched_rel_x,
matched_rel_y,
matched_rel_z,
matched_rel_roll_deg,
matched_rel_pitch_deg,
matched_rel_yaw_deg,
pgo_before_error,
pgo_after_error,
loop_chi2,
rk_loop_th,
edge_level,
gate_loop_kf_gap,
gate_min_id_interval,
gate_closest_id,
gate_range,
gate_ndt_score,
gate_robust_chi2,
extra_json
```

字段拿不到时：

- `ndt_iter`: 当前 PCL NDT 调用点没有读取迭代次数，先写 `NaN`，TODO：确认 `getFinalNumIteration()` 可用性。
- 每级 NDT 中间 score：可先每个 resolution 写单独行；若只保留最终 score，需要在 `extra_json` 标注 `per_resolution_score_unavailable=false/true`。
- cloud 文件路径：仅 `debug_save_candidate_clouds=true` 时写入 `extra_json`，默认不保存。

## 6. 新增 JSONL 事件格式

文件：

```text
<debug_output_dir>/loop_events_<YYYYMMDD-HHMMSS>-<pid>-slam.jsonl
```

仅写关键事件和异常，不逐帧刷屏。示例：

```json
{
  "run_id": "20260513-xxxx-slam",
  "event_time_ros": 0.0,
  "event_time_wall": 0.0,
  "event_id": 4001,
  "module": "loop_debug",
  "current_kf_id": 128,
  "history_kf_id": 42,
  "status": "REJECTED_NDT_SCORE",
  "severity": "INFO",
  "reason": "ndt_score <= ndt_score_th",
  "xy_distance": 13.2,
  "ndt_score": 1.7,
  "ndt_score_th": 2.2,
  "loop_chi2": null,
  "rk_loop_th": 1.04,
  "suggestion": "inspect repeated structure and tune max_range/ndt_score_th/closest_id_th"
}
```

事件 ID 建议：

- `4001 CANDIDATE_GENERATED`
- `4002 RANGE_REJECTED`
- `4003 CLOSEST_ID_REJECTED_OR_BREAK`
- `4011 NDT_SCORE_REJECTED`
- `4012 NDT_PASSED`
- `4021 PGO_EDGE_ADDED`
- `4022 PGO_ROBUST_REJECTED`
- `4023 LOOP_ACCEPTED`
- `4031 LOOP_KF_GAP_SKIPPED`
- `4099 LOOP_DEBUG_SUMMARY`

## 7. Summary 报告

文件：

```text
<debug_output_dir>/loop_summary_<YYYYMMDD-HHMMSS>-<pid>-slam.md
```

内容：

- 运行信息：run_id、start/end、duration、config path。
- 参数快照：`loop_kf_gap`, `min_id_interval`, `closest_id_th`, `max_range`, `ndt_score_th`, `rk_loop_th`, `with_height`。
- 候选生成统计：总关键帧、检测触发、gap skip、range reject、candidate count。
- NDT 统计：score mean/p50/p95/max、NDT reject/pass 数。
- PGO 统计：edge added、accepted、robust rejected、chi2 mean/p95/max。
- 参数评估建议：
  - 大量 range 内候选但 NDT reject：优先评估 `ndt_score_th` 和重复结构。
  - accepted loop 的 chi2 接近或超过 `rk_loop_th`：优先评估 `rk_loop_th`。
  - 同一通道大量近邻候选：优先评估 `closest_id_th`, `min_id_interval`, NMS。
  - 错误回环集中在高度不同区域：优先评估 `with_height`。
- 明确标注：参数建议需要结合点云、轨迹图和人工复核，不自动改变算法行为。

## 8. 验证命令

静态检查：

```bash
cd /home/gitee/lightning_ws
rg -n "rclcpp|ament|rosidl|rosbag2|ros1_bridge" src/lightning || true
rg -n "rk_loop_th|debug_log_enable|loop_candidates_|loop_events_|loop_summary_" src/lightning
```

编译：

```bash
cd /home/gitee/lightning_ws
catkin_make --build build_catkin_make --force-cmake \
  -DROS_EDITION=ROS1 \
  -DCATKIN_BLACKLIST_PACKAGES=livox_ros_driver2 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_DEVEL_PREFIX=/home/gitee/lightning_ws/devel_catkin_make \
  -j2
```

短 bag 验证：

```bash
cd /home/gitee/lightning_ws
bash src/lightning/scripts/check_loop_debug_logs.sh \
  --bag /home/ZNY_data/test/2026-05-01-19-27-35.bag \
  --config src/lightning/config/default_livox.yaml
```

手工运行：

```bash
cd /home/gitee/lightning_ws
python3 - <<'PY'
import yaml
cfg = "src/lightning/config/default_livox.yaml"
out = "/tmp/loop_debug_enabled.yaml"
data = yaml.safe_load(open(cfg))
data.setdefault("loop_closing", {})["debug_log_enable"] = True
data["loop_closing"]["debug_output_dir"] = "/home/gitee/lightning_ws/data/loop_debug_manual"
yaml.safe_dump(data, open(out, "w"), allow_unicode=True, sort_keys=False)
print(out)
PY
bash src/lightning/scripts/run_slam_offline_mid360.sh \
  --input_bag /home/ZNY_data/test/2026-05-01-19-27-35.bag \
  --config /tmp/loop_debug_enabled.yaml
```

CSV 后处理示例：

```bash
python3 - <<'PY'
import pandas as pd
p = "/home/gitee/lightning_ws/data/loop_debug_manual/loop_candidates_latest.csv"
df = pd.read_csv(p)
print(df.groupby(["stage", "reject_reason"]).size())
print(df[["current_kf_id", "history_kf_id", "xy_distance", "ndt_score", "loop_chi2", "accepted"]].tail())
PY
```

## 9. 风险点

- 回环检测在关键帧线程内执行；CSV 写入必须轻量，禁止保存点云作为默认行为。
- 记录所有 rejected candidate 可能增大日志体积，需由 `debug_log_rejected_candidates` 控制。
- PCL NDT 的迭代次数、收敛标志接口需要以当前编译版本确认；拿不到先写 `NaN`。
- `closest_id_th` 当前逻辑是 `break` 而不是 `continue`，日志必须准确标注为 `CLOSEST_ID_BREAK`，不要误写成普通 reject。
- `last_loop_kf_` 只在生成候选后更新，`loop_kf_gap` 的含义是“距上次有候选的关键帧”，不是“距上次 accepted loop”。
- PGO robust gate 当前比较是 `chi2 > delta`，不是 `chi2 > delta^2`；日志和 summary 必须按现有实现记录，不能自行改公式。

## 10. 回滚方式

- 若只想关闭新功能：将 `loop_closing.debug_log_enable=false`。
- 若需要代码级回滚：
  - 从 `src/CMakeLists.txt` 移除 `utils/loop_debug_logger.cc`。
  - 从 `LoopClosing` 移除 logger 成员和 debug 调用。
  - 保留 `rk_loop_th` 配置读取也不改变旧默认行为；如需完全回滚，可删除 yaml 中新增 debug 配置和 `rk_loop_th`。
- 回滚后原有 `slam_evaluation.csv`、`slam_trajectory.tum` 输出保持不变。
