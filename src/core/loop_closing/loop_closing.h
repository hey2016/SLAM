//
// Created by xiang on 25-4-21.
//

#ifndef LIGHTNING_LOOP_CLOSING_H
#define LIGHTNING_LOOP_CLOSING_H

#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "utils/async_message_process.h"
#include "utils/loop_debug_logger.h"

#include "core/graph/optimizer.h"
#include "core/types/edge_se3.h"

#include <limits>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lightning {

/**
 * 基于grid ndt的回环检测
 */
class LoopClosing {
   public:
    struct Options {
        Options() {}

        bool verbose_ = true;       // 输出调试信息
        bool online_mode_ = false;  // 切换离线-在线模式

        int loop_kf_gap_ = 20;       // 每隔多少个关键帧检查一次
        int min_id_interval_ = 20;   // 被检查的关键帧ID间隔
        int closest_id_th_ = 50;     // 历史关键帧与当前帧的ID间隔
        double max_range_ = 30.0;    // 候选帧的最大距离
        double ndt_score_th_ = 1.0;  // ndt位姿分值
        bool lidar_auto_ndt_inlier_gate_enable_ = false;
        double ndt_inlier_max_dist_m_ = 0.5;
        double ndt_inlier_ratio_th_ = 0.35;

        /// 图优化权重
        double motion_trans_noise_ = 0.1;               // 位移权重
        double motion_rot_noise_ = 3.0 * M_PI / 180.0;  // 旋转权重

        double loop_trans_noise_ = 0.2;               // 位移权重
        double loop_rot_noise_ = 3.0 * M_PI / 180.0;  // 旋转权重

        double rk_loop_th_ = 5.2 / 5;  // 回环的RK阈值

        bool lidar_auto_candidate_cluster_enable_ = false;
        int hist_cluster_id_gap_ = 20;
        double hist_cluster_radius_m_ = 5.0;
        int keep_per_cluster_ = 1;
        bool lidar_auto_same_curr_kf_nms_enable_ = true;
        int same_curr_kf_keep_top_ = 1;
        bool same_curr_kf_fallback_enable_ = true;
        int same_curr_kf_fallback_top_k_ = 3;

        bool lidar_auto_source_accumulation_enable_ = false;
        int source_accumulation_frame_count_ = 3;
        int source_accumulation_min_frames_ = 2;
        double source_accumulation_max_time_span_sec_ = 1.0;
        double source_accumulation_voxel_leaf_size_m_ = 0.0;
        bool source_accumulation_debug_log_ = false;

        bool source_scan_accum_enable_ = false;
        int source_scan_accum_max_scans_ = 5;
        int source_scan_accum_min_scans_ = 3;
        double source_scan_accum_time_sec_ = 0.5;
        std::string source_scan_accum_ref_ = "current";
        std::string source_scan_accum_pose_type_ = "LIO";
        double source_scan_accum_max_trans_m_ = 1.2;
        double source_scan_accum_max_yaw_deg_ = 8.0;
        double source_scan_accum_voxel_leaf_m_ = 0.10;
        int source_scan_accum_max_points_ = 30000;
        int source_scan_accum_min_points_ = 3000;
        double source_scan_accum_max_yaw_rate_degps_ = 30.0;
        double source_scan_accum_max_trans_rate_mps_ = 2.5;
        bool source_scan_accum_require_monotonic_stamp_ = true;
        bool source_scan_accum_fallback_single_ = true;
        bool source_scan_accum_debug_enable_ = false;
        std::string source_scan_accum_debug_csv_ = "loop_source_scan_accum_debug.csv";

        bool lidar_auto_init_to_ndt_gate_enable_ = false;
        double init_to_ndt_max_xy_m_ = 1.5;
        double init_to_ndt_max_yaw_deg_ = 8.0;
        double init_to_ndt_max_z_m_ = 1.0;

        bool lidar_auto_pgo_trial_commit_enable_ = true;
        bool lidar_auto_risk_combo_gate_enable_ = false;
        bool risk_combo_reject_low_score_margin_and_large_correction_ = true;
        bool risk_combo_reject_low_score_margin_and_near_max_range_ = true;
        bool risk_combo_reject_low_score_margin_and_local_pose_delta_large_ = true;
        bool risk_combo_reject_large_correction_and_near_max_range_ = true;
        double risk_low_score_margin_th_ = 0.3;
        double risk_near_max_range_ratio_ = 0.8;
        double risk_large_correction_trans_m_ = 0.8;
        double risk_large_correction_yaw_deg_ = 10.0;
        double risk_local_pose_delta_large_m_ = 0.3;

        bool lidar_auto_pgo_impact_gate_enable_ = false;
        double pgo_impact_max_pose_delta_near_loop_m_ = 3.0;
        double pgo_impact_max_local_straightness_delta_m_ = 3.0;
        int pgo_impact_max_affected_kf_count_ = 120;
        bool pgo_impact_apply_to_auto_lidar_loop_only_ = true;

        bool lidar_auto_adjacent_pose_gate_enable_ = true;
        bool adjacent_pose_gate_reject_on_violation_ = true;
        std::string adjacent_pose_gate_scope_ = "between_loop";
        int adjacent_pose_gate_max_pair_id_gap_ = 5;
        int adjacent_pose_gate_min_pair_count_ = 5;
        double adjacent_pose_gate_max_delta_xy_m_ = 0.35;
        double adjacent_pose_gate_max_delta_yaw_deg_ = 5.0;
        double adjacent_pose_gate_p95_delta_xy_m_ = 0.15;
        double adjacent_pose_gate_p95_delta_yaw_deg_ = 2.5;
        int adjacent_pose_gate_log_top_k_ = 5;
        bool adjacent_shape_deformation_enable_ = true;
        bool adjacent_shape_deformation_reject_on_violation_ = false;
        std::string adjacent_shape_align_mode_ = "se2_umeyama_no_scale";
        std::string adjacent_shape_scope_ = "near_loop";
        int adjacent_shape_min_pose_count_ = 20;
        double adjacent_shape_min_path_length_m_ = 20.0;
        double adjacent_shape_endpoint_radius_m_ = 60.0;
        double adjacent_shape_local_window_path_m_ = 30.0;
        double adjacent_shape_local_window_stride_m_ = 10.0;
        int adjacent_shape_max_windows_per_endpoint_ = 8;
        double adjacent_shape_max_delta_p95_m_ = 0.35;
        double adjacent_shape_max_delta_max_m_ = 0.80;
        double adjacent_shape_max_delta_mean_m_ = 0.20;

        bool with_height_ = true;
        double height_noise_ = 0.1;

        bool debug_log_enable_ = false;
        std::string debug_log_dir_;
        bool debug_log_candidates_ = true;
        bool debug_log_matches_ = true;
        bool debug_log_gate_decisions_ = true;
        bool debug_log_pgo_impact_ = true;
        int debug_flush_every_n_ = 1;
        int debug_max_suspects_ = 100;
        bool debug_save_figures_ = true;
        bool debug_save_submap_preview_ = false;
        bool debug_loop_alignment_live_enable_ = false;
        std::string debug_loop_alignment_live_dir_ = "loop_alignment_live";
        int debug_loop_alignment_live_max_points_ = 60000;
        bool debug_loop_alignment_live_save_points_ = true;
        bool debug_loop_alignment_live_save_png_ = true;
        int debug_loop_alignment_live_max_events_ = 0;
    };

    LoopClosing(Options options = Options()) { options_ = options; }
    ~LoopClosing();

    void Init(const std::string yaml_path);

    /// 向回环中添加一个关键帧
    void AddKF(Keyframe::Ptr kf);

    /// 如果检测到新地回环并发生了优化，则调用回调
    using LoopClosedCallback = std::function<void()>;
    void SetLoopClosedCB(LoopClosedCallback cb) { loop_cb_ = cb; }

    struct EvaluationInfo {
        Keyframe::Ptr keyframe;
        uint64_t loop_candidate_id = 0;
        bool loop_accepted = false;
        double loop_score = 0.0;
        double pgo_before_error = std::numeric_limits<double>::quiet_NaN();
        double pgo_after_error = std::numeric_limits<double>::quiet_NaN();
        std::string loop_status;
        std::string loop_reject_reason;
        double loop_chi2 = std::numeric_limits<double>::quiet_NaN();
        double loop_robust_delta = std::numeric_limits<double>::quiet_NaN();
    };
    using EvaluationCallback = std::function<void(const EvaluationInfo&)>;
    void SetEvaluationCallback(EvaluationCallback cb) { evaluation_cb_ = std::move(cb); }

    struct SourceScanAccumProviderResult {
        bool success = false;
        CloudPtr cloud_lidar = nullptr;
        int scan_count = 0;
        double time_span_sec = 0.0;
        long points_before_downsample = 0;
        long points_after_downsample = 0;
        std::string fallback_reason;
    };
    using SourceScanAccumProvider =
        std::function<bool(double curr_stamp_sec, const SE3& T_w_curr, SourceScanAccumProviderResult* out)>;
    void SetSourceScanAccumProvider(SourceScanAccumProvider provider) {
        source_scan_accum_provider_ = std::move(provider);
    }

    struct AdjacentPoseDeformationStats {
        bool valid = false;
        int pair_count = 0;

        double max_delta_xy_m = 0.0;
        double mean_delta_xy_m = 0.0;
        double p95_delta_xy_m = 0.0;

        double max_delta_yaw_deg = 0.0;
        double mean_delta_yaw_deg = 0.0;
        double p95_delta_yaw_deg = 0.0;

        double max_delta_z_m = 0.0;
        double max_delta_trans_m = 0.0;

        int worst_xy_pair_from = -1;
        int worst_xy_pair_to = -1;
        int worst_yaw_pair_from = -1;
        int worst_yaw_pair_to = -1;

        std::string gate_result = "not_evaluated";
        std::string reject_reason;
        std::string top_delta_pairs;

        bool shape_valid = false;
        int shape_pose_count = 0;
        double shape_path_length_before_m = 0.0;
        double shape_delta_max_m = 0.0;
        double shape_delta_p95_m = 0.0;
        double shape_delta_mean_m = 0.0;
        int shape_worst_kf_id = -1;
        std::string shape_scope = "near_loop";
        int shape_local_window_count = 0;
        int shape_local_valid_window_count = 0;
        double shape_local_max_delta_max_m = 0.0;
        double shape_local_max_delta_p95_m = 0.0;
        double shape_local_max_delta_mean_m = 0.0;
        std::string shape_worst_endpoint;
        int shape_worst_window_start_kf_id = -1;
        int shape_worst_window_end_kf_id = -1;
        std::string shape_gate_result = "not_evaluated";
        std::string shape_gate_reject_reason;
    };

    static AdjacentPoseDeformationStats ComputeAdjacentPoseDeformationStats(
        const std::unordered_map<unsigned long, SE3>& poses_before,
        const std::unordered_map<unsigned long, SE3>& poses_after,
        unsigned long hist_kf_id, unsigned long curr_kf_id, const Options& options);
    static bool EvaluateAdjacentPoseGate(const Options& options, AdjacentPoseDeformationStats* stats);

   protected:
    void HandleKF(Keyframe::Ptr kf);

    void DetectLoopCandidates();
    void ClusterLoopCandidates();

    /// 计算回环候选位姿
    void ComputeLoopCandidates();

    /// 计算单个回环候选
    void ComputeForCandidate(LoopCandidate& c);

    CloudPtr BuildNdtSourceCloud(const Keyframe::Ptr& curr_kf, LoopCandidate& c);
    void ExportLoopAlignmentLiveDebug(const LoopCandidate& c, const CloudPtr& target_world,
                                      const CloudPtr& source_lidar, const SE3& T_w_hist,
                                      const SE3& T_w_source_initial, const SE3& T_w_source_ndt);

    /// 优化位姿
    void PoseOptimization();

    void LogKeyframe(const Keyframe::Ptr& kf);
    void LogCandidateGate(const Keyframe::Ptr& hist_kf, bool pass_id_gap, bool pass_closest_id, bool pass_range,
                          int candidate_rank);
    void LogGateDecision(const LoopCandidate& c, const std::string& final_status,
                         const std::string& reject_primary, const std::string& reject_secondary = "",
                         bool pgo_pass = false, const std::string& risk_flags = "", double risk_score = 0.0,
                         bool nms_pass = true, bool correction_pass = true, bool pose_writeback = false,
                         bool edge_committed = false, const std::string& risk_combo_gate_result = "",
                         const std::string& risk_combo_reject_reason = "",
                         const std::string& pgo_impact_gate_result = "",
                         const std::string& pgo_impact_reject_reason = "",
                         double max_pose_delta_near_loop_m = std::numeric_limits<double>::quiet_NaN(),
                         double mean_pose_delta_near_loop_m = std::numeric_limits<double>::quiet_NaN(),
                         long affected_kf_count = -1,
                         double local_straightness_delta = std::numeric_limits<double>::quiet_NaN(),
                         int curr_kf_candidate_count = -1,
                         bool selected_for_pgo_trial = false,
                         bool suppressed_by_same_curr_kf_nms = false,
                         const AdjacentPoseDeformationStats* adjacent_pose_stats = nullptr);

    Options options_;

    Keyframe::Ptr last_kf_ = nullptr;
    Keyframe::Ptr last_loop_kf_ = nullptr;
    Keyframe::Ptr cur_kf_ = nullptr;
    std::vector<Keyframe::Ptr> all_keyframes_;
    std::vector<LoopCandidate> candidates_;

    AsyncMessageProcess<Keyframe::Ptr> kf_thread_;

    std::shared_ptr<miao::Optimizer> optimizer_ = nullptr;

    Mat6d info_motion_ = Mat6d::Identity();  // 关键帧间的运动信息阵
    Mat6d info_loops_ = Mat6d::Identity();   // 回环帧的信息矩阵

    std::vector<std::shared_ptr<miao::VertexSE3>> kf_vert_;
    std::vector<std::shared_ptr<miao::EdgeSE3>> edge_loops_;

    LoopClosedCallback loop_cb_;
    EvaluationCallback evaluation_cb_;
    SourceScanAccumProvider source_scan_accum_provider_;
    std::unique_ptr<LoopDebugLogger> loop_debug_logger_;
    uint64_t loop_alignment_event_id_ = 0;
    bool loop_alignment_events_header_written_ = false;
};

}  // namespace lightning

#endif  // LIGHTNING_LOOP_CLOSING_H
