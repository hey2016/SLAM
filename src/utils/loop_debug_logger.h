//
// Auditable loop-closing debug logs.
//

#ifndef LIGHTNING_LOOP_DEBUG_LOGGER_H
#define LIGHTNING_LOOP_DEBUG_LOGGER_H

#include <fstream>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

namespace lightning {

class LoopDebugLogger {
   public:
    struct Options {
        bool enable = false;
        std::string debug_log_dir;
        bool log_candidates = true;
        bool log_matches = true;
        bool log_gate_decisions = true;
        bool log_pgo_impact = true;
        int flush_every_n = 1;
        int max_suspects = 100;
        bool save_figures = true;
        bool save_submap_preview = false;

        int loop_kf_gap = 20;
        int min_id_interval = 20;
        int closest_id_th = 50;
        double max_range = 30.0;
        double ndt_score_th = 1.0;
        bool lidar_auto_ndt_inlier_gate_enable = false;
        double ndt_inlier_max_dist_m = 0.5;
        double ndt_inlier_ratio_th = 0.35;
        double rk_loop_th = 5.2 / 5.0;
        bool with_height = true;
        double height_noise = 0.1;
        bool lidar_auto_candidate_cluster_enable = false;
        int hist_cluster_id_gap = 20;
        double hist_cluster_radius_m = 5.0;
        int keep_per_cluster = 1;
        bool lidar_auto_same_curr_kf_nms_enable = true;
        int same_curr_kf_keep_top = 1;
        bool same_curr_kf_fallback_enable = true;
        int same_curr_kf_fallback_top_k = 3;
        bool lidar_auto_source_accumulation_enable = false;
        int source_accumulation_frame_count = 3;
        int source_accumulation_min_frames = 2;
        double source_accumulation_max_time_span_sec = 1.0;
        double source_accumulation_voxel_leaf_size_m = 0.0;
        bool source_accumulation_debug_log = false;
        bool lidar_auto_init_to_ndt_gate_enable = false;
        double init_to_ndt_max_xy_m = 1.5;
        double init_to_ndt_max_yaw_deg = 8.0;
        double init_to_ndt_max_z_m = 1.0;
        bool lidar_auto_pgo_trial_commit_enable = true;
        bool lidar_auto_risk_combo_gate_enable = false;
        bool risk_combo_reject_low_score_margin_and_large_correction = true;
        bool risk_combo_reject_low_score_margin_and_near_max_range = true;
        bool risk_combo_reject_low_score_margin_and_local_pose_delta_large = true;
        bool risk_combo_reject_large_correction_and_near_max_range = true;
        double risk_low_score_margin_th = 0.3;
        double risk_near_max_range_ratio = 0.8;
        double risk_large_correction_trans_m = 0.8;
        double risk_large_correction_yaw_deg = 10.0;
        double risk_local_pose_delta_large_m = 0.3;
        bool lidar_auto_pgo_impact_gate_enable = false;
        double pgo_impact_max_pose_delta_near_loop_m = 3.0;
        double pgo_impact_max_local_straightness_delta_m = 3.0;
        int pgo_impact_max_affected_kf_count = 120;
        bool pgo_impact_apply_to_auto_lidar_loop_only = true;
        bool lidar_auto_adjacent_pose_gate_enable = true;
        bool adjacent_pose_gate_reject_on_violation = true;
        std::string adjacent_pose_gate_scope = "between_loop";
        int adjacent_pose_gate_max_pair_id_gap = 5;
        int adjacent_pose_gate_min_pair_count = 5;
        double adjacent_pose_gate_max_delta_xy_m = 0.35;
        double adjacent_pose_gate_max_delta_yaw_deg = 5.0;
        double adjacent_pose_gate_p95_delta_xy_m = 0.15;
        double adjacent_pose_gate_p95_delta_yaw_deg = 2.5;
        int adjacent_pose_gate_log_top_k = 5;
        bool adjacent_shape_deformation_enable = true;
        bool adjacent_shape_deformation_reject_on_violation = false;
        std::string adjacent_shape_align_mode = "se2_umeyama_no_scale";
        std::string adjacent_shape_scope = "near_loop";
        int adjacent_shape_min_pose_count = 20;
        double adjacent_shape_min_path_length_m = 20.0;
        double adjacent_shape_endpoint_radius_m = 60.0;
        double adjacent_shape_local_window_path_m = 30.0;
        double adjacent_shape_local_window_stride_m = 10.0;
        int adjacent_shape_max_windows_per_endpoint = 8;
        double adjacent_shape_max_delta_p95_m = 0.35;
        double adjacent_shape_max_delta_max_m = 0.80;
        double adjacent_shape_max_delta_mean_m = 0.20;
    };

    struct KeyframeRow {
        unsigned long kf_id = 0;
        double stamp = 0.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double roll_deg = 0.0;
        double pitch_deg = 0.0;
        double yaw_deg = 0.0;
        double delta_trans_m = std::numeric_limits<double>::quiet_NaN();
        double delta_yaw_deg = std::numeric_limits<double>::quiet_NaN();
        long point_count = 0;
    };

    struct CandidateRow {
        unsigned long curr_kf_id = 0;
        unsigned long hist_kf_id = 0;
        double curr_stamp = 0.0;
        double hist_stamp = 0.0;
        long id_gap = 0;
        double curr_x = 0.0;
        double curr_y = 0.0;
        double curr_z = 0.0;
        double curr_yaw_deg = 0.0;
        double hist_x = 0.0;
        double hist_y = 0.0;
        double hist_z = 0.0;
        double hist_yaw_deg = 0.0;
        double xy_dist_m = std::numeric_limits<double>::quiet_NaN();
        double z_diff_m = std::numeric_limits<double>::quiet_NaN();
        double yaw_diff_deg = std::numeric_limits<double>::quiet_NaN();
        int candidate_rank = -1;
        std::string candidate_source = "loop_scan";
        bool pass_id_gap = false;
        bool pass_closest_id = false;
        bool pass_range = false;
        double range_threshold_m = std::numeric_limits<double>::quiet_NaN();
        int submap_kf_count = 0;
        std::string submap_mode = "history_window_40_step4_world";
    };

    struct MatchRow {
        unsigned long curr_kf_id = 0;
        unsigned long hist_kf_id = 0;
        std::string match_method = "NDT";
        bool ndt_converged = false;
        int ndt_iter = -1;
        double ndt_score = std::numeric_limits<double>::quiet_NaN();
        double ndt_score_threshold = std::numeric_limits<double>::quiet_NaN();
        double fitness_score = std::numeric_limits<double>::quiet_NaN();
        long source_points = 0;
        long target_points = 0;
        double overlap_ratio = std::numeric_limits<double>::quiet_NaN();
        double inlier_ratio = std::numeric_limits<double>::quiet_NaN();
        double correction_trans_m = std::numeric_limits<double>::quiet_NaN();
        double correction_yaw_deg = std::numeric_limits<double>::quiet_NaN();
        double result_dx = std::numeric_limits<double>::quiet_NaN();
        double result_dy = std::numeric_limits<double>::quiet_NaN();
        double result_dz = std::numeric_limits<double>::quiet_NaN();
        double result_dyaw_deg = std::numeric_limits<double>::quiet_NaN();
        double match_time_ms = std::numeric_limits<double>::quiet_NaN();
        bool source_accum_enabled = false;
        bool source_accum_used = false;
        int source_accum_frames = 0;
        long source_accum_raw_points = 0;
        std::string source_accum_fallback_reason;
        std::string source_type = "SINGLE_FRAME";
        int source_scan_count = 0;
        double source_time_span_sec = 0.0;
        long source_points_before_downsample = 0;
        long source_points_after_downsample = 0;
        bool source_accum_hard_fail = false;
    };

    struct SourceAccumRow {
        unsigned long curr_kf_id = 0;
        unsigned long hist_kf_id = 0;
        bool enabled = false;
        bool used = false;
        int configured_frame_count = 3;
        int configured_min_frames = 2;
        double configured_max_time_span_sec = 1.0;
        double configured_voxel_leaf_size_m = 0.0;
        int used_frames = 0;
        long raw_points = 0;
        long source_points = 0;
        std::string fallback_reason;
        std::string source_type = "SINGLE_FRAME";
        int source_scan_count = 0;
        double source_time_span_sec = 0.0;
        long source_points_before_downsample = 0;
        long source_points_after_downsample = 0;
        bool source_accum_hard_fail = false;
    };

    struct GateRow {
        unsigned long curr_kf_id = 0;
        unsigned long hist_kf_id = 0;
        bool gate_id_gap_pass = false;
        bool gate_range_pass = false;
        bool gate_yaw_pass = true;
        bool gate_z_pass = true;
        bool gate_ndt_score_pass = false;
        bool gate_ndt_inlier_pass = true;
        double ndt_inlier_ratio = std::numeric_limits<double>::quiet_NaN();
        double ndt_inlier_ratio_threshold = std::numeric_limits<double>::quiet_NaN();
        bool gate_correction_pass = true;
        bool gate_nms_pass = true;
        bool gate_pgo_chi2_pass = false;
        std::string final_status = "candidate_only";
        std::string reject_reason_primary;
        std::string reject_reason_secondary;
        std::string risk_flags;
        double risk_score = 0.0;
        bool risk_combo_gate_enable = false;
        std::string risk_combo_gate_result;
        std::string risk_combo_reject_reason;
        bool pgo_impact_gate_enable = false;
        double max_pose_delta_near_loop_m = std::numeric_limits<double>::quiet_NaN();
        double mean_pose_delta_near_loop_m = std::numeric_limits<double>::quiet_NaN();
        long affected_kf_count = -1;
        double local_straightness_delta = std::numeric_limits<double>::quiet_NaN();
        std::string pgo_impact_gate_result;
        std::string pgo_impact_reject_reason;
        bool adjacent_pose_gate_enable = false;
        bool adjacent_pose_gate_reject_on_violation = true;
        long adjacent_pair_count = -1;
        double adjacent_max_delta_xy_m = std::numeric_limits<double>::quiet_NaN();
        double adjacent_mean_delta_xy_m = std::numeric_limits<double>::quiet_NaN();
        double adjacent_p95_delta_xy_m = std::numeric_limits<double>::quiet_NaN();
        double adjacent_max_delta_yaw_deg = std::numeric_limits<double>::quiet_NaN();
        double adjacent_mean_delta_yaw_deg = std::numeric_limits<double>::quiet_NaN();
        double adjacent_p95_delta_yaw_deg = std::numeric_limits<double>::quiet_NaN();
        double adjacent_max_delta_z_m = std::numeric_limits<double>::quiet_NaN();
        double adjacent_max_delta_trans_m = std::numeric_limits<double>::quiet_NaN();
        int adjacent_worst_xy_pair_from = -1;
        int adjacent_worst_xy_pair_to = -1;
        int adjacent_worst_yaw_pair_from = -1;
        int adjacent_worst_yaw_pair_to = -1;
        std::string adjacent_pose_gate_result;
        std::string adjacent_pose_gate_reject_reason;
        std::string adjacent_top_delta_pairs;
        bool shape_deformation_enable = false;
        bool shape_deformation_reject_on_violation = false;
        bool shape_valid = false;
        int shape_pose_count = -1;
        double shape_path_length_before_m = std::numeric_limits<double>::quiet_NaN();
        double shape_delta_max_m = std::numeric_limits<double>::quiet_NaN();
        double shape_delta_p95_m = std::numeric_limits<double>::quiet_NaN();
        double shape_delta_mean_m = std::numeric_limits<double>::quiet_NaN();
        int shape_worst_kf_id = -1;
        std::string shape_scope;
        int shape_local_window_count = -1;
        int shape_local_valid_window_count = -1;
        double shape_local_max_delta_max_m = std::numeric_limits<double>::quiet_NaN();
        double shape_local_max_delta_p95_m = std::numeric_limits<double>::quiet_NaN();
        double shape_local_max_delta_mean_m = std::numeric_limits<double>::quiet_NaN();
        std::string shape_worst_endpoint;
        int shape_worst_window_start_kf_id = -1;
        int shape_worst_window_end_kf_id = -1;
        std::string shape_gate_result;
        std::string shape_gate_reject_reason;
        bool committed = false;
        bool pose_writeback = false;
        bool edge_committed = false;
        int curr_kf_candidate_count = -1;
        bool selected_for_pgo_trial = false;
        bool suppressed_by_same_curr_kf_nms = false;
        int candidate_rank = -1;
    };

    struct CandidateClusterRow {
        unsigned long curr_kf_id = 0;
        int raw_candidate_count = 0;
        int clustered_candidate_count = 0;
        int cluster_id = -1;
        unsigned long hist_kf_id = 0;
        double hist_pose_x = std::numeric_limits<double>::quiet_NaN();
        double hist_pose_y = std::numeric_limits<double>::quiet_NaN();
        std::string selected_or_suppressed;
        std::string suppress_reason;
    };

    struct InitToNdtRow {
        unsigned long curr_kf_id = 0;
        unsigned long hist_kf_id = 0;
        double ndt_score = std::numeric_limits<double>::quiet_NaN();
        bool converged = false;
        double init_to_ndt_xy = std::numeric_limits<double>::quiet_NaN();
        double init_to_ndt_yaw_deg = std::numeric_limits<double>::quiet_NaN();
        double init_to_ndt_z = std::numeric_limits<double>::quiet_NaN();
        bool accepted = false;
        std::string reject_reason;
    };

    struct EdgeRow {
        unsigned long curr_kf_id = 0;
        unsigned long hist_kf_id = 0;
        double dx = std::numeric_limits<double>::quiet_NaN();
        double dy = std::numeric_limits<double>::quiet_NaN();
        double dz = std::numeric_limits<double>::quiet_NaN();
        double dyaw_deg = std::numeric_limits<double>::quiet_NaN();
        std::string information_diag;
        double loop_chi2 = std::numeric_limits<double>::quiet_NaN();
        double rk_loop_th = std::numeric_limits<double>::quiet_NaN();
        bool accepted = false;
        bool pose_writeback = false;
        bool edge_committed = false;
    };

    struct PgoImpactRow {
        unsigned long curr_kf_id = 0;
        unsigned long hist_kf_id = 0;
        double loop_chi2_before = std::numeric_limits<double>::quiet_NaN();
        double loop_chi2_after = std::numeric_limits<double>::quiet_NaN();
        double rk_loop_th = std::numeric_limits<double>::quiet_NaN();
        double pgo_error_before = std::numeric_limits<double>::quiet_NaN();
        double pgo_error_after = std::numeric_limits<double>::quiet_NaN();
        double pgo_error_delta = std::numeric_limits<double>::quiet_NaN();
        double max_pose_delta_all_m = std::numeric_limits<double>::quiet_NaN();
        double max_pose_delta_all_yaw_deg = std::numeric_limits<double>::quiet_NaN();
        double max_pose_delta_near_loop_m = std::numeric_limits<double>::quiet_NaN();
        double max_pose_delta_near_loop_yaw_deg = std::numeric_limits<double>::quiet_NaN();
        double mean_pose_delta_near_loop_m = std::numeric_limits<double>::quiet_NaN();
        long affected_kf_count = 0;
        double start_end_error_before_m = std::numeric_limits<double>::quiet_NaN();
        double start_end_error_after_m = std::numeric_limits<double>::quiet_NaN();
        double local_straightness_before = std::numeric_limits<double>::quiet_NaN();
        double local_straightness_after = std::numeric_limits<double>::quiet_NaN();
        double local_straightness_delta = std::numeric_limits<double>::quiet_NaN();
        bool pgo_impact_gate_enable = false;
        std::string pgo_impact_gate_result;
        std::string pgo_impact_reject_reason;
        bool adjacent_pose_gate_enable = false;
        bool adjacent_pose_gate_reject_on_violation = true;
        long adjacent_pair_count = -1;
        double adjacent_max_delta_xy_m = std::numeric_limits<double>::quiet_NaN();
        double adjacent_mean_delta_xy_m = std::numeric_limits<double>::quiet_NaN();
        double adjacent_p95_delta_xy_m = std::numeric_limits<double>::quiet_NaN();
        double adjacent_max_delta_yaw_deg = std::numeric_limits<double>::quiet_NaN();
        double adjacent_mean_delta_yaw_deg = std::numeric_limits<double>::quiet_NaN();
        double adjacent_p95_delta_yaw_deg = std::numeric_limits<double>::quiet_NaN();
        double adjacent_max_delta_z_m = std::numeric_limits<double>::quiet_NaN();
        double adjacent_max_delta_trans_m = std::numeric_limits<double>::quiet_NaN();
        int adjacent_worst_xy_pair_from = -1;
        int adjacent_worst_xy_pair_to = -1;
        int adjacent_worst_yaw_pair_from = -1;
        int adjacent_worst_yaw_pair_to = -1;
        std::string adjacent_pose_gate_result;
        std::string adjacent_pose_gate_reject_reason;
        std::string adjacent_top_delta_pairs;
        bool shape_deformation_enable = false;
        bool shape_deformation_reject_on_violation = false;
        bool shape_valid = false;
        int shape_pose_count = -1;
        double shape_path_length_before_m = std::numeric_limits<double>::quiet_NaN();
        double shape_delta_max_m = std::numeric_limits<double>::quiet_NaN();
        double shape_delta_p95_m = std::numeric_limits<double>::quiet_NaN();
        double shape_delta_mean_m = std::numeric_limits<double>::quiet_NaN();
        int shape_worst_kf_id = -1;
        std::string shape_scope;
        int shape_local_window_count = -1;
        int shape_local_valid_window_count = -1;
        double shape_local_max_delta_max_m = std::numeric_limits<double>::quiet_NaN();
        double shape_local_max_delta_p95_m = std::numeric_limits<double>::quiet_NaN();
        double shape_local_max_delta_mean_m = std::numeric_limits<double>::quiet_NaN();
        std::string shape_worst_endpoint;
        int shape_worst_window_start_kf_id = -1;
        int shape_worst_window_end_kf_id = -1;
        std::string shape_gate_result;
        std::string shape_gate_reject_reason;
        bool pose_writeback = false;
    };

    bool Init(const std::string& config_path, const Options& options, const std::string& run_prefix);
    bool Enabled() const { return enabled_; }
    std::string OutputDir() const { return output_dir_; }

    void WriteKeyframe(const KeyframeRow& row);
    void WriteCandidate(const CandidateRow& row);
    void WriteMatch(const MatchRow& row);
    void WriteSourceAccum(const SourceAccumRow& row);
    void WriteGateDecision(const GateRow& row);
    void WriteCandidateCluster(const CandidateClusterRow& row);
    void WriteInitToNdt(const InitToNdtRow& row);
    void WriteEdge(const EdgeRow& row);
    void WritePgoImpact(const PgoImpactRow& row);
    void WriteSuspect(const CandidateRow& candidate, const MatchRow& match, const EdgeRow& edge, const PgoImpactRow& pgo,
                      const GateRow& gate);
    void Finish();

   private:
    static std::string CsvEscape(const std::string& value);
    static std::string JsonEscape(const std::string& value);
    static std::string FormatDouble(double value);
    static std::string FormatBool(bool value);
    static std::string FormatLong(long value);
    void FlushIfNeeded(std::ofstream& stream, int& counter);
    void WriteMetadata(const std::string& config_path, const Options& options, const std::string& run_prefix);
    void WriteSummary();

    mutable std::mutex mutex_;
    bool enabled_ = false;
    Options options_;
    std::string run_id_;
    std::string output_dir_;
    double start_wall_time_ = 0.0;

    std::ofstream keyframes_;
    std::ofstream candidates_;
    std::ofstream matches_;
    std::ofstream source_accum_;
    std::ofstream gates_;
    std::ofstream candidate_clusters_;
    std::ofstream init_to_ndt_;
    std::ofstream edges_;
    std::ofstream pgo_;
    std::ofstream suspects_;
    int keyframes_since_flush_ = 0;
    int candidates_since_flush_ = 0;
    int matches_since_flush_ = 0;
    int source_accum_since_flush_ = 0;
    int gates_since_flush_ = 0;
    int candidate_clusters_since_flush_ = 0;
    int init_to_ndt_since_flush_ = 0;
    int edges_since_flush_ = 0;
    int pgo_since_flush_ = 0;
    int suspects_since_flush_ = 0;

    long keyframe_count_ = 0;
    long candidate_count_ = 0;
    long match_count_ = 0;
    long accepted_count_ = 0;
    long rejected_count_ = 0;
    long suspect_count_ = 0;
};

}  // namespace lightning

#endif  // LIGHTNING_LOOP_DEBUG_LOGGER_H
