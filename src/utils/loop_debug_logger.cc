#include "utils/loop_debug_logger.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <unistd.h>

namespace lightning {
namespace {

double WallTimeSec() {
    using Clock = std::chrono::system_clock;
    return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

std::string MakeRunId(const std::string& prefix) {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream os;
    os << std::put_time(&tm, "%Y%m%d-%H%M%S") << "-" << getpid();
    if (!prefix.empty()) os << "-" << prefix;
    return os.str();
}

std::string EnvOrUnknown(const char* name) {
    const char* value = std::getenv(name);
    if (value == nullptr || std::string(value).empty()) {
        return "unknown";
    }
    return value;
}

std::string JsonEscapeLocal(const std::string& value) {
    std::string out;
    for (char ch : value) {
        if (ch == '"' || ch == '\\') {
            out += '\\';
            out += ch;
        } else if (ch == '\n') {
            out += "\\n";
        } else if (ch == '\r') {
            out += "\\r";
        } else if (ch == '\t') {
            out += "\\t";
        } else {
            out += ch;
        }
    }
    return out;
}

template <typename T>
std::string YamlValueOrNa(const YAML::Node& node, const std::string& key) {
    if (!node || !node[key]) return "NaN";
    try {
        std::ostringstream os;
        os << node[key].as<T>();
        return os.str();
    } catch (...) {
        return "NaN";
    }
}

void WriteJsonStringField(std::ostream& os, const std::string& key, const std::string& value, bool comma = true) {
    os << "  \"" << key << "\": \"" << JsonEscapeLocal(value) << "\"";
    if (comma) os << ",";
    os << "\n";
}

}  // namespace

bool LoopDebugLogger::Init(const std::string& config_path, const Options& options, const std::string& run_prefix) {
    std::unique_lock<std::mutex> lock(mutex_);
    options_ = options;
    enabled_ = false;
    if (!options_.enable) {
        return false;
    }

    try {
        run_id_ = MakeRunId(run_prefix);
        start_wall_time_ = WallTimeSec();
        output_dir_ = options_.debug_log_dir.empty() ? "data/loop_debug" : options_.debug_log_dir;
        std::filesystem::create_directories(output_dir_);

        keyframes_.open(output_dir_ + "/keyframes.csv", std::ios::out | std::ios::trunc);
        candidates_.open(output_dir_ + "/loop_candidates.csv", std::ios::out | std::ios::trunc);
        matches_.open(output_dir_ + "/loop_matches.csv", std::ios::out | std::ios::trunc);
        gates_.open(output_dir_ + "/loop_gate_decisions.csv", std::ios::out | std::ios::trunc);
        candidate_clusters_.open(output_dir_ + "/loop_candidate_clusters.csv", std::ios::out | std::ios::trunc);
        init_to_ndt_.open(output_dir_ + "/loop_init_to_ndt_debug.csv", std::ios::out | std::ios::trunc);
        edges_.open(output_dir_ + "/loop_edges.csv", std::ios::out | std::ios::trunc);
        pgo_.open(output_dir_ + "/loop_pgo_impact.csv", std::ios::out | std::ios::trunc);
        suspects_.open(output_dir_ + "/loop_suspects.csv", std::ios::out | std::ios::trunc);
        if (options_.source_scan_accum_debug_enable) {
            const std::string source_accum_csv =
                options_.source_scan_accum_debug_csv.empty() ? "loop_source_scan_accum_debug.csv"
                                                             : options_.source_scan_accum_debug_csv;
            source_accum_.open(output_dir_ + "/" + source_accum_csv, std::ios::out | std::ios::trunc);
        }
        if (!keyframes_.is_open() || !candidates_.is_open() || !matches_.is_open() || !gates_.is_open() ||
            !candidate_clusters_.is_open() || !init_to_ndt_.is_open() || !edges_.is_open() || !pgo_.is_open() ||
            !suspects_.is_open() || (options_.source_scan_accum_debug_enable && !source_accum_.is_open())) {
            LOG(WARNING) << "failed to open loop debug logs under " << output_dir_;
            return false;
        }

        keyframes_ << "kf_id,stamp,x,y,z,roll_deg,pitch_deg,yaw_deg,delta_trans_m,delta_yaw_deg,point_count\n";
        candidates_ << "curr_kf_id,hist_kf_id,curr_stamp,hist_stamp,id_gap,curr_x,curr_y,curr_z,curr_yaw_deg,"
                       "hist_x,hist_y,hist_z,hist_yaw_deg,xy_dist_m,z_diff_m,yaw_diff_deg,candidate_rank,"
                       "candidate_source,pass_id_gap,pass_closest_id,pass_range,range_threshold_m,submap_kf_count,"
                       "submap_mode\n";
        matches_ << "curr_kf_id,hist_kf_id,match_method,ndt_converged,ndt_iter,ndt_score,ndt_score_threshold,"
                    "fitness_score,source_points,target_points,overlap_ratio,inlier_ratio,correction_trans_m,"
                    "correction_yaw_deg,result_dx,result_dy,result_dz,result_dyaw_deg,match_time_ms,"
                    "source_accum_enabled,source_accum_used,source_accum_frames,source_accum_raw_points,"
                    "source_accum_fallback_reason,source_type,source_scan_count,source_time_span_sec,"
                    "source_points_before_downsample,source_points_after_downsample,source_accum_hard_fail\n";
        if (source_accum_.is_open()) {
            source_accum_ << "curr_kf_id,hist_kf_id,enabled,used,configured_frame_count,configured_min_frames,"
                             "configured_max_time_span_sec,configured_voxel_leaf_size_m,used_frames,raw_points,"
                             "source_points,fallback_reason,source_type,source_scan_count,source_time_span_sec,"
                             "source_points_before_downsample,source_points_after_downsample,source_accum_hard_fail\n";
        }
        gates_ << "curr_kf_id,hist_kf_id,gate_id_gap_pass,gate_range_pass,gate_yaw_pass,gate_z_pass,"
                  "gate_ndt_score_pass,gate_ndt_inlier_pass,ndt_inlier_ratio,ndt_inlier_ratio_threshold,"
                  "gate_correction_pass,gate_nms_pass,gate_pgo_chi2_pass,final_status,"
                  "reject_reason_primary,reject_reason_secondary,risk_flags,risk_score,risk_combo_gate_enable,"
                  "risk_combo_gate_result,risk_combo_reject_reason,pgo_impact_gate_enable,"
                  "max_pose_delta_near_loop_m,mean_pose_delta_near_loop_m,affected_kf_count,"
                  "local_straightness_delta,pgo_impact_gate_result,pgo_impact_reject_reason,"
                  "committed,pose_writeback,edge_committed,curr_kf_candidate_count,selected_for_pgo_trial,"
                  "suppressed_by_same_curr_kf_nms,candidate_rank,adjacent_pose_gate_enable,"
                  "adjacent_pose_gate_reject_on_violation,adjacent_pair_count,adjacent_max_delta_xy_m,"
                  "adjacent_mean_delta_xy_m,adjacent_p95_delta_xy_m,adjacent_max_delta_yaw_deg,"
                  "adjacent_mean_delta_yaw_deg,adjacent_p95_delta_yaw_deg,adjacent_max_delta_z_m,"
                  "adjacent_max_delta_trans_m,adjacent_worst_xy_pair_from,adjacent_worst_xy_pair_to,"
                  "adjacent_worst_yaw_pair_from,adjacent_worst_yaw_pair_to,adjacent_pose_gate_result,"
                  "adjacent_pose_gate_reject_reason,adjacent_top_delta_pairs,shape_deformation_enable,"
                  "shape_deformation_reject_on_violation,shape_valid,shape_pose_count,"
                  "shape_path_length_before_m,shape_delta_max_m,shape_delta_p95_m,shape_delta_mean_m,"
                  "shape_worst_kf_id,shape_scope,shape_local_window_count,"
                  "shape_local_valid_window_count,shape_local_max_delta_max_m,"
                  "shape_local_max_delta_p95_m,shape_local_max_delta_mean_m,shape_worst_endpoint,"
                  "shape_worst_window_start_kf_id,shape_worst_window_end_kf_id,"
                  "shape_gate_result,shape_gate_reject_reason\n";
        candidate_clusters_ << "curr_kf_id,raw_candidate_count,clustered_candidate_count,cluster_id,hist_kf_id,"
                               "hist_pose_x,hist_pose_y,selected_or_suppressed,suppress_reason\n";
        init_to_ndt_ << "curr_kf_id,hist_kf_id,ndt_score,converged,init_to_ndt_xy,init_to_ndt_yaw_deg,"
                        "init_to_ndt_z,accepted,reject_reason\n";
        edges_ << "curr_kf_id,hist_kf_id,dx,dy,dz,dyaw_deg,information_diag,loop_chi2,rk_loop_th,accepted,"
                  "pose_writeback,edge_committed\n";
        pgo_ << "curr_kf_id,hist_kf_id,loop_chi2_before,loop_chi2_after,rk_loop_th,pgo_error_before,"
                "pgo_error_after,pgo_error_delta,max_pose_delta_all_m,max_pose_delta_all_yaw_deg,"
                "max_pose_delta_near_loop_m,max_pose_delta_near_loop_yaw_deg,mean_pose_delta_near_loop_m,"
                "affected_kf_count,start_end_error_before_m,start_end_error_after_m,local_straightness_before,"
                "local_straightness_after,local_straightness_delta,pgo_impact_gate_enable,"
                "pgo_impact_gate_result,pgo_impact_reject_reason,pose_writeback,adjacent_pose_gate_enable,"
                "adjacent_pose_gate_reject_on_violation,adjacent_pair_count,adjacent_max_delta_xy_m,"
                "adjacent_mean_delta_xy_m,adjacent_p95_delta_xy_m,adjacent_max_delta_yaw_deg,"
                "adjacent_mean_delta_yaw_deg,adjacent_p95_delta_yaw_deg,adjacent_max_delta_z_m,"
                "adjacent_max_delta_trans_m,adjacent_worst_xy_pair_from,adjacent_worst_xy_pair_to,"
                "adjacent_worst_yaw_pair_from,adjacent_worst_yaw_pair_to,adjacent_pose_gate_result,"
                "adjacent_pose_gate_reject_reason,adjacent_top_delta_pairs,shape_deformation_enable,"
                "shape_deformation_reject_on_violation,shape_valid,shape_pose_count,"
                "shape_path_length_before_m,shape_delta_max_m,shape_delta_p95_m,shape_delta_mean_m,"
                "shape_worst_kf_id,shape_scope,shape_local_window_count,"
                "shape_local_valid_window_count,shape_local_max_delta_max_m,"
                "shape_local_max_delta_p95_m,shape_local_max_delta_mean_m,shape_worst_endpoint,"
                "shape_worst_window_start_kf_id,shape_worst_window_end_kf_id,"
                "shape_gate_result,shape_gate_reject_reason\n";
        suspects_ << "rank,curr_kf_id,hist_kf_id,risk_score,risk_flags,final_status,ndt_score,ndt_score_threshold,"
                     "xy_dist_m,range_threshold_m,correction_trans_m,correction_yaw_deg,loop_chi2,rk_loop_th,"
                     "pgo_error_before,pgo_error_after,max_pose_delta_near_loop_m\n";

        WriteMetadata(config_path, options_, run_prefix);
        enabled_ = true;
        LOG(INFO) << "loop debug log dir: " << output_dir_;
        return true;
    } catch (const std::exception& e) {
        LOG(WARNING) << "failed to init loop debug logger: " << e.what();
        return false;
    } catch (...) {
        LOG(WARNING) << "failed to init loop debug logger: unknown exception";
        return false;
    }
}

void LoopDebugLogger::WriteKeyframe(const KeyframeRow& row) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!enabled_ || !keyframes_.is_open()) return;
    try {
        keyframes_ << row.kf_id << "," << FormatDouble(row.stamp) << "," << FormatDouble(row.x) << ","
                   << FormatDouble(row.y) << "," << FormatDouble(row.z) << "," << FormatDouble(row.roll_deg) << ","
                   << FormatDouble(row.pitch_deg) << "," << FormatDouble(row.yaw_deg) << ","
                   << FormatDouble(row.delta_trans_m) << "," << FormatDouble(row.delta_yaw_deg) << ","
                   << row.point_count << "\n";
        keyframe_count_++;
        FlushIfNeeded(keyframes_, keyframes_since_flush_);
    } catch (...) {
        LOG(WARNING) << "failed to write loop keyframe debug row";
    }
}

void LoopDebugLogger::WriteCandidate(const CandidateRow& row) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!enabled_ || !options_.log_candidates || !candidates_.is_open()) return;
    try {
        candidates_ << row.curr_kf_id << "," << row.hist_kf_id << "," << FormatDouble(row.curr_stamp) << ","
                    << FormatDouble(row.hist_stamp) << "," << row.id_gap << "," << FormatDouble(row.curr_x) << ","
                    << FormatDouble(row.curr_y) << "," << FormatDouble(row.curr_z) << ","
                    << FormatDouble(row.curr_yaw_deg) << "," << FormatDouble(row.hist_x) << ","
                    << FormatDouble(row.hist_y) << "," << FormatDouble(row.hist_z) << ","
                    << FormatDouble(row.hist_yaw_deg) << "," << FormatDouble(row.xy_dist_m) << ","
                    << FormatDouble(row.z_diff_m) << "," << FormatDouble(row.yaw_diff_deg) << ","
                    << row.candidate_rank << "," << CsvEscape(row.candidate_source) << ","
                    << FormatBool(row.pass_id_gap) << "," << FormatBool(row.pass_closest_id) << ","
                    << FormatBool(row.pass_range) << "," << FormatDouble(row.range_threshold_m) << ","
                    << row.submap_kf_count << "," << CsvEscape(row.submap_mode) << "\n";
        candidate_count_++;
        FlushIfNeeded(candidates_, candidates_since_flush_);
    } catch (...) {
        LOG(WARNING) << "failed to write loop candidate debug row";
    }
}

void LoopDebugLogger::WriteMatch(const MatchRow& row) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!enabled_ || !options_.log_matches || !matches_.is_open()) return;
    try {
        matches_ << row.curr_kf_id << "," << row.hist_kf_id << "," << CsvEscape(row.match_method) << ","
                 << FormatBool(row.ndt_converged) << "," << row.ndt_iter << "," << FormatDouble(row.ndt_score) << ","
                 << FormatDouble(row.ndt_score_threshold) << "," << FormatDouble(row.fitness_score) << ","
                 << row.source_points << "," << row.target_points << "," << FormatDouble(row.overlap_ratio) << ","
                 << FormatDouble(row.inlier_ratio) << "," << FormatDouble(row.correction_trans_m) << ","
                 << FormatDouble(row.correction_yaw_deg) << "," << FormatDouble(row.result_dx) << ","
                 << FormatDouble(row.result_dy) << "," << FormatDouble(row.result_dz) << ","
                 << FormatDouble(row.result_dyaw_deg) << "," << FormatDouble(row.match_time_ms) << ","
                 << FormatBool(row.source_accum_enabled) << "," << FormatBool(row.source_accum_used) << ","
                 << row.source_accum_frames << "," << row.source_accum_raw_points << ","
                 << CsvEscape(row.source_accum_fallback_reason) << "," << CsvEscape(row.source_type) << ","
                 << row.source_scan_count << "," << FormatDouble(row.source_time_span_sec) << ","
                 << row.source_points_before_downsample << "," << row.source_points_after_downsample << ","
                 << FormatBool(row.source_accum_hard_fail) << "\n";
        match_count_++;
        FlushIfNeeded(matches_, matches_since_flush_);
    } catch (...) {
        LOG(WARNING) << "failed to write loop match debug row";
    }
}

void LoopDebugLogger::WriteSourceAccum(const SourceAccumRow& row) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!enabled_ || !options_.source_scan_accum_debug_enable || !source_accum_.is_open()) return;
    try {
        source_accum_ << row.curr_kf_id << "," << row.hist_kf_id << "," << FormatBool(row.enabled) << ","
                      << FormatBool(row.used) << "," << row.configured_frame_count << ","
                      << row.configured_min_frames << "," << FormatDouble(row.configured_max_time_span_sec) << ","
                      << FormatDouble(row.configured_voxel_leaf_size_m) << "," << row.used_frames << ","
                      << row.raw_points << "," << row.source_points << "," << CsvEscape(row.fallback_reason) << ","
                      << CsvEscape(row.source_type) << "," << row.source_scan_count << ","
                      << FormatDouble(row.source_time_span_sec) << "," << row.source_points_before_downsample << ","
                      << row.source_points_after_downsample << "," << FormatBool(row.source_accum_hard_fail) << "\n";
        FlushIfNeeded(source_accum_, source_accum_since_flush_);
    } catch (...) {
        LOG(WARNING) << "failed to write loop source accumulation debug row";
    }
}

void LoopDebugLogger::WriteGateDecision(const GateRow& row) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!enabled_ || !options_.log_gate_decisions || !gates_.is_open()) return;
    try {
        gates_ << row.curr_kf_id << "," << row.hist_kf_id << "," << FormatBool(row.gate_id_gap_pass) << ","
               << FormatBool(row.gate_range_pass) << "," << FormatBool(row.gate_yaw_pass) << ","
               << FormatBool(row.gate_z_pass) << "," << FormatBool(row.gate_ndt_score_pass) << ","
               << FormatBool(row.gate_ndt_inlier_pass) << "," << FormatDouble(row.ndt_inlier_ratio) << ","
               << FormatDouble(row.ndt_inlier_ratio_threshold) << ","
               << FormatBool(row.gate_correction_pass) << "," << FormatBool(row.gate_nms_pass) << ","
               << FormatBool(row.gate_pgo_chi2_pass) << "," << CsvEscape(row.final_status) << ","
               << CsvEscape(row.reject_reason_primary) << "," << CsvEscape(row.reject_reason_secondary) << ","
               << CsvEscape(row.risk_flags) << "," << FormatDouble(row.risk_score) << ","
               << FormatBool(row.risk_combo_gate_enable) << "," << CsvEscape(row.risk_combo_gate_result) << ","
               << CsvEscape(row.risk_combo_reject_reason) << "," << FormatBool(row.pgo_impact_gate_enable) << ","
               << FormatDouble(row.max_pose_delta_near_loop_m) << ","
               << FormatDouble(row.mean_pose_delta_near_loop_m) << ","
               << FormatLong(row.affected_kf_count) << "," << FormatDouble(row.local_straightness_delta) << ","
               << CsvEscape(row.pgo_impact_gate_result) << "," << CsvEscape(row.pgo_impact_reject_reason) << ","
               << FormatBool(row.committed) << ","
               << FormatBool(row.pose_writeback) << "," << FormatBool(row.edge_committed) << ","
               << row.curr_kf_candidate_count << "," << FormatBool(row.selected_for_pgo_trial) << ","
               << FormatBool(row.suppressed_by_same_curr_kf_nms) << "," << row.candidate_rank << ","
               << FormatBool(row.adjacent_pose_gate_enable) << ","
               << FormatBool(row.adjacent_pose_gate_reject_on_violation) << ","
               << FormatLong(row.adjacent_pair_count) << "," << FormatDouble(row.adjacent_max_delta_xy_m) << ","
               << FormatDouble(row.adjacent_mean_delta_xy_m) << ","
               << FormatDouble(row.adjacent_p95_delta_xy_m) << ","
               << FormatDouble(row.adjacent_max_delta_yaw_deg) << ","
               << FormatDouble(row.adjacent_mean_delta_yaw_deg) << ","
               << FormatDouble(row.adjacent_p95_delta_yaw_deg) << ","
               << FormatDouble(row.adjacent_max_delta_z_m) << ","
               << FormatDouble(row.adjacent_max_delta_trans_m) << "," << row.adjacent_worst_xy_pair_from << ","
               << row.adjacent_worst_xy_pair_to << "," << row.adjacent_worst_yaw_pair_from << ","
               << row.adjacent_worst_yaw_pair_to << "," << CsvEscape(row.adjacent_pose_gate_result) << ","
               << CsvEscape(row.adjacent_pose_gate_reject_reason) << ","
               << CsvEscape(row.adjacent_top_delta_pairs) << ","
               << FormatBool(row.shape_deformation_enable) << ","
               << FormatBool(row.shape_deformation_reject_on_violation) << ","
               << FormatBool(row.shape_valid) << "," << row.shape_pose_count << ","
               << FormatDouble(row.shape_path_length_before_m) << ","
               << FormatDouble(row.shape_delta_max_m) << ","
               << FormatDouble(row.shape_delta_p95_m) << ","
               << FormatDouble(row.shape_delta_mean_m) << "," << row.shape_worst_kf_id << ","
               << CsvEscape(row.shape_scope) << "," << row.shape_local_window_count << ","
               << row.shape_local_valid_window_count << ","
               << FormatDouble(row.shape_local_max_delta_max_m) << ","
               << FormatDouble(row.shape_local_max_delta_p95_m) << ","
               << FormatDouble(row.shape_local_max_delta_mean_m) << ","
               << CsvEscape(row.shape_worst_endpoint) << ","
               << row.shape_worst_window_start_kf_id << ","
               << row.shape_worst_window_end_kf_id << ","
               << CsvEscape(row.shape_gate_result) << ","
               << CsvEscape(row.shape_gate_reject_reason) << "\n";
        if (row.final_status == "accepted" || row.final_status == "accepted_high_risk" ||
            row.final_status == "committed_low_risk") {
            accepted_count_++;
        } else if (row.final_status.find("rejected_") == 0) {
            rejected_count_++;
        }
        FlushIfNeeded(gates_, gates_since_flush_);
    } catch (...) {
        LOG(WARNING) << "failed to write loop gate debug row";
    }
}

void LoopDebugLogger::WriteCandidateCluster(const CandidateClusterRow& row) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!enabled_ || !candidate_clusters_.is_open()) return;
    try {
        candidate_clusters_ << row.curr_kf_id << "," << row.raw_candidate_count << ","
                            << row.clustered_candidate_count << "," << row.cluster_id << "," << row.hist_kf_id
                            << "," << FormatDouble(row.hist_pose_x) << "," << FormatDouble(row.hist_pose_y) << ","
                            << CsvEscape(row.selected_or_suppressed) << "," << CsvEscape(row.suppress_reason)
                            << "\n";
        FlushIfNeeded(candidate_clusters_, candidate_clusters_since_flush_);
    } catch (...) {
        LOG(WARNING) << "failed to write loop candidate cluster debug row";
    }
}

void LoopDebugLogger::WriteInitToNdt(const InitToNdtRow& row) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!enabled_ || !init_to_ndt_.is_open()) return;
    try {
        init_to_ndt_ << row.curr_kf_id << "," << row.hist_kf_id << "," << FormatDouble(row.ndt_score) << ","
                     << FormatBool(row.converged) << "," << FormatDouble(row.init_to_ndt_xy) << ","
                     << FormatDouble(row.init_to_ndt_yaw_deg) << "," << FormatDouble(row.init_to_ndt_z) << ","
                     << FormatBool(row.accepted) << "," << CsvEscape(row.reject_reason) << "\n";
        FlushIfNeeded(init_to_ndt_, init_to_ndt_since_flush_);
    } catch (...) {
        LOG(WARNING) << "failed to write loop init-to-NDT debug row";
    }
}

void LoopDebugLogger::WriteEdge(const EdgeRow& row) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!enabled_ || !edges_.is_open()) return;
    try {
        edges_ << row.curr_kf_id << "," << row.hist_kf_id << "," << FormatDouble(row.dx) << ","
               << FormatDouble(row.dy) << "," << FormatDouble(row.dz) << "," << FormatDouble(row.dyaw_deg) << ","
               << CsvEscape(row.information_diag) << "," << FormatDouble(row.loop_chi2) << ","
               << FormatDouble(row.rk_loop_th) << "," << FormatBool(row.accepted) << ","
               << FormatBool(row.pose_writeback) << "," << FormatBool(row.edge_committed) << "\n";
        FlushIfNeeded(edges_, edges_since_flush_);
    } catch (...) {
        LOG(WARNING) << "failed to write loop edge debug row";
    }
}

void LoopDebugLogger::WritePgoImpact(const PgoImpactRow& row) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!enabled_ || !options_.log_pgo_impact || !pgo_.is_open()) return;
    try {
        pgo_ << row.curr_kf_id << "," << row.hist_kf_id << "," << FormatDouble(row.loop_chi2_before) << ","
             << FormatDouble(row.loop_chi2_after) << "," << FormatDouble(row.rk_loop_th) << ","
             << FormatDouble(row.pgo_error_before) << "," << FormatDouble(row.pgo_error_after) << ","
             << FormatDouble(row.pgo_error_delta) << "," << FormatDouble(row.max_pose_delta_all_m) << ","
             << FormatDouble(row.max_pose_delta_all_yaw_deg) << "," << FormatDouble(row.max_pose_delta_near_loop_m)
             << "," << FormatDouble(row.max_pose_delta_near_loop_yaw_deg) << ","
             << FormatDouble(row.mean_pose_delta_near_loop_m) << "," << row.affected_kf_count << ","
             << FormatDouble(row.start_end_error_before_m) << "," << FormatDouble(row.start_end_error_after_m) << ","
             << FormatDouble(row.local_straightness_before) << "," << FormatDouble(row.local_straightness_after)
             << "," << FormatDouble(row.local_straightness_delta) << ","
             << FormatBool(row.pgo_impact_gate_enable) << "," << CsvEscape(row.pgo_impact_gate_result) << ","
             << CsvEscape(row.pgo_impact_reject_reason) << "," << FormatBool(row.pose_writeback) << ","
             << FormatBool(row.adjacent_pose_gate_enable) << ","
             << FormatBool(row.adjacent_pose_gate_reject_on_violation) << ","
             << FormatLong(row.adjacent_pair_count) << "," << FormatDouble(row.adjacent_max_delta_xy_m) << ","
             << FormatDouble(row.adjacent_mean_delta_xy_m) << "," << FormatDouble(row.adjacent_p95_delta_xy_m)
             << "," << FormatDouble(row.adjacent_max_delta_yaw_deg) << ","
             << FormatDouble(row.adjacent_mean_delta_yaw_deg) << ","
             << FormatDouble(row.adjacent_p95_delta_yaw_deg) << "," << FormatDouble(row.adjacent_max_delta_z_m)
             << "," << FormatDouble(row.adjacent_max_delta_trans_m) << ","
             << row.adjacent_worst_xy_pair_from << "," << row.adjacent_worst_xy_pair_to << ","
             << row.adjacent_worst_yaw_pair_from << "," << row.adjacent_worst_yaw_pair_to << ","
             << CsvEscape(row.adjacent_pose_gate_result) << ","
             << CsvEscape(row.adjacent_pose_gate_reject_reason) << ","
             << CsvEscape(row.adjacent_top_delta_pairs) << ","
             << FormatBool(row.shape_deformation_enable) << ","
             << FormatBool(row.shape_deformation_reject_on_violation) << ","
             << FormatBool(row.shape_valid) << "," << row.shape_pose_count << ","
             << FormatDouble(row.shape_path_length_before_m) << ","
             << FormatDouble(row.shape_delta_max_m) << ","
             << FormatDouble(row.shape_delta_p95_m) << ","
             << FormatDouble(row.shape_delta_mean_m) << "," << row.shape_worst_kf_id << ","
             << CsvEscape(row.shape_scope) << "," << row.shape_local_window_count << ","
             << row.shape_local_valid_window_count << ","
             << FormatDouble(row.shape_local_max_delta_max_m) << ","
             << FormatDouble(row.shape_local_max_delta_p95_m) << ","
             << FormatDouble(row.shape_local_max_delta_mean_m) << ","
             << CsvEscape(row.shape_worst_endpoint) << ","
             << row.shape_worst_window_start_kf_id << ","
             << row.shape_worst_window_end_kf_id << ","
             << CsvEscape(row.shape_gate_result) << ","
             << CsvEscape(row.shape_gate_reject_reason) << "\n";
        FlushIfNeeded(pgo_, pgo_since_flush_);
    } catch (...) {
        LOG(WARNING) << "failed to write loop pgo impact debug row";
    }
}

void LoopDebugLogger::WriteSuspect(const CandidateRow& candidate, const MatchRow& match, const EdgeRow& edge,
                                   const PgoImpactRow& pgo, const GateRow& gate) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!enabled_ || !suspects_.is_open() || gate.risk_score <= 0.0 || suspect_count_ >= options_.max_suspects) return;
    try {
        suspect_count_++;
        suspects_ << suspect_count_ << "," << candidate.curr_kf_id << "," << candidate.hist_kf_id << ","
                  << FormatDouble(gate.risk_score) << "," << CsvEscape(gate.risk_flags) << ","
                  << CsvEscape(gate.final_status) << "," << FormatDouble(match.ndt_score) << ","
                  << FormatDouble(match.ndt_score_threshold) << "," << FormatDouble(candidate.xy_dist_m) << ","
                  << FormatDouble(candidate.range_threshold_m) << "," << FormatDouble(match.correction_trans_m) << ","
                  << FormatDouble(match.correction_yaw_deg) << "," << FormatDouble(edge.loop_chi2) << ","
                  << FormatDouble(edge.rk_loop_th) << "," << FormatDouble(pgo.pgo_error_before) << ","
                  << FormatDouble(pgo.pgo_error_after) << "," << FormatDouble(pgo.max_pose_delta_near_loop_m) << "\n";
        FlushIfNeeded(suspects_, suspects_since_flush_);
    } catch (...) {
        LOG(WARNING) << "failed to write loop suspect debug row";
    }
}

void LoopDebugLogger::Finish() {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!enabled_) return;
    try {
        WriteSummary();
        if (keyframes_.is_open()) keyframes_.close();
        if (candidates_.is_open()) candidates_.close();
        if (matches_.is_open()) matches_.close();
        if (gates_.is_open()) gates_.close();
        if (candidate_clusters_.is_open()) candidate_clusters_.close();
        if (init_to_ndt_.is_open()) init_to_ndt_.close();
        if (edges_.is_open()) edges_.close();
        if (pgo_.is_open()) pgo_.close();
        if (suspects_.is_open()) suspects_.close();
        if (source_accum_.is_open()) source_accum_.close();
    } catch (...) {
        LOG(WARNING) << "failed to finish loop debug logger";
    }
    enabled_ = false;
}

std::string LoopDebugLogger::CsvEscape(const std::string& value) {
    bool need_quote = value.find_first_of(",\"\n\r") != std::string::npos;
    if (!need_quote) return value;
    std::string out = "\"";
    for (char ch : value) {
        if (ch == '"') out += "\"\"";
        else out += ch;
    }
    out += "\"";
    return out;
}

std::string LoopDebugLogger::JsonEscape(const std::string& value) {
    std::string out;
    for (char ch : value) {
        if (ch == '"' || ch == '\\') {
            out += '\\';
            out += ch;
        } else if (ch == '\n') {
            out += "\\n";
        } else if (ch == '\r') {
            out += "\\r";
        } else if (ch == '\t') {
            out += "\\t";
        } else {
            out += ch;
        }
    }
    return out;
}

std::string LoopDebugLogger::FormatDouble(double value) {
    if (!std::isfinite(value)) return "NaN";
    std::ostringstream os;
    os << std::fixed << std::setprecision(9) << value;
    return os.str();
}

std::string LoopDebugLogger::FormatBool(bool value) {
    return value ? "true" : "false";
}

std::string LoopDebugLogger::FormatLong(long value) {
    if (value < 0) return "NaN";
    return std::to_string(value);
}

void LoopDebugLogger::FlushIfNeeded(std::ofstream& stream, int& counter) {
    counter++;
    if (options_.flush_every_n <= 1 || counter >= options_.flush_every_n) {
        stream.flush();
        counter = 0;
    }
}

void LoopDebugLogger::WriteMetadata(const std::string& config_path, const Options& options, const std::string& run_prefix) {
    std::ofstream out(output_dir_ + "/run_metadata.json", std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        LOG(WARNING) << "failed to open loop debug metadata";
        return;
    }
    YAML::Node root;
    try {
        root = YAML::LoadFile(config_path);
    } catch (...) {
    }
    const YAML::Node fasterlio = root["fasterlio"];
    const YAML::Node pgo = root["pgo"];
    out << "{\n";
    WriteJsonStringField(out, "run_id", run_id_);
    WriteJsonStringField(out, "run_prefix", run_prefix);
    WriteJsonStringField(out, "bag_path", EnvOrUnknown("LIGHTNING_BAG_PATH"));
    WriteJsonStringField(out, "config_path", config_path);
    WriteJsonStringField(out, "input_config_path", EnvOrUnknown("LIGHTNING_INPUT_CONFIG_PATH"));
    WriteJsonStringField(out, "effective_config_path", EnvOrUnknown("LIGHTNING_EFFECTIVE_CONFIG_PATH"));
    WriteJsonStringField(out, "git_commit", EnvOrUnknown("LIGHTNING_GIT_COMMIT"));
    WriteJsonStringField(out, "git_branch", EnvOrUnknown("LIGHTNING_GIT_BRANCH"));
    out << "  \"program_start_wall_time\": " << FormatDouble(start_wall_time_) << ",\n";
    out << "  \"loop_closing\": {\n";
    out << "    \"loop_kf_gap\": " << options.loop_kf_gap << ",\n";
    out << "    \"min_id_interval\": " << options.min_id_interval << ",\n";
    out << "    \"closest_id_th\": " << options.closest_id_th << ",\n";
    out << "    \"max_range\": " << FormatDouble(options.max_range) << ",\n";
    out << "    \"ndt_score_th\": " << FormatDouble(options.ndt_score_th) << ",\n";
    out << "    \"lidar_auto_ndt_inlier_gate\": {\n";
    out << "      \"enable\": " << FormatBool(options.lidar_auto_ndt_inlier_gate_enable) << ",\n";
    out << "      \"max_inlier_distance_m\": " << FormatDouble(options.ndt_inlier_max_dist_m) << ",\n";
    out << "      \"min_inlier_ratio\": " << FormatDouble(options.ndt_inlier_ratio_th) << "\n";
    out << "    },\n";
    out << "    \"rk_loop_th\": " << FormatDouble(options.rk_loop_th) << ",\n";
    out << "    \"with_height\": " << FormatBool(options.with_height) << ",\n";
    out << "    \"height_noise\": " << FormatDouble(options.height_noise) << ",\n";
    out << "    \"lidar_auto_candidate_cluster\": {\n";
    out << "      \"enable\": " << FormatBool(options.lidar_auto_candidate_cluster_enable) << ",\n";
    out << "      \"hist_cluster_id_gap\": " << options.hist_cluster_id_gap << ",\n";
    out << "      \"hist_cluster_radius_m\": " << FormatDouble(options.hist_cluster_radius_m) << ",\n";
    out << "      \"keep_per_cluster\": " << options.keep_per_cluster << "\n";
    out << "    },\n";
    out << "    \"lidar_auto_same_curr_kf_nms\": {\n";
    out << "      \"enable\": " << FormatBool(options.lidar_auto_same_curr_kf_nms_enable) << ",\n";
    out << "      \"keep_top\": " << options.same_curr_kf_keep_top << ",\n";
    out << "      \"fallback_enable\": " << FormatBool(options.same_curr_kf_fallback_enable) << ",\n";
    out << "      \"fallback_top_k\": " << options.same_curr_kf_fallback_top_k << "\n";
    out << "    },\n";
    out << "    \"source_scan_accum\": {\n";
    out << "      \"enable\": " << FormatBool(options.source_scan_accum_enable) << ",\n";
    out << "      \"max_scans\": " << options.source_scan_accum_max_scans << ",\n";
    out << "      \"min_scans\": " << options.source_scan_accum_min_scans << ",\n";
    out << "      \"time_sec\": " << FormatDouble(options.source_scan_accum_time_sec) << ",\n";
    out << "      \"ref\": \"" << JsonEscapeLocal(options.source_scan_accum_ref) << "\",\n";
    out << "      \"pose_type\": \"" << JsonEscapeLocal(options.source_scan_accum_pose_type) << "\",\n";
    out << "      \"max_trans_m\": " << FormatDouble(options.source_scan_accum_max_trans_m) << ",\n";
    out << "      \"max_yaw_deg\": " << FormatDouble(options.source_scan_accum_max_yaw_deg) << ",\n";
    out << "      \"voxel_leaf_m\": " << FormatDouble(options.source_scan_accum_voxel_leaf_m) << ",\n";
    out << "      \"max_points\": " << options.source_scan_accum_max_points << ",\n";
    out << "      \"min_points\": " << options.source_scan_accum_min_points << ",\n";
    out << "      \"max_yaw_rate_degps\": " << FormatDouble(options.source_scan_accum_max_yaw_rate_degps) << ",\n";
    out << "      \"max_trans_rate_mps\": " << FormatDouble(options.source_scan_accum_max_trans_rate_mps) << ",\n";
    out << "      \"require_monotonic_stamp\": " << FormatBool(options.source_scan_accum_require_monotonic_stamp)
        << ",\n";
    out << "      \"fallback_single\": " << FormatBool(options.source_scan_accum_fallback_single) << ",\n";
    out << "      \"debug_enable\": " << FormatBool(options.source_scan_accum_debug_enable) << ",\n";
    out << "      \"debug_csv\": \"" << JsonEscapeLocal(options.source_scan_accum_debug_csv) << "\"\n";
    out << "    },\n";
    out << "    \"lidar_auto_init_to_ndt_gate\": {\n";
    out << "      \"enable\": " << FormatBool(options.lidar_auto_init_to_ndt_gate_enable) << ",\n";
    out << "      \"max_xy_m\": " << FormatDouble(options.init_to_ndt_max_xy_m) << ",\n";
    out << "      \"max_yaw_deg\": " << FormatDouble(options.init_to_ndt_max_yaw_deg) << ",\n";
    out << "      \"max_z_m\": " << FormatDouble(options.init_to_ndt_max_z_m) << "\n";
    out << "    },\n";
    out << "    \"lidar_auto_pgo_trial_commit\": {\n";
    out << "      \"enable\": " << FormatBool(options.lidar_auto_pgo_trial_commit_enable) << "\n";
    out << "    },\n";
    out << "    \"lidar_auto_risk_combo_gate\": {\n";
    out << "      \"enable\": " << FormatBool(options.lidar_auto_risk_combo_gate_enable) << ",\n";
    out << "      \"reject_low_score_margin_and_large_correction\": "
        << FormatBool(options.risk_combo_reject_low_score_margin_and_large_correction) << ",\n";
    out << "      \"reject_low_score_margin_and_near_max_range\": "
        << FormatBool(options.risk_combo_reject_low_score_margin_and_near_max_range) << ",\n";
    out << "      \"reject_low_score_margin_and_local_pose_delta_large\": "
        << FormatBool(options.risk_combo_reject_low_score_margin_and_local_pose_delta_large) << ",\n";
    out << "      \"reject_large_correction_and_near_max_range\": "
        << FormatBool(options.risk_combo_reject_large_correction_and_near_max_range) << ",\n";
    out << "      \"low_score_margin_th\": " << FormatDouble(options.risk_low_score_margin_th) << ",\n";
    out << "      \"near_max_range_ratio\": " << FormatDouble(options.risk_near_max_range_ratio) << ",\n";
    out << "      \"large_correction_trans_m\": " << FormatDouble(options.risk_large_correction_trans_m) << ",\n";
    out << "      \"large_correction_yaw_deg\": " << FormatDouble(options.risk_large_correction_yaw_deg) << ",\n";
    out << "      \"local_pose_delta_large_m\": " << FormatDouble(options.risk_local_pose_delta_large_m) << "\n";
    out << "    },\n";
    out << "    \"lidar_auto_pgo_impact_gate\": {\n";
    out << "      \"enable\": " << FormatBool(options.lidar_auto_pgo_impact_gate_enable) << ",\n";
    out << "      \"max_pose_delta_near_loop_m\": "
        << FormatDouble(options.pgo_impact_max_pose_delta_near_loop_m) << ",\n";
    out << "      \"max_local_straightness_delta_m\": "
        << FormatDouble(options.pgo_impact_max_local_straightness_delta_m) << ",\n";
    out << "      \"max_affected_kf_count\": " << options.pgo_impact_max_affected_kf_count << ",\n";
    out << "      \"max_affected_kf_count_diagnostic_only\": true,\n";
    out << "      \"apply_to_auto_lidar_loop_only\": "
        << FormatBool(options.pgo_impact_apply_to_auto_lidar_loop_only) << "\n";
    out << "    },\n";
    out << "    \"lidar_auto_adjacent_pose_gate\": {\n";
    out << "      \"enable\": " << FormatBool(options.lidar_auto_adjacent_pose_gate_enable) << ",\n";
    out << "      \"reject_on_violation\": " << FormatBool(options.adjacent_pose_gate_reject_on_violation) << ",\n";
    out << "      \"scope\": \"" << JsonEscape(options.adjacent_pose_gate_scope) << "\",\n";
    out << "      \"max_pair_id_gap\": " << options.adjacent_pose_gate_max_pair_id_gap << ",\n";
    out << "      \"min_pair_count\": " << options.adjacent_pose_gate_min_pair_count << ",\n";
    out << "      \"max_adjacent_rel_delta_xy_m\": "
        << FormatDouble(options.adjacent_pose_gate_max_delta_xy_m) << ",\n";
    out << "      \"max_adjacent_rel_delta_yaw_deg\": "
        << FormatDouble(options.adjacent_pose_gate_max_delta_yaw_deg) << ",\n";
    out << "      \"p95_adjacent_rel_delta_xy_m\": "
        << FormatDouble(options.adjacent_pose_gate_p95_delta_xy_m) << ",\n";
    out << "      \"p95_adjacent_rel_delta_yaw_deg\": "
        << FormatDouble(options.adjacent_pose_gate_p95_delta_yaw_deg) << ",\n";
    out << "      \"log_top_k\": " << options.adjacent_pose_gate_log_top_k << ",\n";
    out << "      \"shape_deformation_enable\": "
        << FormatBool(options.adjacent_shape_deformation_enable) << ",\n";
    out << "      \"shape_deformation_reject_on_violation\": "
        << FormatBool(options.adjacent_shape_deformation_reject_on_violation) << ",\n";
    out << "      \"shape_align_mode\": \"" << JsonEscape(options.adjacent_shape_align_mode) << "\",\n";
    out << "      \"shape_scope\": \"" << JsonEscape(options.adjacent_shape_scope) << "\",\n";
    out << "      \"shape_min_pose_count\": " << options.adjacent_shape_min_pose_count << ",\n";
    out << "      \"shape_min_path_length_m\": "
        << FormatDouble(options.adjacent_shape_min_path_length_m) << ",\n";
    out << "      \"shape_endpoint_radius_m\": "
        << FormatDouble(options.adjacent_shape_endpoint_radius_m) << ",\n";
    out << "      \"shape_local_window_path_m\": "
        << FormatDouble(options.adjacent_shape_local_window_path_m) << ",\n";
    out << "      \"shape_local_window_stride_m\": "
        << FormatDouble(options.adjacent_shape_local_window_stride_m) << ",\n";
    out << "      \"shape_max_windows_per_endpoint\": "
        << options.adjacent_shape_max_windows_per_endpoint << ",\n";
    out << "      \"max_shape_delta_p95_m\": "
        << FormatDouble(options.adjacent_shape_max_delta_p95_m) << ",\n";
    out << "      \"max_shape_delta_max_m\": "
        << FormatDouble(options.adjacent_shape_max_delta_max_m) << ",\n";
    out << "      \"max_shape_delta_mean_m\": "
        << FormatDouble(options.adjacent_shape_max_delta_mean_m) << "\n";
    out << "    },\n";
    out << "    \"debug_log_enable\": " << FormatBool(options.enable) << ",\n";
    out << "    \"debug_log_dir\": \"" << JsonEscape(output_dir_) << "\"\n";
    out << "  },\n";
    out << "  \"keyframe\": {\n";
    out << "    \"kf_dis_th\": " << YamlValueOrNa<double>(fasterlio, "kf_dis_th") << ",\n";
    out << "    \"kf_angle_th\": " << YamlValueOrNa<double>(fasterlio, "kf_angle_th") << "\n";
    out << "  },\n";
    out << "  \"pgo\": {\n";
    out << "    \"lidar_odom_pos_noise\": " << YamlValueOrNa<double>(pgo, "lidar_odom_pos_noise") << ",\n";
    out << "    \"lidar_odom_ang_noise\": " << YamlValueOrNa<double>(pgo, "lidar_odom_ang_noise") << ",\n";
    out << "    \"lidar_loc_pos_noise\": " << YamlValueOrNa<double>(pgo, "lidar_loc_pos_noise") << ",\n";
    out << "    \"lidar_loc_ang_noise\": " << YamlValueOrNa<double>(pgo, "lidar_loc_ang_noise") << "\n";
    out << "  }\n";
    out << "}\n";
}

void LoopDebugLogger::WriteSummary() {
    std::ofstream out(output_dir_ + "/loop_debug_summary.md", std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        LOG(WARNING) << "failed to open loop debug summary";
        return;
    }
    out << "# Loop Debug Summary\n\n";
    out << "- run_id: " << run_id_ << "\n";
    out << "- output_dir: " << output_dir_ << "\n";
    out << "- duration_wall_sec: " << FormatDouble(WallTimeSec() - start_wall_time_) << "\n";
    out << "- loop_kf_gap: " << options_.loop_kf_gap << "\n";
    out << "- min_id_interval: " << options_.min_id_interval << "\n";
    out << "- closest_id_th: " << options_.closest_id_th << "\n";
    out << "- max_range: " << FormatDouble(options_.max_range) << "\n";
    out << "- ndt_score_th: " << FormatDouble(options_.ndt_score_th) << "\n";
    out << "- lidar_auto_ndt_inlier_gate.enable: "
        << FormatBool(options_.lidar_auto_ndt_inlier_gate_enable) << "\n";
    out << "- lidar_auto_ndt_inlier_gate.max_inlier_distance_m: "
        << FormatDouble(options_.ndt_inlier_max_dist_m) << "\n";
    out << "- lidar_auto_ndt_inlier_gate.min_inlier_ratio: "
        << FormatDouble(options_.ndt_inlier_ratio_th) << "\n";
    out << "- rk_loop_th: " << FormatDouble(options_.rk_loop_th) << "\n";
    out << "- with_height: " << FormatBool(options_.with_height) << "\n";
    out << "- height_noise: " << FormatDouble(options_.height_noise) << "\n\n";
    out << "- lidar_auto_candidate_cluster.enable: "
        << FormatBool(options_.lidar_auto_candidate_cluster_enable) << "\n";
    out << "- lidar_auto_candidate_cluster.hist_cluster_id_gap: " << options_.hist_cluster_id_gap << "\n";
    out << "- lidar_auto_candidate_cluster.hist_cluster_radius_m: "
        << FormatDouble(options_.hist_cluster_radius_m) << "\n";
    out << "- lidar_auto_candidate_cluster.keep_per_cluster: " << options_.keep_per_cluster << "\n\n";
    out << "- lidar_auto_same_curr_kf_nms.enable: "
        << FormatBool(options_.lidar_auto_same_curr_kf_nms_enable) << "\n";
    out << "- lidar_auto_same_curr_kf_nms.keep_top: " << options_.same_curr_kf_keep_top << "\n";
    out << "- lidar_auto_same_curr_kf_nms.fallback_enable: "
        << FormatBool(options_.same_curr_kf_fallback_enable) << "\n";
    out << "- lidar_auto_same_curr_kf_nms.fallback_top_k: "
        << options_.same_curr_kf_fallback_top_k << "\n\n";
    out << "- source_scan_accum.enable: " << FormatBool(options_.source_scan_accum_enable) << "\n";
    out << "- source_scan_accum.max_scans: " << options_.source_scan_accum_max_scans << "\n";
    out << "- source_scan_accum.min_scans: " << options_.source_scan_accum_min_scans << "\n";
    out << "- source_scan_accum.time_sec: " << FormatDouble(options_.source_scan_accum_time_sec) << "\n";
    out << "- source_scan_accum.ref: " << options_.source_scan_accum_ref << "\n";
    out << "- source_scan_accum.pose_type: " << options_.source_scan_accum_pose_type << "\n";
    out << "- source_scan_accum.max_trans_m: " << FormatDouble(options_.source_scan_accum_max_trans_m) << "\n";
    out << "- source_scan_accum.max_yaw_deg: " << FormatDouble(options_.source_scan_accum_max_yaw_deg) << "\n";
    out << "- source_scan_accum.voxel_leaf_m: " << FormatDouble(options_.source_scan_accum_voxel_leaf_m) << "\n";
    out << "- source_scan_accum.max_points: " << options_.source_scan_accum_max_points << "\n";
    out << "- source_scan_accum.min_points: " << options_.source_scan_accum_min_points << "\n";
    out << "- source_scan_accum.max_yaw_rate_degps: "
        << FormatDouble(options_.source_scan_accum_max_yaw_rate_degps) << "\n";
    out << "- source_scan_accum.max_trans_rate_mps: "
        << FormatDouble(options_.source_scan_accum_max_trans_rate_mps) << "\n";
    out << "- source_scan_accum.require_monotonic_stamp: "
        << FormatBool(options_.source_scan_accum_require_monotonic_stamp) << "\n";
    out << "- source_scan_accum.fallback_single: " << FormatBool(options_.source_scan_accum_fallback_single) << "\n";
    out << "- source_scan_accum.debug_enable: " << FormatBool(options_.source_scan_accum_debug_enable) << "\n";
    out << "- source_scan_accum.debug_csv: " << options_.source_scan_accum_debug_csv << "\n";
    out << "- source_scan_accum.source_frame_transform: p_curr_body = T_w_curr^-1 * T_w_i * p_i_body\n\n";
    out << "- lidar_auto_init_to_ndt_gate.enable: "
        << FormatBool(options_.lidar_auto_init_to_ndt_gate_enable) << "\n";
    out << "- lidar_auto_init_to_ndt_gate.max_xy_m: " << FormatDouble(options_.init_to_ndt_max_xy_m) << "\n";
    out << "- lidar_auto_init_to_ndt_gate.max_yaw_deg: "
        << FormatDouble(options_.init_to_ndt_max_yaw_deg) << "\n";
    out << "- lidar_auto_init_to_ndt_gate.max_z_m: " << FormatDouble(options_.init_to_ndt_max_z_m) << "\n\n";
    out << "- lidar_auto_pgo_trial_commit.enable: "
        << FormatBool(options_.lidar_auto_pgo_trial_commit_enable) << "\n\n";
    out << "- lidar_auto_risk_combo_gate.enable: "
        << FormatBool(options_.lidar_auto_risk_combo_gate_enable) << "\n";
    out << "- lidar_auto_risk_combo_gate.reject_low_score_margin_and_large_correction: "
        << FormatBool(options_.risk_combo_reject_low_score_margin_and_large_correction) << "\n";
    out << "- lidar_auto_risk_combo_gate.reject_low_score_margin_and_near_max_range: "
        << FormatBool(options_.risk_combo_reject_low_score_margin_and_near_max_range) << "\n";
    out << "- lidar_auto_risk_combo_gate.reject_low_score_margin_and_local_pose_delta_large: "
        << FormatBool(options_.risk_combo_reject_low_score_margin_and_local_pose_delta_large) << "\n";
    out << "- lidar_auto_risk_combo_gate.reject_large_correction_and_near_max_range: "
        << FormatBool(options_.risk_combo_reject_large_correction_and_near_max_range) << "\n";
    out << "- lidar_auto_risk_combo_gate.low_score_margin_th: "
        << FormatDouble(options_.risk_low_score_margin_th) << "\n";
    out << "- lidar_auto_risk_combo_gate.near_max_range_ratio: "
        << FormatDouble(options_.risk_near_max_range_ratio) << "\n";
    out << "- lidar_auto_risk_combo_gate.large_correction_trans_m: "
        << FormatDouble(options_.risk_large_correction_trans_m) << "\n";
    out << "- lidar_auto_risk_combo_gate.large_correction_yaw_deg: "
        << FormatDouble(options_.risk_large_correction_yaw_deg) << "\n";
    out << "- lidar_auto_risk_combo_gate.local_pose_delta_large_m: "
        << FormatDouble(options_.risk_local_pose_delta_large_m) << "\n\n";
    out << "- lidar_auto_pgo_impact_gate.enable: "
        << FormatBool(options_.lidar_auto_pgo_impact_gate_enable) << "\n";
    out << "- lidar_auto_pgo_impact_gate.max_pose_delta_near_loop_m: "
        << FormatDouble(options_.pgo_impact_max_pose_delta_near_loop_m) << "\n";
    out << "- lidar_auto_pgo_impact_gate.max_local_straightness_delta_m: "
        << FormatDouble(options_.pgo_impact_max_local_straightness_delta_m) << "\n";
    out << "- lidar_auto_pgo_impact_gate.max_affected_kf_count: "
        << options_.pgo_impact_max_affected_kf_count << " (diagnostic only; not a reject gate)\n";
    out << "- lidar_auto_pgo_impact_gate.apply_to_auto_lidar_loop_only: "
        << FormatBool(options_.pgo_impact_apply_to_auto_lidar_loop_only) << "\n\n";
    out << "- lidar_auto_adjacent_pose_gate.enable: "
        << FormatBool(options_.lidar_auto_adjacent_pose_gate_enable) << "\n";
    out << "- lidar_auto_adjacent_pose_gate.reject_on_violation: "
        << FormatBool(options_.adjacent_pose_gate_reject_on_violation) << "\n";
    out << "- lidar_auto_adjacent_pose_gate.scope: " << options_.adjacent_pose_gate_scope << "\n";
    out << "- lidar_auto_adjacent_pose_gate.max_pair_id_gap: "
        << options_.adjacent_pose_gate_max_pair_id_gap << "\n";
    out << "- lidar_auto_adjacent_pose_gate.min_pair_count: "
        << options_.adjacent_pose_gate_min_pair_count << "\n";
    out << "- lidar_auto_adjacent_pose_gate.max_adjacent_rel_delta_xy_m: "
        << FormatDouble(options_.adjacent_pose_gate_max_delta_xy_m) << "\n";
    out << "- lidar_auto_adjacent_pose_gate.max_adjacent_rel_delta_yaw_deg: "
        << FormatDouble(options_.adjacent_pose_gate_max_delta_yaw_deg) << "\n";
    out << "- lidar_auto_adjacent_pose_gate.p95_adjacent_rel_delta_xy_m: "
        << FormatDouble(options_.adjacent_pose_gate_p95_delta_xy_m) << "\n";
    out << "- lidar_auto_adjacent_pose_gate.p95_adjacent_rel_delta_yaw_deg: "
        << FormatDouble(options_.adjacent_pose_gate_p95_delta_yaw_deg) << "\n";
    out << "- lidar_auto_adjacent_pose_gate.shape_deformation_enable: "
        << FormatBool(options_.adjacent_shape_deformation_enable) << "\n";
    out << "- lidar_auto_adjacent_pose_gate.shape_deformation_reject_on_violation: "
        << FormatBool(options_.adjacent_shape_deformation_reject_on_violation) << "\n";
    out << "- lidar_auto_adjacent_pose_gate.shape_align_mode: "
        << options_.adjacent_shape_align_mode << "\n";
    out << "- lidar_auto_adjacent_pose_gate.shape_scope: "
        << options_.adjacent_shape_scope << "\n";
    out << "- lidar_auto_adjacent_pose_gate.shape_min_pose_count: "
        << options_.adjacent_shape_min_pose_count << "\n";
    out << "- lidar_auto_adjacent_pose_gate.shape_min_path_length_m: "
        << FormatDouble(options_.adjacent_shape_min_path_length_m) << "\n";
    out << "- lidar_auto_adjacent_pose_gate.shape_endpoint_radius_m: "
        << FormatDouble(options_.adjacent_shape_endpoint_radius_m) << "\n";
    out << "- lidar_auto_adjacent_pose_gate.shape_local_window_path_m: "
        << FormatDouble(options_.adjacent_shape_local_window_path_m) << "\n";
    out << "- lidar_auto_adjacent_pose_gate.shape_local_window_stride_m: "
        << FormatDouble(options_.adjacent_shape_local_window_stride_m) << "\n";
    out << "- lidar_auto_adjacent_pose_gate.shape_max_windows_per_endpoint: "
        << options_.adjacent_shape_max_windows_per_endpoint << "\n";
    out << "- lidar_auto_adjacent_pose_gate.max_shape_delta_p95_m: "
        << FormatDouble(options_.adjacent_shape_max_delta_p95_m) << "\n";
    out << "- lidar_auto_adjacent_pose_gate.max_shape_delta_max_m: "
        << FormatDouble(options_.adjacent_shape_max_delta_max_m) << "\n";
    out << "- lidar_auto_adjacent_pose_gate.max_shape_delta_mean_m: "
        << FormatDouble(options_.adjacent_shape_max_delta_mean_m) << "\n\n";
    out << "## Counts\n\n";
    out << "- keyframes: " << keyframe_count_ << "\n";
    out << "- candidates: " << candidate_count_ << "\n";
    out << "- matches: " << match_count_ << "\n";
    out << "- accepted: " << accepted_count_ << "\n";
    out << "- rejected: " << rejected_count_ << "\n";
    out << "- suspects_written: " << suspect_count_ << "\n\n";
    out << "## TODO\n\n";
    out << "- overlap_ratio is reserved; inlier_ratio is computed from final NDT source-to-target nearest-neighbor checks.\n";
    out << "- LiDAR candidate NMS uses optional history keyframe ID/XY clustering before NDT; gate_nms_pass records cluster suppression when enabled.\n";
    out << "- local_straightness uses a simple path-length/chord proxy and needs field validation for cattle-farm corridors.\n";
}

}  // namespace lightning
