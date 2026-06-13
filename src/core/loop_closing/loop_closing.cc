//
// Created by xiang on 25-4-21.
//

#include "core/loop_closing/loop_closing.h"
#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "utils/loop_debug_logger.h"
#include "utils/pointcloud_utils.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ndt.h>
#include <sstream>
#include <unordered_map>

#include "core/opti_algo/algo_select.h"
#include "core/robust_kernel/cauchy.h"
#include "core/types/edge_se3.h"
#include "core/types/edge_se3_height_prior.h"
#include "core/types/vertex_se3.h"
#include "io/yaml_io.h"

namespace lightning {
namespace {

constexpr double kRadToDeg = 180.0 / M_PI;

template <typename T>
T YamlGetOr(const YAML::Node& node, const std::string& key, const T& fallback) {
    if (!node || !node[key]) return fallback;
    try {
        return node[key].as<T>();
    } catch (...) {
        return fallback;
    }
}

double YawDeg(const SE3& pose) {
    return pose.so3().matrix().eulerAngles(0, 1, 2).z() * kRadToDeg;
}

double HeadingYawDeg(const SE3& pose) {
    const auto R = pose.so3().matrix();
    return std::atan2(R(1, 0), R(0, 0)) * kRadToDeg;
}

double RollDeg(const SE3& pose) {
    return pose.so3().matrix().eulerAngles(0, 1, 2).x() * kRadToDeg;
}

double PitchDeg(const SE3& pose) {
    return pose.so3().matrix().eulerAngles(0, 1, 2).y() * kRadToDeg;
}

double NormalizeYawDeg(double yaw) {
    while (yaw > 180.0) yaw -= 360.0;
    while (yaw < -180.0) yaw += 360.0;
    return yaw;
}

double PoseYawDiffDeg(const SE3& a, const SE3& b) {
    return NormalizeYawDeg(YawDeg(a) - YawDeg(b));
}

double HeadingYawDiffDeg(const SE3& a, const SE3& b) {
    return NormalizeYawDeg(HeadingYawDeg(a) - HeadingYawDeg(b));
}

double ComputeSourceInlierRatio(const CloudPtr& target_world, const CloudPtr& source_lidar,
                                const Mat4f& T_w_source, double max_inlier_dist_m,
                                long* inlier_count_out) {
    if (inlier_count_out) *inlier_count_out = 0;
    if (!target_world || !source_lidar || target_world->empty() || source_lidar->empty() ||
        !std::isfinite(max_inlier_dist_m) || max_inlier_dist_m <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    CloudPtr finite_target(new PointCloudType);
    finite_target->reserve(target_world->size());
    for (const auto& p : target_world->points) {
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
            finite_target->push_back(p);
        }
    }
    if (finite_target->empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    finite_target->is_dense = true;
    finite_target->height = 1;
    finite_target->width = finite_target->size();

    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(finite_target);

    const double max_dist2 = max_inlier_dist_m * max_inlier_dist_m;
    long valid_source_count = 0;
    long inlier_count = 0;
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    for (const auto& p : source_lidar->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            continue;
        }
        const Eigen::Vector4f src(p.x, p.y, p.z, 1.0f);
        const Eigen::Vector4f dst = T_w_source * src;
        if (!std::isfinite(dst.x()) || !std::isfinite(dst.y()) || !std::isfinite(dst.z())) {
            continue;
        }
        PointType query;
        query.x = dst.x();
        query.y = dst.y();
        query.z = dst.z();
        valid_source_count++;
        if (kdtree.nearestKSearch(query, 1, indices, distances) > 0 &&
            static_cast<double>(distances.front()) <= max_dist2) {
            inlier_count++;
        }
    }

    if (inlier_count_out) *inlier_count_out = inlier_count;
    if (valid_source_count <= 0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return static_cast<double>(inlier_count) / static_cast<double>(valid_source_count);
}

std::string CsvEscapeLocal(const std::string& value) {
    if (value.find_first_of(",\"\n\r") == std::string::npos) {
        return value;
    }
    std::string out = "\"";
    for (char ch : value) {
        if (ch == '"') {
            out += "\"\"";
        } else {
            out += ch;
        }
    }
    out += "\"";
    return out;
}

LoopDebugLogger::KeyframeRow MakeKeyframeRow(const Keyframe::Ptr& kf, const Keyframe::Ptr& last) {
    LoopDebugLogger::KeyframeRow row;
    if (!kf) return row;
    const auto state = kf->GetState();
    const auto pose = kf->GetOptPose();
    const auto t = pose.translation();
    row.kf_id = kf->GetID();
    row.stamp = state.timestamp_;
    row.x = t.x();
    row.y = t.y();
    row.z = t.z();
    row.roll_deg = RollDeg(pose);
    row.pitch_deg = PitchDeg(pose);
    row.yaw_deg = YawDeg(pose);
    const auto cloud = kf->GetCloud();
    row.point_count = cloud ? static_cast<long>(cloud->size()) : 0;
    if (last) {
        const auto last_pose = last->GetOptPose();
        row.delta_trans_m = (pose.translation() - last_pose.translation()).norm();
        row.delta_yaw_deg = std::fabs(PoseYawDiffDeg(pose, last_pose));
    }
    return row;
}

LoopDebugLogger::CandidateRow MakeCandidateRow(const Keyframe::Ptr& hist, const Keyframe::Ptr& cur,
                                               const LoopClosing::Options& options, int candidate_rank) {
    LoopDebugLogger::CandidateRow row;
    if (!hist || !cur) return row;
    const auto hist_state = hist->GetState();
    const auto cur_state = cur->GetState();
    const auto hist_pose = hist->GetOptPose();
    const auto cur_pose = cur->GetOptPose();
    const auto ht = hist_pose.translation();
    const auto ct = cur_pose.translation();
    row.curr_kf_id = cur->GetID();
    row.hist_kf_id = hist->GetID();
    row.curr_stamp = cur_state.timestamp_;
    row.hist_stamp = hist_state.timestamp_;
    row.id_gap = std::labs(static_cast<long>(cur->GetID()) - static_cast<long>(hist->GetID()));
    row.curr_x = ct.x();
    row.curr_y = ct.y();
    row.curr_z = ct.z();
    row.curr_yaw_deg = YawDeg(cur_pose);
    row.hist_x = ht.x();
    row.hist_y = ht.y();
    row.hist_z = ht.z();
    row.hist_yaw_deg = YawDeg(hist_pose);
    row.xy_dist_m = (ht.head<2>() - ct.head<2>()).norm();
    row.z_diff_m = std::fabs(ht.z() - ct.z());
    row.yaw_diff_deg = std::fabs(PoseYawDiffDeg(cur_pose, hist_pose));
    row.candidate_rank = candidate_rank;
    row.pass_id_gap = row.id_gap >= options.min_id_interval_;
    row.pass_closest_id = row.id_gap >= options.closest_id_th_;
    row.pass_range = row.xy_dist_m < options.max_range_;
    row.range_threshold_m = options.max_range_;
    return row;
}

LoopDebugLogger::MatchRow MakeMatchRow(const LoopCandidate& c, const LoopClosing::Options& options) {
    LoopDebugLogger::MatchRow row;
    row.curr_kf_id = c.idx2_;
    row.hist_kf_id = c.idx1_;
    row.ndt_converged = c.ndt_converged_;
    row.ndt_iter = c.ndt_iter_;
    row.ndt_score = c.ndt_score_;
    row.ndt_score_threshold = options.ndt_score_th_;
    row.fitness_score = c.fitness_score_;
    row.inlier_ratio = c.ndt_inlier_ratio_;
    row.source_points = c.source_points_;
    row.target_points = c.target_points_;
    row.correction_trans_m = c.correction_trans_m_;
    row.correction_yaw_deg = c.correction_yaw_deg_;
    row.result_dx = c.Tij_.translation().x();
    row.result_dy = c.Tij_.translation().y();
    row.result_dz = c.Tij_.translation().z();
    row.result_dyaw_deg = YawDeg(c.Tij_);
    row.match_time_ms = c.match_time_ms_;
    row.source_accum_enabled = c.source_accum_enabled_;
    row.source_accum_used = c.source_accum_used_;
    row.source_accum_frames = c.source_accum_frames_;
    row.source_accum_raw_points = c.source_accum_raw_points_;
    row.source_accum_fallback_reason = c.source_accum_fallback_reason_;
    row.source_type = c.source_type_;
    row.source_scan_count = c.source_scan_count_;
    row.source_time_span_sec = c.source_time_span_sec_;
    row.source_points_before_downsample = c.source_points_before_downsample_;
    row.source_points_after_downsample = c.source_points_after_downsample_;
    row.source_accum_hard_fail = c.source_accum_hard_fail_;
    return row;
}

LoopDebugLogger::SourceAccumRow MakeSourceAccumRow(const LoopCandidate& c, const LoopClosing::Options& options) {
    LoopDebugLogger::SourceAccumRow row;
    row.curr_kf_id = c.idx2_;
    row.hist_kf_id = c.idx1_;
    row.enabled = c.source_accum_enabled_;
    row.used = c.source_accum_used_;
    row.configured_frame_count = options.source_scan_accum_max_scans_;
    row.configured_min_frames = options.source_scan_accum_min_scans_;
    row.configured_max_time_span_sec = options.source_scan_accum_time_sec_;
    row.configured_voxel_leaf_size_m = options.source_scan_accum_voxel_leaf_m_;
    row.used_frames = c.source_accum_frames_;
    row.raw_points = c.source_accum_raw_points_;
    row.source_points = c.source_points_;
    row.fallback_reason = c.source_accum_fallback_reason_;
    row.source_type = c.source_type_;
    row.source_scan_count = c.source_scan_count_;
    row.source_time_span_sec = c.source_time_span_sec_;
    row.source_points_before_downsample = c.source_points_before_downsample_;
    row.source_points_after_downsample = c.source_points_after_downsample_;
    row.source_accum_hard_fail = c.source_accum_hard_fail_;
    return row;
}

LoopDebugLogger::InitToNdtRow MakeInitToNdtRow(const LoopCandidate& c, bool accepted,
                                               const std::string& reject_reason) {
    LoopDebugLogger::InitToNdtRow row;
    row.curr_kf_id = c.idx2_;
    row.hist_kf_id = c.idx1_;
    row.ndt_score = c.ndt_score_;
    row.converged = c.ndt_converged_;
    row.init_to_ndt_xy = c.init_to_ndt_xy_m_;
    row.init_to_ndt_yaw_deg = c.init_to_ndt_yaw_deg_;
    row.init_to_ndt_z = c.init_to_ndt_z_m_;
    row.accepted = accepted;
    row.reject_reason = reject_reason;
    return row;
}

std::string JoinFlags(const std::vector<std::string>& flags) {
    std::string out;
    for (const auto& f : flags) {
        if (!out.empty()) out += ";";
        out += f;
    }
    return out;
}

void AddRiskFlag(std::vector<std::string>& flags, const std::string& flag) {
    if (std::find(flags.begin(), flags.end(), flag) == flags.end()) {
        flags.emplace_back(flag);
    }
}

bool HasRiskFlag(const std::vector<std::string>& flags, const std::string& flag) {
    return std::find(flags.begin(), flags.end(), flag) != flags.end();
}

std::vector<std::string> BuildPrePgoRiskFlags(const LoopCandidate& c, const LoopClosing::Options& options) {
    std::vector<std::string> risk_flags;
    if (c.ndt_score_ - options.ndt_score_th_ < options.risk_low_score_margin_th_) {
        AddRiskFlag(risk_flags, "low_score_margin");
    }
    if (std::isfinite(c.xy_dist_m_) && c.xy_dist_m_ > options.risk_near_max_range_ratio_ * options.max_range_) {
        AddRiskFlag(risk_flags, "near_max_range");
    }
    if ((std::isfinite(c.correction_trans_m_) &&
         c.correction_trans_m_ > options.risk_large_correction_trans_m_) ||
        (std::isfinite(c.correction_yaw_deg_) &&
         c.correction_yaw_deg_ > options.risk_large_correction_yaw_deg_)) {
        AddRiskFlag(risk_flags, "large_correction");
    }
    return risk_flags;
}

std::string RiskComboRejectReason(const LoopClosing::Options& options, const std::vector<std::string>& risk_flags) {
    if (!options.lidar_auto_risk_combo_gate_enable_) return "";
    const bool low_score_margin = HasRiskFlag(risk_flags, "low_score_margin");
    const bool large_correction = HasRiskFlag(risk_flags, "large_correction");
    const bool near_max_range = HasRiskFlag(risk_flags, "near_max_range");
    const bool local_pose_delta_large = HasRiskFlag(risk_flags, "local_pose_delta_large");
    if (options.risk_combo_reject_low_score_margin_and_large_correction_ && low_score_margin && large_correction) {
        return "low_score_margin+large_correction";
    }
    if (options.risk_combo_reject_low_score_margin_and_near_max_range_ && low_score_margin && near_max_range) {
        return "low_score_margin+near_max_range";
    }
    if (options.risk_combo_reject_low_score_margin_and_local_pose_delta_large_ && low_score_margin &&
        local_pose_delta_large) {
        return "low_score_margin+local_pose_delta_large";
    }
    if (options.risk_combo_reject_large_correction_and_near_max_range_ && large_correction && near_max_range) {
        return "large_correction+near_max_range";
    }
    return "";
}

double PathStraightness(const std::vector<SE3>& poses) {
    if (poses.size() < 3) return std::numeric_limits<double>::quiet_NaN();
    double path = 0.0;
    for (size_t i = 1; i < poses.size(); ++i) {
        path += (poses[i].translation().head<2>() - poses[i - 1].translation().head<2>()).norm();
    }
    const double chord = (poses.back().translation().head<2>() - poses.front().translation().head<2>()).norm();
    if (chord < 1e-6) return std::numeric_limits<double>::quiet_NaN();
    return path / chord - 1.0;
}

bool HasValidNdtResult(const LoopCandidate& c) {
    return std::isfinite(c.ndt_score_) && (c.ndt_converged_ || c.ndt_score_ > 0.0);
}

double Percentile95(std::vector<double> values) {
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    const size_t index = std::min(values.size() - 1,
                                  static_cast<size_t>(std::ceil(0.95 * static_cast<double>(values.size()))) - 1);
    return values[index];
}

double PathLength2d(const std::vector<Eigen::Vector2d>& points) {
    double length = 0.0;
    for (size_t i = 1; i < points.size(); ++i) {
        length += (points[i] - points[i - 1]).norm();
    }
    return length;
}

struct ShapeSample {
    unsigned long id = 0;
    Eigen::Vector2d before = Eigen::Vector2d::Zero();
    Eigen::Vector2d after = Eigen::Vector2d::Zero();
    double path_s = 0.0;
};

struct ShapeWindowStats {
    bool valid = false;
    int pose_count = 0;
    double path_length_m = 0.0;
    double delta_max_m = 0.0;
    double delta_p95_m = 0.0;
    double delta_mean_m = 0.0;
    unsigned long worst_kf_id = 0;
    unsigned long start_kf_id = 0;
    unsigned long end_kf_id = 0;
    std::string endpoint;
    std::string reason;
};

ShapeWindowStats ComputeShapeWindowStats(const std::vector<ShapeSample>& samples,
                                         size_t begin, size_t end,
                                         const LoopClosing::Options& options,
                                         const std::string& endpoint) {
    ShapeWindowStats out;
    out.endpoint = endpoint;
    if (begin >= end || end > samples.size()) {
        out.reason = "insufficient_local_shape_window";
        return out;
    }

    out.pose_count = static_cast<int>(end - begin);
    out.start_kf_id = samples[begin].id;
    out.end_kf_id = samples[end - 1].id;
    out.path_length_m = samples[end - 1].path_s - samples[begin].path_s;
    if (out.pose_count < options.adjacent_shape_min_pose_count_ ||
        out.path_length_m < options.adjacent_shape_min_path_length_m_) {
        out.reason = "insufficient_local_shape_window";
        return out;
    }

    Eigen::Vector2d before_mean = Eigen::Vector2d::Zero();
    Eigen::Vector2d after_mean = Eigen::Vector2d::Zero();
    for (size_t i = begin; i < end; ++i) {
        before_mean += samples[i].before;
        after_mean += samples[i].after;
    }
    before_mean /= static_cast<double>(out.pose_count);
    after_mean /= static_cast<double>(out.pose_count);

    double sxx = 0.0;
    double sxy = 0.0;
    for (size_t i = begin; i < end; ++i) {
        const Eigen::Vector2d a = samples[i].after - after_mean;
        const Eigen::Vector2d b = samples[i].before - before_mean;
        sxx += a.x() * b.x() + a.y() * b.y();
        sxy += a.x() * b.y() - a.y() * b.x();
    }
    if (std::hypot(sxx, sxy) < 1e-12) {
        out.reason = "shape_alignment_degenerate";
        return out;
    }

    const double theta = std::atan2(sxy, sxx);
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    Eigen::Matrix2d R;
    R << c, -s, s, c;
    const Eigen::Vector2d t = before_mean - R * after_mean;

    std::vector<double> deltas;
    deltas.reserve(out.pose_count);
    double sum_delta = 0.0;
    for (size_t i = begin; i < end; ++i) {
        const Eigen::Vector2d aligned_after = R * samples[i].after + t;
        const double delta = (samples[i].before - aligned_after).norm();
        deltas.emplace_back(delta);
        sum_delta += delta;
        if (delta > out.delta_max_m) {
            out.delta_max_m = delta;
            out.worst_kf_id = samples[i].id;
        }
    }
    out.valid = true;
    out.delta_mean_m = sum_delta / static_cast<double>(out.pose_count);
    out.delta_p95_m = Percentile95(deltas);
    out.reason.clear();
    return out;
}

std::vector<size_t> SelectEndpointWindowStarts(const std::vector<ShapeSample>& samples,
                                               size_t range_begin, size_t range_end,
                                               double endpoint_s,
                                               const LoopClosing::Options& options) {
    std::vector<std::pair<double, size_t>> candidates;
    if (range_begin >= range_end || range_end > samples.size()) return {};
    const double range_min_s = samples[range_begin].path_s;
    const double range_max_s = samples[range_end - 1].path_s;
    const double window_path = std::max(0.0, options.adjacent_shape_local_window_path_m_);
    const double stride = std::max(1e-6, options.adjacent_shape_local_window_stride_m_);

    for (double start_s = range_min_s; start_s <= range_max_s + 1e-9; start_s += stride) {
        const auto start_it = std::lower_bound(samples.begin() + static_cast<long>(range_begin),
                                               samples.begin() + static_cast<long>(range_end), start_s,
                                               [](const ShapeSample& sample, double value) {
                                                   return sample.path_s < value;
                                               });
        if (start_it == samples.begin() + static_cast<long>(range_end)) break;
        const size_t start_idx = static_cast<size_t>(std::distance(samples.begin(), start_it));
        const double end_s = samples[start_idx].path_s + window_path;
        const auto end_it = std::upper_bound(samples.begin() + static_cast<long>(start_idx),
                                             samples.begin() + static_cast<long>(range_end), end_s,
                                             [](double value, const ShapeSample& sample) {
                                                 return value < sample.path_s;
                                             });
        if (end_it == samples.begin() + static_cast<long>(start_idx)) continue;
        const size_t end_idx = static_cast<size_t>(std::distance(samples.begin(), end_it));
        if (end_idx <= start_idx) continue;
        const double center_s = 0.5 * (samples[start_idx].path_s + samples[end_idx - 1].path_s);
        candidates.emplace_back(std::fabs(center_s - endpoint_s), start_idx);
    }

    std::sort(candidates.begin(), candidates.end(), [](const auto& lhs, const auto& rhs) {
        if (lhs.first != rhs.first) return lhs.first < rhs.first;
        return lhs.second < rhs.second;
    });
    const int max_windows = std::max(0, options.adjacent_shape_max_windows_per_endpoint_);
    std::vector<size_t> starts;
    const size_t keep = max_windows == 0 ? candidates.size()
                                         : std::min(candidates.size(), static_cast<size_t>(max_windows));
    starts.reserve(keep);
    for (size_t i = 0; i < keep; ++i) {
        starts.emplace_back(candidates[i].second);
    }
    std::sort(starts.begin(), starts.end());
    starts.erase(std::unique(starts.begin(), starts.end()), starts.end());
    return starts;
}

void AccumulateShapeWindow(const ShapeWindowStats& window,
                           LoopClosing::AdjacentPoseDeformationStats* stats) {
    if (stats == nullptr) return;
    if (!window.valid) return;
    stats->shape_local_valid_window_count++;
    stats->shape_valid = true;
    stats->shape_local_max_delta_max_m =
        std::max(stats->shape_local_max_delta_max_m, window.delta_max_m);
    stats->shape_local_max_delta_p95_m =
        std::max(stats->shape_local_max_delta_p95_m, window.delta_p95_m);
    stats->shape_local_max_delta_mean_m =
        std::max(stats->shape_local_max_delta_mean_m, window.delta_mean_m);
    stats->shape_delta_p95_m = std::max(stats->shape_delta_p95_m, window.delta_p95_m);
    stats->shape_delta_mean_m = std::max(stats->shape_delta_mean_m, window.delta_mean_m);
    if (window.delta_max_m > stats->shape_delta_max_m) {
        stats->shape_delta_max_m = window.delta_max_m;
        stats->shape_worst_kf_id = static_cast<int>(window.worst_kf_id);
        stats->shape_worst_endpoint = window.endpoint;
        stats->shape_worst_window_start_kf_id = static_cast<int>(window.start_kf_id);
        stats->shape_worst_window_end_kf_id = static_cast<int>(window.end_kf_id);
    }
}

void FillAdjacentPgoFields(LoopDebugLogger::PgoImpactRow& row,
                           const LoopClosing::AdjacentPoseDeformationStats& stats,
                           const LoopClosing::Options& options) {
    row.adjacent_pose_gate_enable = options.lidar_auto_adjacent_pose_gate_enable_;
    row.adjacent_pose_gate_reject_on_violation = options.adjacent_pose_gate_reject_on_violation_;
    row.adjacent_pair_count = stats.pair_count;
    row.adjacent_max_delta_xy_m = stats.max_delta_xy_m;
    row.adjacent_mean_delta_xy_m = stats.mean_delta_xy_m;
    row.adjacent_p95_delta_xy_m = stats.p95_delta_xy_m;
    row.adjacent_max_delta_yaw_deg = stats.max_delta_yaw_deg;
    row.adjacent_mean_delta_yaw_deg = stats.mean_delta_yaw_deg;
    row.adjacent_p95_delta_yaw_deg = stats.p95_delta_yaw_deg;
    row.adjacent_max_delta_z_m = stats.max_delta_z_m;
    row.adjacent_max_delta_trans_m = stats.max_delta_trans_m;
    row.adjacent_worst_xy_pair_from = stats.worst_xy_pair_from;
    row.adjacent_worst_xy_pair_to = stats.worst_xy_pair_to;
    row.adjacent_worst_yaw_pair_from = stats.worst_yaw_pair_from;
    row.adjacent_worst_yaw_pair_to = stats.worst_yaw_pair_to;
    row.adjacent_pose_gate_result = stats.gate_result;
    row.adjacent_pose_gate_reject_reason = stats.reject_reason;
    row.adjacent_top_delta_pairs = stats.top_delta_pairs;
    row.shape_deformation_enable = options.adjacent_shape_deformation_enable_;
    row.shape_deformation_reject_on_violation =
        options.adjacent_shape_deformation_reject_on_violation_;
    row.shape_valid = stats.shape_valid;
    row.shape_pose_count = stats.shape_pose_count;
    row.shape_path_length_before_m = stats.shape_path_length_before_m;
    row.shape_delta_max_m = stats.shape_delta_max_m;
    row.shape_delta_p95_m = stats.shape_delta_p95_m;
    row.shape_delta_mean_m = stats.shape_delta_mean_m;
    row.shape_worst_kf_id = stats.shape_worst_kf_id;
    row.shape_scope = stats.shape_scope;
    row.shape_local_window_count = stats.shape_local_window_count;
    row.shape_local_valid_window_count = stats.shape_local_valid_window_count;
    row.shape_local_max_delta_max_m = stats.shape_local_max_delta_max_m;
    row.shape_local_max_delta_p95_m = stats.shape_local_max_delta_p95_m;
    row.shape_local_max_delta_mean_m = stats.shape_local_max_delta_mean_m;
    row.shape_worst_endpoint = stats.shape_worst_endpoint;
    row.shape_worst_window_start_kf_id = stats.shape_worst_window_start_kf_id;
    row.shape_worst_window_end_kf_id = stats.shape_worst_window_end_kf_id;
    row.shape_gate_result = stats.shape_gate_result;
    row.shape_gate_reject_reason = stats.shape_gate_reject_reason;
}

void FillAdjacentGateFields(LoopDebugLogger::GateRow& row,
                            const LoopClosing::AdjacentPoseDeformationStats& stats,
                            const LoopClosing::Options& options) {
    row.adjacent_pose_gate_enable = options.lidar_auto_adjacent_pose_gate_enable_;
    row.adjacent_pose_gate_reject_on_violation = options.adjacent_pose_gate_reject_on_violation_;
    row.adjacent_pair_count = stats.pair_count;
    row.adjacent_max_delta_xy_m = stats.max_delta_xy_m;
    row.adjacent_mean_delta_xy_m = stats.mean_delta_xy_m;
    row.adjacent_p95_delta_xy_m = stats.p95_delta_xy_m;
    row.adjacent_max_delta_yaw_deg = stats.max_delta_yaw_deg;
    row.adjacent_mean_delta_yaw_deg = stats.mean_delta_yaw_deg;
    row.adjacent_p95_delta_yaw_deg = stats.p95_delta_yaw_deg;
    row.adjacent_max_delta_z_m = stats.max_delta_z_m;
    row.adjacent_max_delta_trans_m = stats.max_delta_trans_m;
    row.adjacent_worst_xy_pair_from = stats.worst_xy_pair_from;
    row.adjacent_worst_xy_pair_to = stats.worst_xy_pair_to;
    row.adjacent_worst_yaw_pair_from = stats.worst_yaw_pair_from;
    row.adjacent_worst_yaw_pair_to = stats.worst_yaw_pair_to;
    row.adjacent_pose_gate_result = stats.gate_result;
    row.adjacent_pose_gate_reject_reason = stats.reject_reason;
    row.adjacent_top_delta_pairs = stats.top_delta_pairs;
    row.shape_deformation_enable = options.adjacent_shape_deformation_enable_;
    row.shape_deformation_reject_on_violation =
        options.adjacent_shape_deformation_reject_on_violation_;
    row.shape_valid = stats.shape_valid;
    row.shape_pose_count = stats.shape_pose_count;
    row.shape_path_length_before_m = stats.shape_path_length_before_m;
    row.shape_delta_max_m = stats.shape_delta_max_m;
    row.shape_delta_p95_m = stats.shape_delta_p95_m;
    row.shape_delta_mean_m = stats.shape_delta_mean_m;
    row.shape_worst_kf_id = stats.shape_worst_kf_id;
    row.shape_scope = stats.shape_scope;
    row.shape_local_window_count = stats.shape_local_window_count;
    row.shape_local_valid_window_count = stats.shape_local_valid_window_count;
    row.shape_local_max_delta_max_m = stats.shape_local_max_delta_max_m;
    row.shape_local_max_delta_p95_m = stats.shape_local_max_delta_p95_m;
    row.shape_local_max_delta_mean_m = stats.shape_local_max_delta_mean_m;
    row.shape_worst_endpoint = stats.shape_worst_endpoint;
    row.shape_worst_window_start_kf_id = stats.shape_worst_window_start_kf_id;
    row.shape_worst_window_end_kf_id = stats.shape_worst_window_end_kf_id;
    row.shape_gate_result = stats.shape_gate_result;
    row.shape_gate_reject_reason = stats.shape_gate_reject_reason;
}

}  // namespace

LoopClosing::AdjacentPoseDeformationStats LoopClosing::ComputeAdjacentPoseDeformationStats(
    const std::unordered_map<unsigned long, SE3>& poses_before,
    const std::unordered_map<unsigned long, SE3>& poses_after,
    unsigned long hist_kf_id, unsigned long curr_kf_id, const Options& options) {
    AdjacentPoseDeformationStats stats;
    stats.gate_result = options.lidar_auto_adjacent_pose_gate_enable_ ? "not_evaluated" : "disabled";
    stats.shape_gate_result = options.adjacent_shape_deformation_enable_ ? "not_evaluated" : "disabled";

    const unsigned long min_id = std::min(hist_kf_id, curr_kf_id);
    const unsigned long max_id = std::max(hist_kf_id, curr_kf_id);
    std::vector<unsigned long> ordered_ids;
    ordered_ids.reserve(poses_after.size());
    for (const auto& item : poses_after) {
        if (item.first < min_id || item.first > max_id) continue;
        if (poses_before.find(item.first) == poses_before.end()) continue;
        ordered_ids.emplace_back(item.first);
    }
    std::sort(ordered_ids.begin(), ordered_ids.end());

    std::vector<double> delta_xys;
    std::vector<double> delta_yaws;
    std::vector<Eigen::Vector2d> before_shape_points;
    std::vector<Eigen::Vector2d> after_shape_points;
    std::vector<unsigned long> shape_ids;
    std::vector<ShapeSample> shape_samples;
    struct PairMetric {
        unsigned long from_id = 0;
        unsigned long to_id = 0;
        double delta_xy = 0.0;
        double delta_yaw = 0.0;
        double delta_z = 0.0;
        double delta_trans = 0.0;
    };
    std::vector<PairMetric> pair_metrics;
    delta_xys.reserve(ordered_ids.size());
    delta_yaws.reserve(ordered_ids.size());
    pair_metrics.reserve(ordered_ids.size());

    double sum_xy = 0.0;
    double sum_yaw = 0.0;
    before_shape_points.reserve(ordered_ids.size());
    after_shape_points.reserve(ordered_ids.size());
    shape_ids.reserve(ordered_ids.size());
    shape_samples.reserve(ordered_ids.size());
    for (const auto id : ordered_ids) {
        const auto before = poses_before.find(id);
        const auto after = poses_after.find(id);
        if (before == poses_before.end() || after == poses_after.end()) continue;
        const Eigen::Vector2d before_xy(before->second.translation().x(), before->second.translation().y());
        const Eigen::Vector2d after_xy(after->second.translation().x(), after->second.translation().y());
        before_shape_points.emplace_back(before_xy);
        after_shape_points.emplace_back(after_xy);
        shape_ids.emplace_back(id);
        ShapeSample sample;
        sample.id = id;
        sample.before = before_xy;
        sample.after = after_xy;
        if (!shape_samples.empty()) {
            sample.path_s = shape_samples.back().path_s + (before_xy - shape_samples.back().before).norm();
        }
        shape_samples.emplace_back(sample);
    }

    for (size_t i = 1; i < ordered_ids.size(); ++i) {
        const unsigned long from_id = ordered_ids[i - 1];
        const unsigned long to_id = ordered_ids[i];
        if (to_id < from_id || static_cast<int>(to_id - from_id) > options.adjacent_pose_gate_max_pair_id_gap_) {
            continue;
        }

        const auto before_i = poses_before.find(from_id);
        const auto before_j = poses_before.find(to_id);
        const auto after_i = poses_after.find(from_id);
        const auto after_j = poses_after.find(to_id);
        if (before_i == poses_before.end() || before_j == poses_before.end() ||
            after_i == poses_after.end() || after_j == poses_after.end()) {
            continue;
        }

        const SE3 T_rel_before = before_i->second.inverse() * before_j->second;
        const SE3 T_rel_after = after_i->second.inverse() * after_j->second;
        const SE3 T_delta_rel = T_rel_before.inverse() * T_rel_after;
        const auto trans = T_delta_rel.translation();
        const double delta_xy = trans.head<2>().norm();
        const double delta_z = std::fabs(trans.z());
        const double delta_trans = trans.norm();
        const double delta_yaw = std::fabs(NormalizeYawDeg(HeadingYawDeg(T_delta_rel)));

        stats.pair_count++;
        sum_xy += delta_xy;
        sum_yaw += delta_yaw;
        delta_xys.emplace_back(delta_xy);
        delta_yaws.emplace_back(delta_yaw);
        pair_metrics.push_back({from_id, to_id, delta_xy, delta_yaw, delta_z, delta_trans});
        if (delta_xy > stats.max_delta_xy_m) {
            stats.max_delta_xy_m = delta_xy;
            stats.worst_xy_pair_from = static_cast<int>(from_id);
            stats.worst_xy_pair_to = static_cast<int>(to_id);
        }
        if (delta_yaw > stats.max_delta_yaw_deg) {
            stats.max_delta_yaw_deg = delta_yaw;
            stats.worst_yaw_pair_from = static_cast<int>(from_id);
            stats.worst_yaw_pair_to = static_cast<int>(to_id);
        }
        stats.max_delta_z_m = std::max(stats.max_delta_z_m, delta_z);
        stats.max_delta_trans_m = std::max(stats.max_delta_trans_m, delta_trans);
    }

    if (stats.pair_count > 0) {
        stats.mean_delta_xy_m = sum_xy / static_cast<double>(stats.pair_count);
        stats.mean_delta_yaw_deg = sum_yaw / static_cast<double>(stats.pair_count);
        stats.p95_delta_xy_m = Percentile95(delta_xys);
        stats.p95_delta_yaw_deg = Percentile95(delta_yaws);
        std::sort(pair_metrics.begin(), pair_metrics.end(), [](const PairMetric& lhs, const PairMetric& rhs) {
            if (lhs.delta_xy != rhs.delta_xy) return lhs.delta_xy > rhs.delta_xy;
            return lhs.delta_yaw > rhs.delta_yaw;
        });
        std::ostringstream os;
        const int top_k = std::min(options.adjacent_pose_gate_log_top_k_,
                                   static_cast<int>(pair_metrics.size()));
        for (int i = 0; i < top_k; ++i) {
            if (i > 0) os << ";";
            os << pair_metrics[i].from_id << "->" << pair_metrics[i].to_id
               << ":xy=" << pair_metrics[i].delta_xy
               << "/yaw=" << pair_metrics[i].delta_yaw
               << "/z=" << pair_metrics[i].delta_z
               << "/trans=" << pair_metrics[i].delta_trans;
        }
        stats.top_delta_pairs = os.str();
    } else {
        stats.p95_delta_xy_m = std::numeric_limits<double>::quiet_NaN();
        stats.p95_delta_yaw_deg = std::numeric_limits<double>::quiet_NaN();
    }

    stats.shape_scope = options.adjacent_shape_scope_;
    stats.shape_pose_count = static_cast<int>(before_shape_points.size());
    stats.shape_path_length_before_m = PathLength2d(before_shape_points);
    if (!options.adjacent_shape_deformation_enable_) {
        stats.shape_gate_result = "disabled";
    } else if (stats.shape_pose_count < options.adjacent_shape_min_pose_count_) {
        stats.shape_gate_result = "insufficient_shape_pose_count";
        stats.shape_gate_reject_reason = "insufficient_shape_pose_count";
    } else if (stats.shape_path_length_before_m < options.adjacent_shape_min_path_length_m_) {
        stats.shape_gate_result = "insufficient_shape_path_length";
        stats.shape_gate_reject_reason = "insufficient_shape_path_length";
    } else if (options.adjacent_shape_scope_ == "between_loop_full") {
        const auto full_window = ComputeShapeWindowStats(shape_samples, 0, shape_samples.size(), options, "full");
        stats.shape_local_window_count = 1;
        if (!full_window.valid) {
            stats.shape_gate_result = full_window.reason.empty() ? "insufficient_shape_window" : full_window.reason;
            stats.shape_gate_reject_reason = stats.shape_gate_result;
        } else {
            AccumulateShapeWindow(full_window, &stats);
            stats.shape_gate_result = "pass";
        }
    } else {
        const auto process_endpoint = [&](unsigned long endpoint_id, const std::string& endpoint_name) {
            const auto it = std::find_if(shape_samples.begin(), shape_samples.end(),
                                         [&](const ShapeSample& sample) { return sample.id == endpoint_id; });
            if (it == shape_samples.end()) return;
            const size_t endpoint_index = static_cast<size_t>(std::distance(shape_samples.begin(), it));
            const double endpoint_s = shape_samples[endpoint_index].path_s;
            const double min_s = endpoint_s - options.adjacent_shape_endpoint_radius_m_;
            const double max_s = endpoint_s + options.adjacent_shape_endpoint_radius_m_;
            const auto begin_it = std::lower_bound(shape_samples.begin(), shape_samples.end(), min_s,
                                                   [](const ShapeSample& sample, double value) {
                                                       return sample.path_s < value;
                                                   });
            const auto end_it = std::upper_bound(shape_samples.begin(), shape_samples.end(), max_s,
                                                 [](double value, const ShapeSample& sample) {
                                                     return value < sample.path_s;
                                                 });
            const size_t range_begin = static_cast<size_t>(std::distance(shape_samples.begin(), begin_it));
            const size_t range_end = static_cast<size_t>(std::distance(shape_samples.begin(), end_it));
            const auto starts = SelectEndpointWindowStarts(shape_samples, range_begin, range_end,
                                                           endpoint_s, options);
            for (const auto start_idx : starts) {
                const double end_s = shape_samples[start_idx].path_s +
                                     options.adjacent_shape_local_window_path_m_;
                const auto window_end_it =
                    std::upper_bound(shape_samples.begin() + static_cast<long>(start_idx),
                                     shape_samples.begin() + static_cast<long>(range_end), end_s,
                                     [](double value, const ShapeSample& sample) {
                                         return value < sample.path_s;
                                     });
                const size_t end_idx = static_cast<size_t>(std::distance(shape_samples.begin(), window_end_it));
                stats.shape_local_window_count++;
                const auto window = ComputeShapeWindowStats(shape_samples, start_idx, end_idx, options,
                                                            endpoint_name);
                AccumulateShapeWindow(window, &stats);
            }
        };

        process_endpoint(hist_kf_id, "hist");
        if (curr_kf_id != hist_kf_id) {
            process_endpoint(curr_kf_id, "curr");
        }

        if (stats.shape_valid) {
            stats.shape_gate_result = "pass";
        } else {
            stats.shape_gate_result = "insufficient_local_shape_window";
            stats.shape_gate_reject_reason = "insufficient_local_shape_window";
        }
    }
    return stats;
}

bool LoopClosing::EvaluateAdjacentPoseGate(const Options& options, AdjacentPoseDeformationStats* stats) {
    if (stats == nullptr) return false;
    if (!options.lidar_auto_adjacent_pose_gate_enable_) {
        stats->gate_result = "disabled";
        return false;
    }
    if (stats->pair_count < options.adjacent_pose_gate_min_pair_count_) {
        stats->valid = false;
        stats->gate_result = "insufficient_adjacent_pair_count";
        stats->reject_reason = "insufficient_adjacent_pair_count";
        return false;
    }

    stats->valid = true;
    if (stats->max_delta_xy_m > options.adjacent_pose_gate_max_delta_xy_m_) {
        stats->reject_reason = "adjacent_rel_delta_xy";
    } else if (stats->max_delta_yaw_deg > options.adjacent_pose_gate_max_delta_yaw_deg_) {
        stats->reject_reason = "adjacent_rel_delta_yaw";
    } else if (std::isfinite(stats->p95_delta_xy_m) &&
               stats->p95_delta_xy_m > options.adjacent_pose_gate_p95_delta_xy_m_) {
        stats->reject_reason = "adjacent_rel_p95_xy";
    } else if (std::isfinite(stats->p95_delta_yaw_deg) &&
               stats->p95_delta_yaw_deg > options.adjacent_pose_gate_p95_delta_yaw_deg_) {
        stats->reject_reason = "adjacent_rel_p95_yaw";
    }

    if (options.adjacent_shape_deformation_enable_ && stats->shape_valid) {
        if (stats->shape_delta_max_m > options.adjacent_shape_max_delta_max_m_) {
            stats->shape_gate_reject_reason = "shape_delta_max";
        } else if (std::isfinite(stats->shape_delta_p95_m) &&
                   stats->shape_delta_p95_m > options.adjacent_shape_max_delta_p95_m_) {
            stats->shape_gate_reject_reason = "shape_delta_p95";
        } else if (stats->shape_delta_mean_m > options.adjacent_shape_max_delta_mean_m_) {
            stats->shape_gate_reject_reason = "shape_delta_mean";
        }
        if (stats->shape_gate_reject_reason.empty()) {
            stats->shape_gate_result = "pass";
        } else {
            stats->shape_gate_result = options.adjacent_shape_deformation_reject_on_violation_
                                           ? "reject"
                                           : "monitor_only_shape_deformation_violation";
            if (stats->reject_reason.empty()) {
                stats->reject_reason = stats->shape_gate_reject_reason;
            }
        }
    }

    if (stats->reject_reason.empty()) {
        stats->gate_result = "pass";
        return false;
    }
    if (stats->shape_gate_reject_reason == stats->reject_reason &&
        !options.adjacent_shape_deformation_reject_on_violation_) {
        stats->gate_result = "monitor_only_shape_deformation_violation";
        return false;
    }
    if (stats->shape_gate_reject_reason == stats->reject_reason &&
        options.adjacent_shape_deformation_reject_on_violation_) {
        stats->gate_result = "reject";
        return true;
    }
    stats->gate_result = options.adjacent_pose_gate_reject_on_violation_ ? "reject" :
                                                                  "monitor_only_adjacent_pose_violation";
    return options.adjacent_pose_gate_reject_on_violation_;
}

LoopClosing::~LoopClosing() {
    if (loop_debug_logger_) {
        loop_debug_logger_->Finish();
    }
    if (options_.online_mode_) {
        kf_thread_.Quit();
    }
}

void LoopClosing::Init(const std::string yaml_path) {
    /// setup miao
    miao::OptimizerConfig config(miao::AlgorithmType::LEVENBERG_MARQUARDT,
                                 miao::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN, false);
    config.incremental_mode_ = true;
    optimizer_ = miao::SetupOptimizer<6, 3>(config);

    info_motion_.setIdentity();
    info_motion_.block<3, 3>(0, 0) =
        Mat3d::Identity() * 1.0 / (options_.motion_trans_noise_ * options_.motion_trans_noise_);
    info_motion_.block<3, 3>(3, 3) =
        Mat3d::Identity() * 1.0 / (options_.motion_rot_noise_ * options_.motion_rot_noise_);

    info_loops_.setIdentity();
    info_loops_.block<3, 3>(0, 0) = Mat3d::Identity() * 1.0 / (options_.loop_trans_noise_ * options_.loop_trans_noise_);
    info_loops_.block<3, 3>(3, 3) = Mat3d::Identity() * 1.0 / (options_.loop_rot_noise_ * options_.loop_rot_noise_);

    if (!yaml_path.empty()) {
        YAML_IO yaml(yaml_path);

        options_.loop_kf_gap_ = yaml.GetValue<int>("loop_closing", "loop_kf_gap");
        options_.min_id_interval_ = yaml.GetValue<int>("loop_closing", "min_id_interval");
        options_.closest_id_th_ = yaml.GetValue<int>("loop_closing", "closest_id_th");
        options_.max_range_ = yaml.GetValue<double>("loop_closing", "max_range");
        options_.ndt_score_th_ = yaml.GetValue<double>("loop_closing", "ndt_score_th");
        options_.with_height_ = yaml.GetValue<bool>("loop_closing", "with_height");
        try {
            YAML::Node root = YAML::LoadFile(yaml_path);
            const auto loop = root["loop_closing"];
            options_.rk_loop_th_ = YamlGetOr<double>(loop, "rk_loop_th", options_.rk_loop_th_);
            options_.height_noise_ = YamlGetOr<double>(loop, "height_noise", options_.height_noise_);
            const auto ndt_inlier_gate = loop["lidar_auto_ndt_inlier_gate"];
            options_.lidar_auto_ndt_inlier_gate_enable_ =
                YamlGetOr<bool>(ndt_inlier_gate, "enable", options_.lidar_auto_ndt_inlier_gate_enable_);
            options_.ndt_inlier_max_dist_m_ =
                YamlGetOr<double>(ndt_inlier_gate, "max_inlier_distance_m", options_.ndt_inlier_max_dist_m_);
            options_.ndt_inlier_ratio_th_ =
                YamlGetOr<double>(ndt_inlier_gate, "min_inlier_ratio", options_.ndt_inlier_ratio_th_);
            const auto cluster = loop["lidar_auto_candidate_cluster"];
            options_.lidar_auto_candidate_cluster_enable_ =
                YamlGetOr<bool>(cluster, "enable", options_.lidar_auto_candidate_cluster_enable_);
            options_.hist_cluster_id_gap_ = YamlGetOr<int>(cluster, "hist_cluster_id_gap", options_.hist_cluster_id_gap_);
            options_.hist_cluster_radius_m_ =
                YamlGetOr<double>(cluster, "hist_cluster_radius_m", options_.hist_cluster_radius_m_);
            options_.keep_per_cluster_ = YamlGetOr<int>(cluster, "keep_per_cluster", options_.keep_per_cluster_);
            const auto same_curr_nms = loop["lidar_auto_same_curr_kf_nms"];
            options_.lidar_auto_same_curr_kf_nms_enable_ =
                YamlGetOr<bool>(same_curr_nms, "enable", options_.lidar_auto_same_curr_kf_nms_enable_);
            options_.same_curr_kf_keep_top_ =
                YamlGetOr<int>(same_curr_nms, "keep_top", options_.same_curr_kf_keep_top_);
            options_.same_curr_kf_fallback_enable_ =
                YamlGetOr<bool>(same_curr_nms, "fallback_enable", options_.same_curr_kf_fallback_enable_);
            options_.same_curr_kf_fallback_top_k_ =
                YamlGetOr<int>(same_curr_nms, "fallback_top_k", options_.same_curr_kf_fallback_top_k_);
            options_.source_scan_accum_enable_ =
                YamlGetOr<bool>(loop, "source_scan_accum_enable", options_.source_scan_accum_enable_);
            options_.source_scan_accum_max_scans_ =
                YamlGetOr<int>(loop, "source_scan_accum_max_scans", options_.source_scan_accum_max_scans_);
            options_.source_scan_accum_min_scans_ =
                YamlGetOr<int>(loop, "source_scan_accum_min_scans", options_.source_scan_accum_min_scans_);
            options_.source_scan_accum_time_sec_ =
                YamlGetOr<double>(loop, "source_scan_accum_time_sec", options_.source_scan_accum_time_sec_);
            options_.source_scan_accum_ref_ =
                YamlGetOr<std::string>(loop, "source_scan_accum_ref", options_.source_scan_accum_ref_);
            options_.source_scan_accum_pose_type_ =
                YamlGetOr<std::string>(loop, "source_scan_accum_pose_type", options_.source_scan_accum_pose_type_);
            options_.source_scan_accum_max_trans_m_ =
                YamlGetOr<double>(loop, "source_scan_accum_max_trans_m", options_.source_scan_accum_max_trans_m_);
            options_.source_scan_accum_max_yaw_deg_ =
                YamlGetOr<double>(loop, "source_scan_accum_max_yaw_deg", options_.source_scan_accum_max_yaw_deg_);
            options_.source_scan_accum_voxel_leaf_m_ =
                YamlGetOr<double>(loop, "source_scan_accum_voxel_leaf_m", options_.source_scan_accum_voxel_leaf_m_);
            options_.source_scan_accum_max_points_ =
                YamlGetOr<int>(loop, "source_scan_accum_max_points", options_.source_scan_accum_max_points_);
            options_.source_scan_accum_min_points_ =
                YamlGetOr<int>(loop, "source_scan_accum_min_points", options_.source_scan_accum_min_points_);
            options_.source_scan_accum_max_yaw_rate_degps_ =
                YamlGetOr<double>(loop, "source_scan_accum_max_yaw_rate_degps",
                                  options_.source_scan_accum_max_yaw_rate_degps_);
            options_.source_scan_accum_max_trans_rate_mps_ =
                YamlGetOr<double>(loop, "source_scan_accum_max_trans_rate_mps",
                                  options_.source_scan_accum_max_trans_rate_mps_);
            options_.source_scan_accum_require_monotonic_stamp_ =
                YamlGetOr<bool>(loop, "source_scan_accum_require_monotonic_stamp",
                                options_.source_scan_accum_require_monotonic_stamp_);
            options_.source_scan_accum_fallback_single_ =
                YamlGetOr<bool>(loop, "source_scan_accum_fallback_single",
                                options_.source_scan_accum_fallback_single_);
            options_.source_scan_accum_debug_enable_ =
                YamlGetOr<bool>(loop, "source_scan_accum_debug_enable", options_.source_scan_accum_debug_enable_);
            options_.source_scan_accum_debug_csv_ =
                YamlGetOr<std::string>(loop, "source_scan_accum_debug_csv", options_.source_scan_accum_debug_csv_);
            const auto init_to_ndt_gate = loop["lidar_auto_init_to_ndt_gate"];
            options_.lidar_auto_init_to_ndt_gate_enable_ =
                YamlGetOr<bool>(init_to_ndt_gate, "enable", options_.lidar_auto_init_to_ndt_gate_enable_);
            options_.init_to_ndt_max_xy_m_ =
                YamlGetOr<double>(init_to_ndt_gate, "max_xy_m", options_.init_to_ndt_max_xy_m_);
            options_.init_to_ndt_max_yaw_deg_ =
                YamlGetOr<double>(init_to_ndt_gate, "max_yaw_deg", options_.init_to_ndt_max_yaw_deg_);
            options_.init_to_ndt_max_z_m_ =
                YamlGetOr<double>(init_to_ndt_gate, "max_z_m", options_.init_to_ndt_max_z_m_);
            const auto pgo_trial_commit = loop["lidar_auto_pgo_trial_commit"];
            options_.lidar_auto_pgo_trial_commit_enable_ =
                YamlGetOr<bool>(pgo_trial_commit, "enable", options_.lidar_auto_pgo_trial_commit_enable_);
            if (!options_.lidar_auto_pgo_trial_commit_enable_) {
                LOG(WARNING) << "loop_closing.lidar_auto_pgo_trial_commit.enable=false is unsafe for automatic "
                                "LiDAR loop closure; forcing enable=true";
                options_.lidar_auto_pgo_trial_commit_enable_ = true;
            }
            const auto risk_combo_gate = loop["lidar_auto_risk_combo_gate"];
            options_.lidar_auto_risk_combo_gate_enable_ =
                YamlGetOr<bool>(risk_combo_gate, "enable", options_.lidar_auto_risk_combo_gate_enable_);
            options_.risk_combo_reject_low_score_margin_and_large_correction_ =
                YamlGetOr<bool>(risk_combo_gate, "reject_low_score_margin_and_large_correction",
                                options_.risk_combo_reject_low_score_margin_and_large_correction_);
            options_.risk_combo_reject_low_score_margin_and_near_max_range_ =
                YamlGetOr<bool>(risk_combo_gate, "reject_low_score_margin_and_near_max_range",
                                options_.risk_combo_reject_low_score_margin_and_near_max_range_);
            options_.risk_combo_reject_low_score_margin_and_local_pose_delta_large_ =
                YamlGetOr<bool>(risk_combo_gate, "reject_low_score_margin_and_local_pose_delta_large",
                                options_.risk_combo_reject_low_score_margin_and_local_pose_delta_large_);
            options_.risk_combo_reject_large_correction_and_near_max_range_ =
                YamlGetOr<bool>(risk_combo_gate, "reject_large_correction_and_near_max_range",
                                options_.risk_combo_reject_large_correction_and_near_max_range_);
            options_.risk_low_score_margin_th_ =
                YamlGetOr<double>(risk_combo_gate, "low_score_margin_th", options_.risk_low_score_margin_th_);
            options_.risk_near_max_range_ratio_ =
                YamlGetOr<double>(risk_combo_gate, "near_max_range_ratio", options_.risk_near_max_range_ratio_);
            options_.risk_large_correction_trans_m_ =
                YamlGetOr<double>(risk_combo_gate, "large_correction_trans_m", options_.risk_large_correction_trans_m_);
            options_.risk_large_correction_yaw_deg_ =
                YamlGetOr<double>(risk_combo_gate, "large_correction_yaw_deg", options_.risk_large_correction_yaw_deg_);
            options_.risk_local_pose_delta_large_m_ =
                YamlGetOr<double>(risk_combo_gate, "local_pose_delta_large_m", options_.risk_local_pose_delta_large_m_);
            const auto pgo_impact_gate = loop["lidar_auto_pgo_impact_gate"];
            options_.lidar_auto_pgo_impact_gate_enable_ =
                YamlGetOr<bool>(pgo_impact_gate, "enable", options_.lidar_auto_pgo_impact_gate_enable_);
            options_.pgo_impact_max_pose_delta_near_loop_m_ =
                YamlGetOr<double>(pgo_impact_gate, "max_pose_delta_near_loop_m",
                                  options_.pgo_impact_max_pose_delta_near_loop_m_);
            options_.pgo_impact_max_local_straightness_delta_m_ =
                YamlGetOr<double>(pgo_impact_gate, "max_local_straightness_delta_m",
                                  options_.pgo_impact_max_local_straightness_delta_m_);
            options_.pgo_impact_max_affected_kf_count_ =
                YamlGetOr<int>(pgo_impact_gate, "max_affected_kf_count",
                               options_.pgo_impact_max_affected_kf_count_);
            options_.pgo_impact_apply_to_auto_lidar_loop_only_ =
                YamlGetOr<bool>(pgo_impact_gate, "apply_to_auto_lidar_loop_only",
                                options_.pgo_impact_apply_to_auto_lidar_loop_only_);
            const auto adjacent_pose_gate = loop["lidar_auto_adjacent_pose_gate"];
            options_.lidar_auto_adjacent_pose_gate_enable_ =
                YamlGetOr<bool>(adjacent_pose_gate, "enable", options_.lidar_auto_adjacent_pose_gate_enable_);
            options_.adjacent_pose_gate_reject_on_violation_ =
                YamlGetOr<bool>(adjacent_pose_gate, "reject_on_violation",
                                options_.adjacent_pose_gate_reject_on_violation_);
            options_.adjacent_pose_gate_scope_ =
                YamlGetOr<std::string>(adjacent_pose_gate, "scope", options_.adjacent_pose_gate_scope_);
            options_.adjacent_pose_gate_max_pair_id_gap_ =
                YamlGetOr<int>(adjacent_pose_gate, "max_pair_id_gap",
                               options_.adjacent_pose_gate_max_pair_id_gap_);
            options_.adjacent_pose_gate_min_pair_count_ =
                YamlGetOr<int>(adjacent_pose_gate, "min_pair_count",
                               options_.adjacent_pose_gate_min_pair_count_);
            options_.adjacent_pose_gate_max_delta_xy_m_ =
                YamlGetOr<double>(adjacent_pose_gate, "max_adjacent_rel_delta_xy_m",
                                  options_.adjacent_pose_gate_max_delta_xy_m_);
            options_.adjacent_pose_gate_max_delta_yaw_deg_ =
                YamlGetOr<double>(adjacent_pose_gate, "max_adjacent_rel_delta_yaw_deg",
                                  options_.adjacent_pose_gate_max_delta_yaw_deg_);
            options_.adjacent_pose_gate_p95_delta_xy_m_ =
                YamlGetOr<double>(adjacent_pose_gate, "p95_adjacent_rel_delta_xy_m",
                                  options_.adjacent_pose_gate_p95_delta_xy_m_);
            options_.adjacent_pose_gate_p95_delta_yaw_deg_ =
                YamlGetOr<double>(adjacent_pose_gate, "p95_adjacent_rel_delta_yaw_deg",
                                  options_.adjacent_pose_gate_p95_delta_yaw_deg_);
            options_.adjacent_pose_gate_log_top_k_ =
                YamlGetOr<int>(adjacent_pose_gate, "log_top_k", options_.adjacent_pose_gate_log_top_k_);
            options_.adjacent_shape_deformation_enable_ =
                YamlGetOr<bool>(adjacent_pose_gate, "shape_deformation_enable",
                                options_.adjacent_shape_deformation_enable_);
            options_.adjacent_shape_deformation_reject_on_violation_ =
                YamlGetOr<bool>(adjacent_pose_gate, "shape_deformation_reject_on_violation",
                                options_.adjacent_shape_deformation_reject_on_violation_);
            options_.adjacent_shape_align_mode_ =
                YamlGetOr<std::string>(adjacent_pose_gate, "shape_align_mode",
                                       options_.adjacent_shape_align_mode_);
            options_.adjacent_shape_scope_ =
                YamlGetOr<std::string>(adjacent_pose_gate, "shape_scope",
                                       options_.adjacent_shape_scope_);
            options_.adjacent_shape_min_pose_count_ =
                YamlGetOr<int>(adjacent_pose_gate, "shape_min_pose_count",
                               options_.adjacent_shape_min_pose_count_);
            options_.adjacent_shape_min_path_length_m_ =
                YamlGetOr<double>(adjacent_pose_gate, "shape_min_path_length_m",
                                  options_.adjacent_shape_min_path_length_m_);
            options_.adjacent_shape_endpoint_radius_m_ =
                YamlGetOr<double>(adjacent_pose_gate, "shape_endpoint_radius_m",
                                  options_.adjacent_shape_endpoint_radius_m_);
            options_.adjacent_shape_local_window_path_m_ =
                YamlGetOr<double>(adjacent_pose_gate, "shape_local_window_path_m",
                                  options_.adjacent_shape_local_window_path_m_);
            options_.adjacent_shape_local_window_stride_m_ =
                YamlGetOr<double>(adjacent_pose_gate, "shape_local_window_stride_m",
                                  options_.adjacent_shape_local_window_stride_m_);
            options_.adjacent_shape_max_windows_per_endpoint_ =
                YamlGetOr<int>(adjacent_pose_gate, "shape_max_windows_per_endpoint",
                               options_.adjacent_shape_max_windows_per_endpoint_);
            options_.adjacent_shape_max_delta_p95_m_ =
                YamlGetOr<double>(adjacent_pose_gate, "max_shape_delta_p95_m",
                                  options_.adjacent_shape_max_delta_p95_m_);
            options_.adjacent_shape_max_delta_max_m_ =
                YamlGetOr<double>(adjacent_pose_gate, "max_shape_delta_max_m",
                                  options_.adjacent_shape_max_delta_max_m_);
            options_.adjacent_shape_max_delta_mean_m_ =
                YamlGetOr<double>(adjacent_pose_gate, "max_shape_delta_mean_m",
                                  options_.adjacent_shape_max_delta_mean_m_);
            if (options_.hist_cluster_id_gap_ < 0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_candidate_cluster.hist_cluster_id_gap="
                             << options_.hist_cluster_id_gap_ << ", fallback to 20";
                options_.hist_cluster_id_gap_ = 20;
            }
            if (!std::isfinite(options_.hist_cluster_radius_m_) || options_.hist_cluster_radius_m_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_candidate_cluster.hist_cluster_radius_m="
                             << options_.hist_cluster_radius_m_ << ", fallback to 5.0";
                options_.hist_cluster_radius_m_ = 5.0;
            }
            if (options_.keep_per_cluster_ < 1) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_candidate_cluster.keep_per_cluster="
                             << options_.keep_per_cluster_ << ", fallback to 1";
                options_.keep_per_cluster_ = 1;
            }
            if (options_.same_curr_kf_keep_top_ < 1) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_same_curr_kf_nms.keep_top="
                             << options_.same_curr_kf_keep_top_ << ", fallback to 1";
                options_.same_curr_kf_keep_top_ = 1;
            }
            if (options_.same_curr_kf_keep_top_ > 1) {
                LOG(WARNING) << "loop_closing.lidar_auto_same_curr_kf_nms.keep_top="
                             << options_.same_curr_kf_keep_top_
                             << " would allow multiple automatic LiDAR loops for one curr_kf; cap to 1";
                options_.same_curr_kf_keep_top_ = 1;
            }
            if (options_.same_curr_kf_fallback_top_k_ < 1) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_same_curr_kf_nms.fallback_top_k="
                             << options_.same_curr_kf_fallback_top_k_ << ", fallback to 1";
                options_.same_curr_kf_fallback_top_k_ = 1;
            }
            if (options_.source_scan_accum_max_scans_ < 1 || options_.source_scan_accum_max_scans_ > 20) {
                LOG(WARNING) << "invalid loop_closing.source_scan_accum_max_scans="
                             << options_.source_scan_accum_max_scans_ << ", clamp to [1,20]";
                options_.source_scan_accum_max_scans_ =
                    std::min(20, std::max(1, options_.source_scan_accum_max_scans_));
            }
            if (options_.source_scan_accum_min_scans_ < 1 ||
                options_.source_scan_accum_min_scans_ > options_.source_scan_accum_max_scans_) {
                LOG(WARNING) << "invalid loop_closing.source_scan_accum_min_scans="
                             << options_.source_scan_accum_min_scans_ << ", fallback to "
                             << std::min(3, options_.source_scan_accum_max_scans_);
                options_.source_scan_accum_min_scans_ = std::min(3, options_.source_scan_accum_max_scans_);
            }
            if (!std::isfinite(options_.source_scan_accum_time_sec_) ||
                options_.source_scan_accum_time_sec_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.source_scan_accum_time_sec="
                             << options_.source_scan_accum_time_sec_ << ", fallback to 0.5";
                options_.source_scan_accum_time_sec_ = 0.5;
            }
            if (options_.source_scan_accum_ref_ != "current") {
                LOG(WARNING) << "unsupported loop_closing.source_scan_accum_ref="
                             << options_.source_scan_accum_ref_ << ", fallback to current";
                options_.source_scan_accum_ref_ = "current";
            }
            if (options_.source_scan_accum_pose_type_ != "LIO") {
                LOG(WARNING) << "unsupported loop_closing.source_scan_accum_pose_type="
                             << options_.source_scan_accum_pose_type_ << ", fallback to LIO";
                options_.source_scan_accum_pose_type_ = "LIO";
            }
            if (!std::isfinite(options_.source_scan_accum_max_trans_m_) ||
                options_.source_scan_accum_max_trans_m_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.source_scan_accum_max_trans_m="
                             << options_.source_scan_accum_max_trans_m_ << ", fallback to 1.2";
                options_.source_scan_accum_max_trans_m_ = 1.2;
            }
            if (!std::isfinite(options_.source_scan_accum_max_yaw_deg_) ||
                options_.source_scan_accum_max_yaw_deg_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.source_scan_accum_max_yaw_deg="
                             << options_.source_scan_accum_max_yaw_deg_ << ", fallback to 8.0";
                options_.source_scan_accum_max_yaw_deg_ = 8.0;
            }
            if (!std::isfinite(options_.source_scan_accum_voxel_leaf_m_) ||
                options_.source_scan_accum_voxel_leaf_m_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.source_scan_accum_voxel_leaf_m="
                             << options_.source_scan_accum_voxel_leaf_m_ << ", fallback to 0.10";
                options_.source_scan_accum_voxel_leaf_m_ = 0.10;
            }
            if (options_.source_scan_accum_max_points_ < 1) {
                LOG(WARNING) << "invalid loop_closing.source_scan_accum_max_points="
                             << options_.source_scan_accum_max_points_ << ", fallback to 30000";
                options_.source_scan_accum_max_points_ = 30000;
            }
            if (options_.source_scan_accum_min_points_ < 0) {
                LOG(WARNING) << "invalid loop_closing.source_scan_accum_min_points="
                             << options_.source_scan_accum_min_points_ << ", fallback to 3000";
                options_.source_scan_accum_min_points_ = 3000;
            }
            if (!std::isfinite(options_.source_scan_accum_max_yaw_rate_degps_) ||
                options_.source_scan_accum_max_yaw_rate_degps_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.source_scan_accum_max_yaw_rate_degps="
                             << options_.source_scan_accum_max_yaw_rate_degps_ << ", fallback to 30.0";
                options_.source_scan_accum_max_yaw_rate_degps_ = 30.0;
            }
            if (!std::isfinite(options_.source_scan_accum_max_trans_rate_mps_) ||
                options_.source_scan_accum_max_trans_rate_mps_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.source_scan_accum_max_trans_rate_mps="
                             << options_.source_scan_accum_max_trans_rate_mps_ << ", fallback to 2.5";
                options_.source_scan_accum_max_trans_rate_mps_ = 2.5;
            }
            if (options_.source_scan_accum_debug_csv_.empty()) {
                LOG(WARNING) << "empty loop_closing.source_scan_accum_debug_csv, fallback to "
                                "loop_source_scan_accum_debug.csv";
                options_.source_scan_accum_debug_csv_ = "loop_source_scan_accum_debug.csv";
            }
            if (!std::isfinite(options_.init_to_ndt_max_xy_m_) || options_.init_to_ndt_max_xy_m_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_init_to_ndt_gate.max_xy_m="
                             << options_.init_to_ndt_max_xy_m_ << ", fallback to 1.5";
                options_.init_to_ndt_max_xy_m_ = 1.5;
            }
            if (!std::isfinite(options_.init_to_ndt_max_yaw_deg_) || options_.init_to_ndt_max_yaw_deg_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_init_to_ndt_gate.max_yaw_deg="
                             << options_.init_to_ndt_max_yaw_deg_ << ", fallback to 8.0";
                options_.init_to_ndt_max_yaw_deg_ = 8.0;
            }
            if (!std::isfinite(options_.init_to_ndt_max_z_m_) || options_.init_to_ndt_max_z_m_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_init_to_ndt_gate.max_z_m="
                             << options_.init_to_ndt_max_z_m_ << ", fallback to 1.0";
                options_.init_to_ndt_max_z_m_ = 1.0;
            }
            if (!std::isfinite(options_.risk_low_score_margin_th_) || options_.risk_low_score_margin_th_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_risk_combo_gate.low_score_margin_th="
                             << options_.risk_low_score_margin_th_ << ", fallback to 0.3";
                options_.risk_low_score_margin_th_ = 0.3;
            }
            if (!std::isfinite(options_.risk_near_max_range_ratio_) || options_.risk_near_max_range_ratio_ <= 0.0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_risk_combo_gate.near_max_range_ratio="
                             << options_.risk_near_max_range_ratio_ << ", fallback to 0.8";
                options_.risk_near_max_range_ratio_ = 0.8;
            }
            if (!std::isfinite(options_.risk_large_correction_trans_m_) ||
                options_.risk_large_correction_trans_m_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_risk_combo_gate.large_correction_trans_m="
                             << options_.risk_large_correction_trans_m_ << ", fallback to 0.8";
                options_.risk_large_correction_trans_m_ = 0.8;
            }
            if (!std::isfinite(options_.risk_large_correction_yaw_deg_) ||
                options_.risk_large_correction_yaw_deg_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_risk_combo_gate.large_correction_yaw_deg="
                             << options_.risk_large_correction_yaw_deg_ << ", fallback to 10.0";
                options_.risk_large_correction_yaw_deg_ = 10.0;
            }
            if (!std::isfinite(options_.risk_local_pose_delta_large_m_) ||
                options_.risk_local_pose_delta_large_m_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_risk_combo_gate.local_pose_delta_large_m="
                             << options_.risk_local_pose_delta_large_m_ << ", fallback to 0.3";
                options_.risk_local_pose_delta_large_m_ = 0.3;
            }
            if (!std::isfinite(options_.pgo_impact_max_pose_delta_near_loop_m_) ||
                options_.pgo_impact_max_pose_delta_near_loop_m_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_pgo_impact_gate.max_pose_delta_near_loop_m="
                             << options_.pgo_impact_max_pose_delta_near_loop_m_ << ", fallback to 3.0";
                options_.pgo_impact_max_pose_delta_near_loop_m_ = 3.0;
            }
            if (!std::isfinite(options_.pgo_impact_max_local_straightness_delta_m_) ||
                options_.pgo_impact_max_local_straightness_delta_m_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_pgo_impact_gate.max_local_straightness_delta_m="
                             << options_.pgo_impact_max_local_straightness_delta_m_ << ", fallback to 3.0";
                options_.pgo_impact_max_local_straightness_delta_m_ = 3.0;
            }
            if (options_.adjacent_pose_gate_scope_ != "between_loop") {
                LOG(WARNING) << "unsupported loop_closing.lidar_auto_adjacent_pose_gate.scope="
                             << options_.adjacent_pose_gate_scope_ << ", fallback to between_loop";
                options_.adjacent_pose_gate_scope_ = "between_loop";
            }
            if (options_.adjacent_pose_gate_max_pair_id_gap_ < 1) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_adjacent_pose_gate.max_pair_id_gap="
                             << options_.adjacent_pose_gate_max_pair_id_gap_ << ", fallback to 5";
                options_.adjacent_pose_gate_max_pair_id_gap_ = 5;
            }
            if (options_.adjacent_pose_gate_min_pair_count_ < 1) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_adjacent_pose_gate.min_pair_count="
                             << options_.adjacent_pose_gate_min_pair_count_ << ", fallback to 5";
                options_.adjacent_pose_gate_min_pair_count_ = 5;
            }
            if (!std::isfinite(options_.adjacent_pose_gate_max_delta_xy_m_) ||
                options_.adjacent_pose_gate_max_delta_xy_m_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_adjacent_pose_gate.max_adjacent_rel_delta_xy_m="
                             << options_.adjacent_pose_gate_max_delta_xy_m_ << ", fallback to 0.35";
                options_.adjacent_pose_gate_max_delta_xy_m_ = 0.35;
            }
            if (!std::isfinite(options_.adjacent_pose_gate_max_delta_yaw_deg_) ||
                options_.adjacent_pose_gate_max_delta_yaw_deg_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_adjacent_pose_gate.max_adjacent_rel_delta_yaw_deg="
                             << options_.adjacent_pose_gate_max_delta_yaw_deg_ << ", fallback to 5.0";
                options_.adjacent_pose_gate_max_delta_yaw_deg_ = 5.0;
            }
            if (!std::isfinite(options_.adjacent_pose_gate_p95_delta_xy_m_) ||
                options_.adjacent_pose_gate_p95_delta_xy_m_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_adjacent_pose_gate.p95_adjacent_rel_delta_xy_m="
                             << options_.adjacent_pose_gate_p95_delta_xy_m_ << ", fallback to 0.15";
                options_.adjacent_pose_gate_p95_delta_xy_m_ = 0.15;
            }
            if (!std::isfinite(options_.adjacent_pose_gate_p95_delta_yaw_deg_) ||
                options_.adjacent_pose_gate_p95_delta_yaw_deg_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_adjacent_pose_gate.p95_adjacent_rel_delta_yaw_deg="
                             << options_.adjacent_pose_gate_p95_delta_yaw_deg_ << ", fallback to 2.5";
                options_.adjacent_pose_gate_p95_delta_yaw_deg_ = 2.5;
            }
            if (options_.adjacent_pose_gate_log_top_k_ < 0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_adjacent_pose_gate.log_top_k="
                             << options_.adjacent_pose_gate_log_top_k_ << ", fallback to 5";
                options_.adjacent_pose_gate_log_top_k_ = 5;
            }
            if (options_.adjacent_shape_align_mode_ != "se2_umeyama_no_scale") {
                LOG(WARNING) << "unsupported loop_closing.lidar_auto_adjacent_pose_gate.shape_align_mode="
                             << options_.adjacent_shape_align_mode_ << ", fallback to se2_umeyama_no_scale";
                options_.adjacent_shape_align_mode_ = "se2_umeyama_no_scale";
            }
            if (options_.adjacent_shape_scope_ != "near_loop" &&
                options_.adjacent_shape_scope_ != "between_loop_full") {
                LOG(WARNING) << "unsupported loop_closing.lidar_auto_adjacent_pose_gate.shape_scope="
                             << options_.adjacent_shape_scope_ << ", fallback to near_loop";
                options_.adjacent_shape_scope_ = "near_loop";
            }
            if (options_.adjacent_shape_min_pose_count_ < 2) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_adjacent_pose_gate.shape_min_pose_count="
                             << options_.adjacent_shape_min_pose_count_ << ", fallback to 20";
                options_.adjacent_shape_min_pose_count_ = 20;
            }
            if (!std::isfinite(options_.adjacent_shape_min_path_length_m_) ||
                options_.adjacent_shape_min_path_length_m_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_adjacent_pose_gate.shape_min_path_length_m="
                             << options_.adjacent_shape_min_path_length_m_ << ", fallback to 20.0";
                options_.adjacent_shape_min_path_length_m_ = 20.0;
            }
            if (!std::isfinite(options_.adjacent_shape_endpoint_radius_m_) ||
                options_.adjacent_shape_endpoint_radius_m_ <= 0.0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_adjacent_pose_gate.shape_endpoint_radius_m="
                             << options_.adjacent_shape_endpoint_radius_m_ << ", fallback to 60.0";
                options_.adjacent_shape_endpoint_radius_m_ = 60.0;
            }
            if (!std::isfinite(options_.adjacent_shape_local_window_path_m_) ||
                options_.adjacent_shape_local_window_path_m_ <= 0.0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_adjacent_pose_gate.shape_local_window_path_m="
                             << options_.adjacent_shape_local_window_path_m_ << ", fallback to 30.0";
                options_.adjacent_shape_local_window_path_m_ = 30.0;
            }
            if (!std::isfinite(options_.adjacent_shape_local_window_stride_m_) ||
                options_.adjacent_shape_local_window_stride_m_ <= 0.0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_adjacent_pose_gate.shape_local_window_stride_m="
                             << options_.adjacent_shape_local_window_stride_m_ << ", fallback to 10.0";
                options_.adjacent_shape_local_window_stride_m_ = 10.0;
            }
            if (options_.adjacent_shape_max_windows_per_endpoint_ < 0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_adjacent_pose_gate.shape_max_windows_per_endpoint="
                             << options_.adjacent_shape_max_windows_per_endpoint_ << ", fallback to 8";
                options_.adjacent_shape_max_windows_per_endpoint_ = 8;
            }
            if (!std::isfinite(options_.adjacent_shape_max_delta_p95_m_) ||
                options_.adjacent_shape_max_delta_p95_m_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_adjacent_pose_gate.max_shape_delta_p95_m="
                             << options_.adjacent_shape_max_delta_p95_m_ << ", fallback to 0.35";
                options_.adjacent_shape_max_delta_p95_m_ = 0.35;
            }
            if (!std::isfinite(options_.adjacent_shape_max_delta_max_m_) ||
                options_.adjacent_shape_max_delta_max_m_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_adjacent_pose_gate.max_shape_delta_max_m="
                             << options_.adjacent_shape_max_delta_max_m_ << ", fallback to 0.80";
                options_.adjacent_shape_max_delta_max_m_ = 0.80;
            }
            if (!std::isfinite(options_.adjacent_shape_max_delta_mean_m_) ||
                options_.adjacent_shape_max_delta_mean_m_ < 0.0) {
                LOG(WARNING) << "invalid loop_closing.lidar_auto_adjacent_pose_gate.max_shape_delta_mean_m="
                             << options_.adjacent_shape_max_delta_mean_m_ << ", fallback to 0.20";
                options_.adjacent_shape_max_delta_mean_m_ = 0.20;
            }
            if (!std::isfinite(options_.height_noise_) || options_.height_noise_ <= 0.0) {
                LOG(WARNING) << "invalid loop_closing.height_noise=" << options_.height_noise_
                             << ", fallback to 0.1";
                options_.height_noise_ = 0.1;
            }
            options_.debug_log_enable_ = YamlGetOr<bool>(loop, "debug_log_enable", options_.debug_log_enable_);
            options_.debug_log_dir_ = YamlGetOr<std::string>(loop, "debug_log_dir", options_.debug_log_dir_);
            options_.debug_log_candidates_ =
                YamlGetOr<bool>(loop, "debug_log_candidates", options_.debug_log_candidates_);
            options_.debug_log_matches_ = YamlGetOr<bool>(loop, "debug_log_matches", options_.debug_log_matches_);
            options_.debug_log_gate_decisions_ =
                YamlGetOr<bool>(loop, "debug_log_gate_decisions", options_.debug_log_gate_decisions_);
            options_.debug_log_pgo_impact_ =
                YamlGetOr<bool>(loop, "debug_log_pgo_impact", options_.debug_log_pgo_impact_);
            options_.debug_flush_every_n_ = YamlGetOr<int>(loop, "debug_flush_every_n", options_.debug_flush_every_n_);
            options_.debug_max_suspects_ = YamlGetOr<int>(loop, "debug_max_suspects", options_.debug_max_suspects_);
            options_.debug_save_figures_ = YamlGetOr<bool>(loop, "debug_save_figures", options_.debug_save_figures_);
            options_.debug_save_submap_preview_ =
                YamlGetOr<bool>(loop, "debug_save_submap_preview", options_.debug_save_submap_preview_);
            options_.debug_loop_alignment_live_enable_ =
                YamlGetOr<bool>(loop, "debug_loop_alignment_live_enable",
                                options_.debug_loop_alignment_live_enable_);
            options_.debug_loop_alignment_live_dir_ =
                YamlGetOr<std::string>(loop, "debug_loop_alignment_live_dir",
                                       options_.debug_loop_alignment_live_dir_);
            options_.debug_loop_alignment_live_max_points_ =
                YamlGetOr<int>(loop, "debug_loop_alignment_live_max_points",
                               options_.debug_loop_alignment_live_max_points_);
            options_.debug_loop_alignment_live_save_points_ =
                YamlGetOr<bool>(loop, "debug_loop_alignment_live_save_points",
                                options_.debug_loop_alignment_live_save_points_);
            options_.debug_loop_alignment_live_save_png_ =
                YamlGetOr<bool>(loop, "debug_loop_alignment_live_save_png",
                                options_.debug_loop_alignment_live_save_png_);
            options_.debug_loop_alignment_live_max_events_ =
                YamlGetOr<int>(loop, "debug_loop_alignment_live_max_events",
                               options_.debug_loop_alignment_live_max_events_);
            if (options_.debug_log_dir_.empty()) {
                const std::string eval_dir = YamlGetOr<std::string>(root["system"], "evaluation_output_dir", "");
                options_.debug_log_dir_ = eval_dir.empty() ? "data/loop_debug" : eval_dir + "/loop_debug";
            }
            if (options_.debug_loop_alignment_live_dir_.empty()) {
                LOG(WARNING) << "empty loop_closing.debug_loop_alignment_live_dir, fallback to loop_alignment_live";
                options_.debug_loop_alignment_live_dir_ = "loop_alignment_live";
            }
            if (options_.debug_loop_alignment_live_max_points_ < 1) {
                LOG(WARNING) << "invalid loop_closing.debug_loop_alignment_live_max_points="
                             << options_.debug_loop_alignment_live_max_points_ << ", fallback to 60000";
                options_.debug_loop_alignment_live_max_points_ = 60000;
            }
            if (options_.debug_loop_alignment_live_max_events_ < 0) {
                LOG(WARNING) << "invalid loop_closing.debug_loop_alignment_live_max_events="
                             << options_.debug_loop_alignment_live_max_events_ << ", fallback to 0";
                options_.debug_loop_alignment_live_max_events_ = 0;
            }
        } catch (const std::exception& e) {
            LOG(WARNING) << "failed to read optional loop debug config: " << e.what();
        }
    }
    LOG(INFO) << "loop closing params: loop_kf_gap=" << options_.loop_kf_gap_
              << ", min_id_interval=" << options_.min_id_interval_ << ", closest_id_th=" << options_.closest_id_th_
              << ", max_range=" << options_.max_range_ << ", ndt_score_th=" << options_.ndt_score_th_
              << ", lidar_auto_ndt_inlier_gate.enable=" << options_.lidar_auto_ndt_inlier_gate_enable_
              << ", ndt_inlier_max_dist_m=" << options_.ndt_inlier_max_dist_m_
              << ", ndt_inlier_ratio_th=" << options_.ndt_inlier_ratio_th_
              << ", rk_loop_th=" << options_.rk_loop_th_ << ", with_height=" << options_.with_height_
              << ", height_noise=" << options_.height_noise_
              << ", lidar_auto_candidate_cluster.enable=" << options_.lidar_auto_candidate_cluster_enable_
              << ", hist_cluster_id_gap=" << options_.hist_cluster_id_gap_
              << ", hist_cluster_radius_m=" << options_.hist_cluster_radius_m_
              << ", keep_per_cluster=" << options_.keep_per_cluster_
              << ", lidar_auto_same_curr_kf_nms.enable=" << options_.lidar_auto_same_curr_kf_nms_enable_
              << ", same_curr_kf_keep_top=" << options_.same_curr_kf_keep_top_
              << ", same_curr_kf_fallback_enable=" << options_.same_curr_kf_fallback_enable_
              << ", same_curr_kf_fallback_top_k=" << options_.same_curr_kf_fallback_top_k_
              << ", source_scan_accum_enable=" << options_.source_scan_accum_enable_
              << ", source_scan_accum_max_scans=" << options_.source_scan_accum_max_scans_
              << ", source_scan_accum_min_scans=" << options_.source_scan_accum_min_scans_
              << ", source_scan_accum_time_sec=" << options_.source_scan_accum_time_sec_
              << ", source_scan_accum_ref=" << options_.source_scan_accum_ref_
              << ", source_scan_accum_pose_type=" << options_.source_scan_accum_pose_type_
              << ", source_scan_accum_max_trans_m=" << options_.source_scan_accum_max_trans_m_
              << ", source_scan_accum_max_yaw_deg=" << options_.source_scan_accum_max_yaw_deg_
              << ", source_scan_accum_voxel_leaf_m=" << options_.source_scan_accum_voxel_leaf_m_
              << ", source_scan_accum_max_points=" << options_.source_scan_accum_max_points_
              << ", source_scan_accum_min_points=" << options_.source_scan_accum_min_points_
              << ", source_scan_accum_max_yaw_rate_degps=" << options_.source_scan_accum_max_yaw_rate_degps_
              << ", source_scan_accum_max_trans_rate_mps=" << options_.source_scan_accum_max_trans_rate_mps_
              << ", source_scan_accum_require_monotonic_stamp="
              << options_.source_scan_accum_require_monotonic_stamp_
              << ", source_scan_accum_fallback_single=" << options_.source_scan_accum_fallback_single_
              << ", source_scan_accum_debug_enable=" << options_.source_scan_accum_debug_enable_
              << ", source_scan_accum_debug_csv=" << options_.source_scan_accum_debug_csv_
              << ", lidar_auto_init_to_ndt_gate.enable=" << options_.lidar_auto_init_to_ndt_gate_enable_
              << ", init_to_ndt_max_xy_m=" << options_.init_to_ndt_max_xy_m_
              << ", init_to_ndt_max_yaw_deg=" << options_.init_to_ndt_max_yaw_deg_
              << ", init_to_ndt_max_z_m=" << options_.init_to_ndt_max_z_m_
              << ", lidar_auto_pgo_trial_commit.enable=" << options_.lidar_auto_pgo_trial_commit_enable_
              << ", lidar_auto_risk_combo_gate.enable=" << options_.lidar_auto_risk_combo_gate_enable_
              << ", risk_low_score_margin_th=" << options_.risk_low_score_margin_th_
              << ", risk_near_max_range_ratio=" << options_.risk_near_max_range_ratio_
              << ", risk_large_correction_trans_m=" << options_.risk_large_correction_trans_m_
              << ", risk_large_correction_yaw_deg=" << options_.risk_large_correction_yaw_deg_
              << ", risk_local_pose_delta_large_m=" << options_.risk_local_pose_delta_large_m_
              << ", lidar_auto_pgo_impact_gate.enable=" << options_.lidar_auto_pgo_impact_gate_enable_
              << ", pgo_impact_max_pose_delta_near_loop_m=" << options_.pgo_impact_max_pose_delta_near_loop_m_
              << ", pgo_impact_max_local_straightness_delta_m="
              << options_.pgo_impact_max_local_straightness_delta_m_
              << ", pgo_impact_max_affected_kf_count_diagnostic_only="
              << options_.pgo_impact_max_affected_kf_count_
              << ", pgo_impact_apply_to_auto_lidar_loop_only="
              << options_.pgo_impact_apply_to_auto_lidar_loop_only_
              << ", lidar_auto_adjacent_pose_gate.enable=" << options_.lidar_auto_adjacent_pose_gate_enable_
              << ", adjacent_pose_gate_reject_on_violation="
              << options_.adjacent_pose_gate_reject_on_violation_
              << ", adjacent_pose_gate_scope=" << options_.adjacent_pose_gate_scope_
              << ", adjacent_pose_gate_max_pair_id_gap=" << options_.adjacent_pose_gate_max_pair_id_gap_
              << ", adjacent_pose_gate_min_pair_count=" << options_.adjacent_pose_gate_min_pair_count_
              << ", adjacent_pose_gate_max_delta_xy_m=" << options_.adjacent_pose_gate_max_delta_xy_m_
              << ", adjacent_pose_gate_max_delta_yaw_deg="
              << options_.adjacent_pose_gate_max_delta_yaw_deg_
              << ", adjacent_pose_gate_p95_delta_xy_m=" << options_.adjacent_pose_gate_p95_delta_xy_m_
              << ", adjacent_pose_gate_p95_delta_yaw_deg="
              << options_.adjacent_pose_gate_p95_delta_yaw_deg_
              << ", adjacent_pose_gate_log_top_k=" << options_.adjacent_pose_gate_log_top_k_
              << ", adjacent_shape_deformation_enable=" << options_.adjacent_shape_deformation_enable_
              << ", adjacent_shape_deformation_reject_on_violation="
              << options_.adjacent_shape_deformation_reject_on_violation_
              << ", adjacent_shape_align_mode=" << options_.adjacent_shape_align_mode_
              << ", adjacent_shape_scope=" << options_.adjacent_shape_scope_
              << ", adjacent_shape_min_pose_count=" << options_.adjacent_shape_min_pose_count_
              << ", adjacent_shape_min_path_length_m=" << options_.adjacent_shape_min_path_length_m_
              << ", adjacent_shape_endpoint_radius_m=" << options_.adjacent_shape_endpoint_radius_m_
              << ", adjacent_shape_local_window_path_m=" << options_.adjacent_shape_local_window_path_m_
              << ", adjacent_shape_local_window_stride_m=" << options_.adjacent_shape_local_window_stride_m_
              << ", adjacent_shape_max_windows_per_endpoint=" << options_.adjacent_shape_max_windows_per_endpoint_
              << ", adjacent_shape_max_delta_p95_m=" << options_.adjacent_shape_max_delta_p95_m_
              << ", adjacent_shape_max_delta_max_m=" << options_.adjacent_shape_max_delta_max_m_
              << ", adjacent_shape_max_delta_mean_m=" << options_.adjacent_shape_max_delta_mean_m_
              << ", debug_loop_alignment_live_enable=" << options_.debug_loop_alignment_live_enable_
              << ", debug_loop_alignment_live_dir=" << options_.debug_loop_alignment_live_dir_
              << ", debug_loop_alignment_live_max_points=" << options_.debug_loop_alignment_live_max_points_
              << ", debug_loop_alignment_live_save_points=" << options_.debug_loop_alignment_live_save_points_
              << ", debug_loop_alignment_live_save_png=" << options_.debug_loop_alignment_live_save_png_
              << ", debug_loop_alignment_live_max_events=" << options_.debug_loop_alignment_live_max_events_;

    if (options_.debug_log_enable_) {
        LoopDebugLogger::Options dbg;
        dbg.enable = options_.debug_log_enable_;
        dbg.debug_log_dir = options_.debug_log_dir_;
        dbg.log_candidates = options_.debug_log_candidates_;
        dbg.log_matches = options_.debug_log_matches_;
        dbg.log_gate_decisions = options_.debug_log_gate_decisions_;
        dbg.log_pgo_impact = options_.debug_log_pgo_impact_;
        dbg.flush_every_n = options_.debug_flush_every_n_;
        dbg.max_suspects = options_.debug_max_suspects_;
        dbg.save_figures = options_.debug_save_figures_;
        dbg.save_submap_preview = options_.debug_save_submap_preview_;
        dbg.loop_kf_gap = options_.loop_kf_gap_;
        dbg.min_id_interval = options_.min_id_interval_;
        dbg.closest_id_th = options_.closest_id_th_;
        dbg.max_range = options_.max_range_;
        dbg.ndt_score_th = options_.ndt_score_th_;
        dbg.lidar_auto_ndt_inlier_gate_enable = options_.lidar_auto_ndt_inlier_gate_enable_;
        dbg.ndt_inlier_max_dist_m = options_.ndt_inlier_max_dist_m_;
        dbg.ndt_inlier_ratio_th = options_.ndt_inlier_ratio_th_;
        dbg.rk_loop_th = options_.rk_loop_th_;
        dbg.with_height = options_.with_height_;
        dbg.height_noise = options_.height_noise_;
        dbg.lidar_auto_candidate_cluster_enable = options_.lidar_auto_candidate_cluster_enable_;
        dbg.hist_cluster_id_gap = options_.hist_cluster_id_gap_;
        dbg.hist_cluster_radius_m = options_.hist_cluster_radius_m_;
        dbg.keep_per_cluster = options_.keep_per_cluster_;
        dbg.lidar_auto_same_curr_kf_nms_enable = options_.lidar_auto_same_curr_kf_nms_enable_;
        dbg.same_curr_kf_keep_top = options_.same_curr_kf_keep_top_;
        dbg.same_curr_kf_fallback_enable = options_.same_curr_kf_fallback_enable_;
        dbg.same_curr_kf_fallback_top_k = options_.same_curr_kf_fallback_top_k_;
        dbg.source_scan_accum_enable = options_.source_scan_accum_enable_;
        dbg.source_scan_accum_max_scans = options_.source_scan_accum_max_scans_;
        dbg.source_scan_accum_min_scans = options_.source_scan_accum_min_scans_;
        dbg.source_scan_accum_time_sec = options_.source_scan_accum_time_sec_;
        dbg.source_scan_accum_ref = options_.source_scan_accum_ref_;
        dbg.source_scan_accum_pose_type = options_.source_scan_accum_pose_type_;
        dbg.source_scan_accum_max_trans_m = options_.source_scan_accum_max_trans_m_;
        dbg.source_scan_accum_max_yaw_deg = options_.source_scan_accum_max_yaw_deg_;
        dbg.source_scan_accum_voxel_leaf_m = options_.source_scan_accum_voxel_leaf_m_;
        dbg.source_scan_accum_max_points = options_.source_scan_accum_max_points_;
        dbg.source_scan_accum_min_points = options_.source_scan_accum_min_points_;
        dbg.source_scan_accum_max_yaw_rate_degps = options_.source_scan_accum_max_yaw_rate_degps_;
        dbg.source_scan_accum_max_trans_rate_mps = options_.source_scan_accum_max_trans_rate_mps_;
        dbg.source_scan_accum_require_monotonic_stamp = options_.source_scan_accum_require_monotonic_stamp_;
        dbg.source_scan_accum_fallback_single = options_.source_scan_accum_fallback_single_;
        dbg.source_scan_accum_debug_enable = options_.source_scan_accum_debug_enable_;
        dbg.source_scan_accum_debug_csv = options_.source_scan_accum_debug_csv_;
        dbg.lidar_auto_init_to_ndt_gate_enable = options_.lidar_auto_init_to_ndt_gate_enable_;
        dbg.init_to_ndt_max_xy_m = options_.init_to_ndt_max_xy_m_;
        dbg.init_to_ndt_max_yaw_deg = options_.init_to_ndt_max_yaw_deg_;
        dbg.init_to_ndt_max_z_m = options_.init_to_ndt_max_z_m_;
        dbg.lidar_auto_pgo_trial_commit_enable = options_.lidar_auto_pgo_trial_commit_enable_;
        dbg.lidar_auto_risk_combo_gate_enable = options_.lidar_auto_risk_combo_gate_enable_;
        dbg.risk_combo_reject_low_score_margin_and_large_correction =
            options_.risk_combo_reject_low_score_margin_and_large_correction_;
        dbg.risk_combo_reject_low_score_margin_and_near_max_range =
            options_.risk_combo_reject_low_score_margin_and_near_max_range_;
        dbg.risk_combo_reject_low_score_margin_and_local_pose_delta_large =
            options_.risk_combo_reject_low_score_margin_and_local_pose_delta_large_;
        dbg.risk_combo_reject_large_correction_and_near_max_range =
            options_.risk_combo_reject_large_correction_and_near_max_range_;
        dbg.risk_low_score_margin_th = options_.risk_low_score_margin_th_;
        dbg.risk_near_max_range_ratio = options_.risk_near_max_range_ratio_;
        dbg.risk_large_correction_trans_m = options_.risk_large_correction_trans_m_;
        dbg.risk_large_correction_yaw_deg = options_.risk_large_correction_yaw_deg_;
        dbg.risk_local_pose_delta_large_m = options_.risk_local_pose_delta_large_m_;
        dbg.lidar_auto_pgo_impact_gate_enable = options_.lidar_auto_pgo_impact_gate_enable_;
        dbg.pgo_impact_max_pose_delta_near_loop_m = options_.pgo_impact_max_pose_delta_near_loop_m_;
        dbg.pgo_impact_max_local_straightness_delta_m =
            options_.pgo_impact_max_local_straightness_delta_m_;
        dbg.pgo_impact_max_affected_kf_count = options_.pgo_impact_max_affected_kf_count_;
        dbg.pgo_impact_apply_to_auto_lidar_loop_only = options_.pgo_impact_apply_to_auto_lidar_loop_only_;
        dbg.lidar_auto_adjacent_pose_gate_enable = options_.lidar_auto_adjacent_pose_gate_enable_;
        dbg.adjacent_pose_gate_reject_on_violation = options_.adjacent_pose_gate_reject_on_violation_;
        dbg.adjacent_pose_gate_scope = options_.adjacent_pose_gate_scope_;
        dbg.adjacent_pose_gate_max_pair_id_gap = options_.adjacent_pose_gate_max_pair_id_gap_;
        dbg.adjacent_pose_gate_min_pair_count = options_.adjacent_pose_gate_min_pair_count_;
        dbg.adjacent_pose_gate_max_delta_xy_m = options_.adjacent_pose_gate_max_delta_xy_m_;
        dbg.adjacent_pose_gate_max_delta_yaw_deg = options_.adjacent_pose_gate_max_delta_yaw_deg_;
        dbg.adjacent_pose_gate_p95_delta_xy_m = options_.adjacent_pose_gate_p95_delta_xy_m_;
        dbg.adjacent_pose_gate_p95_delta_yaw_deg = options_.adjacent_pose_gate_p95_delta_yaw_deg_;
        dbg.adjacent_pose_gate_log_top_k = options_.adjacent_pose_gate_log_top_k_;
        dbg.adjacent_shape_deformation_enable = options_.adjacent_shape_deformation_enable_;
        dbg.adjacent_shape_deformation_reject_on_violation =
            options_.adjacent_shape_deformation_reject_on_violation_;
        dbg.adjacent_shape_align_mode = options_.adjacent_shape_align_mode_;
        dbg.adjacent_shape_scope = options_.adjacent_shape_scope_;
        dbg.adjacent_shape_min_pose_count = options_.adjacent_shape_min_pose_count_;
        dbg.adjacent_shape_min_path_length_m = options_.adjacent_shape_min_path_length_m_;
        dbg.adjacent_shape_endpoint_radius_m = options_.adjacent_shape_endpoint_radius_m_;
        dbg.adjacent_shape_local_window_path_m = options_.adjacent_shape_local_window_path_m_;
        dbg.adjacent_shape_local_window_stride_m = options_.adjacent_shape_local_window_stride_m_;
        dbg.adjacent_shape_max_windows_per_endpoint = options_.adjacent_shape_max_windows_per_endpoint_;
        dbg.adjacent_shape_max_delta_p95_m = options_.adjacent_shape_max_delta_p95_m_;
        dbg.adjacent_shape_max_delta_max_m = options_.adjacent_shape_max_delta_max_m_;
        dbg.adjacent_shape_max_delta_mean_m = options_.adjacent_shape_max_delta_mean_m_;
        loop_debug_logger_ = std::make_unique<LoopDebugLogger>();
        if (!loop_debug_logger_->Init(yaml_path, dbg, "slam")) {
            loop_debug_logger_.reset();
        }
    }

    if (options_.online_mode_) {
        LOG(INFO) << "loop closing module is running in online mode";
        kf_thread_.SetProcFunc([this](Keyframe::Ptr kf) { HandleKF(kf); });
        kf_thread_.SetName("handle loop closure");
        kf_thread_.Start();
    }
}

void LoopClosing::AddKF(Keyframe::Ptr kf) {
    if (options_.online_mode_) {
        kf_thread_.AddMessage(kf);
    } else {
        HandleKF(kf);
    }
}

void LoopClosing::HandleKF(Keyframe::Ptr kf) {
    if (kf == last_kf_) {
        return;
    }

    cur_kf_ = kf;
    all_keyframes_.emplace_back(kf);
    LogKeyframe(kf);

    // 检测回环候选
    DetectLoopCandidates();

    if (options_.verbose_) {
        LOG(INFO) << "lc: get kf " << cur_kf_->GetID() << " candi: " << candidates_.size();
    }

    // 计算回环位姿
    ComputeLoopCandidates();

    // 位姿图优化
    PoseOptimization();

    last_kf_ = kf;
}

void LoopClosing::DetectLoopCandidates() {
    candidates_.clear();

    auto& kfs_mapping = all_keyframes_;
    Keyframe::Ptr check_first = nullptr;

    if (last_loop_kf_ == nullptr) {
        last_loop_kf_ = cur_kf_;
        return;
    }

    if (last_loop_kf_ && (cur_kf_->GetID() - last_loop_kf_->GetID()) <= options_.loop_kf_gap_) {
        LOG(INFO) << "skip because last loop kf: " << last_loop_kf_->GetID();
        return;
    }

    for (auto kf : kfs_mapping) {
        if (check_first != nullptr && abs(int(kf->GetID() - check_first->GetID())) <= options_.min_id_interval_) {
            // 同条轨迹内，跳过一定的ID区间
            LogCandidateGate(kf, false, true, false, -1);
            continue;
        }

        if (abs(int(kf->GetID() - cur_kf_->GetID())) < options_.closest_id_th_) {
            /// 在同一条轨迹中，如果间隔太近，就不考虑回环
            LogCandidateGate(kf, true, false, false, -1);
            break;
        }

        Vec3d dt = kf->GetOptPose().translation() - cur_kf_->GetOptPose().translation();
        double t2d = dt.head<2>().norm();  // x-y distance
        double range_th = options_.max_range_;

        if (t2d < range_th) {
            LoopCandidate c(kf->GetID(), cur_kf_->GetID());
            c.Tij_ = kf->GetLIOPose().inverse() * cur_kf_->GetLIOPose();
            c.candidate_rank_ = static_cast<int>(candidates_.size()) + 1;
            c.xy_dist_m_ = t2d;
            c.z_diff_m_ = std::fabs(dt.z());
            c.yaw_diff_deg_ = std::fabs(PoseYawDiffDeg(kf->GetOptPose(), cur_kf_->GetOptPose()));

            candidates_.emplace_back(c);
            LogCandidateGate(kf, true, true, true, c.candidate_rank_);
            check_first = kf;
        } else {
            LogCandidateGate(kf, true, true, false, -1);
        }
    }

    if (!candidates_.empty()) {
        last_loop_kf_ = cur_kf_;
    }

    ClusterLoopCandidates();

    if (options_.verbose_ && !candidates_.empty()) {
        LOG(INFO) << "lc candi: " << candidates_.size();
    }
}

void LoopClosing::ClusterLoopCandidates() {
    if (!options_.lidar_auto_candidate_cluster_enable_ || candidates_.empty()) {
        return;
    }

    const int raw_count = static_cast<int>(candidates_.size());
    std::vector<int> parent(candidates_.size());
    std::iota(parent.begin(), parent.end(), 0);

    auto find_root = [&parent](int x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    };

    auto unite = [&parent, &find_root](int a, int b) {
        const int ra = find_root(a);
        const int rb = find_root(b);
        if (ra != rb) parent[rb] = ra;
    };

    for (size_t i = 0; i < candidates_.size(); ++i) {
        const auto& a = candidates_[i];
        const auto& pose_a = all_keyframes_.at(a.idx1_)->GetOptPose();
        for (size_t j = i + 1; j < candidates_.size(); ++j) {
            const auto& b = candidates_[j];
            const long hist_id_gap =
                std::labs(static_cast<long>(a.idx1_) - static_cast<long>(b.idx1_));
            const auto& pose_b = all_keyframes_.at(b.idx1_)->GetOptPose();
            const double hist_xy_dist =
                (pose_a.translation().head<2>() - pose_b.translation().head<2>()).norm();
            if (hist_id_gap <= options_.hist_cluster_id_gap_ || hist_xy_dist <= options_.hist_cluster_radius_m_) {
                unite(static_cast<int>(i), static_cast<int>(j));
            }
        }
    }

    std::unordered_map<int, std::vector<int>> groups_by_root;
    for (size_t i = 0; i < candidates_.size(); ++i) {
        groups_by_root[find_root(static_cast<int>(i))].push_back(static_cast<int>(i));
    }

    std::vector<std::vector<int>> groups;
    groups.reserve(groups_by_root.size());
    for (auto& item : groups_by_root) {
        groups.emplace_back(std::move(item.second));
    }
    std::sort(groups.begin(), groups.end(), [](const std::vector<int>& a, const std::vector<int>& b) {
        return a.front() < b.front();
    });

    auto better_candidate = [this](int lhs, int rhs) {
        const auto& a = candidates_[lhs];
        const auto& b = candidates_[rhs];
        const bool a_has_ndt = HasValidNdtResult(a);
        const bool b_has_ndt = HasValidNdtResult(b);
        if (a_has_ndt != b_has_ndt) return a_has_ndt;
        if (a_has_ndt && b_has_ndt && a.ndt_score_ != b.ndt_score_) return a.ndt_score_ > b.ndt_score_;
        const double a_xy = std::isfinite(a.xy_dist_m_) ? a.xy_dist_m_ : std::numeric_limits<double>::infinity();
        const double b_xy = std::isfinite(b.xy_dist_m_) ? b.xy_dist_m_ : std::numeric_limits<double>::infinity();
        if (a_xy != b_xy) return a_xy < b_xy;
        if (a.candidate_rank_ != b.candidate_rank_) return a.candidate_rank_ < b.candidate_rank_;
        return a.idx1_ < b.idx1_;
    };

    std::vector<LoopCandidate> clustered;
    clustered.reserve(candidates_.size());
    int clustered_count = 0;
    for (const auto& group : groups) {
        clustered_count += std::min(options_.keep_per_cluster_, static_cast<int>(group.size()));
    }

    int cluster_id = 0;
    for (auto& group : groups) {
        cluster_id++;
        std::stable_sort(group.begin(), group.end(), better_candidate);

        std::vector<bool> selected(candidates_.size(), false);
        const int keep_count = std::min(options_.keep_per_cluster_, static_cast<int>(group.size()));
        for (int i = 0; i < keep_count; ++i) {
            selected[group[i]] = true;
        }

        for (size_t rank = 0; rank < group.size(); ++rank) {
            const int raw_index = group[rank];
            auto& c = candidates_[raw_index];
            c.hist_cluster_id_ = cluster_id;
            c.hist_cluster_rank_ = static_cast<int>(rank) + 1;
            const auto hist_pose = all_keyframes_.at(c.idx1_)->GetOptPose();
            if (loop_debug_logger_) {
                LoopDebugLogger::CandidateClusterRow row;
                row.curr_kf_id = c.idx2_;
                row.raw_candidate_count = raw_count;
                row.clustered_candidate_count = clustered_count;
                row.cluster_id = cluster_id;
                row.hist_kf_id = c.idx1_;
                row.hist_pose_x = hist_pose.translation().x();
                row.hist_pose_y = hist_pose.translation().y();
                row.selected_or_suppressed = selected[raw_index] ? "selected" : "suppressed";
                row.suppress_reason = selected[raw_index] ? "" : "hist_cluster_suppressed";
                loop_debug_logger_->WriteCandidateCluster(row);
            }

            if (selected[raw_index]) {
                clustered.emplace_back(c);
            } else {
                LogGateDecision(c, "rejected_cluster_nms", "hist_cluster_suppressed", "", false, "", 0.0, false);
            }
        }
    }

    for (size_t i = 0; i < clustered.size(); ++i) {
        clustered[i].candidate_rank_ = static_cast<int>(i) + 1;
    }

    if (options_.verbose_) {
        LOG(INFO) << "cluster candidates: raw=" << raw_count << " clustered=" << clustered.size()
                  << " suppressed=" << raw_count - static_cast<int>(clustered.size());
    }

    candidates_.swap(clustered);
}

void LoopClosing::ComputeLoopCandidates() {
    if (candidates_.empty()) {
        return;
    }

    // 执行计算
    std::for_each(candidates_.begin(), candidates_.end(), [this](LoopCandidate& c) { ComputeForCandidate(c); });
    // 保存通过 NDT 与 init-to-NDT 门控的候选，后续每簇只选一个代表进入 PGO。
    std::vector<LoopCandidate> qualified_candidates;
    qualified_candidates.reserve(candidates_.size());
    for (size_t candidate_index = 0; candidate_index < candidates_.size(); ++candidate_index) {
        const auto& lc = candidates_[candidate_index];
        if (lc.source_accum_hard_fail_) {
            const std::string reject_reason = "SOURCE_ACCUM_FAILED";
            if (evaluation_cb_) {
                EvaluationInfo info;
                info.keyframe = cur_kf_;
                info.loop_candidate_id = lc.idx1_;
                info.loop_accepted = false;
                info.loop_score = lc.ndt_score_;
                info.loop_status = "rejected_source_accum_failed";
                info.loop_reject_reason = reject_reason;
                evaluation_cb_(info);
            }
            LogGateDecision(lc, "rejected_source_accum_failed", reject_reason,
                            lc.source_accum_fallback_reason_, false, "", 0.0, true, true);
            if (loop_debug_logger_) {
                loop_debug_logger_->WriteInitToNdt(MakeInitToNdtRow(lc, false, reject_reason));
            }
            continue;
        }
        // LOG(INFO) << "candi " << lc.idx1_ << ", " << lc.idx2_ << " s: " << lc.ndt_score_;
        const bool ndt_pass = lc.ndt_score_ > options_.ndt_score_th_;
        const bool inlier_pass = !options_.lidar_auto_ndt_inlier_gate_enable_ ||
                                 (std::isfinite(lc.ndt_inlier_ratio_) &&
                                  lc.ndt_inlier_ratio_ >= options_.ndt_inlier_ratio_th_);
        bool correction_pass = true;
        if (options_.lidar_auto_init_to_ndt_gate_enable_) {
            correction_pass = std::isfinite(lc.init_to_ndt_xy_m_) && std::isfinite(lc.init_to_ndt_yaw_deg_) &&
                              std::isfinite(lc.init_to_ndt_z_m_) &&
                              lc.init_to_ndt_xy_m_ <= options_.init_to_ndt_max_xy_m_ &&
                              lc.init_to_ndt_yaw_deg_ <= options_.init_to_ndt_max_yaw_deg_ &&
                              lc.init_to_ndt_z_m_ <= options_.init_to_ndt_max_z_m_;
        }

        bool accepted_for_ndt_stage = false;
        std::string reject_reason;
        if (ndt_pass && inlier_pass && correction_pass) {
            qualified_candidates.emplace_back(lc);
            accepted_for_ndt_stage = true;
        } else if (!ndt_pass) {
            reject_reason = "ndt_score_below_threshold";
            if (evaluation_cb_) {
                EvaluationInfo info;
                info.keyframe = cur_kf_;
                info.loop_candidate_id = lc.idx1_;
                info.loop_accepted = false;
                info.loop_score = lc.ndt_score_;
                info.loop_status = "rejected_ndt";
                info.loop_reject_reason = reject_reason;
                evaluation_cb_(info);
            }
            LogGateDecision(lc, "rejected_ndt_score", reject_reason, "", false, "", 0.0, true, correction_pass);
        } else if (!inlier_pass) {
            reject_reason = "ndt_inlier_ratio_below_threshold";
            if (evaluation_cb_) {
                EvaluationInfo info;
                info.keyframe = cur_kf_;
                info.loop_candidate_id = lc.idx1_;
                info.loop_accepted = false;
                info.loop_score = lc.ndt_score_;
                info.loop_status = "rejected_ndt_inlier";
                info.loop_reject_reason = reject_reason;
                evaluation_cb_(info);
            }
            if (options_.verbose_) {
                LOG(INFO) << "reject loop candidate by ndt_inlier_ratio_below_threshold: curr=" << lc.idx2_
                          << " hist=" << lc.idx1_ << " ndt_score=" << lc.ndt_score_
                          << " inlier_ratio=" << lc.ndt_inlier_ratio_ << "/"
                          << options_.ndt_inlier_ratio_th_
                          << " max_inlier_distance_m=" << options_.ndt_inlier_max_dist_m_;
            }
            LogGateDecision(lc, "rejected_ndt_inlier", reject_reason, "", false, "", 0.0, true, correction_pass);
        } else if (evaluation_cb_) {
            reject_reason = "init_to_ndt_too_large";
            EvaluationInfo info;
            info.keyframe = cur_kf_;
            info.loop_candidate_id = lc.idx1_;
            info.loop_accepted = false;
            info.loop_score = lc.ndt_score_;
            info.loop_status = "rejected_init_to_ndt";
            info.loop_reject_reason = reject_reason;
            evaluation_cb_(info);
        } else {
            reject_reason = "init_to_ndt_too_large";
        }

        if (ndt_pass && !correction_pass) {
            if (options_.verbose_) {
                LOG(INFO) << "reject loop candidate by init_to_ndt_too_large: curr=" << lc.idx2_
                          << " hist=" << lc.idx1_ << " ndt_score=" << lc.ndt_score_
                          << " init_to_ndt_xy=" << lc.init_to_ndt_xy_m_ << "/"
                          << options_.init_to_ndt_max_xy_m_ << " init_to_ndt_yaw_deg="
                          << lc.init_to_ndt_yaw_deg_ << "/" << options_.init_to_ndt_max_yaw_deg_
                          << " init_to_ndt_z=" << lc.init_to_ndt_z_m_ << "/"
                          << options_.init_to_ndt_max_z_m_;
            }
            LogGateDecision(lc, "rejected_init_to_ndt", "init_to_ndt_too_large", "", false, "", 0.0, true, false);
        }
        if (loop_debug_logger_) {
            loop_debug_logger_->WriteInitToNdt(MakeInitToNdtRow(lc, accepted_for_ndt_stage, reject_reason));
        }
    }

    std::vector<LoopCandidate> pgo_ready_candidates;
    pgo_ready_candidates.reserve(qualified_candidates.size());
    std::unordered_map<int, size_t> best_by_cluster;

    auto cluster_key = [](const LoopCandidate& c, size_t index) {
        return c.hist_cluster_id_ > 0 ? c.hist_cluster_id_ : -static_cast<int>(index) - 1;
    };
    auto better_final_candidate = [](const LoopCandidate& a, const LoopCandidate& b) {
        if (a.ndt_score_ != b.ndt_score_) return a.ndt_score_ > b.ndt_score_;
        const double a_xy =
            std::isfinite(a.init_to_ndt_xy_m_) ? a.init_to_ndt_xy_m_ : std::numeric_limits<double>::infinity();
        const double b_xy =
            std::isfinite(b.init_to_ndt_xy_m_) ? b.init_to_ndt_xy_m_ : std::numeric_limits<double>::infinity();
        if (a_xy != b_xy) return a_xy < b_xy;
        const int a_cluster_rank = a.hist_cluster_rank_ > 0 ? a.hist_cluster_rank_ : std::numeric_limits<int>::max();
        const int b_cluster_rank = b.hist_cluster_rank_ > 0 ? b.hist_cluster_rank_ : std::numeric_limits<int>::max();
        if (a_cluster_rank != b_cluster_rank) return a_cluster_rank < b_cluster_rank;
        if (a.candidate_rank_ != b.candidate_rank_) return a.candidate_rank_ < b.candidate_rank_;
        return a.idx1_ < b.idx1_;
    };

    for (size_t i = 0; i < qualified_candidates.size(); ++i) {
        const int key = cluster_key(qualified_candidates[i], i);
        const auto best_it = best_by_cluster.find(key);
        if (best_it == best_by_cluster.end() ||
            better_final_candidate(qualified_candidates[i], qualified_candidates[best_it->second])) {
            best_by_cluster[key] = i;
        }
    }

    std::vector<bool> selected_for_pgo(qualified_candidates.size(), false);
    for (const auto& item : best_by_cluster) {
        if (item.second < selected_for_pgo.size()) {
            selected_for_pgo[item.second] = true;
        }
    }

    for (size_t i = 0; i < qualified_candidates.size(); ++i) {
        const auto& lc = qualified_candidates[i];
        if (selected_for_pgo[i]) {
            pgo_ready_candidates.emplace_back(lc);
        } else {
            if (evaluation_cb_) {
                EvaluationInfo info;
                info.keyframe = cur_kf_;
                info.loop_candidate_id = lc.idx1_;
                info.loop_accepted = false;
                info.loop_score = lc.ndt_score_;
                info.loop_status = "rejected_cluster_nms";
                info.loop_reject_reason = "cluster_non_best_after_ndt";
                evaluation_cb_(info);
            }
            LogGateDecision(lc, "rejected_cluster_nms", "cluster_non_best_after_ndt", "", false, "", 0.0, false,
                            true);
        }
    }

    std::vector<LoopCandidate> succ_candidates;
    succ_candidates.reserve(pgo_ready_candidates.size());
    const int curr_kf_candidate_count = static_cast<int>(pgo_ready_candidates.size());
    std::vector<size_t> final_order(pgo_ready_candidates.size());
    std::iota(final_order.begin(), final_order.end(), 0);
    std::stable_sort(final_order.begin(), final_order.end(), [&](size_t lhs, size_t rhs) {
        return better_final_candidate(pgo_ready_candidates[lhs], pgo_ready_candidates[rhs]);
    });

    std::vector<bool> selected_by_same_curr_nms(pgo_ready_candidates.size(), false);
    int same_curr_trial_count = static_cast<int>(pgo_ready_candidates.size());
    if (options_.lidar_auto_same_curr_kf_nms_enable_) {
        const int configured_top_k = options_.same_curr_kf_fallback_enable_
                                         ? options_.same_curr_kf_fallback_top_k_
                                         : options_.same_curr_kf_keep_top_;
        same_curr_trial_count = std::min(configured_top_k, static_cast<int>(pgo_ready_candidates.size()));
    }
    for (size_t rank = 0; rank < final_order.size(); ++rank) {
        const size_t candidate_index = final_order[rank];
        pgo_ready_candidates[candidate_index].candidate_rank_ = static_cast<int>(rank) + 1;
        pgo_ready_candidates[candidate_index].same_curr_kf_candidate_count_ = curr_kf_candidate_count;
        if (static_cast<int>(rank) < same_curr_trial_count) {
            selected_by_same_curr_nms[candidate_index] = true;
        }
    }

    for (size_t rank = 0; rank < final_order.size(); ++rank) {
        const size_t candidate_index = final_order[rank];
        const auto& lc = pgo_ready_candidates[candidate_index];
        if (selected_by_same_curr_nms[candidate_index]) {
            succ_candidates.emplace_back(lc);
            LogGateDecision(lc, "candidate_only", "", "", false, "", 0.0, true, true, false, false, "", "", "", "",
                            std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::quiet_NaN(), -1,
                            std::numeric_limits<double>::quiet_NaN(), curr_kf_candidate_count, false, false);
        } else {
            if (evaluation_cb_) {
                EvaluationInfo info;
                info.keyframe = cur_kf_;
                info.loop_candidate_id = lc.idx1_;
                info.loop_accepted = false;
                info.loop_score = lc.ndt_score_;
                info.loop_status = "rejected_same_curr_kf_suppressed";
                info.loop_reject_reason =
                    options_.same_curr_kf_fallback_enable_ ? "same_curr_kf_fallback_rank_exceeded"
                                                           : "same_curr_kf_not_top1";
                evaluation_cb_(info);
            }
            const std::string same_curr_reject_reason =
                options_.same_curr_kf_fallback_enable_ ? "same_curr_kf_fallback_rank_exceeded"
                                                       : "same_curr_kf_not_top1";
            LogGateDecision(lc, "rejected_same_curr_kf_suppressed", same_curr_reject_reason, "", false, "", 0.0,
                            false, true, false, false, "", "", "", "",
                            std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::quiet_NaN(), -1,
                            std::numeric_limits<double>::quiet_NaN(), curr_kf_candidate_count, false, true);
        }
    }

    if (options_.verbose_) {
        LOG(INFO) << "success: " << succ_candidates.size() << "/" << candidates_.size()
                  << ", qualified_after_ndt=" << qualified_candidates.size()
                  << ", pgo_ready_before_same_curr_nms=" << pgo_ready_candidates.size()
                  << ", same_curr_trial_count=" << same_curr_trial_count
                  << ", same_curr_suppressed=" << pgo_ready_candidates.size() - succ_candidates.size();
    }

    candidates_.swap(succ_candidates);
}

CloudPtr LoopClosing::BuildNdtSourceCloud(const Keyframe::Ptr& curr_kf, LoopCandidate& c) {
    c.source_accum_enabled_ = options_.source_scan_accum_enable_;
    c.source_accum_used_ = false;
    c.source_accum_frames_ = 0;
    c.source_accum_raw_points_ = 0;
    c.source_accum_fallback_reason_.clear();
    c.source_type_ = "SINGLE_FRAME";
    c.source_scan_count_ = 0;
    c.source_time_span_sec_ = 0.0;
    c.source_points_before_downsample_ = 0;
    c.source_points_after_downsample_ = 0;
    c.source_accum_hard_fail_ = false;

    if (!curr_kf) {
        c.source_accum_fallback_reason_ = "null_current_keyframe";
        return nullptr;
    }

    CloudPtr single_frame = curr_kf->GetCloud();
    c.source_points_before_downsample_ = single_frame ? static_cast<long>(single_frame->size()) : 0;
    c.source_points_after_downsample_ = c.source_points_before_downsample_;

    if (options_.source_scan_accum_enable_) {
        SourceScanAccumProviderResult result;
        const double curr_stamp = curr_kf->GetState().timestamp_;
        const SE3 T_w_curr = curr_kf->GetLIOPose();
        bool ok = false;
        if (source_scan_accum_provider_) {
            try {
                ok = source_scan_accum_provider_(curr_stamp, T_w_curr, &result);
            } catch (const std::exception& e) {
                result.fallback_reason = std::string("ACCUM_PROVIDER_EXCEPTION:") + e.what();
                ok = false;
            } catch (...) {
                result.fallback_reason = "ACCUM_PROVIDER_EXCEPTION";
                ok = false;
            }
        } else {
            result.fallback_reason = "ACCUM_PROVIDER_UNAVAILABLE";
        }

        c.source_scan_count_ = result.scan_count;
        c.source_accum_frames_ = result.scan_count;
        c.source_time_span_sec_ = result.time_span_sec;
        c.source_accum_raw_points_ = result.points_before_downsample;
        c.source_points_before_downsample_ = result.points_before_downsample;
        c.source_points_after_downsample_ = result.points_after_downsample;

        if (ok && result.success && result.cloud_lidar && !result.cloud_lidar->empty()) {
            c.source_type_ = "RAW_SCAN_ACCUM";
            c.source_accum_used_ = true;
            c.source_accum_fallback_reason_.clear();
            return result.cloud_lidar;
        }

        c.source_accum_fallback_reason_ =
            result.fallback_reason.empty() ? "ACCUM_UNKNOWN_FAILURE" : result.fallback_reason;
        if (options_.source_scan_accum_fallback_single_) {
            c.source_type_ = "SINGLE_FRAME_FALLBACK";
            c.source_points_before_downsample_ = single_frame ? static_cast<long>(single_frame->size()) : 0;
            c.source_points_after_downsample_ = c.source_points_before_downsample_;
            return single_frame;
        }

        c.source_type_ = "SOURCE_ACCUM_FAILED";
        c.source_accum_hard_fail_ = true;
        return nullptr;
    }

    c.source_accum_fallback_reason_ = "disabled";
    return single_frame;
}

void LoopClosing::ExportLoopAlignmentLiveDebug(const LoopCandidate& c, const CloudPtr& target_world,
                                               const CloudPtr& source_lidar, const SE3& T_w_hist,
                                               const SE3& T_w_source_initial, const SE3& T_w_source_ndt) {
    if (!options_.debug_loop_alignment_live_enable_ || !options_.debug_loop_alignment_live_save_points_) {
        return;
    }
    if (!target_world || !source_lidar || target_world->empty() || source_lidar->empty()) {
        return;
    }
    if (options_.debug_loop_alignment_live_max_events_ > 0 &&
        static_cast<int>(loop_alignment_event_id_) >= options_.debug_loop_alignment_live_max_events_) {
        return;
    }

    try {
        const std::filesystem::path base_dir =
            std::filesystem::path(options_.debug_log_dir_) / options_.debug_loop_alignment_live_dir_;
        const std::filesystem::path points_dir = base_dir / "points";
        std::filesystem::create_directories(points_dir);

        const uint64_t event_id = ++loop_alignment_event_id_;
        std::ostringstream stem;
        stem << "loop_" << std::setw(5) << std::setfill('0') << event_id << "_" << c.idx2_ << "_" << c.idx1_;
        const std::filesystem::path points_path = points_dir / (stem.str() + "_points.csv");
        const std::filesystem::path bev_path = base_dir / (stem.str() + "_bev.png");
        const std::filesystem::path yside_path = base_dir / (stem.str() + "_yside.png");

        const size_t target_count = target_world->size();
        const size_t source_count = source_lidar->size();
        const size_t total_count = target_count + source_count * 2;
        const size_t max_points = static_cast<size_t>(std::max(1, options_.debug_loop_alignment_live_max_points_));
        const size_t stride = std::max<size_t>(1, static_cast<size_t>(std::ceil(static_cast<double>(total_count) /
                                                                               static_cast<double>(max_points))));

        std::ofstream points(points_path, std::ios::out | std::ios::trunc);
        if (!points.is_open()) {
            LOG(WARNING) << "failed to open loop alignment points file: " << points_path;
            return;
        }
        points << std::fixed << std::setprecision(6);
        points << "cloud,x,y,z\n";

        const SE3 T_hist_w = T_w_hist.inverse();
        const SE3 T_hist_source_initial = T_hist_w * T_w_source_initial;
        const SE3 T_hist_source_ndt = T_hist_w * T_w_source_ndt;

        auto write_cloud = [&](const std::string& name, const CloudPtr& cloud, const SE3& T_out_in,
                               size_t offset) {
            for (size_t i = 0; i < cloud->size(); ++i) {
                if (((i + offset) % stride) != 0) {
                    continue;
                }
                const auto& p = cloud->points[i];
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
                    continue;
                }
                const Vec3d out = T_out_in * Vec3d(p.x, p.y, p.z);
                points << name << "," << out.x() << "," << out.y() << "," << out.z() << "\n";
            }
        };

        write_cloud("target", target_world, T_hist_w, 0);
        write_cloud("source_before", source_lidar, T_hist_source_initial, target_count);
        write_cloud("source_after", source_lidar, T_hist_source_ndt, target_count + source_count);
        points.close();

        const std::filesystem::path events_path = base_dir / "loop_alignment_events.csv";
        const bool need_header = !loop_alignment_events_header_written_ ||
                                 !std::filesystem::exists(events_path) ||
                                 std::filesystem::file_size(events_path) == 0;
        std::ofstream events(events_path, std::ios::out | std::ios::app);
        if (!events.is_open()) {
            LOG(WARNING) << "failed to open loop alignment events file: " << events_path;
            return;
        }
        if (need_header) {
            events << "event_id,curr_kf_id,hist_kf_id,source_type,ndt_score,converged,points_file,bev_png,"
                      "yside_png,source_points,target_points,source_scan_count,source_time_span_sec,"
                      "source_points_before_downsample,source_points_after_downsample\n";
            loop_alignment_events_header_written_ = true;
        }
        events << event_id << "," << c.idx2_ << "," << c.idx1_ << "," << CsvEscapeLocal(c.source_type_) << ","
               << std::fixed << std::setprecision(9) << c.ndt_score_ << "," << (c.ndt_converged_ ? "true" : "false")
               << "," << CsvEscapeLocal(points_path.string()) << "," << CsvEscapeLocal(bev_path.string()) << ","
               << CsvEscapeLocal(yside_path.string()) << "," << c.source_points_ << "," << c.target_points_ << ","
               << c.source_scan_count_ << "," << c.source_time_span_sec_ << ","
               << c.source_points_before_downsample_ << "," << c.source_points_after_downsample_ << "\n";
        events.flush();
    } catch (const std::exception& e) {
        LOG(WARNING) << "failed to export loop alignment live debug: " << e.what();
    } catch (...) {
        LOG(WARNING) << "failed to export loop alignment live debug: unknown exception";
    }
}

void LoopClosing::ComputeForCandidate(lightning::LoopCandidate& c) {
    // LOG(INFO) << "aligning " << c.idx1_ << " with " << c.idx2_;
    const int submap_idx_range = 40;
    auto kf1 = all_keyframes_.at(c.idx1_), kf2 = all_keyframes_.at(c.idx2_);

    auto build_submap = [this](int given_id, bool build_in_world, int* used_kf_count) -> CloudPtr {
        CloudPtr submap(new PointCloudType);
        if (used_kf_count) *used_kf_count = 0;
        for (int idx = -submap_idx_range; idx < submap_idx_range; idx += 4) {
            int id = idx + given_id;
            if (id < 0 || id >= all_keyframes_.size()) {
                continue;
            }

            auto kf = all_keyframes_[id];
            CloudPtr cloud = kf->GetCloud();

            // RemoveGround(cloud, 0.1);

            if (cloud->empty()) {
                continue;
            }

            // 转到世界系下
            SE3 Twb = kf->GetOptPose();

            if (!build_in_world) {
                Twb = all_keyframes_.at(given_id)->GetOptPose().inverse() * Twb;
            }

            CloudPtr cloud_trans(new PointCloudType);
            pcl::transformPointCloud(*cloud, *cloud_trans, Twb.matrix());

            *submap += *cloud_trans;
            if (used_kf_count) (*used_kf_count)++;
        }
        return submap;
    };

    int submap_kf_count = 0;
    auto submap_kf1 = build_submap(kf1->GetID(), true, &submap_kf_count);

    CloudPtr submap_kf2 = BuildNdtSourceCloud(kf2, c);
    c.submap_kf_count_ = submap_kf_count;

    if (!submap_kf1 || !submap_kf2 || submap_kf1->empty() || submap_kf2->empty()) {
        c.ndt_score_ = 0;
        c.source_points_ = submap_kf2 ? static_cast<long>(submap_kf2->size()) : 0;
        c.target_points_ = submap_kf1 ? static_cast<long>(submap_kf1->size()) : 0;
        if (loop_debug_logger_) {
            loop_debug_logger_->WriteMatch(MakeMatchRow(c, options_));
            loop_debug_logger_->WriteSourceAccum(MakeSourceAccumRow(c, options_));
        }
        return;
    }

    Mat4f Tw2 = kf2->GetOptPose().matrix().cast<float>();
    const Mat4f initial_Tw2 = Tw2;
    const auto t0 = std::chrono::steady_clock::now();

    /// 不同分辨率下的匹配
    CloudPtr output(new PointCloudType);
    std::vector<double> res{10.0, 5.0, 2.0, 1.0};

    CloudPtr rough_map1, rough_map2;

    for (auto& r : res) {
        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        ndt.setTransformationEpsilon(0.05);
        ndt.setStepSize(0.7);
        ndt.setMaximumIterations(40);

        ndt.setResolution(r);
        rough_map1 = VoxelGrid(submap_kf1, r * 0.1);
        rough_map2 = VoxelGrid(submap_kf2, r * 0.1);
        ndt.setInputTarget(rough_map1);
        ndt.setInputSource(rough_map2);

        ndt.align(*output, Tw2);
        Tw2 = ndt.getFinalTransformation();

        c.ndt_score_ = ndt.getTransformationProbability();
        c.ndt_converged_ = ndt.hasConverged();
        c.ndt_iter_ = ndt.getFinalNumIteration();
        try {
            c.fitness_score_ = ndt.getFitnessScore();
        } catch (...) {
            c.fitness_score_ = std::numeric_limits<double>::quiet_NaN();
        }
        c.source_points_ = rough_map2 ? static_cast<long>(rough_map2->size()) : 0;
        c.target_points_ = rough_map1 ? static_cast<long>(rough_map1->size()) : 0;
    }
    const auto t1 = std::chrono::steady_clock::now();
    c.match_time_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
    c.ndt_inlier_ratio_ =
        ComputeSourceInlierRatio(rough_map1, rough_map2, Tw2, options_.ndt_inlier_max_dist_m_, &c.ndt_inlier_count_);

    Mat4d T = Tw2.cast<double>();
    Quatd q(T.block<3, 3>(0, 0));
    q.normalize();
    Vec3d t = T.block<3, 1>(0, 3);

    const SE3 ndt_pose(q, t);
    const Quatd q_initial(initial_Tw2.block<3, 3>(0, 0).cast<double>());
    const Vec3d t_initial = initial_Tw2.block<3, 1>(0, 3).cast<double>();
    const SE3 init_pose(q_initial.normalized(), t_initial);
    const SE3 T_init = kf1->GetOptPose().inverse() * init_pose;
    const SE3 T_ndt = kf1->GetOptPose().inverse() * ndt_pose;
    const SE3 init_to_ndt = T_init.inverse() * T_ndt;
    const Vec3d init_to_ndt_t = init_to_ndt.translation();

    c.Tij_ = T_ndt;
    c.correction_trans_m_ = (Tw2.block<3, 1>(0, 3) - initial_Tw2.block<3, 1>(0, 3)).norm();
    // Risk scoring uses the loop-edge relative yaw change, while init_to_ndt_yaw_deg_
    // keeps the world-heading refine delta for the local-convergence gate.
    c.correction_yaw_deg_ = std::fabs(NormalizeYawDeg(YawDeg(T_ndt) - YawDeg(T_init)));
    c.init_to_ndt_xy_m_ = init_to_ndt_t.head<2>().norm();
    c.init_to_ndt_yaw_deg_ = std::fabs(HeadingYawDiffDeg(ndt_pose, init_pose));
    c.init_to_ndt_z_m_ = std::fabs(init_to_ndt_t.z());
    ExportLoopAlignmentLiveDebug(c, submap_kf1, submap_kf2, kf1->GetOptPose(), init_pose, ndt_pose);
    if (loop_debug_logger_) {
        loop_debug_logger_->WriteMatch(MakeMatchRow(c, options_));
        loop_debug_logger_->WriteSourceAccum(MakeSourceAccumRow(c, options_));
    }

    // pcl::io::savePCDFileBinaryCompressed(
    //     "./data/lc_" + std::to_string(c.idx1_) + "_" + std::to_string(c.idx2_) + "_out.pcd", *output);
    // pcl::io::savePCDFileBinaryCompressed(
    //     "./data/lc_" + std::to_string(c.idx1_) + "_" + std::to_string(c.idx2_) + "_tgt.pcd", *rough_map1);
}

void LoopClosing::PoseOptimization() {
    struct CurrentLoopEdge {
        LoopCandidate candidate;
        std::shared_ptr<miao::EdgeSE3> edge;
        double chi2_before = std::numeric_limits<double>::quiet_NaN();
        double chi2_after = std::numeric_limits<double>::quiet_NaN();
        double pgo_before_error = std::numeric_limits<double>::quiet_NaN();
        double pgo_after_error = std::numeric_limits<double>::quiet_NaN();
        bool accepted = false;
        bool pose_writeback = false;
        bool edge_committed = false;
        std::string reject_reason_primary;
        std::vector<std::string> risk_flags;
        std::string risk_combo_gate_result;
        std::string risk_combo_reject_reason;
        LoopDebugLogger::PgoImpactRow pgo_impact;
        std::string pgo_impact_gate_result;
        std::string pgo_impact_reject_reason;
        AdjacentPoseDeformationStats adjacent_pose_stats;
    };

    auto snapshot_vertex_estimates = [this]() {
        std::unordered_map<unsigned long, SE3> snapshot;
        for (const auto& vert : kf_vert_) {
            if (vert) snapshot[vert->GetId()] = vert->Estimate();
        }
        return snapshot;
    };

    auto restore_vertex_estimates = [this](const std::unordered_map<unsigned long, SE3>& snapshot) {
        for (const auto& vert : kf_vert_) {
            if (!vert) continue;
            auto it = snapshot.find(vert->GetId());
            if (it != snapshot.end()) {
                vert->SetEstimate(it->second);
            }
        }
    };

    auto snapshot_keyframe_opt_poses = [this]() {
        std::unordered_map<unsigned long, SE3> snapshot;
        for (const auto& kf : all_keyframes_) {
            if (kf) snapshot[kf->GetID()] = kf->GetOptPose();
        }
        return snapshot;
    };

    auto restore_keyframe_opt_poses = [this](const std::unordered_map<unsigned long, SE3>& snapshot) {
        for (const auto& item : snapshot) {
            if (item.first < all_keyframes_.size() && all_keyframes_[item.first]) {
                all_keyframes_[item.first]->SetOptPose(item.second);
            }
        }
    };

    auto collect_vertex_poses = [this]() {
        std::unordered_map<unsigned long, SE3> poses;
        for (const auto& vert : kf_vert_) {
            if (vert) poses[vert->GetId()] = vert->Estimate();
        }
        return poses;
    };

    auto write_vertex_poses_to_keyframes = [this]() {
        for (const auto& vert : kf_vert_) {
            if (!vert) continue;
            const auto id = vert->GetId();
            if (id < all_keyframes_.size() && all_keyframes_[id]) {
                all_keyframes_[id]->SetOptPose(vert->Estimate());
            }
        }
    };

    auto pgo_impact_reject_reason = [this](const LoopDebugLogger::PgoImpactRow& row) {
        if (!options_.lidar_auto_pgo_impact_gate_enable_) return std::string();
        std::string reason;
        auto append_reason = [&](const std::string& item) {
            if (!reason.empty()) reason += ";";
            reason += item;
        };
        if (std::isfinite(row.max_pose_delta_near_loop_m) &&
            row.max_pose_delta_near_loop_m > options_.pgo_impact_max_pose_delta_near_loop_m_) {
            append_reason("max_pose_delta_near_loop_m");
        }
        if (std::isfinite(row.local_straightness_delta) &&
            std::fabs(row.local_straightness_delta) >
                options_.pgo_impact_max_local_straightness_delta_m_) {
            append_reason("local_straightness_delta");
        }
        return reason;
    };

    auto remove_loop_edge = [this](const std::shared_ptr<miao::EdgeSE3>& edge) {
        if (!edge) return false;
        edge->SetLevel(1);
        const bool removed = optimizer_->RemoveEdge(edge);
        edge_loops_.erase(std::remove(edge_loops_.begin(), edge_loops_.end(), edge), edge_loops_.end());
        return removed;
    };

    auto v = std::dynamic_pointer_cast<miao::VertexSE3>(optimizer_->GetVertex(cur_kf_->GetID()));
    if (!v) {
        v = std::make_shared<miao::VertexSE3>();
        v->SetId(cur_kf_->GetID());
        v->SetEstimate(cur_kf_->GetOptPose());
        if (optimizer_->AddVertex(v)) {
            kf_vert_.emplace_back(v);
        } else {
            v = std::dynamic_pointer_cast<miao::VertexSE3>(optimizer_->GetVertex(cur_kf_->GetID()));
            if (!v) {
                LOG(WARNING) << "failed to add current loop graph vertex kf=" << cur_kf_->GetID();
                for (auto& c : candidates_) {
                    LogGateDecision(c, "rejected_graph_add_failed", "current_vertex_add_failed", "", false, "",
                                    0.0, true, true, false, false,
                                    options_.lidar_auto_risk_combo_gate_enable_ ? "not_evaluated" : "disabled", "");
                }
                return;
            }
        }
    }

    /// 上一个关键帧的运动约束
    for (int i = 1; i < 3; i++) {
        int id = cur_kf_->GetID() - i;
        if (id >= 0) {
            auto last_kf = all_keyframes_[id];
            auto e = std::make_shared<miao::EdgeSE3>();
            e->SetVertex(0, optimizer_->GetVertex(last_kf->GetID()));
            e->SetVertex(1, v);

            SE3 motion = last_kf->GetLIOPose().inverse() * cur_kf_->GetLIOPose();
            e->SetMeasurement(motion);
            e->SetInformation(info_motion_);
            if (!optimizer_->AddEdge(e) && options_.verbose_) {
                LOG(WARNING) << "failed to add loop motion edge curr=" << cur_kf_->GetID() << " last=" << last_kf->GetID();
            }
        }
    }

    if (options_.with_height_) {
        /// 高度约束
        auto e = std::make_shared<miao::EdgeHeightPrior>();
        e->SetVertex(0, v);
        e->SetMeasurement(0);
        e->SetInformation(Mat1d::Identity() * 1.0 / (options_.height_noise_ * options_.height_noise_));
        if (!optimizer_->AddEdge(e) && options_.verbose_) {
            LOG(WARNING) << "failed to add loop height edge curr=" << cur_kf_->GetID();
        }
    }

    auto compute_pgo_impact = [&](const CurrentLoopEdge& loop_edge,
                                  const std::unordered_map<unsigned long, SE3>& poses_before,
                                  const std::unordered_map<unsigned long, SE3>& poses_after,
                                  double pgo_before_error, double edge_pgo_after_error) {
        const auto& c = loop_edge.candidate;
        LoopDebugLogger::PgoImpactRow pgo_row;
        pgo_row.curr_kf_id = c.idx2_;
        pgo_row.hist_kf_id = c.idx1_;
        pgo_row.loop_chi2_before = loop_edge.chi2_before;
        pgo_row.loop_chi2_after = loop_edge.chi2_after;
        pgo_row.rk_loop_th = options_.rk_loop_th_;
        pgo_row.pgo_error_before = pgo_before_error;
        pgo_row.pgo_error_after = edge_pgo_after_error;
        pgo_row.pgo_error_delta = edge_pgo_after_error - pgo_before_error;
        pgo_row.max_pose_delta_all_m = 0.0;
        pgo_row.max_pose_delta_all_yaw_deg = 0.0;
        pgo_row.max_pose_delta_near_loop_m = 0.0;
        pgo_row.max_pose_delta_near_loop_yaw_deg = 0.0;
        pgo_row.pgo_impact_gate_enable = options_.lidar_auto_pgo_impact_gate_enable_;

        const unsigned long min_id = std::min(c.idx1_, c.idx2_);
        const unsigned long max_id = std::max(c.idx1_, c.idx2_);
        double near_delta_sum = 0.0;
        std::vector<SE3> local_before;
        std::vector<SE3> local_after;
        for (const auto& item : poses_after) {
            const auto before_it = poses_before.find(item.first);
            if (before_it == poses_before.end()) continue;
            const SE3& before = before_it->second;
            const SE3& after = item.second;
            const double trans_delta = (after.translation() - before.translation()).norm();
            const double yaw_delta = std::fabs(PoseYawDiffDeg(after, before));
            pgo_row.max_pose_delta_all_m = std::max(pgo_row.max_pose_delta_all_m, trans_delta);
            pgo_row.max_pose_delta_all_yaw_deg = std::max(pgo_row.max_pose_delta_all_yaw_deg, yaw_delta);
            if (item.first >= min_id && item.first <= max_id) {
                pgo_row.max_pose_delta_near_loop_m = std::max(pgo_row.max_pose_delta_near_loop_m, trans_delta);
                pgo_row.max_pose_delta_near_loop_yaw_deg =
                    std::max(pgo_row.max_pose_delta_near_loop_yaw_deg, yaw_delta);
                near_delta_sum += trans_delta;
                pgo_row.affected_kf_count++;
                local_before.emplace_back(before);
                local_after.emplace_back(after);
            }
        }
        if (pgo_row.affected_kf_count > 0) {
            pgo_row.mean_pose_delta_near_loop_m =
                near_delta_sum / static_cast<double>(pgo_row.affected_kf_count);
        }
        auto before_hist = poses_before.find(c.idx1_);
        auto before_curr = poses_before.find(c.idx2_);
        auto after_hist = poses_after.find(c.idx1_);
        auto after_curr = poses_after.find(c.idx2_);
        if (before_hist != poses_before.end() && before_curr != poses_before.end()) {
            pgo_row.start_end_error_before_m =
                (before_curr->second.translation() - before_hist->second.translation()).norm();
        }
        if (after_hist != poses_after.end() && after_curr != poses_after.end()) {
            pgo_row.start_end_error_after_m =
                (after_curr->second.translation() - after_hist->second.translation()).norm();
        }
        pgo_row.local_straightness_before = PathStraightness(local_before);
        pgo_row.local_straightness_after = PathStraightness(local_after);
        if (std::isfinite(pgo_row.local_straightness_before) &&
            std::isfinite(pgo_row.local_straightness_after)) {
            pgo_row.local_straightness_delta =
                pgo_row.local_straightness_after - pgo_row.local_straightness_before;
        }
        return pgo_row;
    };

    std::vector<CurrentLoopEdge> current_loop_edges;
    std::vector<LoopCandidate> suppressed_after_commit;
    std::stable_sort(candidates_.begin(), candidates_.end(), [](const LoopCandidate& lhs, const LoopCandidate& rhs) {
        const int lhs_rank = lhs.candidate_rank_ > 0 ? lhs.candidate_rank_ : std::numeric_limits<int>::max();
        const int rhs_rank = rhs.candidate_rank_ > 0 ? rhs.candidate_rank_ : std::numeric_limits<int>::max();
        return lhs_rank < rhs_rank;
    });

    int cnt_outliers = 0;
    int accepted_loop_count_for_current = 0;

    for (const auto& c : candidates_) {
        if (accepted_loop_count_for_current >= options_.same_curr_kf_keep_top_) {
            suppressed_after_commit.emplace_back(c);
            continue;
        }

        auto risk_flags = BuildPrePgoRiskFlags(c, options_);
        const std::string pre_combo_reject_reason = RiskComboRejectReason(options_, risk_flags);
        if (!pre_combo_reject_reason.empty()) {
            cnt_outliers++;
            if (options_.verbose_) {
                LOG(INFO) << "risk combo reject before PGO curr=" << c.idx2_ << " hist=" << c.idx1_
                          << " reason=" << pre_combo_reject_reason << " flags=" << JoinFlags(risk_flags);
            }
            LogGateDecision(c, "rejected_risk_combo", "risk_combo_reject", "", false, JoinFlags(risk_flags),
                            static_cast<double>(risk_flags.size()), true, true, false, false,
                            "pre_reject:" + pre_combo_reject_reason, pre_combo_reject_reason);
            continue;
        }

        auto hist_vertex = optimizer_->GetVertex(c.idx1_);
        auto curr_vertex = optimizer_->GetVertex(c.idx2_);
        if (!hist_vertex || !curr_vertex) {
            std::string secondary;
            if (!hist_vertex) secondary += "hist_vertex_missing";
            if (!curr_vertex) {
                if (!secondary.empty()) secondary += ";";
                secondary += "curr_vertex_missing";
            }
            if (options_.verbose_) {
                LOG(WARNING) << "reject loop graph add curr=" << c.idx2_ << " hist=" << c.idx1_
                             << " reason=" << secondary;
            }
            LogGateDecision(c, "rejected_graph_add_failed", "missing_graph_vertex", secondary, false,
                            JoinFlags(risk_flags),
                            static_cast<double>(risk_flags.size()), true, true, false, false,
                            options_.lidar_auto_risk_combo_gate_enable_ ? "pre_pass" : "disabled", "");
            cnt_outliers++;
            continue;
        }

        const auto poses_before = snapshot_keyframe_opt_poses();
        const auto vertex_estimates_before = snapshot_vertex_estimates();
        const auto keyframe_poses_before = snapshot_keyframe_opt_poses();

        auto e = std::make_shared<miao::EdgeSE3>();
        e->SetVertex(0, hist_vertex);
        e->SetVertex(1, curr_vertex);
        e->SetMeasurement(c.Tij_);
        e->SetInformation(info_loops_);

        auto rk = std::make_shared<miao::RobustKernelCauchy>();
        rk->SetDelta(options_.rk_loop_th_);
        e->SetRobustKernel(rk);

        if (!optimizer_->AddEdge(e)) {
            if (options_.verbose_) {
                LOG(WARNING) << "failed to add trial loop edge curr=" << c.idx2_ << " hist=" << c.idx1_;
            }
            LogGateDecision(c, "rejected_graph_add_failed", "loop_add_edge_failed", "", false,
                            JoinFlags(risk_flags),
                            static_cast<double>(risk_flags.size()), true, true, false, false,
                            options_.lidar_auto_risk_combo_gate_enable_ ? "pre_pass" : "disabled", "");
            cnt_outliers++;
            continue;
        }

        CurrentLoopEdge loop_edge;
        loop_edge.candidate = c;
        loop_edge.edge = e;
        loop_edge.risk_flags = std::move(risk_flags);
        loop_edge.risk_combo_gate_result = options_.lidar_auto_risk_combo_gate_enable_ ? "pre_pass" : "disabled";

        optimizer_->InitializeOptimization();
        optimizer_->SetVerbose(false);
        optimizer_->ComputeActiveErrors();
        loop_edge.pgo_before_error = optimizer_->ActiveRobustChi2();
        loop_edge.chi2_before = loop_edge.edge->Chi2();

        optimizer_->Optimize(20);
        optimizer_->ComputeActiveErrors();
        loop_edge.pgo_after_error = optimizer_->ActiveRobustChi2();
        const auto trial_poses_after = collect_vertex_poses();

        loop_edge.chi2_after = loop_edge.edge->Chi2();
        if (loop_edge.edge->GetRobustKernel() == nullptr ||
            loop_edge.chi2_after > loop_edge.edge->GetRobustKernel()->Delta()) {
            loop_edge.accepted = false;
            loop_edge.reject_reason_primary = "robust_chi2_above_delta";
        } else {
            loop_edge.accepted = true;
        }

        if (std::isfinite(loop_edge.chi2_after) && loop_edge.chi2_after > 0.8 * options_.rk_loop_th_) {
            AddRiskFlag(loop_edge.risk_flags, "high_chi2_margin");
        }
        loop_edge.pgo_impact =
            compute_pgo_impact(loop_edge, poses_before, trial_poses_after, loop_edge.pgo_before_error,
                               loop_edge.pgo_after_error);
        if (std::isfinite(loop_edge.pgo_impact.max_pose_delta_near_loop_m) &&
            loop_edge.pgo_impact.max_pose_delta_near_loop_m > options_.risk_local_pose_delta_large_m_) {
            AddRiskFlag(loop_edge.risk_flags, "local_pose_delta_large");
        }
        if (std::isfinite(loop_edge.pgo_after_error) && std::isfinite(loop_edge.pgo_before_error) &&
            loop_edge.pgo_after_error >= loop_edge.pgo_before_error) {
            AddRiskFlag(loop_edge.risk_flags, "pgo_error_not_reduced");
        }

        const std::string impact_reject_reason = pgo_impact_reject_reason(loop_edge.pgo_impact);
        if (options_.lidar_auto_pgo_impact_gate_enable_) {
            if (impact_reject_reason.empty()) {
                loop_edge.pgo_impact_gate_result = "pass";
            } else {
                loop_edge.pgo_impact_gate_result = "reject";
                loop_edge.pgo_impact_reject_reason = impact_reject_reason;
                loop_edge.pgo_impact.pgo_impact_reject_reason = impact_reject_reason;
            }
            loop_edge.pgo_impact.pgo_impact_gate_result = loop_edge.pgo_impact_gate_result;
        } else {
            loop_edge.pgo_impact_gate_result = "disabled";
            loop_edge.pgo_impact.pgo_impact_gate_result = "disabled";
        }
        if (loop_edge.accepted && !impact_reject_reason.empty()) {
            loop_edge.accepted = false;
            loop_edge.reject_reason_primary = "pgo_impact_too_large";
            if (options_.verbose_) {
                LOG(INFO) << "pgo impact reject after trial PGO curr=" << c.idx2_ << " hist=" << c.idx1_
                          << " reason=" << impact_reject_reason
                          << " max_pose_delta_near_loop_m="
                          << loop_edge.pgo_impact.max_pose_delta_near_loop_m << "/"
                          << options_.pgo_impact_max_pose_delta_near_loop_m_
                          << " local_straightness_delta="
                          << loop_edge.pgo_impact.local_straightness_delta << "/"
                          << options_.pgo_impact_max_local_straightness_delta_m_
                          << " affected_kf_count=" << loop_edge.pgo_impact.affected_kf_count
                          << " (diagnostic_only)";
            }
        }

        loop_edge.adjacent_pose_stats =
            ComputeAdjacentPoseDeformationStats(poses_before, trial_poses_after, c.idx1_, c.idx2_, options_);
        const bool adjacent_gate_reject = EvaluateAdjacentPoseGate(options_, &loop_edge.adjacent_pose_stats);
        FillAdjacentPgoFields(loop_edge.pgo_impact, loop_edge.adjacent_pose_stats, options_);
        if (options_.verbose_ && options_.lidar_auto_adjacent_pose_gate_enable_) {
            LOG(INFO) << "[loop_adjacent_pose_gate] curr=" << c.idx2_ << " hist=" << c.idx1_
                      << " pair_count=" << loop_edge.adjacent_pose_stats.pair_count
                      << " max_xy=" << loop_edge.adjacent_pose_stats.max_delta_xy_m
                      << " worst_xy_pair=" << loop_edge.adjacent_pose_stats.worst_xy_pair_from << "->"
                      << loop_edge.adjacent_pose_stats.worst_xy_pair_to
                      << " p95_xy=" << loop_edge.adjacent_pose_stats.p95_delta_xy_m
                      << " max_yaw_deg=" << loop_edge.adjacent_pose_stats.max_delta_yaw_deg
                      << " worst_yaw_pair=" << loop_edge.adjacent_pose_stats.worst_yaw_pair_from << "->"
                      << loop_edge.adjacent_pose_stats.worst_yaw_pair_to
                      << " p95_yaw_deg=" << loop_edge.adjacent_pose_stats.p95_delta_yaw_deg
                      << " reject=" << adjacent_gate_reject
                      << " reason=" << loop_edge.adjacent_pose_stats.reject_reason
                      << " top_pairs=" << loop_edge.adjacent_pose_stats.top_delta_pairs;
            LOG(INFO) << "[loop_shape_gate] curr=" << c.idx2_ << " hist=" << c.idx1_
                      << " scope=" << loop_edge.adjacent_pose_stats.shape_scope
                      << " valid=" << loop_edge.adjacent_pose_stats.shape_valid
                      << " pose_count=" << loop_edge.adjacent_pose_stats.shape_pose_count
                      << " path=" << loop_edge.adjacent_pose_stats.shape_path_length_before_m
                      << " local_windows=" << loop_edge.adjacent_pose_stats.shape_local_window_count
                      << "/" << loop_edge.adjacent_pose_stats.shape_local_valid_window_count
                      << " max=" << loop_edge.adjacent_pose_stats.shape_delta_max_m
                      << " p95=" << loop_edge.adjacent_pose_stats.shape_delta_p95_m
                      << " mean=" << loop_edge.adjacent_pose_stats.shape_delta_mean_m
                      << " worst_kf=" << loop_edge.adjacent_pose_stats.shape_worst_kf_id
                      << " worst_endpoint=" << loop_edge.adjacent_pose_stats.shape_worst_endpoint
                      << " worst_window="
                      << loop_edge.adjacent_pose_stats.shape_worst_window_start_kf_id
                      << "->" << loop_edge.adjacent_pose_stats.shape_worst_window_end_kf_id
                      << " reject=" << (adjacent_gate_reject &&
                                        loop_edge.adjacent_pose_stats.shape_gate_result == "reject")
                      << " reason=" << loop_edge.adjacent_pose_stats.shape_gate_reject_reason;
        }
        if (loop_edge.accepted && adjacent_gate_reject) {
            loop_edge.accepted = false;
            loop_edge.reject_reason_primary = "pgo_adjacent_pose_deformation";
            if (options_.verbose_) {
                LOG(INFO) << "adjacent pose reject after trial PGO curr=" << c.idx2_ << " hist=" << c.idx1_
                          << " reason=" << loop_edge.adjacent_pose_stats.reject_reason
                          << " max_xy=" << loop_edge.adjacent_pose_stats.max_delta_xy_m << "/"
                          << options_.adjacent_pose_gate_max_delta_xy_m_
                          << " p95_xy=" << loop_edge.adjacent_pose_stats.p95_delta_xy_m << "/"
                          << options_.adjacent_pose_gate_p95_delta_xy_m_
                          << " max_yaw_deg=" << loop_edge.adjacent_pose_stats.max_delta_yaw_deg << "/"
                          << options_.adjacent_pose_gate_max_delta_yaw_deg_
                          << " p95_yaw_deg=" << loop_edge.adjacent_pose_stats.p95_delta_yaw_deg << "/"
                          << options_.adjacent_pose_gate_p95_delta_yaw_deg_;
            }
        }

        const std::string post_combo_reject_reason = RiskComboRejectReason(options_, loop_edge.risk_flags);
        if (loop_edge.accepted && !post_combo_reject_reason.empty()) {
            loop_edge.accepted = false;
            loop_edge.reject_reason_primary = "risk_combo_reject";
            loop_edge.risk_combo_reject_reason = post_combo_reject_reason;
            loop_edge.risk_combo_gate_result = "post_reject:" + post_combo_reject_reason;
            if (options_.verbose_) {
                LOG(INFO) << "risk combo reject after trial PGO curr=" << c.idx2_ << " hist=" << c.idx1_
                          << " reason=" << post_combo_reject_reason
                          << " flags=" << JoinFlags(loop_edge.risk_flags);
            }
        } else if (loop_edge.risk_combo_gate_result.empty() ||
                   loop_edge.risk_combo_gate_result == "pre_pass") {
            loop_edge.risk_combo_gate_result =
                options_.lidar_auto_risk_combo_gate_enable_ ? "post_pass" : "disabled";
        }

        if (loop_edge.accepted) {
            loop_edge.pose_writeback = true;
            loop_edge.edge_committed = true;
            loop_edge.pgo_impact.pose_writeback = true;
            loop_edge.edge->SetRobustKernel(nullptr);
            if (std::find(edge_loops_.begin(), edge_loops_.end(), loop_edge.edge) == edge_loops_.end()) {
                edge_loops_.emplace_back(loop_edge.edge);
            }
            write_vertex_poses_to_keyframes();
            accepted_loop_count_for_current++;
            current_loop_edges.emplace_back(std::move(loop_edge));
        } else {
            cnt_outliers++;
            restore_vertex_estimates(vertex_estimates_before);
            restore_keyframe_opt_poses(keyframe_poses_before);
            const bool removed = remove_loop_edge(loop_edge.edge);
            if (!removed && options_.verbose_) {
                LOG(WARNING) << "failed to remove rejected trial loop edge curr=" << loop_edge.candidate.idx2_
                             << " hist=" << loop_edge.candidate.idx1_;
            }
            current_loop_edges.emplace_back(std::move(loop_edge));
            optimizer_->InitializeOptimization();
        }
    }

    if (options_.verbose_) {
        LOG(INFO) << "loop fallback trials: attempted=" << current_loop_edges.size()
                  << ", rejected=" << cnt_outliers
                  << ", committed=" << accepted_loop_count_for_current
                  << ", suppressed_after_commit=" << suppressed_after_commit.size();
    }

    if (accepted_loop_count_for_current > 0 && loop_cb_) {
        loop_cb_();
    }

    for (const auto& c : suppressed_after_commit) {
        if (evaluation_cb_) {
            EvaluationInfo info;
            info.keyframe = cur_kf_;
            info.loop_candidate_id = c.idx1_;
            info.loop_accepted = false;
            info.loop_score = c.ndt_score_;
            info.loop_status = "rejected_same_curr_kf_suppressed";
            info.loop_reject_reason = "same_curr_kf_committed_earlier";
            evaluation_cb_(info);
        }
        LogGateDecision(c, "rejected_same_curr_kf_suppressed", "same_curr_kf_committed_earlier", "", false, "",
                        0.0, true, true, false, false, "", "", "", "",
                        std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::quiet_NaN(), -1,
                        std::numeric_limits<double>::quiet_NaN(), c.same_curr_kf_candidate_count_, false, true);
    }

    if (evaluation_cb_) {
        for (const auto& loop_edge : current_loop_edges) {
            const auto& c = loop_edge.candidate;
            EvaluationInfo info;
            info.keyframe = cur_kf_;
            info.loop_candidate_id = c.idx1_;
            info.loop_accepted = loop_edge.edge_committed;
            info.loop_score = c.ndt_score_;
            info.pgo_before_error = loop_edge.pgo_before_error;
            info.pgo_after_error = loop_edge.pgo_after_error;
            info.loop_chi2 = loop_edge.chi2_after;
            info.loop_robust_delta = options_.rk_loop_th_;
            if (loop_edge.edge_committed) {
                info.loop_status = "accepted";
            } else {
                if (loop_edge.reject_reason_primary == "risk_combo_reject") {
                    info.loop_status = "rejected_risk_combo";
                } else if (loop_edge.reject_reason_primary == "pgo_impact_too_large") {
                    info.loop_status = "rejected_pgo_impact";
                } else if (loop_edge.reject_reason_primary == "pgo_adjacent_pose_deformation") {
                    info.loop_status = "rejected_adjacent_pose";
                } else {
                    info.loop_status = "rejected_no_commit";
                }
                info.loop_reject_reason = loop_edge.reject_reason_primary.empty() ? "robust_chi2_above_delta"
                                                                                  : loop_edge.reject_reason_primary;
            }
            evaluation_cb_(info);
        }
    }

    if (loop_debug_logger_) {
        for (size_t i = 0; i < current_loop_edges.size(); ++i) {
            const auto& c = current_loop_edges[i].candidate;
            const bool accepted = current_loop_edges[i].edge_committed;

            LoopDebugLogger::EdgeRow edge_row;
            edge_row.curr_kf_id = c.idx2_;
            edge_row.hist_kf_id = c.idx1_;
            edge_row.dx = c.Tij_.translation().x();
            edge_row.dy = c.Tij_.translation().y();
            edge_row.dz = c.Tij_.translation().z();
            edge_row.dyaw_deg = YawDeg(c.Tij_);
            edge_row.information_diag = "trans_noise=" + std::to_string(options_.loop_trans_noise_) +
                                        ";rot_noise_rad=" + std::to_string(options_.loop_rot_noise_);
            edge_row.loop_chi2 = current_loop_edges[i].chi2_after;
            edge_row.rk_loop_th = options_.rk_loop_th_;
            edge_row.accepted = accepted;
            edge_row.pose_writeback = current_loop_edges[i].pose_writeback;
            edge_row.edge_committed = current_loop_edges[i].edge_committed;
            loop_debug_logger_->WriteEdge(edge_row);

            auto pgo_row = current_loop_edges[i].pgo_impact;
            pgo_row.loop_chi2_after = current_loop_edges[i].chi2_after;
            pgo_row.pose_writeback = current_loop_edges[i].pose_writeback;
            loop_debug_logger_->WritePgoImpact(pgo_row);

            const auto& risk_flags = current_loop_edges[i].risk_flags;
            const double risk_score = static_cast<double>(risk_flags.size());
            std::string final_status;
            if (accepted) {
                final_status = risk_score > 0.0 ? "accepted_high_risk" : "committed_low_risk";
            } else if (current_loop_edges[i].reject_reason_primary == "risk_combo_reject") {
                final_status = "rejected_risk_combo";
            } else if (current_loop_edges[i].reject_reason_primary == "pgo_impact_too_large") {
                final_status = "rejected_pgo_impact";
            } else if (current_loop_edges[i].reject_reason_primary == "pgo_adjacent_pose_deformation") {
                final_status = "rejected_adjacent_pose";
            } else {
                final_status = "rejected_no_commit";
            }
            const std::string reject_primary =
                accepted ? "" : (current_loop_edges[i].reject_reason_primary.empty() ? "robust_chi2_above_delta"
                                                                                     : current_loop_edges[i].reject_reason_primary);
            const bool pgo_chi2_pass = std::isfinite(current_loop_edges[i].chi2_after) &&
                                       current_loop_edges[i].chi2_after <= options_.rk_loop_th_;
            LogGateDecision(c, final_status, reject_primary, "", pgo_chi2_pass,
                            JoinFlags(risk_flags), risk_score, true, true, current_loop_edges[i].pose_writeback,
                            current_loop_edges[i].edge_committed, current_loop_edges[i].risk_combo_gate_result,
                            current_loop_edges[i].risk_combo_reject_reason,
                            current_loop_edges[i].pgo_impact_gate_result,
                            current_loop_edges[i].pgo_impact_reject_reason,
                            current_loop_edges[i].pgo_impact.max_pose_delta_near_loop_m,
                            current_loop_edges[i].pgo_impact.mean_pose_delta_near_loop_m,
                            current_loop_edges[i].pgo_impact.affected_kf_count,
                            current_loop_edges[i].pgo_impact.local_straightness_delta,
                            c.same_curr_kf_candidate_count_, true, false,
                            &current_loop_edges[i].adjacent_pose_stats);
            if ((accepted || final_status == "rejected_risk_combo") && risk_score > 0.0) {
                const auto hist = all_keyframes_.at(c.idx1_);
                const auto cur = all_keyframes_.at(c.idx2_);
                auto candidate_row = MakeCandidateRow(hist, cur, options_, c.candidate_rank_);
                candidate_row.submap_kf_count = c.submap_kf_count_;
                LoopDebugLogger::GateRow gate_row;
                gate_row.curr_kf_id = c.idx2_;
                gate_row.hist_kf_id = c.idx1_;
                gate_row.gate_ndt_score_pass = c.ndt_score_ > options_.ndt_score_th_;
                gate_row.gate_ndt_inlier_pass =
                    !options_.lidar_auto_ndt_inlier_gate_enable_ ||
                    (std::isfinite(c.ndt_inlier_ratio_) && c.ndt_inlier_ratio_ >= options_.ndt_inlier_ratio_th_);
                gate_row.ndt_inlier_ratio = c.ndt_inlier_ratio_;
                gate_row.ndt_inlier_ratio_threshold = options_.ndt_inlier_ratio_th_;
                gate_row.final_status = final_status;
                gate_row.risk_flags = JoinFlags(risk_flags);
                gate_row.risk_score = risk_score;
                gate_row.risk_combo_gate_enable = options_.lidar_auto_risk_combo_gate_enable_;
                gate_row.risk_combo_gate_result = current_loop_edges[i].risk_combo_gate_result;
                gate_row.risk_combo_reject_reason = current_loop_edges[i].risk_combo_reject_reason;
                gate_row.pgo_impact_gate_enable = options_.lidar_auto_pgo_impact_gate_enable_;
                gate_row.max_pose_delta_near_loop_m =
                    current_loop_edges[i].pgo_impact.max_pose_delta_near_loop_m;
                gate_row.mean_pose_delta_near_loop_m =
                    current_loop_edges[i].pgo_impact.mean_pose_delta_near_loop_m;
                gate_row.affected_kf_count = current_loop_edges[i].pgo_impact.affected_kf_count;
                gate_row.local_straightness_delta =
                    current_loop_edges[i].pgo_impact.local_straightness_delta;
                gate_row.pgo_impact_gate_result = current_loop_edges[i].pgo_impact_gate_result;
                gate_row.pgo_impact_reject_reason = current_loop_edges[i].pgo_impact_reject_reason;
                FillAdjacentGateFields(gate_row, current_loop_edges[i].adjacent_pose_stats, options_);
                gate_row.committed = current_loop_edges[i].edge_committed;
                gate_row.pose_writeback = current_loop_edges[i].pose_writeback;
                gate_row.edge_committed = current_loop_edges[i].edge_committed;
                gate_row.curr_kf_candidate_count = c.same_curr_kf_candidate_count_;
                gate_row.selected_for_pgo_trial = true;
                gate_row.suppressed_by_same_curr_kf_nms = false;
                gate_row.candidate_rank = c.candidate_rank_;
                loop_debug_logger_->WriteSuspect(candidate_row, MakeMatchRow(c, options_), edge_row, pgo_row, gate_row);
            }
        }
    }

    LOG(INFO) << "optimize finished, loops: " << edge_loops_.size();

    // LOG(INFO) << "lc: cur kf " << cur_kf_->GetID() << ", opt: " << cur_kf_->GetOptPose().translation().transpose()
    //           << ", lio: " << cur_kf_->GetLIOPose().translation().transpose();
}

void LoopClosing::LogKeyframe(const Keyframe::Ptr& kf) {
    if (!loop_debug_logger_) return;
    loop_debug_logger_->WriteKeyframe(MakeKeyframeRow(kf, last_kf_));
}

void LoopClosing::LogCandidateGate(const Keyframe::Ptr& hist_kf, bool pass_id_gap, bool pass_closest_id,
                                   bool pass_range, int candidate_rank) {
    if (!loop_debug_logger_ || !hist_kf || !cur_kf_) return;
    auto row = MakeCandidateRow(hist_kf, cur_kf_, options_, candidate_rank);
    row.pass_id_gap = pass_id_gap;
    row.pass_closest_id = pass_closest_id;
    row.pass_range = pass_range;
    loop_debug_logger_->WriteCandidate(row);

    LoopDebugLogger::GateRow gate;
    gate.curr_kf_id = row.curr_kf_id;
    gate.hist_kf_id = row.hist_kf_id;
    gate.gate_id_gap_pass = pass_id_gap && pass_closest_id;
    gate.gate_range_pass = pass_range;
    gate.gate_yaw_pass = true;
    gate.gate_z_pass = true;
    gate.gate_ndt_score_pass = false;
    gate.gate_ndt_inlier_pass = true;
    gate.ndt_inlier_ratio = std::numeric_limits<double>::quiet_NaN();
    gate.ndt_inlier_ratio_threshold = options_.ndt_inlier_ratio_th_;
    gate.gate_correction_pass = true;
    gate.gate_nms_pass = true;
    gate.gate_pgo_chi2_pass = false;
    gate.risk_combo_gate_enable = options_.lidar_auto_risk_combo_gate_enable_;
    gate.risk_combo_gate_result = "not_evaluated";
    gate.pgo_impact_gate_enable = options_.lidar_auto_pgo_impact_gate_enable_;
    gate.pgo_impact_gate_result = "not_evaluated";
    gate.curr_kf_candidate_count = candidates_.empty() ? -1 : static_cast<int>(candidates_.size());
    gate.candidate_rank = candidate_rank;
    if (!pass_id_gap) {
        gate.final_status = "rejected_id_gap";
        gate.reject_reason_primary = "min_id_interval";
    } else if (!pass_closest_id) {
        gate.final_status = "rejected_id_gap";
        gate.reject_reason_primary = "closest_id_break";
    } else if (!pass_range) {
        gate.final_status = "rejected_range";
        gate.reject_reason_primary = "xy_distance_above_max_range";
    } else {
        gate.final_status = "candidate_only";
    }
    loop_debug_logger_->WriteGateDecision(gate);
}

void LoopClosing::LogGateDecision(const LoopCandidate& c, const std::string& final_status,
                                  const std::string& reject_primary, const std::string& reject_secondary,
                                  bool pgo_pass, const std::string& risk_flags, double risk_score, bool nms_pass,
                                  bool correction_pass, bool pose_writeback, bool edge_committed,
                                  const std::string& risk_combo_gate_result,
                                  const std::string& risk_combo_reject_reason,
                                  const std::string& pgo_impact_gate_result,
                                  const std::string& pgo_impact_reject_reason,
                                  double max_pose_delta_near_loop_m,
                                  double mean_pose_delta_near_loop_m,
                                  long affected_kf_count,
                                  double local_straightness_delta,
                                  int curr_kf_candidate_count,
                                  bool selected_for_pgo_trial,
                                  bool suppressed_by_same_curr_kf_nms,
                                  const AdjacentPoseDeformationStats* adjacent_pose_stats) {
    if (!loop_debug_logger_) return;
    LoopDebugLogger::GateRow gate;
    gate.curr_kf_id = c.idx2_;
    gate.hist_kf_id = c.idx1_;
    gate.gate_id_gap_pass = true;
    gate.gate_range_pass = std::isfinite(c.xy_dist_m_) ? c.xy_dist_m_ < options_.max_range_ : true;
    gate.gate_yaw_pass = true;
    gate.gate_z_pass = true;
    gate.gate_ndt_score_pass = c.ndt_score_ > options_.ndt_score_th_;
    gate.gate_ndt_inlier_pass =
        !options_.lidar_auto_ndt_inlier_gate_enable_ ||
        (std::isfinite(c.ndt_inlier_ratio_) && c.ndt_inlier_ratio_ >= options_.ndt_inlier_ratio_th_);
    gate.ndt_inlier_ratio = c.ndt_inlier_ratio_;
    gate.ndt_inlier_ratio_threshold = options_.ndt_inlier_ratio_th_;
    gate.gate_correction_pass = correction_pass;
    gate.gate_nms_pass = nms_pass;
    gate.gate_pgo_chi2_pass = pgo_pass;
    gate.final_status = final_status;
    gate.reject_reason_primary = reject_primary;
    gate.reject_reason_secondary = reject_secondary;
    gate.risk_flags = risk_flags;
    gate.risk_score = risk_score;
    gate.risk_combo_gate_enable = options_.lidar_auto_risk_combo_gate_enable_;
    gate.risk_combo_gate_result = risk_combo_gate_result.empty() ? "not_evaluated" : risk_combo_gate_result;
    gate.risk_combo_reject_reason = risk_combo_reject_reason;
    gate.pgo_impact_gate_enable = options_.lidar_auto_pgo_impact_gate_enable_;
    gate.max_pose_delta_near_loop_m = max_pose_delta_near_loop_m;
    gate.mean_pose_delta_near_loop_m = mean_pose_delta_near_loop_m;
    gate.affected_kf_count = affected_kf_count;
    gate.local_straightness_delta = local_straightness_delta;
    gate.pgo_impact_gate_result =
        pgo_impact_gate_result.empty() ? "not_evaluated" : pgo_impact_gate_result;
    gate.pgo_impact_reject_reason = pgo_impact_reject_reason;
    if (adjacent_pose_stats != nullptr) {
        FillAdjacentGateFields(gate, *adjacent_pose_stats, options_);
    } else {
        gate.adjacent_pose_gate_enable = options_.lidar_auto_adjacent_pose_gate_enable_;
        gate.adjacent_pose_gate_reject_on_violation = options_.adjacent_pose_gate_reject_on_violation_;
        gate.adjacent_pose_gate_result = options_.lidar_auto_adjacent_pose_gate_enable_ ? "not_evaluated" : "disabled";
    }
    gate.committed = edge_committed;
    gate.pose_writeback = pose_writeback;
    gate.edge_committed = edge_committed;
    gate.curr_kf_candidate_count =
        curr_kf_candidate_count >= 0 ? curr_kf_candidate_count
                                     : (c.same_curr_kf_candidate_count_ >= 0
                                            ? c.same_curr_kf_candidate_count_
                                            : static_cast<int>(candidates_.size()));
    gate.selected_for_pgo_trial = selected_for_pgo_trial;
    gate.suppressed_by_same_curr_kf_nms = suppressed_by_same_curr_kf_nms;
    gate.candidate_rank = c.candidate_rank_;
    loop_debug_logger_->WriteGateDecision(gate);
}

}  // namespace lightning
