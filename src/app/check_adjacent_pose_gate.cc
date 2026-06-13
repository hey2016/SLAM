#include "core/loop_closing/loop_closing.h"

#include <glog/logging.h>

#include <cmath>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

lightning::SE3 PoseXYYaw(double x, double y, double yaw_deg) {
    const double yaw_rad = yaw_deg * M_PI / 180.0;
    Eigen::AngleAxisd yaw(yaw_rad, lightning::Vec3d::UnitZ());
    return lightning::SE3(lightning::Quatd(yaw), lightning::Vec3d(x, y, 0.0));
}

std::unordered_map<unsigned long, lightning::SE3> MakeLinePoses(int count) {
    std::unordered_map<unsigned long, lightning::SE3> poses;
    for (int i = 0; i < count; ++i) {
        poses.emplace(static_cast<unsigned long>(i), PoseXYYaw(static_cast<double>(i), 0.0, 0.0));
    }
    return poses;
}

std::unordered_map<unsigned long, lightning::SE3> MakeArcPoses(int count, double curvature) {
    std::unordered_map<unsigned long, lightning::SE3> poses;
    const double center = 0.5 * static_cast<double>(count - 1);
    for (int i = 0; i < count; ++i) {
        const double x = static_cast<double>(i);
        const double y = curvature * (x - center) * (x - center);
        poses.emplace(static_cast<unsigned long>(i), PoseXYYaw(x, y, 0.0));
    }
    return poses;
}

std::unordered_map<unsigned long, lightning::SE3> MakeMiddleDriftPoses(int count) {
    auto poses = MakeLinePoses(count);
    for (int i = 0; i < count; ++i) {
        if (i < 45 || i > 75) {
            continue;
        }
        const double phase = (static_cast<double>(i - 45) / 30.0) * M_PI;
        const double y = 4.0 * std::sin(phase);
        poses[static_cast<unsigned long>(i)] = PoseXYYaw(static_cast<double>(i), y, 0.0);
    }
    return poses;
}

std::unordered_map<unsigned long, lightning::SE3> MakeEndpointDriftPoses(int count) {
    auto poses = MakeLinePoses(count);
    for (int i = 0; i < std::min(count, 35); ++i) {
        const double x = static_cast<double>(i);
        const double y = 0.035 * x * x;
        poses[static_cast<unsigned long>(i)] = PoseXYYaw(x, y, 0.0);
    }
    return poses;
}

std::unordered_map<unsigned long, lightning::SE3> TransformPoses(
    const std::unordered_map<unsigned long, lightning::SE3>& poses, double tx, double ty, double yaw_deg) {
    std::unordered_map<unsigned long, lightning::SE3> out;
    const auto T = PoseXYYaw(tx, ty, yaw_deg);
    for (const auto& item : poses) {
        out.emplace(item.first, T * item.second);
    }
    return out;
}

bool Near(double lhs, double rhs, double eps = 1e-9) {
    return std::fabs(lhs - rhs) <= eps;
}

lightning::LoopClosing::Options ShapeOptions() {
    lightning::LoopClosing::Options options;
    options.lidar_auto_adjacent_pose_gate_enable_ = true;
    options.adjacent_pose_gate_reject_on_violation_ = true;
    options.adjacent_pose_gate_scope_ = "between_loop";
    options.adjacent_pose_gate_max_pair_id_gap_ = 5;
    options.adjacent_pose_gate_min_pair_count_ = 5;
    options.adjacent_pose_gate_max_delta_xy_m_ = 100.0;
    options.adjacent_pose_gate_max_delta_yaw_deg_ = 100.0;
    options.adjacent_pose_gate_p95_delta_xy_m_ = 100.0;
    options.adjacent_pose_gate_p95_delta_yaw_deg_ = 100.0;
    options.adjacent_shape_deformation_enable_ = true;
    options.adjacent_shape_deformation_reject_on_violation_ = true;
    options.adjacent_shape_align_mode_ = "se2_umeyama_no_scale";
    options.adjacent_shape_scope_ = "near_loop";
    options.adjacent_shape_min_pose_count_ = 20;
    options.adjacent_shape_min_path_length_m_ = 20.0;
    options.adjacent_shape_endpoint_radius_m_ = 60.0;
    options.adjacent_shape_local_window_path_m_ = 30.0;
    options.adjacent_shape_local_window_stride_m_ = 10.0;
    options.adjacent_shape_max_windows_per_endpoint_ = 8;
    options.adjacent_shape_max_delta_p95_m_ = 0.35;
    options.adjacent_shape_max_delta_max_m_ = 0.80;
    options.adjacent_shape_max_delta_mean_m_ = 0.20;
    return options;
}

bool ExpectPass(const std::string& name,
                const std::unordered_map<unsigned long, lightning::SE3>& before,
                const std::unordered_map<unsigned long, lightning::SE3>& after,
                const lightning::LoopClosing::Options& options, int expected_pair_count) {
    auto stats = lightning::LoopClosing::ComputeAdjacentPoseDeformationStats(before, after, 0, 9, options);
    const bool reject = lightning::LoopClosing::EvaluateAdjacentPoseGate(options, &stats);
    if (reject || stats.pair_count != expected_pair_count || !Near(stats.max_delta_xy_m, 0.0, 1e-6) ||
        !Near(stats.max_delta_yaw_deg, 0.0, 1e-6)) {
        std::cerr << name << " failed: reject=" << reject << " pairs=" << stats.pair_count
                  << " max_xy=" << stats.max_delta_xy_m << " max_yaw=" << stats.max_delta_yaw_deg
                  << " reason=" << stats.reject_reason << "\n";
        return false;
    }
    return true;
}

bool ExpectRejectReason(const std::string& name,
                        const std::unordered_map<unsigned long, lightning::SE3>& before,
                        const std::unordered_map<unsigned long, lightning::SE3>& after,
                        const lightning::LoopClosing::Options& options,
                        const std::string& expected_reason) {
    auto stats = lightning::LoopClosing::ComputeAdjacentPoseDeformationStats(before, after, 0, 9, options);
    const bool reject = lightning::LoopClosing::EvaluateAdjacentPoseGate(options, &stats);
    if (!reject || stats.reject_reason != expected_reason) {
        std::cerr << name << " failed: reject=" << reject << " reason=" << stats.reject_reason
                  << " expected=" << expected_reason << " max_xy=" << stats.max_delta_xy_m
                  << " max_yaw=" << stats.max_delta_yaw_deg << "\n";
        return false;
    }
    return true;
}

bool ExpectShapePass(const std::string& name,
                     const std::unordered_map<unsigned long, lightning::SE3>& before,
                     const std::unordered_map<unsigned long, lightning::SE3>& after,
                     const lightning::LoopClosing::Options& options) {
    auto stats = lightning::LoopClosing::ComputeAdjacentPoseDeformationStats(
        before, after, 0, static_cast<unsigned long>(before.size() - 1), options);
    const bool reject = lightning::LoopClosing::EvaluateAdjacentPoseGate(options, &stats);
    if (reject || !stats.shape_valid || stats.shape_delta_max_m > 1e-6 ||
        stats.shape_delta_p95_m > 1e-6 || stats.shape_delta_mean_m > 1e-6) {
        std::cerr << name << " failed: reject=" << reject << " valid=" << stats.shape_valid
                  << " max=" << stats.shape_delta_max_m << " p95=" << stats.shape_delta_p95_m
                  << " mean=" << stats.shape_delta_mean_m << " reason=" << stats.shape_gate_reject_reason
                  << "\n";
        return false;
    }
    return true;
}

bool ExpectShapeReject(const std::string& name,
                       const std::unordered_map<unsigned long, lightning::SE3>& before,
                       const std::unordered_map<unsigned long, lightning::SE3>& after,
                       const lightning::LoopClosing::Options& options,
                       const std::string& expected_reason) {
    auto stats = lightning::LoopClosing::ComputeAdjacentPoseDeformationStats(
        before, after, 0, static_cast<unsigned long>(before.size() - 1), options);
    const bool reject = lightning::LoopClosing::EvaluateAdjacentPoseGate(options, &stats);
    if (!reject || !stats.shape_valid || stats.shape_gate_reject_reason != expected_reason) {
        std::cerr << name << " failed: reject=" << reject << " valid=" << stats.shape_valid
                  << " shape_reason=" << stats.shape_gate_reject_reason << " expected=" << expected_reason
                  << " max=" << stats.shape_delta_max_m << " p95=" << stats.shape_delta_p95_m
                  << " mean=" << stats.shape_delta_mean_m << "\n";
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    (void)argc;
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::WARNING;

    lightning::LoopClosing::Options options;
    options.lidar_auto_adjacent_pose_gate_enable_ = true;
    options.adjacent_pose_gate_reject_on_violation_ = true;
    options.adjacent_pose_gate_scope_ = "between_loop";
    options.adjacent_pose_gate_max_pair_id_gap_ = 5;
    options.adjacent_pose_gate_min_pair_count_ = 5;
    options.adjacent_pose_gate_max_delta_xy_m_ = 0.35;
    options.adjacent_pose_gate_max_delta_yaw_deg_ = 5.0;
    options.adjacent_pose_gate_p95_delta_xy_m_ = 0.15;
    options.adjacent_pose_gate_p95_delta_yaw_deg_ = 2.5;
    options.adjacent_pose_gate_log_top_k_ = 5;

    const auto before = MakeLinePoses(10);

    if (!ExpectPass("no_deformation", before, before, options, 9)) {
        return 1;
    }

    auto translated = before;
    for (auto& item : translated) {
        item.second = lightning::SE3(lightning::Quatd::Identity(), lightning::Vec3d(1.0, 0.0, 0.0)) * item.second;
    }
    if (!ExpectPass("rigid_translation", before, translated, options, 9)) {
        return 1;
    }

    auto xy_split = before;
    xy_split[5] = PoseXYYaw(5.6, 0.0, 0.0);
    if (!ExpectRejectReason("xy_pair_deformation", before, xy_split, options, "adjacent_rel_delta_xy")) {
        return 1;
    }

    auto yaw_jump = before;
    yaw_jump[5] = PoseXYYaw(5.0, 0.0, 8.0);
    if (!ExpectRejectReason("yaw_pair_deformation", before, yaw_jump, options, "adjacent_rel_delta_yaw")) {
        return 1;
    }

    std::unordered_map<unsigned long, lightning::SE3> unordered_before;
    std::unordered_map<unsigned long, lightning::SE3> unordered_after;
    for (int id : {8, 1, 5, 0, 9, 3, 7, 2, 6, 4}) {
        unordered_before.emplace(static_cast<unsigned long>(id), before.at(static_cast<unsigned long>(id)));
        unordered_after.emplace(static_cast<unsigned long>(id), before.at(static_cast<unsigned long>(id)));
    }
    if (!ExpectPass("unordered_input", unordered_before, unordered_after, options, 9)) {
        return 1;
    }

    auto few_before = MakeLinePoses(3);
    auto few_after = few_before;
    options.adjacent_pose_gate_min_pair_count_ = 5;
    auto few_stats = lightning::LoopClosing::ComputeAdjacentPoseDeformationStats(few_before, few_after, 0, 2, options);
    const bool few_reject = lightning::LoopClosing::EvaluateAdjacentPoseGate(options, &few_stats);
    if (few_reject || few_stats.valid || few_stats.reject_reason != "insufficient_adjacent_pair_count") {
        std::cerr << "insufficient_pairs failed: reject=" << few_reject << " valid=" << few_stats.valid
                  << " pairs=" << few_stats.pair_count << " reason=" << few_stats.reject_reason << "\n";
        return 1;
    }

    const auto long_before = MakeLinePoses(30);
    auto shape_options = ShapeOptions();
    if (!ExpectShapePass("shape_rigid_translation", long_before,
                         TransformPoses(long_before, 5.0, -2.0, 0.0), shape_options)) {
        return 1;
    }
    if (!ExpectShapePass("shape_rigid_rotation", long_before,
                         TransformPoses(long_before, 0.0, 0.0, 10.0), shape_options)) {
        return 1;
    }
    if (!ExpectShapePass("shape_rigid_translation_rotation", long_before,
                         TransformPoses(long_before, 5.0, -2.0, 15.0), shape_options)) {
        return 1;
    }
    if (!ExpectShapeReject("shape_arc_deformation", long_before, MakeArcPoses(30, 0.04),
                           shape_options, "shape_delta_max")) {
        return 1;
    }

    auto local_shape_options = shape_options;
    local_shape_options.adjacent_shape_min_pose_count_ = 10;
    local_shape_options.adjacent_shape_min_path_length_m_ = 10.0;
    local_shape_options.adjacent_shape_endpoint_radius_m_ = 20.0;
    local_shape_options.adjacent_shape_local_window_path_m_ = 15.0;
    local_shape_options.adjacent_shape_local_window_stride_m_ = 5.0;
    local_shape_options.adjacent_shape_max_windows_per_endpoint_ = 4;
    const auto very_long_before = MakeLinePoses(120);
    if (!ExpectShapePass("shape_near_loop_ignores_middle_global_drift",
                         very_long_before, MakeMiddleDriftPoses(120), local_shape_options)) {
        return 1;
    }
    if (!ExpectShapeReject("shape_near_loop_endpoint_deformation",
                           very_long_before, MakeEndpointDriftPoses(120),
                           local_shape_options, "shape_delta_max")) {
        return 1;
    }

    auto full_scope_options = local_shape_options;
    full_scope_options.adjacent_shape_scope_ = "between_loop_full";
    if (!ExpectShapeReject("shape_full_scope_middle_global_drift",
                           very_long_before, MakeMiddleDriftPoses(120),
                           full_scope_options, "shape_delta_max")) {
        return 1;
    }

    auto monitor_options = shape_options;
    monitor_options.adjacent_shape_deformation_reject_on_violation_ = false;
    auto monitor_stats = lightning::LoopClosing::ComputeAdjacentPoseDeformationStats(
        long_before, MakeArcPoses(30, 0.04), 0, 29, monitor_options);
    const bool monitor_reject = lightning::LoopClosing::EvaluateAdjacentPoseGate(monitor_options, &monitor_stats);
    if (monitor_reject || monitor_stats.shape_gate_result != "monitor_only_shape_deformation_violation" ||
        monitor_stats.shape_gate_reject_reason != "shape_delta_max") {
        std::cerr << "shape_monitor_only failed: reject=" << monitor_reject
                  << " result=" << monitor_stats.shape_gate_result
                  << " reason=" << monitor_stats.shape_gate_reject_reason << "\n";
        return 1;
    }

    auto shape_few_before = MakeLinePoses(10);
    auto shape_few_stats = lightning::LoopClosing::ComputeAdjacentPoseDeformationStats(
        shape_few_before, shape_few_before, 0, 9, shape_options);
    const bool shape_few_reject = lightning::LoopClosing::EvaluateAdjacentPoseGate(shape_options, &shape_few_stats);
    if (shape_few_reject || shape_few_stats.shape_valid ||
        shape_few_stats.shape_gate_reject_reason != "insufficient_shape_pose_count") {
        std::cerr << "shape_insufficient_pose_count failed: reject=" << shape_few_reject
                  << " valid=" << shape_few_stats.shape_valid
                  << " reason=" << shape_few_stats.shape_gate_reject_reason << "\n";
        return 1;
    }

    auto short_path_options = shape_options;
    short_path_options.adjacent_shape_min_pose_count_ = 5;
    short_path_options.adjacent_shape_min_path_length_m_ = 20.0;
    auto short_path_before = MakeLinePoses(10);
    auto short_path_stats = lightning::LoopClosing::ComputeAdjacentPoseDeformationStats(
        short_path_before, short_path_before, 0, 9, short_path_options);
    const bool short_path_reject = lightning::LoopClosing::EvaluateAdjacentPoseGate(short_path_options,
                                                                                   &short_path_stats);
    if (short_path_reject || short_path_stats.shape_valid ||
        short_path_stats.shape_gate_reject_reason != "insufficient_shape_path_length") {
        std::cerr << "shape_insufficient_path_length failed: reject=" << short_path_reject
                  << " valid=" << short_path_stats.shape_valid
                  << " reason=" << short_path_stats.shape_gate_reject_reason << "\n";
        return 1;
    }

    std::cout << "adjacent pose gate check passed\n";
    return 0;
}
