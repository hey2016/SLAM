#include "core/lio/laser_mapping.h"

#include <glog/logging.h>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

lightning::SE3 PoseXYYaw(double x, double y, double yaw_deg) {
    const double yaw_rad = yaw_deg * M_PI / 180.0;
    Eigen::AngleAxisd yaw(yaw_rad, lightning::Vec3d::UnitZ());
    return lightning::SE3(lightning::Quatd(yaw), lightning::Vec3d(x, y, 0.0));
}

lightning::CloudPtr MakeCloud(size_t point_count = 1) {
    lightning::CloudPtr cloud(new lightning::PointCloudType());
    cloud->reserve(point_count);
    for (size_t i = 0; i < point_count; ++i) {
        lightning::PointType p;
        p.x = static_cast<float>(i) * 0.01f;
        p.y = 0.0f;
        p.z = 0.0f;
        p.intensity = 1.0f;
        cloud->push_back(p);
    }
    return cloud;
}

lightning::LaserMapping::RawScanCacheItem MakeItem(int64_t seq, double stamp, const lightning::SE3& pose,
                                                    bool valid_pose = true, size_t point_count = 1) {
    lightning::LaserMapping::RawScanCacheItem item;
    item.seq = seq;
    item.stamp_sec = stamp;
    item.cloud_lidar = MakeCloud(point_count);
    item.pose_lio_lidar_in_world = pose;
    item.has_lio_pose = valid_pose;
    return item;
}

bool ExpectReason(const std::string& name, const std::vector<lightning::LaserMapping::RawScanCacheItem>& cache,
                  double curr_stamp, const lightning::SE3& curr_pose,
                  const lightning::LaserMapping::Options& options,
                  const std::string& expected_reason, bool expected_valid) {
    lightning::LaserMapping::SourceScanAccumSelection selection;
    const bool ok = lightning::LaserMapping::SelectSourceScanAccumCandidateFromCache(cache, curr_stamp, curr_pose,
                                                                                     options, &selection);
    if (ok != expected_valid || selection.valid != expected_valid || selection.reason != expected_reason) {
        std::cerr << name << " failed: ok=" << ok << " valid=" << selection.valid
                  << " reason=" << selection.reason << " expected_valid=" << expected_valid
                  << " expected_reason=" << expected_reason << "\n";
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

    lightning::LaserMapping::Options options;
    options.source_scan_accum_max_scans_ = 5;
    options.source_scan_accum_min_scans_ = 3;
    options.source_scan_accum_time_sec_ = 0.5;
    options.source_scan_accum_max_trans_m_ = 1.2;
    options.source_scan_accum_max_yaw_deg_ = 8.0;
    options.source_scan_accum_min_points_ = 0;
    options.source_scan_accum_max_yaw_rate_degps_ = 30.0;
    options.source_scan_accum_max_trans_rate_mps_ = 2.5;
    options.source_scan_accum_require_monotonic_stamp_ = true;

    const double curr_stamp = 10.0;
    const lightning::SE3 curr_pose = PoseXYYaw(0.4, 0.0, 0.0);

    std::vector<lightning::LaserMapping::RawScanCacheItem> ok_cache;
    ok_cache.push_back(MakeItem(0, 9.6, PoseXYYaw(0.0, 0.0, 0.0)));
    ok_cache.push_back(MakeItem(1, 9.7, PoseXYYaw(0.1, 0.0, 0.0)));
    ok_cache.push_back(MakeItem(2, 9.8, PoseXYYaw(0.2, 0.0, 0.0)));
    ok_cache.push_back(MakeItem(3, 9.9, PoseXYYaw(0.3, 0.0, 0.0)));
    ok_cache.push_back(MakeItem(4, 10.0, curr_pose));

    lightning::LaserMapping::SourceScanAccumSelection ok_selection;
    if (!lightning::LaserMapping::SelectSourceScanAccumCandidateFromCache(ok_cache, curr_stamp, curr_pose, options,
                                                                           &ok_selection)) {
        std::cerr << "valid selection failed: " << ok_selection.reason << "\n";
        return 1;
    }
    if (ok_selection.selected_count != 5 || ok_selection.frames.front().seq != 0 ||
        ok_selection.frames.back().seq != 4 || ok_selection.time_span_sec < 0.39 ||
        ok_selection.time_span_sec > 0.41) {
        std::cerr << "valid selection returned unexpected frames/count/span\n";
        return 1;
    }

    std::vector<lightning::LaserMapping::RawScanCacheItem> too_few;
    too_few.push_back(MakeItem(0, 9.9, PoseXYYaw(0.3, 0.0, 0.0)));
    too_few.push_back(MakeItem(1, 10.0, curr_pose));
    if (!ExpectReason("too_few", too_few, curr_stamp, curr_pose, options, "ACCUM_TOO_FEW_SCANS", false)) {
        return 1;
    }

    std::vector<lightning::LaserMapping::RawScanCacheItem> time_gap;
    time_gap.push_back(MakeItem(0, 9.0, PoseXYYaw(0.0, 0.0, 0.0)));
    time_gap.push_back(MakeItem(1, 9.1, PoseXYYaw(0.1, 0.0, 0.0)));
    if (!ExpectReason("time_gap", time_gap, curr_stamp, curr_pose, options, "ACCUM_TIME_GAP_TOO_LARGE", false)) {
        return 1;
    }

    std::vector<lightning::LaserMapping::RawScanCacheItem> no_pose;
    no_pose.push_back(MakeItem(0, 10.0, curr_pose, false));
    if (!ExpectReason("no_pose", no_pose, curr_stamp, curr_pose, options, "ACCUM_NO_VALID_POSE", false)) {
        return 1;
    }

    std::vector<lightning::LaserMapping::RawScanCacheItem> trans_large;
    trans_large.push_back(MakeItem(0, 9.8, PoseXYYaw(0.0, 0.0, 0.0)));
    trans_large.push_back(MakeItem(1, 9.9, PoseXYYaw(1.0, 0.0, 0.0)));
    trans_large.push_back(MakeItem(2, 10.0, PoseXYYaw(2.0, 0.0, 0.0)));
    if (!ExpectReason("trans_large", trans_large, curr_stamp, PoseXYYaw(2.0, 0.0, 0.0), options,
                      "ACCUM_TRANS_TOO_LARGE", false)) {
        return 1;
    }

    std::vector<lightning::LaserMapping::RawScanCacheItem> yaw_large;
    yaw_large.push_back(MakeItem(0, 9.8, PoseXYYaw(0.2, 0.0, 20.0)));
    yaw_large.push_back(MakeItem(1, 9.9, PoseXYYaw(0.3, 0.0, 10.0)));
    yaw_large.push_back(MakeItem(2, 10.0, curr_pose));
    if (!ExpectReason("yaw_large", yaw_large, curr_stamp, curr_pose, options, "ACCUM_YAW_TOO_LARGE", false)) {
        return 1;
    }

    std::vector<lightning::LaserMapping::RawScanCacheItem> stamp_non_monotonic;
    stamp_non_monotonic.push_back(MakeItem(0, 9.9, PoseXYYaw(0.3, 0.0, 0.0)));
    stamp_non_monotonic.push_back(MakeItem(1, 9.8, PoseXYYaw(0.2, 0.0, 0.0)));
    stamp_non_monotonic.push_back(MakeItem(2, 10.0, curr_pose));
    if (!ExpectReason("stamp_non_monotonic", stamp_non_monotonic, curr_stamp, curr_pose, options,
                      "ACCUM_STAMP_NON_MONOTONIC", false)) {
        return 1;
    }

    std::vector<lightning::LaserMapping::RawScanCacheItem> trans_rate_large;
    trans_rate_large.push_back(MakeItem(0, 9.9, PoseXYYaw(0.0, 0.0, 0.0)));
    trans_rate_large.push_back(MakeItem(1, 9.95, PoseXYYaw(0.2, 0.0, 0.0)));
    trans_rate_large.push_back(MakeItem(2, 10.0, PoseXYYaw(0.4, 0.0, 0.0)));
    if (!ExpectReason("trans_rate_large", trans_rate_large, curr_stamp, PoseXYYaw(0.4, 0.0, 0.0), options,
                      "ACCUM_TRANS_RATE_TOO_LARGE", false)) {
        return 1;
    }

    std::vector<lightning::LaserMapping::RawScanCacheItem> yaw_rate_large;
    yaw_rate_large.push_back(MakeItem(0, 9.9, PoseXYYaw(0.0, 0.0, 5.0)));
    yaw_rate_large.push_back(MakeItem(1, 9.95, PoseXYYaw(0.1, 0.0, 2.5)));
    yaw_rate_large.push_back(MakeItem(2, 10.0, PoseXYYaw(0.2, 0.0, 0.0)));
    if (!ExpectReason("yaw_rate_large", yaw_rate_large, curr_stamp, PoseXYYaw(0.2, 0.0, 0.0), options,
                      "ACCUM_YAW_RATE_TOO_LARGE", false)) {
        return 1;
    }

    lightning::LaserMapping::Options min_points_options = options;
    min_points_options.source_scan_accum_min_points_ = 3000;
    min_points_options.source_scan_accum_voxel_leaf_m_ = 0.0;
    lightning::LaserMapping::SourceScanAccumCloudResult cloud_result;
    if (lightning::LaserMapping::BuildSourceScanAccumCloudFromSelection(ok_selection, curr_pose, min_points_options,
                                                                        &cloud_result) ||
        cloud_result.fallback_reason != "ACCUM_TOO_FEW_POINTS") {
        std::cerr << "too_few_points failed: ok=" << cloud_result.success
                  << " reason=" << cloud_result.fallback_reason << "\n";
        return 1;
    }

    std::cout << "source scan accumulation selection check passed\n";
    return 0;
}
