//
// Created by xiang on 25-3-12.
//

#ifndef LIGHTNING_LOOP_CANDIDATE_H
#define LIGHTNING_LOOP_CANDIDATE_H

#include "common/eigen_types.h"

#include <limits>
#include <string>

namespace lightning {

/**
 * 回环检测候选帧
 */
struct LoopCandidate {
    LoopCandidate() {}
    LoopCandidate(uint64_t id1, uint64_t id2) : idx1_(id1), idx2_(id2) {}

    uint64_t idx1_ = 0;
    uint64_t idx2_ = 0;
    SE3 Tij_;

    double ndt_score_ = 0.0;
    bool ndt_converged_ = false;
    int ndt_iter_ = -1;
    double fitness_score_ = std::numeric_limits<double>::quiet_NaN();
    double ndt_inlier_ratio_ = std::numeric_limits<double>::quiet_NaN();
    long ndt_inlier_count_ = 0;
    long source_points_ = 0;
    long target_points_ = 0;
    bool source_accum_enabled_ = false;
    bool source_accum_used_ = false;
    int source_accum_frames_ = 0;
    long source_accum_raw_points_ = 0;
    std::string source_accum_fallback_reason_;
    std::string source_type_ = "SINGLE_FRAME";
    int source_scan_count_ = 0;
    double source_time_span_sec_ = 0.0;
    long source_points_before_downsample_ = 0;
    long source_points_after_downsample_ = 0;
    bool source_accum_hard_fail_ = false;
    double correction_trans_m_ = std::numeric_limits<double>::quiet_NaN();
    double correction_yaw_deg_ = std::numeric_limits<double>::quiet_NaN();
    double init_to_ndt_xy_m_ = std::numeric_limits<double>::quiet_NaN();
    double init_to_ndt_yaw_deg_ = std::numeric_limits<double>::quiet_NaN();
    double init_to_ndt_z_m_ = std::numeric_limits<double>::quiet_NaN();
    double match_time_ms_ = std::numeric_limits<double>::quiet_NaN();
    int submap_kf_count_ = 0;
    int candidate_rank_ = -1;
    long alignment_event_id_ = -1;
    int hist_cluster_id_ = -1;
    int hist_cluster_rank_ = -1;
    int same_curr_kf_candidate_count_ = -1;
    double xy_dist_m_ = std::numeric_limits<double>::quiet_NaN();
    double z_diff_m_ = std::numeric_limits<double>::quiet_NaN();
    double yaw_diff_deg_ = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace lightning

#endif  // LIGHTNING_LOOP_CANDIDATE_H
