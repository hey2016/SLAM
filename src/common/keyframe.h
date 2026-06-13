//
// Created by xiang on 25-3-12.
//

#ifndef LIGHTNING_KEYFRAME_H
#define LIGHTNING_KEYFRAME_H

#include "common/eigen_types.h"
#include "common/nav_state.h"
#include "common/point_def.h"
#include "common/std_types.h"

#include <vector>

namespace lightning {

/// 关键帧描述
/// NOTE: 在添加后端后，需要加锁
class Keyframe {
   public:
    using Ptr = std::shared_ptr<Keyframe>;

    struct SourceScanFrame {
        CloudPtr cloud = nullptr;  // undistorted source scan in this frame's body/LiDAR coordinates
        SE3 pose;                  // LIO pose of this source scan in world coordinates
        double stamp = 0.0;
    };

    Keyframe() {}
    Keyframe(unsigned long id, CloudPtr cloud, NavState state)
        : id_(id), cloud_(cloud), state_(state), pose_lio_(state.GetPose()) {
        timestamp_ = state_.timestamp_;
        pose_opt_ = pose_lio_;
    }

    unsigned long GetID() const { return id_; }
    CloudPtr GetCloud() const { return cloud_; }

    void SetSourceScanFrames(const std::vector<SourceScanFrame>& frames) {
        UL lock(data_mutex_);
        source_scan_frames_ = frames;
    }

    std::vector<SourceScanFrame> GetSourceScanFrames() {
        UL lock(data_mutex_);
        return source_scan_frames_;
    }

    SE3 GetLIOPose() {
        UL lock(data_mutex_);
        return pose_lio_;
    }

    void SetLIOPose(const SE3& pose) {
        UL lock(data_mutex_);
        pose_lio_ = pose;

        // also set opt
        pose_opt_ = pose_lio_;
    }

    SE3 GetOptPose() {
        UL lock(data_mutex_);
        return pose_opt_;
    }

    void SetOptPose(const SE3& pose) {
        UL lock(data_mutex_);
        pose_opt_ = pose;
    }

    void SetState(NavState s) {
        UL lock(data_mutex_);
        state_ = s;
    }

    NavState GetState() {
        UL lock(data_mutex_);
        return state_;
    }

   protected:
    unsigned long id_ = 0;

    double timestamp_ = 0;
    CloudPtr cloud_ = nullptr;  /// 降采样之后的点云
    std::vector<SourceScanFrame> source_scan_frames_;

    std::mutex data_mutex_;
    SE3 pose_lio_;  // 前端的pose
    SE3 pose_opt_;  // 后端优化后的pose

    NavState state_;  // 卡尔曼滤波器状态
};

}  // namespace lightning

#endif  // LIGHTNING_KEYFRAME_H
