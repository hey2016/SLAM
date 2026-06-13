#include "ui/pangolin_window_impl.h"

#include <glog/logging.h>

#include <exception>

namespace lightning::ui {

PangolinWindow::PangolinWindow() { impl_ = std::make_shared<PangolinWindowImpl>(); }
PangolinWindow::~PangolinWindow() {
    try {
        Quit();
    } catch (const std::exception& e) {
        LOG(WARNING) << "[ui] ignored exception while destroying PangolinWindow: " << e.what();
    } catch (...) {
        LOG(WARNING) << "[ui] ignored unknown exception while destroying PangolinWindow";
    }
}

bool PangolinWindow::Init() {
    impl_->cloud_global_need_update_.store(false);
    impl_->kf_result_need_update_.store(false);
    impl_->lidarloc_need_update_.store(false);
    impl_->current_scan_need_update_.store(false);

    bool inited = impl_->Init();
    // 创建渲染线程
    if (inited) {
        try {
            impl_->render_stopped_.store(false);
            impl_->render_thread_ = std::thread([impl = impl_]() { impl->Render(); });
        } catch (const std::exception& e) {
            LOG(ERROR) << "[ui] failed to start Pangolin render thread: " << e.what();
            impl_->exit_flag_.store(true);
            impl_->render_stopped_.store(true);
            impl_->DeInit();
            return false;
        }
    }
    return inited;
}

void PangolinWindow::Reset(const std::vector<Keyframe::Ptr>& keyframes) {
    if (!impl_ || impl_->IsStopped()) {
        return;
    }
    impl_->Reset(keyframes);
}

void PangolinWindow::Quit() {
    if (!impl_) {
        return;
    }
    impl_->exit_flag_.store(true);
    if (impl_->render_thread_.joinable()) {
        try {
            impl_->render_thread_.join();
        } catch (const std::exception& e) {
            LOG(WARNING) << "[ui] failed to join Pangolin render thread: " << e.what();
        }
    }
    impl_->DeInit();
}

void PangolinWindow::UpdatePointCloudGlobal(const std::map<int, CloudPtr>& cloud) {
    if (!impl_ || impl_->IsStopped()) {
        return;
    }
    std::lock_guard<std::mutex> lock(impl_->mtx_map_cloud_);
    impl_->cloud_global_map_ = cloud;
    impl_->cloud_global_need_update_.store(true);
}

void PangolinWindow::UpdatePointCloudDynamic(const std::map<int, CloudPtr>& cloud) {
    if (!impl_ || impl_->IsStopped()) {
        return;
    }
    std::unique_lock<std::mutex> lock(impl_->mtx_map_cloud_);
    impl_->cloud_dynamic_map_.clear();  // need deep copy

    for (auto& cp : cloud) {
        CloudPtr c(new PointCloudType());
        *c = *cp.second;
        impl_->cloud_dynamic_map_.emplace(cp.first, c);
    }

    for (auto iter = impl_->cloud_dynamic_map_.begin(); iter != impl_->cloud_dynamic_map_.end();) {
        if (cloud.find(iter->first) == cloud.end()) {
            iter = impl_->cloud_dynamic_map_.erase(iter);
        } else {
            iter++;
        }
    }

    impl_->cloud_dynamic_need_update_.store(true);
}

void PangolinWindow::UpdateNavState(const NavState& state) {
    if (!impl_ || impl_->IsStopped()) {
        return;
    }
    std::unique_lock<std::mutex> lock_lio_res(impl_->mtx_nav_state_);

    impl_->pose_ = state.GetPose();
    impl_->vel_ = state.GetVel();
    impl_->bias_acc_ = state.Getba();
    impl_->bias_gyr_ = state.Getbg();
    impl_->confidence_ = state.confidence_;

    impl_->kf_result_need_update_.store(true);
}

void PangolinWindow::UpdateRecentPose(const SE3& pose) {
    if (!impl_ || impl_->IsStopped()) {
        return;
    }
    std::lock_guard<std::mutex> lock(impl_->mtx_nav_state_);
    impl_->newest_frontend_pose_ = pose;
}

void PangolinWindow::UpdatePredictPose(const SE3& pose) {
    if (!impl_ || impl_->IsStopped()) {
        return;
    }
    UL lock(impl_->mtx_nav_state_);
    impl_->predicted_pose_ = pose;
}

void PangolinWindow::UpdateScan(CloudPtr cloud, const SE3& pose) {
    if (!impl_ || impl_->IsStopped()) {
        return;
    }
    std::lock_guard<std::mutex> lock(impl_->mtx_current_scan_);
    std::lock_guard<std::mutex> lock2(impl_->mtx_nav_state_);

    *impl_->current_scan_ = *cloud;  // need deep copy
    impl_->current_scan_pose_ = pose;
    impl_->current_scan_need_update_.store(true);
}

void PangolinWindow::UpdateKF(std::shared_ptr<Keyframe> kf) {
    if (!impl_ || impl_->IsStopped()) {
        return;
    }
    UL lock(impl_->mtx_current_scan_);
    impl_->all_keyframes_.emplace_back(kf);
}

void PangolinWindow::SetCurrentScanSize(int current_scan_size) {
    if (!impl_) {
        return;
    }
    impl_->max_size_of_current_scan_ = current_scan_size;
}

void PangolinWindow::SetTImuLidar(const SE3& T_imu_lidar) {
    if (!impl_ || impl_->IsStopped()) {
        return;
    }
    impl_->T_imu_lidar_ = T_imu_lidar;
}

bool PangolinWindow::ShouldQuit() { return !impl_ || impl_->IsStopped() || pangolin::ShouldQuit(); }

}  // namespace lightning::ui
