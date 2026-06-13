#include "utils/evaluation_writer.h"

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <cstdio>

#include <glog/logging.h>

namespace lightning {

EvaluationWriter::~EvaluationWriter() {
    std::unique_lock<std::mutex> lock(mutex_);
    if (csv_file_.is_open()) {
        csv_file_.flush();
        csv_file_.close();
    }
    if (tum_file_.is_open()) {
        tum_file_.flush();
        tum_file_.close();
    }
}

bool EvaluationWriter::Init(const std::string& output_dir, const std::string& prefix) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (output_dir.empty()) {
        enabled_ = false;
        return false;
    }

    output_dir_ = output_dir;
    std::filesystem::create_directories(output_dir_);
    csv_path_ = output_dir_ + "/" + prefix + "_evaluation.csv";
    trajectory_path_ = output_dir_ + "/" + prefix + "_trajectory.tum";
    optimized_trajectory_path_ = output_dir_ + "/" + prefix + "_trajectory_optimized.tum";
    map_quality_input_path_ = output_dir_ + "/" + prefix + "_map_quality_input.yaml";
    report_path_ = output_dir_ + "/" + prefix + "_evaluation_report.md";

    csv_file_.open(csv_path_, std::ios::out | std::ios::trunc);
    tum_file_.open(trajectory_path_, std::ios::out | std::ios::trunc);
    if (!csv_file_.is_open() || !tum_file_.is_open()) {
        LOG(ERROR) << "failed to open evaluation outputs under " << output_dir_;
        enabled_ = false;
        return false;
    }

    csv_file_ << "timestamp,pose_x,pose_y,pose_z,roll,pitch,yaw,lidar_odom_score,keyframe_id,"
                 "loop_candidate_id,loop_accepted,loop_score,pgo_before_error,pgo_after_error,"
                 "loop_status,loop_reject_reason,loop_chi2,loop_robust_delta\n";
    enabled_ = true;
    WriteNoGroundTruthReport(false);
    LOG(INFO) << "evaluation csv: " << csv_path_;
    LOG(INFO) << "evaluation tum: " << trajectory_path_;
    LOG(INFO) << "evaluation optimized tum: " << optimized_trajectory_path_;
    LOG(INFO) << "evaluation report: " << report_path_;
    return true;
}

void EvaluationWriter::WriteRecord(const EvaluationRecord& record) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!enabled_) {
        return;
    }

    const auto& pose = record.pose;
    const Vec3d t = pose.translation();
    const Vec3d rpy = pose.so3().matrix().eulerAngles(0, 1, 2);
    const Quatd q = pose.unit_quaternion();

    csv_file_ << std::fixed << std::setprecision(18) << record.timestamp << "," << t.x() << "," << t.y() << ","
              << t.z() << "," << rpy.x() << "," << rpy.y() << "," << rpy.z() << ",";
    WriteOptional(csv_file_, record.score);
    csv_file_ << ",";
    WriteOptional(csv_file_, record.keyframe_id);
    csv_file_ << ",";
    WriteOptional(csv_file_, record.loop_candidate_id);
    csv_file_ << ",";
    WriteOptionalBool(csv_file_, record.loop_accepted);
    csv_file_ << ",";
    WriteOptional(csv_file_, record.loop_score);
    csv_file_ << ",";
    WriteOptional(csv_file_, record.pgo_before_error);
    csv_file_ << ",";
    WriteOptional(csv_file_, record.pgo_after_error);
    csv_file_ << "," << record.loop_status << "," << record.loop_reject_reason << ",";
    WriteOptional(csv_file_, record.loop_chi2);
    csv_file_ << ",";
    WriteOptional(csv_file_, record.loop_robust_delta);
    csv_file_ << "\n";

    if (record.write_trajectory) {
        tum_file_ << std::fixed << std::setprecision(18) << record.timestamp << " " << t.x() << " " << t.y() << " "
                  << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    }
    csv_file_.flush();
    if (record.write_trajectory) {
        tum_file_.flush();
    }
}

void EvaluationWriter::RewriteOptimizedTrajectory(const std::vector<EvaluationTrajectoryPoint>& points) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!enabled_ || optimized_trajectory_path_.empty()) {
        return;
    }

    const std::string tmp_path = optimized_trajectory_path_ + ".tmp";
    std::ofstream file(tmp_path, std::ios::out | std::ios::trunc);
    if (!file.is_open()) {
        LOG(ERROR) << "failed to open optimized trajectory tmp file: " << tmp_path;
        return;
    }

    for (const auto& point : points) {
        const auto& pose = point.pose;
        const Vec3d t = pose.translation();
        const Quatd q = pose.unit_quaternion();
        file << std::fixed << std::setprecision(18) << point.timestamp << " " << t.x() << " " << t.y() << " "
             << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    }
    file.flush();
    file.close();

    std::error_code ec;
    std::filesystem::rename(tmp_path, optimized_trajectory_path_, ec);
    if (ec) {
        std::filesystem::remove(optimized_trajectory_path_, ec);
        ec.clear();
        std::filesystem::rename(tmp_path, optimized_trajectory_path_, ec);
    }
    if (ec) {
        LOG(ERROR) << "failed to replace optimized trajectory file: " << optimized_trajectory_path_
                   << ", error: " << ec.message();
        std::remove(tmp_path.c_str());
    }
}

void EvaluationWriter::WriteLocalizationResult(const loc::LocalizationResult& result, long keyframe_id) {
    if (!result.valid_) {
        return;
    }

    EvaluationRecord record;
    record.timestamp = result.timestamp_;
    record.pose = result.pose_;
    record.score = result.confidence_;
    record.keyframe_id = keyframe_id;
    record.pgo_before_error = result.pgo_before_error_;
    record.pgo_after_error = result.pgo_after_error_;
    WriteRecord(record);
}

void EvaluationWriter::WriteMapQualityInput(const std::string& global_pcd_path, const std::string& map_pgm_path) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!enabled_) {
        return;
    }

    std::ofstream file(map_quality_input_path_, std::ios::out | std::ios::trunc);
    if (!file.is_open()) {
        LOG(ERROR) << "failed to open map quality input file: " << map_quality_input_path_;
        return;
    }

    file << "global_pcd: " << global_pcd_path << "\n";
    file << "map_pgm: " << map_pgm_path << "\n";
    file << "trajectory_tum: " << trajectory_path_ << "\n";
    file << "optimized_trajectory_tum: " << optimized_trajectory_path_ << "\n";
    LOG(INFO) << "map quality input: " << map_quality_input_path_;
}

void EvaluationWriter::WriteNoGroundTruthReport(bool dry_run) {
    std::ofstream file(report_path_, std::ios::out | std::ios::trunc);
    if (!file.is_open()) {
        LOG(ERROR) << "failed to open evaluation report file: " << report_path_;
        return;
    }

    file << "# Lightning Evaluation Report\n\n";
    file << "dry_run: " << (dry_run ? "true" : "false") << "\n";
    file << "rtk_ground_truth: unavailable\n";
    file << "ate: not_computed\n";
    file << "rpe: not_computed\n";
    file << "reason: no RTK or external ground-truth trajectory was provided for this run.\n";
}

void EvaluationWriter::WriteOptional(std::ostream& os, double value) {
    if (std::isfinite(value)) {
        os << value;
    }
}

void EvaluationWriter::WriteOptional(std::ostream& os, long value) {
    if (value >= 0) {
        os << value;
    }
}

void EvaluationWriter::WriteOptionalBool(std::ostream& os, int value) {
    if (value >= 0) {
        os << value;
    }
}

}  // namespace lightning
