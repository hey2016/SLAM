//
// Evaluation output helpers for ROS1 runs.
//

#ifndef LIGHTNING_EVALUATION_WRITER_H
#define LIGHTNING_EVALUATION_WRITER_H

#include <fstream>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include "common/eigen_types.h"
#include "core/localization/localization_result.h"

namespace lightning {

struct EvaluationRecord {
    double timestamp = 0.0;
    SE3 pose;
    double score = std::numeric_limits<double>::quiet_NaN();
    long keyframe_id = -1;
    long loop_candidate_id = -1;
    int loop_accepted = -1;
    double loop_score = std::numeric_limits<double>::quiet_NaN();
    double pgo_before_error = std::numeric_limits<double>::quiet_NaN();
    double pgo_after_error = std::numeric_limits<double>::quiet_NaN();
    std::string loop_status;
    std::string loop_reject_reason;
    double loop_chi2 = std::numeric_limits<double>::quiet_NaN();
    double loop_robust_delta = std::numeric_limits<double>::quiet_NaN();
    bool write_trajectory = true;
};

struct EvaluationTrajectoryPoint {
    double timestamp = 0.0;
    SE3 pose;
    long keyframe_id = -1;
};

class EvaluationWriter {
   public:
    EvaluationWriter() = default;
    ~EvaluationWriter();

    bool Init(const std::string& output_dir, const std::string& prefix);
    bool Enabled() const { return enabled_; }

    void WriteRecord(const EvaluationRecord& record);
    void RewriteOptimizedTrajectory(const std::vector<EvaluationTrajectoryPoint>& points);
    void WriteLocalizationResult(const loc::LocalizationResult& result, long keyframe_id = -1);
    void WriteMapQualityInput(const std::string& global_pcd_path, const std::string& map_pgm_path);
    void WriteNoGroundTruthReport(bool dry_run = false);

    std::string TrajectoryPath() const { return trajectory_path_; }
    std::string OptimizedTrajectoryPath() const { return optimized_trajectory_path_; }
    std::string ReportPath() const { return report_path_; }

   private:
    static void WriteOptional(std::ostream& os, double value);
    static void WriteOptional(std::ostream& os, long value);
    static void WriteOptionalBool(std::ostream& os, int value);

    std::mutex mutex_;
    bool enabled_ = false;
    std::string output_dir_;
    std::string csv_path_;
    std::string trajectory_path_;
    std::string optimized_trajectory_path_;
    std::string map_quality_input_path_;
    std::string report_path_;
    std::ofstream csv_file_;
    std::ofstream tum_file_;
};

}  // namespace lightning

#endif  // LIGHTNING_EVALUATION_WRITER_H
