#ifndef LIGHTNING_ODOM_BASE_DIAG_H
#define LIGHTNING_ODOM_BASE_DIAG_H

#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <yaml-cpp/yaml.h>

#include "common/eigen_types.h"
#include "common/nav_state.h"
#include "core/localization/localization_result.h"

namespace lightning {

class OdomBaseDiagLogger {
   public:
    struct Config {
        bool enable = true;
        bool csv_enable = true;
        bool jsonl_enable = true;
        bool summary_enable = true;
        std::string output_dir = "data/loc_reports";
        std::string event_output_dir = "data/loc_health";
        int flush_every_n = 1;
        int log_every_n = 1;

        double vehicle_max_speed_mps = 4.0;
        double trans_warn_factor = 1.3;
        double trans_error_factor = 1.8;
        double min_trans_warn_m = 0.12;
        double min_trans_error_m = 0.25;

        double yaw_rate_warn_degps = 90.0;
        double yaw_rate_error_degps = 150.0;
        double min_yaw_warn_deg = 3.0;
        double min_yaw_error_deg = 6.0;

        double odom_min_hz = 20.0;
        double odom_max_age_sec = 0.15;
        double odom_max_gap_sec = 0.20;
        double tf_lookup_timeout_sec = 0.02;
        double tf_max_dt_sec = 0.05;
        double tf_max_age_sec = 0.05;

        int source_switch_warn_count = 5;
        double source_switch_window_sec = 10.0;
        int source_switch_error_count = 20;

        bool stationary_enable = true;
        double stationary_speed_mps = 0.05;
        double stationary_trans_warn_m = 0.03;
        double stationary_trans_error_m = 0.08;
        double stationary_yaw_warn_deg = 1.0;
        double stationary_yaw_error_deg = 3.0;

        bool compare_with_lio = true;
        bool compare_with_map_odom = true;
        bool compare_with_final_pose = true;
        double odom_lio_delta_warn_m = 0.20;
        double odom_lio_delta_error_m = 0.40;
        double odom_lio_yaw_warn_deg = 3.0;
        double odom_lio_yaw_error_deg = 6.0;

        double keep_recent_window_sec = 30.0;
        bool write_fault_window_on_event = true;
        double fault_window_pre_sec = 10.0;
        double fault_window_post_sec = 10.0;
    };

    struct TfObservation {
        bool lookup_enable = true;
        bool lookup_ok = false;
        std::string parent_frame = "odom";
        std::string child_frame = "base_link";
        double query_stamp = 0.0;
        double lookup_timeout_sec = 0.02;
        geometry_msgs::TransformStamped tf_msg;
        std::string error_type = "NONE";
        std::string error_string;
    };

    OdomBaseDiagLogger();
    ~OdomBaseDiagLogger();

    bool Init(const std::string& yaml_path, const std::string& run_label = "");
    bool Enabled() const { return config_.enable; }
    double TfLookupTimeoutSec() const { return config_.tf_lookup_timeout_sec; }
    void Finish();

    void ObserveOdom(const std::string& topic, const nav_msgs::Odometry& odom, double arrival_ros_time = 0.0);
    void ObserveTf(const TfObservation& observation);
    void ObserveLio(const NavState& state);
    void ObserveFinalPose(const loc::LocalizationResult& result, const std::string& source);

   private:
    struct PoseSample {
        bool valid = false;
        std::string source = "unknown";
        std::string topic;
        double stamp = 0.0;
        double arrival_time = 0.0;
        std::string frame_id;
        std::string child_frame_id;
        SE3 pose;
        double twist_vx = 0.0;
        double twist_vy = 0.0;
        double twist_wz = 0.0;
        double speed_mps = 0.0;
        bool finite = false;
    };

    struct TfSample {
        bool lookup_enable = false;
        bool lookup_ok = false;
        std::string parent_frame = "odom";
        std::string child_frame = "base_link";
        double query_stamp = 0.0;
        double stamp = 0.0;
        double lookup_timeout_sec = 0.02;
        double lookup_dt_sec = 0.0;
        std::string error_type = "NONE";
        std::string error_string;
        SE3 pose;
        bool pose_valid = false;
    };

    struct DeltaSample {
        bool valid = false;
        double stamp = 0.0;
        double delta_xy = 0.0;
        double delta_yaw_deg = 0.0;
    };

    struct Row {
        std::vector<std::string> cells;
        double ros_time = 0.0;
        std::string diagnosis_status;
        std::string severity;
    };

    static Config LoadConfig(const std::string& yaml_path);
    static std::string MakeRunId(const std::string& run_label);
    static double NowWall();
    static double NowRos(double fallback = 0.0);
    static std::string JsonEscape(const std::string& text);
    static std::string CsvEscape(const std::string& text);
    static std::string FormatDouble(double value);
    static double Percentile(std::vector<double> values, double ratio);
    static double MeanOrZero(const std::vector<double>& values);
    static double MaxOrZero(const std::vector<double>& values);
    static double YawDeg(const SE3& pose);
    static double RelYawDeg(const SE3& from, const SE3& to);
    static Vec3d RpyDeg(const SE3& pose);
    static double Norm2d(const Vec3d& v);
    static bool PoseFinite(const SE3& pose);
    static SE3 PoseFromOdom(const nav_msgs::Odometry& odom);
    static SE3 PoseFromTf(const geometry_msgs::TransformStamped& tf_msg);
    static bool OdomFinite(const nav_msgs::Odometry& odom);

    Row BuildRow(double ros_time_hint);
    void WriteCsvRow(const Row& row);
    void WriteEventIfNeeded(const Row& row);
    void WriteSummary();
    void MaybeWriteFaultWindow(const Row& row);
    void AppendRecentRow(const Row& row);
    void AppendPendingFaultWindows(const Row& row);
    int EventIdFor(const std::string& diagnosis, const std::string& motion_status,
                   const std::string& odom_status, const std::string& tf_error_type) const;

    Config config_;
    bool initialized_ = false;
    bool finished_ = false;
    std::string yaml_path_;
    std::string run_id_;
    double start_wall_time_ = 0.0;
    double start_ros_time_ = 0.0;
    int csv_rows_since_flush_ = 0;
    int event_seq_ = 0;

    std::ofstream csv_;
    std::ofstream jsonl_;
    mutable std::mutex mutex_;

    PoseSample latest_odom_;
    PoseSample prev_odom_;
    bool has_odom_ = false;
    double prev_odom_arrival_ = 0.0;
    double odom_hz_ema_ = 0.0;
    int64_t odom_total_count_ = 0;
    int64_t odom_valid_count_ = 0;
    int64_t odom_non_monotonic_count_ = 0;
    int64_t odom_timeout_count_ = 0;
    int64_t odom_low_rate_count_ = 0;
    std::vector<double> odom_ages_;
    std::vector<double> odom_gaps_;

    TfSample latest_tf_;
    int64_t tf_lookup_count_ = 0;
    int64_t tf_lookup_ok_count_ = 0;
    int64_t tf_lookup_fail_count_ = 0;
    int64_t tf_timeout_count_ = 0;
    int64_t tf_future_count_ = 0;
    int64_t tf_past_count_ = 0;
    int64_t tf_disconnected_count_ = 0;
    std::vector<double> tf_lookup_dts_;

    PoseSample prev_selected_;
    std::string prev_source_selected_;
    std::deque<double> source_switch_times_;
    int64_t source_switch_count_ = 0;
    int64_t max_source_switch_count_window_ = 0;
    int64_t tf_source_switch_jump_count_ = 0;

    DeltaSample latest_lio_delta_;
    NavState prev_lio_state_;
    bool has_prev_lio_state_ = false;

    DeltaSample latest_final_delta_;
    loc::LocalizationResult prev_final_result_;
    bool has_prev_final_result_ = false;

    double last_selected_speed_ = 0.0;
    double last_selected_yaw_rate_ = 0.0;
    double last_selected_dt_ = 0.0;

    std::string last_diagnosis_status_;
    int64_t total_rows_ = 0;
    int64_t chassis_suspect_count_ = 0;
    int64_t chassis_jump_count_ = 0;
    int64_t stationary_drift_count_ = 0;
    int64_t odom_lio_diverged_count_ = 0;
    int64_t odom_msg_tf_inconsistent_count_ = 0;
    int64_t odom_normal_map_odom_jump_count_ = 0;
    int64_t tf_lookup_error_count_ = 0;
    int64_t source_switching_count_ = 0;
    double first_chassis_jump_time_ = 0.0;
    double first_tf_error_time_ = 0.0;

    std::vector<double> selected_delta_xys_;
    std::vector<double> selected_yaw_rates_;

    std::string csv_header_;
    std::deque<Row> recent_rows_;
    struct PendingFaultWindow {
        std::string path;
        double event_ros_time = 0.0;
        double end_ros_time = 0.0;
    };
    std::vector<PendingFaultWindow> pending_fault_windows_;
};

using OdomBaseDiagLoggerPtr = std::shared_ptr<OdomBaseDiagLogger>;

}  // namespace lightning

#endif  // LIGHTNING_ODOM_BASE_DIAG_H
