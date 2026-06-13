#ifndef LIGHTNING_LIO_GUESS_DIAG_H
#define LIGHTNING_LIO_GUESS_DIAG_H

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

namespace lightning {

class LioGuessDiagLogger {
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
        double min_trans_warn_m = 0.15;
        double min_trans_error_m = 0.25;

        double yaw_rate_warn_degps = 90.0;
        double yaw_rate_error_degps = 150.0;
        double min_yaw_warn_deg = 3.0;
        double min_yaw_error_deg = 6.0;

        bool compare_with_chassis_odom = true;
        double lio_odom_delta_warn_m = 0.20;
        double lio_odom_delta_error_m = 0.40;
        double lio_odom_yaw_warn_deg = 3.0;
        double lio_odom_yaw_error_deg = 6.0;

        double guess_to_ndt_warn_m = 0.30;
        double guess_to_ndt_error_m = 0.60;
        double guess_to_ndt_yaw_warn_deg = 5.0;
        double guess_to_ndt_yaw_error_deg = 10.0;

        int suspect_confirm_frames = 2;
        int error_confirm_frames = 2;

        double keep_recent_window_sec = 30.0;
        bool write_fault_window_on_event = true;
        double fault_window_pre_sec = 10.0;
        double fault_window_post_sec = 10.0;
    };

    struct MatchDebug {
        bool ndt_valid = false;
        bool ndt_converged = false;
        double ndt_confidence = 0.0;
        double ndt_score = 0.0;
        SE3 ndt_pose;

        bool icp_valid = false;
        bool icp_converged = false;
        bool icp_accepted = false;
        SE3 icp_pose;
    };

    struct TraceInput {
        double ros_time = 0.0;
        int64_t scan_seq = 0;

        bool lio_valid = false;
        double lio_stamp = 0.0;
        bool last_lo_valid = false;
        SE3 last_lo_pose;
        bool current_lo_valid = false;
        SE3 current_lo_pose;
        bool lo_reliable = true;

        bool last_abs_valid = false;
        SE3 last_abs_pose;
        bool guess_valid = false;
        SE3 guess_from_lo;

        MatchDebug match;

        bool final_pose_valid = false;
        SE3 final_pose;
        bool loc_success = false;
        bool loc_inited = false;
        std::string loc_stage;
        std::string reject_reason;
        int point_count = 0;
        int match_fail_count = 0;
    };

    LioGuessDiagLogger();
    ~LioGuessDiagLogger();

    bool Init(const std::string& yaml_path, const std::string& run_label = "");
    bool Enabled() const { return config_.enable; }
    void Finish();

    void ObserveChassisOdom(const nav_msgs::Odometry& odom, double arrival_ros_time = 0.0);
    void ObserveChassisTf(const geometry_msgs::TransformStamped& tf_msg);
    void WriteTrace(const TraceInput& input);

   private:
    struct ChassisState {
        bool valid = false;
        double stamp = 0.0;
        std::string source;
        SE3 pose;
        bool delta_valid = false;
        double delta_xy = 0.0;
        double delta_yaw_deg = 0.0;
        double delta_x = 0.0;
        double delta_y = 0.0;
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
    static double YawDeg(const SE3& pose);
    static double RelYawDeg(const SE3& from, const SE3& to);
    static double Norm2d(const Vec3d& v);
    static bool PoseFinite(const SE3& pose);
    static SE3 PoseFromOdom(const nav_msgs::Odometry& odom);
    static SE3 PoseFromTf(const geometry_msgs::TransformStamped& tf_msg);

    ChassisState BuildChassisState(const SE3& pose, double stamp, const std::string& source,
                                   const ChassisState& prev) const;
    Row BuildRow(const TraceInput& input);
    void WriteCsvRow(const Row& row);
    void WriteEventIfNeeded(const Row& row, const TraceInput& input, double lio_delta_xy,
                            double trans_error_th, double lio_delta_yaw_deg, double yaw_error_th,
                            double guess_to_ndt_xy, double ndt_confidence);
    void WriteSummary();
    void MaybeWriteFaultWindow(const Row& row);
    void AppendRecentRow(const Row& row);
    void AppendPendingFaultWindows(const Row& row);
    int EventIdFor(const std::string& status, const std::string& motion_status,
                   const std::string& odom_status) const;

    Config config_;
    bool initialized_ = false;
    bool finished_ = false;
    std::string yaml_path_;
    std::string run_id_;
    double start_wall_time_ = 0.0;
    double start_ros_time_ = 0.0;
    int csv_rows_since_flush_ = 0;
    int event_seq_ = 0;
    int64_t trace_seq_ = 0;

    std::ofstream csv_;
    std::ofstream jsonl_;
    mutable std::mutex mutex_;

    ChassisState prev_odom_;
    ChassisState latest_odom_;
    ChassisState prev_tf_;
    ChassisState latest_tf_;

    std::string last_diagnosis_status_;
    double last_event_ros_time_ = 0.0;
    double last_lio_speed_ = 0.0;
    double last_lio_yaw_rate_ = 0.0;
    double last_lio_dt_ = 0.0;
    double prev_trace_lio_stamp_ = 0.0;

    int64_t total_frames_ = 0;
    int64_t valid_lio_frames_ = 0;
    int64_t lio_suspect_count_ = 0;
    int64_t lio_jump_count_ = 0;
    int64_t lio_odom_diverged_count_ = 0;
    int64_t input_gap_count_ = 0;
    int64_t guess_recovered_count_ = 0;
    int64_t guess_accepted_count_ = 0;
    int64_t ndt_self_jump_count_ = 0;
    double first_lio_jump_time_ = 0.0;
    double first_ndt_self_jump_time_ = 0.0;
    double first_input_gap_time_ = 0.0;
    double first_lio_odom_diverged_time_ = 0.0;

    std::vector<double> lio_dts_;
    std::vector<double> lio_delta_xys_;
    std::vector<double> lio_yaw_rates_;
    std::vector<double> last_abs_to_guess_xys_;
    std::vector<double> guess_to_ndt_xys_;
    std::vector<double> guess_to_final_xys_;

    std::deque<Row> recent_rows_;
    struct PendingFaultWindow {
        std::string path;
        double event_ros_time = 0.0;
        double end_ros_time = 0.0;
    };
    std::vector<PendingFaultWindow> pending_fault_windows_;
};

using LioGuessDiagLoggerPtr = std::shared_ptr<LioGuessDiagLogger>;

}  // namespace lightning

#endif  // LIGHTNING_LIO_GUESS_DIAG_H
