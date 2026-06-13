#ifndef LIGHTNING_INPUT_HEALTH_LOGGER_H
#define LIGHTNING_INPUT_HEALTH_LOGGER_H

#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>

namespace lightning {

class InputHealthLogger {
   public:
    struct SourceConfig {
        double min_hz = 0.0;
        double max_age_sec = 1.0;
        double max_gap_sec = 1.0;
        double max_delay_sec = 1.0;
        int min_points = 0;
    };

    struct Config {
        bool enable = true;
        double check_period_sec = 1.0;
        double warn_period_sec = 5.0;
        double startup_grace_sec = 2.0;
        double info_period_sec = 5.0;

        bool csv_enable = true;
        bool jsonl_enable = true;
        bool summary_enable = true;
        std::string output_dir = "data/loc_reports";
        std::string event_output_dir = "data/loc_health";
        int flush_every_n = 1;
        int log_every_n = 1;
        double keep_recent_window_sec = 30.0;
        bool write_fault_window_on_event = true;
        double fault_window_pre_sec = 10.0;
        double fault_window_post_sec = 10.0;

        SourceConfig imu;
        SourceConfig lidar;
        SourceConfig odom;
        double tf_odom_base_max_age_sec = 0.05;
        double tf_lookup_timeout_sec = 0.02;
        double tf_max_lookup_dt_sec = 0.05;
    };

    struct TfRecord {
        std::string parent_frame = "odom";
        std::string child_frame = "base_link";
        double query_stamp = 0.0;
        double latest_tf_stamp = 0.0;
        double lookup_dt_sec = 0.0;
        double lookup_timeout_sec = 0.02;
        bool lookup_ok = false;
        std::string error_type;
        std::string error_string;
    };

    InputHealthLogger();
    ~InputHealthLogger();

    bool Init(const std::string& yaml_path, const std::string& run_label = "");
    bool Enabled() const { return config_.enable; }
    double CheckPeriodSec() const { return config_.check_period_sec; }
    double TfLookupTimeoutSec() const { return config_.tf_lookup_timeout_sec; }

    void RegisterImu(const std::string& topic);
    void RegisterLidar(const std::string& topic);
    void RegisterOdom(const std::string& topic, bool enabled);
    void RegisterTf(const std::string& parent_frame, const std::string& child_frame, bool enabled);

    void ObserveImu(const std::string& topic, const sensor_msgs::Imu& msg, double arrival_ros_time = 0.0,
                    int raw_queue_size = 0);
    void ObservePointCloud2(const std::string& topic, const sensor_msgs::PointCloud2& msg,
                            double arrival_ros_time = 0.0, int raw_queue_size = 0);
    void ObserveLivox(const std::string& topic, const livox_ros_driver2::CustomMsg& msg,
                      double arrival_ros_time = 0.0, int raw_queue_size = 0);
    void ObserveOdom(const std::string& topic, const nav_msgs::Odometry& msg, double arrival_ros_time = 0.0,
                     int raw_queue_size = 0);
    void ObserveTf(const TfRecord& record, double arrival_ros_time = 0.0);
    void Tick(double ros_time = 0.0);
    void Finish();

   private:
    struct Row {
        std::string run_id;
        double wall_time = 0.0;
        double ros_time = 0.0;
        std::string topic;
        std::string frame_id;
        std::string child_frame_id;
        double msg_stamp = 0.0;
        double arrival_time = 0.0;
        double age_sec = 0.0;
        double dt_msg_sec = 0.0;
        double dt_arrival_sec = 0.0;
        double hz_inst_msg = 0.0;
        double hz_inst_arrival = 0.0;
        double hz_ema = 0.0;
        std::string hz_basis = "source_stamp";
        SourceConfig thresholds;
        int64_t count_total = 0;
        int64_t count_ok = 0;
        int64_t count_warn = 0;
        int64_t count_error = 0;
        int64_t non_monotonic_count = 0;
        int64_t gap_count = 0;
        int64_t timeout_count = 0;
        int64_t low_rate_count = 0;
        int64_t delay_count = 0;
        std::string status = "OK";
        std::string reason = "ok";
        std::string severity = "INFO";
        int raw_queue_size = 0;
        std::string extra_json = "{}";
    };

    struct SourceState {
        std::string kind;
        std::string topic;
        SourceConfig thresholds;
        bool enabled = true;
        bool received = false;
        double first_seen_ros = 0.0;
        double last_msg_stamp = 0.0;
        double last_arrival_time = 0.0;
        double last_status_ros = 0.0;
        double hz_ema = 0.0;
        int64_t count_total = 0;
        int64_t count_ok = 0;
        int64_t count_warn = 0;
        int64_t count_error = 0;
        int64_t non_monotonic_count = 0;
        int64_t gap_count = 0;
        int64_t timeout_count = 0;
        int64_t low_rate_count = 0;
        int64_t delay_count = 0;
        std::string last_status;
        std::string last_reason;
        double first_error_time = 0.0;
        double last_error_time = 0.0;
        double max_age = 0.0;
        double max_gap = 0.0;
        double hz_sum = 0.0;
        int64_t hz_samples = 0;
        std::vector<double> ages;
    };

    struct PendingFaultWindow {
        std::string path;
        double event_ros_time = 0.0;
        double end_ros_time = 0.0;
    };

    static Config LoadConfig(const std::string& yaml_path);
    static std::string MakeRunId();
    static double NowWall();
    static double NowRos(double fallback = 0.0);
    static bool IsFinite(double value);
    static std::string JsonEscape(const std::string& text);
    static std::string CsvEscape(const std::string& text);
    static std::string FormatDouble(double value);
    static double Percentile(std::vector<double> values, double ratio);

    void RegisterSource(const std::string& kind, const std::string& topic, const SourceConfig& thresholds,
                        bool enabled);
    void ObserveMessage(const std::string& kind, const std::string& topic, const std::string& frame_id,
                        const std::string& child_frame_id, double msg_stamp, double arrival_ros_time,
                        const std::string& hz_basis, int raw_queue_size, std::string extra_json,
                        const std::string& forced_status = "", const std::string& forced_reason = "");
    Row BuildRow(SourceState& state, const std::string& frame_id, const std::string& child_frame_id,
                 double msg_stamp, double arrival_time, const std::string& hz_basis, int raw_queue_size,
                 const std::string& extra_json, const std::string& forced_status,
                 const std::string& forced_reason);
    void WriteRow(const Row& row);
    void WriteEventIfNeeded(SourceState& state, const Row& row);
    void WriteEvent(const SourceState& state, const Row& row, int event_id, const std::string& suggestion);
    void WriteFaultWindow(const Row& row);
    void AppendRecentRow(const Row& row);
    void AppendPendingFaultWindows(const Row& row);
    void WriteSummary();
    int EventIdFor(const std::string& kind, const std::string& status) const;
    std::string SuggestionFor(const std::string& kind, const std::string& status) const;

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
    std::map<std::string, SourceState> states_;
    std::deque<Row> recent_rows_;
    std::vector<PendingFaultWindow> pending_fault_windows_;
    mutable std::mutex mutex_;
};

using InputHealthLoggerPtr = std::shared_ptr<InputHealthLogger>;

}  // namespace lightning

#endif  // LIGHTNING_INPUT_HEALTH_LOGGER_H
