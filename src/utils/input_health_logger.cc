#include "utils/input_health_logger.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <unistd.h>

#include <glog/logging.h>
#include <ros/ros.h>

namespace lightning {
namespace {

template <typename T>
T GetYamlOr(const YAML::Node& node, const std::string& key, const T& fallback) {
    if (node && node[key]) {
        try {
            return node[key].as<T>();
        } catch (const std::exception& e) {
            LOG(WARNING) << "invalid health_check config key " << key << ": " << e.what();
        }
    }
    return fallback;
}

double StampToSec(const ros::Time& stamp) {
    return stamp.isZero() ? 0.0 : stamp.toSec();
}

bool Finite3(double x, double y, double z) {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

void AddSourceConfig(const YAML::Node& root, const std::string& key, InputHealthLogger::SourceConfig* cfg) {
    const YAML::Node node = root[key];
    cfg->min_hz = GetYamlOr<double>(node, "min_hz", cfg->min_hz);
    cfg->max_age_sec = GetYamlOr<double>(node, "max_age_sec", cfg->max_age_sec);
    cfg->max_gap_sec = GetYamlOr<double>(node, "max_gap_sec", cfg->max_gap_sec);
    cfg->max_delay_sec = GetYamlOr<double>(node, "max_delay_sec", cfg->max_delay_sec);
    cfg->min_points = GetYamlOr<int>(node, "min_points", cfg->min_points);
}

}  // namespace

InputHealthLogger::InputHealthLogger() = default;

InputHealthLogger::~InputHealthLogger() {
    Finish();
}

InputHealthLogger::Config InputHealthLogger::LoadConfig(const std::string& yaml_path) {
    Config cfg;
    cfg.imu.min_hz = 100.0;
    cfg.imu.max_age_sec = 0.10;
    cfg.imu.max_gap_sec = 0.05;
    cfg.imu.max_delay_sec = 0.10;
    cfg.lidar.min_hz = 5.0;
    cfg.lidar.max_age_sec = 0.60;
    cfg.lidar.max_gap_sec = 0.30;
    cfg.lidar.max_delay_sec = 0.60;
    cfg.lidar.min_points = 1000;
    cfg.odom.min_hz = 20.0;
    cfg.odom.max_age_sec = 0.15;
    cfg.odom.max_gap_sec = 0.20;
    cfg.odom.max_delay_sec = 0.15;

    try {
        const YAML::Node yaml = YAML::LoadFile(yaml_path);
        const YAML::Node health = yaml["health_check"];
        cfg.enable = GetYamlOr<bool>(health, "enable", cfg.enable);
        cfg.check_period_sec = GetYamlOr<double>(health, "check_period_sec", cfg.check_period_sec);
        cfg.warn_period_sec = GetYamlOr<double>(health, "warn_period_sec", cfg.warn_period_sec);
        cfg.startup_grace_sec = GetYamlOr<double>(health, "startup_grace_sec", cfg.startup_grace_sec);
        cfg.info_period_sec = GetYamlOr<double>(health, "info_period_sec", cfg.info_period_sec);

        const YAML::Node log = health["input_health_log"];
        cfg.csv_enable = GetYamlOr<bool>(log, "csv_enable", cfg.csv_enable);
        cfg.jsonl_enable = GetYamlOr<bool>(log, "jsonl_enable", cfg.jsonl_enable);
        cfg.summary_enable = GetYamlOr<bool>(log, "summary_enable", cfg.summary_enable);
        cfg.output_dir = GetYamlOr<std::string>(log, "output_dir", cfg.output_dir);
        cfg.event_output_dir = GetYamlOr<std::string>(log, "event_output_dir", cfg.event_output_dir);
        cfg.flush_every_n = std::max(1, GetYamlOr<int>(log, "flush_every_n", cfg.flush_every_n));
        cfg.log_every_n = std::max(1, GetYamlOr<int>(log, "log_every_n", cfg.log_every_n));
        cfg.keep_recent_window_sec = GetYamlOr<double>(log, "keep_recent_window_sec", cfg.keep_recent_window_sec);
        cfg.write_fault_window_on_event =
            GetYamlOr<bool>(log, "write_fault_window_on_event", cfg.write_fault_window_on_event);
        cfg.fault_window_pre_sec = GetYamlOr<double>(log, "fault_window_pre_sec", cfg.fault_window_pre_sec);
        cfg.fault_window_post_sec = GetYamlOr<double>(log, "fault_window_post_sec", cfg.fault_window_post_sec);
        cfg.enable = GetYamlOr<bool>(log, "enable", cfg.enable);

        AddSourceConfig(health, "imu", &cfg.imu);
        AddSourceConfig(health, "lidar", &cfg.lidar);
        AddSourceConfig(health, "odom", &cfg.odom);

        const YAML::Node tf = health["tf"];
        cfg.tf_odom_base_max_age_sec = GetYamlOr<double>(tf, "odom_base_max_age_sec", cfg.tf_odom_base_max_age_sec);
        cfg.tf_lookup_timeout_sec = GetYamlOr<double>(tf, "lookup_timeout_sec", cfg.tf_lookup_timeout_sec);
        cfg.tf_max_lookup_dt_sec = GetYamlOr<double>(tf, "max_lookup_dt_sec", cfg.tf_max_lookup_dt_sec);
    } catch (const std::exception& e) {
        LOG(WARNING) << "failed to read input health config from " << yaml_path << ", using defaults: " << e.what();
    }
    return cfg;
}

bool InputHealthLogger::Init(const std::string& yaml_path, const std::string& run_label) {
    std::lock_guard<std::mutex> lock(mutex_);
    yaml_path_ = yaml_path;
    config_ = LoadConfig(yaml_path);
    if (!config_.enable) {
        initialized_ = true;
        return true;
    }

    run_id_ = MakeRunId();
    if (!run_label.empty()) {
        run_id_ += "-" + run_label;
    }
    start_wall_time_ = NowWall();
    start_ros_time_ = NowRos(start_wall_time_);

    std::error_code ec;
    std::filesystem::create_directories(config_.output_dir, ec);
    if (ec) {
        LOG(WARNING) << "failed to create input health output dir " << config_.output_dir << ": " << ec.message();
    }
    std::filesystem::create_directories(config_.event_output_dir, ec);
    if (ec) {
        LOG(WARNING) << "failed to create input health event dir " << config_.event_output_dir << ": " << ec.message();
    }

    if (config_.csv_enable) {
        const std::string path = config_.output_dir + "/input_health_" + run_id_ + ".csv";
        csv_.open(path, std::ios::out | std::ios::trunc);
        if (!csv_.is_open()) {
            LOG(WARNING) << "failed to open input health csv: " << path;
        } else {
            csv_ << "run_id,wall_time,ros_time,topic,frame_id,child_frame_id,msg_stamp,arrival_time,age_sec,"
                    "dt_msg_sec,dt_arrival_sec,hz_inst_msg,hz_inst_arrival,hz_ema,hz_basis,min_hz,max_age_sec,"
                    "max_gap_sec,max_delay_sec,count_total,count_ok,count_warn,count_error,non_monotonic_count,"
                    "gap_count,timeout_count,low_rate_count,delay_count,status,reason,severity,raw_queue_size,"
                    "extra_json\n";
        }
    }
    if (config_.jsonl_enable) {
        const std::string path = config_.event_output_dir + "/input_health_events_" + run_id_ + ".jsonl";
        jsonl_.open(path, std::ios::out | std::ios::trunc);
        if (!jsonl_.is_open()) {
            LOG(WARNING) << "failed to open input health jsonl: " << path;
        }
    }

    initialized_ = true;
    return true;
}

void InputHealthLogger::RegisterImu(const std::string& topic) {
    RegisterSource("imu", topic, config_.imu, true);
}

void InputHealthLogger::RegisterLidar(const std::string& topic) {
    RegisterSource("lidar", topic, config_.lidar, true);
}

void InputHealthLogger::RegisterOdom(const std::string& topic, bool enabled) {
    RegisterSource("odom", topic.empty() ? "/odom" : topic, config_.odom, enabled);
}

void InputHealthLogger::RegisterTf(const std::string& parent_frame, const std::string& child_frame, bool enabled) {
    SourceConfig cfg;
    cfg.min_hz = 1.0 / std::max(1e-6, config_.check_period_sec);
    cfg.max_age_sec = config_.tf_odom_base_max_age_sec;
    cfg.max_gap_sec = config_.tf_max_lookup_dt_sec;
    cfg.max_delay_sec = config_.tf_max_lookup_dt_sec;
    RegisterSource("tf", "/tf:" + parent_frame + "->" + child_frame, cfg, enabled);
}

void InputHealthLogger::RegisterSource(const std::string& kind, const std::string& topic,
                                       const SourceConfig& thresholds, bool enabled) {
    if (!config_.enable || topic.empty()) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    SourceState& state = states_[topic];
    state.kind = kind;
    state.topic = topic;
    state.thresholds = thresholds;
    state.enabled = enabled;
}

void InputHealthLogger::ObserveImu(const std::string& topic, const sensor_msgs::Imu& msg, double arrival_ros_time,
                                   int raw_queue_size) {
    const bool av_ok =
        Finite3(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    const bool la_ok =
        Finite3(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    std::ostringstream extra;
    extra << "{\"angular_velocity_finite\":" << (av_ok ? "true" : "false")
          << ",\"linear_acceleration_finite\":" << (la_ok ? "true" : "false") << "}";
    ObserveMessage("imu", topic, msg.header.frame_id, "", StampToSec(msg.header.stamp), arrival_ros_time,
                   "source_stamp", raw_queue_size, extra.str(), (av_ok && la_ok) ? "" : "NON_MONOTONIC",
                   (av_ok && la_ok) ? "" : "imu contains NaN/Inf");
}

void InputHealthLogger::ObservePointCloud2(const std::string& topic, const sensor_msgs::PointCloud2& msg,
                                           double arrival_ros_time, int raw_queue_size) {
    const int points = static_cast<int>(msg.width * msg.height);
    std::ostringstream extra;
    extra << "{\"points_count\":" << points << ",\"min_points\":" << config_.lidar.min_points << "}";
    ObserveMessage("lidar", topic, msg.header.frame_id, "", StampToSec(msg.header.stamp), arrival_ros_time,
                   "source_stamp", raw_queue_size, extra.str(), points < config_.lidar.min_points ? "LIDAR_TOO_FEW_POINTS" : "",
                   points < config_.lidar.min_points ? "points below min_points" : "");
}

void InputHealthLogger::ObserveLivox(const std::string& topic, const livox_ros_driver2::CustomMsg& msg,
                                     double arrival_ros_time, int raw_queue_size) {
    const int points = static_cast<int>(msg.points.size());
    std::ostringstream extra;
    extra << "{\"points_count\":" << points << ",\"min_points\":" << config_.lidar.min_points << "}";
    ObserveMessage("lidar", topic, msg.header.frame_id, "", StampToSec(msg.header.stamp), arrival_ros_time,
                   "source_stamp", raw_queue_size, extra.str(), points < config_.lidar.min_points ? "LIDAR_TOO_FEW_POINTS" : "",
                   points < config_.lidar.min_points ? "points below min_points" : "");
}

void InputHealthLogger::ObserveOdom(const std::string& topic, const nav_msgs::Odometry& msg, double arrival_ros_time,
                                    int raw_queue_size) {
    const auto& p = msg.pose.pose.position;
    const auto& q = msg.pose.pose.orientation;
    const auto& lv = msg.twist.twist.linear;
    const auto& av = msg.twist.twist.angular;
    const bool finite = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(q.x) &&
                        std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
                        Finite3(lv.x, lv.y, lv.z) && Finite3(av.x, av.y, av.z);
    std::ostringstream extra;
    extra << "{\"pose_twist_finite\":" << (finite ? "true" : "false") << ",\"linear_speed\":"
          << std::sqrt(lv.x * lv.x + lv.y * lv.y + lv.z * lv.z) << "}";
    ObserveMessage("odom", topic, msg.header.frame_id, msg.child_frame_id, StampToSec(msg.header.stamp),
                   arrival_ros_time, "source_stamp", raw_queue_size, extra.str(), finite ? "" : "NON_MONOTONIC",
                   finite ? "" : "odom contains NaN/Inf");
}

void InputHealthLogger::ObserveTf(const TfRecord& record, double arrival_ros_time) {
    std::ostringstream extra;
    extra << "{\"query_stamp\":" << record.query_stamp << ",\"latest_tf_stamp\":" << record.latest_tf_stamp
          << ",\"lookup_ok\":" << (record.lookup_ok ? "true" : "false")
          << ",\"lookup_dt_sec\":" << record.lookup_dt_sec
          << ",\"lookup_timeout_sec\":" << record.lookup_timeout_sec
          << ",\"parent_frame\":\"" << JsonEscape(record.parent_frame) << "\""
          << ",\"child_frame\":\"" << JsonEscape(record.child_frame) << "\""
          << ",\"error_type\":\"" << JsonEscape(record.error_type) << "\""
          << ",\"error_string\":\"" << JsonEscape(record.error_string) << "\"}";
    const std::string topic = "/tf:" + record.parent_frame + "->" + record.child_frame;
    std::string forced_status;
    std::string forced_reason;
    if (!record.lookup_ok) {
        forced_status = record.error_type.empty() ? "TF_TIMEOUT" : record.error_type;
        forced_reason = record.error_string.empty() ? "tf lookup failed" : record.error_string;
    } else if (std::fabs(record.lookup_dt_sec) > config_.tf_max_lookup_dt_sec) {
        forced_status = "STALE";
        forced_reason = "tf lookup dt above max_lookup_dt_sec";
    }
    ObserveMessage("tf", topic, record.parent_frame, record.child_frame, record.latest_tf_stamp, arrival_ros_time,
                   "tf_stamp", 0, extra.str(), forced_status, forced_reason);
}

void InputHealthLogger::ObserveMessage(const std::string& kind, const std::string& topic, const std::string& frame_id,
                                       const std::string& child_frame_id, double msg_stamp, double arrival_ros_time,
                                       const std::string& hz_basis, int raw_queue_size, std::string extra_json,
                                       const std::string& forced_status, const std::string& forced_reason) {
    if (!config_.enable || !initialized_) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    SourceState& state = states_[topic];
    if (state.kind.empty()) {
        SourceConfig cfg = config_.lidar;
        if (kind == "imu") cfg = config_.imu;
        if (kind == "odom") cfg = config_.odom;
        state.kind = kind;
        state.topic = topic;
        state.thresholds = cfg;
        state.enabled = true;
    }
    Row row = BuildRow(state, frame_id, child_frame_id, msg_stamp, arrival_ros_time, hz_basis, raw_queue_size,
                       std::move(extra_json), forced_status, forced_reason);
    WriteRow(row);
    WriteEventIfNeeded(state, row);
    AppendRecentRow(row);
}

InputHealthLogger::Row InputHealthLogger::BuildRow(SourceState& state, const std::string& frame_id,
                                                   const std::string& child_frame_id, double msg_stamp,
                                                   double arrival_time, const std::string& hz_basis,
                                                   int raw_queue_size, const std::string& extra_json,
                                                   const std::string& forced_status,
                                                   const std::string& forced_reason) {
    Row row;
    row.run_id = run_id_;
    row.wall_time = NowWall();
    row.ros_time = arrival_time > 0 ? arrival_time : NowRos(row.wall_time);
    row.topic = state.topic;
    row.frame_id = frame_id;
    row.child_frame_id = child_frame_id;
    row.msg_stamp = msg_stamp;
    row.arrival_time = row.ros_time;
    row.age_sec = msg_stamp > 0 ? row.arrival_time - msg_stamp : 0.0;
    row.hz_basis = hz_basis;
    row.thresholds = state.thresholds;
    row.raw_queue_size = raw_queue_size;
    row.extra_json = extra_json.empty() ? "{}" : extra_json;

    state.count_total++;
    if (!state.received) {
        state.first_seen_ros = row.ros_time;
        state.received = true;
    } else {
        row.dt_msg_sec = msg_stamp > 0 && state.last_msg_stamp > 0 ? msg_stamp - state.last_msg_stamp : 0.0;
        row.dt_arrival_sec = row.arrival_time - state.last_arrival_time;
        if (row.dt_msg_sec > 0) row.hz_inst_msg = 1.0 / row.dt_msg_sec;
        if (row.dt_arrival_sec > 0) row.hz_inst_arrival = 1.0 / row.dt_arrival_sec;
    }

    const double hz_inst = row.hz_inst_msg > 0 ? row.hz_inst_msg : row.hz_inst_arrival;
    if (hz_inst > 0 && std::isfinite(hz_inst)) {
        constexpr double kAlpha = 0.2;
        state.hz_ema = state.hz_ema <= 0 ? hz_inst : (1.0 - kAlpha) * state.hz_ema + kAlpha * hz_inst;
        state.hz_sum += hz_inst;
        state.hz_samples++;
    }
    row.hz_ema = state.hz_ema;

    row.status = "OK";
    row.reason = "ok";
    row.severity = "INFO";

    const bool in_startup = row.ros_time - start_ros_time_ < config_.startup_grace_sec;
    if (!state.enabled) {
        row.status = "DISABLED";
        row.reason = "input source disabled";
    } else if (in_startup) {
        row.status = "STARTUP_WAIT";
        row.reason = "within startup_grace_sec";
    }

    if (!forced_status.empty()) {
        row.status = forced_status;
        row.reason = forced_reason;
    } else if (msg_stamp > 0 && state.last_msg_stamp > 0 && msg_stamp < state.last_msg_stamp) {
        row.status = "NON_MONOTONIC";
        row.reason = "msg_stamp moved backwards";
        state.non_monotonic_count++;
    } else if (row.dt_msg_sec > state.thresholds.max_gap_sec ||
               row.dt_arrival_sec > state.thresholds.max_gap_sec) {
        row.status = "GAP";
        row.reason = "message gap above max_gap_sec";
        state.gap_count++;
    } else if (row.age_sec > state.thresholds.max_delay_sec) {
        row.status = "DELAY_HIGH";
        row.reason = "arrival_time - msg_stamp above max_delay_sec";
        state.delay_count++;
    } else if (row.age_sec > state.thresholds.max_age_sec) {
        row.status = "STALE";
        row.reason = "message age above max_age_sec";
    } else if (!in_startup && state.thresholds.min_hz > 0 && state.hz_ema > 0 && state.hz_ema < state.thresholds.min_hz) {
        row.status = "LOW_RATE";
        row.reason = "hz_ema below min_hz";
        state.low_rate_count++;
    }

    if (row.status == "OK" || row.status == "STARTUP_WAIT" || row.status == "DISABLED") {
        row.severity = "INFO";
        state.count_ok++;
    } else if (row.status == "NON_MONOTONIC" || row.status.find("TF_") == 0 || row.status == "NOT_RECEIVED") {
        row.severity = "ERROR";
        state.count_error++;
    } else {
        row.severity = "WARN";
        state.count_warn++;
    }

    state.max_age = std::max(state.max_age, std::fabs(row.age_sec));
    state.max_gap = std::max(state.max_gap, std::max(std::fabs(row.dt_msg_sec), std::fabs(row.dt_arrival_sec)));
    if (std::isfinite(row.age_sec)) {
        state.ages.push_back(row.age_sec);
        if (state.ages.size() > 100000) {
            state.ages.erase(state.ages.begin(), state.ages.begin() + 1000);
        }
    }
    if (row.severity == "ERROR") {
        if (state.first_error_time <= 0) state.first_error_time = row.ros_time;
        state.last_error_time = row.ros_time;
    }

    row.count_total = state.count_total;
    row.count_ok = state.count_ok;
    row.count_warn = state.count_warn;
    row.count_error = state.count_error;
    row.non_monotonic_count = state.non_monotonic_count;
    row.gap_count = state.gap_count;
    row.timeout_count = state.timeout_count;
    row.low_rate_count = state.low_rate_count;
    row.delay_count = state.delay_count;

    state.last_msg_stamp = msg_stamp > 0 ? msg_stamp : state.last_msg_stamp;
    state.last_arrival_time = row.arrival_time;
    return row;
}

void InputHealthLogger::Tick(double ros_time) {
    if (!config_.enable || !initialized_) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = ros_time > 0 ? ros_time : NowRos(NowWall());
    for (auto& kv : states_) {
        SourceState& state = kv.second;
        Row row;
        row.run_id = run_id_;
        row.wall_time = NowWall();
        row.ros_time = now;
        row.topic = state.topic;
        row.arrival_time = now;
        row.hz_ema = state.hz_ema;
        row.hz_basis = state.kind == "tf" ? "tf_stamp" : "arrival_time";
        row.thresholds = state.thresholds;
        row.count_total = state.count_total;
        row.count_ok = state.count_ok;
        row.count_warn = state.count_warn;
        row.count_error = state.count_error;
        row.non_monotonic_count = state.non_monotonic_count;
        row.gap_count = state.gap_count;
        row.timeout_count = state.timeout_count;
        row.low_rate_count = state.low_rate_count;
        row.delay_count = state.delay_count;
        if (!state.enabled) {
            row.status = "DISABLED";
            row.reason = "input source disabled";
        } else if (now - start_ros_time_ < config_.startup_grace_sec) {
            row.status = "STARTUP_WAIT";
            row.reason = "within startup_grace_sec";
        } else if (!state.received) {
            row.status = "NOT_RECEIVED";
            row.reason = "no message received after startup_grace_sec";
            row.severity = "ERROR";
            state.timeout_count++;
        } else {
            row.msg_stamp = state.last_msg_stamp;
            row.age_sec = now - state.last_msg_stamp;
            if (row.age_sec > state.thresholds.max_age_sec) {
                row.status = state.kind == "tf" ? "TF_TIMEOUT" : "TIMEOUT";
                row.reason = "last input older than max_age_sec";
                row.severity = "ERROR";
                state.timeout_count++;
            } else {
                continue;
            }
        }
        if (row.severity.empty()) {
            row.severity = row.status == "NOT_RECEIVED" ? "ERROR" : "INFO";
        }
        if (row.severity == "ERROR") {
            state.count_error++;
            if (state.first_error_time <= 0) state.first_error_time = row.ros_time;
            state.last_error_time = row.ros_time;
        }
        row.count_error = state.count_error;
        row.timeout_count = state.timeout_count;
        WriteRow(row);
        WriteEventIfNeeded(state, row);
        AppendRecentRow(row);
    }
}

void InputHealthLogger::WriteRow(const Row& row) {
    if (!csv_.is_open()) {
        return;
    }
    if (config_.log_every_n > 1 && row.count_total > 0 && row.count_total % config_.log_every_n != 0 &&
        row.status == "OK") {
        return;
    }
    csv_ << CsvEscape(row.run_id) << "," << FormatDouble(row.wall_time) << "," << FormatDouble(row.ros_time) << ","
         << CsvEscape(row.topic) << "," << CsvEscape(row.frame_id) << "," << CsvEscape(row.child_frame_id) << ","
         << FormatDouble(row.msg_stamp) << "," << FormatDouble(row.arrival_time) << ","
         << FormatDouble(row.age_sec) << "," << FormatDouble(row.dt_msg_sec) << ","
         << FormatDouble(row.dt_arrival_sec) << "," << FormatDouble(row.hz_inst_msg) << ","
         << FormatDouble(row.hz_inst_arrival) << "," << FormatDouble(row.hz_ema) << ","
         << CsvEscape(row.hz_basis) << "," << FormatDouble(row.thresholds.min_hz) << ","
         << FormatDouble(row.thresholds.max_age_sec) << "," << FormatDouble(row.thresholds.max_gap_sec) << ","
         << FormatDouble(row.thresholds.max_delay_sec) << "," << row.count_total << "," << row.count_ok << ","
         << row.count_warn << "," << row.count_error << "," << row.non_monotonic_count << ","
         << row.gap_count << "," << row.timeout_count << "," << row.low_rate_count << "," << row.delay_count << ","
         << CsvEscape(row.status) << "," << CsvEscape(row.reason) << "," << CsvEscape(row.severity) << ","
         << row.raw_queue_size << "," << CsvEscape(row.extra_json) << "\n";
    csv_rows_since_flush_++;
    if (csv_rows_since_flush_ >= config_.flush_every_n) {
        csv_.flush();
        csv_rows_since_flush_ = 0;
    }
}

void InputHealthLogger::WriteEventIfNeeded(SourceState& state, const Row& row) {
    const bool status_changed = row.status != state.last_status || row.reason != state.last_reason;
    const bool abnormal = row.severity == "WARN" || row.severity == "ERROR";
    const bool throttled = state.last_status_ros > 0 && row.ros_time - state.last_status_ros < config_.warn_period_sec;
    if ((status_changed || abnormal) && (!throttled || status_changed)) {
        WriteEvent(state, row, status_changed && row.status == "OK" ? 1099 : EventIdFor(state.kind, row.status),
                   SuggestionFor(state.kind, row.status));
        state.last_status_ros = row.ros_time;
    }
    state.last_status = row.status;
    state.last_reason = row.reason;
}

void InputHealthLogger::WriteEvent(const SourceState& state, const Row& row, int event_id,
                                   const std::string& suggestion) {
    if (!jsonl_.is_open()) {
        return;
    }
    jsonl_ << "{\"run_id\":\"" << JsonEscape(row.run_id) << "\",\"event_time_ros\":" << FormatDouble(row.ros_time)
           << ",\"event_time_wall\":" << FormatDouble(row.wall_time) << ",\"event_id\":" << event_id
           << ",\"module\":\"input_health\",\"topic\":\"" << JsonEscape(row.topic) << "\",\"status\":\""
           << JsonEscape(row.status) << "\",\"severity\":\"" << JsonEscape(row.severity)
           << "\",\"reason\":\"" << JsonEscape(row.reason) << "\",\"count_total\":" << row.count_total
           << ",\"hz_ema\":" << FormatDouble(row.hz_ema) << ",\"min_hz\":" << FormatDouble(row.thresholds.min_hz)
           << ",\"age_sec\":" << FormatDouble(row.age_sec)
           << ",\"max_age_sec\":" << FormatDouble(row.thresholds.max_age_sec)
           << ",\"dt_msg_sec\":" << FormatDouble(row.dt_msg_sec)
           << ",\"dt_arrival_sec\":" << FormatDouble(row.dt_arrival_sec)
           << ",\"suggestion\":\"" << JsonEscape(suggestion) << "\"}\n";
    jsonl_.flush();
    if (config_.write_fault_window_on_event && (row.severity == "WARN" || row.severity == "ERROR")) {
        WriteFaultWindow(row);
    }
    (void)state;
}

void InputHealthLogger::WriteFaultWindow(const Row& row) {
    const std::string path = config_.event_output_dir + "/input_health_fault_window_" + run_id_ + "_" +
                             std::to_string(++event_seq_) + ".jsonl";
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        return;
    }
    const double from = row.ros_time - config_.fault_window_pre_sec;
    for (const auto& r : recent_rows_) {
        if (r.ros_time >= from && r.ros_time <= row.ros_time + config_.fault_window_post_sec) {
            out << "{\"ros_time\":" << FormatDouble(r.ros_time) << ",\"topic\":\"" << JsonEscape(r.topic)
                << "\",\"status\":\"" << JsonEscape(r.status) << "\",\"severity\":\"" << JsonEscape(r.severity)
                << "\",\"reason\":\"" << JsonEscape(r.reason) << "\"}\n";
        }
    }
    out << "{\"ros_time\":" << FormatDouble(row.ros_time) << ",\"topic\":\"" << JsonEscape(row.topic)
        << "\",\"status\":\"" << JsonEscape(row.status) << "\",\"severity\":\"" << JsonEscape(row.severity)
        << "\",\"reason\":\"" << JsonEscape(row.reason) << "\",\"event\":true}\n";
    if (config_.fault_window_post_sec > 0) {
        PendingFaultWindow pending;
        pending.path = path;
        pending.event_ros_time = row.ros_time;
        pending.end_ros_time = row.ros_time + config_.fault_window_post_sec;
        pending_fault_windows_.push_back(std::move(pending));
    }
}

void InputHealthLogger::AppendRecentRow(const Row& row) {
    recent_rows_.push_back(row);
    const double keep_from = row.ros_time - std::max(config_.keep_recent_window_sec, config_.fault_window_pre_sec);
    while (!recent_rows_.empty() && recent_rows_.front().ros_time < keep_from) {
        recent_rows_.pop_front();
    }
    AppendPendingFaultWindows(row);
}

void InputHealthLogger::AppendPendingFaultWindows(const Row& row) {
    if (pending_fault_windows_.empty()) {
        return;
    }
    for (auto it = pending_fault_windows_.begin(); it != pending_fault_windows_.end();) {
        if (row.ros_time > it->event_ros_time && row.ros_time <= it->end_ros_time) {
            std::ofstream out(it->path, std::ios::out | std::ios::app);
            if (out.is_open()) {
                out << "{\"ros_time\":" << FormatDouble(row.ros_time) << ",\"topic\":\"" << JsonEscape(row.topic)
                    << "\",\"status\":\"" << JsonEscape(row.status) << "\",\"severity\":\""
                    << JsonEscape(row.severity) << "\",\"reason\":\"" << JsonEscape(row.reason)
                    << "\",\"post_window\":true}\n";
            }
        }
        if (row.ros_time > it->end_ros_time) {
            it = pending_fault_windows_.erase(it);
        } else {
            ++it;
        }
    }
}

void InputHealthLogger::Finish() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (finished_) {
        return;
    }
    finished_ = true;
    if (initialized_ && config_.enable && config_.summary_enable) {
        WriteSummary();
    }
    if (csv_.is_open()) csv_.close();
    if (jsonl_.is_open()) jsonl_.close();
}

void InputHealthLogger::WriteSummary() {
    std::error_code ec;
    std::filesystem::create_directories(config_.output_dir, ec);
    const std::string path = config_.output_dir + "/input_health_summary_" + run_id_ + ".md";
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        LOG(WARNING) << "failed to open input health summary: " << path;
        return;
    }
    const double end_wall = NowWall();
    out << "# Input Health Summary\n\n";
    out << "- run_id: `" << run_id_ << "`\n";
    out << "- start_time_wall: `" << FormatDouble(start_wall_time_) << "`\n";
    out << "- end_time_wall: `" << FormatDouble(end_wall) << "`\n";
    out << "- duration_sec: `" << FormatDouble(end_wall - start_wall_time_) << "`\n";
    out << "- config_file: `" << yaml_path_ << "`\n\n";
    out << "| topic | total_count | ok_ratio | warn_count | error_count | min_hz | mean_hz | p95_age | max_age | max_gap | non_monotonic_count | timeout_count | first_error_time | last_error_time |\n";
    out << "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n";
    bool odom_missing = false;
    bool tf_error = false;
    bool sensor_timeout = false;
    for (const auto& kv : states_) {
        const auto& s = kv.second;
        const double ok_ratio = s.count_total > 0 ? static_cast<double>(s.count_ok) / s.count_total : 0.0;
        const double mean_hz = s.hz_samples > 0 ? s.hz_sum / s.hz_samples : 0.0;
        const double p95_age = Percentile(s.ages, 0.95);
        out << "| `" << s.topic << "` | " << s.count_total << " | " << FormatDouble(ok_ratio) << " | "
            << s.count_warn << " | " << s.count_error << " | " << FormatDouble(s.thresholds.min_hz) << " | "
            << FormatDouble(mean_hz) << " | " << FormatDouble(p95_age) << " | " << FormatDouble(s.max_age)
            << " | " << FormatDouble(s.max_gap) << " | " << s.non_monotonic_count << " | " << s.timeout_count
            << " | " << FormatDouble(s.first_error_time) << " | " << FormatDouble(s.last_error_time) << " |\n";
        if (s.kind == "odom" && (!s.received || s.timeout_count > 0)) odom_missing = true;
        if (s.kind == "tf" && s.count_error > 0) tf_error = true;
        if ((s.kind == "imu" || s.kind == "lidar") && s.timeout_count > 0) sensor_timeout = true;
    }
    out << "\n## Root Cause Guess\n\n";
    if (odom_missing) {
        out << "- 定位未启动或定位异常前如果 `/odom` 长期 NOT_RECEIVED/TIMEOUT，优先怀疑 odom 未接入或 odom->base_link TF 未发布。\n";
    }
    if (sensor_timeout) {
        out << "- IMU 或 LiDAR 出现 TIMEOUT，优先怀疑传感器断流、驱动异常、rosbag 回放停顿或 CPU 负载导致回调延迟。\n";
    }
    if (tf_error) {
        out << "- TF 查询出现 timeout/future/past/disconnected，优先怀疑 TF 时间同步或 TF 树连接问题。\n";
    }
    if (!odom_missing && !sensor_timeout && !tf_error) {
        out << "- 未观察到明确输入断流根因，需继续结合定位内部 trace 判断。\n";
    }
    out << "\n根因初判需要结合 lidar_loc_match_trace、lio_trace、map_odom_trace 进一步确认。\n";
}

int InputHealthLogger::EventIdFor(const std::string& kind, const std::string& status) const {
    if (kind == "imu") {
        if (status == "LOW_RATE") return 1001;
        if (status == "TIMEOUT" || status == "STALE" || status == "NOT_RECEIVED") return 1002;
        if (status == "NON_MONOTONIC") return 1003;
    }
    if (kind == "lidar") {
        if (status == "LOW_RATE") return 1011;
        if (status == "TIMEOUT" || status == "STALE" || status == "NOT_RECEIVED") return 1012;
        if (status == "NON_MONOTONIC") return 1013;
        if (status == "LIDAR_TOO_FEW_POINTS") return 1014;
    }
    if (kind == "odom") {
        if (status == "LOW_RATE") return 1021;
        if (status == "TIMEOUT" || status == "STALE" || status == "NOT_RECEIVED") return 1022;
        if (status == "NON_MONOTONIC") return 1023;
    }
    if (kind == "tf") {
        if (status == "TF_TIMEOUT") return 1031;
        if (status == "TF_EXTRAPOLATION_FUTURE") return 1032;
        if (status == "TF_EXTRAPOLATION_PAST") return 1033;
        if (status == "TF_DISCONNECTED") return 1034;
    }
    return 1099;
}

std::string InputHealthLogger::SuggestionFor(const std::string& kind, const std::string& status) const {
    if (status == "LOW_RATE") return "check sensor driver, rosbag playback rate, CPU load, or timestamp source";
    if (status == "TIMEOUT" || status == "NOT_RECEIVED" || status == "TF_TIMEOUT")
        return "check topic publisher, TF publisher, network, driver, and rosbag playback";
    if (status == "NON_MONOTONIC") return "check timestamp source and rosbag ordering";
    if (status.find("TF_") == 0) return "check TF tree connectivity and time synchronization";
    if (kind == "lidar" && status == "LIDAR_TOO_FEW_POINTS") return "check LiDAR driver, ROI filtering, or bag contents";
    return "correlate with localization traces and sensor logs";
}

std::string InputHealthLogger::MakeRunId() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d-%H%M%S") << "-" << getpid();
    return oss.str();
}

double InputHealthLogger::NowWall() {
    return ros::WallTime::now().toSec();
}

double InputHealthLogger::NowRos(double fallback) {
    try {
        if (!ros::isInitialized()) {
            ros::Time::init();
        }
        const ros::Time now = ros::Time::now();
        return now.isZero() ? fallback : now.toSec();
    } catch (const ros::TimeNotInitializedException&) {
        return fallback;
    }
}

bool InputHealthLogger::IsFinite(double value) {
    return std::isfinite(value);
}

std::string InputHealthLogger::JsonEscape(const std::string& text) {
    std::ostringstream out;
    for (char c : text) {
        switch (c) {
            case '\\': out << "\\\\"; break;
            case '"': out << "\\\""; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default: out << c; break;
        }
    }
    return out.str();
}

std::string InputHealthLogger::CsvEscape(const std::string& text) {
    if (text.find_first_of(",\"\n\r") == std::string::npos) {
        return text;
    }
    std::string out = "\"";
    for (char c : text) {
        if (c == '"') out += '"';
        out += c;
    }
    out += '"';
    return out;
}

std::string InputHealthLogger::FormatDouble(double value) {
    if (!std::isfinite(value)) return "";
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << value;
    return oss.str();
}

double InputHealthLogger::Percentile(std::vector<double> values, double ratio) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const size_t idx = static_cast<size_t>(std::min<double>(values.size() - 1, std::floor(ratio * (values.size() - 1))));
    return values[idx];
}

}  // namespace lightning
