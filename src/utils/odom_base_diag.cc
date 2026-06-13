#include "utils/odom_base_diag.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <unistd.h>

#include <glog/logging.h>
#include <ros/ros.h>

namespace lightning {
namespace {

constexpr double kRadToDeg = 180.0 / M_PI;

template <typename T>
T GetYamlOr(const YAML::Node& node, const std::string& key, const T& fallback) {
    try {
        if (node && node[key]) {
            return node[key].as<T>();
        }
    } catch (const YAML::Exception& e) {
        LOG(WARNING) << "invalid odom_base_diagnostics config key " << key << ": " << e.what();
    }
    return fallback;
}

std::string BoolStr(bool value) { return value ? "true" : "false"; }

}  // namespace

OdomBaseDiagLogger::OdomBaseDiagLogger() = default;

OdomBaseDiagLogger::~OdomBaseDiagLogger() { Finish(); }

OdomBaseDiagLogger::Config OdomBaseDiagLogger::LoadConfig(const std::string& yaml_path) {
    Config cfg;
    try {
        const YAML::Node yaml = YAML::LoadFile(yaml_path);
        const YAML::Node root = yaml["odom_base_diagnostics"];
        if (!root) {
            return cfg;
        }
        cfg.enable = GetYamlOr<bool>(root, "enable", cfg.enable);
        cfg.csv_enable = GetYamlOr<bool>(root, "csv_enable", cfg.csv_enable);
        cfg.jsonl_enable = GetYamlOr<bool>(root, "jsonl_enable", cfg.jsonl_enable);
        cfg.summary_enable = GetYamlOr<bool>(root, "summary_enable", cfg.summary_enable);
        cfg.output_dir = GetYamlOr<std::string>(root, "output_dir", cfg.output_dir);
        cfg.event_output_dir = GetYamlOr<std::string>(root, "event_output_dir", cfg.event_output_dir);
        cfg.flush_every_n = std::max(1, GetYamlOr<int>(root, "flush_every_n", cfg.flush_every_n));
        cfg.log_every_n = std::max(1, GetYamlOr<int>(root, "log_every_n", cfg.log_every_n));

        cfg.vehicle_max_speed_mps = GetYamlOr<double>(root, "vehicle_max_speed_mps", cfg.vehicle_max_speed_mps);
        cfg.trans_warn_factor = GetYamlOr<double>(root, "trans_warn_factor", cfg.trans_warn_factor);
        cfg.trans_error_factor = GetYamlOr<double>(root, "trans_error_factor", cfg.trans_error_factor);
        cfg.min_trans_warn_m = GetYamlOr<double>(root, "min_trans_warn_m", cfg.min_trans_warn_m);
        cfg.min_trans_error_m = GetYamlOr<double>(root, "min_trans_error_m", cfg.min_trans_error_m);
        cfg.yaw_rate_warn_degps = GetYamlOr<double>(root, "yaw_rate_warn_degps", cfg.yaw_rate_warn_degps);
        cfg.yaw_rate_error_degps = GetYamlOr<double>(root, "yaw_rate_error_degps", cfg.yaw_rate_error_degps);
        cfg.min_yaw_warn_deg = GetYamlOr<double>(root, "min_yaw_warn_deg", cfg.min_yaw_warn_deg);
        cfg.min_yaw_error_deg = GetYamlOr<double>(root, "min_yaw_error_deg", cfg.min_yaw_error_deg);

        cfg.odom_min_hz = GetYamlOr<double>(root, "odom_min_hz", cfg.odom_min_hz);
        cfg.odom_max_age_sec = GetYamlOr<double>(root, "odom_max_age_sec", cfg.odom_max_age_sec);
        cfg.odom_max_gap_sec = GetYamlOr<double>(root, "odom_max_gap_sec", cfg.odom_max_gap_sec);
        cfg.tf_lookup_timeout_sec = GetYamlOr<double>(root, "tf_lookup_timeout_sec", cfg.tf_lookup_timeout_sec);
        cfg.tf_max_dt_sec = GetYamlOr<double>(root, "tf_max_dt_sec", cfg.tf_max_dt_sec);
        cfg.tf_max_age_sec = GetYamlOr<double>(root, "tf_max_age_sec", cfg.tf_max_age_sec);

        cfg.source_switch_warn_count = std::max(1, GetYamlOr<int>(root, "source_switch_warn_count",
                                                                  cfg.source_switch_warn_count));
        cfg.source_switch_window_sec =
            GetYamlOr<double>(root, "source_switch_window_sec", cfg.source_switch_window_sec);
        cfg.source_switch_error_count = std::max(cfg.source_switch_warn_count,
                                                 GetYamlOr<int>(root, "source_switch_error_count",
                                                                cfg.source_switch_error_count));

        cfg.stationary_enable = GetYamlOr<bool>(root, "stationary_enable", cfg.stationary_enable);
        cfg.stationary_speed_mps = GetYamlOr<double>(root, "stationary_speed_mps", cfg.stationary_speed_mps);
        cfg.stationary_trans_warn_m =
            GetYamlOr<double>(root, "stationary_trans_warn_m", cfg.stationary_trans_warn_m);
        cfg.stationary_trans_error_m =
            GetYamlOr<double>(root, "stationary_trans_error_m", cfg.stationary_trans_error_m);
        cfg.stationary_yaw_warn_deg =
            GetYamlOr<double>(root, "stationary_yaw_warn_deg", cfg.stationary_yaw_warn_deg);
        cfg.stationary_yaw_error_deg =
            GetYamlOr<double>(root, "stationary_yaw_error_deg", cfg.stationary_yaw_error_deg);

        cfg.compare_with_lio = GetYamlOr<bool>(root, "compare_with_lio", cfg.compare_with_lio);
        cfg.compare_with_map_odom = GetYamlOr<bool>(root, "compare_with_map_odom", cfg.compare_with_map_odom);
        cfg.compare_with_final_pose = GetYamlOr<bool>(root, "compare_with_final_pose", cfg.compare_with_final_pose);
        cfg.odom_lio_delta_warn_m = GetYamlOr<double>(root, "odom_lio_delta_warn_m", cfg.odom_lio_delta_warn_m);
        cfg.odom_lio_delta_error_m = GetYamlOr<double>(root, "odom_lio_delta_error_m", cfg.odom_lio_delta_error_m);
        cfg.odom_lio_yaw_warn_deg = GetYamlOr<double>(root, "odom_lio_yaw_warn_deg", cfg.odom_lio_yaw_warn_deg);
        cfg.odom_lio_yaw_error_deg = GetYamlOr<double>(root, "odom_lio_yaw_error_deg", cfg.odom_lio_yaw_error_deg);

        cfg.keep_recent_window_sec =
            GetYamlOr<double>(root, "keep_recent_window_sec", cfg.keep_recent_window_sec);
        cfg.write_fault_window_on_event =
            GetYamlOr<bool>(root, "write_fault_window_on_event", cfg.write_fault_window_on_event);
        cfg.fault_window_pre_sec = GetYamlOr<double>(root, "fault_window_pre_sec", cfg.fault_window_pre_sec);
        cfg.fault_window_post_sec = GetYamlOr<double>(root, "fault_window_post_sec", cfg.fault_window_post_sec);
    } catch (const YAML::Exception& e) {
        LOG(WARNING) << "failed to load odom_base_diagnostics config from " << yaml_path << ": " << e.what();
    }
    return cfg;
}

bool OdomBaseDiagLogger::Init(const std::string& yaml_path, const std::string& run_label) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialized_) {
        return config_.enable;
    }
    yaml_path_ = yaml_path;
    config_ = LoadConfig(yaml_path);
    if (!config_.enable) {
        initialized_ = true;
        return false;
    }

    run_id_ = MakeRunId(run_label);
    start_wall_time_ = NowWall();
    start_ros_time_ = NowRos(start_wall_time_);
    std::filesystem::create_directories(config_.output_dir);
    std::filesystem::create_directories(config_.event_output_dir);
    std::filesystem::create_directories(config_.output_dir + "/fault_windows");

    csv_header_ =
        "run_id,wall_time,ros_time,query_stamp,source_selected,source_prev,source_switch,"
        "source_switch_count_window,odom_msg_valid,odom_topic,odom_msg_stamp,odom_arrival_time,odom_age_sec,"
        "odom_dt_msg_sec,odom_dt_arrival_sec,odom_hz_ema,odom_frame_id,odom_child_frame_id,odom_msg_pose_x,"
        "odom_msg_pose_y,odom_msg_pose_z,odom_msg_roll_deg,odom_msg_pitch_deg,odom_msg_yaw_deg,odom_msg_twist_vx,"
        "odom_msg_twist_vy,odom_msg_twist_wz,odom_msg_speed_mps,odom_msg_finite,odom_msg_status,tf_lookup_enable,"
        "tf_lookup_ok,tf_parent_frame,tf_child_frame,tf_query_stamp,tf_stamp,tf_age_sec,tf_lookup_dt_sec,"
        "tf_lookup_timeout_sec,tf_error_type,tf_error_string,tf_pose_x,tf_pose_y,tf_pose_z,tf_roll_deg,"
        "tf_pitch_deg,tf_yaw_deg,selected_pose_x,selected_pose_y,selected_pose_z,selected_roll_deg,"
        "selected_pitch_deg,selected_yaw_deg,selected_delta_x,selected_delta_y,selected_delta_z,selected_delta_xy,"
        "selected_delta_yaw_deg,selected_speed_mps,selected_yaw_rate_degps,selected_acc_mps2,"
        "selected_yaw_acc_degps2,motion_trans_warn_th_m,motion_trans_error_th_m,motion_yaw_warn_th_deg,"
        "motion_yaw_error_th_deg,motion_status,stationary_detected,stationary_drift_xy,"
        "stationary_drift_yaw_deg,stationary_status,lio_valid,lio_delta_xy,lio_delta_yaw_deg,"
        "odom_vs_lio_delta_xy,odom_vs_lio_delta_yaw_deg,odom_lio_consistency_status,map_odom_valid,"
        "map_odom_delta_xy,map_odom_delta_yaw_deg,final_pose_valid,final_map_base_delta_xy,"
        "final_map_base_delta_yaw_deg,diagnosis_status,diagnosis_reason,severity,extra_json";
    if (config_.csv_enable) {
        const std::string path = config_.output_dir + "/odom_base_trace_" + run_id_ + ".csv";
        csv_.open(path, std::ios::out | std::ios::trunc);
        if (!csv_.is_open()) {
            LOG(WARNING) << "failed to open odom base csv: " << path;
        } else {
            csv_ << csv_header_ << "\n";
        }
    }
    if (config_.jsonl_enable) {
        const std::string path = config_.event_output_dir + "/odom_base_events_" + run_id_ + ".jsonl";
        jsonl_.open(path, std::ios::out | std::ios::trunc);
        if (!jsonl_.is_open()) {
            LOG(WARNING) << "failed to open odom base jsonl: " << path;
        }
    }
    initialized_ = true;
    return true;
}

void OdomBaseDiagLogger::ObserveOdom(const std::string& topic, const nav_msgs::Odometry& odom,
                                     double arrival_ros_time) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!config_.enable || !initialized_) {
        return;
    }
    const double stamp = odom.header.stamp.toSec();
    const double arrival = arrival_ros_time > 0.0 ? arrival_ros_time : NowRos(NowWall());
    const bool finite = OdomFinite(odom);
    PoseSample sample;
    sample.valid = finite && std::isfinite(stamp) && stamp > 0.0;
    sample.source = "odom_msg";
    sample.topic = topic.empty() ? "/odom" : topic;
    sample.stamp = stamp;
    sample.arrival_time = arrival;
    sample.frame_id = odom.header.frame_id;
    sample.child_frame_id = odom.child_frame_id;
    sample.pose = PoseFromOdom(odom);
    sample.twist_vx = odom.twist.twist.linear.x;
    sample.twist_vy = odom.twist.twist.linear.y;
    sample.twist_wz = odom.twist.twist.angular.z;
    sample.speed_mps = std::hypot(sample.twist_vx, sample.twist_vy);
    sample.finite = finite;

    if (has_odom_ && sample.valid && latest_odom_.valid) {
        const double dt_msg = sample.stamp - latest_odom_.stamp;
        if (dt_msg <= 0.0) {
            odom_non_monotonic_count_++;
        } else {
            odom_gaps_.push_back(dt_msg);
            const double hz = 1.0 / dt_msg;
            odom_hz_ema_ = odom_hz_ema_ <= 0.0 ? hz : 0.8 * odom_hz_ema_ + 0.2 * hz;
        }
    }
    if (prev_odom_arrival_ > 0.0 && arrival > prev_odom_arrival_) {
        const double hz = 1.0 / (arrival - prev_odom_arrival_);
        if (odom_hz_ema_ <= 0.0) {
            odom_hz_ema_ = hz;
        }
    }
    prev_odom_arrival_ = arrival;
    prev_odom_ = latest_odom_;
    latest_odom_ = sample;
    has_odom_ = true;
    odom_total_count_++;
    if (sample.valid) {
        odom_valid_count_++;
    }

    Row row = BuildRow(arrival);
    WriteCsvRow(row);
    AppendRecentRow(row);
}

void OdomBaseDiagLogger::ObserveTf(const TfObservation& observation) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!config_.enable || !initialized_) {
        return;
    }
    latest_tf_.lookup_enable = observation.lookup_enable;
    latest_tf_.lookup_ok = observation.lookup_ok;
    latest_tf_.parent_frame = observation.parent_frame;
    latest_tf_.child_frame = observation.child_frame;
    latest_tf_.query_stamp = observation.query_stamp > 0.0 ? observation.query_stamp : NowRos(NowWall());
    latest_tf_.lookup_timeout_sec = observation.lookup_timeout_sec > 0.0 ? observation.lookup_timeout_sec
                                                                         : config_.tf_lookup_timeout_sec;
    latest_tf_.error_type = observation.lookup_ok ? "NONE" :
                            (observation.error_type.empty() ? "TF_UNKNOWN" : observation.error_type);
    latest_tf_.error_string = observation.error_string;
    tf_lookup_count_++;
    if (observation.lookup_ok) {
        latest_tf_.stamp = observation.tf_msg.header.stamp.toSec();
        latest_tf_.lookup_dt_sec = latest_tf_.query_stamp - latest_tf_.stamp;
        latest_tf_.pose = PoseFromTf(observation.tf_msg);
        latest_tf_.pose_valid = PoseFinite(latest_tf_.pose) && latest_tf_.stamp > 0.0;
        tf_lookup_ok_count_++;
        tf_lookup_dts_.push_back(std::fabs(latest_tf_.lookup_dt_sec));
    } else {
        latest_tf_.pose_valid = false;
        latest_tf_.stamp = 0.0;
        latest_tf_.lookup_dt_sec = 0.0;
        tf_lookup_fail_count_++;
        if (latest_tf_.error_type == "TF_TIMEOUT") {
            tf_timeout_count_++;
        } else if (latest_tf_.error_type == "TF_EXTRAPOLATION_FUTURE") {
            tf_future_count_++;
        } else if (latest_tf_.error_type == "TF_EXTRAPOLATION_PAST") {
            tf_past_count_++;
        } else if (latest_tf_.error_type == "TF_DISCONNECTED") {
            tf_disconnected_count_++;
        }
    }

    Row row = BuildRow(latest_tf_.query_stamp);
    WriteCsvRow(row);
    AppendRecentRow(row);
}

void OdomBaseDiagLogger::ObserveLio(const NavState& state) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!config_.enable || !initialized_ || !state.pose_is_ok_) {
        return;
    }
    if (has_prev_lio_state_ && state.timestamp_ > prev_lio_state_.timestamp_) {
        const SE3 prev(prev_lio_state_.rot_, prev_lio_state_.pos_);
        const SE3 curr(state.rot_, state.pos_);
        const SE3 delta = prev.inverse() * curr;
        latest_lio_delta_.valid = true;
        latest_lio_delta_.stamp = state.timestamp_;
        latest_lio_delta_.delta_xy = Norm2d(delta.translation());
        latest_lio_delta_.delta_yaw_deg = RelYawDeg(SE3(), delta);
    }
    prev_lio_state_ = state;
    has_prev_lio_state_ = true;
}

void OdomBaseDiagLogger::ObserveFinalPose(const loc::LocalizationResult& result, const std::string& source) {
    (void)source;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!config_.enable || !initialized_ || !result.valid_) {
        return;
    }
    if (has_prev_final_result_ && result.timestamp_ > prev_final_result_.timestamp_) {
        const SE3 delta = prev_final_result_.pose_.inverse() * result.pose_;
        latest_final_delta_.valid = true;
        latest_final_delta_.stamp = result.timestamp_;
        latest_final_delta_.delta_xy = Norm2d(delta.translation());
        latest_final_delta_.delta_yaw_deg = RelYawDeg(SE3(), delta);
    }
    prev_final_result_ = result;
    has_prev_final_result_ = true;
}

void OdomBaseDiagLogger::Finish() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (finished_) {
        return;
    }
    if (initialized_ && config_.enable && config_.summary_enable) {
        WriteSummary();
    }
    if (csv_.is_open()) csv_.close();
    if (jsonl_.is_open()) jsonl_.close();
    finished_ = true;
}

OdomBaseDiagLogger::Row OdomBaseDiagLogger::BuildRow(double ros_time_hint) {
    Row row;
    const double wall_time = NowWall();
    const double ros_time = ros_time_hint > 0.0 ? ros_time_hint : NowRos(wall_time);
    const double query_stamp = latest_tf_.query_stamp > 0.0 ? latest_tf_.query_stamp : ros_time;

    const bool tf_fresh = latest_tf_.lookup_ok && latest_tf_.pose_valid &&
                          std::fabs(query_stamp - latest_tf_.stamp) <= config_.tf_max_age_sec;
    PoseSample selected;
    std::string source_selected = "unknown";
    if (tf_fresh) {
        selected.valid = true;
        selected.source = "external_tf";
        selected.stamp = latest_tf_.stamp;
        selected.arrival_time = query_stamp;
        selected.frame_id = latest_tf_.parent_frame;
        selected.child_frame_id = latest_tf_.child_frame;
        selected.pose = latest_tf_.pose;
        selected.finite = true;
        source_selected = "external_tf";
    } else if (latest_odom_.valid) {
        selected = latest_odom_;
        source_selected = "odom_msg";
    } else if (latest_lio_delta_.valid) {
        selected.valid = false;
        selected.source = "internal_lio";
        source_selected = "internal_lio";
    }

    const std::string source_prev = prev_source_selected_;
    const bool source_switch = !source_prev.empty() && source_selected != source_prev;
    if (source_switch) {
        source_switch_count_++;
        source_switch_times_.push_back(ros_time);
    }
    while (!source_switch_times_.empty() &&
           source_switch_times_.front() < ros_time - config_.source_switch_window_sec) {
        source_switch_times_.pop_front();
    }
    max_source_switch_count_window_ =
        std::max<int64_t>(max_source_switch_count_window_, static_cast<int64_t>(source_switch_times_.size()));

    double odom_dt_msg = 0.0;
    double odom_dt_arrival = 0.0;
    if (latest_odom_.valid && prev_odom_.valid) {
        odom_dt_msg = latest_odom_.stamp - prev_odom_.stamp;
        odom_dt_arrival = latest_odom_.arrival_time - prev_odom_.arrival_time;
    }
    const double odom_age = latest_odom_.valid ? ros_time - latest_odom_.stamp : 0.0;
    if (latest_odom_.valid) {
        odom_ages_.push_back(std::fabs(odom_age));
    }
    std::string odom_status = "OK";
    if (!has_odom_) {
        odom_status = "NOT_RECEIVED";
    } else if (!latest_odom_.finite) {
        odom_status = "INVALID_NAN";
    } else if (odom_dt_msg < 0.0) {
        odom_status = "NON_MONOTONIC";
    } else if (odom_dt_msg > config_.odom_max_gap_sec) {
        odom_status = "GAP";
    } else if (latest_odom_.valid && odom_age > config_.odom_max_age_sec) {
        odom_status = "TIMEOUT";
    } else if (odom_hz_ema_ > 0.0 && odom_hz_ema_ < config_.odom_min_hz) {
        odom_status = "LOW_RATE";
    }
    if (odom_status == "TIMEOUT" || odom_status == "NOT_RECEIVED") {
        odom_timeout_count_++;
    }
    if (odom_status == "LOW_RATE") {
        odom_low_rate_count_++;
    }

    double selected_dt = 0.0;
    bool have_prev_selected = selected.valid && prev_selected_.valid;
    if (have_prev_selected) {
        selected_dt = selected.stamp - prev_selected_.stamp;
    }
    const double safe_dt = selected_dt > 1e-6 ? selected_dt : 0.0;
    const SE3 selected_delta = have_prev_selected ? prev_selected_.pose.inverse() * selected.pose : SE3();
    const double delta_x = have_prev_selected ? selected_delta.translation().x() : 0.0;
    const double delta_y = have_prev_selected ? selected_delta.translation().y() : 0.0;
    const double delta_z = have_prev_selected ? selected_delta.translation().z() : 0.0;
    const double delta_xy = have_prev_selected ? Norm2d(selected_delta.translation()) : 0.0;
    const double delta_yaw = have_prev_selected ? RelYawDeg(SE3(), selected_delta) : 0.0;
    const double speed = safe_dt > 0.0 ? delta_xy / safe_dt : 0.0;
    const double yaw_rate = safe_dt > 0.0 ? std::fabs(delta_yaw) / safe_dt : 0.0;
    const double acc = safe_dt > 0.0 && last_selected_dt_ > 0.0 ? (speed - last_selected_speed_) / safe_dt : 0.0;
    const double yaw_acc =
        safe_dt > 0.0 && last_selected_dt_ > 0.0 ? (yaw_rate - last_selected_yaw_rate_) / safe_dt : 0.0;

    const double trans_warn_th =
        std::max(config_.min_trans_warn_m, config_.vehicle_max_speed_mps * safe_dt * config_.trans_warn_factor);
    const double trans_error_th =
        std::max(config_.min_trans_error_m, config_.vehicle_max_speed_mps * safe_dt * config_.trans_error_factor);
    const double yaw_warn_th = std::max(config_.min_yaw_warn_deg, config_.yaw_rate_warn_degps * safe_dt);
    const double yaw_error_th = std::max(config_.min_yaw_error_deg, config_.yaw_rate_error_degps * safe_dt);

    std::string motion_status = "OK";
    if (!selected.valid) {
        motion_status = "INVALID";
    } else if (have_prev_selected && selected_dt <= 0.0) {
        motion_status = "NON_MONOTONIC";
    } else if (safe_dt > config_.odom_max_gap_sec && have_prev_selected) {
        motion_status = "TIME_GAP";
    } else if (delta_xy > trans_error_th) {
        motion_status = "TRANS_ERROR";
    } else if (std::fabs(delta_yaw) > yaw_error_th) {
        motion_status = "YAW_ERROR";
    } else if (delta_xy > trans_warn_th) {
        motion_status = "TRANS_WARN";
    } else if (std::fabs(delta_yaw) > yaw_warn_th) {
        motion_status = "YAW_WARN";
    }

    const bool stationary_detected =
        config_.stationary_enable && latest_odom_.valid && latest_odom_.speed_mps <= config_.stationary_speed_mps;
    std::string stationary_status = "OK";
    if (stationary_detected &&
        (delta_xy > config_.stationary_trans_error_m || std::fabs(delta_yaw) > config_.stationary_yaw_error_deg)) {
        stationary_status = "STATIONARY_ERROR";
    } else if (stationary_detected &&
               (delta_xy > config_.stationary_trans_warn_m || std::fabs(delta_yaw) > config_.stationary_yaw_warn_deg)) {
        stationary_status = "STATIONARY_WARN";
    }

    double odom_vs_lio_xy = 0.0;
    double odom_vs_lio_yaw = 0.0;
    std::string odom_lio_status = "OK";
    const bool lio_valid = config_.compare_with_lio && latest_lio_delta_.valid &&
                           std::fabs(latest_lio_delta_.stamp - selected.stamp) < std::max(1.0, 3.0 * safe_dt);
    if (lio_valid && selected.valid && have_prev_selected) {
        odom_vs_lio_xy = std::fabs(delta_xy - latest_lio_delta_.delta_xy);
        odom_vs_lio_yaw = std::fabs(std::fabs(delta_yaw) - std::fabs(latest_lio_delta_.delta_yaw_deg));
        if (odom_vs_lio_xy > config_.odom_lio_delta_error_m ||
            odom_vs_lio_yaw > config_.odom_lio_yaw_error_deg) {
            odom_lio_status = "ODOM_LIO_ERROR";
        } else if (odom_vs_lio_xy > config_.odom_lio_delta_warn_m ||
                   odom_vs_lio_yaw > config_.odom_lio_yaw_warn_deg) {
            odom_lio_status = "ODOM_LIO_WARN";
        }
    } else {
        odom_lio_status = "LIO_MISSING";
    }

    double odom_tf_xy = 0.0;
    double odom_tf_yaw = 0.0;
    bool odom_tf_inconsistent = false;
    if (latest_odom_.valid && latest_tf_.lookup_ok && latest_tf_.pose_valid &&
        std::fabs(latest_odom_.stamp - latest_tf_.stamp) <= std::max(0.2, config_.tf_max_dt_sec)) {
        const SE3 diff = latest_odom_.pose.inverse() * latest_tf_.pose;
        odom_tf_xy = Norm2d(diff.translation());
        odom_tf_yaw = std::fabs(RelYawDeg(SE3(), diff));
        odom_tf_inconsistent = odom_tf_xy > config_.odom_lio_delta_warn_m ||
                               odom_tf_yaw > config_.odom_lio_yaw_warn_deg;
    }

    std::string diagnosis = "OK";
    std::string reason = "ok";
    std::string severity = "INFO";
    const bool source_switching_warn =
        static_cast<int>(source_switch_times_.size()) >= config_.source_switch_warn_count;
    const bool source_switching_error =
        static_cast<int>(source_switch_times_.size()) >= config_.source_switch_error_count;
    const bool motion_warn = motion_status == "TRANS_WARN" || motion_status == "YAW_WARN";
    const bool motion_error = motion_status == "TRANS_ERROR" || motion_status == "YAW_ERROR";
    const bool motion_gap = motion_status == "TIME_GAP" || motion_status == "NON_MONOTONIC";
    const bool map_odom_jump = false;  // 当前定位链路没有显式 map->odom 样本，保留字段用于与 trace 对齐。

    if (latest_tf_.lookup_enable && !latest_tf_.lookup_ok) {
        diagnosis = "TF_LOOKUP_ERROR";
        reason = "odom->base_link tf lookup failed";
        severity = "ERROR";
        if (first_tf_error_time_ == 0.0) first_tf_error_time_ = ros_time;
    } else if (source_switch && motion_error) {
        diagnosis = "TF_SOURCE_SWITCH_JUMP";
        reason = "source switched with selected odom jump";
        severity = "ERROR";
    } else if (source_switching_warn) {
        diagnosis = "TF_SOURCE_SWITCHING";
        reason = "odom source switches frequently";
        severity = source_switching_error ? "ERROR" : "WARN";
    } else if (odom_status == "TIMEOUT" || odom_status == "NOT_RECEIVED" || odom_status == "LOW_RATE") {
        diagnosis = "CHASSIS_ODOM_TIMEOUT";
        reason = "odom message timeout or low rate";
        severity = odom_status == "LOW_RATE" ? "WARN" : "ERROR";
    } else if (motion_gap) {
        diagnosis = "CHASSIS_ODOM_TIMEOUT";
        reason = "selected odom timestamp gap or non-monotonic dt";
        severity = "ERROR";
    } else if (stationary_status != "OK") {
        diagnosis = "STATIONARY_ODOM_DRIFT";
        reason = "odom drifts while stationary speed is near zero";
        severity = stationary_status == "STATIONARY_ERROR" ? "ERROR" : "WARN";
    } else if (motion_error) {
        diagnosis = "CHASSIS_ODOM_JUMP";
        reason = "selected odom->base_link delta exceeds vehicle speed bound";
        severity = "ERROR";
    } else if (motion_warn) {
        diagnosis = "CHASSIS_ODOM_SUSPECT";
        reason = "selected odom->base_link delta exceeds warning threshold";
        severity = "WARN";
    } else if (odom_tf_inconsistent) {
        diagnosis = "ODOM_MSG_TF_INCONSISTENT";
        reason = "odom message pose differs from odom->base_link tf";
        severity = "WARN";
    } else if (odom_lio_status == "ODOM_LIO_ERROR") {
        diagnosis = "ODOM_LIO_DIVERGED";
        reason = "odom delta diverges from lio delta";
        severity = "ERROR";
    } else if (map_odom_jump) {
        diagnosis = "ODOM_NORMAL_MAP_ODOM_JUMP";
        reason = "odom is smooth but map->odom jumps";
        severity = "ERROR";
    } else if (!lio_valid || !latest_final_delta_.valid) {
        diagnosis = "UNKNOWN_NEED_MORE_TRACE";
        reason = "missing lio or final pose comparison";
        severity = "INFO";
    }

    total_rows_++;
    if (diagnosis == "CHASSIS_ODOM_SUSPECT") {
        chassis_suspect_count_++;
    } else if (diagnosis == "CHASSIS_ODOM_JUMP") {
        chassis_jump_count_++;
        if (first_chassis_jump_time_ == 0.0) first_chassis_jump_time_ = ros_time;
    } else if (diagnosis == "STATIONARY_ODOM_DRIFT") {
        stationary_drift_count_++;
    } else if (diagnosis == "ODOM_LIO_DIVERGED") {
        odom_lio_diverged_count_++;
    } else if (diagnosis == "ODOM_MSG_TF_INCONSISTENT") {
        odom_msg_tf_inconsistent_count_++;
    } else if (diagnosis == "ODOM_NORMAL_MAP_ODOM_JUMP") {
        odom_normal_map_odom_jump_count_++;
    } else if (diagnosis == "TF_LOOKUP_ERROR") {
        tf_lookup_error_count_++;
    } else if (diagnosis == "TF_SOURCE_SWITCHING") {
        source_switching_count_++;
    } else if (diagnosis == "TF_SOURCE_SWITCH_JUMP") {
        tf_source_switch_jump_count_++;
    }
    if (selected.valid && have_prev_selected && safe_dt > 0.0 && !motion_gap) {
        selected_delta_xys_.push_back(delta_xy);
        selected_yaw_rates_.push_back(yaw_rate);
    }

    auto add = [&row](const auto& value) {
        std::ostringstream oss;
        oss << value;
        row.cells.push_back(oss.str());
    };
    auto add_double = [&row](double value) { row.cells.push_back(FormatDouble(value)); };
    auto add_bool = [&row](bool value) { row.cells.push_back(BoolStr(value)); };
    const Vec3d selected_rpy = selected.valid ? RpyDeg(selected.pose) : Vec3d::Zero();
    const Vec3d odom_rpy = latest_odom_.valid ? RpyDeg(latest_odom_.pose) : Vec3d::Zero();
    const Vec3d tf_rpy = latest_tf_.pose_valid ? RpyDeg(latest_tf_.pose) : Vec3d::Zero();
    const std::string extra_json = "{\"odom_tf_delta_xy\":" + FormatDouble(odom_tf_xy) +
                                   ",\"odom_tf_delta_yaw_deg\":" + FormatDouble(odom_tf_yaw) + "}";

    add(run_id_);
    add_double(wall_time);
    add_double(ros_time);
    add_double(query_stamp);
    add(source_selected);
    add(source_prev);
    add_bool(source_switch);
    add(static_cast<int>(source_switch_times_.size()));
    add_bool(latest_odom_.valid);
    add(latest_odom_.topic);
    add_double(latest_odom_.stamp);
    add_double(latest_odom_.arrival_time);
    add_double(odom_age);
    add_double(odom_dt_msg);
    add_double(odom_dt_arrival);
    add_double(odom_hz_ema_);
    add(latest_odom_.frame_id);
    add(latest_odom_.child_frame_id);
    add_double(latest_odom_.valid ? latest_odom_.pose.translation().x() : 0.0);
    add_double(latest_odom_.valid ? latest_odom_.pose.translation().y() : 0.0);
    add_double(latest_odom_.valid ? latest_odom_.pose.translation().z() : 0.0);
    add_double(odom_rpy.x());
    add_double(odom_rpy.y());
    add_double(odom_rpy.z());
    add_double(latest_odom_.twist_vx);
    add_double(latest_odom_.twist_vy);
    add_double(latest_odom_.twist_wz);
    add_double(latest_odom_.speed_mps);
    add_bool(latest_odom_.finite);
    add(odom_status);
    add_bool(latest_tf_.lookup_enable);
    add_bool(latest_tf_.lookup_ok);
    add(latest_tf_.parent_frame);
    add(latest_tf_.child_frame);
    add_double(latest_tf_.query_stamp);
    add_double(latest_tf_.stamp);
    add_double(latest_tf_.stamp > 0.0 ? query_stamp - latest_tf_.stamp : 0.0);
    add_double(latest_tf_.lookup_dt_sec);
    add_double(latest_tf_.lookup_timeout_sec);
    add(latest_tf_.error_type);
    add(latest_tf_.error_string);
    add_double(latest_tf_.pose_valid ? latest_tf_.pose.translation().x() : 0.0);
    add_double(latest_tf_.pose_valid ? latest_tf_.pose.translation().y() : 0.0);
    add_double(latest_tf_.pose_valid ? latest_tf_.pose.translation().z() : 0.0);
    add_double(tf_rpy.x());
    add_double(tf_rpy.y());
    add_double(tf_rpy.z());
    add_double(selected.valid ? selected.pose.translation().x() : 0.0);
    add_double(selected.valid ? selected.pose.translation().y() : 0.0);
    add_double(selected.valid ? selected.pose.translation().z() : 0.0);
    add_double(selected_rpy.x());
    add_double(selected_rpy.y());
    add_double(selected_rpy.z());
    add_double(delta_x);
    add_double(delta_y);
    add_double(delta_z);
    add_double(delta_xy);
    add_double(delta_yaw);
    add_double(speed);
    add_double(yaw_rate);
    add_double(acc);
    add_double(yaw_acc);
    add_double(trans_warn_th);
    add_double(trans_error_th);
    add_double(yaw_warn_th);
    add_double(yaw_error_th);
    add(motion_status);
    add_bool(stationary_detected);
    add_double(stationary_detected ? delta_xy : 0.0);
    add_double(stationary_detected ? delta_yaw : 0.0);
    add(stationary_status);
    add_bool(lio_valid);
    add_double(latest_lio_delta_.valid ? latest_lio_delta_.delta_xy : 0.0);
    add_double(latest_lio_delta_.valid ? latest_lio_delta_.delta_yaw_deg : 0.0);
    add_double(odom_vs_lio_xy);
    add_double(odom_vs_lio_yaw);
    add(odom_lio_status);
    add_bool(false);
    add_double(0.0);
    add_double(0.0);
    add_bool(latest_final_delta_.valid);
    add_double(latest_final_delta_.valid ? latest_final_delta_.delta_xy : 0.0);
    add_double(latest_final_delta_.valid ? latest_final_delta_.delta_yaw_deg : 0.0);
    add(diagnosis);
    add(reason);
    add(severity);
    add(extra_json);

    row.ros_time = ros_time;
    row.diagnosis_status = diagnosis;
    row.severity = severity;
    WriteEventIfNeeded(row);

    if (selected.valid) {
        prev_selected_ = selected;
        prev_source_selected_ = source_selected;
        last_selected_speed_ = speed;
        last_selected_yaw_rate_ = yaw_rate;
        if (safe_dt > 0.0) {
            last_selected_dt_ = safe_dt;
        }
    }
    return row;
}

void OdomBaseDiagLogger::WriteCsvRow(const Row& row) {
    if (!csv_.is_open()) {
        return;
    }
    if (config_.log_every_n > 1 && total_rows_ % config_.log_every_n != 0 && row.severity == "INFO") {
        return;
    }
    for (std::size_t i = 0; i < row.cells.size(); ++i) {
        if (i) csv_ << ",";
        csv_ << CsvEscape(row.cells[i]);
    }
    csv_ << "\n";
    csv_rows_since_flush_++;
    if (csv_rows_since_flush_ >= config_.flush_every_n) {
        csv_.flush();
        csv_rows_since_flush_ = 0;
    }
}

void OdomBaseDiagLogger::WriteEventIfNeeded(const Row& row) {
    const bool status_changed = row.diagnosis_status != last_diagnosis_status_;
    const bool abnormal = row.severity == "WARN" || row.severity == "ERROR";
    if (!jsonl_.is_open() || (!status_changed && !abnormal)) {
        last_diagnosis_status_ = row.diagnosis_status;
        return;
    }
    const std::string motion_status = row.cells.size() > 66 ? row.cells[66] : "";
    const std::string odom_status = row.cells.size() > 29 ? row.cells[29] : "";
    const std::string tf_error_type = row.cells.size() > 39 ? row.cells[39] : "NONE";
    const int event_id = row.diagnosis_status == "OK" ? 3099
                                                       : EventIdFor(row.diagnosis_status, motion_status, odom_status,
                                                                    tf_error_type);
    jsonl_ << "{\"run_id\":\"" << JsonEscape(run_id_) << "\",\"event_time_ros\":" << FormatDouble(row.ros_time)
           << ",\"event_time_wall\":" << FormatDouble(NowWall())
           << ",\"event_id\":" << event_id
           << ",\"module\":\"odom_base_diag\",\"status\":\"" << JsonEscape(row.diagnosis_status)
           << "\",\"severity\":\"" << JsonEscape(row.severity) << "\",\"reason\":\""
           << JsonEscape(row.cells.size() > 84 ? row.cells[84] : "") << "\",\"source_selected\":\""
           << JsonEscape(row.cells.size() > 4 ? row.cells[4] : "unknown") << "\",\"dt_sec\":"
           << FormatDouble(last_selected_dt_) << ",\"vehicle_max_speed_mps\":"
           << FormatDouble(config_.vehicle_max_speed_mps) << ",\"selected_delta_xy\":"
           << JsonEscape(row.cells.size() > 56 ? row.cells[56] : "0") << ",\"trans_error_th\":"
           << JsonEscape(row.cells.size() > 63 ? row.cells[63] : "0") << ",\"selected_delta_yaw_deg\":"
           << JsonEscape(row.cells.size() > 57 ? row.cells[57] : "0") << ",\"yaw_error_th_deg\":"
           << JsonEscape(row.cells.size() > 65 ? row.cells[65] : "0") << ",\"tf_error_type\":\""
           << JsonEscape(tf_error_type) << "\",\"source_switch\":"
           << (row.cells.size() > 6 ? row.cells[6] : "false") << ",\"odom_vs_lio_delta_xy\":"
           << (row.cells.size() > 74 ? row.cells[74] : "0")
           << ",\"suggestion\":\"check chassis odom publisher, TF timestamp, encoder speed, frame_id/child_frame_id, and compare with LIO trace\"}\n";
    jsonl_.flush();
    last_diagnosis_status_ = row.diagnosis_status;
    if (config_.write_fault_window_on_event &&
        (row.diagnosis_status == "CHASSIS_ODOM_JUMP" || row.diagnosis_status == "TF_SOURCE_SWITCH_JUMP" ||
         row.diagnosis_status == "ODOM_MSG_TF_INCONSISTENT" ||
         row.diagnosis_status == "STATIONARY_ODOM_DRIFT" || row.diagnosis_status == "ODOM_LIO_DIVERGED")) {
        MaybeWriteFaultWindow(row);
    }
}

void OdomBaseDiagLogger::WriteSummary() {
    const std::string path = config_.output_dir + "/odom_base_summary_" + run_id_ + ".md";
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        LOG(WARNING) << "failed to open odom base summary: " << path;
        return;
    }
    const double end_wall = NowWall();
    out << "# Odom Base Diagnostics Summary\n\n";
    out << "- run_id: `" << run_id_ << "`\n";
    out << "- start_time_wall: `" << FormatDouble(start_wall_time_) << "`\n";
    out << "- end_time_wall: `" << FormatDouble(end_wall) << "`\n";
    out << "- duration_sec: `" << FormatDouble(end_wall - start_wall_time_) << "`\n";
    out << "- config_file: `" << yaml_path_ << "`\n";
    out << "- vehicle_max_speed_mps = " << FormatDouble(config_.vehicle_max_speed_mps) << "\n\n";

    out << "## Thresholds\n\n";
    out << "- trans_warn_factor: `" << FormatDouble(config_.trans_warn_factor) << "`\n";
    out << "- trans_error_factor: `" << FormatDouble(config_.trans_error_factor) << "`\n";
    out << "- yaw_rate_warn_degps: `" << FormatDouble(config_.yaw_rate_warn_degps) << "`\n";
    out << "- yaw_rate_error_degps: `" << FormatDouble(config_.yaw_rate_error_degps) << "`\n";
    out << "- odom_min_hz: `" << FormatDouble(config_.odom_min_hz) << "`\n";
    out << "- tf_lookup_timeout_sec: `" << FormatDouble(config_.tf_lookup_timeout_sec) << "`\n\n";

    out << "## Odom Message Statistics\n\n";
    out << "| metric | value |\n|---|---:|\n";
    out << "| total_count | " << odom_total_count_ << " |\n";
    out << "| valid_count | " << odom_valid_count_ << " |\n";
    out << "| mean_hz | " << FormatDouble(MeanOrZero(odom_gaps_) > 0.0 ? 1.0 / MeanOrZero(odom_gaps_) : 0.0) << " |\n";
    out << "| p95_age | " << FormatDouble(Percentile(odom_ages_, 0.95)) << " |\n";
    out << "| max_age | " << FormatDouble(MaxOrZero(odom_ages_)) << " |\n";
    out << "| max_gap | " << FormatDouble(MaxOrZero(odom_gaps_)) << " |\n";
    out << "| non_monotonic_count | " << odom_non_monotonic_count_ << " |\n";
    out << "| timeout_count | " << odom_timeout_count_ << " |\n";
    out << "| low_rate_count | " << odom_low_rate_count_ << " |\n\n";

    out << "## TF Query Statistics\n\n";
    out << "| metric | value |\n|---|---:|\n";
    out << "| lookup_count | " << tf_lookup_count_ << " |\n";
    out << "| lookup_ok_count | " << tf_lookup_ok_count_ << " |\n";
    out << "| lookup_fail_count | " << tf_lookup_fail_count_ << " |\n";
    out << "| timeout_count | " << tf_timeout_count_ << " |\n";
    out << "| future_extrapolation_count | " << tf_future_count_ << " |\n";
    out << "| past_extrapolation_count | " << tf_past_count_ << " |\n";
    out << "| disconnected_count | " << tf_disconnected_count_ << " |\n";
    out << "| p95_lookup_dt | " << FormatDouble(Percentile(tf_lookup_dts_, 0.95)) << " |\n";
    out << "| max_lookup_dt | " << FormatDouble(MaxOrZero(tf_lookup_dts_)) << " |\n\n";

    out << "## Odom Base Motion Statistics\n\n";
    out << "| metric | value |\n|---|---:|\n";
    out << "| mean_delta_xy | " << FormatDouble(MeanOrZero(selected_delta_xys_)) << " |\n";
    out << "| p95_delta_xy | " << FormatDouble(Percentile(selected_delta_xys_, 0.95)) << " |\n";
    out << "| max_delta_xy | " << FormatDouble(MaxOrZero(selected_delta_xys_)) << " |\n";
    out << "| mean_yaw_rate | " << FormatDouble(MeanOrZero(selected_yaw_rates_)) << " |\n";
    out << "| p95_yaw_rate | " << FormatDouble(Percentile(selected_yaw_rates_, 0.95)) << " |\n";
    out << "| max_yaw_rate | " << FormatDouble(MaxOrZero(selected_yaw_rates_)) << " |\n";
    out << "| CHASSIS_ODOM_SUSPECT count | " << chassis_suspect_count_ << " |\n";
    out << "| CHASSIS_ODOM_JUMP count | " << chassis_jump_count_ << " |\n";
    out << "| STATIONARY_ODOM_DRIFT count | " << stationary_drift_count_ << " |\n\n";

    out << "## Source Switching\n\n";
    const double denom = std::max<int64_t>(1, total_rows_);
    out << "- external_tf ratio: `" << FormatDouble(tf_lookup_ok_count_ / denom) << "`\n";
    out << "- internal_lio ratio: `0.000000`\n";
    out << "- fallback ratio: `0.000000`\n";
    out << "- source_switch_count: " << source_switch_count_ << "\n";
    out << "- max_source_switch_count_window: " << max_source_switch_count_window_ << "\n";
    out << "- TF_SOURCE_SWITCH_JUMP count: " << tf_source_switch_jump_count_ << "\n\n";

    out << "## Cross Chain Comparison\n\n";
    out << "- ODOM_LIO_DIVERGED count: " << odom_lio_diverged_count_ << "\n";
    out << "- ODOM_NORMAL_MAP_ODOM_JUMP count: " << odom_normal_map_odom_jump_count_ << "\n";
    out << "- ODOM_MSG_TF_INCONSISTENT count: " << odom_msg_tf_inconsistent_count_ << "\n\n";

    out << "## Root Cause Guess\n\n";
    if (first_chassis_jump_time_ > 0.0) {
        out << "- 优先怀疑底盘 odom 或 odom->base_link TF 先跳。\n";
    } else if (first_tf_error_time_ > 0.0 || tf_source_switch_jump_count_ > 0) {
        out << "- 优先怀疑 TF 查询失败或 external/internal source 切换造成最终轨迹扰动。\n";
    } else if (odom_normal_map_odom_jump_count_ > 0) {
        out << "- 优先怀疑 NDT / LidarLoc / map->odom 输出链路，不应归因为底盘 odom。\n";
    } else if (odom_lio_diverged_count_ > 0) {
        out << "- 只能判定 odom 与 LIO 不一致，需要结合 lio_guess_trace、input_health、lidar_loc_match_trace 进一步确认。\n";
    } else {
        out << "- 未观察到底盘 odom / odom->base_link 的明确根因。\n";
    }
    out << "\n上述根因初判需要运行日志/测试结果进一步确认。\n";
}

void OdomBaseDiagLogger::MaybeWriteFaultWindow(const Row& row) {
    const std::string path = config_.output_dir + "/fault_windows/fault_odom_base_" + FormatDouble(row.ros_time) +
                             "_" + std::to_string(++event_seq_) + ".csv";
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        return;
    }
    out << csv_header_ << "\n";
    for (const auto& r : recent_rows_) {
        if (r.ros_time >= row.ros_time - config_.fault_window_pre_sec && r.ros_time <= row.ros_time) {
            for (std::size_t i = 0; i < r.cells.size(); ++i) {
                if (i) out << ",";
                out << CsvEscape(r.cells[i]);
            }
            out << "\n";
        }
    }
    PendingFaultWindow pending;
    pending.path = path;
    pending.event_ros_time = row.ros_time;
    pending.end_ros_time = row.ros_time + config_.fault_window_post_sec;
    pending_fault_windows_.push_back(std::move(pending));
}

void OdomBaseDiagLogger::AppendRecentRow(const Row& row) {
    recent_rows_.push_back(row);
    const double keep_from = row.ros_time - std::max(config_.keep_recent_window_sec, config_.fault_window_pre_sec);
    while (!recent_rows_.empty() && recent_rows_.front().ros_time < keep_from) {
        recent_rows_.pop_front();
    }
    AppendPendingFaultWindows(row);
}

void OdomBaseDiagLogger::AppendPendingFaultWindows(const Row& row) {
    for (auto it = pending_fault_windows_.begin(); it != pending_fault_windows_.end();) {
        if (row.ros_time > it->event_ros_time && row.ros_time <= it->end_ros_time) {
            std::ofstream out(it->path, std::ios::out | std::ios::app);
            if (out.is_open()) {
                for (std::size_t i = 0; i < row.cells.size(); ++i) {
                    if (i) out << ",";
                    out << CsvEscape(row.cells[i]);
                }
                out << "\n";
            }
        }
        if (row.ros_time > it->end_ros_time) {
            it = pending_fault_windows_.erase(it);
        } else {
            ++it;
        }
    }
}

int OdomBaseDiagLogger::EventIdFor(const std::string& diagnosis, const std::string& motion_status,
                                   const std::string& odom_status, const std::string& tf_error_type) const {
    if (diagnosis == "CHASSIS_ODOM_TIMEOUT") {
        if (odom_status == "LOW_RATE") return 3012;
        if (odom_status == "NON_MONOTONIC") return 3013;
        return 3011;
    }
    if (diagnosis == "TF_LOOKUP_ERROR") {
        if (tf_error_type == "TF_EXTRAPOLATION_FUTURE") return 3022;
        if (tf_error_type == "TF_EXTRAPOLATION_PAST") return 3023;
        if (tf_error_type == "TF_DISCONNECTED") return 3024;
        return 3021;
    }
    if (diagnosis == "TF_SOURCE_SWITCHING") return 3031;
    if (diagnosis == "TF_SOURCE_SWITCH_JUMP") return 3033;
    if (diagnosis == "ODOM_MSG_TF_INCONSISTENT") return 3041;
    if (diagnosis == "STATIONARY_ODOM_DRIFT") return 3051;
    if (diagnosis == "ODOM_LIO_DIVERGED") return 3061;
    if (diagnosis == "ODOM_NORMAL_MAP_ODOM_JUMP") return 3071;
    if (motion_status == "TRANS_WARN") return 3001;
    if (motion_status == "TRANS_ERROR") return 3002;
    if (motion_status == "YAW_WARN") return 3003;
    if (motion_status == "YAW_ERROR") return 3004;
    return 3099;
}

std::string OdomBaseDiagLogger::MakeRunId(const std::string& run_label) {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d-%H%M%S") << "-" << getpid();
    if (!run_label.empty()) {
        oss << "-" << run_label;
    }
    return oss.str();
}

double OdomBaseDiagLogger::NowWall() { return ros::WallTime::now().toSec(); }

double OdomBaseDiagLogger::NowRos(double fallback) {
    try {
        const ros::Time t = ros::Time::now();
        return t.isZero() ? fallback : t.toSec();
    } catch (const ros::TimeNotInitializedException&) {
        return fallback;
    }
}

std::string OdomBaseDiagLogger::JsonEscape(const std::string& text) {
    std::ostringstream oss;
    for (const char c : text) {
        if (c == '"' || c == '\\') {
            oss << '\\' << c;
        } else if (c == '\n') {
            oss << "\\n";
        } else if (c == '\r') {
            oss << "\\r";
        } else {
            oss << c;
        }
    }
    return oss.str();
}

std::string OdomBaseDiagLogger::CsvEscape(const std::string& text) {
    if (text.find_first_of(",\"\n\r") == std::string::npos) {
        return text;
    }
    std::string out = "\"";
    for (const char c : text) {
        if (c == '"') out += '"';
        out += c;
    }
    out += '"';
    return out;
}

std::string OdomBaseDiagLogger::FormatDouble(double value) {
    if (!std::isfinite(value)) {
        return "nan";
    }
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << value;
    return oss.str();
}

double OdomBaseDiagLogger::Percentile(std::vector<double> values, double ratio) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const std::size_t idx =
        std::min(values.size() - 1, static_cast<std::size_t>(std::floor(ratio * (values.size() - 1))));
    return values[idx];
}

double OdomBaseDiagLogger::MeanOrZero(const std::vector<double>& values) {
    return values.empty() ? 0.0 : std::accumulate(values.begin(), values.end(), 0.0) /
                                      static_cast<double>(values.size());
}

double OdomBaseDiagLogger::MaxOrZero(const std::vector<double>& values) {
    return values.empty() ? 0.0 : *std::max_element(values.begin(), values.end());
}

double OdomBaseDiagLogger::YawDeg(const SE3& pose) { return pose.so3().log().z() * kRadToDeg; }

double OdomBaseDiagLogger::RelYawDeg(const SE3& from, const SE3& to) {
    const SE3 delta = from.inverse() * to;
    return delta.so3().log().z() * kRadToDeg;
}

Vec3d OdomBaseDiagLogger::RpyDeg(const SE3& pose) {
    return pose.so3().matrix().eulerAngles(0, 1, 2) * kRadToDeg;
}

double OdomBaseDiagLogger::Norm2d(const Vec3d& v) { return std::hypot(v.x(), v.y()); }

bool OdomBaseDiagLogger::PoseFinite(const SE3& pose) {
    return pose.translation().allFinite() && pose.unit_quaternion().coeffs().allFinite();
}

SE3 OdomBaseDiagLogger::PoseFromOdom(const nav_msgs::Odometry& odom) {
    const auto& p = odom.pose.pose.position;
    const auto& q = odom.pose.pose.orientation;
    return SE3(Quatd(q.w, q.x, q.y, q.z), Vec3d(p.x, p.y, p.z));
}

SE3 OdomBaseDiagLogger::PoseFromTf(const geometry_msgs::TransformStamped& tf_msg) {
    const auto& p = tf_msg.transform.translation;
    const auto& q = tf_msg.transform.rotation;
    return SE3(Quatd(q.w, q.x, q.y, q.z), Vec3d(p.x, p.y, p.z));
}

bool OdomBaseDiagLogger::OdomFinite(const nav_msgs::Odometry& odom) {
    const auto& p = odom.pose.pose.position;
    const auto& q = odom.pose.pose.orientation;
    const auto& t = odom.twist.twist;
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(q.x) &&
           std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) && std::isfinite(t.linear.x) &&
           std::isfinite(t.linear.y) && std::isfinite(t.angular.z);
}

}  // namespace lightning
