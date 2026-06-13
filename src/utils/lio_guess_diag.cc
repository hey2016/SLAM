#include "utils/lio_guess_diag.h"

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
constexpr double kDegToRad = M_PI / 180.0;

template <typename T>
T GetYamlOr(const YAML::Node& node, const std::string& key, const T& fallback) {
    try {
        if (node && node[key]) {
            return node[key].as<T>();
        }
    } catch (const YAML::Exception& e) {
        LOG(WARNING) << "invalid lio_guess_diagnostics config key " << key << ": " << e.what();
    }
    return fallback;
}

double SumOrZero(const std::vector<double>& values) {
    return std::accumulate(values.begin(), values.end(), 0.0);
}

double MeanOrZero(const std::vector<double>& values) {
    return values.empty() ? 0.0 : SumOrZero(values) / static_cast<double>(values.size());
}

double MaxOrZero(const std::vector<double>& values) {
    return values.empty() ? 0.0 : *std::max_element(values.begin(), values.end());
}

}  // namespace

LioGuessDiagLogger::LioGuessDiagLogger() = default;

LioGuessDiagLogger::~LioGuessDiagLogger() { Finish(); }

LioGuessDiagLogger::Config LioGuessDiagLogger::LoadConfig(const std::string& yaml_path) {
    Config cfg;
    try {
        const YAML::Node yaml = YAML::LoadFile(yaml_path);
        const YAML::Node root = yaml["lio_guess_diagnostics"];
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

        cfg.compare_with_chassis_odom =
            GetYamlOr<bool>(root, "compare_with_chassis_odom", cfg.compare_with_chassis_odom);
        cfg.lio_odom_delta_warn_m = GetYamlOr<double>(root, "lio_odom_delta_warn_m", cfg.lio_odom_delta_warn_m);
        cfg.lio_odom_delta_error_m = GetYamlOr<double>(root, "lio_odom_delta_error_m", cfg.lio_odom_delta_error_m);
        cfg.lio_odom_yaw_warn_deg = GetYamlOr<double>(root, "lio_odom_yaw_warn_deg", cfg.lio_odom_yaw_warn_deg);
        cfg.lio_odom_yaw_error_deg = GetYamlOr<double>(root, "lio_odom_yaw_error_deg", cfg.lio_odom_yaw_error_deg);

        cfg.guess_to_ndt_warn_m = GetYamlOr<double>(root, "guess_to_ndt_warn_m", cfg.guess_to_ndt_warn_m);
        cfg.guess_to_ndt_error_m = GetYamlOr<double>(root, "guess_to_ndt_error_m", cfg.guess_to_ndt_error_m);
        cfg.guess_to_ndt_yaw_warn_deg =
            GetYamlOr<double>(root, "guess_to_ndt_yaw_warn_deg", cfg.guess_to_ndt_yaw_warn_deg);
        cfg.guess_to_ndt_yaw_error_deg =
            GetYamlOr<double>(root, "guess_to_ndt_yaw_error_deg", cfg.guess_to_ndt_yaw_error_deg);

        cfg.suspect_confirm_frames = std::max(1, GetYamlOr<int>(root, "suspect_confirm_frames",
                                                                cfg.suspect_confirm_frames));
        cfg.error_confirm_frames = std::max(1, GetYamlOr<int>(root, "error_confirm_frames", cfg.error_confirm_frames));

        cfg.keep_recent_window_sec =
            GetYamlOr<double>(root, "keep_recent_window_sec", cfg.keep_recent_window_sec);
        cfg.write_fault_window_on_event =
            GetYamlOr<bool>(root, "write_fault_window_on_event", cfg.write_fault_window_on_event);
        cfg.fault_window_pre_sec = GetYamlOr<double>(root, "fault_window_pre_sec", cfg.fault_window_pre_sec);
        cfg.fault_window_post_sec = GetYamlOr<double>(root, "fault_window_post_sec", cfg.fault_window_post_sec);
    } catch (const YAML::Exception& e) {
        LOG(WARNING) << "failed to load lio_guess_diagnostics config from " << yaml_path << ": " << e.what();
    }
    return cfg;
}

bool LioGuessDiagLogger::Init(const std::string& yaml_path, const std::string& run_label) {
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

    if (config_.csv_enable) {
        const std::string path = config_.output_dir + "/lio_guess_trace_" + run_id_ + ".csv";
        csv_.open(path, std::ios::out | std::ios::trunc);
        if (!csv_.is_open()) {
            LOG(WARNING) << "failed to open lio guess csv: " << path;
        } else {
            csv_ << "run_id,wall_time,ros_time,frame_id,scan_seq,"
                    "lio_valid,lio_stamp,lio_dt_sec,lio_pose_x,lio_pose_y,lio_pose_z,lio_roll_deg,lio_pitch_deg,"
                    "lio_yaw_deg,lio_delta_x,lio_delta_y,lio_delta_z,lio_delta_xy,lio_delta_yaw_deg,lio_speed_mps,"
                    "lio_yaw_rate_degps,lio_acc_mps2,lio_yaw_acc_degps2,motion_trans_warn_th_m,"
                    "motion_trans_error_th_m,motion_yaw_warn_th_deg,motion_yaw_error_th_deg,lio_motion_status,"
                    "last_abs_pose_x,last_abs_pose_y,last_abs_pose_z,last_abs_pose_yaw_deg,guess_from_lo_x,"
                    "guess_from_lo_y,guess_from_lo_z,guess_from_lo_yaw_deg,last_abs_to_guess_xy,"
                    "last_abs_to_guess_yaw_deg,chassis_odom_valid,chassis_odom_stamp,chassis_odom_source,"
                    "chassis_odom_delta_xy,chassis_odom_delta_yaw_deg,lio_vs_odom_delta_xy,"
                    "lio_vs_odom_delta_yaw_deg,lio_odom_consistency_status,ndt_valid,ndt_converged,"
                    "ndt_confidence,ndt_score,ndt_pose_x,ndt_pose_y,ndt_pose_z,ndt_pose_yaw_deg,"
                    "guess_to_ndt_xy,guess_to_ndt_yaw_deg,icp_valid,icp_converged,icp_accepted,icp_pose_x,"
                    "icp_pose_y,icp_pose_z,icp_pose_yaw_deg,ndt_to_icp_xy,ndt_to_icp_yaw_deg,final_pose_valid,"
                    "final_pose_x,final_pose_y,final_pose_z,final_pose_yaw_deg,guess_to_final_xy,"
                    "guess_to_final_yaw_deg,ndt_to_final_xy,ndt_to_final_yaw_deg,loc_success,loc_inited,"
                    "loc_stage,reject_reason,diagnosis_status,diagnosis_reason,extra_json\n";
        }
    }
    if (config_.jsonl_enable) {
        const std::string path = config_.event_output_dir + "/lio_guess_events_" + run_id_ + ".jsonl";
        jsonl_.open(path, std::ios::out | std::ios::trunc);
        if (!jsonl_.is_open()) {
            LOG(WARNING) << "failed to open lio guess jsonl: " << path;
        }
    }
    initialized_ = true;
    return true;
}

void LioGuessDiagLogger::ObserveChassisOdom(const nav_msgs::Odometry& odom, double arrival_ros_time) {
    (void)arrival_ros_time;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!config_.enable) {
        return;
    }
    const double stamp = odom.header.stamp.toSec();
    ChassisState current = BuildChassisState(PoseFromOdom(odom), stamp, odom.header.frame_id + "->" +
                                                                           odom.child_frame_id,
                                             prev_odom_);
    prev_odom_ = current;
    latest_odom_ = current;
}

void LioGuessDiagLogger::ObserveChassisTf(const geometry_msgs::TransformStamped& tf_msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!config_.enable) {
        return;
    }
    const double stamp = tf_msg.header.stamp.toSec();
    ChassisState current = BuildChassisState(PoseFromTf(tf_msg), stamp,
                                             "tf:" + tf_msg.header.frame_id + "->" + tf_msg.child_frame_id, prev_tf_);
    prev_tf_ = current;
    latest_tf_ = current;
}

void LioGuessDiagLogger::WriteTrace(const TraceInput& input) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!config_.enable || !initialized_) {
        return;
    }
    TraceInput filled = input;
    if (filled.scan_seq <= 0) {
        filled.scan_seq = ++trace_seq_;
    } else {
        trace_seq_ = std::max(trace_seq_, filled.scan_seq);
    }
    Row row = BuildRow(filled);
    WriteCsvRow(row);
    AppendRecentRow(row);
}

void LioGuessDiagLogger::Finish() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (finished_) {
        return;
    }
    if (initialized_ && config_.enable && config_.summary_enable) {
        WriteSummary();
    }
    if (csv_.is_open()) {
        csv_.close();
    }
    if (jsonl_.is_open()) {
        jsonl_.close();
    }
    finished_ = true;
}

LioGuessDiagLogger::ChassisState LioGuessDiagLogger::BuildChassisState(const SE3& pose, double stamp,
                                                                       const std::string& source,
                                                                       const ChassisState& prev) const {
    ChassisState state;
    state.valid = PoseFinite(pose) && std::isfinite(stamp) && stamp > 0.0;
    state.stamp = stamp;
    state.source = source;
    state.pose = pose;
    if (state.valid && prev.valid && stamp > prev.stamp) {
        const SE3 delta = prev.pose.inverse() * pose;
        state.delta_valid = true;
        state.delta_x = delta.translation().x();
        state.delta_y = delta.translation().y();
        state.delta_xy = Norm2d(delta.translation());
        state.delta_yaw_deg = RelYawDeg(SE3(), delta);
    }
    return state;
}

LioGuessDiagLogger::Row LioGuessDiagLogger::BuildRow(const TraceInput& input) {
    Row row;
    const double wall_time = NowWall();
    const double ros_time = input.ros_time > 0 ? input.ros_time : NowRos(wall_time);

    const bool lio_valid = input.lio_valid && input.last_lo_valid && input.current_lo_valid &&
                           PoseFinite(input.last_lo_pose) && PoseFinite(input.current_lo_pose);
    const SE3 lio_delta = lio_valid ? input.last_lo_pose.inverse() * input.current_lo_pose : SE3();
    double dt = 0.0;
    const bool have_prev_lio_stamp = input.lio_stamp > 0 && prev_trace_lio_stamp_ > 0;
    if (have_prev_lio_stamp) {
        dt = input.lio_stamp - prev_trace_lio_stamp_;
    }
    if (input.lio_stamp > 0) {
        prev_trace_lio_stamp_ = input.lio_stamp;
    }

    const double lio_delta_x = lio_valid ? lio_delta.translation().x() : 0.0;
    const double lio_delta_y = lio_valid ? lio_delta.translation().y() : 0.0;
    const double lio_delta_z = lio_valid ? lio_delta.translation().z() : 0.0;
    const double lio_delta_xy = lio_valid ? Norm2d(lio_delta.translation()) : 0.0;
    const double lio_delta_yaw_deg = lio_valid ? RelYawDeg(SE3(), lio_delta) : 0.0;

    const double safe_dt = dt > 1e-6 ? dt : 0.0;
    const double trans_warn_th = std::max(config_.min_trans_warn_m, config_.vehicle_max_speed_mps * safe_dt *
                                                                         config_.trans_warn_factor);
    const double trans_error_th = std::max(config_.min_trans_error_m, config_.vehicle_max_speed_mps * safe_dt *
                                                                           config_.trans_error_factor);
    const double yaw_warn_th = std::max(config_.min_yaw_warn_deg, config_.yaw_rate_warn_degps * safe_dt);
    const double yaw_error_th = std::max(config_.min_yaw_error_deg, config_.yaw_rate_error_degps * safe_dt);

    const double lio_speed = safe_dt > 0 ? lio_delta_xy / safe_dt : 0.0;
    const double lio_yaw_rate = safe_dt > 0 ? std::fabs(lio_delta_yaw_deg) / safe_dt : 0.0;
    const double prev_lio_dt = last_lio_dt_;
    const double lio_acc = safe_dt > 0 && prev_lio_dt > 0 ? (lio_speed - last_lio_speed_) / safe_dt : 0.0;
    const double lio_yaw_acc = safe_dt > 0 && prev_lio_dt > 0 ? (lio_yaw_rate - last_lio_yaw_rate_) / safe_dt : 0.0;
    last_lio_speed_ = lio_speed;
    last_lio_yaw_rate_ = lio_yaw_rate;
    if (safe_dt > 0) {
        last_lio_dt_ = safe_dt;
    }

    std::string motion_status = "OK";
    if (!lio_valid) {
        motion_status = "INVALID";
    } else if (have_prev_lio_stamp && safe_dt <= 0.0) {
        motion_status = "NON_MONOTONIC";
    } else if (prev_lio_dt > 0 && safe_dt > std::max(0.5, 3.0 * prev_lio_dt)) {
        motion_status = "TIME_GAP";
    } else if (lio_delta_xy > trans_error_th) {
        motion_status = "TRANS_ERROR";
    } else if (std::fabs(lio_delta_yaw_deg) > yaw_error_th) {
        motion_status = "YAW_ERROR";
    } else if (lio_delta_xy > trans_warn_th) {
        motion_status = "TRANS_WARN";
    } else if (std::fabs(lio_delta_yaw_deg) > yaw_warn_th) {
        motion_status = "YAW_WARN";
    }

    ChassisState chassis;
    if (latest_odom_.valid) {
        chassis = latest_odom_;
    } else if (latest_tf_.valid) {
        chassis = latest_tf_;
    }
    const bool chassis_valid = config_.compare_with_chassis_odom && chassis.valid && chassis.delta_valid &&
                               std::fabs(chassis.stamp - input.lio_stamp) < std::max(1.0, 3.0 * safe_dt);
    const double lio_vs_odom_xy =
        chassis_valid ? std::hypot(lio_delta_x - chassis.delta_x, lio_delta_y - chassis.delta_y) : 0.0;
    const double lio_vs_odom_yaw = chassis_valid ? std::fabs(lio_delta_yaw_deg - chassis.delta_yaw_deg) : 0.0;
    std::string odom_status = "OK";
    if (!config_.compare_with_chassis_odom || !chassis_valid) {
        odom_status = "ODOM_MISSING";
    } else if (lio_vs_odom_xy > config_.lio_odom_delta_error_m ||
               lio_vs_odom_yaw > config_.lio_odom_yaw_error_deg) {
        odom_status = "LIO_ODOM_ERROR";
    } else if (lio_vs_odom_xy > config_.lio_odom_delta_warn_m ||
               lio_vs_odom_yaw > config_.lio_odom_yaw_warn_deg) {
        odom_status = "LIO_ODOM_WARN";
    }

    const double last_abs_to_guess_xy = input.last_abs_valid && input.guess_valid
                                            ? Norm2d((input.last_abs_pose.inverse() * input.guess_from_lo).translation())
                                            : 0.0;
    const double last_abs_to_guess_yaw = input.last_abs_valid && input.guess_valid
                                             ? RelYawDeg(input.last_abs_pose, input.guess_from_lo)
                                             : 0.0;
    const double guess_to_ndt_xy = input.guess_valid && input.match.ndt_valid
                                       ? Norm2d((input.guess_from_lo.inverse() * input.match.ndt_pose).translation())
                                       : 0.0;
    const double guess_to_ndt_yaw = input.guess_valid && input.match.ndt_valid
                                        ? RelYawDeg(input.guess_from_lo, input.match.ndt_pose)
                                        : 0.0;
    const double ndt_to_icp_xy = input.match.ndt_valid && input.match.icp_valid
                                     ? Norm2d((input.match.ndt_pose.inverse() * input.match.icp_pose).translation())
                                     : 0.0;
    const double ndt_to_icp_yaw = input.match.ndt_valid && input.match.icp_valid
                                      ? RelYawDeg(input.match.ndt_pose, input.match.icp_pose)
                                      : 0.0;
    const double guess_to_final_xy = input.guess_valid && input.final_pose_valid
                                         ? Norm2d((input.guess_from_lo.inverse() * input.final_pose).translation())
                                         : 0.0;
    const double guess_to_final_yaw = input.guess_valid && input.final_pose_valid
                                          ? RelYawDeg(input.guess_from_lo, input.final_pose)
                                          : 0.0;
    const double ndt_to_final_xy = input.match.ndt_valid && input.final_pose_valid
                                       ? Norm2d((input.match.ndt_pose.inverse() * input.final_pose).translation())
                                       : 0.0;
    const double ndt_to_final_yaw = input.match.ndt_valid && input.final_pose_valid
                                        ? RelYawDeg(input.match.ndt_pose, input.final_pose)
                                        : 0.0;

    std::string diagnosis = "OK";
    std::string reason = "ok";
    const bool lio_warn = motion_status == "TRANS_WARN" || motion_status == "YAW_WARN";
    const bool lio_error = motion_status == "TRANS_ERROR" || motion_status == "YAW_ERROR";
    const bool input_gap = motion_status == "TIME_GAP" || motion_status == "NON_MONOTONIC";
    const bool guess_ndt_large = guess_to_ndt_xy > config_.guess_to_ndt_warn_m ||
                                 std::fabs(guess_to_ndt_yaw) > config_.guess_to_ndt_yaw_warn_deg;
    const bool final_near_ndt = ndt_to_final_xy <= config_.guess_to_ndt_warn_m &&
                                std::fabs(ndt_to_final_yaw) <= config_.guess_to_ndt_yaw_warn_deg;
    const bool final_near_guess = guess_to_final_xy <= config_.guess_to_ndt_warn_m &&
                                  std::fabs(guess_to_final_yaw) <= config_.guess_to_ndt_yaw_warn_deg;

    if (input_gap) {
        diagnosis = "INPUT_GAP";
        reason = "lio timestamp gap or non-monotonic dt";
    } else if (odom_status == "LIO_ODOM_ERROR") {
        diagnosis = "LIO_ODOM_DIVERGED";
        reason = "lio delta diverges from chassis odom delta";
    } else if ((lio_warn || lio_error) && guess_ndt_large && final_near_ndt) {
        diagnosis = "GUESS_BIASED_NDT_RECOVERED";
        reason = "lio guess differs from ndt but final pose follows ndt";
    } else if ((lio_warn || lio_error) && !guess_ndt_large && final_near_guess) {
        diagnosis = "GUESS_BIASED_NDT_ACCEPTED";
        reason = "lio guess likely accepted by ndt/final pose";
    } else if (!lio_warn && !lio_error && odom_status == "OK" && guess_ndt_large && final_near_ndt) {
        diagnosis = "NDT_SELF_JUMP";
        reason = "lio and odom are smooth but ndt output jumps away from guess";
    } else if (lio_error) {
        diagnosis = "LIO_JUMP";
        reason = "lio delta exceeds vehicle motion error threshold";
    } else if (lio_warn) {
        diagnosis = "LIO_SUSPECT";
        reason = "lio delta exceeds vehicle motion warning threshold";
    } else if (!input.match.ndt_valid || !input.final_pose_valid) {
        diagnosis = "UNKNOWN_NEED_MORE_TRACE";
        reason = "missing ndt or final pose";
    }

    std::string severity = "INFO";
    if (diagnosis == "LIO_JUMP" || diagnosis == "LIO_ODOM_DIVERGED" ||
        diagnosis == "GUESS_BIASED_NDT_ACCEPTED" || diagnosis == "NDT_SELF_JUMP" || diagnosis == "INPUT_GAP") {
        severity = "ERROR";
    } else if (diagnosis == "LIO_SUSPECT" || diagnosis == "GUESS_BIASED_NDT_RECOVERED" ||
               odom_status == "LIO_ODOM_WARN") {
        severity = "WARN";
    }

    total_frames_++;
    if (lio_valid) {
        valid_lio_frames_++;
    }
    if (safe_dt > 0) {
        lio_dts_.push_back(safe_dt);
    }
    lio_delta_xys_.push_back(lio_delta_xy);
    lio_yaw_rates_.push_back(lio_yaw_rate);
    last_abs_to_guess_xys_.push_back(last_abs_to_guess_xy);
    if (input.match.ndt_valid) {
        guess_to_ndt_xys_.push_back(guess_to_ndt_xy);
    }
    if (input.final_pose_valid) {
        guess_to_final_xys_.push_back(guess_to_final_xy);
    }
    if (diagnosis == "LIO_SUSPECT") {
        lio_suspect_count_++;
    } else if (diagnosis == "LIO_JUMP") {
        lio_jump_count_++;
        if (first_lio_jump_time_ == 0.0) first_lio_jump_time_ = ros_time;
    } else if (diagnosis == "LIO_ODOM_DIVERGED") {
        lio_odom_diverged_count_++;
        if (first_lio_odom_diverged_time_ == 0.0) first_lio_odom_diverged_time_ = ros_time;
    } else if (diagnosis == "INPUT_GAP") {
        input_gap_count_++;
        if (first_input_gap_time_ == 0.0) first_input_gap_time_ = ros_time;
    } else if (diagnosis == "GUESS_BIASED_NDT_RECOVERED") {
        guess_recovered_count_++;
    } else if (diagnosis == "GUESS_BIASED_NDT_ACCEPTED") {
        guess_accepted_count_++;
    } else if (diagnosis == "NDT_SELF_JUMP") {
        ndt_self_jump_count_++;
        if (first_ndt_self_jump_time_ == 0.0) first_ndt_self_jump_time_ = ros_time;
    }

    const std::string extra_json =
        "{\"points\":" + std::to_string(input.point_count) + ",\"lo_reliable\":" +
        std::string(input.lo_reliable ? "true" : "false") + ",\"match_fail_count\":" +
        std::to_string(input.match_fail_count) + "}";

    auto add = [&row](const auto& value) {
        std::ostringstream oss;
        oss << value;
        row.cells.push_back(oss.str());
    };
    auto add_double = [&row](double value) { row.cells.push_back(FormatDouble(value)); };
    auto add_bool = [&row](bool value) { row.cells.push_back(value ? "true" : "false"); };
    auto add_pose = [&](const SE3& pose, bool valid) {
        add_double(valid ? pose.translation().x() : 0.0);
        add_double(valid ? pose.translation().y() : 0.0);
        add_double(valid ? pose.translation().z() : 0.0);
    };
    auto add_pose_yaw = [&](const SE3& pose, bool valid) { add_double(valid ? YawDeg(pose) : 0.0); };

    add(run_id_);
    add_double(wall_time);
    add_double(ros_time);
    add("map");
    add(input.scan_seq);
    add_bool(lio_valid);
    add_double(input.lio_stamp);
    add_double(safe_dt);
    add_pose(input.current_lo_pose, lio_valid);
    add_double(0.0);
    add_double(0.0);
    add_pose_yaw(input.current_lo_pose, lio_valid);
    add_double(lio_delta_x);
    add_double(lio_delta_y);
    add_double(lio_delta_z);
    add_double(lio_delta_xy);
    add_double(lio_delta_yaw_deg);
    add_double(lio_speed);
    add_double(lio_yaw_rate);
    add_double(lio_acc);
    add_double(lio_yaw_acc);
    add_double(trans_warn_th);
    add_double(trans_error_th);
    add_double(yaw_warn_th);
    add_double(yaw_error_th);
    add(motion_status);
    add_pose(input.last_abs_pose, input.last_abs_valid);
    add_pose_yaw(input.last_abs_pose, input.last_abs_valid);
    add_pose(input.guess_from_lo, input.guess_valid);
    add_pose_yaw(input.guess_from_lo, input.guess_valid);
    add_double(last_abs_to_guess_xy);
    add_double(last_abs_to_guess_yaw);
    add_bool(chassis_valid);
    add_double(chassis_valid ? chassis.stamp : 0.0);
    add(chassis_valid ? chassis.source : "");
    add_double(chassis_valid ? chassis.delta_xy : 0.0);
    add_double(chassis_valid ? chassis.delta_yaw_deg : 0.0);
    add_double(lio_vs_odom_xy);
    add_double(lio_vs_odom_yaw);
    add(odom_status);
    add_bool(input.match.ndt_valid);
    add_bool(input.match.ndt_converged);
    add_double(input.match.ndt_confidence);
    add_double(input.match.ndt_score);
    add_pose(input.match.ndt_pose, input.match.ndt_valid);
    add_pose_yaw(input.match.ndt_pose, input.match.ndt_valid);
    add_double(guess_to_ndt_xy);
    add_double(guess_to_ndt_yaw);
    add_bool(input.match.icp_valid);
    add_bool(input.match.icp_converged);
    add_bool(input.match.icp_accepted);
    add_pose(input.match.icp_pose, input.match.icp_valid);
    add_pose_yaw(input.match.icp_pose, input.match.icp_valid);
    add_double(ndt_to_icp_xy);
    add_double(ndt_to_icp_yaw);
    add_bool(input.final_pose_valid);
    add_pose(input.final_pose, input.final_pose_valid);
    add_pose_yaw(input.final_pose, input.final_pose_valid);
    add_double(guess_to_final_xy);
    add_double(guess_to_final_yaw);
    add_double(ndt_to_final_xy);
    add_double(ndt_to_final_yaw);
    add_bool(input.loc_success);
    add_bool(input.loc_inited);
    add(input.loc_stage);
    add(input.reject_reason);
    add(diagnosis);
    add(reason);
    add(extra_json);

    row.ros_time = ros_time;
    row.diagnosis_status = diagnosis;
    row.severity = severity;
    WriteEventIfNeeded(row, input, lio_delta_xy, trans_error_th, lio_delta_yaw_deg, yaw_error_th, guess_to_ndt_xy,
                       input.match.ndt_confidence);
    return row;
}

void LioGuessDiagLogger::WriteCsvRow(const Row& row) {
    if (!csv_.is_open()) {
        return;
    }
    if (config_.log_every_n > 1 && total_frames_ % config_.log_every_n != 0 && row.severity == "INFO") {
        return;
    }
    for (std::size_t i = 0; i < row.cells.size(); ++i) {
        if (i) {
            csv_ << ",";
        }
        csv_ << CsvEscape(row.cells[i]);
    }
    csv_ << "\n";
    csv_rows_since_flush_++;
    if (csv_rows_since_flush_ >= config_.flush_every_n) {
        csv_.flush();
        csv_rows_since_flush_ = 0;
    }
}

void LioGuessDiagLogger::WriteEventIfNeeded(const Row& row, const TraceInput& input, double lio_delta_xy,
                                            double trans_error_th, double lio_delta_yaw_deg, double yaw_error_th,
                                            double guess_to_ndt_xy, double ndt_confidence) {
    const std::string motion_status = row.cells.size() > 27 ? row.cells[27] : "";
    const std::string odom_status = row.cells.size() > 45 ? row.cells[45] : "";
    const bool status_changed = row.diagnosis_status != last_diagnosis_status_;
    const bool abnormal = row.severity == "WARN" || row.severity == "ERROR";
    if (!jsonl_.is_open() || (!status_changed && !abnormal)) {
        last_diagnosis_status_ = row.diagnosis_status;
        return;
    }
    std::string event_status = row.diagnosis_status;
    std::string event_reason = row.cells.size() > 79 ? row.cells[79] : "";
    if (event_status == "OK" && odom_status == "LIO_ODOM_WARN") {
        event_status = "LIO_ODOM_DIVERGED";
        event_reason = "lio delta diverges from chassis odom warning threshold";
    }
    const int event_id = EventIdFor(event_status, motion_status, odom_status);
    jsonl_ << "{\"run_id\":\"" << JsonEscape(run_id_) << "\",\"event_time_ros\":" << FormatDouble(row.ros_time)
           << ",\"event_time_wall\":" << FormatDouble(NowWall())
           << ",\"event_id\":" << event_id
           << ",\"module\":\"lio_guess_diag\",\"status\":\"" << JsonEscape(event_status)
           << "\",\"severity\":\"" << JsonEscape(row.severity) << "\",\"reason\":\""
           << JsonEscape(event_reason) << "\",\"lio_dt_sec\":"
           << FormatDouble(row.cells.size() > 7 ? std::stod(row.cells[7]) : 0.0)
           << ",\"vehicle_max_speed_mps\":" << FormatDouble(config_.vehicle_max_speed_mps)
           << ",\"lio_delta_xy\":" << FormatDouble(lio_delta_xy)
           << ",\"trans_error_th\":" << FormatDouble(trans_error_th)
           << ",\"lio_delta_yaw_deg\":" << FormatDouble(lio_delta_yaw_deg)
           << ",\"yaw_error_th_deg\":" << FormatDouble(yaw_error_th)
           << ",\"guess_to_ndt_xy\":" << FormatDouble(guess_to_ndt_xy)
           << ",\"ndt_confidence\":" << FormatDouble(ndt_confidence)
           << ",\"loc_success\":" << (input.loc_success ? "true" : "false")
           << ",\"suggestion\":\"check LIO input health, IMU-LiDAR time sync, deskew, extrinsic, and compare with chassis odom\"}\n";
    jsonl_.flush();
    last_diagnosis_status_ = row.diagnosis_status;
    last_event_ros_time_ = row.ros_time;
    if (config_.write_fault_window_on_event &&
        (row.diagnosis_status == "LIO_JUMP" || row.diagnosis_status == "LIO_ODOM_DIVERGED" ||
         row.diagnosis_status == "GUESS_BIASED_NDT_ACCEPTED" || row.diagnosis_status == "NDT_SELF_JUMP")) {
        MaybeWriteFaultWindow(row);
    }
}

void LioGuessDiagLogger::WriteSummary() {
    const std::string path = config_.output_dir + "/lio_guess_summary_" + run_id_ + ".md";
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        LOG(WARNING) << "failed to open lio guess summary: " << path;
        return;
    }
    const double end_wall = NowWall();
    out << "# LIO Guess Diagnostics Summary\n\n";
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
    out << "- guess_to_ndt_warn_m: `" << FormatDouble(config_.guess_to_ndt_warn_m) << "`\n";
    out << "- guess_to_ndt_error_m: `" << FormatDouble(config_.guess_to_ndt_error_m) << "`\n\n";
    out << "## LIO Statistics\n\n";
    out << "| metric | value |\n|---|---:|\n";
    out << "| total_frames | " << total_frames_ << " |\n";
    out << "| valid_lio_frames | " << valid_lio_frames_ << " |\n";
    out << "| mean_lio_hz | " << FormatDouble(MeanOrZero(lio_dts_) > 0 ? 1.0 / MeanOrZero(lio_dts_) : 0.0) << " |\n";
    out << "| p95_lio_dt | " << FormatDouble(Percentile(lio_dts_, 0.95)) << " |\n";
    out << "| max_lio_dt | " << FormatDouble(MaxOrZero(lio_dts_)) << " |\n";
    out << "| mean_lio_delta_xy | " << FormatDouble(MeanOrZero(lio_delta_xys_)) << " |\n";
    out << "| p95_lio_delta_xy | " << FormatDouble(Percentile(lio_delta_xys_, 0.95)) << " |\n";
    out << "| max_lio_delta_xy | " << FormatDouble(MaxOrZero(lio_delta_xys_)) << " |\n";
    out << "| mean_lio_yaw_rate | " << FormatDouble(MeanOrZero(lio_yaw_rates_)) << " |\n";
    out << "| p95_lio_yaw_rate | " << FormatDouble(Percentile(lio_yaw_rates_, 0.95)) << " |\n";
    out << "| max_lio_yaw_rate | " << FormatDouble(MaxOrZero(lio_yaw_rates_)) << " |\n\n";
    out << "## Diagnosis Counts\n\n";
    out << "- LIO_SUSPECT count: " << lio_suspect_count_ << "\n";
    out << "- LIO_JUMP count: " << lio_jump_count_ << "\n";
    out << "- LIO_ODOM_DIVERGED count: " << lio_odom_diverged_count_ << "\n";
    out << "- INPUT_GAP count: " << input_gap_count_ << "\n";
    out << "- GUESS_BIASED_NDT_RECOVERED count: " << guess_recovered_count_ << "\n";
    out << "- GUESS_BIASED_NDT_ACCEPTED count: " << guess_accepted_count_ << "\n";
    out << "- NDT_SELF_JUMP count: " << ndt_self_jump_count_ << "\n\n";
    out << "## Guess Statistics\n\n";
    out << "| metric | mean | p95 | max |\n|---|---:|---:|---:|\n";
    out << "| last_abs_to_guess_xy | " << FormatDouble(MeanOrZero(last_abs_to_guess_xys_)) << " | "
        << FormatDouble(Percentile(last_abs_to_guess_xys_, 0.95)) << " | "
        << FormatDouble(MaxOrZero(last_abs_to_guess_xys_)) << " |\n";
    out << "| guess_to_ndt_xy | " << FormatDouble(MeanOrZero(guess_to_ndt_xys_)) << " | "
        << FormatDouble(Percentile(guess_to_ndt_xys_, 0.95)) << " | "
        << FormatDouble(MaxOrZero(guess_to_ndt_xys_)) << " |\n";
    out << "| guess_to_final_xy | " << FormatDouble(MeanOrZero(guess_to_final_xys_)) << " | "
        << FormatDouble(Percentile(guess_to_final_xys_, 0.95)) << " | "
        << FormatDouble(MaxOrZero(guess_to_final_xys_)) << " |\n\n";
    out << "## Root Cause Guess\n\n";
    if (first_input_gap_time_ > 0.0) {
        out << "- 优先怀疑传感器断流、时间戳异常或 rosbag 回放时间问题。\n";
    } else if (first_lio_jump_time_ > 0.0 &&
               (first_ndt_self_jump_time_ == 0.0 || first_lio_jump_time_ <= first_ndt_self_jump_time_)) {
        out << "- 优先怀疑 LIO 里程计跳变导致 guess_from_lo 带偏。\n";
    } else if (ndt_self_jump_count_ > 0 && lio_jump_count_ == 0) {
        out << "- 优先怀疑 NDT 在重复结构中发生错误匹配。\n";
    } else if (lio_odom_diverged_count_ > 0) {
        out << "- 优先怀疑 LIO 或底盘 odom 至少一方存在异常，需要结合 odom_base_trace 判断。\n";
    } else {
        out << "- 未观察到明确 LIO/guess_from_lo 根因。\n";
    }
    out << "\n上述根因初判需要结合 input_health、odom_base_trace、lidar_loc_match_trace、map_odom_trace 和实车现象进一步确认。\n";
    out << "\n需要运行日志/测试结果进一步确认。\n";
}

void LioGuessDiagLogger::MaybeWriteFaultWindow(const Row& row) {
    const std::string path = config_.output_dir + "/fault_windows/fault_lio_guess_" + FormatDouble(row.ros_time) +
                             "_" + std::to_string(++event_seq_) + ".csv";
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        return;
    }
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

void LioGuessDiagLogger::AppendRecentRow(const Row& row) {
    recent_rows_.push_back(row);
    const double keep_from = row.ros_time - std::max(config_.keep_recent_window_sec, config_.fault_window_pre_sec);
    while (!recent_rows_.empty() && recent_rows_.front().ros_time < keep_from) {
        recent_rows_.pop_front();
    }
    AppendPendingFaultWindows(row);
}

void LioGuessDiagLogger::AppendPendingFaultWindows(const Row& row) {
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

int LioGuessDiagLogger::EventIdFor(const std::string& status, const std::string& motion_status,
                                   const std::string& odom_status) const {
    if (status == "INPUT_GAP") return 2041;
    if (status == "LIO_ODOM_DIVERGED") return odom_status == "LIO_ODOM_WARN" ? 2011 : 2012;
    if (status == "GUESS_BIASED_NDT_RECOVERED") return 2021;
    if (status == "GUESS_BIASED_NDT_ACCEPTED") return 2022;
    if (status == "NDT_SELF_JUMP") return 2031;
    if (motion_status == "TRANS_WARN") return 2001;
    if (motion_status == "TRANS_ERROR") return 2002;
    if (motion_status == "YAW_WARN") return 2003;
    if (motion_status == "YAW_ERROR") return 2004;
    return 2099;
}

std::string LioGuessDiagLogger::MakeRunId(const std::string& run_label) {
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

double LioGuessDiagLogger::NowWall() { return ros::WallTime::now().toSec(); }

double LioGuessDiagLogger::NowRos(double fallback) {
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

std::string LioGuessDiagLogger::JsonEscape(const std::string& text) {
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

std::string LioGuessDiagLogger::CsvEscape(const std::string& text) {
    if (text.find_first_of(",\"\n\r") == std::string::npos) {
        return text;
    }
    std::string escaped = "\"";
    for (char c : text) {
        if (c == '"') escaped += "\"\"";
        else escaped += c;
    }
    escaped += "\"";
    return escaped;
}

std::string LioGuessDiagLogger::FormatDouble(double value) {
    if (!std::isfinite(value)) {
        return "nan";
    }
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << value;
    return oss.str();
}

double LioGuessDiagLogger::Percentile(std::vector<double> values, double ratio) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const std::size_t idx = std::min(values.size() - 1, static_cast<std::size_t>(std::floor(ratio * values.size())));
    return values[idx];
}

double LioGuessDiagLogger::YawDeg(const SE3& pose) {
    const auto R = pose.rotationMatrix();
    return std::atan2(R(1, 0), R(0, 0)) * kRadToDeg;
}

double LioGuessDiagLogger::RelYawDeg(const SE3& from, const SE3& to) {
    const SE3 delta = from.inverse() * to;
    return delta.so3().log().z() * kRadToDeg;
}

double LioGuessDiagLogger::Norm2d(const Vec3d& v) { return std::hypot(v.x(), v.y()); }

bool LioGuessDiagLogger::PoseFinite(const SE3& pose) {
    return pose.translation().allFinite() && pose.unit_quaternion().coeffs().allFinite();
}

SE3 LioGuessDiagLogger::PoseFromOdom(const nav_msgs::Odometry& odom) {
    const auto& p = odom.pose.pose.position;
    const auto& q = odom.pose.pose.orientation;
    return SE3(Quatd(q.w, q.x, q.y, q.z), Vec3d(p.x, p.y, p.z));
}

SE3 LioGuessDiagLogger::PoseFromTf(const geometry_msgs::TransformStamped& tf_msg) {
    const auto& p = tf_msg.transform.translation;
    const auto& q = tf_msg.transform.rotation;
    return SE3(Quatd(q.w, q.x, q.y, q.z), Vec3d(p.x, p.y, p.z));
}

}  // namespace lightning
