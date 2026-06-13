//
// Created by xiang on 25-9-12.
//

#include "core/system/loc_system.h"
#include "core/localization/localization.h"
#include "io/yaml_io.h"
#include "utils/evaluation_writer.h"
#include "utils/input_health_logger.h"
#include "wrapper/ros_utils.h"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <csignal>
#include <tf2/exceptions.h>

namespace lightning {
namespace {

std::string GetYamlStringOr(const YAML::Node& node, const std::string& key, const std::string& fallback) {
    if (node && node[key]) {
        return node[key].as<std::string>();
    }
    return fallback;
}

}  // namespace

LocSystem::LocSystem(LocSystem::Options options) : options_(options) {
    /// handle ctrl-c
    std::signal(SIGINT, lightning::debug::SigHandle);
}

LocSystem::~LocSystem() { loc_->Finish(); }

bool LocSystem::Init(const std::string &yaml_path) {
    loc::Localization::Options opt;
    opt.online_mode_ = true;
    loc_ = std::make_shared<loc::Localization>(opt);

    YAML_IO yaml(yaml_path);
    auto yaml_node = YAML::LoadFile(yaml_path);
    map_frame_ = GetYamlStringOr(yaml_node["frames"], "map", "map");
    odom_frame_ = GetYamlStringOr(yaml_node["frames"], "odom", "odom");
    base_frame_ = GetYamlStringOr(yaml_node["frames"], "base_link", "base_link");
    livox_frame_ = GetYamlStringOr(yaml_node["frames"], "livox_frame", "livox_frame");
    path_msg_.header.frame_id = map_frame_;

    ROS_WARN_STREAM("TF policy: localization publishes " << map_frame_ << "->" << base_frame_
                    << " when pub_tf is true. Check external publishers for duplicate "
                    << "map->base_link, map->odom, odom->base_link before enabling other TF sources. "
                    << "Configured odom=" << odom_frame_ << ", livox_frame=" << livox_frame_);

    std::string map_path = yaml.GetValue<std::string>("system", "map_path");
    const bool evaluation_enabled =
        yaml_node["system"]["evaluation"] && yaml_node["system"]["evaluation"].as<bool>();
    if (evaluation_enabled && yaml_node["system"]["evaluation_output_dir"]) {
        const std::string evaluation_output_dir = yaml_node["system"]["evaluation_output_dir"].as<std::string>();
        if (!evaluation_output_dir.empty()) {
            evaluation_writer_ = std::make_shared<EvaluationWriter>();
            evaluation_writer_->Init(evaluation_output_dir, "loc");
            evaluation_writer_->WriteMapQualityInput(map_path + "/global.pcd", map_path + "/map.pgm");
            loc_->SetEvaluationCallback([this](const loc::LocalizationResult& result) {
                evaluation_writer_->WriteLocalizationResult(result);
            });
        }
    }

    LOG(INFO) << "online mode, creating ROS1 node ... ";

    /// subscribers
    node_ = std::make_unique<ros::NodeHandle>();

    imu_topic_ = yaml.GetValue<std::string>("common", "imu_topic");
    cloud_topic_ = yaml.GetValue<std::string>("common", "lidar_topic");
    livox_topic_ = yaml.GetValue<std::string>("common", "livox_lidar_topic");
    input_odom_topic_ = GetYamlStringOr(yaml_node["health_check"]["odom"], "topic", "/odom");

    input_health_logger_ = std::make_shared<InputHealthLogger>();
    input_health_logger_->Init(yaml_path, "loc_online");
    if (input_health_logger_->Enabled()) {
        input_health_logger_->RegisterImu(imu_topic_);
        input_health_logger_->RegisterLidar(cloud_topic_);
        input_health_logger_->RegisterLidar(livox_topic_);
        input_health_logger_->RegisterOdom(input_odom_topic_, !input_odom_topic_.empty());
        input_health_logger_->RegisterTf(odom_frame_, base_frame_, true);
    }

    imu_sub_ = node_->subscribe<sensor_msgs::Imu>(
        imu_topic_, 2000, [this](const sensor_msgs::ImuConstPtr& msg) {
            if (input_health_logger_) {
                input_health_logger_->ObserveImu(imu_topic_, *msg);
            }
            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

            ProcessIMU(imu);
        });

    cloud_sub_ = node_->subscribe<sensor_msgs::PointCloud2>(
        cloud_topic_, 100, [this](const sensor_msgs::PointCloud2ConstPtr& cloud) {
            if (input_health_logger_) {
                input_health_logger_->ObservePointCloud2(cloud_topic_, *cloud);
            }
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    livox_sub_ = node_->subscribe<livox_ros_driver2::CustomMsg>(
        livox_topic_, 100, [this](const livox_ros_driver2::CustomMsgConstPtr& cloud) {
            if (input_health_logger_) {
                input_health_logger_->ObserveLivox(livox_topic_, *cloud);
            }
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    if (!input_odom_topic_.empty()) {
        input_odom_sub_ = node_->subscribe<nav_msgs::Odometry>(
            input_odom_topic_, 200, [this](const nav_msgs::OdometryConstPtr& msg) {
                if (input_health_logger_) {
                    input_health_logger_->ObserveOdom(input_odom_topic_, *msg);
                }
                if (loc_) {
                    loc_->ObserveChassisOdom(input_odom_topic_, *msg);
                }
            });
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    input_health_timer_ = node_->createTimer(ros::Duration(std::max(0.1, input_health_logger_->CheckPeriodSec())),
                                             &LocSystem::CheckInputTfHealth, this);

    odom_pub_ = node_->advertise<nav_msgs::Odometry>("lightning/loc/odom", 50);
    path_pub_ = node_->advertise<nav_msgs::Path>("lightning/loc/path", 5, true);
    marker_pub_ = node_->advertise<visualization_msgs::Marker>("lightning/loc/pose_marker", 5);

    if (options_.pub_tf_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
        loc_->SetTFCallback([this](const geometry_msgs::TransformStamped &pose) {
            tf_broadcaster_->sendTransform(pose);
            PublishLocalizationOutputs(pose);
        });
    } else {
        loc_->SetTFCallback([this](const geometry_msgs::TransformStamped &pose) { PublishLocalizationOutputs(pose); });
    }

    bool ret = loc_->Init(yaml_path, map_path);
    if (ret) {
        LOG(INFO) << "online loc node has been created.";
    }

    return ret;
}

void LocSystem::SetInitPose(const SE3 &pose) {
    LOG(INFO) << "set init pose: " << pose.translation().transpose() << ", "
              << pose.unit_quaternion().coeffs().transpose();

    loc_->SetExternalPose(pose.unit_quaternion(), pose.translation());
    loc_started_ = true;
}

void LocSystem::ProcessIMU(const IMUPtr &imu) {
    if (loc_started_) {
        loc_->ProcessIMUMsg(imu);
    }
}

void LocSystem::ProcessLidar(const sensor_msgs::PointCloud2ConstPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLidarMsg(cloud);
    }
}

void LocSystem::ProcessLidar(const livox_ros_driver2::CustomMsgConstPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLivoxLidarMsg(cloud);
    }
}

void LocSystem::Spin() {
    if (node_ != nullptr) {
        ros::spin();
    }
}

nav_msgs::Odometry LocSystem::MakeOdometryMsg(const geometry_msgs::TransformStamped& pose_msg) const {
    nav_msgs::Odometry odom;
    odom.header = pose_msg.header;
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pose_msg.transform.translation.x;
    odom.pose.pose.position.y = pose_msg.transform.translation.y;
    odom.pose.pose.position.z = pose_msg.transform.translation.z;
    odom.pose.pose.orientation = pose_msg.transform.rotation;
    return odom;
}

visualization_msgs::Marker LocSystem::MakePoseMarker(const geometry_msgs::TransformStamped& pose_msg) const {
    visualization_msgs::Marker marker;
    marker.header = pose_msg.header;
    marker.header.frame_id = map_frame_;
    marker.ns = "lightning_loc_pose";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.8;
    marker.scale.y = 0.12;
    marker.scale.z = 0.12;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.9;
    marker.color.b = 0.3;
    marker.pose.position.x = pose_msg.transform.translation.x;
    marker.pose.position.y = pose_msg.transform.translation.y;
    marker.pose.position.z = pose_msg.transform.translation.z;
    marker.pose.orientation = pose_msg.transform.rotation;
    return marker;
}

void LocSystem::PublishLocalizationOutputs(const geometry_msgs::TransformStamped& pose_msg) {
    const auto odom = MakeOdometryMsg(pose_msg);
    odom_pub_.publish(odom);

    geometry_msgs::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    path_msg_.header.stamp = odom.header.stamp;
    path_msg_.poses.emplace_back(pose);
    path_pub_.publish(path_msg_);
    marker_pub_.publish(MakePoseMarker(pose_msg));
}

void LocSystem::CheckInputTfHealth(const ros::TimerEvent& event) {
    (void)event;
    if (!input_health_logger_ && !loc_) {
        return;
    }
    if (input_health_logger_ && input_health_logger_->Enabled()) {
        input_health_logger_->Tick();
    }
    InputHealthLogger::TfRecord record;
    record.parent_frame = odom_frame_;
    record.child_frame = base_frame_;
    record.query_stamp = ros::Time::now().toSec();
    record.lookup_timeout_sec =
        input_health_logger_ ? input_health_logger_->TfLookupTimeoutSec() : 0.02;
    geometry_msgs::TransformStamped tf_msg;
    try {
        tf_msg = tf_buffer_->lookupTransform(odom_frame_, base_frame_, ros::Time(0),
                                             ros::Duration(record.lookup_timeout_sec));
        record.lookup_ok = true;
        record.latest_tf_stamp = tf_msg.header.stamp.toSec();
        record.lookup_dt_sec = record.query_stamp - record.latest_tf_stamp;
        if (loc_) {
            loc_->ObserveChassisTfQuery(odom_frame_, base_frame_, record.query_stamp, record.lookup_timeout_sec, true,
                                        tf_msg);
        }
    } catch (const tf2::ExtrapolationException& e) {
        record.lookup_ok = false;
        const std::string what = e.what();
        record.error_type =
            what.find("future") != std::string::npos ? "TF_EXTRAPOLATION_FUTURE" : "TF_EXTRAPOLATION_PAST";
        record.error_string = what;
    } catch (const tf2::ConnectivityException& e) {
        record.lookup_ok = false;
        record.error_type = "TF_DISCONNECTED";
        record.error_string = e.what();
    } catch (const tf2::LookupException& e) {
        record.lookup_ok = false;
        record.error_type = "TF_DISCONNECTED";
        record.error_string = e.what();
    } catch (const tf2::TimeoutException& e) {
        record.lookup_ok = false;
        record.error_type = "TF_TIMEOUT";
        record.error_string = e.what();
    } catch (const tf2::TransformException& e) {
        record.lookup_ok = false;
        record.error_type = "TF_TIMEOUT";
        record.error_string = e.what();
    }
    if (!record.lookup_ok && loc_) {
        loc_->ObserveChassisTfQuery(odom_frame_, base_frame_, record.query_stamp, record.lookup_timeout_sec, false,
                                    tf_msg, record.error_type, record.error_string);
    }
    if (input_health_logger_ && input_health_logger_->Enabled()) {
        input_health_logger_->ObserveTf(record);
    }
}

}  // namespace lightning
