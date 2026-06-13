//
// Created by xiang on 25-5-6.
//

#include "core/system/slam.h"
#include "core/g2p5/g2p5.h"
#include "core/lio/laser_mapping.h"
#include "core/loop_closing/loop_closing.h"
#include "core/maps/tiled_map.h"
#include "ui/pangolin_window.h"
#include "utils/evaluation_writer.h"
#include "utils/input_health_logger.h"
#include "wrapper/ros_utils.h"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <pcl_conversions/pcl_conversions.h>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <csignal>
#include <limits>
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

SlamSystem::SlamSystem(lightning::SlamSystem::Options options) : options_(options) {
    /// handle ctrl-c
    std::signal(SIGINT, lightning::debug::SigHandle);
}

bool SlamSystem::Init(const std::string& yaml_path) {
    lio_ = std::make_shared<LaserMapping>();
    if (!lio_->Init(yaml_path)) {
        LOG(ERROR) << "failed to init lio module";
        return false;
    }

    auto yaml = YAML::LoadFile(yaml_path);
    options_.with_loop_closing_ = yaml["system"]["with_loop_closing"].as<bool>();
    options_.with_visualization_ = yaml["system"]["with_ui"].as<bool>();
    options_.with_2dvisualization_ = yaml["system"]["with_2dui"].as<bool>();
    options_.with_gridmap_ = yaml["system"]["with_g2p5"].as<bool>();
    options_.step_on_kf_ = yaml["system"]["step_on_kf"].as<bool>();
    map_frame_ = GetYamlStringOr(yaml["frames"], "map", "map");
    odom_frame_ = GetYamlStringOr(yaml["frames"], "odom", "odom");
    base_frame_ = GetYamlStringOr(yaml["frames"], "base_link", "base_link");
    livox_frame_ = GetYamlStringOr(yaml["frames"], "livox_frame", "livox_frame");
    path_msg_.header.frame_id = map_frame_;

    ROS_WARN_STREAM("TF policy: SLAM does not publish TF in this stage. Frames are map=" << map_frame_
                    << ", odom=" << odom_frame_ << ", base_link=" << base_frame_
                    << ", livox_frame=" << livox_frame_
                    << ". Check external publishers for duplicate map->base_link, map->odom, odom->base_link.");

    const bool evaluation_enabled = yaml["system"]["evaluation"] && yaml["system"]["evaluation"].as<bool>();
    if (evaluation_enabled && yaml["system"]["evaluation_output_dir"]) {
        const std::string evaluation_output_dir = yaml["system"]["evaluation_output_dir"].as<std::string>();
        if (!evaluation_output_dir.empty()) {
            evaluation_writer_ = std::make_shared<EvaluationWriter>();
            evaluation_writer_->Init(evaluation_output_dir, "slam");
        }
    }

    if (options_.with_loop_closing_) {
        LOG(INFO) << "slam with loop closing";
        LoopClosing::Options options;
        options.online_mode_ = options_.online_mode_;
        lc_ = std::make_shared<LoopClosing>(options);
        lc_->Init(yaml_path);
        lc_->SetSourceScanAccumProvider([this](double curr_stamp_sec, const SE3& T_w_curr,
                                                LoopClosing::SourceScanAccumProviderResult* out) {
            if (!out) {
                return false;
            }
            *out = LoopClosing::SourceScanAccumProviderResult();
            if (!lio_) {
                out->fallback_reason = "ACCUM_PROVIDER_LIO_UNAVAILABLE";
                return false;
            }
            LaserMapping::SourceScanAccumCloudResult lio_result;
            const bool ok = lio_->BuildSourceScanAccumCloud(curr_stamp_sec, T_w_curr, &lio_result);
            out->success = lio_result.success;
            out->cloud_lidar = lio_result.cloud_lidar;
            out->scan_count = lio_result.scan_count;
            out->time_span_sec = lio_result.time_span_sec;
            out->points_before_downsample = lio_result.points_before_downsample;
            out->points_after_downsample = lio_result.points_after_downsample;
            out->fallback_reason = lio_result.fallback_reason;
            return ok;
        });
        if (evaluation_writer_ && evaluation_writer_->Enabled()) {
            lc_->SetEvaluationCallback([this](const LoopClosing::EvaluationInfo& info) {
                if (!info.keyframe) {
                    return;
                }
                EvaluationRecord record;
                record.timestamp = info.keyframe->GetState().timestamp_;
                record.pose = info.keyframe->GetOptPose();
                record.keyframe_id = static_cast<long>(info.keyframe->GetID());
                record.loop_candidate_id = static_cast<long>(info.loop_candidate_id);
                record.loop_accepted = info.loop_accepted ? 1 : 0;
                record.loop_score = info.loop_score;
                record.pgo_before_error = info.pgo_before_error;
                record.pgo_after_error = info.pgo_after_error;
                record.loop_status = info.loop_status;
                record.loop_reject_reason = info.loop_reject_reason;
                record.loop_chi2 = info.loop_chi2;
                record.loop_robust_delta = info.loop_robust_delta;
                record.write_trajectory = false;
                evaluation_writer_->WriteRecord(record);
            });
        }
    }

    if (options_.with_visualization_) {
        LOG(INFO) << "slam with 3D UI";
        ui_ = std::make_shared<ui::PangolinWindow>();
        if (ui_->Init()) {
            lio_->SetUI(ui_);
        } else {
            LOG(WARNING) << "slam 3D UI disabled because Pangolin initialization failed";
            ui_.reset();
        }
    }

    if (options_.with_gridmap_) {
        g2p5::G2P5::Options opt;
        opt.online_mode_ = options_.online_mode_;

        g2p5_ = std::make_shared<g2p5::G2P5>(opt);
        g2p5_->Init(yaml_path);

        if (options_.with_2dvisualization_) {
            g2p5_->SetMapUpdateCallback([this](g2p5::G2P5MapPtr map) {
                cv::Mat image = map->ToCV();
                cv::imshow("map", image);

                if (options_.step_on_kf_) {
                    cv::waitKey(0);

                } else {
                    cv::waitKey(10);
                }
            });
        }
    }

    if (lc_) {
        lc_->SetLoopClosedCB([this]() {
            if (g2p5_) {
                g2p5_->RedrawGlobalMap();
            }
            if (ui_ && lio_) {
                ui_->Reset(lio_->GetAllKeyframes());
            }
            RewriteOptimizedTrajectory(true);
        });
    }

    if (options_.online_mode_) {
        LOG(INFO) << "online mode, creating ROS1 node ... ";

        /// subscribers
        node_ = std::make_unique<ros::NodeHandle>();

        imu_topic_ = yaml["common"]["imu_topic"].as<std::string>();
        cloud_topic_ = yaml["common"]["lidar_topic"].as<std::string>();
        livox_topic_ = yaml["common"]["livox_lidar_topic"].as<std::string>();
        input_odom_topic_ = GetYamlStringOr(yaml["health_check"]["odom"], "topic", "/odom");
        input_health_logger_ = std::make_shared<InputHealthLogger>();
        input_health_logger_->Init(yaml_path, "slam_online");
        if (input_health_logger_->Enabled()) {
            input_health_logger_->RegisterImu(imu_topic_);
            input_health_logger_->RegisterLidar(cloud_topic_);
            input_health_logger_->RegisterLidar(livox_topic_);
            input_health_logger_->RegisterOdom(input_odom_topic_, !input_odom_topic_.empty());
            input_health_logger_->RegisterTf(odom_frame_, base_frame_, true);
        }
        odom_pub_ = node_->advertise<nav_msgs::Odometry>("lightning/slam/odom", 20);
        path_pub_ = node_->advertise<nav_msgs::Path>("lightning/slam/path", 5, true);
        keyframe_cloud_pub_ = node_->advertise<sensor_msgs::PointCloud2>("lightning/slam/keyframe_cloud", 2);
        marker_pub_ = node_->advertise<visualization_msgs::Marker>("lightning/slam/keyframes", 5);

        imu_sub_ = node_->subscribe<sensor_msgs::Imu>(
            imu_topic_, 2000, [this](const sensor_msgs::ImuConstPtr& msg) {
                if (input_health_logger_) {
                    input_health_logger_->ObserveImu(imu_topic_, *msg);
                }
                IMUPtr imu = std::make_shared<IMU>();
                imu->timestamp = ToSec(msg->header.stamp);
                imu->linear_acceleration =
                    Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                imu->angular_velocity =
                    Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

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
                });
        }

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        input_health_timer_ = node_->createTimer(ros::Duration(std::max(0.1, input_health_logger_->CheckPeriodSec())),
                                                 &SlamSystem::CheckInputTfHealth, this);

        savemap_service_ = node_->advertiseService(
            "lightning/save_map",
            static_cast<bool (SlamSystem::*)(SaveMapService::Request&, SaveMapService::Response&)>(
                &SlamSystem::SaveMap),
            this);

        LOG(INFO) << "online slam node has been created.";
    }

    return true;
}

void SlamSystem::CheckInputTfHealth(const ros::TimerEvent& event) {
    (void)event;
    if (!input_health_logger_ || !input_health_logger_->Enabled()) {
        return;
    }
    input_health_logger_->Tick();
    InputHealthLogger::TfRecord record;
    record.parent_frame = odom_frame_;
    record.child_frame = base_frame_;
    record.query_stamp = ros::Time::now().toSec();
    record.lookup_timeout_sec = input_health_logger_->TfLookupTimeoutSec();
    try {
        const auto tf_msg = tf_buffer_->lookupTransform(
            odom_frame_, base_frame_, ros::Time(0), ros::Duration(record.lookup_timeout_sec));
        record.lookup_ok = true;
        record.latest_tf_stamp = tf_msg.header.stamp.toSec();
        record.lookup_dt_sec = record.query_stamp - record.latest_tf_stamp;
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
    input_health_logger_->ObserveTf(record);
}

SlamSystem::~SlamSystem() {
    if (ui_) {
        ui_->Quit();
    }
}

void SlamSystem::StartSLAM(std::string map_name) {
    map_name_ = map_name;
    running_ = true;
}

bool SlamSystem::SaveMap(SaveMapService::Request& request, SaveMapService::Response& response) {
    map_name_ = request.map_id;
    std::string save_path = "./data/" + map_name_ + "/";

    SaveMap(save_path);
    response.response = 0;
    return true;
}

void SlamSystem::SaveMap(const std::string& path) {
    std::string save_path = path;
    if (save_path.empty()) {
        save_path = "./data/" + map_name_ + "/";
    }

    LOG(INFO) << "slam map saving to " << save_path;

    if (!std::filesystem::exists(save_path)) {
        std::filesystem::create_directories(save_path);
    } else {
        std::filesystem::remove_all(save_path);
        std::filesystem::create_directories(save_path);
    }

    // auto global_map_no_loop = lio_->GetGlobalMap(true);
    auto global_map = lio_->GetGlobalMap(!options_.with_loop_closing_);
    // auto global_map_raw = lio_->GetGlobalMap(!options_.with_loop_closing_, false, 0.1);

    TiledMap::Options tm_options;
    tm_options.map_path_ = save_path;

    TiledMap tm(tm_options);
    SE3 start_pose = lio_->GetAllKeyframes().front()->GetOptPose();
    tm.ConvertFromFullPCD(global_map, start_pose, save_path);

    pcl::io::savePCDFileBinaryCompressed(save_path + "/global.pcd", *global_map);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_no_loop.pcd", *global_map_no_loop);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_raw.pcd", *global_map_raw);

    if (options_.with_gridmap_) {
        /// 存为ROS兼容的模式
        auto map = g2p5_->GetNewestMap()->ToROS();
        const int width = map.info.width;
        const int height = map.info.height;

        cv::Mat nav_image(height, width, CV_8UC1);
        for (int y = 0; y < height; ++y) {
            const int rowStartIndex = y * width;
            for (int x = 0; x < width; ++x) {
                const int index = rowStartIndex + x;
                int8_t data = map.data[index];
                if (data == 0) {                                   // Free
                    nav_image.at<uchar>(height - 1 - y, x) = 255;  // White
                } else if (data == 100) {                          // Occupied
                    nav_image.at<uchar>(height - 1 - y, x) = 0;    // Black
                } else {                                           // Unknown
                    nav_image.at<uchar>(height - 1 - y, x) = 128;  // Gray
                }
            }
        }

        cv::imwrite(save_path + "/map.pgm", nav_image);

        /// yaml
        std::ofstream yamlFile(save_path + "/map.yaml");
        if (!yamlFile.is_open()) {
            LOG(ERROR) << "failed to write map.yaml";
            return;  // 文件打开失败
        }

        try {
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "image" << YAML::Value << "map.pgm";
            emitter << YAML::Key << "mode" << YAML::Value << "trinary";
            emitter << YAML::Key << "width" << YAML::Value << map.info.width;
            emitter << YAML::Key << "height" << YAML::Value << map.info.height;
            emitter << YAML::Key << "resolution" << YAML::Value << float(0.05);
            std::vector<double> orig{map.info.origin.position.x, map.info.origin.position.y, 0};
            emitter << YAML::Key << "origin" << YAML::Value << orig;
            emitter << YAML::Key << "negate" << YAML::Value << 0;
            emitter << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
            emitter << YAML::Key << "free_thresh" << YAML::Value << 0.25;

            emitter << YAML::EndMap;

            yamlFile << emitter.c_str();
            yamlFile.close();
        } catch (...) {
            yamlFile.close();
            return;
        }
    }

    RewriteOptimizedTrajectory(true);
    if (evaluation_writer_ && evaluation_writer_->Enabled()) {
        evaluation_writer_->WriteMapQualityInput(save_path + "/global.pcd", save_path + "/map.pgm");
    }

    LOG(INFO) << "map saved";
}

void SlamSystem::ProcessIMU(const lightning::IMUPtr& imu) {
    if (running_ == false) {
        return;
    }
    lio_->ProcessIMU(imu);
}

void SlamSystem::ProcessLidar(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    if (running_ == false) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    lio_->Run();

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }

    WriteEvaluationKeyframe(cur_kf_);
    PublishKeyframeOutputs(cur_kf_);
}

void SlamSystem::ProcessLidar(const livox_ros_driver2::CustomMsgConstPtr& cloud) {
    if (running_ == false) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    lio_->Run();

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }

    WriteEvaluationKeyframe(cur_kf_);
    PublishKeyframeOutputs(cur_kf_);
}

void SlamSystem::Spin() {
    if (options_.online_mode_ && node_ != nullptr) {
        ros::spin();
    }
}

void SlamSystem::WriteEvaluationKeyframe(const Keyframe::Ptr& kf) {
    if (!evaluation_writer_ || !evaluation_writer_->Enabled() || !kf) {
        return;
    }

    const auto state = kf->GetState();
    EvaluationRecord record;
    record.timestamp = state.timestamp_;
    record.pose = kf->GetOptPose();
    if (state.confidence_ > 0.0) {
        record.score = state.confidence_;
    } else {
        record.score = std::numeric_limits<double>::quiet_NaN();
    }
    record.keyframe_id = static_cast<long>(kf->GetID());
    evaluation_writer_->WriteRecord(record);
    RewriteOptimizedTrajectory(false);
}

void SlamSystem::RewriteOptimizedTrajectory(bool force) {
    if (!evaluation_writer_ || !evaluation_writer_->Enabled() || !lio_) {
        return;
    }

    const auto keyframes = lio_->GetAllKeyframes();
    if (keyframes.empty()) {
        return;
    }
    if (!force && keyframes.size() - last_optimized_trajectory_kf_count_ < 10) {
        return;
    }

    std::vector<EvaluationTrajectoryPoint> points;
    points.reserve(keyframes.size());
    for (const auto& kf : keyframes) {
        if (!kf) {
            continue;
        }
        EvaluationTrajectoryPoint point;
        point.timestamp = kf->GetState().timestamp_;
        point.pose = kf->GetOptPose();
        point.keyframe_id = static_cast<long>(kf->GetID());
        points.emplace_back(point);
    }

    evaluation_writer_->RewriteOptimizedTrajectory(points);
    last_optimized_trajectory_kf_count_ = keyframes.size();
}

nav_msgs::Odometry SlamSystem::MakeOdometryMsg(const Keyframe::Ptr& kf) const {
    nav_msgs::Odometry odom;
    if (!kf) {
        return odom;
    }

    const auto state = kf->GetState();
    const auto pose = kf->GetOptPose();
    odom.header.stamp = math::FromSec(state.timestamp_);
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pose.translation().x();
    odom.pose.pose.position.y = pose.translation().y();
    odom.pose.pose.position.z = pose.translation().z();
    odom.pose.pose.orientation.x = pose.so3().unit_quaternion().x();
    odom.pose.pose.orientation.y = pose.so3().unit_quaternion().y();
    odom.pose.pose.orientation.z = pose.so3().unit_quaternion().z();
    odom.pose.pose.orientation.w = pose.so3().unit_quaternion().w();
    return odom;
}

visualization_msgs::Marker SlamSystem::MakeKeyframeMarker(const Keyframe::Ptr& kf) const {
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "lightning_slam_keyframes";
    marker.id = kf ? static_cast<int>(kf->GetID()) : 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.35;
    marker.scale.y = 0.35;
    marker.scale.z = 0.35;
    marker.color.a = 1.0;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    if (kf) {
        const auto pose = kf->GetOptPose();
        marker.pose.position.x = pose.translation().x();
        marker.pose.position.y = pose.translation().y();
        marker.pose.position.z = pose.translation().z();
        marker.pose.orientation.w = 1.0;
    }
    return marker;
}

void SlamSystem::PublishKeyframeOutputs(const Keyframe::Ptr& kf) {
    if (!options_.online_mode_ || !kf) {
        return;
    }

    const auto odom = MakeOdometryMsg(kf);
    odom_pub_.publish(odom);

    geometry_msgs::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    path_msg_.header.stamp = odom.header.stamp;
    path_msg_.poses.emplace_back(pose);
    path_pub_.publish(path_msg_);

    if (kf->GetCloud()) {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*kf->GetCloud(), cloud_msg);
        cloud_msg.header = odom.header;
        keyframe_cloud_pub_.publish(cloud_msg);
    }

    marker_pub_.publish(MakeKeyframeMarker(kf));
}

}  // namespace lightning
