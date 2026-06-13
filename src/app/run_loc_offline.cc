//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <unistd.h>

#include "core/localization/localization.h"
#include "ui/pangolin_window.h"
#include "utils/timer.h"
#include "utils/evaluation_writer.h"
#include "utils/input_health_logger.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

#include "io/yaml_io.h"
#include <yaml-cpp/yaml.h>

DEFINE_string(input_bag, "", "输入数据包");
DEFINE_string(config, "./config/default.yaml", "配置文件");
DEFINE_string(map_path, "./data/new_map/", "地图路径");
DEFINE_string(output_dir, "", "评估输出目录");
DEFINE_string(bag_odom_topic, "/odom", "BAG中用于导出TUM的里程计话题");
DEFINE_int32(sleep_usec, 0, "每条离线bag消息处理后的休眠微秒数，0表示不休眠");

namespace {

using lightning::Quatd;
using lightning::SE3;
using lightning::Vec3d;

void ApplyRosPrivateArgCompat(int argc, char** argv) {
    constexpr const char* kConfigPrefix = "_config:=";
    constexpr const char* kInputBagPrefix = "_input_bag:=";
    constexpr const char* kMapPathPrefix = "_map_path:=";
    constexpr const char* kOutputDirPrefix = "_output_dir:=";
    constexpr const char* kBagOdomTopicPrefix = "_bag_odom_topic:=";
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg.rfind(kConfigPrefix, 0) == 0) {
            FLAGS_config = arg.substr(std::strlen(kConfigPrefix));
        } else if (arg.rfind(kInputBagPrefix, 0) == 0) {
            FLAGS_input_bag = arg.substr(std::strlen(kInputBagPrefix));
        } else if (arg.rfind(kMapPathPrefix, 0) == 0) {
            FLAGS_map_path = arg.substr(std::strlen(kMapPathPrefix));
        } else if (arg.rfind(kOutputDirPrefix, 0) == 0) {
            FLAGS_output_dir = arg.substr(std::strlen(kOutputDirPrefix));
        } else if (arg.rfind(kBagOdomTopicPrefix, 0) == 0) {
            FLAGS_bag_odom_topic = arg.substr(std::strlen(kBagOdomTopicPrefix));
        }
    }
}

class TumWriter {
   public:
    bool Open(const std::string& path) {
        path_ = path;
        file_.open(path_, std::ios::out | std::ios::trunc);
        if (!file_.is_open()) {
            LOG(ERROR) << "failed to open tum output: " << path_;
            return false;
        }
        return true;
    }

    void Write(double timestamp, const SE3& pose) {
        if (!file_.is_open() || !std::isfinite(timestamp)) {
            return;
        }
        const Vec3d t = pose.translation();
        const Quatd q = pose.unit_quaternion();
        file_ << std::fixed << std::setprecision(18) << timestamp << " " << t.x() << " " << t.y() << " " << t.z()
              << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
        ++count_;
    }

    size_t Count() const { return count_; }
    const std::string& Path() const { return path_; }

   private:
    std::string path_;
    std::ofstream file_;
    size_t count_ = 0;
};

SE3 PoseFromOdom(const nav_msgs::Odometry& odom) {
    const auto& p = odom.pose.pose.position;
    const auto& q = odom.pose.pose.orientation;
    return SE3(Quatd(q.w, q.x, q.y, q.z), Vec3d(p.x, p.y, p.z));
}

void SleepAfterMessageIfRequested() {
    if (FLAGS_sleep_usec > 0) {
        usleep(static_cast<useconds_t>(FLAGS_sleep_usec));
    }
}

}  // namespace

/// 运行定位的测试
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    ros::init(argc, argv, "run_loc_offline");
    google::ParseCommandLineFlags(&argc, &argv, true);
    ApplyRosPrivateArgCompat(argc, argv);

    if (FLAGS_input_bag.empty()) {
        LOG(ERROR) << "未指定输入数据";
        return -1;
    }

    using namespace lightning;

    RosbagIO rosbag(FLAGS_input_bag);

    loc::Localization::Options options;
    options.online_mode_ = false;

    loc::Localization loc(options);
    auto yaml_node = YAML::LoadFile(FLAGS_config);
    std::shared_ptr<EvaluationWriter> evaluation_writer;
    std::shared_ptr<TumWriter> global_loc_tum;
    std::shared_ptr<TumWriter> lio_local_tum;
    std::shared_ptr<TumWriter> bag_odom_tum;
    auto input_health_logger = std::make_shared<InputHealthLogger>();
    input_health_logger->Init(FLAGS_config, "loc_offline");

    if (!FLAGS_output_dir.empty()) {
        std::filesystem::create_directories(FLAGS_output_dir);

        global_loc_tum = std::make_shared<TumWriter>();
        lio_local_tum = std::make_shared<TumWriter>();
        bag_odom_tum = std::make_shared<TumWriter>();

        global_loc_tum->Open(FLAGS_output_dir + "/global_loc.tum");
        lio_local_tum->Open(FLAGS_output_dir + "/lio_local.tum");
        bag_odom_tum->Open(FLAGS_output_dir + "/bag_odom.tum");

        LOG(INFO) << "offline output dir: " << FLAGS_output_dir;
        LOG(INFO) << "global loc tum: " << global_loc_tum->Path();
        LOG(INFO) << "lio local tum: " << lio_local_tum->Path();
        LOG(INFO) << "bag odom tum: " << bag_odom_tum->Path();
    }

    const bool evaluation_enabled =
        yaml_node["system"]["evaluation"] && yaml_node["system"]["evaluation"].as<bool>();
    if (evaluation_enabled && (yaml_node["system"]["evaluation_output_dir"] || !FLAGS_output_dir.empty())) {
        const std::string evaluation_output_dir =
            FLAGS_output_dir.empty() ? yaml_node["system"]["evaluation_output_dir"].as<std::string>() : FLAGS_output_dir;
        if (!evaluation_output_dir.empty()) {
            evaluation_writer = std::make_shared<EvaluationWriter>();
            evaluation_writer->Init(evaluation_output_dir, "loc");
            evaluation_writer->WriteMapQualityInput(FLAGS_map_path + "/global.pcd", FLAGS_map_path + "/map.pgm");
        }
    }
    if (evaluation_writer || global_loc_tum) {
        loc.SetLidarLocCallback([evaluation_writer, global_loc_tum](const loc::LocalizationResult& result) {
            auto lidar_loc_result = result;
            lidar_loc_result.valid_ = lidar_loc_result.lidar_loc_valid_;
            if (evaluation_writer) {
                evaluation_writer->WriteLocalizationResult(lidar_loc_result);
            }
            if (global_loc_tum && lidar_loc_result.valid_) {
                global_loc_tum->Write(lidar_loc_result.timestamp_, lidar_loc_result.pose_);
            }
        });
    }
    if (lio_local_tum) {
        loc.SetLidarOdomCallback([lio_local_tum](const NavState& state) {
            if (state.pose_is_ok_) {
                lio_local_tum->Write(state.timestamp_, state.GetPose());
            }
        });
    }
    if (!loc.Init(FLAGS_config, FLAGS_map_path)) {
        LOG(ERROR) << "failed to init localization";
        return -1;
    }

    lightning::YAML_IO yaml(FLAGS_config);
    std::string lidar_topic = yaml.GetValue<std::string>("common", "lidar_topic");
    std::string imu_topic = yaml.GetValue<std::string>("common", "imu_topic");
    const std::string livox_lidar_topic = "/livox/lidar";
    if (input_health_logger->Enabled()) {
        input_health_logger->RegisterImu(imu_topic);
        input_health_logger->RegisterLidar(lidar_topic);
        input_health_logger->RegisterLidar(livox_lidar_topic);
        input_health_logger->RegisterOdom(FLAGS_bag_odom_topic, !FLAGS_bag_odom_topic.empty());
        input_health_logger->RegisterTf("odom", "base_link", false);
    }

    rosbag.AddHandle(imu_topic, [input_health_logger](const RosbagIO::MsgType& m) {
        auto msg = m.instantiate<sensor_msgs::Imu>();
        if (msg && input_health_logger->Enabled()) {
            input_health_logger->ObserveImu(m.getTopic(), *msg, ToSec(m.getTime()));
            input_health_logger->Tick(ToSec(m.getTime()));
        }
        return true;
    });
    rosbag.AddHandle(lidar_topic, [input_health_logger](const RosbagIO::MsgType& m) {
        auto msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (msg && input_health_logger->Enabled()) {
            input_health_logger->ObservePointCloud2(m.getTopic(), *msg, ToSec(m.getTime()));
            input_health_logger->Tick(ToSec(m.getTime()));
        }
        return true;
    });
    rosbag.AddHandle(livox_lidar_topic, [input_health_logger](const RosbagIO::MsgType& m) {
        auto msg = m.instantiate<livox_ros_driver2::CustomMsg>();
        if (msg && input_health_logger->Enabled()) {
            input_health_logger->ObserveLivox(m.getTopic(), *msg, ToSec(m.getTime()));
            input_health_logger->Tick(ToSec(m.getTime()));
        }
        return true;
    });

    if (!FLAGS_bag_odom_topic.empty()) {
        rosbag.AddHandle(FLAGS_bag_odom_topic, [bag_odom_tum, input_health_logger, &loc](const RosbagIO::MsgType& m) {
            auto msg = m.instantiate<nav_msgs::Odometry>();
            if (msg) {
                if (input_health_logger->Enabled()) {
                    input_health_logger->ObserveOdom(m.getTopic(), *msg, ToSec(m.getTime()));
                    input_health_logger->Tick(ToSec(m.getTime()));
                }
                loc.ObserveChassisOdom(m.getTopic(), *msg, ToSec(m.getTime()));
                if (bag_odom_tum) {
                    bag_odom_tum->Write(ToSec(msg->header.stamp), PoseFromOdom(*msg));
                }
            }
            return true;
        });
        LOG(INFO) << "bag odom topic: " << FLAGS_bag_odom_topic;
    }
    LOG(INFO) << "offline sleep usec: " << FLAGS_sleep_usec;

    rosbag
        .AddImuHandle(imu_topic,
                      [&loc](IMUPtr imu) {
                          loc.ProcessIMUMsg(imu);
                          SleepAfterMessageIfRequested();
                          return true;
                      })
        .AddPointCloud2Handle(lidar_topic,
                              [&loc](sensor_msgs::PointCloud2ConstPtr cloud) {
                                  loc.ProcessLidarMsg(cloud);
                                  SleepAfterMessageIfRequested();
                                  return true;
                              })
        .AddLivoxCloudHandle("/livox/lidar",
                             [&loc](livox_ros_driver2::CustomMsgConstPtr cloud) {
                                 loc.ProcessLivoxLidarMsg(cloud);
                                 SleepAfterMessageIfRequested();
                                 return true;
                             })
        .Go();
    input_health_logger->Finish();

    Timer::PrintAll();
    loc.Finish();

    if (global_loc_tum) {
        LOG(INFO) << "lidar loc tum samples: " << global_loc_tum->Count();
    }
    if (lio_local_tum) {
        LOG(INFO) << "lio local tum samples: " << lio_local_tum->Count();
    }
    if (bag_odom_tum) {
        LOG(INFO) << "bag odom tum samples: " << bag_odom_tum->Count();
    }

    LOG(INFO) << "done";

    return 0;
}
