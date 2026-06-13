//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include <cstring>

#include "core/system/slam.h"
#include "ui/pangolin_window.h"
#include "utils/input_health_logger.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

#include "io/yaml_io.h"

DEFINE_string(input_bag, "", "输入数据包");
DEFINE_string(config, "./config/default.yaml", "配置文件");
DEFINE_string(output_dir, "", "输出地图目录");

namespace {

void ApplyRosPrivateArgCompat(int argc, char** argv) {
    constexpr const char* kConfigPrefix = "_config:=";
    constexpr const char* kInputBagPrefix = "_input_bag:=";
    constexpr const char* kOutputDirPrefix = "_output_dir:=";
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg.rfind(kConfigPrefix, 0) == 0) {
            FLAGS_config = arg.substr(std::strlen(kConfigPrefix));
        } else if (arg.rfind(kInputBagPrefix, 0) == 0) {
            FLAGS_input_bag = arg.substr(std::strlen(kInputBagPrefix));
        } else if (arg.rfind(kOutputDirPrefix, 0) == 0) {
            FLAGS_output_dir = arg.substr(std::strlen(kOutputDirPrefix));
        }
    }
}

}  // namespace

/// 运行一个LIO前端，带可视化
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    ros::init(argc, argv, "run_slam_offline");
    google::ParseCommandLineFlags(&argc, &argv, true);
    ApplyRosPrivateArgCompat(argc, argv);

    if (FLAGS_input_bag.empty()) {
        LOG(ERROR) << "未指定输入数据";
        return -1;
    }

    using namespace lightning;

    RosbagIO rosbag(FLAGS_input_bag);

    SlamSystem::Options options;
    options.online_mode_ = false;

    SlamSystem slam(options);

    /// 实时模式好像掉帧掉的比较厉害？

    if (!slam.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init slam";
        return -1;
    }

    slam.StartSLAM("new_map");

    lightning::YAML_IO yaml(FLAGS_config);
    std::string lidar_topic = yaml.GetValue<std::string>("common", "lidar_topic");
    std::string imu_topic = yaml.GetValue<std::string>("common", "imu_topic");
    const std::string livox_lidar_topic = "/livox/lidar";
    auto input_health_logger = std::make_shared<InputHealthLogger>();
    input_health_logger->Init(FLAGS_config, "slam_offline");
    if (input_health_logger->Enabled()) {
        input_health_logger->RegisterImu(imu_topic);
        input_health_logger->RegisterLidar(lidar_topic);
        input_health_logger->RegisterLidar(livox_lidar_topic);
        input_health_logger->RegisterOdom("/odom", true);
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
    rosbag.AddHandle("/odom", [input_health_logger](const RosbagIO::MsgType& m) {
        auto msg = m.instantiate<nav_msgs::Odometry>();
        if (msg && input_health_logger->Enabled()) {
            input_health_logger->ObserveOdom(m.getTopic(), *msg, ToSec(m.getTime()));
            input_health_logger->Tick(ToSec(m.getTime()));
        }
        return true;
    });

    rosbag
        /// IMU 的处理
        .AddImuHandle(imu_topic,
                      [&slam](IMUPtr imu) {
                          slam.ProcessIMU(imu);
                          return true;
                      })

        /// lidar 的处理
        .AddPointCloud2Handle(lidar_topic,
                              [&slam](sensor_msgs::PointCloud2ConstPtr msg) {
                                  slam.ProcessLidar(msg);
                                  return true;
                              })
        /// livox 的处理
        .AddLivoxCloudHandle(livox_lidar_topic,
                             [&slam](livox_ros_driver2::CustomMsgConstPtr cloud) {
                                 slam.ProcessLidar(cloud);
                                 return true;
                             })
        .Go();
    input_health_logger->Finish();

    slam.SaveMap(FLAGS_output_dir);
    Timer::PrintAll();

    LOG(INFO) << "done";

    return 0;
}
