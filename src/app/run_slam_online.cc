//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include "core/system/slam.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

DEFINE_string(config, "./config/default.yaml", "配置文件");

/// 运行一个LIO前端，带可视化
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    ros::init(argc, argv, "run_slam_online");
    google::ParseCommandLineFlags(&argc, &argv, true);
    ros::NodeHandle pnh("~");
    pnh.param<std::string>("config", FLAGS_config, FLAGS_config);

    using namespace lightning;

    SlamSystem::Options options;
    options.online_mode_ = true;

    SlamSystem slam(options);
    if (!slam.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init slam";
        return -1;
    }

    slam.StartSLAM("new_map");
    slam.Spin();

    Timer::PrintAll();

    ros::shutdown();

    LOG(INFO) << "done";

    return 0;
}
