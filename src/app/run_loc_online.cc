//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include "core/system/loc_system.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

DEFINE_string(config, "./config/default.yaml", "配置文件");

/// 运行定位的测试
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    ros::init(argc, argv, "run_loc_online");
    google::ParseCommandLineFlags(&argc, &argv, true);
    ros::NodeHandle pnh("~");
    pnh.param<std::string>("config", FLAGS_config, FLAGS_config);

    using namespace lightning;

    LocSystem::Options opt;
    LocSystem loc(opt);

    if (!loc.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init loc";
    }

    /// 默认起点开始定位
    loc.SetInitPose(SE3());
    loc.Spin();

    ros::shutdown();

    return 0;
}
