//
// Created by xiang on 23-12-14.
//

#include "bag_io.h"

#include <glog/logging.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <unistd.h>

#include <iomanip>
#include <iostream>
#include <limits>

namespace lightning {

void RosbagIO::Go(int sleep_usec) {
    rosbag::Bag bag;
    bag.open(bag_file_, rosbag::bagmode::Read);
    rosbag::View view(bag);
    const size_t total_msgs = view.size();
    size_t processed_msgs = 0;
    size_t last_reported_percent = std::numeric_limits<size_t>::max();
    size_t last_reported_msg = 0;

    auto report_progress = [&](const std::string &topic, bool force) {
        if (total_msgs == 0) {
            return;
        }

        const double percent = 100.0 * static_cast<double>(processed_msgs) / static_cast<double>(total_msgs);
        const size_t percent_floor = static_cast<size_t>(percent);
        const bool percent_changed = percent_floor != last_reported_percent;
        const bool enough_msgs = processed_msgs - last_reported_msg >= 1000;
        if (!force && processed_msgs != 1 && !percent_changed && !enough_msgs) {
            return;
        }

        std::cout << "[slam_progress] processed=" << processed_msgs << "/" << total_msgs << " percent=" << std::fixed
                  << std::setprecision(1) << percent << "% topic=" << topic << std::endl;
        last_reported_percent = percent_floor;
        last_reported_msg = processed_msgs;
    };

    for (const auto &m : view) {
        const std::string &topic = m.getTopic();

        auto generic_iter = process_func_.find(topic);
        if (generic_iter != process_func_.end()) {
            generic_iter->second(m);
        }

        auto imu_iter = imu_process_func_.find(topic);
        if (imu_iter != imu_process_func_.end()) {
            auto msg = m.instantiate<sensor_msgs::Imu>();
            if (msg) {
                IMUPtr imu = std::make_shared<IMU>();
                imu->timestamp = ToSec(msg->header.stamp);
                imu->linear_acceleration =
                    Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                imu->angular_velocity =
                    Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
                imu_iter->second(imu);
            }
        }

        auto point_iter = point_cloud2_process_func_.find(topic);
        if (point_iter != point_cloud2_process_func_.end()) {
            auto msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (msg) {
                point_iter->second(msg);
            }
        }

        auto livox_iter = livox_process_func_.find(topic);
        if (livox_iter != livox_process_func_.end()) {
            auto msg = m.instantiate<livox_ros_driver2::CustomMsg>();
            if (msg) {
                livox_iter->second(msg);
            }
        }

        if (sleep_usec > 0) {
            usleep(sleep_usec);
        }

        ++processed_msgs;
        report_progress(topic, processed_msgs == total_msgs);

        if (lightning::debug::flg_exit) {
            bag.close();
            return;
        }
    }

    bag.close();
    LOG(INFO) << "bag " << bag_file_ << " finished.";
}

}  // namespace lightning
