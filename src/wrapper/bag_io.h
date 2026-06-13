//
// Created by xiang on 23-12-14.
//

#ifndef LIGHTNING_BAG_IO_H
#define LIGHTNING_BAG_IO_H

#include <functional>
#include <map>
#include <string>
#include <csignal>

#include <nav_msgs/Odometry.h>
#include <rosbag/message_instance.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <livox_ros_driver2/CustomMsg.h>

#include "common/imu.h"
#include "common/odom.h"
#include "common/point_def.h"
#include "core/lightning_math.hpp"
#include "io/dataset_type.h"
#include "wrapper/ros_utils.h"

namespace lightning {

/**
 * ROS1 bag reader.
 * 指定一个包名，添加一些回调函数，就可以顺序遍历这个包。
 */
class RosbagIO {
   public:
    explicit RosbagIO(std::string bag_file, DatasetType dataset_type = DatasetType::NCLT)
        : bag_file_(std::move(bag_file)), dataset_type_(dataset_type) {
        /// handle ctrl-c
        std::signal(SIGINT, lightning::debug::SigHandle);
    }

    using MsgType = rosbag::MessageInstance;
    using MessageProcessFunction = std::function<bool(const MsgType &m)>;

    /// 一些方便直接使用的topics, messages
    using Scan2DHandle = std::function<bool(sensor_msgs::LaserScanConstPtr)>;
    using PointCloud2Handle = std::function<bool(sensor_msgs::PointCloud2ConstPtr)>;
    using LivoxCloud2Handle = std::function<bool(livox_ros_driver2::CustomMsgConstPtr)>;
    using FullPointCloudHandle = std::function<bool(FullCloudPtr)>;
    using ImuHandle = std::function<bool(IMUPtr)>;
    using OdomHandle = std::function<bool(const OdomPtr &)>;

    /**
     * 遍历文件内容，调用回调函数
     * @param sleep_usec 每调用一个回调后的等待时间
     */
    void Go(int sleep_usec = 0);

    /// 通用处理函数
    RosbagIO &AddHandle(const std::string &topic_name, MessageProcessFunction func) {
        process_func_.emplace(topic_name, func);
        return *this;
    }

    /// point cloud 2 处理
    RosbagIO &AddPointCloud2Handle(const std::string &topic_name, PointCloud2Handle f) {
        point_cloud2_process_func_.emplace(topic_name, std::move(f));
        return *this;
    }

    /// livox 处理
    RosbagIO &AddLivoxCloudHandle(const std::string &topic_name, LivoxCloud2Handle f) {
        livox_process_func_.emplace(topic_name, std::move(f));
        return *this;
    }

    RosbagIO &AddImuHandle(const std::string &topic_name, ImuHandle f) {
        imu_process_func_.emplace(topic_name, std::move(f));
        return *this;
    }

    /// 清除现有的处理函数
    void CleanProcessFunc() {
        process_func_.clear();
        point_cloud2_process_func_.clear();
        livox_process_func_.clear();
        imu_process_func_.clear();
    }

   private:
    std::map<std::string, MessageProcessFunction> process_func_;
    std::map<std::string, PointCloud2Handle> point_cloud2_process_func_;
    std::map<std::string, LivoxCloud2Handle> livox_process_func_;
    std::map<std::string, ImuHandle> imu_process_func_;

    std::string bag_file_;
    DatasetType dataset_type_ = DatasetType::NCLT;
};
}  // namespace lightning

#endif  // SLAM_ROS_BAG_IO_H
