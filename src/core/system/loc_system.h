//
// Created by xiang on 25-9-8.
//

#ifndef LIGHTNING_LOC_SYSTEM_H
#define LIGHTNING_LOC_SYSTEM_H

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <livox_ros_driver2/CustomMsg.h>

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"

namespace lightning {

namespace loc {
class Localization;
}
class EvaluationWriter;
class InputHealthLogger;

class LocSystem {
   public:
    struct Options {
        bool pub_tf_ = true;  // 是否发布tf
    };

    explicit LocSystem(Options options);
    ~LocSystem();

    /// 初始化，地图路径在yaml里配置
    bool Init(const std::string& yaml_path);

    /// 设置初始化位姿
    void SetInitPose(const SE3& pose);

    /// 处理IMU
    void ProcessIMU(const lightning::IMUPtr& imu);

    /// 处理点云
    void ProcessLidar(const sensor_msgs::PointCloud2ConstPtr& cloud);
    void ProcessLidar(const livox_ros_driver2::CustomMsgConstPtr& cloud);

    /// 实时模式下的spin
    void Spin();

   private:
    void PublishLocalizationOutputs(const geometry_msgs::TransformStamped& pose_msg);
    nav_msgs::Odometry MakeOdometryMsg(const geometry_msgs::TransformStamped& pose_msg) const;
    visualization_msgs::Marker MakePoseMarker(const geometry_msgs::TransformStamped& pose_msg) const;
    void CheckInputTfHealth(const ros::TimerEvent& event);

    Options options_;

    std::shared_ptr<loc::Localization> loc_ = nullptr;  // 定位接口
    std::shared_ptr<EvaluationWriter> evaluation_writer_ = nullptr;

    std::atomic_bool loc_started_ = false;  // 是否开启定位
    std::atomic_bool map_loaded_ = false;   // 地图是否已载入

    /// 实时模式下的ROS1 node, subscribers
    std::unique_ptr<ros::NodeHandle> node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = nullptr;

    std::string imu_topic_;
    std::string cloud_topic_;
    std::string livox_topic_;
    std::string input_odom_topic_ = "/odom";
    std::string map_frame_ = "map";
    std::string odom_frame_ = "odom";
    std::string base_frame_ = "base_link";
    std::string livox_frame_ = "livox_frame";

    ros::Subscriber imu_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber livox_sub_;
    ros::Subscriber input_odom_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher path_pub_;
    ros::Publisher marker_pub_;
    ros::Timer input_health_timer_;
    nav_msgs::Path path_msg_;
    std::shared_ptr<InputHealthLogger> input_health_logger_ = nullptr;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = nullptr;
};

};  // namespace lightning

#endif  // LIGHTNING_LOC_SYSTEM_H
