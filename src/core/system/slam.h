//
// Created by xiang on 25-5-6.
//

#ifndef LIGHTNING_SLAM_H
#define LIGHTNING_SLAM_H

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <visualization_msgs/Marker.h>

#include "lightning/SaveMap.h"
#include <livox_ros_driver2/CustomMsg.h>

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"

namespace lightning {

class LaserMapping;  //  lio 前端
class LoopClosing;   // 回环检测
class EvaluationWriter;
class InputHealthLogger;

namespace ui {
class PangolinWindow;
}

namespace g2p5 {
class G2P5;
}

/**
 * SLAM 系统调用接口
 */
class SlamSystem {
   public:
    struct Options {
        Options() {}

        bool online_mode_ = true;  // 在线模式，在线模式下会起一些子线程来做异步处理

        bool with_cc_ = true;               // 是否需要带交叉验证
        bool with_gridmap_ = true;          // 是否需要2D栅格
        bool with_loop_closing_ = true;     // 是否需要回环检测
        bool with_visualization_ = true;    // 是否需要可视化UI
        bool with_2dvisualization_ = true;  // 是否需要2D可视化UI

        bool step_on_kf_ = true;  // 是否在关键帧处暂停p
    };

    using SaveMapService = ::lightning::SaveMap;

    SlamSystem(Options options);
    ~SlamSystem();

    /// 初始化
    bool Init(const std::string& yaml_path);

    /// 对外部交互接口
    /// 开始建图，输入地图名称
    void StartSLAM(std::string map_name);

    /// 保存地图，默认保存至./data/地图名/ 下方
    void SaveMap(const std::string& path = "");

    /// 处理IMU
    void ProcessIMU(const lightning::IMUPtr& imu);

    /// 处理点云
    void ProcessLidar(const sensor_msgs::PointCloud2ConstPtr& cloud);
    void ProcessLidar(const livox_ros_driver2::CustomMsgConstPtr& cloud);

    /// 实时模式下的spin
    void Spin();

   private:
    /// ros端保存地图的实现
    bool SaveMap(SaveMapService::Request& request, SaveMapService::Response& response);
    void WriteEvaluationKeyframe(const Keyframe::Ptr& kf);
    void RewriteOptimizedTrajectory(bool force);
    void PublishKeyframeOutputs(const Keyframe::Ptr& kf);
    nav_msgs::Odometry MakeOdometryMsg(const Keyframe::Ptr& kf) const;
    visualization_msgs::Marker MakeKeyframeMarker(const Keyframe::Ptr& kf) const;
    void CheckInputTfHealth(const ros::TimerEvent& event);

    Options options_;
    std::atomic_bool running_ = false;

    ros::ServiceServer savemap_service_;

    std::string map_name_;  // 地图名

    std::shared_ptr<LaserMapping> lio_ = nullptr;       // lio 前端
    std::shared_ptr<LoopClosing> lc_ = nullptr;         // 回环检测
    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;  // ui
    std::shared_ptr<g2p5::G2P5> g2p5_ = nullptr;        // 栅格地图

    Keyframe::Ptr cur_kf_ = nullptr;
    std::shared_ptr<EvaluationWriter> evaluation_writer_ = nullptr;
    std::size_t last_optimized_trajectory_kf_count_ = 0;

    /// 实时模式下的ROS1 node, subscribers
    std::unique_ptr<ros::NodeHandle> node_;
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
    ros::Publisher keyframe_cloud_pub_;
    ros::Publisher marker_pub_;
    ros::Timer input_health_timer_;
    nav_msgs::Path path_msg_;
    std::shared_ptr<InputHealthLogger> input_health_logger_ = nullptr;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = nullptr;
};
}  // namespace lightning

#endif  // LIGHTNING_SLAM_H
