#ifndef MERGE_CLOUDS_H
#define MERGE_CLOUDS_H


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl_ros/transforms.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

// Point cloud format for Luminar lidar
struct PointXYZIRT
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    float ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (float, ring, ring) (float, time, time)
)

class PointCloudCombiner : public rclcpp::Node {
    public:

        PointCloudCombiner(const std::string &name);
        ~PointCloudCombiner() = default;

    private:
        void publishCombinedPointCloud();
        void timerCallback();
        void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg, const std::string topicName);

        pcl::PointCloud<PointXYZIRT>::Ptr getTargetCloud(const std::string& topic);

        // Point Clouds
        pcl::PointCloud<PointXYZIRT>::Ptr merged_cloud_;
        pcl::PointCloud<PointXYZIRT>::Ptr front_cloud_;
        pcl::PointCloud<PointXYZIRT>::Ptr left_cloud_;
        pcl::PointCloud<PointXYZIRT>::Ptr right_cloud_;

        const std::string PARAM_MERGED_PC_TOPIC = "merged_pc_topic";
        const std::string PARAM_FRONT_PC_TOPIC = "front_pc_topic";
        const std::string PARAM_LEFT_PC_TOPIC = "left_pc_topic";
        const std::string PARAM_RIGHT_PC_TOPIC = "right_pc_topic";

        const std::string PARAM_FRONT_LIDAR_FRAME = "front_lidar_frame";
        const std::string PARAM_RIGHT_LIDAR_FRAME = "right_lidar_frame";
        const std::string PARAM_LEFT_LIDAR_FRAME = "left_lidar_frame";

        const std::string PARAM_PUB_FREQ = "pub_freq";

        std::string merged_pc_topic_;
        std::string front_pc_topic_;
        std::string left_pc_topic_;
        std::string right_pc_topic_;

        std::string front_lidar_frame_;
        std::string left_lidar_frame_;
        std::string right_lidar_frame_;
        float pub_freq_;
        rclcpp::Time latest_time_;

        int left_update_;
        int right_update_;
        int front_update_;

        //Publisher
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr combined_publisher_;

        //Subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr left_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr right_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr front_subscriber_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif  // MERGE_CLOUDS_H
