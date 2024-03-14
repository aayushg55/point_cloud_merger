#ifndef MERGE_CLOUDS_H
#define MERGE_CLOUDS_H

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

#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

// BOOST
#include <boost/format.hpp>
#include <boost/circular_buffer.hpp>

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

struct PointXYZP
{
    PCL_ADD_POINT4D;
    uint8_t existence_probability_percent;
    float reflectance;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZP,
    (float, x, x) (float, y, y) (float, z, z) (uint8_t, existence_probability_percent, existence_probability_percent)
    (float, reflectance, reflectance)
)

// typedef pcl::PointXYZ PointType;
typedef PointXYZP PointType;

class PointCloudCombiner : public rclcpp::Node {
    public:

        PointCloudCombiner(const std::string &name);
        ~PointCloudCombiner() = default;

    private:
        void publishCombinedPointCloud();
        void timerCallback();
        void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg, const std::string topicName);
        void callbackRight(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg);
        void callbackLeft(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg);
        void callbackFront(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg);
        void callbackRear(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg);

        void process(pcl::PointCloud<PointType>::Ptr cloud_in, 
            pcl::PointCloud<PointType>::Ptr cloud_out, 
            boost::circular_buffer<pcl::PointCloud<PointType>::Ptr>& buff,
            Eigen::Matrix4f transform,
            bool do_transform = true);

        pcl::PointCloud<PointType>::Ptr getTargetCloud(const std::string& topic);

        // Point Clouds
        pcl::PointCloud<PointType>::Ptr merged_cloud_;
        pcl::PointCloud<PointType>::Ptr front_cloud_;
        pcl::PointCloud<PointType>::Ptr left_cloud_;
        pcl::PointCloud<PointType>::Ptr right_cloud_;
        pcl::PointCloud<PointType>::Ptr rear_cloud_;

        pcl::PointCloud<PointType>::Ptr front_cloud_filtered_;
        pcl::PointCloud<PointType>::Ptr left_cloud_filtered_;
        pcl::PointCloud<PointType>::Ptr right_cloud_filtered_;
        pcl::PointCloud<PointType>::Ptr rear_cloud_filtered_;

        boost::circular_buffer<pcl::PointCloud<PointType>::Ptr> front_buff_;
        boost::circular_buffer<pcl::PointCloud<PointType>::Ptr> left_buff_;
        boost::circular_buffer<pcl::PointCloud<PointType>::Ptr> right_buff_;
        boost::circular_buffer<pcl::PointCloud<PointType>::Ptr> rear_buff_;
        int buff_capacity_;

        pcl::StatisticalOutlierRemoval<PointType> sor_;
        pcl::ConditionalRemoval<PointType> condrem_;
        // Preprocessing
        pcl::CropBox<PointType> crop_;
        pcl::VoxelGrid<PointType> voxel_;

        const std::string PARAM_MERGED_PC_TOPIC = "merged_pc_topic";
        const std::string PARAM_FRONT_PC_TOPIC = "front_pc_topic";
        const std::string PARAM_LEFT_PC_TOPIC = "left_pc_topic";
        const std::string PARAM_RIGHT_PC_TOPIC = "right_pc_topic";
        const std::string PARAM_REAR_PC_TOPIC = "rear_pc_topic";

        const std::string PARAM_FRONT_LIDAR_FRAME = "front_lidar_frame";
        const std::string PARAM_RIGHT_LIDAR_FRAME = "right_lidar_frame";
        const std::string PARAM_LEFT_LIDAR_FRAME = "left_lidar_frame";
        const std::string PARAM_REAR_LIDAR_FRAME = "rear_lidar_frame";

        const std::string PARAM_PUB_FREQ = "pub_freq";
        const std::string PARAM_CROP_BOX_SIZE = "crop_size";
        const std::string PARAM_VOXEL_RES = "voxel_res";

        std::string merged_pc_topic_;
        std::string front_pc_topic_;
        std::string left_pc_topic_;
        std::string right_pc_topic_;
        std::string rear_pc_topic_;

        std::string front_lidar_frame_;
        std::string left_lidar_frame_;
        std::string right_lidar_frame_;
        std::string rear_lidar_frame_;

        float pub_freq_;
        double crop_size_;
        double voxel_res_;

        rclcpp::Time latest_time_;

        int left_update_;
        int right_update_;
        int front_update_;

        std::atomic<bool> left_recv_;
        std::atomic<bool> front_recv_;
        std::atomic<bool> right_recv_;
        std::atomic<bool> rear_recv_;

        // Callback groups
        rclcpp::CallbackGroup::SharedPtr left_cb_group, right_cb_group, front_cb_group, rear_cb_group;

        //Publisher
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr combined_publisher_;

        //Subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr left_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr right_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr front_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rear_subscriber_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        Eigen::Matrix4f front_2_right_, front_2_left_, front_2_rear_;
};

#endif  // MERGE_CLOUDS_H
