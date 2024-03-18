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

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

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

struct PointXYZPDE
{
    PCL_ADD_POINT4D;
    uint8_t existence_probability_percent;
    float depth;
    float elevation;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZPDE,
    (float, x, x) (float, y, y) (float, z, z) 
    (uint8_t, existence_probability_percent, existence_probability_percent)
    (float, depth, depth)
    (float, elevation, elevation)
)

// typedef pcl::PointXYZ PointType;
typedef PointXYZPDE PointType;

class PointCloudCombiner : public rclcpp::Node {
    public:
        PointCloudCombiner(const std::string &name);
        ~PointCloudCombiner() = default;

    private:
        void publishCombinedPointcloud();
        void callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg, const std::string &topicName);
        void processPointcloud(
            pcl::PointCloud<PointType>::Ptr &cloud_in,
            boost::circular_buffer<std::pair<pcl::PointCloud<PointType>::Ptr, rclcpp::Time>> &buff,
            // boost::circular_buffer<pcl::PointCloud<PointType>::Ptr> &buff,
            const Eigen::Matrix4f &transform,
            const std::string &topicName,
            const rclcpp::Time &timestamp);

        void declareAndGetParameters();
        void createPublishers();
        void createSubscribers();
        void initializePointcloudsAndBuffers();
        void setupTransforms();
        void createTimer();
        void setupFilters();

        // Point Clouds
        pcl::PointCloud<PointType>::Ptr merged_cloud_;
        // std::shared_ptr<pcl::PointCloud<PointType>> merged_cloud_;
        std::unordered_map<std::string, pcl::PointCloud<PointType>::Ptr> pointclouds_;
        std::unordered_map<std::string, boost::circular_buffer<std::pair<pcl::PointCloud<PointType>::Ptr, rclcpp::Time>>> circular_buffers_;
        // std::unordered_map<std::string, boost::circular_buffer<pcl::PointCloud<PointType>::Ptr>> circular_buffers_;


        int buff_capacity_;

        pcl::StatisticalOutlierRemoval<PointType> sor_;
        pcl::ConditionalRemoval<PointType> condrem_;
        // Preprocessing
        pcl::CropBox<PointType> crop_rear_front_;
        pcl::CropBox<PointType> crop_left_right_;

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
        const std::string PARAM_USE_4_LIADR = "use_4_lidar";
        const std::string PARAM_CUTOFF_DIST = "cutoff_dist";
        const std::string PARAM_CUTOFF_ELEVATION = "cutoff_elevation";

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
        double cutoff_dist_;
        double cutoff_elevation_;
        double cur_stamp_;

        bool use_4_lidar_;
        std::unordered_map<std::string, bool> recv_;
        
        rclcpp::Time latest_time_;

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

        std::unordered_map<std::string, Eigen::Matrix4f> transforms_;
        std::vector<std::string> topics_;
};

// Helper function to declare and retrieve parameters
template <typename T>
struct identity { typedef T type; };

template <typename T>
void declare_param(rclcpp::Node* node, const std::string param_name, T& param, const typename identity<T>::type& default_value) {
    node->declare_parameter(param_name, default_value);
    if (!node->get_parameter(param_name, param)) {
    RCLCPP_WARN(node->get_logger(), "Parameter %s not found", param_name.c_str());
    }
}

#endif  // MERGE_CLOUDS_H