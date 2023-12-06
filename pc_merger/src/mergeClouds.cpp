#include "mergeClouds.hpp"
#include <chrono>

PointCloudCombiner::PointCloudCombiner(const std::string &name) 
    : Node(name)
{
    // Declare parameters
    this->declare_parameter(PARAM_MERGED_PC_TOPIC);
    this->declare_parameter(PARAM_FRONT_PC_TOPIC);
    this->declare_parameter(PARAM_LEFT_PC_TOPIC);
    this->declare_parameter(PARAM_RIGHT_PC_TOPIC);

    this->declare_parameter(PARAM_FRONT_LIDAR_FRAME);
    this->declare_parameter(PARAM_LEFT_LIDAR_FRAME);
    this->declare_parameter(PARAM_RIGHT_LIDAR_FRAME);

    // Read parameters
    if (!this->get_parameter(PARAM_MERGED_PC_TOPIC, merged_pc_topic_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_MERGED_PC_TOPIC.c_str());
    }
    if (!this->get_parameter(PARAM_FRONT_PC_TOPIC, front_pc_topic_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_FRONT_PC_TOPIC.c_str());
    }
    if (!this->get_parameter(PARAM_LEFT_PC_TOPIC, left_pc_topic_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_LEFT_PC_TOPIC.c_str());
    }
    if (!this->get_parameter(PARAM_RIGHT_PC_TOPIC, right_pc_topic_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_RIGHT_PC_TOPIC.c_str());
    }
    if (!this->get_parameter(PARAM_FRONT_LIDAR_FRAME, front_lidar_frame_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_FRONT_LIDAR_FRAME.c_str());
    }
    if (!this->get_parameter(PARAM_LEFT_LIDAR_FRAME, left_lidar_frame_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_LEFT_LIDAR_FRAME.c_str());
    }
    if (!this->get_parameter(PARAM_RIGHT_LIDAR_FRAME, right_lidar_frame_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_RIGHT_LIDAR_FRAME.c_str());
    }

    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))  // Keep the last 10 messages
        .best_effort()            // Use best-effort reliability
        .durability_volatile();
    // Create publishers for the combined point cloud
    combined_publisher_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(merged_pc_topic_, 1);

    // // Create subscribers for the individual point clouds
    std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg)> cb_front = std::bind(
        &PointCloudCombiner::cloudHandler, this, std::placeholders::_1, front_pc_topic_);
    std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg)> cb_left = std::bind(
        &PointCloudCombiner::cloudHandler, this, std::placeholders::_1, left_pc_topic_);
    std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg)> cb_right = std::bind(
        &PointCloudCombiner::cloudHandler, this, std::placeholders::_1, right_pc_topic_);

    front_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        front_pc_topic_, qos_profile, (cb_front));
    right_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        right_pc_topic_, qos_profile, (cb_right));
    left_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        left_pc_topic_, qos_profile, (cb_left));

    left_cloud_.reset(new pcl::PointCloud<PointXYZIRT>());
    right_cloud_.reset(new pcl::PointCloud<PointXYZIRT>());
    front_cloud_.reset(new pcl::PointCloud<PointXYZIRT>());
    merged_cloud_.reset(new pcl::PointCloud<PointXYZIRT>());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create a timer that calls the timerCallback function at a fixed rate
    timer_ = this->create_wall_timer(std::chrono::milliseconds(40), std::bind(&PointCloudCombiner::timerCallback, this));
    num_left_ = num_right_ = num_front_ = 0;
  }

void PointCloudCombiner::timerCallback() {
    publishCombinedPointCloud();
}

pcl::PointCloud<PointXYZIRT>::Ptr PointCloudCombiner::getTargetCloud(const std::string& topic) {
    if (topic == front_pc_topic_) {
        return front_cloud_;
    } else if (topic == right_pc_topic_) {
        return right_cloud_;
    } else if (topic == left_pc_topic_) {
        return left_cloud_;
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown topic: %s", topic.c_str());
        return nullptr;
    }
}

void PointCloudCombiner::cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg, const std::string topicName) {
    auto start_time = std::chrono::high_resolution_clock::now();
    latest_time_ = cloudMsg->header.stamp;

    auto targetCloud = getTargetCloud(topicName);
    targetCloud->clear();

    // Transform the point cloud to front_lidar_frame_
    if (topicName != front_pc_topic_)
        pcl_ros::transformPointCloud(front_lidar_frame_, *cloudMsg, *cloudMsg, *tf_buffer_);
    pcl::fromROSMsg(*cloudMsg, *targetCloud);
} 

// Publish the combined point cloud
void PointCloudCombiner::publishCombinedPointCloud() {
    // Merge the point clouds
    merged_cloud_->clear();
    *merged_cloud_ += *left_cloud_;
    *merged_cloud_ += *right_cloud_;
    *merged_cloud_ += *front_cloud_;

    sensor_msgs::msg::PointCloud2 combined_msg;
    pcl::toROSMsg(*merged_cloud_, combined_msg);
    combined_msg.header.stamp = latest_time_;
    combined_msg.header.frame_id = front_lidar_frame_;
    combined_publisher_->publish(combined_msg);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudCombiner>("point_cloud_combiner"));
  rclcpp::shutdown();
  return 0;
}