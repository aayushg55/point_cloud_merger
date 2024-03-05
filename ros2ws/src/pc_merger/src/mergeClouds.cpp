#include "mergeClouds.hpp"
#include <chrono>

using namespace std::chrono;
PointCloudCombiner::PointCloudCombiner(const std::string &name) 
    : Node(name)
{
    // Declare parameters
    this->declare_parameter(PARAM_MERGED_PC_TOPIC, "/luminar_merged_points" );
    this->declare_parameter(PARAM_FRONT_PC_TOPIC, "/luminar_front_points" );
    this->declare_parameter(PARAM_LEFT_PC_TOPIC, "/luminar_left_points" );
    this->declare_parameter(PARAM_RIGHT_PC_TOPIC, "/luminar_right_points" );

    this->declare_parameter(PARAM_FRONT_LIDAR_FRAME, "luminar_front");
    this->declare_parameter(PARAM_LEFT_LIDAR_FRAME, "luminar_left");
    this->declare_parameter(PARAM_RIGHT_LIDAR_FRAME, "luminar_right");

    this->declare_parameter(PARAM_PUB_FREQ, 60.0);

    // // Read parameters
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
    if (!this->get_parameter(PARAM_PUB_FREQ, pub_freq_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_PUB_FREQ.c_str());
    }

    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))  // Keep the last 10 messages
        .best_effort()            // Use best-effort reliability
        .durability_volatile();
    // Create publishers for the combined point cloud
    combined_publisher_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(merged_pc_topic_, 1);

    // // Create subscribers for the individual point clouds
    this->left_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto left_sub_opt = rclcpp::SubscriptionOptions();
    left_sub_opt.callback_group = this->left_cb_group;
    left_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(left_pc_topic_, qos_profile,
      std::bind(&PointCloudCombiner::callbackLeft, this, std::placeholders::_1), left_sub_opt);

    this->front_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto front_sub_opt = rclcpp::SubscriptionOptions();
    front_sub_opt.callback_group = this->front_cb_group;
    front_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(front_pc_topic_, qos_profile,
      std::bind(&PointCloudCombiner::callbackFront, this, std::placeholders::_1), front_sub_opt);

    this->right_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto right_sub_opt = rclcpp::SubscriptionOptions();
    right_sub_opt.callback_group = this->right_cb_group;
    right_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(right_pc_topic_, qos_profile,
      std::bind(&PointCloudCombiner::callbackRight, this, std::placeholders::_1), right_sub_opt);
    // std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg)> cb_front = std::bind(
    //     &PointCloudCombiner::cloudHandler, this, std::placeholders::_1, front_pc_topic_);
    // std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg)> cb_left = std::bind(
    //     &PointCloudCombiner::cloudHandler, this, std::placeholders::_1, left_pc_topic_);
    // std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg)> cb_right = std::bind(
    //     &PointCloudCombiner::cloudHandler, this, std::placeholders::_1, right_pc_topic_);

    // front_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //     front_pc_topic_, qos_profile, (cb_front));
    // right_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //     right_pc_topic_, qos_profile, (cb_right));
    // left_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //     left_pc_topic_, qos_profile, (cb_left));

    left_cloud_.reset(new pcl::PointCloud<PointXYZIRT>());
    right_cloud_.reset(new pcl::PointCloud<PointXYZIRT>());
    front_cloud_.reset(new pcl::PointCloud<PointXYZIRT>());
    merged_cloud_.reset(new pcl::PointCloud<PointXYZIRT>());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    geometry_msgs::msg::TransformStamped f2l, f2r;
    try {
        f2l = tf_buffer_->lookupTransform(
            front_lidar_frame_,
            left_lidar_frame_,
            tf2::TimePointZero,
            5s
        );
        f2r = tf_buffer_->lookupTransform(
            front_lidar_frame_, 
            right_lidar_frame_,
            tf2::TimePointZero,
            5s
        );
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
        this->get_logger(), "Could not transform between lidar frames: %s", ex.what());
        return;
    }
    front_2_left_ = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf q(f2l.transform.rotation.w, f2l.transform.rotation.x, f2l.transform.rotation.y, f2l.transform.rotation.z);
    Eigen::Vector3f p(f2l.transform.translation.x, f2l.transform.translation.y, f2l.transform.translation.z);
    front_2_left_.block(0, 0, 3, 3) = q.toRotationMatrix();
    front_2_left_.block(0, 3, 3, 1) = p;

    front_2_right_ = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf q2(f2r.transform.rotation.w, f2r.transform.rotation.x, f2r.transform.rotation.y, f2r.transform.rotation.z);
    Eigen::Vector3f p2(f2r.transform.translation.x, f2r.transform.translation.y, f2r.transform.translation.z);
    front_2_right_.block(0, 0, 3, 3) = q2.toRotationMatrix();
    front_2_right_.block(0, 3, 3, 1) = p2;

    // Create a timer that calls the timerCallback function at a fixed rate
    int timer_dur = int(1000/pub_freq_);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_dur), std::bind(&PointCloudCombiner::timerCallback, this));
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

void PointCloudCombiner::callbackLeft(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg) {
    auto start = high_resolution_clock::now();

    left_recv_ = true;
    left_cloud_->clear();

    // Transform the point cloud to front_lidar_frame_
    pcl::fromROSMsg(*cloudMsg, *left_cloud_);
    pcl::transformPointCloud(*left_cloud_, *left_cloud_, front_2_left_);
    // pcl_ros::transformPointCloud(front_lidar_frame_, *cloudMsg, *cloudMsg, *tf_buffer_);

    // pcl::fromROSMsg(*cloudMsg, *left_cloud_);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    // std::cout << "left " << duration.count() << std::endl;
}

void PointCloudCombiner::callbackRight(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg) {
    auto start = high_resolution_clock::now();

    right_recv_ = true;
    right_cloud_->clear();

    // Transform the point cloud to front_lidar_frame_
    pcl::fromROSMsg(*cloudMsg, *right_cloud_);
    pcl::transformPointCloud(*right_cloud_, *right_cloud_, front_2_right_);
    // pcl_ros::transformPointCloud(front_lidar_frame_, *cloudMsg, *cloudMsg, *tf_buffer_);

    // pcl::fromROSMsg(*cloudMsg, *right_cloud_);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    // std::cout << "right " << duration.count() << std::endl;
}

void PointCloudCombiner::callbackFront(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg) {
    auto start = high_resolution_clock::now();

    front_recv_ = true;
    front_cloud_->clear();
    latest_time_ = cloudMsg->header.stamp;

    pcl::fromROSMsg(*cloudMsg, *front_cloud_);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    // std::cout << "front " << duration.count() << std::endl;
}

void PointCloudCombiner::cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg, const std::string topicName) {
    // pcl::PointCloud<PointXYZIRT>::Ptr targetCloud;
    // auto targetCloud = getTargetCloud(topicName);
    // targetCloud->clear();

    // Transform the point cloud to front_lidar_frame_
    if (topicName == left_pc_topic_) {
        left_cloud_->clear();
        pcl::fromROSMsg(*cloudMsg, *left_cloud_);
        pcl::transformPointCloud(*left_cloud_, *left_cloud_, front_2_left_);
    }
    else if (topicName == right_pc_topic_) {
        right_cloud_->clear();
        pcl::fromROSMsg(*cloudMsg, *right_cloud_);
        pcl::transformPointCloud(*right_cloud_, *right_cloud_, front_2_right_);
    }
    else {
        front_cloud_->clear();
        pcl::fromROSMsg(*cloudMsg, *front_cloud_);
        latest_time_ = cloudMsg->header.stamp;
    }
    // publishCombinedPointCloud();
    // std::cout << "handled cloud" << std::endl;
    // pcl::fromROSMsg(*cloudMsg, *targetCloud);
}

// Publish the combined point cloud
void PointCloudCombiner::publishCombinedPointCloud() {
    // if (!(left_recv_ && front_recv_ && right_recv_))
    //     return;
    auto start = high_resolution_clock::now();

    // Merge the point clouds
    // std::cout << "starting pub cloud" << std::endl;

    merged_cloud_->clear();
    *merged_cloud_ += *left_cloud_;
    *merged_cloud_ += *right_cloud_;
    *merged_cloud_ += *front_cloud_;
    // std::cout << "pub cloud with " << merged_cloud_->points.size() << std::endl;

    sensor_msgs::msg::PointCloud2 combined_msg;
    pcl::toROSMsg(*merged_cloud_, combined_msg);
    combined_msg.header.stamp = latest_time_;
    combined_msg.header.frame_id = front_lidar_frame_;
    combined_publisher_->publish(combined_msg);

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    // std::cout << duration.count() << std::endl;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);  
  auto node = std::make_shared<PointCloudCombiner>("point_cloud_combiner");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}