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
    this->declare_parameter(PARAM_REAR_PC_TOPIC, "/luminar_rear_points" );

    this->declare_parameter(PARAM_FRONT_LIDAR_FRAME, "luminar_front");
    this->declare_parameter(PARAM_LEFT_LIDAR_FRAME, "luminar_left");
    this->declare_parameter(PARAM_RIGHT_LIDAR_FRAME, "luminar_right");
    this->declare_parameter(PARAM_REAR_LIDAR_FRAME, "luminar_rear");

    this->declare_parameter(PARAM_PUB_FREQ, 30.0);

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
    if (!this->get_parameter(PARAM_REAR_PC_TOPIC, rear_pc_topic_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_REAR_PC_TOPIC.c_str());
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
    if (!this->get_parameter(PARAM_REAR_LIDAR_FRAME, rear_lidar_frame_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_REAR_LIDAR_FRAME.c_str());
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

    this->rear_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto rear_sub_opt = rclcpp::SubscriptionOptions();
    rear_sub_opt.callback_group = this->rear_cb_group;
    rear_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(rear_pc_topic_, qos_profile,
      std::bind(&PointCloudCombiner::callbackRear, this, std::placeholders::_1), rear_sub_opt);

    left_cloud_.reset(new pcl::PointCloud<PointType>());
    right_cloud_.reset(new pcl::PointCloud<PointType>());
    front_cloud_.reset(new pcl::PointCloud<PointType>());
    rear_cloud_.reset(new pcl::PointCloud<PointType>());

    merged_cloud_.reset(new pcl::PointCloud<PointType>());

    left_cloud_filtered_.reset(new pcl::PointCloud<PointType>());
    right_cloud_filtered_.reset(new pcl::PointCloud<PointType>());
    front_cloud_filtered_.reset(new pcl::PointCloud<PointType>());
    rear_cloud_filtered_.reset(new pcl::PointCloud<PointType>());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    geometry_msgs::msg::TransformStamped f2l, f2r, f2rr;
    try {
        int timeout_ms_ = 50;
        while(!tf_buffer_->canTransform(front_lidar_frame_, left_lidar_frame_, tf2::TimePointZero, 0ms) && rclcpp::ok()) {
            RCLCPP_INFO(
                    get_logger(), "waiting %d ms for %s->%s transform to become available",
                    timeout_ms_, front_lidar_frame_.c_str(), left_lidar_frame_.c_str());
            rclcpp::sleep_for(std::chrono::milliseconds(timeout_ms_));
        }
        RCLCPP_INFO(get_logger(), "transform available");
        f2l = tf_buffer_->lookupTransform(
            front_lidar_frame_,
            left_lidar_frame_,
            tf2::TimePointZero
        );
        f2r = tf_buffer_->lookupTransform(
            front_lidar_frame_, 
            right_lidar_frame_,
            tf2::TimePointZero
        );
        f2rr = tf_buffer_->lookupTransform(
            front_lidar_frame_, 
            rear_lidar_frame_,
            tf2::TimePointZero
        );
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
        this->get_logger(), "!!Could not transform between lidar frames: %s", ex.what());
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

    front_2_rear_ = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf q3(f2rr.transform.rotation.w, f2rr.transform.rotation.x, f2rr.transform.rotation.y, f2rr.transform.rotation.z);
    Eigen::Vector3f p3(f2rr.transform.translation.x, f2rr.transform.translation.y, f2rr.transform.translation.z);
    front_2_rear_.block(0, 0, 3, 3) = q3.toRotationMatrix();
    front_2_rear_.block(0, 3, 3, 1) = p3;

    // Create a timer that calls the timerCallback function at a fixed rate
    int timer_dur = int(1000/pub_freq_);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_dur), std::bind(&PointCloudCombiner::timerCallback, this));
  
    // setup filter
    // this->sor_.setMeanK(50);
    // this->sor_.setStddevMulThresh (1.0);

    // pcl::ConditionAnd<PointType>::Ptr intensity_cond (new pcl::ConditionAnd<PointType> ());
    // intensity_cond->addComparison(pcl::FieldComparison<PointType>::ConstPtr (
    //     new pcl::FieldComparison<PointType> ("ring", pcl::ComparisonOps::GT, 0.01))
    // );
    // intensity_cond->addComparison(pcl::FieldComparison<PointType>::ConstPtr (
    //     new pcl::FieldComparison<PointType> ("intensity", pcl::ComparisonOps::GT, 0.5))
    // );
    // this->condrem_.setCondition(intensity_cond);
  }

void PointCloudCombiner::timerCallback() {
    publishCombinedPointCloud();
}

pcl::PointCloud<PointType>::Ptr PointCloudCombiner::getTargetCloud(const std::string& topic) {
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

void PointCloudCombiner::filter(pcl::PointCloud<PointType>::Ptr cloud_in, pcl::PointCloud<PointType>::Ptr cloud_filtered) {
    // this->sor_.setInputCloud(cloud);
    // this->sor_.filter(*cloud);
    // this->condrem_.setInputCloud(cloud);
    // this->condrem_.filter(*cloud);
    cloud_filtered->clear();
    // uint8_t threshold = 128;
    for (const auto& point: cloud_in->points) {
        // if (point.existence_probability_percent > threshold) {
        //     cloud_filtered->push_back(point);
        // }
        cloud_filtered->push_back(point);
    }
}

void PointCloudCombiner::callbackRear(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg) {
    rear_cloud_->clear();

    // Transform the point cloud to front_lidar_frame_
    pcl::fromROSMsg(*cloudMsg, *rear_cloud_);
    filter(rear_cloud_, rear_cloud_filtered_);
    pcl::transformPointCloud(*rear_cloud_filtered_, *rear_cloud_filtered_, front_2_rear_);

}


void PointCloudCombiner::callbackLeft(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg) {
    auto start = high_resolution_clock::now();

    left_recv_ = true;
    left_cloud_->clear();

    // Transform the point cloud to front_lidar_frame_
    pcl::fromROSMsg(*cloudMsg, *left_cloud_);

    // printf("cloud had %d points\n", left_cloud_->points.size());
    filter(left_cloud_, left_cloud_filtered_);
    // printf("cloud has %d points\n", left_cloud_filtered_->points.size());

    pcl::transformPointCloud(*left_cloud_filtered_, *left_cloud_filtered_, front_2_left_);

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    // std::cout << "left " << duration.count() << std::endl;
}

void PointCloudCombiner::callbackRight(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg) {
    right_recv_ = true;
    right_cloud_->clear();

    // Transform the point cloud to front_lidar_frame_
    pcl::fromROSMsg(*cloudMsg, *right_cloud_);
    filter(right_cloud_, right_cloud_filtered_);
    pcl::transformPointCloud(*right_cloud_filtered_, *right_cloud_filtered_, front_2_right_);

}

void PointCloudCombiner::callbackFront(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg) {
    front_recv_ = true;
    front_cloud_->clear();
    latest_time_ = cloudMsg->header.stamp;

    pcl::fromROSMsg(*cloudMsg, *front_cloud_);
    
    // double min_i, max_i;
    // double mean_i;
    // min_i = INFINITY;
    // max_i = -INFINITY;
    // mean_i = 0;
    // for (int i = 0; i < front_cloud_->points.size(); i++) {
    //     float val = (float) front_cloud_->points[i].existence_probability_percent;

    //     double prob = val/255.0;
    //     std::cout << prob << " " << val << std::endl;
    //     min_i = std::min(min_i, prob);
    //     max_i = std::max(max_i, prob);
    //     mean_i += prob;
    // }
    // mean_i /= front_cloud_->points.size();
    // printf("mean, max, min: %f %f %f \n", mean_i, max_i, min_i);

    filter(front_cloud_, front_cloud_filtered_);
}

void PointCloudCombiner::cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg, const std::string topicName) {
    // pcl::PointCloud<PointType>::Ptr targetCloud;
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
    merged_cloud_->clear();
    *merged_cloud_ += *left_cloud_filtered_;
    *merged_cloud_ += *right_cloud_filtered_;
    *merged_cloud_ += *front_cloud_filtered_;
    *merged_cloud_ += *rear_cloud_filtered_;

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