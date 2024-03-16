#include "mergeClouds.hpp"
#include <chrono>
#include <functional>

using namespace std::chrono;
using namespace std;

PointCloudCombiner::PointCloudCombiner(const std::string &name)
    : Node(name) {
  declareAndGetParameters();

  createPublishers();

  createSubscribers();

  initializePointcloudsAndBuffers();

  setupTransforms();

  createTimer();

  setupFilters();
}

void PointCloudCombiner::declareAndGetParameters() {

  declare_param(this, PARAM_MERGED_PC_TOPIC, merged_pc_topic_, "/luminar_merged");
  declare_param(this, PARAM_FRONT_PC_TOPIC, front_pc_topic_, "/luminar_front/points");
  declare_param(this, PARAM_LEFT_PC_TOPIC, left_pc_topic_, "/luminar_left/points");
  declare_param(this, PARAM_RIGHT_PC_TOPIC, right_pc_topic_, "/luminar_right/points");
  declare_param(this, PARAM_REAR_PC_TOPIC, rear_pc_topic_, "/luminar_rear/points");

  declare_param(this, PARAM_FRONT_LIDAR_FRAME, front_lidar_frame_, "luminar_front");
  declare_param(this, PARAM_LEFT_LIDAR_FRAME, left_lidar_frame_, "luminar_left");
  declare_param(this, PARAM_RIGHT_LIDAR_FRAME, right_lidar_frame_, "luminar_right");
  declare_param(this, PARAM_REAR_LIDAR_FRAME, rear_lidar_frame_, "luminar_rear");

  declare_param(this, PARAM_PUB_FREQ, pub_freq_, 12.0);
  declare_param(this, PARAM_CROP_BOX_SIZE, crop_size_, 3.0);
  declare_param(this, PARAM_VOXEL_RES, voxel_res_, 0.3);
  declare_param(this, PARAM_USE_4_LIADR, use_4_lidar_, true);

  topics_ = {left_pc_topic_, right_pc_topic_, front_pc_topic_};
  if (use_4_lidar_)
    topics_.push_back(rear_pc_topic_);
}

void PointCloudCombiner::createPublishers() {
  rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))
                                .best_effort()
                                .durability_volatile();

  // Create publisher for the combined point cloud
  combined_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(merged_pc_topic_, qos_profile);
}

void PointCloudCombiner::createSubscribers() {
  rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))
                                .best_effort()
                                .durability_volatile();

  // Helper function to create subscribers
  auto create_subscriber = [this, &qos_profile](
                               const std::string &topic_name,
                               const rclcpp::CallbackGroup::SharedPtr &cb_group,
                               const std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr)> &callback) {
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group;
    return this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_name, qos_profile, callback, sub_options);
  };

  // Create callback groups
  auto left_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto right_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto front_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto rear_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create subscribers with their respective callback functions
  left_subscriber_ = create_subscriber(
      left_pc_topic_, left_cb_group,
      std::bind(&PointCloudCombiner::callback, this, std::placeholders::_1, left_pc_topic_));

  right_subscriber_ = create_subscriber(
      right_pc_topic_, right_cb_group,
      std::bind(&PointCloudCombiner::callback, this, std::placeholders::_1, right_pc_topic_));

  front_subscriber_ = create_subscriber(
      front_pc_topic_, front_cb_group,
      std::bind(&PointCloudCombiner::callback, this, std::placeholders::_1, front_pc_topic_));

  rear_subscriber_ = create_subscriber(
      rear_pc_topic_, rear_cb_group,
      std::bind(&PointCloudCombiner::callback, this, std::placeholders::_1, rear_pc_topic_));
}

void PointCloudCombiner::initializePointcloudsAndBuffers() {
  // Initialize point clouds
  for (auto &topic : topics_) {
      // pointclouds_[left_pc_topic_] = std::make_shared<pcl::PointCloud<PointType>>();
    pcl::PointCloud<PointType>::Ptr new_cloud (new pcl::PointCloud<PointType>());
    pointclouds_[topic] = new_cloud;
  }
  merged_cloud_.reset(new pcl::PointCloud<PointType>());

  // Initialize circular buffers with empty point clouds
  buff_capacity_ = 3;
  for (auto &topic : topics_) {
    circular_buffers_[topic].set_capacity(buff_capacity_);
    for (int i = 0; i < buff_capacity_; ++i) {
      // circular_buffers_[topic].push_back(std::make_pair(std::make_shared<pcl::PointCloud<PointType>>(), rclcpp::Time()));
      pcl::PointCloud<PointType>::Ptr new_cloud (new pcl::PointCloud<PointType>());

      circular_buffers_[topic].push_back(new_cloud);
    }
  }
}

void PointCloudCombiner::setupTransforms() {
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Helper function to get transform between frames
  auto get_transform = [this](const std::string &from_frame, const std::string &to_frame) {
    geometry_msgs::msg::TransformStamped trans;
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    try {
      int timeout_ms_ = 50;
      while (!tf_buffer_->canTransform(from_frame, to_frame, tf2::TimePointZero, 0ms) && rclcpp::ok()) {
        RCLCPP_INFO(
            get_logger(), "waiting %d ms for %s->%s transform to become available",
            timeout_ms_, from_frame.c_str(), to_frame.c_str());
        rclcpp::sleep_for(std::chrono::milliseconds(timeout_ms_));
      }
      trans = tf_buffer_->lookupTransform(from_frame, to_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(
          this->get_logger(), "!!Could not transform between lidar frames: %s", ex.what());
      return transform_matrix;
    }

    Eigen::Quaternionf q(trans.transform.rotation.w, trans.transform.rotation.x,
                         trans.transform.rotation.y, trans.transform.rotation.z);
    Eigen::Vector3f p(trans.transform.translation.x, trans.transform.translation.y,
                      trans.transform.translation.z);
    transform_matrix.block(0, 0, 3, 3) = q.toRotationMatrix();
    transform_matrix.block(0, 3, 3, 1) = p;
    return transform_matrix;
  };

  // Get transforms and store them in a map
  transforms_[left_pc_topic_] = get_transform(front_lidar_frame_, left_lidar_frame_);
  transforms_[right_pc_topic_] = get_transform(front_lidar_frame_, right_lidar_frame_);
  if (use_4_lidar_)
    transforms_[rear_pc_topic_] = get_transform(front_lidar_frame_, rear_lidar_frame_);
  transforms_[front_pc_topic_] = get_transform(front_lidar_frame_, front_lidar_frame_);
}

void PointCloudCombiner::createTimer() {
  int timer_dur = int(1000 / pub_freq_);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(timer_dur),
      std::bind(&PointCloudCombiner::publishCombinedPointcloud, this));
}

void PointCloudCombiner::setupFilters() {
  // Setup crop box filter
  crop_.setNegative(true);
  crop_.setMin(Eigen::Vector4f(-crop_size_, -crop_size_, -crop_size_, 1.0));
  crop_.setMax(Eigen::Vector4f(crop_size_, crop_size_, crop_size_, 1.0));

  // Setup voxel grid filter
  voxel_.setLeafSize(voxel_res_, voxel_res_, voxel_res_);
}

void PointCloudCombiner::callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg, const std::string &topic_name) {
  // Get the corresponding point cloud and circular buffer
  auto &point_cloud = pointclouds_[topic_name];
  auto &circular_buffer = circular_buffers_[topic_name];

  // Clear the point cloud
  point_cloud->clear();

  // Transform the point cloud to front_lidar_frame_
  pcl::fromROSMsg(*cloudMsg, *point_cloud);

  // Process the point cloud and add it to the circular buffer
  processPointcloud(point_cloud, circular_buffer, transforms_[topic_name], topic_name, cloudMsg->header.stamp);
}

void PointCloudCombiner::processPointcloud(
    pcl::PointCloud<PointType>::Ptr &cloud_in,
    boost::circular_buffer<pcl::PointCloud<PointType>::Ptr> &buff,
    // boost::circular_buffer<std::pair<pcl::PointCloud<PointType>::Ptr, rclcpp::Time>> &buff,
    const Eigen::Matrix4f &transform,
    const std::string &topicName,
    const rclcpp::Time &timestamp) {
  // Create a new point cloud for the filtered result
  pcl::PointCloud<PointType>::Ptr new_cloud (new pcl::PointCloud<PointType>());

  // Crop Box Filter
  crop_.setInputCloud(cloud_in);
  crop_.filter(*new_cloud);


  // // Voxel Grid Filter
  voxel_.setInputCloud(new_cloud);
  voxel_.filter(*new_cloud);

  // Transform the point cloud

  pcl::transformPointCloud(*new_cloud, *new_cloud, transform);

  if (topicName == front_pc_topic_) {
    front_rec_ = true;
    latest_time_ = timestamp;
  }

  // Add the transformed cloud to the circular buffer
  buff.push_back(new_cloud);
  // buff.push_back(std::make_pair(new_cloud, timestamp));
}

void PointCloudCombiner::publishCombinedPointcloud() {
  if (front_rec_) {
    // Merge the point clouds from the circular buffers
    merged_cloud_->clear();
    double avg_time = 0;
    for (auto &topic : topics_) {
      auto entry = circular_buffers_[topic].back();
      // *merged_cloud_ += *(entry.first);
      *merged_cloud_ += *(entry);
      // avg_time += entry.second.seconds();
    }
    // avg_time /= topics_.size();

    sensor_msgs::msg::PointCloud2 combined_msg;
    pcl::toROSMsg(*merged_cloud_, combined_msg);
    combined_msg.header.stamp = latest_time_;
    // combined_msg.header.stamp = rclcpp::Time(avg_time);
    combined_msg.header.frame_id = front_lidar_frame_;
    combined_publisher_->publish(combined_msg);
  }


}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudCombiner>("point_cloud_combiner");
  // rclcpp::executors::MultiThreadedExecutor executor;
  // executor.add_node(node);
  // executor.spin();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}