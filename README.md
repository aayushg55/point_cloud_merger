# Lidar Point Cloud Merger
Combines the point clouds from 3 topics and publishes the merged point cloud. 


## Steps
Prerequisites: Ubuntu 20.04/ ROS2 Galactic
The lidar topics and frames should be configured in config.yaml
1. Install pcl-ros
   ```bash
   sudo apt-get install ros-galactic-pcl-ros
   ```
3. Clone the repository
4. Build the package:
   ```bash
   cd ~/point_cloud_merger/ros2_ws
   colcon_build
   ```
5. Run the package:
   ```bash
   source install/setup.bash
   ros2 launch pc_merger run.launch.py
   ```
