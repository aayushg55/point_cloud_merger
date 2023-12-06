# Lidar Point Cloud Merger
Combines the point clouds from 3 topics and publishes the merged point cloud. 


## Steps
Prerequisites: Ubuntu 20.04/ ROS2 Galactic
The lidar topics and frames should be configured in config.yaml
1. Clone the repository into ~/ros2_ws/src
2. Build the package:
   ```bash
   cd ~/pc_merger/src
   colcon_build
   ```
3. Run the package:
   ```bash
   source install/setup.bash
   ros2 launch pc_merger run.launch.py
   ```
