# Intel-RealSense

This repository is created to serve as the guide for the LIDAR Camera L515.

ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true publish_tf:=true unite_imu_method:=2 enable_gyro:=true enable_accel:=true enable_infra2:=true enable_infra1:=true




ros2 launch rtabmap_launch rtabmap.launch.py     rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3"     depth_topic:=/camera/aligned_depth_to_color/image_raw     rgb_topic:=/camera/color/image_raw     camera_info_topic:=/camera/color/camera_info     approx_sync:=false     wait_imu_to_init:=true     imu_topic:=/rtabmap/imu frame_id:=camera_link  depth_camera_info_topic:=/camera/depth/camera_info qos:=0


ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -p use_mag:=false --ros-args -p publish_tf:=true --ros-args -p world_frame:="enu" --ros-args --remap /imu/data_raw:=/camera/imu --ros-args --remap /imu/data:=/rtabmap/imu --ros-args -p fixed_frame:=camera_link
