# Intel-RealSense

L515-LiDAR Camera is the depth camera that use LiDAR depth technology for indoor environment with ideal range 0.25 m to 9 m. The depth technology is LiDAR with FOV 70° x 55°(±3°), Minimum depth distance(Min-Z) at max resolution: ~25 cm, Depth Accuracy: ~5 mm to ~14 mm thru 9 m2, Depth frame rate: 30 fps, and Depth output resolution: Up to 1024 × 768. Additionally, L515 is become with RGB camera that have RGB frame resolution: 1920 × 1080, RGB sensor FOV (H × V): 70° × 43° (±3°), RGB frame rate: 30 fps, and RGB sensor resolution: 2 MP. this product is widely use in logistics industrial, robotics or 3D scanning and ETC.

<p align="center">
  <img src=Images/lidar_camera_gallery_6.jpg alt="L515" width="400">
</p>

And this project will serve you, how to use the intel-realsense products with ROS2

# Getting Started

## Prerequisites

To use Intel-Realsense products you need to install this prerequisites first before using the products.

⚠️ **Warning:** make sure you use ubuntu 22.04 with ROS2 humble.

### Lib-RealSense package

You can see the all information about lib-realsense in this repository: https://github.com/IntelRealSense/librealsense

To use Intel-RealSense products you need to install this package with specific version of it because Intel will delete the products that end of support for make package lighter.

So in this repository will guide you how to install this package that can use with both L515 and T265 simultaneously.

#### Installing the packages:

* Register the server's public key:

    ```bs
    sudo mkdir -p /etc/apt/keyrings
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
    ```

* Make sure apt HTTPS support is installed: `sudo apt-get install apt-transport-https`
* Add the server to the list of repositories:

    ```bs
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    sudo tee /etc/apt/sources.list.d/librealsense.list
    sudo apt-get update
    ```
    
* Install the libraries

    ```bs
    sudo apt-get install librealsense2-udev-rules=2.53.1-0~realsense0.8251
    sudo apt-get install librealsense2=2.53.1-0~realsense0.8251
    sudo apt-get install librealsense2-dev=2.53.1-0~realsense0.8251
    sudo apt-get install librealsense2-dbg=2.53.1-0~realsense0.8251
    sudo apt-get install librealsense2-dkms
    ```

#### Uninstalling the Packages:

**Important** Removing Debian package is allowed only when no other installed packages directly refer to it.

In this project you can uninstall the package via this bash script below.

```bs
sudo apt remove librealsense2-dkms
sudo apt remove librealsense2-dbg
sudo apt remove librealsense2-dev
sudo apt remove librealsense2
sudo apt remove librealsense2-udev-rules
sudo apt autoremove
sudo apt autoclean
```

ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true publish_tf:=true unite_imu_method:=2 enable_gyro:=true enable_accel:=true enable_infra2:=true enable_infra1:=true




ros2 launch rtabmap_launch rtabmap.launch.py     rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3"     depth_topic:=/camera/aligned_depth_to_color/image_raw     rgb_topic:=/camera/color/image_raw     camera_info_topic:=/camera/color/camera_info     approx_sync:=false     wait_imu_to_init:=true     imu_topic:=/rtabmap/imu frame_id:=camera_link  depth_camera_info_topic:=/camera/depth/camera_info qos:=0


ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -p use_mag:=false --ros-args -p publish_tf:=true --ros-args -p world_frame:="enu" --ros-args --remap /imu/data_raw:=/camera/imu --ros-args --remap /imu/data:=/rtabmap/imu --ros-args -p fixed_frame:=camera_link
