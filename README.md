# Intel-RealSense

This project is use 2 devices from Intel-RealSense that are L515 and T265. So this repository will serve you, how to use the intel-realsense products with ROS2

# Getting Started

## Prerequisites

To use Intel-Realsense products you need to install this prerequisites first before using the products.

⚠️ **Warning:** make sure you use ubuntu 22.04 with ROS2 humble.

### Lib-RealSense package

You can see the all information about lib-realsense in this repository: https://github.com/IntelRealSense/librealsense

To use Intel-RealSense products you need to install this package with specific version of it because Intel will delete the products that end of support for make package lighter.

So in this repository will guide you how to install this package that can use with both L515 and T265 simultaneously.

* Installing the packages:

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

* Uninstalling the Packages:

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

### realsense-ros package

In this project we use ROS2-humble to be our middleware. So we need to install ros-wrapper for communicate between realsense products and ROS2.

So in this repository will guide you how to install this package that can use with both L515 and T265 simultaneously.

* Installing the packages:

    **Make sure you are in** `<your workspace>/src`

    * Clone realsense-ros package to src folder.

        ```bs
        git clone https://github.com/IntelRealSense/realsense-ros.git -b 4.51.1
        ```

    * Build and source the package.

        ```bs
        cd ~/<your workspace>/
        colcon build
        source install/setup.bash
        ```

**After install** you can test the lib-realsense2 via this bash script below and connect the realsense product to your computer.

```bs
ros2 launch realsense2_camera rs_launch.py
```

If in the terminal say "Find realsense device" it's mean you completely finish install the package.

# Intel-Realsense-L515-LiDAR-Camera

The Intel® RealSense™ LiDAR Camera L515 is a compact, high-resolution depth camera designed for precise indoor 3D scanning and depth sensing applications. Utilizing solid-state LiDAR technology, it offers accurate depth measurements with low power consumption.

**Key Features:**

* Depth Sensing: Provides depth measurements ranging from 0.25 meters to 9 meters, with a field of view of 70° x 55°.

* High Resolution: Delivers depth output at up to 1024 x 768 resolution at 30 frames per second, and RGB images at 1920 x 1080 resolution.

* Compact Design: Measures 61 mm in diameter and 26 mm in height, weighing approximately 100 grams, making it suitable for integration into various devices.

* Low Power Consumption: Operates at less than 3.5 watts during depth streaming, enhancing energy efficiency.

<p align="center">
  <img src=Images/lidar_camera_gallery_6.jpg alt="L515" width="400">
</p>

## Usages

To start the camera node in ROS:

```bs
ros2 launch realsense2_camera rs_launch.py
```

But if you want to run/launch camera with specific parameters:

```bs
ros2 run realsense2_camera realsense2_camera_node --ros-args -p enable_color:=false -p spatial_filter.enable:=true -p temporal_filter.enable:=true
```

or, with a launch file:

```bs
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 pointcloud.enable:=true
```

We recommend you to use launch file for start the camera node with or without parameters.

more information in https://github.com/IntelRealSense/realsense-ros/tree/4.51.1

# Intel-Realsense-T265-LiDAR-Camera

The Intel® RealSense™ Tracking Camera T265 is a stand-alone, inside-out tracking device designed to provide high-performance guidance and navigation for various applications, including robotics, drones, and augmented reality (AR) systems. It utilizes proprietary Visual Inertial Odometry (VIO) technology to deliver precise six-degrees-of-freedom (6DoF) tracking without relying on external sensors or GPS.

**Key Features:**

* Visual-Inertial Tracking: The T265 combines inputs from dual fisheye cameras and an Inertial Measurement Unit (IMU) to perform simultaneous localization and mapping (SLAM), enabling accurate position and orientation tracking.

* Embedded Processing: Powered by the Intel® Movidius™ Myriad™ 2 Vision Processing Unit (VPU), the T265 processes tracking data internally, reducing the computational load on the host system.

* Low Power Consumption: Designed for efficiency, the T265 operates at approximately 1.5 watts, making it suitable for battery-powered devices like drones and mobile robots.

* Compact Design: With dimensions of approximately 108 mm x 24.5 mm x 12.5 mm and weighing around 55 grams, the T265 is easy to integrate into various platforms.

<p align="center">
  <img src=Images/intel_realsense_tracking_camera_photo_angle_1_675x450-300x200.png alt="L515" width="400">
</p>

## Usages

To start the camera node in ROS:

```bs
ros2 run realsense2_camera realsense2_camera_node --ros-args -p enable_pose:=true -p device_type:=t265
```

more information in https://github.com/IntelRealSense/realsense-ros/tree/4.51.1

# Start two cameras in same time

To start 2 cameras(L515+T265) in same time you need to use specific bash script to start the cameras:

```bs
ros2 launch realsense2_camera rs_d400_and_t265_launch.py
```

or with specific parameters:

```bs
ros2 launch realsense2_camera rs_d400_and_t265_launch.py enable_fisheye12:=true enable_fisheye22:=true
```

more information in https://github.com/IntelRealSense/realsense-ros/tree/4.51.1

# rtabmap with L515

If you want to use L515 with rtabmap you can run this bash script below for launch the camera with rtabmap

* Terminal 1(Camera node):

```bs
ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true publish_tf:=true unite_imu_method:=2 enable_gyro:=true enable_accel:=true enable_infra2:=true enable_infra1:=true
```

* Terminal 2(rtapmap):

```bs
ros2 launch rtabmap_launch rtabmap.launch.py     rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3"     depth_topic:=/camera/aligned_depth_to_color/image_raw     rgb_topic:=/camera/color/image_raw     camera_info_topic:=/camera/color/camera_info     approx_sync:=false     wait_imu_to_init:=true     imu_topic:=/rtabmap/imu frame_id:=camera_link  depth_camera_info_topic:=/camera/depth/camera_info qos:=0
```

* Terminal 3(imu filter):

```bs
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -p use_mag:=false --ros-args -p publish_tf:=true --ros-args -p world_frame:="enu" --ros-args --remap /imu/data_raw:=/camera/imu --ros-args --remap /imu/data:=/rtabmap/imu --ros-args -p fixed_frame:=camera_link
```