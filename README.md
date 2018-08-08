# elp-synchronized-stereo-camera-ros-pkg

A ROS driver for the ELP Dual Lens synchronized stereo camera.

## Documentation

Documentation for original driver (non-synchronized) is available on the ROS wiki: http://wiki.ros.org/elp_stereo_camera

This has been tested on the ELP model: ELP-960P2CAM-V90-VC

## Install from Source

These setup instructions assume that you have Ubuntu 16.04, have ROS Kinetic installed, and have a catkin workspace at `~/catkin_ws`. If you don't, follow the [Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) tutorial before proceeding. The following specific packages should be installed if they aren't already:

```bash
sudo apt-get update
sudo apt-get install ros-kinetic-ros-base ros-kinetic-image-common ros-kinetic-image-transport-plugins ros-kinetic-image-pipeline ros-kinetic-usb-cam
```

Now you can install the elp_stereo_camera package:

```bash
git clone https://github.com/mdkennedy3/elp-synchronized-stereo-camera-ros-pkg  ~/catkin_ws/src/elp_stereo_camera
```
After compiling the package, then setup the udev rules:
```bash
# setup udev rules (navigate to package)
sudo ./scripts/create_udev_rules.sh
```

## How to Run

To simply generate seperate stereo images run the launch file
```bash
roslaunch elp_stereo_synchronized_ros_pkg elp_stereo_camera.launch 
```
(images can easily be viewed using [rqt image viewer](http://wiki.ros.org/rqt_image_view))

To see disparity run 
```bash
roslaunch elp_stereo_synchronized_ros_pkg disparity.launch
```
when visualize_disparity argument is true, a image\_view disparity window will appear


To see the point cloud
```bash
roslaunch elp_stereo_synchronized_ros_pkg point_cloud_rviz_disparity.launch
```














