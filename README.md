# elp-synchronized-stereo-camera-ros-pkg

A ROS driver for the ELP Dual Lens synchronized stereo camera.

## Documentation

Documentation is available on the ROS wiki: http://wiki.ros.org/elp_stereo_camera

## Install from Source

These setup instructions assume that you have Ubuntu 16.04, have ROS Kinetic installed, and have a catkin workspace at `~/catkin_ws`. If you don't, follow the [Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) tutorial before proceeding. The following specific packages should be installed if they aren't already:

```bash
sudo apt-get update
sudo apt-get install ros-kinetic-ros-base ros-kinetic-image-common ros-kinetic-image-transport-plugins ros-kinetic-image-pipeline ros-kinetic-usb-cam
```

Now you can install the elp_stereo_camera package:

```bash
git clone https://github.com/mdkennedy3/elp-synchronized-stereo-camera-ros-pkg  ~/catkin_ws/src/elp_stereo_camera

# setup udev rules (navigate to package)
sudo ./scripts/create_udev_rules.sh

```






















