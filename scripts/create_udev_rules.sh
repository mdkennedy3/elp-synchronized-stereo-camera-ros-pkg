#!/bin/bash

echo ""
echo "This script copies the udev rule to /etc to facilitate bringing"
echo "up the elp stereo usb connection as /dev/elp_sync*"
echo ""

sudo cp `rospack find elp_stereo_synchronized_ros_pkg`/99-elp-stereo-sync-camera.rules /etc/udev/rules.d


echo ""
echo "Restarting udev"
echo ""

sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
