#!/bin/bash

echo ""
echo "This script copies a udev rule to /etc to facilitate bringing"
echo "up the astra usb connection as /dev/astra*"
echo ""

sudo cp `rospack find elp-stereo-synchronized-ros-pkg`/99-elp-stereo-sync-camera.rules /etc/udev/rules.d


echo ""
echo "Restarting udev"
echo ""

sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
