#!/bin/bash

echo ""
echo "This script copies a udev rule to /etc to facilitate bringing"
echo "up the ancube-kit usb connection."
echo ""

sudo cp `rospack find ancubekit_bringup`/99-ancubekit.rules /etc/udev/rules.d/

echo ""
echo "Reload rules"
echo ""
sudo udevadm control --reload-rules
sudo udevadm trigger