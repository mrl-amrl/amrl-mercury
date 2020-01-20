#!/bin/bash
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root" 
   exit 1
fi
apt update
apt install -yq udev libudev-dev python-dev python3-dev python-pip
apt install -yq ros-melodic-dynamic-reconfigure ros-melodic-tf
apt install -yq --install-recommends "jstest*" joystick xboxdrv
pip install pathspec pyyaml PySide2
pip install .
ldconfig
cp udev/*.rules /etc/udev/rules.d/
udevadm control --reload-rules && udevadm trigger