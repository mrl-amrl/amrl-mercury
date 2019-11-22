sudo cp *.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger