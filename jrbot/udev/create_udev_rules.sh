#!/bin/bash
sudo cp rplidar.rules  /etc/udev/rules.d/
sudo cp jrbot_base.rules  /etc/udev/rules.d/
sudo service udev reload
sudo service udev restart
sudo udevadm control --reload && sudo udevadm trigger
