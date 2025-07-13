#!/bin/bash

# give exec permission -> sudo chmod 744 run_leapos.sh
set -e

# Clean ports - Make sure this file destination is consistent!
python3 ~/Leapfrog_FlightController/flush_serial.py

# Source ROS2
echo "Sourcing ROS2"
source /opt/ros/humble/setup.bash
source ~/Leapfrog_FlightController/install/setup.bash

# Source Install path for packages
echo "Loading package path"
. ~/Leapfrog_FlightController/install/local_setup.bash

# Run packages
#echo "Running Sensor Package"
#ros2 launch sensors sensors.launch.py
echo "Running LEAPOS Package"
ros2 launch ~/Leapfrog_FlightController/src/launch/vehicle.launch.py