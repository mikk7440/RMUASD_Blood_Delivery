#!/bin/bash

# Drone home is currently at the HCA Airport test field.
export PX4_HOME_LAT=55.471301
export PX4_HOME_LON=10.324517
export PX4_HOME_ALT=12.0

# Go to firmware folder
cd ~/src/Firmware/

# launch virtual port in background, ttyV6 is the lora port
socat -d -d pty,raw,echo=0,link=/tmp/ttyV5 pty,raw,echo=0,link=/tmp/ttyV6 &
# Bridge udp and serial
socat -d udp4-listen:14540 open:/tmp/ttyV5,raw,nonblock,waitlock=/tmp/s0.locak,echo=0,b115200,crnl &

# Make sitl drone
make px4_sitl_default gazebo_iris

# If you are not reciving any data download..
# Download the firmware. from the px4 git
# And then install firmware/tools/setup/ubuntu.sh
# https://raw.githubusercontent.com/PX4/Devguide/v1.9.0/build_scripts/ubuntu_sim_common_deps.sh
