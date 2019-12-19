#!/bin/bash

# Drone home is currently at the HCA Airport test field.
export PX4_HOME_LAT=55.471876
export PX4_HOME_LON=10.324855

# Go to firmware folder
cd ~/src/Firmware/

# launch virtual port in background, ttyV6 is the lora port
# socat -d -d pty,raw,echo=0,link=/tmp/ttyV5 pty,raw,echo=0,link=/dev/ttyUSB0 &
# Bridge udp and serial
socat -d udp4-listen:14540 open:/dev/ttyUSB0,raw,nonblock,waitlock=/tmp/s0.locak,echo=0,b57600,crnl &

# Make sitl drone
make px4_sitl_default gazebo_iris
