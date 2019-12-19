# RMUASD_Blood_Delivery
This is an autonomous multirotor project. It includes a custom made ground control station developed with a TKinter GUI.  The project is dependent on the PX4 firmware, Gazebo, ROS and a non-public version of the MAVlink-Lora libary.

## Dependencies
The dependencies for the ground control station, which is only tested in ubuntu 18.04 is:
* MAVLink-LoRa - Contact user ChristianBachla on mail chan315@student.sdu.dk or phone 23410987 as he knows the owner of MAVLink-LoRA and has acess the the used version. 
* ROS - Melodic
* Gazebo 9.11
* PX4 [Stable Release v1.9.2](https://github.com/PX4/Firmware/releases/tag/v1.9.2), Needs to be installed under ~src/Firmware  
* Python3 with the following dependencies, install with [pip](https://linuxize.com/post/how-to-install-pip-on-ubuntu-18.04/) unless other is stated:
  * pyttsx3. Installed with: pip install pyttsx3 --user && sudo apt install libespeak1
  * PIL. Installed with: sudo apt-get install python-imaging-tk
  * numpy
  * rospy
  * matplotlib
  * shapely
  * pykml

A raspberry pi 4 is needed with raspbian Buster, ros jubetuc and MAVlink-LoRa, a guide on this can be found [here](Rasbian Buster).

A adruino nano flashed with the Gripper.ino
 
## Ground control station
To get the ground control station to work do the following:
* Move the Ground_control folder into a catkin_ws with MAVlink-LoRa under src->mavlink_lora->scripts
* Move the launch files from the launch folder into catkin_ws -> src->mavlink_lora->launch

## Simulation
To make the simulation
* Move the HITL_ and SITL_simulation.sh bash scripts inro catkin_ws. 

## Raspberry pi
