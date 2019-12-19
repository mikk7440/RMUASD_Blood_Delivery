#!/usr/bin/env python
# /***************************************************************************
# MavLink LoRa node (ROS) upload mission example script
# Copyright (c) 2018, Kjeld Jensen <kjen@mmmi.sdu.dk> <kj@kjen.dk>
# SDU UAS Center, http://sdu.dk/uas
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ****************************************************************************
'''
Script to upload mission for healthdrone test flights

Revision
2019-09-18 FMA First published version
'''
# imports
import rospy
import time
import std_msgs.msg._Empty
# mission messages
from mavlink_lora.msg import mavlink_lora_mission_list
from mavlink_lora.msg import mavlink_lora_mission_item_int, mavlink_lora_mission_ack, mavlink_lora_command_set_mode
# parameters for mission upload
mavlink_lora_pub_topic = '/mavlink_interface/mission/mavlink_upload_mission'
mavlink_lora_set_mode_pub_topic = '/mavlink_interface/command/set_mode'
mavlink_lora_clear_all = '/mavlink_interface/mission/mavlink_clear_all'

time_start = 0
time_end = 0


class UploadMissionNode:
    def __init__(self):
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.way_lat = []
        self.way_lon = []
        self.way_alt = []
        # self.load_mission()
        self.static_alt = 32

        # pubs
        self.mavlink_msg_pub = rospy.Publisher(mavlink_lora_pub_topic, mavlink_lora_mission_list, queue_size=0)
        self.mavlink_set_mode_pub = rospy.Publisher(mavlink_lora_set_mode_pub_topic, mavlink_lora_command_set_mode, queue_size=0)
        self.mavlink_clear_mission = rospy.Publisher(mavlink_lora_clear_all, std_msgs.msg.Empty, queue_size=0)

    def send_mavlink_set_mode(self, mode, custom_mode, custom_sub_mode):
        # make msg and publish it
        msg = mavlink_lora_command_set_mode()
        msg.mode = mode
        msg.custom_mode = custom_mode
        msg.custom_sub_mode = custom_sub_mode

        self.mavlink_set_mode_pub.publish(msg)

    def get_current_mission(self):
        lat = []
        lon = []
        for idx in range(len(self.way_lat)):
            lat.append(self.way_lat[idx]/10000000)
            lon.append(self.way_lon[idx]/10000000)
        return lat, lon

    def load_mission_from_path(self, path, alt):
        self.way_lat = []
        self.way_lon = []
        self.way_alt = []
        for points in path:
            self.way_lat.append(int(float(points[1]) * 10000000))
            self.way_lon.append(int(float(points[0]) * 10000000))
            self.way_alt.append(int(alt))

    def load_mission(self):
        file = open('gc_functions/mission_1.txt', 'r')
        self.way_lat = []
        self.way_lon = []
        self.way_alt = []
        for line in file.readlines():
            o, t, th = line.split(" ")
            self.way_lat.append(int(float(o) * 10000000))
            self.way_lon.append(int(float(t) * 10000000))
            self.way_alt.append(int(th))

    def load_mission2(self):
        file = open('gc_functions/mission_2.txt', 'r')
        self.way_lat = []
        self.way_lon = []
        self.way_alt = []
        for line in file.readlines():
            o, t, th = line.split(" ")
            self.way_lat.append(int(float(o) * 10000000))
            self.way_lon.append(int(float(t) * 10000000))
            self.way_alt.append(int(th))

    def set_waypoints(self, lat, lon, alt, sequ):
        way = mavlink_lora_mission_item_int()
        way.target_system = 0
        way.target_component = 0
        way.seq = sequ
        way.frame = 6
        way.command = 16
        way.param1 = 0
        way.param2 = 5
        way.param3 = 0
        way.x = lat
        way.y = lon
        way.z = alt
        way.autocontinue = 1

        return way

    def upload_mission(self, current_lat, current_lon):
        self.lat = int(current_lat * 10000000)
        self.lon = int(current_lon * 10000000)

        if self.lat == 0 and self.lon == 0:
            print("No GPS Mission not uploaded")
            return

        seq = 0
        missionlist = mavlink_lora_mission_list()

        # Takeoff
        takeoff = mavlink_lora_mission_item_int()
        takeoff.target_system = 0
        takeoff.target_component = 0
        takeoff.seq = seq
        takeoff.frame = 6  # global pos, relative alt_int
        takeoff.command = 22
        takeoff.x = self.lat
        takeoff.y = self.lon
        takeoff.z = self.static_alt
        takeoff.param1 = 5
        takeoff.current = 1
        takeoff.autocontinue = 1

        missionlist.waypoints.append(takeoff)

        for i in range(len(self.way_alt)):
            seq = i + 1
            wp = self.set_waypoints(self.way_lat[i], self.way_lon[i],
                                    self.way_alt[i], seq)
            missionlist.waypoints.append(wp)


        landing = mavlink_lora_mission_item_int()
        landing.target_system = 0
        landing.target_component = 0
        landing.seq = seq + 1
        landing.frame = 6  # global pos, relative alt_int
        landing.command = 21
        landing.param1 = 5  # abort alt
        landing.param2 = 0  # precision landing. 0 = normal landing
        landing.x = self.lat
        landing.y = self.lon
        landing.z = 10
        landing.autocontinue = 0

        missionlist.waypoints.append(landing)
        self.mavlink_msg_pub.publish(missionlist)

    def go_to_loiter_mode(self):
        self.send_mavlink_set_mode(1, 4, 3)

    def update_mission(self, current_lat, current_lon, current_alt, sim):
        # Put uav in loiter mode
        self.send_mavlink_set_mode(1, 4, 3)

        # Clear mission
        # msg = std_msgs.msg.Empty()
        # self.mavlink_clear_mission.publish(msg)


        # Upload mission
        self.lat = int(current_lat * 10000000)
        self.lon = int(current_lon * 10000000)

        if self.lat == 0 and self.lon == 0:
            print("No GPS Mission not uploaded")
            return

        seq = 0
        missionlist = mavlink_lora_mission_list()

        # if sim:
        #     print("Added speed points x2")
        #     speed = mavlink_lora_mission_item_int()
        #     speed.target_system = 0
        #     speed.target_component = 0
        #     speed.seq = seq  # Sequence in the list. Starts from 0 and every item increments by one
        #     speed.frame = 2  # mission command frame
        #     speed.command = 178  # change speed id
        #     speed.param1 = 0  # air_speed
        #     speed.param2 = 4  # m/s
        #     speed.param3 = -1  # no change
        #     speed.param4 = 0  # absolute or relative. relative = 1
        #     speed.autocontinue = 1  # automatic continue to next waypoint when this is reached
        #     missionlist.waypoints.append(speed)
        #     seq += 1
        #
        #     speed1 = mavlink_lora_mission_item_int()
        #     speed1.target_system = 0
        #     speed1.target_component = 0
        #     speed1.seq = seq  # Sequence in the list. Starts from 0 and every item increments by one
        #     speed1.frame = 2  # mission command frame
        #     speed1.command = 178  # change speed id
        #     speed1.param1 = 0  # air_speed
        #     speed1.param2 = 5  # m/s
        #     speed1.param3 = -1  # no change
        #     speed1.param4 = 0  # absolute or relative. relative = 1
        #     speed1.autocontinue = 1  # automatic continue to next waypoint when this is reached
        #     missionlist.waypoints.append(speed1)
        #     seq += 1

        # Add a Takeoff point
        print("Added TakeOff")
        takeoff = mavlink_lora_mission_item_int()
        takeoff.target_system = 0
        takeoff.target_component = 0
        takeoff.seq = seq
        takeoff.frame = 6  # global pos, relative alt_int
        takeoff.command = 22
        takeoff.x = self.lat
        takeoff.y = self.lon
        takeoff.z = self.static_alt
        takeoff.param1 = 5
        takeoff.current = 1
        takeoff.autocontinue = 1
        missionlist.waypoints.append(takeoff)
        seq += 1

        for i in range(len(self.way_alt)):
            wp = self.set_waypoints(self.way_lat[i], self.way_lon[i],
                                    self.way_alt[i], seq)
            missionlist.waypoints.append(wp)
            seq += 1

        landing = mavlink_lora_mission_item_int()
        landing.target_system = 0
        landing.target_component = 0
        landing.seq = seq
        landing.frame = 6  # global pos, relative alt_int
        landing.command = 21
        landing.param1 = 20  # abort alt
        landing.param2 = 0  # precision landing. 0 = normal landing
        landing.x = self.way_lat[len(self.way_lat)-1]
        landing.y = self.way_lon[len(self.way_lon)-1]
        landing.z = 32
        landing.autocontinue = 0

        missionlist.waypoints.append(landing)
        self.mavlink_msg_pub.publish(missionlist)

    def land(self, current_lat, current_lon):
        # Upload mission
        self.lat = int(current_lat * 10000000)
        self.lon = int(current_lon * 10000000)

        if self.lat == 0 and self.lon == 0:
            print("No GPS Mission not uploaded")
            return

        seq = 0
        missionlist = mavlink_lora_mission_list()

        takeoff = mavlink_lora_mission_item_int()
        takeoff.target_system = 0
        takeoff.target_component = 0
        takeoff.seq = seq
        takeoff.frame = 6  # global pos, relative alt_int
        takeoff.command = 22
        takeoff.x = self.lat
        takeoff.y = self.lon
        takeoff.z = self.static_alt
        takeoff.param1 = 5
        takeoff.current = 1
        takeoff.autocontinue = 1
        missionlist.waypoints.append(takeoff)
        seq += 1

        landing = mavlink_lora_mission_item_int()
        landing.target_system = 0
        landing.target_component = 0
        landing.seq = seq
        landing.frame = 6  # global pos, relative alt_int
        landing.command = 21
        landing.param1 = 20  # abort alt
        landing.param2 = 0  # precision landing. 0 = normal landing
        landing.x = self.lat
        landing.y = self.lon
        landing.z = 32
        landing.autocontinue = 0

        missionlist.waypoints.append(landing)
        self.mavlink_msg_pub.publish(missionlist)

    def land_takeoff(self, current_lat, current_lon):
        # Upload mission
        self.lat = int(current_lat * 10000000)
        self.lon = int(current_lon * 10000000)

        if self.lat == 0 and self.lon == 0:
            print("No GPS Mission not uploaded")
            return

        seq = 0
        missionlist = mavlink_lora_mission_list()

        takeoff = mavlink_lora_mission_item_int()
        takeoff.target_system = 0
        takeoff.target_component = 0
        takeoff.seq = seq
        takeoff.frame = 6  # global pos, relative alt_int
        takeoff.command = 21
        takeoff.x = self.lat
        takeoff.y = self.lon
        takeoff.z = 5
        takeoff.param1 = 5
        takeoff.current = 1
        takeoff.autocontinue = 1
        missionlist.waypoints.append(takeoff)
        seq += 1

        landing = mavlink_lora_mission_item_int()
        landing.target_system = 0
        landing.target_component = 0
        landing.seq = seq
        landing.frame = 6  # global pos, relative alt_int
        landing.command = 21
        landing.param1 = 20  # abort alt
        landing.param2 = 0  # precision landing. 0 = normal landing
        landing.x = self.lat
        landing.y = self.lon
        landing.z = 32
        landing.autocontinue = 0
        seq += 1

        takeoff = mavlink_lora_mission_item_int()
        takeoff.target_system = 0
        takeoff.target_component = 0
        takeoff.seq = seq
        takeoff.frame = 6  # global pos, relative alt_int
        takeoff.command = 22
        takeoff.x = self.lat
        takeoff.y = self.lon
        takeoff.z = self.static_alt
        takeoff.param1 = 5
        takeoff.current = 1
        takeoff.autocontinue = 1
        missionlist.waypoints.append(takeoff)
        seq += 1

        missionlist.waypoints.append(landing)
        self.mavlink_msg_pub.publish(missionlist)


    def set_mode_to_mission(self):
        # Set back to mission mode
        self.send_mavlink_set_mode(1, 4, 4)

# if __name__ == '__main__':
#     uploaded_mission = Upload_mission_node()
#     uploaded_mission.upload_mission()
#     # while not (rospy.is_shutdown()):
#     #     # do stuff
#     #     rospy.sleep(5)
