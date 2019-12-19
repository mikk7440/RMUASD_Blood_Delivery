import tkinter as tk
import pyttsx3  # pip install pyttsx3 --user && sudo apt install libespeak1
from PIL import Image, ImageTk  # sudo apt-get install python-imaging-tk
import os
import rospy  # Probably you are not sourced
from mavlink_lora.msg import mavlink_lora_msg, mavlink_lora_pos, mavlink_lora_status, \
    mavlink_lora_mission_ack, mavlink_lora_command_start_mission, \
    mavlink_lora_command_land  # Probably you are not sourced
from copy import deepcopy

import gc_functions.flight_control.flight_control_noviz as fc
from gc_functions.arm_drone import arm_drone
from gc_functions.upload_mission import UploadMissionNode
from gc_functions.geo_fence import is_within_geo_fence, get_map_location, get_geo_fence_xy
from gc_functions.gripper import GripperNode
from gc_functions.droneid import DroneIDNode

import random

from math import pi, sqrt, sin, cos, atan2
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg)
import matplotlib.image as mpimg
import numpy as np

# defines
CURRENT_PATH = os.path.dirname(os.path.abspath(__file__))
R = 6371000  # Assumed Earth radius in meter
DEG2RAD = pi / 180.0
RAD2DEG = 180.0 / pi
pos_for_geofence_breach = [[55.471822,10.323022],[55.472082,10.323765],[55.472388,10.325509],\
                                [55.471889,10.324894],[55.471641,10.324173],[55.471465,10.323612]]


class GroundControl(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent)
        rospy.init_node("Ground_control")
        self.parent = parent

        # Position info
        self.home_alt = 0
        self.position = tk.StringVar()
        tk.Label(parent, textvariable=self.position, justify=tk.LEFT).grid(row=0, column=1)
        self.position.set("Last heard: \nPosition: \nAltitude: ")
        self.last_heard = 0
        self.time_last_heard_pos = 0
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.fly_alt = 32
        self.ground_speed = 0
        self.heading = 0.0
        rospy.Subscriber('/mavlink_pos', mavlink_lora_pos, self.pos_callback)
        rospy.Subscriber('/mavlink_rx', mavlink_lora_msg, self.on_mavlink_msg)
        self.update_position()

        # Path draw parameter
        self.drone_collision_time_out = 120
        self.land_now_flag = False
        self.path = [[0, 0], [0, 0]]  # point 1, point 2
        self.path_old = [[0, 0], [0, 0]]
        self.path_draw = [[0, 0], [0, 0]]  # path[0] all x's  path[1] all y's
        self.obstacles = [[[0, 0], [0, 0]], [[0, 0]]]
        self.obstacles_zone = [[[0, 0], [0, 0]], [[0, 0]]]
        self.drone_list = [['G1', 0, 0, 0, 0, 0, 0, 0, 0], ['G2', 0, 0, 0, 0, 0, 0, 0, 0],
                           ['G3', 0, 0, 0, 0, 0, 0, 0, 0],
                           ['G4', 0, 0, 0, 0, 0, 0, 0, 0]]
        self.target = [10.32345549979534, 55.47129930519026 ]
        self.path_x_start = 0
        self.path_y_start = 0
        self.target_zones = [[55.4718040079795, 10.32307768478619, "A"],
                             [55.47167975123916, 10.32365770570652, "B"],
                             [55.47206847655581, 10.32370825563272, "C"],
                             [55.47173827754129, 10.32443766787085, "D"],
                             [55.47228089795564, 10.32437208048389, "E"],
                             [55.47179993860693, 10.32493302591576, "F"],
                             [55.47130059193550, 10.32451701524021, "G"],
                             [55.47129930519026, 10.32345549979534, "H"]]
        self.status_msg = "Initialise"
        self.current_target_zone = "H"
        self.last_target_zone = "G"
        self.current_zone = "G"

        # Attitude info and update home positions.
        self.attitude = tk.StringVar()
        tk.Label(parent, textvariable=self.attitude, justify=tk.LEFT).grid(row=0, column=2)
        self.attitude.set("Dist2Home: \nYaw: \nPitch: \nROLL: ")
        self.home_lat = 0
        self.home_lon = 0
        self.last_lat = 0
        self.last_lon = 0
        self.last_alt = 0
        self.first_run_attitude_flag = True
        self.update_attitude()

        self.button_height = 7
        self.button_width = 12

        # Battery info
        self.battery_status = tk.StringVar()
        tk.Label(parent, textvariable=self.battery_status, justify=tk.LEFT).grid(sticky=tk.N, row=4, column=1)
        rospy.Subscriber('/mavlink_status', mavlink_lora_status, self.status_callback)
        self.battery_status.set("Battery voltage: ")
        self.battery_volt = 0.0
        self.update_battery()

        # With-in geo fence
        self.geo_fence_status = tk.StringVar()
        self.geo_fence_label = tk.Label(parent, textvariable=self.geo_fence_status, justify=tk.LEFT)
        self.geo_fence_label.grid(sticky=tk.N, row=4, column=2)
        self.geo_fence_status.set("Geo fence: ")
        self.update_geo_fence_status()

        # Arm drone
        self.arm_drone_button = tk.Button(parent, text="Arm", command=self.click_arm, height=self.button_height,
                                          width=self.button_width)
        self.arm_drone_button.grid(sticky=tk.N, row=0, column=0)
        self.arm_drone_button_clicked = False
        self.start_mission_button_clicked = False
        self.slide_min = 0
        self.slide_max = 100
        self.arm_drone_slider = tk.Scale(parent, from_=0, to=100, label="Slide to confirm arm", orient=tk.HORIZONTAL,
                                         length=200, width=25)
        self.arm_drone_slider.bind("<ButtonRelease-1>", self.arm_slider)
        self.arm_drone_slider.grid(sticky=tk.N, row=5, column=2, columnspan=1)

        # Set home
        set_home_button = tk.Button(parent, text="Set home", command=self.click_set_home, height=self.button_height,
                                    width=self.button_width)
        set_home_button.grid(sticky=tk.N, row=3, column=0)

        # Upload mission
        self.upload_mission_button = tk.Button(parent, text="Upload mission", command=self.click_upload_mission,
                                               justify=tk.LEFT, height=self.button_height, width=self.button_width)
        self.upload_mission_button.grid(sticky=tk.N, row=1, column=0)

        # Update mission
        self.update_mission_button = tk.Button(parent, text="Update mission", command=self.click_update_mission,
                                               justify=tk.LEFT, height=self.button_height, width=self.button_width)
        self.update_mission_button.grid(sticky=tk.N, row=5, column=3)

        self.upload_mission_node = UploadMissionNode()
        mavlink_mission_ack_sub = rospy.Subscriber("mavlink_interface/mission/ack", mavlink_lora_mission_ack,
                                                   self.on_ack_received_callback)
        self.waiting_for_mission_ack = 0

        # Start mission:
        self.start_mission_button = tk.Button(parent, text="Start mission", command=self.click_start_mission,
                                              justify=tk.LEFT, height=self.button_height, width=self.button_width)
        self.start_mission_button.grid(sticky=tk.N, row=2, column=0)
        self.mavlink_start_mission_pub = rospy.Publisher('/mavlink_interface/command/start_mission',
                                                         mavlink_lora_command_start_mission, queue_size=0)

        # Update map:
        self.map_fig = get_map_location(self.lat, self.lon)
        # self.map_fig.suptitle('Drone placement:', fontsize=10)
        self.airside_img = mpimg.imread("gc_functions/airfield.png")
        self.canvas = FigureCanvasTkAgg(self.map_fig, master=self.parent)
        self.canvas.draw()
        self.canvas.callbacks.connect('button_press_event', self.set_target_on_click)
        self.canvas.get_tk_widget().grid(sticky=tk.N, row=1, column=1, rowspan=3,
                                         columnspan=2)  # pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        self.update_map()

        # Gripper button
        self.gripper_node = GripperNode()
        self.gripper_b1 = tk.Button(parent, text="Gripper open", command=self.gripper_node.close_gripper,
                                    justify=tk.LEFT, height=self.button_height, width=self.button_width)
        self.gripper_b1.grid(sticky=tk.N, row=4, column=0)
        self.gripper_b2 = tk.Button(parent, text="Gripper close", command=self.gripper_node.open_gripper,
                                    justify=tk.LEFT, height=self.button_height, width=self.button_width)
        self.gripper_b2.grid(sticky=tk.N, row=5, column=0)

        # DroneID setup, not visual!
        self.sim = True
        self.status_toggle_button = tk.Button(parent, text="Sim On", command=self.toggle_sim_status,
                                              justify=tk.LEFT, height=self.button_height, width=self.button_width)
        self.status_toggle_button.grid(sticky=tk.N, row=4, column=3)
        self.drone_id_node = DroneIDNode()
        self.drone_id_func()

        # Land at spot
        land_pub_topic = '/fc/mavlink_interface/command/land'
        self.land_pub = rospy.Publisher(land_pub_topic, mavlink_lora_command_land, queue_size=0)
        self.land_button = tk.Button(parent, text="Land", command=self.land,
                                     justify=tk.LEFT, height=self.button_height, width=self.button_width*3)
        self.land_button.grid(sticky=tk.W, row=5, column=1)

        # Speech setup
        self.engine = pyttsx3.init("espeak")
        self.engine.setProperty('rate', 160)
        self.say("Ground station fucking ready!")

        # Set home
        self.click_set_home()

        # Path planning
        self.this_drone_id = 0
        self.plan_path()

    def land(self):
        self.land_now_flag = not self.land_now_flag
        self.upload_mission_node.land(self.lat, self.lon)
        # msg = mavlink_lora_command_land()
        # msg.lat = self.lat
        # msg.lon = self.lon
        # msg.altitude = 20  # make it go to 10m altitude before starting landing. Can be used for precision landing systems
        # msg.yaw_angle = float('NaN')  # unchanged angle
        # msg.abort_alt = 5
        # msg.precision_land_mode = 0  # 2=required precision landing, 1= opportunistic precision land, 0=gps landing
        #
        # self.land_pub.publish(msg)

    def set_target_on_click(self, event):
        if event.inaxes is not None:
            self.target = [event.xdata, event.ydata]

    def plan_path(self):
        # set new target
        idx = self.letter_to_index(self.current_target_zone)
        dist_to_target = self.gcd_haversine(self.target_zones[idx][0], self.target_zones[idx][1], self.lat, self.lon)
        if dist_to_target <= 6:
            self.current_zone = self.current_target_zone
            # Only update target when rel alt is under 5
            # if self.alt - self.home_alt <= 7:
            get_new_target = self.fly_to_next()
            if get_new_target:
                self.last_target_zone = self.current_target_zone
                self.target[0] = get_new_target[1]
                self.target[1] = get_new_target[0]
                self.current_target_zone = get_new_target[2]

        scale = 100000
        scale_lon, scale_lat = self.lon * scale, self.lat * scale
        fc.set_drone_pos(self.this_drone_id, scale_lon, scale_lat)

        self.drone_id_node.update_drone_list()
        drone_list_raw = deepcopy(self.drone_id_node.drone_list)  # Group one
        self.drone_list = []
        for drones in drone_list_raw:
            #                       Long,        Lat,       Status,    Time
            self.drone_list.append([drones[3], drones[2], drones[8], drones[1]])
        # Remove our own drown from list.
        self.drone_list.pop(3)
        for i, drones in enumerate(self.drone_list):
            # i+1 = id of drone. (we are 0)
            # print("Drone: ", i+1, drones)
            if drones[3] <= self.drone_collision_time_out:
                fc.set_drone_pos(i+1, drones[0] * scale, drones[1]*scale)

        planned_path, status = fc.plan_path(self.target[0] * scale, self.target[1] * scale)
        self.obstacles = deepcopy(fc.obstacles_new)
        self.obstacles_zone = deepcopy(fc.obstacles)

        self.path = []
        self.path_draw = [[self.lon], [self.lat]]
        for points in planned_path:
            lon, lat = round(points[0] / 100000, 7), round(points[1] / 100000, 7)
            self.path.append([lon, lat])
            self.path_draw[0].append(lon)
            self.path_draw[1].append(lat)

        if self.land_now_flag:
            print("Landing")
            pass
        elif not is_within_geo_fence(self.lat, self.lon):
            shortest_distance = self.gcd_haversine(pos_for_geofence_breach[0][0], pos_for_geofence_breach[0][1], self.lat, self.lon)
            shortest_distance_index = 0
            for i in range(len(pos_for_geofence_breach)):
                dist = self.gcd_haversine(pos_for_geofence_breach[i][0], pos_for_geofence_breach[i][1], self.lat, self.lon)
                if dist < shortest_distance:
                    shortest_distance = dist
                    shortest_distance_index = i
            self.path = [[pos_for_geofence_breach[shortest_distance_index][1],
                          pos_for_geofence_breach[shortest_distance_index][0]]]
            self.click_update_mission()
            pass
        elif self.path == []:
            self.upload_mission_node.go_to_loiter_mode()
            print("Going to loiter")
        elif self.path != self.path_old:
            # update_mission_flag = False
            # print('first path then path_old')
            # print(self.path)
            # print(self.path_old)
            # if len(self.path_old) != len(self.path):
            #     update_mission_flag = True
            # else:
            #     for i in range(1, len(self.path_old)):
            #         if self.path_old[i] != self.path[i]:
            #             update_mission_flag = True
            #             break
            # if update_mission_flag:
            self.click_update_mission()
            print("Mission updated")

        self.path_old = deepcopy(self.path)
        if not is_within_geo_fence(self.lat, self.lon):
            self.after(4000, self.plan_path)
        else:
            self.after(2000, self.plan_path)

    def toggle_sim_status(self):
        if self.sim:
            self.sim = False
            self.status_toggle_button.configure(text="Real On")
        else:
            self.sim = True
            self.status_toggle_button.configure(text="Sim On")

    def drone_id_func(self):

        self.status_msg = "FreeFly" #  self.current_target_zone

        dist_to_zones = []
        for i, zone in enumerate(self.target_zones):
            dist = self.gcd_haversine(zone[0], zone[1], self.target[1], self.target[0])
            if dist <= 2:
                self.status_msg = self.target_zones[i][2]
                self.current_target_zone = self.target_zones[i][2]

        idx = self.letter_to_index(self.last_target_zone)
        dist_from_target = self.gcd_haversine(self.target_zones[idx][0], self.target_zones[idx][1], self.lat, self.lon)
        if dist_from_target <= 5:
            self.status_msg = self.last_target_zone

        if self.lat != 0:
            self.drone_id_node.update_log(lat=self.lat, lon=self.lon, alt=self.alt, trk=self.heading,
                                          hsp=self.ground_speed,
                                          eng=round(self.battery_volt / 16.8), sim=self.sim, rst=self.status_msg)

            rsp_code, rsp_txt = self.drone_id_node.send_log()
            if rsp_code != 200:
                print("DID NOT RESPONSE WITH 200")
                print(rsp_code, rsp_txt)

        self.after(500, self.drone_id_func)

    def status_callback(self, msg):
        self.battery_volt = msg.batt_volt / 1000.0

    def on_mavlink_msg(self, msg):
        # save timestamp of last package of anything received from the drone
        self.last_heard = rospy.get_time()

    def on_ack_received_callback(self, msg):
        if msg.result_text == "MAV_MISSION_ACCEPTED":
            self.upload_mission_button.configure(bg="green")
            if self.waiting_for_mission_ack == 1:
                self.waiting_for_mission_ack = 0
                self.upload_mission_node.set_mode_to_mission()

    def pos_callback(self, msg):
        # Last position
        self.last_lat = self.lat
        self.last_lon = self.lon
        self.last_alt = self.alt
        # Current position
        self.lat = msg.lat
        self.lon = msg.lon
        self.alt = msg.alt
        now = rospy.get_time()
        if self.time_last_heard_pos == 0:
            self.ground_speed = 0
        else:
            time = now - self.time_last_heard_pos
            distance = self.gcd_haversine(self.last_lat, self.last_lon, self.lat, self.lon)
            self.ground_speed = distance / time
            self.heading = self.gcd_heading(self.last_lat, self.last_lon, self.lat, self.lon)
        if self.ground_speed == 0:
            self.heading = 0.0
        self.time_last_heard_pos = now

    def update_position(self):
        now = rospy.get_time()

        # update last_heard
        t = now - self.last_heard
        if t < 86400:
            last_heard_text = '%ds' % t
            if t > 10:
                self.parent.configure(background='red')
        else:
            last_heard_text = 'Never'

        # update pos status
        if self.lat == 0 and self.lon == 0:
            pos_text = 'Unknown'
        else:
            pos_text = '%02.5f %03.5f' % (self.lat, self.lon)

        # update altitude status
        if self.alt == 0:
            alt_text = 'Unknown'
        else:
            alt_text = '%.1fm ' % (self.alt - self.home_alt)

        self.position.set(
            "Last heard:      {}\nPosition:         {}\nAltitude:         {}".format(last_heard_text, pos_text,
                                                                                     alt_text))
        self.after(500, self.update_position)

    def update_attitude(self):
        if self.home_lat == 0 and self.home_lon == 0:
            home_text = 'Unknown'
        else:
            home_text = '%.1fm' % self.gcd_haversine(self.lat, self.lon, self.home_lat, self.home_lon)

        grond_speed_text = '%.1fm' % self.ground_speed
        heading_text = '%.1fDeg' % self.heading

        self.attitude.set(
            "Dist2Home:  {}\nGround speed:  {}\nHeading:   {}".format(home_text, grond_speed_text, heading_text))
        self.after(500, self.update_attitude)

    def update_battery(self):
        if self.battery_volt == 0:
            batt_text = 'Unknown'
        else:
            batt_text = '%.1fV' % self.battery_volt

        self.battery_status.set("\n\nBattery voltage: {}".format(batt_text))
        self.after(1000, self.update_battery)

    def update_geo_fence_status(self):
        if (self.lat, self.lon) == (0, 0):
            geo_fence_text = "Unknown"
        elif not is_within_geo_fence(self.lat, self.lon):
            geo_fence_text = "BREACH!"
            self.geo_fence_label.config(bg="red")
        else:
            geo_fence_text = "Good"
            self.geo_fence_label.config(bg="#d9d9d9")

        self.geo_fence_status.set("\n\nGeo fence:  {}".format(geo_fence_text))
        self.after(500, self.update_geo_fence_status)

    def update_map(self):
        # Clear old img
        self.map_fig.clear()
        # Prepare plot
        sup_plot = self.map_fig.add_subplot(111)

        # Get and plot geofence
        y, x = get_geo_fence_xy()
        sup_plot.plot(x, y, 'y--')

        # Plot all obstacles
        for obstacle in self.obstacles:
            obstacle.append(obstacle[0])
            np_obstacle = np.array(obstacle) / 100000  # Scale to fit mat
            obs_lon, obs_lat = np_obstacle[:, 1], np_obstacle[:, 0]

            sup_plot.plot(obs_lon, obs_lat, 'r')

        for zones in self.target_zones:
            sup_plot.plot(zones[1], zones[0], 'bx')

        if len(self.obstacles_zone):
            self.obstacles_zone.pop(0)
        for obstacle in self.obstacles_zone:
            obstacle.append(obstacle[0])
            np_obstacle = np.array(obstacle) / 100000  # Scale to fit mat
            obs_lon, obs_lat = np_obstacle[:, 1], np_obstacle[:, 0]

            sup_plot.plot(obs_lon, obs_lat, 'r--')

        for drones in self.drone_list:
            if drones[3] <= self.drone_collision_time_out:
                sup_plot.plot(drones[0], drones[1], 'mo')
        # Get and plot path
        # sup_plot.plot(self.path_draw[0], self.path_draw[1], 'wx--', self.target[0],
        #               self.target[1], 'wx')
        sup_plot.plot(self.target[0], self.target[1], 'wx')
        # For manual mission mode
        mission_y, mission_x = self.upload_mission_node.get_current_mission()
        mission_x.insert(0, self.path_x_start)
        mission_y.insert(0, self.path_y_start)
        last_idx = len(mission_x) - 1
        sup_plot.plot(mission_x, mission_y, 'w--x', mission_x[last_idx], mission_y[last_idx], 'wo')

        # Plot drone pos
        sup_plot.plot(self.lon, self.lat, 'bo')

        # Add airfield img
        sup_plot.imshow(self.airside_img, extent=[10.322651, 10.325291, 55.471133, 55.472654], aspect='auto')
        sup_plot.set_aspect('auto')
        sup_plot.set_xticks([])
        sup_plot.set_yticks([])
        self.map_fig.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=0, hspace=0)
        self.canvas.draw()
        self.after(1000, self.update_map)

    def click_arm(self):
        self.arm_drone_button_clicked = True
        self.say("Slide to arm")

    def arm_slider(self, arm_slider_value):
        if self.arm_drone_slider.get() != self.slide_max:
            pass
            # self.arm_drone_slider.set(self.slide_min)
        else:
            if self.start_mission_button_clicked:
                self.start_mission_button_clicked = False
                self.say("Starting this shit mission!")
                # make msg and publish it
                cmd = mavlink_lora_command_start_mission()
                # last item isn't needed as PX4 doesn't use last_item for anything.
                cmd.first_item = 0
                cmd.last_item = 0
                self.mavlink_start_mission_pub.publish(cmd)
            elif self.arm_drone_button_clicked:
                arm_drone()
                self.arm_drone_button_clicked = False
                self.arm_drone_button.config(bg="green")
                self.say("This Shit is armed!")
            self.arm_drone_slider.set(self.slide_min)

    def click_upload_mission(self):
        self.upload_mission_button.configure(bg="red")
        self.upload_mission_node.upload_mission(current_lat=self.lat, current_lon=self.lon)

    def click_update_mission(self):
        self.waiting_for_mission_ack = 0
        self.path_x_start = self.lon
        self.path_y_start = self.lat
        self.upload_mission_node.load_mission_from_path(self.path, self.fly_alt)
        self.upload_mission_node.update_mission(current_lat=self.lat, current_lon=self.lon,
                                                current_alt=(self.alt - self.home_alt), sim=self.sim)
        self.waiting_for_mission_ack = 1

    def click_set_home(self):
        self.home_lat = self.lat
        self.home_lon = self.lon
        if self.alt >= 20:
            self.alt = 12
        else:
            self.home_alt = self.alt

    def click_start_mission(self):
        self.start_mission_button_clicked = True
        self.say("Slide to start mission")

    def letter_to_index(self, letter):
        switcher = {
            "A": 0,
            "B": 1,
            "C": 2,
            "D": 3,
            "E": 4,
            "F": 5,
            "G": 6,
            "H": 7
        }
        return switcher.get(letter)

    def gcd_haversine(self, lat1, lon1, lat2, lon2):
        # Function origins from gcs_simple.py from the MAVLink-lora libary
        lat1 *= DEG2RAD
        lon1 *= DEG2RAD
        lat2 *= DEG2RAD
        lon2 *= DEG2RAD
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = sin(dlat / 2.) ** 2 + sin(dlon / 2.) ** 2 * cos(lat1) * cos(lat2)
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        return R * c

    def gcd_heading(self, lat1, lon1, lat2, lon2):
        # Function inspired by the formular from https://www.movable-type.co.uk/scripts/latlong.html
        y = sin(lon2 - lon1) * cos(lat2)
        x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1)
        heading = atan2(y, x) * RAD2DEG
        heading = (heading + 360.00) % 360.00
        return heading

    def fly_to_next(self):
        next_stop = False

        # Check for next target
        for i in range(len(self.target_zones)):
            if self.current_zone == self.target_zones[i][2]:
                next_stop = self.target_zones[(i + 1) % len(self.target_zones)]

        # Check that it is not blocked
        for other_drones in self.drone_list:
            if other_drones[3] <= self.drone_collision_time_out*3:
                if next_stop[2] == other_drones[2]:
                    return False
        return next_stop

    def say(self, text):
        self.engine.say(text)
        self.engine.runAndWait()


if __name__ == "__main__":
    # create a root window
    root = tk.Tk()
    # Icon made by itim2101 https://www.flaticon.com/authors/itim2101
    img = Image.open(CURRENT_PATH + "/gc_functions/icon.png")
    icon = ImageTk.PhotoImage(img)
    root.tk.call('wm', 'iconphoto', root._w, icon)
    # Add a title
    root.title("Ground control")

    # add our example to the root window
    gc = GroundControl(root)
    # gc.pack(fill="both", expand=True)

    # start the event loop
    root.mainloop()
    rospy.spin()
