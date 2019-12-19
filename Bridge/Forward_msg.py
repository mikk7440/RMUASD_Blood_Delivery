#!/usr/bin/env python

# imports
from sys import argv
import rospy
import struct
import RPi.GPIO as GPIO
from mavlink_lora.msg import mavlink_lora_msg, mavlink_lora_command_ack, mavlink_lora_mission_list,\
    mavlink_lora_mission_ack, mavlink_lora_status, mavlink_lora_pos, mavlink_lora_command_land, \
    mavlink_lora_mission_item_int, mavlink_lora_command_start_mission, mavlink_lora_command_ack, \
    mavlink_lora_command_long, mavlink_lora_heartbeat
import std_msgs.msg._Empty

# from std_msgs.msg import Int8


# defines
ros_node_name = 'forward_messenger'

# all used messages or messages that we forward
MAVLINK_MSG_ID_LONG = 76
MAVLINK_MSG_ID_COMMAND_ACK = 77
MAVLINK_MSG_ID_MISSION_ACK = 47
MAVLINK_MSG_ID_GPS_RAW_INT = 24
MAVLINK_MSG_ID_MISSION_ITEM_INT = 73
MAVLINK_MSG_ID_MISSION_COUNT = 44
MAVLINK_MSG_ID_MISSION_REQUEST_INT = 51
MAVLINK_MSG_ID_MISSION_REQUEST = 40
MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33
MAVLINK_MSG_ID_SYS_STATUS = 1

# variables

ros_node_update_interval = 10

# topic names
rx_from_gc_topic = '/gc/mavlink_rx'
tx_to_gc_topic = '/gc/mavlink_tx'

rx_from_fc_topic = '/fc/mavlink_rx'
tx_to_fc_topic = '/fc/mavlink_tx'
status_sub_topic = '/fc/mavlink_status'
land_pub_topic = '/fc/mavlink_interface/command/land'
pos_sub_topic = '/fc/mavlink_pos'

mission_topic = '/mavlink_interface/mission/mavlink_upload_mission'
# mission_ack_topic = '/fc/mavlink_interface/mission/ack'
mission_start_topic = '/fc/mavlink_interface/command/start_mission'
clear_all_topic = '/fc/mavlink_interface/mission/mavlink_clear_all'
command_long_receive_topic = '/gc/mavlink_interface/command/receive'
heartbeat_topic_fc = '/fc/mavlink_heartbeat_rx'


class ros_node():
    def __init__(self):
        # initiate variables
        self.msg = mavlink_lora_msg()
        self.current_lat = 0
        self.current_lon = 0
        self.current_alt = 0
        self.battery_volt = None
        self.min_batt_volt = 13.5 
        self.emergency = False
        self.emergency_prot_started = False
        self.battery_history = []
        self.battery_average = None
        self.status = 0

        # init GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(38, GPIO.OUT)
        GPIO.setup(40, GPIO.OUT)

        # initiate node
        rospy.init_node(ros_node_name)
        self.rate = rospy.Rate(ros_node_update_interval)

        # publishers
        self.msg_to_fc_pub = rospy.Publisher(tx_to_fc_topic, mavlink_lora_msg, queue_size=0)
        self.msg_to_gc_pub = rospy.Publisher(tx_to_gc_topic, mavlink_lora_msg, queue_size=0)
        self.mission_pub = rospy.Publisher("/fc" + mission_topic, mavlink_lora_mission_list, queue_size=0)
        self.land_pub = rospy.Publisher(land_pub_topic, mavlink_lora_command_land, queue_size=0)
        self.mission_start_pub = rospy.Publisher(mission_start_topic, mavlink_lora_command_start_mission, queue_size=0)
        self.mission_clear_pub = rospy.Publisher(clear_all_topic, std_msgs.msg.Empty, queue_size=0)

        # subscribers
        rospy.Subscriber(rx_from_gc_topic, mavlink_lora_msg, self.incoming_message_from_gc_to_fc)
        rospy.Subscriber(rx_from_fc_topic, mavlink_lora_msg, self.incoming_message_from_fc_to_gc)
        rospy.Subscriber(status_sub_topic, mavlink_lora_status, self.status_callback)
        rospy.Subscriber(pos_sub_topic, mavlink_lora_pos, self.pos_callback)
        # rospy.Subscriber(mission_ack_topic, mavlink_lora_mission_ack, self.mission_ack)
        rospy.Subscriber(command_long_receive_topic, mavlink_lora_command_long, self.command_long_callback)
        rospy.Subscriber(heartbeat_topic_fc, mavlink_lora_heartbeat, self.heartbeat_callback)

        # wait until everything is running (important)
        rospy.sleep(3)

    def incoming_message_from_gc_to_fc(self, msg):
        payload = [ord(x) for x in msg.payload]
        if msg.msg_id == MAVLINK_MSG_ID_LONG and not payload[28] == 10:
            print("received message with ID 76, forwarding to FC")
            self.msg_to_fc_pub.publish(msg)
        elif msg.msg_id == MAVLINK_MSG_ID_MISSION_COUNT:  # and not self.emergency:
            print("received message with mission count, forwarding to FC")
            self.msg_to_fc_pub.publish(msg)

        elif msg.msg_id == MAVLINK_MSG_ID_MISSION_ITEM_INT:  # and not self.emergency:
            print("received message with mission item, forwarding to FC")
            self.msg_to_fc_pub.publish(msg)

        # else:
        #     print("received message from GC with ID:")
        #     print(msg.msg_id)

    def incoming_message_from_fc_to_gc(self, msg):
        if msg.msg_id == MAVLINK_MSG_ID_COMMAND_ACK:
            print("received message command ack, forwarding to GC")
            self.msg_to_gc_pub.publish(msg)
        elif msg.msg_id == MAVLINK_MSG_ID_MISSION_ACK:  # and not self.emergency:
            print("received message mission ack, forwarding to GC")
            self.msg_to_gc_pub.publish(msg)

        elif msg.msg_id == MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            print("received message global pos, forwarding to GC")
            self.msg_to_gc_pub.publish(msg)

        elif msg.msg_id == MAVLINK_MSG_ID_GPS_RAW_INT:
            print("received message GPS raw, forwarding to GC")
            self.msg_to_gc_pub.publish(msg)

        elif msg.msg_id == MAVLINK_MSG_ID_MISSION_REQUEST_INT:  # and not self.emergency:
            print("received message mission request_int, forwarding to GC")
            self.msg_to_gc_pub.publish(msg)
        elif msg.msg_id == MAVLINK_MSG_ID_MISSION_REQUEST:  # and not self.emergency:
            print("received message mission request, forwarding to GC")
            self.msg_to_gc_pub.publish(msg)
        elif msg.msg_id == MAVLINK_MSG_ID_SYS_STATUS:
            print("received message status, forwarding to GC")
            self.msg_to_gc_pub.publish(msg)

        # else:
        #     print("received message from FC with ID: " + str(msg.msg_id))

    def status_callback(self, msg):
        self.battery_volt = float(msg.batt_volt)/1000
        if len(self.battery_history) == 0:
            b = self.battery_volt/10
            self.battery_history.extend([b, b, b, b, b, b, b, b, b, b])
        elif self.battery_volt is not 0.0:
            self.battery_history.pop(0)
            self.battery_history.append(self.battery_volt/10)
        self.battery_average = sum(self.battery_history)
        print("Avg battery: " + str(self.battery_average) + "V     Current batt reading: " + str(self.battery_volt))
        if self.battery_average is not 0 and self.battery_average < self.min_batt_volt:
            self.emergency = True
            print("EMERGENCYYY")

    def pos_callback(self, msg):
        self.current_lat = msg.lat
        self.current_lon = msg.lon
        self.current_alt = msg.alt

    def command_long_callback(self, msg):
        if msg.par1 == 1:
            if msg.par2 == 0:
                # open
                GPIO.output(38, GPIO.HIGH)
                GPIO.output(40, GPIO.LOW)
            if msg.par2 == 1:
                # close
                GPIO.output(38, GPIO.LOW)
                GPIO.output(40, GPIO.HIGH)

    def heartbeat_callback(self, msg):
        self.status = msg.system_status


    def test_for_emergency(self, event = None):
        if self.emergency and not self.emergency_prot_started:
            self.emergency_prot_started = True

            if not self.current_lat == 0 and not self.current_lon == 0:
                msg = mavlink_lora_command_land()
                msg.lat = self.current_lat
                msg.lon = self.current_lon
                msg.altitude = 20  # make it go to 10m altitude before starting landing. Can be used for precision landing systems
                msg.yaw_angle = float('NaN')  # unchanged angle
                msg.abort_alt = 5
                msg.precision_land_mode = 0  # 2=required precision landing, 1= opportunistic precision land, 0=gps landing

                self.land_pub.publish(msg)
            else:
                print('NO POSITION AVAILABLE, CAN NOT LAND')
        elif self.emergency and self.status == 3:
            msg = std_msgs.msg.Empty()
            self.mission_clear_pub.publish(msg)


if __name__ == "__main__":
    rn = ros_node()
    rospy.Timer(rospy.Duration(1.0), rn.test_for_emergency)
    rospy.spin()
