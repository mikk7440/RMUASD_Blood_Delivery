import rospy
from mavlink_lora.msg import mavlink_lora_command_long


class GripperNode:
    def __init__(self):
        self.mavlink_cmd = rospy.Publisher("/mavlink_interface/command/send", mavlink_lora_command_long, queue_size=0)
        self.cmd = mavlink_lora_command_long()
        self.cmd.comp_id = 10  # Is our own kind of command
        self.cmd.par1 = 1  # It is a gripper
        self.cmd.par2 = 0  # Open or closed
        self.cmd.par3 = 0  # Not in use
        self.cmd.par4 = 0  # Not in use
        self.cmd.par5 = 0  # Not in use
        self.cmd.par6 = 0  # Not in use
        self.cmd.par7 = 0  # Not in use

    def open_gripper(self):
        self.cmd.par2 = 0  # Maybe open
        self.mavlink_cmd.publish(self.cmd)

    def close_gripper(self):
        self.cmd.par2 = 1  # maybe close
        self.mavlink_cmd.publish(self.cmd)
