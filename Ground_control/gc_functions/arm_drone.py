import rospy
from std_msgs.msg import Bool

mavlink_arm_pub = rospy.Publisher('/mavlink_interface/command/arm_disarm', Bool, queue_size=0)


def arm_drone():
    msg = Bool()
    msg.data = True
    mavlink_arm_pub.publish(msg)
