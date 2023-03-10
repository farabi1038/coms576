#!/usr/bin/env python
import rospy
from cs476.msg import FloatArray


def callback(msg):
    msg_str = " ".join(map(str, msg.nums))
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", msg_str)


def listener():
    rospy.init_node("listener", anonymous=True)
    msg = rospy.wait_for_message("chatter", FloatArray)
    callback(msg)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    listener()