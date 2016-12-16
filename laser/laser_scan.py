# coding=utf-8
"""
light
20161216
process the laser data
"""
import rospy
from sensor_msgs.msg import LaserScan


class LaserData(object):
    def __init__(self, data):
        rospy.loginfo( min(list(set(data.ranges))))


def listener():
    rospy.init_node('LaserScanListener', anonymous=True)

    rospy.Subscriber("scan", LaserScan, LaserData)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
if __name__ == '__main__':
    listener()