#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg
from std_msgs.msg import Int32

pub = rospy.Publisher('/revised_scan', LaserScan, queue_size = 10)
pub_len = rospy.Publisher('/scan_len', Int32, queue_size=10)

rev_scan = LaserScan()

def callback(msg):
    #print(len(msg.ranges)) len is 2019 from 0-360
    msg.ranges = msg.ranges[300:340]
    pub.publish(msg)
    pub_len.publish(len(msg.ranges))

def listener():
    rospy.init_node('revised_scan', anonymous=True)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()