#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

class Nodo(object):
    def __init__(self):
        # Params
        self.image = None
        self.ranges = []
        self.br = CvBridge()
        self.range_min = 0.45
        self.range_max = 10.0
        self.increment = (self.range_max - self.range_min) / 180
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(20)

        # Publishers
        self.pub = rospy.Publisher('imagetimer', Image,queue_size=10)

        # Subscribers
        rospy.Subscriber("/camera/color/image_raw",Image,self.callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def callback(self, msg):
        # rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg, msg.encoding)

    def scan_callback(self, msg):
        self.ranges = msg.ranges


    def start(self):
        rospy.loginfo("Timing images")
        #rospy.spin()
        while not rospy.is_shutdown():
            # rospy.loginfo('publishing image')
            #br = CvBridge()
            if self.image is not None:
                rospy.loginfo('publishing image')
                hsv_img = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
                for i in range(len(self.ranges)):
                    color = (self.ranges[i] - self.range_min) // self.increment
                    if i == len(self.ranges) // 2:
                        hsv_img = cv2.putText(hsv_img, str(round(self.ranges[i], 3)), (hsv_img.shape[1] // 2, hsv_img.shape[0] // 2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        hsv_img = cv2.circle(hsv_img, (i, hsv_img.shape[0] // 2), 2, (color, 255, 0), 1)
                    
                    hsv_img = cv2.circle(hsv_img, (i, hsv_img.shape[0] // 2), 1, (color, 255, 255), 1)
                    

                hsv_img = cv2.cvtColor(hsv_img, cv2.COLOR_HSV2BGR)
                cv2.imshow('Image', hsv_img)
                self.pub.publish(self.br.cv2_to_imgmsg(hsv_img, "rgb8"))
                # self.pub.publish(self.br.cv2_to_imgmsg(self.image, "bgr8"))
            self.loop_rate.sleep()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node("imagetimer111", anonymous=True)
    my_node = Nodo()
    my_node.start()
