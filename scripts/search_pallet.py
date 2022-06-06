#! /usr/bin/env python3

from cmath import sin
from operator import length_hint
from re import search
import rospy
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge, CvBridgeError
import math
import cv2

class SearchImage():

    def __init__(self):


        self.bridge = CvBridge()

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.img_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/search_pallet', Image, queue_size=1)

        self.PALLET_WIDTH = 0.39
        self.PALLET_LENGTH = 0.41
        self.PALLET_DIAG = (self.PALLET_WIDTH ** 2 + self.PALLET_LENGTH ** 2) ** 0.5
        self.MAX_RANGE = 2.0
        self.tolerance = 0.02
        self.angle_increment = float('nan')
        self.angle_max = float('nan')
        self.ranges = None
        self.image = None
        self.image_height = 0
        self.image_width = 0
        self.loop_rate = rospy.Rate(20)

    def xy_of_dot(self, index, dist_value):
        angle = self.angle_of_dot(index)
        return dist_value * math.sin(angle), dist_value * math.cos(angle)

    def angle_of_dot(self, index):
        return self.angle_increment * (index - len(self.ranges) // 2)

    def distance_between_dots(self, first, second):
        x1, y1 = self.xy_of_dot(first, self.ranges[first])
        x2, y2 = self.xy_of_dot(second, self.ranges[second])
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    def search_pallet(self):
        for i in range(len(self.ranges)):
            dist = self.ranges[i]
            if dist > self.MAX_RANGE:
                continue
            #angle_increment = msg.angle_increment
            if dist < self.PALLET_DIAG:
                beta = self.angle_max
            else:
                beta = math.asin(self.PALLET_DIAG / dist)
            min_i = int(max(0, i - beta // self.angle_increment))
            max_i = int(min(len(self.ranges), i + beta // self.angle_increment))
            width_candidates = []
            length_candidates = []
            diag_candidates = []
            for j in range (min_i, max_i):
                s = self.distance_between_dots(i, j)
                if abs(s - self.PALLET_WIDTH) <= self.tolerance:
                    width_candidates.append(j)
                if abs(s - self.PALLET_LENGTH) <= self.tolerance:
                    length_candidates.append(j)
                if abs(s - self.PALLET_DIAG) <= self.tolerance:
                    diag_candidates.append(j)
            for w in width_candidates:
                for l in length_candidates:
                    for d in diag_candidates:
                        s1 = self.distance_between_dots(w, d)
                        s2 = self.distance_between_dots(l, d)
                        s3 = self.distance_between_dots(l, w)
                        if (abs(s1 - self.PALLET_LENGTH) <= self.tolerance and
                            abs(s2 - self.PALLET_WIDTH) <= self.tolerance and
                            abs(s3 - self.PALLET_DIAG) <= self.tolerance):
                            return (i, w, l, d)
            return (-1, -1, -1, -1)

    def scan_callback(self, msg):
        self.angle_increment = msg.angle_increment
        self.angle_max = msg.angle_max
        self.ranges = msg.ranges

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
        self.image_height = msg.height
        self.image_width = msg.width

    def start(self):
        while not rospy.is_shutdown():
            if self.ranges is not None:
                i, w, l, d = self.search_pallet()
                if w != -1:
                    #print (f'Pallet was found at {w}, {l}, {d}')
                    if self.image is not None:
                        self.image = cv2.circle(self.image, (i, self.image_height // 2), 5, (255,255,0), 2)
                        self.image = cv2.circle(self.image, (w, self.image_height // 2), 5, (255,0,0), 2)
                        self.image = cv2.circle(self.image, (l, self.image_height // 2), 5, (0,255,0), 2)
                        self.image = cv2.circle(self.image, (d, self.image_height // 2), 5, (0,0,255), 2)
                # else:
                #     print ("No pallet was found!")
            if self.image is not None:
                self.image = cv2.line(self.image, (0, self.image_height // 2), \
                (self.image_width, self.image_height // 2), (128, 128, 0), 2)
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.image, "rgb8"))
            self.loop_rate.sleep()




if __name__ == "__main__":
    rospy.init_node('DETECT_PALLET', anonymous=True)
    my_node = SearchImage()
    my_node.start()
    # with open('ranges', 'r') as fp:
    #     ranges = [float(x) for x in fp.read().split(", ")]
    









