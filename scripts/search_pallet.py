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
        self.MAX_RANGE = 2.5
        self.tolerance = 0.02
        self.angle_increment = float('nan')
        self.angle_max = float('nan')
        self.ranges = None
        self.image = None
        self.image_height = 0
        self.image_width = 0
        self.loop_rate = rospy.Rate(10)

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
        approx_ranges = self.approximation(self.ranges)  
     
        for i in approx_ranges:
            self.image = cv2.circle(self.image, (i, self.image_height // 2), 5, (255, 255, 255), 2)


        final_candidates = []
        errors = []

        for i in approx_ranges:
            dist = self.ranges[i]
            candidates = [[] for j in range(3)]
            for j in approx_ranges:
                s = self.distance_between_dots(i, j)
                if abs(s - self.PALLET_WIDTH) <= self.tolerance:
                    candidates[0].append(j)
                if abs(s - self.PALLET_LENGTH) <= self.tolerance:
                    candidates[1].append(j)
                if abs(s - self.PALLET_DIAG) <= self.tolerance:
                    candidates[2].append(j)

            # for k in candidates[0]:
            #     self.image = cv2.circle(self.image, (k, self.image_height // 2), 5, (0, 255, 255), 2)
            
            print('first = ', len(approx_ranges))
            print('width = ', candidates[0])
            print('length = ', candidates[1])
            print('diag = ', candidates[2],  '\n\n')

            for w in candidates[0]:
                for l in candidates[1]:
                    for d in candidates[2]:
                        s1 = abs(self.distance_between_dots(w, d) - self.PALLET_LENGTH)
                        s2 = abs(self.distance_between_dots(l, d)  - self.PALLET_WIDTH)
                        s3 = abs(self.distance_between_dots(l, w) - self.PALLET_DIAG)
                        if (s1 <= self.tolerance and s2 <= self.tolerance and s3 <= self.tolerance):
                            # return (i, w, l, d)
                            final_candidates.append((i, w, l, d))
                            errors.append((s1 ** 2 + s2 ** 2 + s3 ** 2) ** 0.5)

        
        if len(errors) == 0:
            return (-1, -1, -1, -1)
        min_value = min(errors)
        min_index = errors.index(min_value)
        return candidates[min_index]
            

    def scan_callback(self, msg):
        self.angle_increment = msg.angle_increment
        self.angle_max = msg.angle_max
        self.ranges = msg.ranges[::-1]

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
        self.image_height = msg.height
        self.image_width = msg.width

    def start(self):
        while not rospy.is_shutdown():
            if self.ranges is not None:
                i, w, l, d = self.search_pallet()
                # i, w, l = self.search_pallet()
                if w != -1:
                    #print (f'Pallet was found at {w}, {l}, {d}')
                    if self.image is not None:
                        self.image = cv2.circle(self.image, (i, self.image_height // 2), 5, (0, 255, 255), 2)
                        self.image = cv2.circle(self.image, (w, self.image_height // 2), 5, (255,0,0), 2)
                        self.image = cv2.circle(self.image, (l, self.image_height // 2), 5, (0,255,0), 2)
                        self.image = cv2.circle(self.image, (d, self.image_height // 2), 5, (0,0,255), 2)
                # else:
                #     print ("No pallet was found!")
            if self.image is not None:
                # self.image = cv2.line(self.image, (0, self.image_height // 2), \
                # (self.image_width, self.image_height // 2), (128, 128, 0), 2)
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.image, "rgb8"))
            self.loop_rate.sleep()




if __name__ == "__main__":
    rospy.init_node('DETECT_PALLET', anonymous=True)
    my_node = SearchImage()
    my_node.start()
    # with open('ranges', 'r') as fp:
    #     ranges = [float(x) for x in fp.read().split(", ")]
    









