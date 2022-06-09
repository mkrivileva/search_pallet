#! /usr/bin/env python3

from cmath import sin
from operator import length_hint
from re import search
import rospy
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import math
import cv2

class LPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.lenght = self.get_lenght()

    def sub(self, other):
        return LPoint(self.x - other.x, self.y - other.y)

    def fromPoolarCoords(r, phi):
        return LPoint(r * math.cos(phi), r * math.sin(phi))

    def get_lenght(self):
        return (self.x ** 2 + self.y ** 2) ** 0.5


class ObstacleExtractor:
    def __init__ (self):
        self.bridge = CvBridge()

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.img_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/extract_obstacles', Image, queue_size=1)
        self.obstacles_pub = rospy.Publisher('/obstacles_coordinate', Int32MultiArray, queue_size=10)

        self.input_points = []
        self.image = None
        self.p_max_group_distance = 0.02
        self.p_distance_proportion = 0.00628
        self.image_height = 0
        self.image_width = 0
        self.ranges = []
        self.MAX_LENGHT = 2
        self.p_min_group_points = 20

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
        self.image_height = msg.height
        self.image_width = msg.width

    def scan_callback(self, scan_msg):
        # self.ranges = scan_msg.ranges[::-1]
        self.ranges = [10. if math.isnan(x) else x for x in scan_msg.ranges[::-1]]
        phi = -scan_msg.angle_max
        for r in self.ranges:
            if r >= scan_msg.range_min and r <= scan_msg.range_max:
                self.input_points.append(LPoint.fromPoolarCoords(r, phi))
            phi += scan_msg.angle_increment


    def extract_obstacles(self):
        start = 0
        obstacle_centers = [0]

        for i in range( len(self.ranges) ):

            cur_range = self.ranges[i]

            if i == 0:
                continue
            
            prev_range = self.ranges[i - 1]

            if cur_range > self.MAX_LENGHT and prev_range > self.MAX_LENGHT:
                start = i
                continue

            distance = abs(prev_range - cur_range)

            if ((distance >= self.p_max_group_distance and i - obstacle_centers[-1] > self.p_min_group_points)
                 or i + 1 == len(self.ranges)):
                center = (i + start) // 2
                start = i
                obstacle_centers.append(center)
                self.draw_obstacle(center)
        msg_data = Int32MultiArray()
        msg_data.data = obstacle_centers
        self.obstacles_pub.publish(msg_data)


    def draw_obstacle(self, k):
        self.image = cv2.circle(self.image, (k, self.image_height // 2), 5, (0, 255, 255), 2)




    def start(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.extract_obstacles()
            if self.image is not None:
                self.image = cv2.line(self.image, (0, self.image_height // 2), (self.image_width, self.image_height // 2), (128, 128, 0), 2)
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.image, "rgb8"))
            rate.sleep()




if __name__ == '__main__':
    rospy.init_node('obstacles_extractor', anonymous=False)
    extractor = ObstacleExtractor()
    extractor.start()