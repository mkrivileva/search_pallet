import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2

rospy.init_node('computer_vision_sample')
bridge = CvBridge()

img = bridge.imgmsg_to_cv2(rospy.wait_for_message('/camera/color/image_raw', Image), 'bgr8')
msg = rospy.wait_for_message('/scan', LaserScan)
with open('params', 'w') as params, open('ranges', 'w') as ranges:
    params.write("angle_min: " + str(msg.angle_min))
    params.write("angle_max: " + str(msg.angle_max))
    params.write("angle_increment: " + str(msg.angle_increment))
    params.write("range_min: " + str(msg.range_min))
    params.write("range_max: " + str(msg.range_max))
    ranges.write(' '.join(str(j) for j in msg.ranges))
cv2.imwrite('frame.jpg', img)