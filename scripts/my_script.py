import cv2
import math
import matplotlib.pyplot as plt



with open('ranges', 'r') as input:
    ranges = [float(i) for i in input.read().split()]
    ranges = ranges[::-1]

norm_ranges = [0. if math.isnan(x) else x for x in ranges]


plt.plot(norm_ranges, 'r.')
plt.show()


img = cv2.imread('frame.jpg')
cv2.imshow('Image', new_image)
cv2.waitKey(0)
