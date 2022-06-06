import cv2
import math

PALLET_WIDTH = 0.39
PALLET_LENGTH = 0.41
PALLET_DIAG = (PALLET_WIDTH ** 2 + PALLET_LENGTH ** 2) ** 0.5
angle_increment = 0.0015031362418085337
tolerance = 0.05
range_min = 0.45
range_max = 10.0
increment = (range_max - range_min) / 180


with open('ranges', 'r') as input:
    ranges = [float(i) for i in input.read().split()]

def angle_of_dot(index):
        return angle_increment * (index - len(ranges) // 2)

def xy_of_dot(index, dist_value):
        angle = angle_of_dot(index)
        return dist_value * math.sin(angle), dist_value * math.cos(angle)

def distance_between_dots(first, second):
        x1, y1 = xy_of_dot(first, ranges[first])
        x2, y2 = xy_of_dot(second, ranges[second])
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

def search_pallet():
        for i in range(len(ranges)):
            dist = ranges[i]
            if dist > 2:
                continue
            #angle_increment = msg.angle_increment
            if dist < PALLET_DIAG:
                beta = 0.44
            else:
                beta = math.asin(PALLET_DIAG / dist)
            min_i = int(max(0, i - beta // angle_increment))
            max_i = int(min(len(ranges), i + beta // angle_increment))
            width_candidates = []
            length_candidates = []
            diag_candidates = []
            for j in range (min_i, max_i):
                s = distance_between_dots(i, j)
                if abs(s - PALLET_WIDTH) <= tolerance:
                    width_candidates.append(j)
                if abs(s - PALLET_LENGTH) <= tolerance:
                    length_candidates.append(j)
                if abs(s - PALLET_DIAG) <= tolerance:
                    diag_candidates.append(j)
            for w in width_candidates:
                for l in length_candidates:
                    for d in diag_candidates:
                        s1 = distance_between_dots(w, d)
                        s2 = distance_between_dots(l, d)
                        if (abs(s1 - PALLET_LENGTH) <= tolerance and
                            abs(s2 - PALLET_WIDTH) <= tolerance):
                            return (w, l, d)
            return (-1, -1, -1)


img = cv2.imread('frame.jpg')


# img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# for i in range(len(ranges)):
#     color = ranges[i] // increment
#     img = cv2.circle(img, (i, img.shape[0] // 2), 1, (color, 255, 255), 1)
#     if i == len(ranges) // 2:
#         img = cv2.putText(img, str(round(ranges[i], 3)), (img.shape[1] // 2, img.shape[0] // 2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
# img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)

w, l, d = search_pallet()
if w != -1:
    print (f'Pallet was found at {w}, {l}, {d}')
    if img is not None:
        img = cv2.circle(img, (w, img.shape[1] // 2), 5, (255,0,0), 2)
        img = cv2.circle(img, (l, img.shape[1] // 2), 5, (0,255,0), 2)
        img = cv2.circle(img, (d, img.shape[1] // 2), 5, (0,0,255), 2)
else:
    print ("No pallet was found!")



cv2.imshow('Image', img)
cv2.waitKey(0)
