import cv2
import numpy as np
from matplotlib import pyplot as plt

# img = cv2.imread('src1.jpg', 0)
video = cv2.VideoCapture(0)

# If the plate is detected, turn plate_flag to True. Otherwise, plate_flag keeps False.
plate_flag = False

# Initialize the center position and direction of component.
x_center, y_center = 0.0, 0.0
direction = 0  # 0 represent left, 1 represent right

ret, frame = video.read()
height, width, channels = frame.shape

# Initialize the edge of four direction.
# In OpenCV image coordinate [x, y], x is height (vertical) and y is width (horizontal).
# In OpenCV point coordinate [x, y], x is width (horizontal) and y is height (vertical).
top_point = [0, height - 1]
bottom_point = [0, 0]
left_point = [width - 1, 0]
right_point = [0, 0]

while True:
    ret, frame = video.read()
    height, width, channels = frame.shape
    if not ret:
        break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)
    low_threshold = 50
    high_threshold = 150
    edges = cv2.Canny(blur_gray, low_threshold, high_threshold)
    cv2.imshow('test', edges)
    if cv2.waitKey(1) == ord('q'):
        break
    for h in range(len(edges)):
        for w in range(len(edges[0])):
            if edges[h, w] > 0:  # Bigger than 0 means this point is on the edge.
                if h < top_point[1]:
                    top_point = [w, h]
                if h > bottom_point[1]:
                    bottom_point = [w, h]
                if w < left_point[0]:
                    left_point = [w, h]
                if w > right_point[0]:
                    right_point = [w, h]


    # for v in edges:
    #     print(type(v))
    #     print(v.shape)
    #     print(len(edges))

cv2.destroyWindow('test')
video.release()
