#!/usr/bin/env python
# Author: Phoenix Fan
# Modification: Measure offset
# Data: 2019-05-14

# Import packages
import os
import cv2
import numpy as np
# import math
import tensorflow as tf
import sys
import socket
import time
from PlateRecognition import PlateRecognition
from PostureAnalysis import PostureAnalysis

# path = './image_test/test.png'
# frame = cv2.imread(path, cv2.IMREAD_COLOR)
# height, width, channles = frame.shape
# cv2.imshow('original image', frame)
# if cv2.waitKey(1) == ord('c'):
#     cv2.destroyWindow('original image')

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the port
# TCP_IP = socket.gethostbyname(socket.gethostname())
TCP_IP = '127.0.0.1'
server_address = (TCP_IP, 10010)
print(sys.stderr, 'starting up on %s port %s' % server_address)
sock.bind(server_address)

# Listen for incoming connections
sock.listen(1)

connection, client_address = sock.accept()
print("Successfully connect to RC client.")
BUFFER_SIZE = 128
message = connection.recv(BUFFER_SIZE).decode()
connection.send(message.encode())

# designate start degree
start_degree = 89
message = connection.recv(BUFFER_SIZE).decode()
if int(message[0]) == 1:
    message = '%d' % start_degree
    connection.send(message.encode())

start_degree -= 1

message = connection.recv(BUFFER_SIZE).decode()
# if int(message[0]) != 2:
#     print("Fail")
#     sys.exit()
# else:
#     print("RC successfully moved from 301 to 300.")

# Initialize webcam feed
# video = cv2.VideoCapture(cv2.CAP_VFW)
CAMERA_OFFSET = 0.0745
VIDEO_WIDTH = 1280
VIDEO_HEIGHT = 720
video = cv2.VideoCapture(0)
video.set(3, VIDEO_WIDTH)  # set the width of video
video.set(4, VIDEO_HEIGHT)  # set the height of video
ret = False
frame = None
while ret is False:
    ret, frame = video.read()
ret = False
height, width, channles = frame.shape

# Define the order of picking up components
def order_component(score, box, prior):
    min_x = 1.0
    prior.clear()
    prior.append(0)
    i = 0
    while score[0, i] > 0.1:
        if box[0, i, 1] < min_x:
            prior[0] = i
            min_x = box[0, i, 1]
        i += 1


H_RULER = 0.6286337930350233  # 0.4714753447762675
V_RULER = 0.4714753447762675  # 0.6286337930350233
VIDEO_WIDTH = 1280
VIDEO_HEIGHT = 720

# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("..")

# Import utilities
from utils import label_map_util

# from utils import visualization_utils as vis_util

# Name of the directory containing the object detection module we're using
MODEL_NAME = 'inference_graph'

# Grab path to current working directory
CWD_PATH = os.getcwd()

# Path to frozen detection graph .pb file, which contains the model that is used
# for object detection.
PATH_TO_CKPT = os.path.join(CWD_PATH, MODEL_NAME, 'frozen_inference_graph.pb')

# Path to label map file
PATH_TO_LABELS = os.path.join(CWD_PATH, 'training', 'labelmap.pbtxt')

# Number of classes the object detector can identify
NUM_CLASSES = 2

# Load the label map.
# Label maps map indices to category names, so that when our convolution
# network predicts `5`, we know that this corresponds to `king`.
# Here we use internal utility functions, but anything that returns a
# dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Load the Tensorflow model into memory.
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')
    sess = tf.Session(graph=detection_graph)

# Define input and output tensors (i.e. data) for the object detection classifier

# Input tensor is the image
image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

# Output tensors are the detection boxes, scores, and classes
# Each box represents a part of the image where a particular object was detected
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

# Each score represents level of confidence for each of the objects.
# The score is shown on the result image, together with the class label.
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

# Number of objects detected
num_detections = detection_graph.get_tensor_by_name('num_detections:0')

boxes, classes, num = 0.0, 0.0, 0.0
scores = np.zeros((1, 300,), dtype=np.float32)

# define the tolerance
ACCURACY = 0.90
TOLERANCE = 0.009
CannotFindComponent = 0

initial_flag = True

while True:
    command = 9  # test
    if command == 9:  # start detection
        print("after plate detection.")
        time.sleep(5)
        while ret is False:
            print('read image from video - object detection 1')
            ret, frame = video.read()
        ret = False
        # cv2.imshow('test', frame)
        # if cv2.waitKey(1) == ord('q'):
        #     break
        print("before object detection.")

        frame_expanded = np.expand_dims(frame, axis=0)
        (boxes, scores, classes, num) = sess.run([detection_boxes, detection_scores, detection_classes, num_detections],
                                                 feed_dict={image_tensor: frame_expanded})
        print("after object detection.")
        if scores[0, 0] < ACCURACY:
            try_count = 0
            while try_count < 5:  # The maximum times of attempting
                try_count += 1
                while ret is False:
                    print('read image from video - object detection 2')
                    ret, frame = video.read()
                ret = False
                print('finish reading image from video - object detection 2')
                # cv2.imshow('test', frame)
                # if cv2.waitKey(1) == ord('q'):
                #     cv2.destroyWindow('test')
                frame_expanded = np.expand_dims(frame, axis=0)
                print('Start object detection.')
                (boxes, scores, classes, num) = sess.run([detection_boxes, detection_scores, detection_classes, num_detections],
                                                         feed_dict={image_tensor: frame_expanded})
                print('Finish object detection.')
                # comp_rcnn(frame)
                if scores[0, 0] >= ACCURACY:
                    break
            else:  # No object is detected, wait for the next command.
                # sock.send('7'.encode())  # '7': 'No component is detected.'
                continue
        # A component is detected.
        print("before detecting posture.")
        pick_prior = []
        order_component(scores, boxes, pick_prior)
        img = frame[int(boxes[0, pick_prior[0], 0] * height):int(boxes[0, pick_prior[0], 2] * height), int(boxes[0, pick_prior[0], 1] * width):int(boxes[0, pick_prior[0], 3] * width), :]
        sub_image_width = abs(int(boxes[0, pick_prior[0], 0] * height) - int(boxes[0, pick_prior[0], 2] * height))
        sub_image_height = abs(int(boxes[0, pick_prior[0], 1] * width) - int(boxes[0, pick_prior[0], 3] * width))
        # test
        # cv2.circle(frame, (int(boxes[0, 0, 1]*width), int(boxes[0, 0, 0]*height)), 5, (255, 0, 0), 5)
        # cv2.imshow("box[0,0] point", frame)
        # cv2.waitKey(0)
        # cv2.destroyWindow("box[0,0] point")
        # test end
        pa = PostureAnalysis(img)
        # h_pivot = boxes[0, pick_prior[0], 1] + pa.grasping_point[0] / VIDEO_WIDTH
        # v_pivot = boxes[0, pick_prior[0], 0] + pa.grasping_point[1] / VIDEO_HEIGHT
        # h_pivot = int(round(boxes[0, pick_prior[0], 1] * VIDEO_WIDTH + pa.grasping_point[0]))
        # v_pivot = int(round(boxes[0, pick_prior[0], 0] * VIDEO_HEIGHT + pa.grasping_point[1]))
        h_pivot = boxes[0, pick_prior[0], 1] + pa.grasping_point[0] / VIDEO_WIDTH
        v_pivot = boxes[0, pick_prior[0], 0] + pa.grasping_point[1] / VIDEO_HEIGHT
        # test
        # test
        min_border = np.zeros((4, 2), dtype=int)  # declare the vertices of the minimum border
        min_border[0] = (int(boxes[0, pick_prior[0], 1] * width) + pa.box[0, 0], int(boxes[0, pick_prior[0], 0] * height) + pa.box[0, 1])
        min_border[1] = (int(boxes[0, pick_prior[0], 1] * width) + pa.box[1, 0], int(boxes[0, pick_prior[0], 0] * height) + pa.box[1, 1])
        min_border[2] = (int(boxes[0, pick_prior[0], 1] * width) + pa.box[2, 0], int(boxes[0, pick_prior[0], 0] * height) + pa.box[2, 1])
        min_border[3] = (int(boxes[0, pick_prior[0], 1] * width) + pa.box[3, 0], int(boxes[0, pick_prior[0], 0] * height) + pa.box[3, 1])
        cv2.line(frame, tuple(min_border[0]), tuple(min_border[1]), (0, 0, 255), 2)
        cv2.line(frame, tuple(min_border[1]), tuple(min_border[2]), (0, 0, 255), 2)
        cv2.line(frame, tuple(min_border[2]), tuple(min_border[3]), (0, 0, 255), 2)
        cv2.line(frame, tuple(min_border[3]), tuple(min_border[0]), (0, 0, 255), 2)
        cv2.circle(frame, (int(boxes[0, pick_prior[0], 1] * width), int(boxes[0, pick_prior[0], 0] * height)), 5, (0, 255, 0), 5)  # fiducial point
        grab_point = (int(round(h_pivot * VIDEO_WIDTH)), int(round(v_pivot * VIDEO_HEIGHT)))  # grab point
        # grab_point = (int(h_pivot * VIDEO_HEIGHT), int(v_pivot * VIDEO_WIDTH))  # grab point
        cv2.circle(frame, grab_point, 5, (255, 0, 0), 5)
        cv2.circle(frame, (int(VIDEO_WIDTH / 2), int(VIDEO_HEIGHT / 2 + 156)), 5, (0, 255, 0), 5)
        cv2.line(frame, (int(boxes[0, pick_prior[0], 1] * width), int(boxes[0, pick_prior[0], 0] * height)), (int(boxes[0, pick_prior[0], 1] * width), int(boxes[0, pick_prior[0], 2] * height)), (100, 100, 100), 3)
        cv2.line(frame, (int(boxes[0, pick_prior[0], 1] * width), int(boxes[0, pick_prior[0], 0] * height)), (int(boxes[0, pick_prior[0], 3] * width), int(boxes[0, pick_prior[0], 0] * height)), (100, 100, 100), 3)
        cv2.line(frame, (int(boxes[0, pick_prior[0], 3] * width), int(boxes[0, pick_prior[0], 0] * height)), (int(boxes[0, pick_prior[0], 3] * width), int(boxes[0, pick_prior[0], 2] * height)), (100, 100, 100), 3)
        cv2.line(frame, (int(boxes[0, pick_prior[0], 1] * width), int(boxes[0, pick_prior[0], 2] * height)), (int(boxes[0, pick_prior[0], 3] * width), int(boxes[0, pick_prior[0], 2] * height)), (100, 100, 100), 3)
        angle = pa.gradient * 180 / np.pi
        cv2.line(frame, (int(VIDEO_WIDTH / 2), 0), (int(VIDEO_WIDTH / 2), VIDEO_HEIGHT), (0, 0, 128), 2)
        cv2.line(frame, (0, int(VIDEO_HEIGHT / 2)), (VIDEO_WIDTH, int(VIDEO_HEIGHT / 2)), (0, 0, 128), 2)
        print(pa.gradient)
        print(angle)
        # Y = angle * X + b

        cv2.imwrite("./image/%d.jpg" % -start_degree, frame)
        # cv2.imshow("grasping_point and box00 position", frame)
        # cv2.waitKey(1000)
        # cv2.destroyWindow("grasping_point and box00 position")
        # if()
        print('Offset of grasping point: %d X %d Y %d' % (-start_degree, grab_point[0] - VIDEO_WIDTH / 2, grab_point[1] - (VIDEO_HEIGHT / 2 + 156)))
        with open('offset.txt', 'a') as f:
            f.write('%d %d %d\n' % (-start_degree, int(round(grab_point[0] - VIDEO_WIDTH / 2)), int(round(grab_point[1] - (VIDEO_HEIGHT / 2 + 156)))))
        start_degree -= 1
        # test end
    message = '0'
    connection.send(message.encode())
    message = connection.recv(BUFFER_SIZE).decode()
    if int(message[0] == 9):
        break
    # elif int(message[0] == 3):
    #     with open('singularity.txt', 'a') as f:
    #         f.write('%d\n' % (-start_degree))
    #     message = "3"
    #     connection.send(message.encode())
    #     start_degree -= 1


    # if initial_flag is True:
    #     initial_flag = False
    # else:
    #     message = connection.recv(BUFFER_SIZE).decode()

cv2.destroyAllWindows()
connection.close()
sock.close()
video.release()
