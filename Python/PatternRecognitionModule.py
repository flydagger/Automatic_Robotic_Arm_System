#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Phoenix Fan
# Modification: Refactor this code to object oriented programming
# Data: 30-05-2019

# Import packages
import os
import cv2
import numpy as np
import tensorflow as tf
import sys
import socket
import time
from PlateRecognition import PlateRecognition
from PostureAnalysis import PostureAnalysis
# Import utilities
from utils import label_map_util

# RECEIVED_CODE = {
#     '0': 'Shutdown.',
#     '8': 'Start plate detection.' - discarded,
#     '9': 'Start object detection.',
# }
#
# SEND_CODE = {
#     '0': 'Invalid command.',
#     '1': 'The object is in the center of the camera. Return the gradient and direction of component',
#     '5': 'The object is not in the center of the camera. Return the coordinate of the object.',
#     '6': 'Check whether the target is in the valid area. If received 'y', continue grasping this target. Otherwise, add this target into the neglected list.',
#     '7': 'No component is detected. The conveyor belt continues moving.',
#     '9': 'Boot successfully.'
#
#   '6': 'A plate is detected. Stop moving the conveyor belt.' - discarded,
#   '8': 'No plate is detected. Start conveyor belt.' - discarded,
# }
#

class SyringeBarrelDetection(object):
    """
    Detect the Barrel of Syringe. Gain the score and coordinates of the object.
    """

    def __init__(self):
        self.H_RULER = 0.64  # 0.4714753447762675
        self.V_RULER = 0.36  # 0.6286337930350233
        self.CAMERA_OFFSET = 0.0745
        self.VIDEO_WIDTH = 1280
        self.VIDEO_HEIGHT = 720
        self.LOCALHOST_IPADDRESS = '127.0.0.1'

        # Initialize webcam feed
        # self.video = cv2.VideoCapture(cv2.CAP_VFW)
        self.video = cv2.VideoCapture(0)
        self.video.set(3, self.VIDEO_WIDTH)  # set the width of self.video
        self.video.set(4, self.VIDEO_HEIGHT)  # set the height of self.video
        self.ret = False
        self.frame = None
        while self.ret is False:
            self.ret, self.frame = self.video.read()
        self.ret = False
        self.height, self.width, self.channles = self.frame.shape

        # This is needed since the notebook is stored in the object_detection folder.
        sys.path.append("..")

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

        self.boxes, self.classes, self.num = 0.0, 0.0, 0.0
        self.scores = np.zeros((1, 300,), dtype=np.float32)

        # define the tolerance
        self.ACCURACY = 0.90
        self.TOLERANCE = 0.005

        # Initialize TCP_IP connection
        self.TCP_PORT = 10000
        self.BUFFER_SIZE = 128
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.LOCALHOST_IPADDRESS, self.TCP_PORT))

        # establish connection with GUI module
        self.message = '9'
        self.sock.send(self.message.encode())

        while True:
            self.message = self.sock.recv(self.BUFFER_SIZE).decode()
            command = int(self.message[0])
            if command == 8 or command == 9:  # start detection
                while self.ret is False:
                    self.ret, self.frame = self.video.read()
                self.ret = False
                frame_expanded = np.expand_dims(self.frame, axis=0)
                (self.boxes, self.scores, self.classes, self.num) = sess.run([detection_boxes, detection_scores, detection_classes, num_detections], feed_dict={image_tensor: frame_expanded})
                if self.scores[0, 0] < self.ACCURACY:
                    try_count = 0
                    while try_count < 5:  # The maximum times of attempting
                        try_count += 1
                        while self.ret is False:
                            self.ret, self.frame = self.video.read()
                        self.ret = False
                        frame_expanded = np.expand_dims(self.frame, axis=0)
                        (self.boxes, self.scores, self.classes, self.num) = sess.run([detection_boxes, detection_scores, detection_classes, num_detections], feed_dict={image_tensor: frame_expanded})
                        if self.scores[0, 0] >= self.ACCURACY:
                            break
                    else:  # No object is detected, wait for the next command.
                        self.sock.send('7'.encode())  # '7': 'No component is detected. The conveyor belt continues moving.'
                        continue
                self.pick_prior = []
                self.orderComponent(self.scores, self.boxes, self.pick_prior)
                for i in range(self.pick_prior.__len__()):
                    self.img = self.frame[int(self.boxes[0, self.pick_prior[i], 0] * self.height):int(self.boxes[0, self.pick_prior[i], 2] * self.height), int(self.boxes[0, self.pick_prior[i], 1] * self.width):int(self.boxes[0, self.pick_prior[i], 3] * self.width), :]
                    # cv2.imshow("img", self.img)
                    # cv2.waitKey()
                    # cv2.destroyWindow("img")
                    self.pa = PostureAnalysis(self.img)
                    self.h_pivot = self.boxes[0, self.pick_prior[i], 1] + self.pa.grasping_point[0] / self.VIDEO_WIDTH
                    self.v_pivot = self.boxes[0, self.pick_prior[i], 0] + self.pa.grasping_point[1] / self.VIDEO_HEIGHT
                    self.postureAnalysisDiagram(self.frame, i)
                    self.v_distance = (self.v_pivot - 0.5) * self.V_RULER - self.CAMERA_OFFSET  # 0.0746m is the distance between the camera and the clip.
                    self.h_distance = (0.5 - self.h_pivot) * self.H_RULER
                    self.message = '6 '
                    if self.h_distance >= 0:
                        self.message += '+'
                    self.message += '%.3f ' % self.h_distance
                    if self.v_distance >= 0:
                        self.message += '+'
                    self.message += '%.3f' % self.v_distance
                    self.sock.send(self.message.encode())
                    valid_area = self.sock.recv(self.BUFFER_SIZE).decode()
                    if int(valid_area[1]) == 0:  # The target is in the valid area.
                        # The pivot is in the center of the camera
                        if abs(self.v_distance) < self.TOLERANCE and abs(self.h_distance) < self.TOLERANCE:
                            if self.pa.gradient < 0:
                                str_gra = '%.3f' % self.pa.gradient
                            else:
                                str_gra = '+' + '%.3f' % self.pa.gradient
                            send_message = '1 ' + str_gra + ' ' + str(self.pa.direction)
                            self.sock.send(send_message.encode())  # '1': 'The object is in the center of the camera. Return the gradient and direction of component'
                            break  # Target is confirmed, posture analysis is complete. So jump out the for loop to stop the process.
                        else:  # The object is not right below the camera. The robot should move. Return a coordinate.
                            if self.v_distance < 0:
                                str_v_dis = '%.3f' % self.v_distance
                            else:
                                str_v_dis = '+' + '%.3f' % self.v_distance
                            if self.h_distance < 0:
                                str_h_dis = '%.3f' % self.h_distance
                            else:
                                str_h_dis = '+' + '%.3f' % self.h_distance
                            send_message = '5 ' + str_h_dis + ' ' + str_v_dis
                            self.sock.send(send_message.encode())  # '5': 'The object is not in the center of the camera. Return the coordinate of the object.'
                            break  # Target is confirmed, position information is replied. So jump out the for loop to stop the process.
                    elif int(valid_area[1]) == 1:  # The target is not in the valid area, so check the next item in pick_prior.
                        continue
            elif command == 0:  # '0': 'Shutdown'
                break
            else:
                self.sock.send('0'.encode())  # '0': 'Invalid command.'
                pass

        cv2.destroyAllWindows()
        self.sock.close()
        self.video.release()

    # Define the order of picking up components
    def orderComponent(self, score, box, prior):
        # marks = score.copy()
        prior.clear()
        for n in range(50):
            min_index = -1
            x = 1.0
            for i in range(300):
                if score[0, i] > 0.8 and i not in prior:
                    if box[0, i, 1] < x:
                        x = box[0, i, 1]
                        min_index = i
            if min_index != -1:
                prior.append(min_index)


    def postureAnalysisDiagram(self, image, num):
        img = image.copy()
        min_border = np.zeros((4, 2), dtype=int)  # declare the vertices of the minimum border
        min_border[0] = (int(self.boxes[0, self.pick_prior[num], 1] * self.width) + self.pa.box[0, 0], int(self.boxes[0, self.pick_prior[num], 0] * self.height) + self.pa.box[0, 1])
        min_border[1] = (int(self.boxes[0, self.pick_prior[num], 1] * self.width) + self.pa.box[1, 0], int(self.boxes[0, self.pick_prior[num], 0] * self.height) + self.pa.box[1, 1])
        min_border[2] = (int(self.boxes[0, self.pick_prior[num], 1] * self.width) + self.pa.box[2, 0], int(self.boxes[0, self.pick_prior[num], 0] * self.height) + self.pa.box[2, 1])
        min_border[3] = (int(self.boxes[0, self.pick_prior[num], 1] * self.width) + self.pa.box[3, 0], int(self.boxes[0, self.pick_prior[num], 0] * self.height) + self.pa.box[3, 1])
        cv2.line(img, tuple(min_border[0]), tuple(min_border[1]), (0, 0, 255), 2)
        cv2.line(img, tuple(min_border[1]), tuple(min_border[2]), (0, 0, 255), 2)
        cv2.line(img, tuple(min_border[2]), tuple(min_border[3]), (0, 0, 255), 2)
        cv2.line(img, tuple(min_border[3]), tuple(min_border[0]), (0, 0, 255), 2)
        cv2.circle(img, (int(self.boxes[0, self.pick_prior[num], 1] * self.width), int(self.boxes[0, self.pick_prior[num], 0] * self.height)), 5, (0, 255, 0), 5)  # fiducial point
        grab_point = (int(round(self.h_pivot * self.VIDEO_WIDTH)), int(round(self.v_pivot * self.VIDEO_HEIGHT)))  # grab point
        cv2.circle(img, grab_point, 5, (255, 0, 0), 5)
        cv2.circle(img, (int(self.VIDEO_WIDTH / 2), int(self.VIDEO_HEIGHT / 2 + 156)), 5, (0, 255, 0), 5)  # center point, the actual grasp point
        cv2.line(img, (int(self.boxes[0, self.pick_prior[num], 1] * self.width), int(self.boxes[0, self.pick_prior[num], 0] * self.height)), (int(self.boxes[0, self.pick_prior[num], 1] * self.width), int(self.boxes[0, self.pick_prior[num], 2] * self.height)), (100, 100, 100), 3)
        cv2.line(img, (int(self.boxes[0, self.pick_prior[num], 1] * self.width), int(self.boxes[0, self.pick_prior[num], 0] * self.height)), (int(self.boxes[0, self.pick_prior[num], 3] * self.width), int(self.boxes[0, self.pick_prior[num], 0] * self.height)), (100, 100, 100), 3)
        cv2.line(img, (int(self.boxes[0, self.pick_prior[num], 3] * self.width), int(self.boxes[0, self.pick_prior[num], 0] * self.height)), (int(self.boxes[0, self.pick_prior[num], 3] * self.width), int(self.boxes[0, self.pick_prior[num], 2] * self.height)), (100, 100, 100), 3)
        cv2.line(img, (int(self.boxes[0, self.pick_prior[num], 1] * self.width), int(self.boxes[0, self.pick_prior[num], 2] * self.height)), (int(self.boxes[0, self.pick_prior[num], 3] * self.width), int(self.boxes[0, self.pick_prior[num], 2] * self.height)), (100, 100, 100), 3)
        angle = self.pa.gradient * 180 / np.pi
        cv2.line(img, (int(self.VIDEO_WIDTH / 2), 0), (int(self.VIDEO_WIDTH / 2), self.VIDEO_HEIGHT), (0, 0, 128), 2)
        cv2.line(img, (0, int(self.VIDEO_HEIGHT / 2)), (self.VIDEO_WIDTH, int(self.VIDEO_HEIGHT / 2)), (0, 0, 128), 2)
        print("gradient: %s" % self.pa.gradient)
        print("angle: %s" % angle)
        cv2.imshow("Illustration", img)
        if cv2.waitKey(1) == ord('q'):
            cv2.destroyWindow("Illustration")

if __name__ == '__main__':
    sbd = SyringeBarrelDetection()
