# -*- coding: utf-8 -*-
"""
Author: Yixiang Fan
Company: Antikythera Robotics Pty Ltd
Version: 1.2
Last modify date: 20/02/2019
Modifications: 1. Remove the rotation of input image.
               2. Modify slopeRatio function.
"""
import cv2
import numpy as np
import math


def pointInRectangle(test_point, sub_rectangle, inclination):
    """
    Use Cross product of Vector to determine if a point is in a rectangle.
    The cross product of adjacent vectors should be positive.
    For example:
        The cross product of the vector from the given point to vertex 0 and the vector from the given point to vertex 1 should be positive.
    :param test_point:
    :param sub_rectangle:
    :param inclination: Left slant is 1. Right slant is -1.
    :return: True: the given point is in the rectangle.
              False: the given point is not in the rectangle.
    """
    v0 = (sub_rectangle[0, 0] - test_point[0], sub_rectangle[0, 1] - test_point[1])
    v1 = (sub_rectangle[1, 0] - test_point[0], sub_rectangle[1, 1] - test_point[1])
    v2 = (sub_rectangle[2, 0] - test_point[0], sub_rectangle[2, 1] - test_point[1])
    v3 = (sub_rectangle[3, 0] - test_point[0], sub_rectangle[3, 1] - test_point[1])
    if inclination * (v0[0] * v1[1] - v0[1] * v1[0]) < 0:
        if inclination * (v1[0] * v2[1] - v1[1] * v2[0]) < 0:
            if inclination * (v2[0] * v3[1] - v2[1] * v3[0]) < 0:
                if inclination * (v3[0] * v0[1] - v3[1] * v0[0]) < 0:
                    return True
    return False


class PostureAnalysis(object):
    """
    Analyze the posture information of an component image.
    Obtain the slope and direction.
    """

    OFFSET_CENTER = ((6, -19),
                     (7, -17),
                     (8, -14),
                     (7, -17),
                     (7, -20),
                     (8, -18),
                     (9, -19),
                     (8, -19),
                     (8, -19),
                     (9, -20),
                     (10, -20),
                     (10, -16),
                     (10, -16),
                     (10, -17),
                     (10, -17),
                     (10, -17),
                     (10, -17),
                     (10, -17),
                     (10, -17),
                     (10, -18),
                     (10, -18),
                     (10, -16),
                     (10, -18),
                     (10, -19),
                     (10, -19),
                     (11, -18),
                     (11, -19),
                     (10, -18),
                     (10, -18),
                     (10, -19),
                     (10, -19),
                     (10, -18),
                     (10, -19),
                     (9, -18),
                     (10, -18),
                     (10, -19),
                     (9, -18),
                     (9, -19),
                     (10, -19),
                     (10, -19),
                     (9, -19),
                     (10, -19),
                     (10, -19),
                     (10, -19),
                     (10, -19),
                     (9, -19),
                     (10, -19),
                     (10, -20),
                     (10, -20),
                     (11, -20),
                     (11, -21),
                     (11, -20),
                     (11, -20),
                     (12, -21),
                     (12, -21),
                     (12, -22),
                     (12, -21),
                     (12, -21),
                     (12, -21),
                     (12, -22),
                     (12, -21),
                     (12, -21),
                     (12, -21),
                     (13, -22),
                     (13, -22),
                     (13, -22),
                     (14, -23),
                     (13, -23),
                     (14, -22),
                     (13, -23),
                     (15, -23),
                     (13, -23),
                     (14, -23),
                     (15, -23),
                     (17, -24),
                     (15, -23),
                     (16, -24),
                     (16, -24),
                     (16, -24),
                     (19, -24),
                     (18, -25),
                     (19, -24),
                     (20, -24),
                     (21, -24),
                     (20, -25),
                     (19, -25),
                     (22, -26),
                     (20, -26),
                     (20, -28),
                     (21, -27),
                     (22, -24),
                     (20, -22),
                     (21, -20),
                     (21, -22),
                     (21, -21),
                     (21, -22),
                     (20, -22),
                     (20, -22),
                     (20, -21),
                     (17, -23),
                     (18, -23),
                     (17, -23),
                     (18, -24),
                     (19, -23),
                     (19, -23),
                     (19, -23),
                     (20, -25),
                     (18, -26),
                     (19, -25),
                     (20, -26),
                     (20, -25),
                     (21, -24),
                     (21, -24),
                     (20, -24),
                     (21, -23),
                     (22, -23),
                     (22, -23),
                     (22, -23),
                     (23, -23),
                     (22, -23),
                     (23, -22),
                     (23, -23),
                     (23, -23),
                     (23, -23),
                     (22, -27),
                     (21, -25),
                     (22, -26),
                     (23, -25),
                     (23, -25),
                     (24, -25),
                     (25, -25),
                     (25, -24),
                     (25, -25),
                     (25, -24),
                     (25, -24),
                     (25, -23),
                     (26, -24),
                     (25, -24),
                     (26, -23),
                     (26, -23),
                     (25, -23),
                     (25, -24),
                     (27, -22),
                     (26, -22),
                     (26, -22),
                     (27, -22),
                     (27, -21),
                     (26, -22),
                     (27, -22),
                     (27, -21),
                     (27, -21),
                     (27, -21),
                     (27, -20),
                     (29, -21),
                     (27, -20),
                     (27, -21),
                     (27, -21),
                     (27, -20),
                     (28, -20),
                     (27, -20),
                     (27, -20),
                     (28, -20),
                     (27, -20),
                     (28, -20),
                     (28, -19),
                     (29, -17),
                     (28, -19),
                     (28, -18),
                     (28, -17),
                     (28, -17),
                     (28, -17),
                     (28, -17),
                     (27, -17),
                     (27, -17),
                     (26, -17),
                     (26, -17),
                     (25, -17),
                     (25, -17),
                     (24, -18),
                     (23, -18),
                     (23, -18),
                     (22, -17),
                     (23, -19),
                     (22, -17),
                     (22, -15),
                     (24, -18),
                     (24, -16),
                     (25, -15),
                     (24, -15),
                     (25, -13),
                     (24, -13),
                     (24, -11),
                     (25, -13),
                     (25, -14),
                     (25, -14),
                     (25, -15),
                     (25, -14),
                     (25, -13),
                     (25, -13),
                     (25, -11),
                     (26, -13),
                     (25, -14),
                     (25, -13),
                     (29, -11),
                     (26, -12),
                     (25, -12),
                     (25, -13),
                     (25, -13),
                     (25, -12),
                     (25, -13),
                     (25, -13),
                     (25, -12),
                     (25, -11),
                     (25, -11),
                     (25, -11),
                     (25, -11),
                     (25, -11),
                     (24, -11),
                     (23, -11),
                     (24, -11),
                     (24, -10),
                     (23, -9),
                     (24, -10),
                     (24, -10),
                     (24, -10),
                     (23, -8),
                     (21, -7),
                     (23, -9),
                     (23, -9),
                     (25, -9),
                     (23, -8),
                     (22, -8),
                     (23, -8),
                     (23, -8),
                     (23, -7),
                     (23, -8),
                     (23, -5),
                     (24, -7),
                     (21, -7),
                     (20, -7),
                     (22, -7),
                     (22, -8),
                     (20, -7),
                     (22, -8),
                     (22, -8),
                     (21, -7),
                     (21, -7),
                     (21, -8),
                     (21, -7),
                     (19, -6),
                     (18, -6),
                     (19, -7),
                     (19, -7),
                     (20, -8),
                     (18, -7),
                     (18, -7),
                     (18, -7),
                     (20, -7),
                     (16, -6),
                     (16, -6),
                     (15, -6),
                     (19, -6),
                     (18, -5),
                     (18, -5),
                     (18, -5),
                     (18, -5),
                     (18, -5),
                     (18, -5),
                     (18, -6),
                     (18, -6),
                     (18, -6),
                     (18, -6),
                     (16, -7),
                     (16, -7),
                     (15, -8),
                     (14, -8),
                     (14, -9),
                     (11, -9),
                     (11, -8),
                     (13, -8),
                     (13, -8),
                     (13, -7),
                     (14, -8),
                     (15, -8),
                     (15, -8),
                     (15, -8),
                     (12, -9),
                     (14, -8),
                     (14, -10),
                     (13, -8),
                     (13, -8),
                     (11, -10),
                     (14, -10),
                     (14, -8),
                     (13, -9),
                     (13, -9),
                     (13, -10),
                     (14, -9),
                     (15, -8),
                     (14, -10),
                     (14, -9),
                     (15, -9),
                     (14, -9),
                     (14, -9),
                     (13, -10),
                     (15, -10),
                     (13, -10),
                     (14, -9),
                     (13, -11),
                     (13, -10),
                     (13, -11),
                     (13, -11),
                     (13, -9),
                     (11, -11),
                     (11, -13),
                     (12, -10),
                     (12, -10),
                     (11, -10),
                     (10, -12),
                     (10, -12),
                     (10, -11),
                     (11, -12),
                     (11, -12),
                     (10, -12),
                     (8, -13),
                     (9, -13),
                     (10, -12),
                     (10, -11),
                     (9, -12),
                     (9, -14),
                     (9, -14),
                     (8, -15),
                     (9, -13),
                     (9, -12),
                     (9, -12),
                     (9, -14),
                     (9, -14),
                     (8, -19),
                     (8, -19),
                     (9, -15),
                     (9, -16),
                     (9, -16),
                     (9, -15),
                     (8, -18),
                     (7, -21),
                     (6, -21),
                     (7, -20),
                     (7, -19),
                     (7, -19),
                     (6, -24),
                     (7, -20),
                     (8, -19),
                     (8, -21),
                     (8, -22),
                     (8, -19),
                     (8, -21),
                     (8, -19),
                     (8, -21),
                     (8, -21),
                     (8, -21),
                     (8, -20))

    def __init__(self, img):
        """
        Analyze the posture and slope of component.
        :param img: The rectangle enclosing the component detected by F-RNN module.
        """
        # img = np.asarray(img)
        self.point = np.zeros((4, 2), dtype=int)
        self.sub_rectangle_list = np.array((4, 4), dtype=int)
        # img_rotated = self.imageRotation(img)
        # test
        # cv2.imwrite("rotated image.png", img_rotated)
        # cv2.imshow('img_rotate', img_rotated)
        # cv2.waitKey(0)
        # cv2.destroyWindow('img_rotate')
        # test end
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # img_gray = img_rotated
        # test
        # cv2.imshow('img_gray', img_gray)
        # cv2.waitKey(0)
        # cv2.destroyWindow('img_gray')
        # test end
        self.height, self.width = img_gray.shape
        # self.gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        GaussianBlur_KERNEL = 3
        blur_gray = cv2.GaussianBlur(img_gray, (GaussianBlur_KERNEL, GaussianBlur_KERNEL), 0)
        LOW_THRESHOLD = 50
        HIGH_THRESHOLD = 150
        self.edges = cv2.Canny(blur_gray, LOW_THRESHOLD, HIGH_THRESHOLD)
        # test
        # cv2.imwrite('edges before erosion and dilation.png', self.edges)
        # cv2.imshow('edges before erosion and dilation', self.edges)
        # cv2.waitKey(0)
        # cv2.destroyWindow('edges before erosion and dilation')
        # test end
        self.Erosion_KERNEL = np.ones((3, 3), np.uint8)
        self.Dilation_KERNEL = np.ones((3, 3), np.uint8)
        self.edges = cv2.dilate(self.edges, self.Dilation_KERNEL, iterations=2)
        # test
        # cv2.imshow('edges after dilation', self.edges)
        # cv2.waitKey(0)
        # cv2.destroyWindow('edges after dilation')
        # cv2.imwrite('edges after dilation.png', self.edges)
        # test end
        im2, contours, hierarchy = cv2.findContours(self.edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        areaMax, indexMax = 0.0, 0
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if area == 0:
                continue
            # print('i: ', i, ', area: ', area)
            if area > areaMax:
                areaMax = area
                indexMax = i
        rect = cv2.minAreaRect(contours[indexMax])
        self.box = cv2.boxPoints(rect)
        self.box = np.asarray(self.box, dtype=int)
        self.edges = cv2.erode(self.edges, self.Erosion_KERNEL, iterations=2)
        # test
        # cv2.line(self.edges, tuple(self.box[0]), tuple(self.box[1]), (255, 255, 255), 1)
        # cv2.line(self.edges, tuple(self.box[1]), tuple(self.box[2]), (255, 255, 255), 1)
        # cv2.line(self.edges, tuple(self.box[2]), tuple(self.box[3]), (255, 255, 255), 1)
        # cv2.line(self.edges, tuple(self.box[3]), tuple(self.box[0]), (255, 255, 255), 1)
        # cv2.imshow('edges_with_box', self.edges)
        # cv2.waitKey(0)
        # cv2.destroyWindow('edges_with_box')
        # test end
        # gradient(radian) : -3.05433 to +3.05433 ; refer to opposite y axis; the bottom of component is at axis origin.
        # If the length is clockwise to the opposite y axis, the gradient is negative.
        # Otherwise the gradient is positive.
        self.gradient = 0.0
        # direction: True represents left
        #            False represents right
        self.direction = 0
        self.grasping_point = np.zeros([2], dtype=float)
        self.directionDetection()
        self.grasping_point[0] -= self.OFFSET_CENTER[int(round(self.gradient * 180 / np.pi)) + 180][0]
        self.grasping_point[1] -= (self.OFFSET_CENTER[int(round(self.gradient * 180 / np.pi)) + 180][1])

    def pointOrderSlantRectangle(self):
        """
        Order the vertices: top - point0
                            short edge (width) to top - point1
                            diagonal to top - point2
                            long edge (length) to top - point3
        :return: A list containing points in clockwise order.
        """
        i_list = [0, 1, 2, 3]
        i0 = np.argmin(self.box[:, 1])
        i2 = np.argmax(self.box[:, 1])
        self.point[0] = self.box[i0].copy()
        self.point[2] = self.box[i2].copy()
        i_list.remove(int(i0))
        i_list.remove(int(i2))
        # distance0 and distance1 are the square of the length of the edge between some vertices
        int(self.box[i_list[0], 0])
        distance0 = pow(int(self.box[i_list[0], 0]) - int(self.point[0, 0]), 2) + pow(int(self.box[i_list[0], 1]) - int(self.point[0, 1]), 2)
        distance1 = pow(int(self.box[i_list[1], 0]) - int(self.point[0, 0]), 2) + pow(int(self.box[i_list[1], 1]) - int(self.point[0, 1]), 2)
        # # distance0 is either bigger or smaller than distance1,
        # # because the minimum frame enclosing the component is a rectangle.
        if distance0 < distance1:
            self.point[1] = self.box[i_list[0]].copy()
            self.point[3] = self.box[i_list[1]].copy()
        else:
            self.point[1] = self.box[i_list[1]].copy()
            self.point[3] = self.box[i_list[0]].copy()

    def pointOrderPerpendicularRectangle(self):
        """
        Order the vertices: top left - point 0
                            top right - point 1
                            bottom left - point 2
                            bottom right - point 3
        :return: A list containing point in clockwise order.
        """
        i_list = [0, 1, 2, 3]
        point_radius = np.zeros((4,), dtype=np.float32)
        for i in range(4):
            point_radius[i] = np.square(self.box[i, 0]) + np.square(self.box[i, 1])
        i0 = np.argmin(point_radius)
        i2 = np.argmax(point_radius)
        self.point[0] = self.box[i0].copy()
        self.point[2] = self.box[i2].copy()
        i_list.remove(int(i0))
        i_list.remove(int(i2))
        if self.box[i_list[0], 0] > self.box[i_list[1], 0]:
            self.point[1] = self.box[i_list[0]].copy()
            self.point[3] = self.box[i_list[1]].copy()
        else:
            self.point[1] = self.box[i_list[1]].copy()
            self.point[3] = self.box[i_list[0]].copy()

    @staticmethod
    def yCoordinateOfPointOnLine(x, line):
        """
        Return the y-axis coordinate of the line regarding to given a x-axis coordinate.
        :param x: The given x-axis coordinate.
        :param line: A line represented by two points. A 2d list.
        :return: Y axis coordinate.
        """
        _y = (x - line[0, 0]) * (line[1, 1] - line[0, 1]) / (line[1, 0] - line[0, 0]) + line[0, 1]
        return int(_y)

    def perpendicularToAxis(self):
        """
        Check if the rectange is nearly perpendicular to any axis.
        :return: True: The rectangle is perpendicular to axis.
                 False: The rectangle is not perpendicular to axis.
        """
        x_list = self.box[:, 0]
        y_list = self.box[:, 1]
        x_list = np.sort(x_list)
        for i in range(3):
            if x_list[i + 1] - x_list[i] < 3:
                return True
        y_list = np.sort(y_list)
        for i in range(3):
            if y_list[i + 1] - y_list[i] < 3:
                return True
        return False

    @staticmethod
    def slopeRatio(head, end):
        """
        Calculate the angle of a vector (head, end) refer to the right above.
        :param head: The start point of the vector.
        :param end: The end point of the vector.
        :return: Return the angle in radian. The angle is limited from -175 to 175 in degree, namely -3.05433 to 3.05433.
                  Right upward is the 0 degree. Right side is positive, left side is negative.
        """
        if end[0] > head[0] and end[1] > head[1]:  # The vector faces low-right.
            return math.atan((end[1] - head[1]) / (end[0] - head[0])) + 1.5708
        elif end[0] > head[0] and end[1] < head[1]:  # The vector faces up-right.
            return math.atan((end[0] - head[0]) / (head[1] - end[1]))
        elif end[0] < head[0] and end[1] < head[1]:  # The vector faces up-left.
            return -math.atan((head[0] - end[0]) / (head[1] - end[1]))
        else:  # The vector faces low-left
            return -math.atan((end[1] - head[1]) / (head[0] - end[0])) - 1.5708

    def directionDetection(self):
        """
        Detect the direction of the target component and calculate the grasping point of the component.
        :return: 1: The target component orients left.
                 0: The target component orients right.
        """
        height, width = self.edges.shape
        countPointsInSubRectangle = np.zeros((4,), dtype=np.int)  # store the number of points in each sub-rectangle
        sub_point_list = np.zeros((4, 4, 2), dtype=int)  # (4, 4, 2) means 4 sub-rectangle, 4 points, 2 axis
        # Check if the rectangle is perpendicular to axis of the image.
        if self.perpendicularToAxis():  # The rectangle is perpendicular to axises.
            self.pointOrderPerpendicularRectangle()
            for i in range(4):
                sub_point_list[i, i] = self.point[i].copy()
            sub_point_list[0, 1] = np.array([(self.point[2, 0] - self.point[0, 0]) / 3 + self.point[0, 0], self.point[0, 1]])
            sub_point_list[1, 0] = np.array([(self.point[2, 0] - self.point[0, 0]) / 3 * 2 + self.point[0, 0], self.point[0, 1]])
            sub_point_list[3, 2] = np.array([sub_point_list[0, 1, 0], self.point[2, 1]])
            sub_point_list[2, 3] = np.array([sub_point_list[1, 0, 0], self.point[2, 1]])
            sub_point_list[0, 3] = np.array([self.point[0, 0], (self.point[2, 1] - self.point[0, 1]) / 3 + self.point[0, 1]])
            sub_point_list[3, 0] = np.array([self.point[0, 0], (self.point[2, 1] - self.point[0, 1]) / 3 * 2 + self.point[0, 1]])
            sub_point_list[1, 2] = np.array([self.point[2, 0], (self.point[2, 1] - self.point[0, 1]) / 3 + self.point[0, 1]])
            sub_point_list[2, 1] = np.array([self.point[2, 0], (self.point[2, 1] - self.point[0, 1]) / 3 * 2 + self.point[0, 1]])
            sub_point_list[0, 2] = np.array([sub_point_list[0, 1, 0], sub_point_list[0, 3, 1]])
            sub_point_list[3, 1] = np.array([sub_point_list[0, 1, 0], sub_point_list[3, 0, 1]])
            sub_point_list[1, 3] = np.array([sub_point_list[1, 0, 0], sub_point_list[0, 3, 1]])
            sub_point_list[2, 0] = np.array([sub_point_list[1, 0, 0], sub_point_list[3, 0, 1]])

            # Count the number of points on edges in each sub-rectangle
            for r in range(height):
                for c in range(width):
                    if self.edges[r, c] > 0:
                        if self.point[0, 0] <= c <= sub_point_list[0, 1, 0]:
                            if self.point[0, 1] <= r <= sub_point_list[0, 3, 1]:
                                countPointsInSubRectangle[0] += 1
                            elif sub_point_list[3, 0, 1] <= r <= self.point[3, 1]:
                                countPointsInSubRectangle[3] += 1
                        elif sub_point_list[1, 0, 0] <= c <= self.point[1, 0]:
                            if self.point[1, 1] <= r <= sub_point_list[1, 2, 1]:
                                countPointsInSubRectangle[1] += 1
                            elif sub_point_list[2, 1, 1] <= r <= self.point[2, 1]:
                                countPointsInSubRectangle[2] += 1
            distance01 = pow(self.point[0, 0] - self.point[1, 0], 2) + pow(self.point[0, 1] - self.point[1, 1], 2)  # The distance between vertice 0 and vertice 1
            distance03 = pow(self.point[0, 0] - self.point[3, 0], 2) + pow(self.point[0, 1] - self.point[3, 1], 2)  # The distance between vertice 0 and vertice 3

            # perpendicular : edge01 is width, edge03 is length, edge01 is shorter than edge03
            if distance01 < distance03:
                if np.argmin(countPointsInSubRectangle) == 0:
                    self.direction = 0
                    self.gradient = 0 * math.pi / 180
                    top = [(self.point[0, 0] + self.point[1, 0]) / 2, (self.point[0, 1] + self.point[1, 1]) / 2]
                    bottom = [(self.point[2, 0] + self.point[3, 0]) / 2, (self.point[2, 1] + self.point[3, 1]) / 2]
                    self.grasping_point[0] = (top[0] + bottom[0]) / 2
                    self.grasping_point[1] = top[1] + (bottom[1] - top[1]) * 0.55
                elif np.argmin(countPointsInSubRectangle) == 1:
                    self.direction = 1
                    self.gradient = 0 * math.pi / 180
                    top = [(self.point[0, 0] + self.point[1, 0]) / 2, (self.point[0, 1] + self.point[1, 1]) / 2]
                    bottom = [(self.point[2, 0] + self.point[3, 0]) / 2, (self.point[2, 1] + self.point[3, 1]) / 2]
                    self.grasping_point[0] = (top[0] + bottom[0]) / 2
                    self.grasping_point[1] = top[1] + (bottom[1] - top[1]) * 0.55
                elif np.argmin(countPointsInSubRectangle) == 2:
                    self.direction = 0
                    self.gradient = -180 * math.pi / 180
                    top = [(self.point[2, 0] + self.point[3, 0]) / 2, (self.point[2, 1] + self.point[3, 1]) / 2]
                    bottom = [(self.point[0, 0] + self.point[1, 0]) / 2, (self.point[0, 1] + self.point[1, 1]) / 2]
                    self.grasping_point[0] = (top[0] + bottom[0]) / 2
                    self.grasping_point[1] = bottom[1] + (top[1] - bottom[1]) * 0.45
                else:
                    self.direction = 1
                    self.gradient = 180 * math.pi / 180
                    top = [(self.point[2, 0] + self.point[3, 0]) / 2, (self.point[2, 1] + self.point[3, 1]) / 2]
                    bottom = [(self.point[0, 0] + self.point[1, 0]) / 2, (self.point[0, 1] + self.point[1, 1]) / 2]
                    self.grasping_point[0] = (top[0] + bottom[0]) / 2
                    self.grasping_point[1] = bottom[1] + (top[1] - bottom[1]) * 0.45
            # edge03 is width, edge01 is length, edge03 is shorter than edge01
            else:
                if np.argmin(countPointsInSubRectangle) == 0:
                    self.direction = 1
                    self.gradient = -90 * math.pi / 180
                    top = [(self.point[0, 0] + self.point[3, 0]) / 2, (self.point[0, 1] + self.point[3, 1]) / 2]
                    bottom = [(self.point[1, 0] + self.point[2, 0]) / 2, (self.point[1, 1] + self.point[2, 1]) / 2]
                    self.grasping_point[0] = top[0] + (bottom[0] - top[0]) * 0.55
                    self.grasping_point[1] = (top[1] + bottom[1]) / 2
                elif np.argmin(countPointsInSubRectangle) == 1:
                    self.direction = 0
                    self.gradient = 90 * math.pi / 180
                    top = [(self.point[1, 0] + self.point[2, 0]) / 2, (self.point[1, 1] + self.point[2, 1]) / 2]
                    bottom = [(self.point[0, 0] + self.point[3, 0]) / 2, (self.point[0, 1] + self.point[3, 1]) / 2]
                    self.grasping_point[0] = bottom[0] + (top[0] - bottom[0]) * 0.45
                    self.grasping_point[1] = (top[1] + bottom[1]) / 2
                elif np.argmin(countPointsInSubRectangle) == 2:
                    self.direction = 1
                    self.gradient = 90 * math.pi / 180
                    top = [(self.point[1, 0] + self.point[2, 0]) / 2, (self.point[1, 1] + self.point[2, 1]) / 2]
                    bottom = [(self.point[0, 0] + self.point[3, 0]) / 2, (self.point[0, 1] + self.point[3, 1]) / 2]
                    self.grasping_point[0] = bottom[0] + (top[0] - bottom[0]) * 0.45
                    self.grasping_point[1] = (top[1] + bottom[1]) / 2
                else:
                    self.direction = 0
                    self.gradient = -90 * math.pi / 180
                    top = [(self.point[0, 0] + self.point[3, 0]) / 2, (self.point[0, 1] + self.point[3, 1]) / 2]
                    bottom = [(self.point[1, 0] + self.point[2, 0]) / 2, (self.point[1, 1] + self.point[2, 1]) / 2]
                    self.grasping_point[0] = top[0] + (bottom[0] - top[0]) * 0.55
                    self.grasping_point[1] = (top[1] + bottom[1]) / 2
            return
        else:  # If the rectangle is slant.
            self.pointOrderSlantRectangle()
            # Decide the points of four sub-rectangles.
            # The points of a sub-rectangle are stored in a numpy array in the same order as function pointOrder:
            # top, short edge, long edge (bottom), short edge
            inclination = 1
            for i in range(4):
                sub_point_list[i, i] = self.point[i].copy()
            if self.point[0, 0] > self.point[1, 0]:  # vertice 0 is to the RIGHT of vertice 1
                inclination = 1
                # point (0, 1) - point 1 of sub-rectangle next to vertice 0
                sub_point_list[0, 1, 0] = (self.point[0, 0] - self.point[1, 0]) / 3 * 2 + self.point[1, 0]  # x coordinate of point (0, 1)
                sub_point_list[0, 1, 1] = (self.point[1, 1] - self.point[0, 1]) / 3 + self.point[0, 1]  # y coordinate of point (0, 1)
                # point (1, 0) - point 0 of sub-rectangle next to vertice 1
                sub_point_list[1, 0, 0] = (self.point[0, 0] - self.point[1, 0]) / 3 + self.point[1, 0]  # x coordinate of point (1, 0)
                sub_point_list[1, 0, 1] = (self.point[1, 1] - self.point[0, 1]) / 3 * 2 + self.point[0, 1]  # y coordinate of point (1, 0)
                # point (0, 3) - point 3 of sub-rectangle next to vertice 0
                sub_point_list[0, 3, 0] = (self.point[3, 0] - self.point[0, 0]) / 3 + self.point[0, 0]  # x coordinate of point (0, 3)
                sub_point_list[0, 3, 1] = (self.point[3, 1] - self.point[0, 1]) / 3 + self.point[0, 1]  # y coordinate of point (0, 3)
                # point (3, 0) - point 0 of sub-rectangle next to vertice 3
                sub_point_list[3, 0, 0] = (self.point[3, 0] - self.point[0, 0]) / 3 * 2 + self.point[0, 0]  # x coordinate of point (3, 0)
                sub_point_list[3, 0, 1] = (self.point[3, 1] - self.point[0, 1]) / 3 * 2 + self.point[0, 1]  # y coordinate of point (3, 0)
                # point (1, 2) - point 2 of sub-rectangle next to vertice 1
                sub_point_list[1, 2, 0] = (self.point[2, 0] - self.point[1, 0]) / 3 + self.point[1, 0]  # x coordinate of point (1, 2)
                sub_point_list[1, 2, 1] = (self.point[2, 1] - self.point[1, 1]) / 3 + self.point[1, 1]  # y coordinate of point (1, 2)
                # point (2, 1) - point 1 of sub-rectangle next to vertice 2
                sub_point_list[2, 1, 0] = (self.point[2, 0] - self.point[1, 0]) / 3 * 2 + self.point[1, 0]  # x coordinate of point (2, 1)
                sub_point_list[2, 1, 1] = (self.point[2, 1] - self.point[1, 1]) / 3 * 2 + self.point[1, 1]  # y coordinate of point (2, 1)
                # point (3, 2) - point 2 of sub-rectangle next to vertice 3
                sub_point_list[3, 2, 0] = (self.point[3, 0] - self.point[2, 0]) / 3 * 2 + self.point[2, 0]  # x coordinate of point (3, 2)
                sub_point_list[3, 2, 1] = (self.point[2, 1] - self.point[3, 1]) / 3 + self.point[3, 1]  # y coordinate of point (3, 2)
                # point (2, 3) - point 3 of sub-rectangle next to vertice 2
                sub_point_list[2, 3, 0] = (self.point[3, 0] - self.point[2, 0]) / 3 + self.point[2, 0]  # x coordinate of point (2, 3)
                sub_point_list[2, 3, 1] = (self.point[2, 1] - self.point[3, 1]) / 3 * 2 + self.point[3, 1]  # y coordinate of point (2, 3)
                # point (0, 2) - point 2 of sub-rectangle next to vertice 0
                sub_point_list[0, 2, 0] = (sub_point_list[3, 2, 0] - sub_point_list[0, 1, 0]) / 3 + sub_point_list[0, 1, 0]  # x coordinate of point (0, 2)
                sub_point_list[0, 2, 1] = (sub_point_list[3, 2, 1] - sub_point_list[0, 1, 1]) / 3 + sub_point_list[0, 1, 1]  # y coordinate of point (0, 2)
                # point (3, 1) - point 1 of sub-rectangle next to vertice 3
                sub_point_list[3, 1, 0] = (sub_point_list[3, 2, 0] - sub_point_list[0, 1, 0]) / 3 * 2 + sub_point_list[0, 1, 0]  # x coordinate of point (3, 1)
                sub_point_list[3, 1, 1] = (sub_point_list[3, 2, 1] - sub_point_list[0, 1, 1]) / 3 * 2 + sub_point_list[0, 1, 1]  # y coordinate of point (3, 1)
                # point (1, 3) - point 3 of sub-rectangle next to vertice 1
                sub_point_list[1, 3, 0] = (sub_point_list[2, 3, 0] - sub_point_list[1, 0, 0]) / 3 + sub_point_list[1, 0, 0]  # x coordinate of point (1, 3)
                sub_point_list[1, 3, 1] = (sub_point_list[2, 3, 1] - sub_point_list[1, 0, 1]) / 3 + sub_point_list[1, 0, 1]  # y coordinate of point (1, 3)
                # point (2, 0) - point 0 of sub-rectangle next to vertice 2
                sub_point_list[2, 0, 0] = (sub_point_list[2, 3, 0] - sub_point_list[1, 0, 0]) / 3 * 2 + sub_point_list[1, 0, 0]  # x coordinate of point (2, 0)
                sub_point_list[2, 0, 1] = (sub_point_list[2, 3, 1] - sub_point_list[1, 0, 1]) / 3 * 2 + sub_point_list[1, 0, 1]  # y coordinate of point (2, 0)
            else:  # vertice 0 is to the LEFT of vertice 1
                inclination = -1
                # point (0, 1) - point 1 of sub-rectangle next to vertice 0
                sub_point_list[0, 1, 0] = (self.point[1, 0] - self.point[0, 0]) / 3 + self.point[0, 0]  # x coordinate of point (0, 1)
                sub_point_list[0, 1, 1] = (self.point[1, 1] - self.point[0, 1]) / 3 + self.point[0, 1]  # y coordinate of point (0, 1)
                # point (1, 0) - point 0 of sub-rectangle next to vertice 1
                sub_point_list[1, 0, 0] = (self.point[1, 0] - self.point[0, 0]) / 3 * 2 + self.point[0, 0]  # x coordinate of point (1, 0)
                sub_point_list[1, 0, 1] = (self.point[1, 1] - self.point[0, 1]) / 3 * 2 + self.point[0, 1]  # y coordinate of point (1, 0)
                # point (0, 3) - point 3 of sub-rectangle next to vertice 0
                sub_point_list[0, 3, 0] = (self.point[0, 0] - self.point[3, 0]) / 3 * 2 + self.point[3, 0]  # x coordinate of point (0, 3)
                sub_point_list[0, 3, 1] = (self.point[3, 1] - self.point[0, 1]) / 3 + self.point[0, 1]  # y coordinate of point (0, 3)
                # point (3, 0) - point 0 of sub-rectangle next to vertice 3
                sub_point_list[3, 0, 0] = (self.point[0, 0] - self.point[3, 0]) / 3 + self.point[3, 0]  # x coordinate of point (3, 0)
                sub_point_list[3, 0, 1] = (self.point[3, 1] - self.point[0, 1]) / 3 * 2 + self.point[0, 1]  # y coordinate of point (3, 0)
                # point (1, 2) - point 2 of sub-rectangle next to vertice 1
                sub_point_list[1, 2, 0] = (self.point[1, 0] - self.point[2, 0]) / 3 * 2 + self.point[2, 0]  # x coordinate of point (1, 2)
                sub_point_list[1, 2, 1] = (self.point[2, 1] - self.point[1, 1]) / 3 + self.point[1, 1]  # y coordinate of point (1, 2)
                # point (2, 1) - point 1 of sub-rectangle next to vertice 2
                sub_point_list[2, 1, 0] = (self.point[1, 0] - self.point[2, 0]) / 3 + self.point[2, 0]  # x coordinate of point (2, 1)
                sub_point_list[2, 1, 1] = (self.point[2, 1] - self.point[1, 1]) / 3 * 2 + self.point[1, 1]  # y coordinate of point (2, 1)
                # point (3, 2) - point 2 of sub-rectangle next to vertice 3
                sub_point_list[3, 2, 0] = (self.point[2, 0] - self.point[3, 0]) / 3 + self.point[3, 0]  # x coordinate of point (3, 2)
                sub_point_list[3, 2, 1] = (self.point[2, 1] - self.point[3, 1]) / 3 + self.point[3, 1]  # y coordinate of point (3, 2)
                # point (2, 3) - point 3 of sub-rectangle next to vertice 2
                sub_point_list[2, 3, 0] = (self.point[2, 0] - self.point[3, 0]) / 3 * 2 + self.point[3, 0]  # x coordinate of point (2, 3)
                sub_point_list[2, 3, 1] = (self.point[2, 1] - self.point[3, 1]) / 3 * 2 + self.point[3, 1]  # y coordinate of point (2, 3)
                # point (0, 2) - point 2 of sub-rectangle next to vertice 0
                sub_point_list[0, 2, 0] = (sub_point_list[1, 2, 0] - sub_point_list[0, 3, 0]) / 3 + sub_point_list[0, 3, 0]  # x coordinate of point (0, 2)
                sub_point_list[0, 2, 1] = (sub_point_list[1, 2, 1] - sub_point_list[0, 3, 1]) / 3 + sub_point_list[0, 3, 1]  # y coordinate of point (0, 2)
                # point (1, 3) - point 3 of sub-rectangle next to vertice 1
                sub_point_list[1, 3, 0] = (sub_point_list[1, 2, 0] - sub_point_list[0, 3, 0]) / 3 * 2 + sub_point_list[0, 3, 0]  # x coordinate of point (1, 3)
                sub_point_list[1, 3, 1] = (sub_point_list[1, 2, 1] - sub_point_list[0, 3, 1]) / 3 * 2 + sub_point_list[0, 3, 1]  # y coordinate of point (1, 3)
                # point (3, 1) - point 1 of sub-rectangle next to vertice 3
                sub_point_list[3, 1, 0] = (sub_point_list[2, 1, 0] - sub_point_list[3, 0, 0]) / 3 + sub_point_list[3, 0, 0]  # x coordinate of point (3, 1)
                sub_point_list[3, 1, 1] = (sub_point_list[2, 1, 1] - sub_point_list[3, 0, 1]) / 3 + sub_point_list[3, 0, 1]  # y coordinate of point (3, 1)
                # point (2, 0) - point 0 of sub-rectangle next to vertice 2
                sub_point_list[2, 0, 0] = (sub_point_list[2, 1, 0] - sub_point_list[3, 0, 0]) / 3 * 2 + sub_point_list[3, 0, 0]  # x coordinate of point (2, 0)
                sub_point_list[2, 0, 1] = (sub_point_list[2, 1, 1] - sub_point_list[3, 0, 1]) / 3 * 2 + sub_point_list[3, 0, 1]  # y coordinate of point (2, 0)
            self.sub_rectangle_list = np.array(
                [[sub_point_list[0, 0], sub_point_list[0, 1], sub_point_list[0, 2], sub_point_list[0, 3]],
                 [sub_point_list[1, 0], sub_point_list[1, 1], sub_point_list[1, 2], sub_point_list[1, 3]],
                 [sub_point_list[2, 0], sub_point_list[2, 1], sub_point_list[2, 2], sub_point_list[2, 3]],
                 [sub_point_list[3, 0], sub_point_list[3, 1], sub_point_list[3, 2], sub_point_list[3, 3]]], dtype=int)
            for r in range(height):
                for c in range(width):
                    if self.edges[r, c] > 0:
                        if pointInRectangle([c, r], self.sub_rectangle_list[0], inclination):
                            countPointsInSubRectangle[0] += 1
                        elif pointInRectangle([c, r], self.sub_rectangle_list[1], inclination):
                            countPointsInSubRectangle[1] += 1
                        elif pointInRectangle([c, r], self.sub_rectangle_list[2], inclination):
                            countPointsInSubRectangle[2] += 1
                        elif pointInRectangle([c, r], self.sub_rectangle_list[3], inclination):
                            countPointsInSubRectangle[3] += 1
            if self.point[1, 0] < self.point[3, 0]:  # the x-coordinate of point 1 is smaller than that of point 3, representing point 1 is to the left of point 3
                if np.argmin(countPointsInSubRectangle) == 0:
                    self.direction = 1
                    self.gradient = self.slopeRatio(self.point[3], self.point[0])
                    top = [(self.point[0, 0] + self.point[1, 0]) / 2, (self.point[0, 1] + self.point[1, 1]) / 2]
                    bottom = [(self.point[2, 0] + self.point[3, 0]) / 2, (self.point[2, 1] + self.point[3, 1]) / 2]
                    self.grasping_point[0] = top[0] + (bottom[0] - top[0]) * 0.55
                    self.grasping_point[1] = top[1] + (bottom[1] - top[1]) * 0.55
                elif np.argmin(countPointsInSubRectangle) == 1:
                    self.direction = 0
                    self.gradient = self.slopeRatio(self.point[3], self.point[0])
                    top = [(self.point[0, 0] + self.point[1, 0]) / 2, (self.point[0, 1] + self.point[1, 1]) / 2]
                    bottom = [(self.point[2, 0] + self.point[3, 0]) / 2, (self.point[2, 1] + self.point[3, 1]) / 2]
                    self.grasping_point[0] = top[0] + (bottom[0] - top[0]) * 0.55
                    self.grasping_point[1] = top[1] + (bottom[1] - top[1]) * 0.55
                elif np.argmin(countPointsInSubRectangle) == 2:
                    self.direction = 1
                    self.gradient = self.slopeRatio(self.point[0], self.point[3])
                    top = [(self.point[2, 0] + self.point[3, 0]) / 2, (self.point[2, 1] + self.point[3, 1]) / 2]
                    bottom = [(self.point[0, 0] + self.point[1, 0]) / 2, (self.point[0, 1] + self.point[1, 1]) / 2]
                    self.grasping_point[0] = bottom[0] + (top[0] - bottom[0]) * 0.45
                    self.grasping_point[1] = bottom[1] + (top[1] - bottom[1]) * 0.45
                else:
                    self.direction = 0
                    self.gradient = self.slopeRatio(self.point[0], self.point[3])
                    top = [(self.point[2, 0] + self.point[3, 0]) / 2, (self.point[2, 1] + self.point[3, 1]) / 2]
                    bottom = [(self.point[0, 0] + self.point[1, 0]) / 2, (self.point[0, 1] + self.point[1, 1]) / 2]
                    self.grasping_point[0] = bottom[0] + (top[0] - bottom[0]) * 0.45
                    self.grasping_point[1] = bottom[1] + (top[1] - bottom[1]) * 0.45
            else:  # point 1 is to the right point 3
                if np.argmin(countPointsInSubRectangle) == 0:
                    self.direction = 0
                    self.gradient = self.slopeRatio(self.point[3], self.point[0])
                    top = [(self.point[0, 0] + self.point[1, 0]) / 2, (self.point[0, 1] + self.point[1, 1]) / 2]
                    bottom = [(self.point[2, 0] + self.point[3, 0]) / 2, (self.point[2, 1] + self.point[3, 1]) / 2]
                    self.grasping_point[0] = bottom[0] + (top[0] - bottom[0]) * 0.45
                    self.grasping_point[1] = top[1] + (bottom[1] - top[1]) * 0.55
                elif np.argmin(countPointsInSubRectangle) == 1:
                    self.direction = 1
                    self.gradient = self.slopeRatio(self.point[3], self.point[0])
                    top = [(self.point[0, 0] + self.point[1, 0]) / 2, (self.point[0, 1] + self.point[1, 1]) / 2]
                    bottom = [(self.point[2, 0] + self.point[3, 0]) / 2, (self.point[2, 1] + self.point[3, 1]) / 2]
                    self.grasping_point[0] = bottom[0] + (top[0] - bottom[0]) * 0.45
                    self.grasping_point[1] = top[1] + (bottom[1] - top[1]) * 0.55
                elif np.argmin(countPointsInSubRectangle) == 2:
                    self.direction = 0
                    self.gradient = self.slopeRatio(self.point[0], self.point[3])
                    top = [(self.point[2, 0] + self.point[3, 0]) / 2, (self.point[2, 1] + self.point[3, 1]) / 2]
                    bottom = [(self.point[0, 0] + self.point[1, 0]) / 2, (self.point[0, 1] + self.point[1, 1]) / 2]
                    self.grasping_point[0] = top[0] + (bottom[0] - top[0]) * 0.55
                    self.grasping_point[1] = bottom[1] + (top[1] - bottom[1]) * 0.45
                else:
                    self.direction = 1
                    self.gradient = self.slopeRatio(self.point[0], self.point[3])
                    top = [(self.point[2, 0] + self.point[3, 0]) / 2, (self.point[2, 1] + self.point[3, 1]) / 2]
                    bottom = [(self.point[0, 0] + self.point[1, 0]) / 2, (self.point[0, 1] + self.point[1, 1]) / 2]
                    self.grasping_point[0] = top[0] + (bottom[0] - top[0]) * 0.55
                    self.grasping_point[1] = bottom[1] + (top[1] - bottom[1]) * 0.45
            return

    @staticmethod
    def imageRotation(img):
        """
        Rotate the image clockwise by 90 degrees.
        :param img:
        :return:
        """
        (h, w) = img.shape[:2]
        (cX, cY) = (w // 2, h // 2)
        M = cv2.getRotationMatrix2D((cX, cY), -90, 1.0)
        cos = np.abs(M[0, 0])
        sin = np.abs(M[0, 1])
        nW = int((h * sin) + (w * cos))
        nH = int((h * cos) + (w * sin))
        M[0, 2] += (nW / 2) - cX
        M[1, 2] += (nH / 2) - cY
        rotate_img = cv2.warpAffine(img, M, (nW, nH))
        return rotate_img


if __name__ == '__main__':
    # video = cv2.VideoCapture(0)
    path = './image_test/1.png'
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    # ret, img = video.read()
    # while ret is False:
    #     print('reading image from video.')
    #     ret, img = video.read()
    # ret = False
    cv2.imshow('detection', img)
    if cv2.waitKey(1) == ord('c'):
        cv2.destroyWindow('detection')
    pa = PostureAnalysis(img)
    print(pa.gradient * 180 / np.pi, pa.direction)
    cv2.drawContours(img, [pa.box], 0, (255, 0, 0), 1)
    cv2.imshow('test', img)
    cv2.waitKey(0)
    cv2.destroyWindow('test')
