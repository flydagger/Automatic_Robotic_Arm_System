import cv2
import numpy as np


class PlateRecognition:
    def __init__(self):
        self.plate_flag = False
        self.edgeVertex = np.zeros((2, 2), dtype=np.int32)
        self.Dilation_KERNEL = np.ones((3, 3), np.uint8)

    def recognition(self, frame):
        height, width, channel = frame.shape
        detect_area = frame[0:height, 0:round(width / 4)]
        # Start edge detection
        gray = cv2.cvtColor(detect_area, cv2.COLOR_BGR2GRAY)
        GaussianKERNEL = 5
        blur_gray = cv2.GaussianBlur(gray, (GaussianKERNEL, GaussianKERNEL), 0)
        # The following two thresholds are allowed to be assigned by user in the future
        LOW_THRESHOLD = 50
        HIGH_THRESHOLD = 150
        edges = cv2.Canny(blur_gray, LOW_THRESHOLD, HIGH_THRESHOLD)
        edges = cv2.dilate(edges, self.Dilation_KERNEL, iterations=1)
        rho = 1  # distance resolution in pixels of the Hough grid
        theta = np.pi / 180  # angular resolution in radians of the Hough grid
        threshold = 15  # minimum number of votes (intersections in Hough grid cell)
        min_line_length = 50  # minimum number of pixels making up a line
        max_line_gap = 20  # maximum gap in pixels between connectable line segments
        # Run Hough on edge detected image
        # Output "lines" is an array containing endpoints of detected line segments
        lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)
        if lines is None:
            return
        for line in lines:
            for x1, y1, x2, y2 in line:
                line_length = np.sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))
                if (x1 == x2 or abs(y1 - y2) / abs(x1 - x2) > 5.0) and line_length > round(height * 0.8):
                    self.edgeVertex = np.array([[x1, y1], [x2, y2]])
                    self.plate_flag = True
                    break


# if __name__ == '__main__':
#     video = cv2.VideoCapture(0)
#     pr = PlateRecognition()
#     while True:
#         ret, frame = video.read()
#         pr.recognition(frame)
#         if pr.plate_flag is True:
#             line_image = np.copy(frame)
#             cv2.line(line_image, tuple(pr.edgeVertex[0]), tuple(pr.edgeVertex[1]), (255, 0, 0), 5)
#             lines_edges = cv2.addWeighted(frame, 0.8, line_image, 1, 0)
#             cv2.imshow('A plate is found.', lines_edges)
#             cv2.waitKey(0)
#             cv2.destroyWindow('A plate is found.')
#             break
#         cv2.imshow('test', frame)
#         cv2.waitKey(1)
#
#     print('Finished: ', pr.edgeVertex)
#     cv2.waitKey(0)
#     cv2.destroyWindow('test')
#     video.release()
