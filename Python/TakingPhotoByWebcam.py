#!/usr/bin/env python
# -*- coding:utf-8 -*-
# Author: Phoenix Fan

import cv2


class TakingPhotoAndSave(object):
    def __init__(self):
        print("This is the constructor method.")
        self.video = cv2.VideoCapture(0)
        self.video.set(3, 1280)
        self.video.set(4, 720)
        while True:
            ret, photo = self.video.read()
            cv2.line(photo, (640, 0), (640, 720), (0, 0, 255), 3)
            cv2.line(photo, (0, 360), (1280, 360), (0, 0, 255), 3)
            cv2.imshow('image', photo)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            save_flag = input("Save this photo?")
            if save_flag == 'y' or save_flag == 'Y':
                name = input("Input a name")
                cv2.imwrite(name+'.png', photo)
            exit_flag = input("Exit?(Y/N)")
            if exit_flag == 'Y' or exit_flag == 'y':
                self.video.release()
                break


if __name__ == '__main__':
    TakingPhotoAndSave()
