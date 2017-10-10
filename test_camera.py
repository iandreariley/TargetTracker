import cv2
import os
from os.path import isdir
import numpy as np
import time

cap = cv2.VideoCapture(0)
video_directory = "videos/"
i = 1

def create_video_directory():
        video_count = 0
        for item in os.listdir(video_directory):
                if isdir(video_directory + item):
                        video_count += 1
        new_video_path = video_directory + "video_" + str(video_count + 1)
        os.mkdir(new_video_path)
        return new_video_path

while not cap.isOpened() and i < 10:
        cap = cv2.VideoCapture(i)
        i += 1

if cap.isOpened():
        print("video connected on %d" % (i - 1))
        stop_key_pressed = False
        frame = 0
        start = time.time()
        video_path = create_video_directory()
        while not stop_key_pressed:
                _, im = cap.read()
                cv2.imwrite(video_path + "/" +  str(frame) + ".png", im)
                cv2.imshow('Preview', im)
                stop_key_pressed = cv2.waitKey(10) != -1
                frame += 1
        end = time.time()
        fps = frame / (end - start)
        print("fps: %f" % fps)

else:
        print("video connection failed")                

