import cv2
import csv
import argparse
import numpy as np
import time
import os


def intify(tup):
    return tuple(map(int, tup))


def draw_bbox(image, bbox, color, center):
    if center:
        top_left = intify((bbox[0] - bbox[2] / 2, bbox[1] - bbox[3] / 2))
        bottom_right = intify((bbox[0] + bbox[2] / 2, bbox[1] + bbox[3] / 2))
    else:
        top_left = intify((bbox[0], bbox[1]))
        bottom_right = intify((bbox[0] + bbox[2], bbox[1] + bbox[3]))
    top_right = (bottom_right[0], top_left[1])
    bottom_left = (top_left[0], bottom_right[1])

    # draw bounding box.
    cv2.line(image, top_left, top_right, color, 4)
    cv2.line(image, top_right, bottom_right, color, 4)
    cv2.line(image, bottom_right, bottom_left, color, 4)
    cv2.line(image, bottom_left, top_left, color, 4)


parser = argparse.ArgumentParser()
parser.add_argument("video_directory", help="directory with sequence images")
args = parser.parse_args()

imgs = sorted(filter(lambda f: f.endswith('png'), os.listdir(args.video_directory)), key=lambda f: int(f.split('.')[0]))
boxes = []
with open(os.path.join(args.video_directory, 'gt.txt'), 'rb') as f:
    reader = csv.reader(f, delimiter=',')
    for row in reader:
        try:
            boxes.append(map(int, row))
        except:
            boxes.append([0, 0, 0, 0])



for i, img_file in enumerate(imgs):
    image = cv2.imread(os.path.join(args.video_directory, img_file))
    draw_bbox(image, boxes[i], (255,255,255), False)
    cv2.imshow('run', image)
    cv2.waitKey(10)
