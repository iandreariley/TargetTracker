#!/usr/bin/env python

import argparse
import cv2
from numpy import empty, nan
import os
from os.path import isdir
import sys
import time
from dronekit import *

import numpy as np

import CMT
import util
from controls import set_velocity_from_image

video_directory = "videos/"
BAUD = 921600

def get_centroid(CMT):
        x = CMT.tl[0] - CMT.tr[0]
        y = CMT.tl[1] - CMT.br[1]
        return np.array([x,y])

def create_video_directory():
        video_count = 0
        for item in os.listdir(video_directory):
                if isdir(video_directory + item):
                        video_count += 1
        new_video_path = video_directory + "video_" + str(video_count + 1)
        os.mkdir(new_video_path)
        return new_video_path

        
CMT = CMT.CMT()

parser = argparse.ArgumentParser(description='Track an object.')

parser.add_argument('--vehicleurl', help='url for the vehicle to control.', default='/dev/ttyUSB0')
parser.add_argument('--preview', dest='preview', action='store_const', const=True, default=None, help='Force preview')
parser.add_argument('--no-preview', dest='preview', action='store_const', const=False, default=None, help='Disable preview')
parser.add_argument('--bbox', dest='bbox', help='Specify initial bounding box.')
parser.add_argument('--quiet', dest='quiet', action='store_true', help='Do not show graphical output (Useful in combination with --output-dir ).')

args = parser.parse_args()
CMT.estimate_scale = True
CMT.estimate_rotation = False
print(args.vehicleurl)
try:
        vehicle = connect(args.vehicleurl, baud=BAUD, wait_ready=False)
        vehicle.mode = VehicleMode('GUIDED')
        vehicle.armed = True
except Exception as inst:
        print inst.args
        print 'Could not connect to vehicle, exiting.'
        sys.exit(1)

 # Clean up
cv2.destroyAllWindows()

preview = args.preview

# If no input path was specified, open camera device
cap = cv2.VideoCapture(0)
i = 1
while not cap.isOpened() and i < 10:
        cap = cv2.VideoCapture(i)
        i += 1

print("connected on %d" % (i - 1))
        
if preview is None:
        preview = True

# Check if videocapture is working
if not cap.isOpened():
        print 'Unable to open video input.'
        sys.exit(1)

while preview:
        print "getting first image capture..."
        status, im = cap.read()
        cv2.imshow('Preview', im)
        k = cv2.waitKey(10)
        if not k == -1:
                break

# Read first frame
status, im0 = cap.read()
im_gray0 = cv2.cvtColor(im0, cv2.COLOR_BGR2GRAY)
im_draw = np.copy(im0)

if args.bbox is not None:
        # Try to disassemble user specified bounding box
        values = args.bbox.split(',')
        try:
                values = [int(v) for v in values]
        except:
                raise Exception('Unable to parse bounding box')
        if len(values) != 4:
                raise Exception('Bounding box must have exactly 4 elements')
        bbox = np.array(values)

        # Convert to point representation, adding singleton dimension
        bbox = util.bb2pts(bbox[None, :])

        # Squeeze
        bbox = bbox[0, :]

        tl = bbox[:2]
        br = bbox[2:4]
else:
        # Get rectangle input from user
        print 'getting user input'
        (tl, br) = util.get_rect(im_draw)

print 'using', tl, br, 'as init bb'


CMT.initialise(im_gray0, tl, br)

frame = 1
video_path = create_video_directory()

stopped = False
start = time.time()
while not stopped:
        # Read image
        status, im = cap.read()
        if not status:
                break
        im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        im_draw = np.copy(im)
        print(im.shape)
        CMT.process_frame(im_gray)

        # Display results
        if CMT.has_result:
                # get centroid of bounding box and update drone velocity vectors.
                center_x,center_y = get_centroid(CMT)
                set_velocity_from_image(vehicle, center_x, center_y)
                
                # Draw updated estimate
                cv2.line(im_draw, CMT.tl, CMT.tr, (255, 0, 0), 4)
                cv2.line(im_draw, CMT.tr, CMT.br, (255, 0, 0), 4)
                cv2.line(im_draw, CMT.br, CMT.bl, (255, 0, 0), 4)
                cv2.line(im_draw, CMT.bl, CMT.tl, (255, 0, 0), 4)

        util.draw_keypoints(CMT.tracked_keypoints, im_draw, (255, 255, 255))
        # this is from simplescale
        util.draw_keypoints(CMT.votes[:, :2], im_draw)  # blue
        util.draw_keypoints(CMT.outliers[:, :2], im_draw, (0, 0, 255))

        cv2.imwrite(video_path + "/" +  str(frame) + ".png", im_draw)

        if not args.quiet:
                cv2.imshow('main', im_draw)

                # Check key input
                k = cv2.waitKey(10)
                key = chr(k & 255)
                if key == 'q':
                        break
                if key == 'd':
                        import ipdb; ipdb.set_trace()

        # Remember image
        im_prev = im_gray

        # Advance frame number
        frame += 1
end = time.time()
print('fps: %f' % (frame / (end - start)))
#        print '{5:04d}: center: {0:.2f},{1:.2f} scale: {2:.2f}, active: {3:03d}, {4:04.0f}ms'.format(CMT.center[0], CMT.center[1], CMT.scale_estimate, CMT.active_keypoints.shape[0], 1000 * (toc - tic), frame)
