#!/usr/bin/env python

import argparse
import cv2
from numpy import empty, nan
import os
from os.path import isdir
import sys
import time
from dronekit import *
import logging

import numpy as np

import CMT
import util
from controls import set_velocity_from_image

video_directory = "videos/"
BAUD = 921600

def get_centroid(CMT):
        x = (CMT.tl[0] + CMT.tr[0]) / 2
        y = (CMT.tl[1] + CMT.br[1]) / 2
        return np.array([x,y])

def create_video_directory():
        video_count = 0
        for item in os.listdir(video_directory):
                if isdir(video_directory + item):
                        video_count += 1
        new_video_path = video_directory + "video_" + str(video_count + 1)
        os.mkdir(new_video_path)
        return new_video_path

def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        logging.info('Waiting for vehicle to initialize before arming')
        time.sleep(1)

    logging.info("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        logging.info(" Waiting for arming...")
        time.sleep(1)

    logging.info("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        logging.info(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            logging.info("Reached target altitude")
            break
        time.sleep(1)


logging.basicConfig(level=logging.INFO)

CMT = CMT.CMT()

parser = argparse.ArgumentParser(description='Track an object.')

parser.add_argument('--vehicleurl', help='url for the vehicle to control.', default='/dev/ttyUSB0')
parser.add_argument('--no-preview', dest='preview', action='store_false', help='No preview, in the event that the bounding box is passed as an argument.')
parser.add_argument('--bbox', dest='bbox', help='Specify initial bounding box.')
parser.add_argument('--quiet', dest='quiet', action='store_true', help='Do not show graphical output (Useful in combination with --output-dir ).')

args = parser.parse_args()
CMT.estimate_scale = True
CMT.estimate_rotation = False
logging.info(args.vehicleurl)
try:
        vehicle = connect(args.vehicleurl, baud=BAUD, wait_ready=True)
        arm_and_takeoff(vehicle, 10)
except Exception as inst:
        logging.warning(inst.args)
        logging.warning('Could not connect to vehicle, landing and exiting.')
        vehicle.mode = VehicleMode("RTL")
        vehicle.close()
        sys.exit(1)

 # Clean up
cv2.destroyAllWindows()
preview = args.preview

# Dirty hack for finding which USB device the camera is on.
cap = cv2.VideoCapture(0)
i = 1
while not cap.isOpened() and i < 10:
        cap = cv2.VideoCapture(i)
        i += 1

logging.info("Video Capture connected on device %d." % (i - 1))

# Check if videocapture is working
if not cap.isOpened():
        logging.warning('Unable to open video input.')
        sys.exit(1)

while preview:
        logging.warning("getting first image capture...")
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
        logging.info('getting user input')
        (tl, br) = util.get_rect(im_draw)

logging.info('using', tl, br, 'as init bb')


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
        CMT.process_frame(im_gray)

        # Display results
        if CMT.has_result:
                # get centroid of bounding box and update drone velocity vectors.
                center_x,center_y = get_centroid(CMT)
                logging.info("x: {0}\ny:{1}\n\n".format(center_x, center_y))
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
logging.info('fps: %f' % (frame / (end - start)))

