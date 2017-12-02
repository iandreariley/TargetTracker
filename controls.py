from dronekit import *
import time
from math import cos, tan, atan, sin, pi, atan2
import numpy as np
import sys
import logging

PIX_WIDTH = 640
PIX_HEIGHT = 480
Y_SHIFT = PIX_HEIGHT / 2
X_SHIFT = PIX_WIDTH / 2
DEGREES_TO_RADIANS = pi / 180
CAM_X_ANGLE = 48.5 * DEGREES_TO_RADIANS
CAM_Y_ANGLE = 28.0 * DEGREES_TO_RADIANS

def condition_yaw(vehicle, heading, relative=False):
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def set_velocity_from_image(vehicle, img_x, img_y, speed=1, debug=False):
    x = img_x - X_SHIFT
    y = -img_y + Y_SHIFT
    yaw = vehicle.attitude.yaw
    pitch = vehicle.attitude.pitch
    roll = vehicle.attitude.roll
    altitude = vehicle.location.global_relative_frame.alt

    x_real = float(x) / float(X_SHIFT)
    y_real = float(y) / float(PIX_HEIGHT) / 2.0
    theta_x = CAM_X_ANGLE * x_real / 2.0
    theta_y = CAM_Y_ANGLE * y_real

    y_rel = altitude / (math.tan(-pitch - theta_y))
    x_rel = y_rel * math.tan(theta_x)

    rbn_roll = np.array([
        [1,0,0],
        [0, cos(roll), sin(roll)],
        [0, -sin(roll), cos(roll)]
    ])

    rbn_yaw = np.array([
        [cos(yaw), sin(yaw), 0],
        [-sin(yaw), cos(yaw), 0],
        [0, 0, 1]
    ])

    rbn_pitch = np.array([
        [cos(pitch), 0, -sin(pitch)],
        [0, 1, 0],
        [sin(pitch), 0, cos(pitch)]
    ])

    position = np.array([x_rel, y_rel, -altitude])

    e, n, d = rbn_roll.dot(rbn_yaw.dot(position))
    horz_plane = np.array([n,e])
    unit_vector = horz_plane / np.linalg.norm(horz_plane)
    final_vector = speed * unit_vector

    if debug:
        print "img_x: %d; img_y: %d" % (img_x, img_y)
        print "x: %d; y: %d" % (x, y)
        print "yaw: %g; pitch: %g; roll: %g" % (yaw, pitch, roll)
        print "altitude: %g" % altitude
        print ""
        print "angles relative to camera frame in degrees:"
        print "theta_x: %g; theta_y: %g" % (theta_x, theta_y)
        print ""
        print "relative position of target:"
        print "target_x: %g; target_y: %g; target_h: %g" % (x_rel, y_rel, -altitude)
        print ""
        print "NED position of target:"
        print "north: %g; east: %g" % (n,e)
        print ""
        print "velocity vectors (should result in speed~5.0):"
        print "x: %g; y: %g" % (unit_vector[0], unit_vector[1])
        print "final-x: %g; final-y: %g" % (final_vector[0], final_vector[1])

    logging.info('north: {0}'.format(n))
    logging.info('east: {0}'.format(e))
    radians = atan2(e, n)
    if radians < 0:
        radians += 2.0 * pi
    set_velocity(vehicle, final_vector[0], final_vector[1])
    condition_yaw(vehicle, radians * 180 / pi)

def set_velocity(vehicle, n, e):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only positions enabled)
        0, 0, 0, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        n, e, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    vehicle.send_mavlink(msg)

