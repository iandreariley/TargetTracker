from dronekit import *
import time
from math import cos, tan, atan, sin, pi
import numpy as np
import sys


PIX_WIDTH = 960
PIX_HEIGHT = 720
Y_SHIFT = PIX_HEIGHT / 2
X_SHIFT = PIX_WIDTH / 2
DEGREES_TO_RADIANS = pi / 180
CAM_X_ANGLE = 48.5 * DEGREES_TO_RADIANS
CAM_Y_ANGLE = 28.0 * DEGREES_TO_RADIANS

vehicle = connect('udpin:0.0.0.0:14551', wait_ready=True)
messages = vehicle.message_factory

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print "Reached target altitude"
            break
        time.sleep(1)


def condition_yaw(heading, relative=False):
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

def get_velocity_from_image(img_x, img_y, attitude, altitude):

    yaw = attitude.yaw
    pitch = attitude.pitch
    roll = attitude.roll
    
    x_real = float(img_x) / float(PIX_WIDTH) / 2.0
    y_real = float(img_y) / float(PIX_HEIGHT) / 2.0
    theta_x = CAM_X_ANGLE * x_real
    theta_y = CAM_Y_ANGLE * y_real

    y_rel = altitude / (math.tan(pitch - theta_y))
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
        
    n, e, d = rbn_roll.dot(rbn_yaw.dot(position))
    horz_plane = np.array([n,e])
    unit_vector = horz_plane / np.linalg.norm(horz_plane)
    final_vector = 5.0 * unit_vector

    if debug:
        print "img_x: %d; img_y: %d" % (img_x, img_y)
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
        print "velocity vectors (should result in speed~5.0):"
        print "x: %g; y: %g" % (unit_vector[0], unit_vector[1])
        print "final-x: %g; final-y: %g" % (final_vector[0], final_vector[1])
    return final_vector

def get_img_position_from_ned(n, e):
    yaw = vehicle.attitude.yaw
    pitch = vehicle.attitude.pitch
    roll = vehicle.attitude.roll
    altitude = vehicle.location.global_relative_frame.alt
    
    x_drone = e * cos(yaw) + n * sin(-yaw)
    y_drone = e * sin(yaw) + n * cos(yaw)

    theta_x = CAM_X_ANGLE / 2.0 + atan(x_drone / y_drone)
    theta_y = -pitch + atan(altitude / y_drone)

    img_x = (theta_x / CAM_X_ANGLE) * PIX_WIDTH
    img_y = float(Y_SHIFT) + (theta_y / CAM_Y_ANGLE * 2.0) * PIX_HEIGHT
    return img_x, img_y
    
def set_velocity_from_image(img_x, img_y, debug=False):
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
    final_vector = 5.0 * unit_vector

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

    set_velocity(final_vector[0], final_vector[1])
    condition_yaw(atan(e / n) * 180 / pi)
    
def set_velocity(n, e):
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


def set_position(north, east, down):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    vehicle.send_mavlink(msg)

def get_input():
    print '<<<',
    return raw_input().strip()

def get_position_input(position_input):
    return map(float, position_input.split())
            
def constant_update():
    print "starting update"
    counter = 0

    # This should represent the middle of the camera, more or less.
    # so the heading shouldn't curve at all
    print "setting heading to due north..."
    condition_yaw(0)
    time.sleep(5)
    n = 200
    e = 200
    while True:
        x, y = get_img_position_from_ned(n, e)
        if counter % 100 == 0:
            set_velocity_from_image(x,y,True)
        else:
            set_velocity_from_image(x,y)
        time.sleep(0.1)
        counter += 1

def repl_loop():
    
    print "\nEnter image x and y coordinates separated by a space and see what happens!"

    command = get_input()
    print "Executing command: %s" % command
    while command != 'q':
        try:
            x,y = get_position_input(command)
            print "x: %d, y: %d" % (x,y)
            set_velocity_from_image(x,y,True)
        except:
            print "Uh oh! Something is wrong with that code, try again."
            print "\n",
        command = get_input()

arm_and_takeoff(20)

if len(sys.argv) > 1 and sys.argv[1] == 'u':
    constant_update()
else:
    repl_loop()


