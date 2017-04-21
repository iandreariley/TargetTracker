from dronekit import *
import time
import math
from math import cos, tan, sin
import numpy as np
import sys


PIX_WIDTH = 960
PIX_HEIGHT = 720
CAM_X_ANGLE = 48.5 * 180 / math.pi
CAM_Y_ANGLE = 28.0 * 180 / math.pi

vehicle = connect('udpin:0.0.0.0:14551', wait_ready=True)
messages = vehicle.message_factory

def get_input():
    print '<<<',
    return raw_input().strip()

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


def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.

    This method is an approximation, and may not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """	
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing;


def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):    
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)

def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
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
        
def constant_update():
    print "starting update"
    counter = 0

    # This should represent the middle of the camera, more or less.
    # so the heading shouldn't curve at all
    x = PIX_WIDTH / 2.0
    y = PIX_HEIGHT / 2.0
    while True:
        if counter % 100 == 0:
            set_velocity_from_image(x,y)
        else:
            set_velocity_from_image(x,y,True)
        time.sleep(0.1)
        counter += 1

def get_position_input(position_input):
    return map(int, position_input.split())

def set_velocity_from_image(img_x, img_y, debug=False):
    yaw = vehicle.attitude.yaw
    pitch = vehicle.attitude.pitch
    roll = vehicle.attitude.roll
    altitude = vehicle.location.global_relative_frame.alt
    alpha = -pitch

    x_real = float(img_x) / float(PIX_WIDTH) / 2.0
    y_real = float(img_y) / float(PIX_HEIGHT) / 2.0
    theta_x = CAM_X_ANGLE * x_real
    theta_y = CAM_Y_ANGLE * y_real

    y_rel = altitude / (math.tan(pitch) - theta_y)
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
    unit_vector = 5.0 * horz_plane / np.linalg.norm(horz_plane)

    if debug:
        print "img_x: %d; img_y: %d" % (img_x, img_y)
        print "yaw: %g; pitch: %g; roll: %g" % (yaw, pitch, roll)
        print "altitude: %g" % altitude
        print ""
        print "angles relative to camera frame in degrees:"
        print "theta_x: %g; theta_y: %g" % (theta_x, theta_y)
        print ""
        print "relative position of target in NED:"
        print "target_x: %g; target_y: %g" % (x_rel, y_rel)
        print ""
        print "velocity vectors (should result in speed~5.0):"
        print "x: %g; y: %g" % (unit_vector[0], unit_vector[1])

    set_velocity(unit_vector[0], unit_vector[1])
    
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

def repl_loop():
    
    print "\nEnter any code you'd like to execute. the vehicle object is called 'vehicle'; so the following will print the vehicle's attitude: 'print vehicle.attitude'"

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


