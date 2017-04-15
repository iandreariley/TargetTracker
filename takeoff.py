from dronekit import connect, VehicleMode
import time

vehicle = connect('udpin:0.0.0.0:14551', wait_ready=True)

def get_input():
    print '<<<',
    return raw_input()

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

arm_and_takeoff(20)

print "\nEnter any code you'd like to execute. the vehicle object is called 'vehicle'; so the following will print the vehicle's attitude: 'print vehicle.attitude'"

command = get_input()

while command != 'q':
    try:
        exec(command)
    except:
        print "Uh oh! Something is wrong with that code, try again."
    print "\n",
    command = get_input()
