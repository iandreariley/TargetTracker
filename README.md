# Cal Poly Pomona Target Tracking Project

This is the current home of the visual target tracking project undertaken as a joint effort between the aerospace and computer science departments at Cal Poly Pomona. But it's mainly an aerospace thing; let's not kid ourselves. Go Broncos.

## Notes:
- For the control_test.py. launch it with no arguments: `python control_test.py`. Launch it with the 'u' option to get it to update constantly with a "target" that is dead center in the image frame: `python control_test.py u`
- Right now it looks like its not updating correctly, the velocity varies wildly, and the copter seems to buck. Altitude remains constant.
- In the constant update state, the loop will print debug values a couple times every second.
- In the read-evaluate-print loop (REPL), the prompt will ask you to enter an X and Y value, separated by a space. Do this and press enter. It will then print a bunch of relavent intermediate values as well as the final velocities in the x and y directions.
- Clone this repo with `git clone https://www.github.com/iandreariley/TargetTracker/`
- To pull updates down use `git pull origin master` once insde the TargetTracker Folder.
- Setup of the hardware is a touch finicky. The following setup has worked in the past:
    - Plug camera into the USB port _before_ booting the TX1
    - Boot the TX1
    - Plug the Pixhawk into the micro-USB port
    - power on the Pixhawk
    - Both devices should now be accesible through the /dev/ttyUSB0 device.
- To connect over WiFi do the following:
    - Boot the TX1. Give it about a minute to boot and setup its access point
    - Connect to the access point. the password is 'enRouteArduPilot'
    - enter the following command `arp -a` and look for an ip address starting with 10
    - enter the following command `ssh -Y ubuntu@10.x.x.x` (fill in the xs based on the previous step)
    - this will log you in to the TX1 as the root user, ubuntu.