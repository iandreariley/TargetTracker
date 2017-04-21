# Cal Poly Pomona Target Tracking Project

This is the current home of the visual target tracking project undertaken as a joint effort between the aerospace and computer science departments at Cal Poly Pomona. But it's mainly an aerospace thing; let's not kid ourselves. Go Broncos.

## Notes:
- For the control_test.py. launch it with no arguments: `python control_test.py`. Launch it with the 'u' option to get it to update constantly with a "target" that is dead center in the image frame: `python control_test.py u`
- Right now it looks like its not updating correctly, the velocity varies wildly, and the copter seems to buck. Altitude remains constant.
- In the constant update state, the loop will print debug values a couple times every second.
- In the read-evaluate-print loop (REPL), the prompt will ask you to enter an X and Y value, separated by a space. Do this and press enter. It will then print a bunch of relavent intermediate values as well as the final velocities in the x and y directions.