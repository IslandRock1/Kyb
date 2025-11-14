
# Python Code

## How to calibrate

First one needs to run the GUI code. Here you can set initial angles for the robot. Etc..

Then the results can be found in the DATA directory, while plots can be found in the Plots directory.

## Code

# GUIControl.py

The GUI file is responsible for most of the control of the robot. Containing sliders and buttons for different commands.

# formatResponse.py

This file will format the text data to the csv file the calibration code expects. This code should probably be done from the GUI or calibrating.

TODO:
* Place this functionality in either GUI code or calibrateSensor.py

# calibrateSensor.py

This file will run the necessary code to calibrate the sensor. This code is written by Jon, and modified by the group. The modifications mainly change the file structure, no modification to the regression or similar has been done.

