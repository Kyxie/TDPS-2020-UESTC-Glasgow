# TDPS_2020_UESTC_Gla
## Introduction
This is a course project of Year 3, Glasgow College, UESTC. Influenced by COVID-19, This course will be conducted by software Webots in this semester.
## Features
The patio is built followed by instructions, which is similar to the scene of Qingshuihe Campus of UESTC.  
1. We need to make the car go along the line by using the camera module.  
2. After 2 turnings, recognize the orange cone, in this place the car need to cast the object to the lake.  
3. Go back to the main street and find the first stop sign, in this place, the car should turn left and go across the bridge.  
4. Turn right when the car finds the tree after the bridge and go straight on until it finds the second stop sign.  
5. After the second stop sign, go through the arch.
6. Continue patrolling the line until the car finds the color box in the middle of the road.  
7. There is a cross road have three different colors, red, blue and yellow, if we change the color of the color box on the road, the car will choose the coresponding line.
8. The project ends when the car reach the red end line.
## Technical Details
The following algorithm is involved in this project:
1. Sobel operator for detecting image edges.  
2. Binarization algorithm and Gaussian filter for image processing.  
3. PID control.