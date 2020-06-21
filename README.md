# TDPS_2020_UESTC_Glasgow
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
7. There is a cross road have three different colors, red, blue and yellow, if we change the color of the color box on the road, the car will choose the corresponding line.
8. The project ends when the car reach the red end line.
## How to run
1. Download the software Webots from cyberbotics, now it is open source: https://github.com/cyberbotics/webots.git.
2. Install.
3. Open the file /words/Patio.wbt.
4. Upload the controler (/controllers/TPDS/TDPS.cpp) for the rover and simulate.
5. If you want to build on your own computer, change the line 52 in file Makefile (/controllers/TPDS/Makefile) to your own location and then compile the code.
## Demo Video
BiliBili: https://www.bilibili.com/video/BV1Rp4y1S7o3?from=search&seid=727746215518421105
