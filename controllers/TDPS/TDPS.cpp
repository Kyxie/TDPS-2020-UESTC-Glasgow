// File:          TDPS.cpp
// Date:          April/26/2020
// Description:   Course Project
// Author:        Kunyang Xie
// GitHub:        abnormal0

// Initial Car Coordinate = [-49 0.2 11.5]
// Initial Duck Coordinate = [-49.1 0.45 11.5]
// Middle Car Coorrdinate = [18 0.2 3.5]
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/lidar.h>
#include <webots/camera.h>
#include <webots/inertial_unit.h>
#include <webots/position_sensor.h>
#include <funcs.hpp>
using namespace std;

// Work Frequency
float TIME_STEP = 10.;
// Class declaration
Funcs funcs;
// Data variables
const float *lidar_val = NULL;
const double *imu_val = NULL;
const unsigned char *img_val = NULL;
// Define the height and width of the camera
int camera_w = 0;
int camera_h = 0;
// Number of lidar beams
int lidar_width = 0;
// State machines
unsigned short duck_seq[5] = {0};
unsigned short bridge_seq[5] = {0};
// The ith stop sign has been taken
// The tree can be regarded as a stop sign
bool taken[3] = {0};
// Speed-angle coefficeint
// This coefficient connects dirction difference and wheel speed
float L = 2.;
// Angle coefficent
// This coefficient keeps the car facing the obstacle
// float ang_k = 5.;
// Target status of car
struct Mobile
{
  // float dir = -pi/2;
  float dir = pi;
  float speed = 5.;
}Mobile;

// Sensors
WbDeviceTag lidar;
WbDeviceTag imu;
WbDeviceTag camera;
WbDeviceTag front_left_wheel;
WbDeviceTag front_right_wheel;
WbDeviceTag back_left_wheel;
WbDeviceTag back_right_wheel;

// Functions initializations
void robot_init(void);
void mainloop(void);
void set_speed(Speed spe);

int main(int argc, char **argv)
{
  // Initialize
  robot_init();
  // Mainloop
  while (wb_robot_step(TIME_STEP) != -1)
    mainloop();
  // Clear memory
  wb_robot_cleanup();
  delete lidar_val;
  delete imu_val;
  delete img_val;
  return 0;
}

void robot_init(void)
{
  wb_robot_init();
  // Get the equipments
  lidar = wb_robot_get_device("lidar");
  imu = wb_robot_get_device("imu");
  camera = wb_robot_get_device("camera");
  front_left_wheel = wb_robot_get_device("front left wheel");
  front_right_wheel = wb_robot_get_device("front right wheel");
  back_left_wheel = wb_robot_get_device("back left wheel");
  back_right_wheel = wb_robot_get_device("back right wheel");
  // Set the motor end angle to infinity
  wb_motor_set_position(front_left_wheel, INFINITY);
  wb_motor_set_position(front_right_wheel, INFINITY);
  wb_motor_set_position(back_left_wheel, INFINITY);
  wb_motor_set_position(back_right_wheel, INFINITY);
  // Enable
  wb_lidar_enable(lidar, TIME_STEP);
  wb_inertial_unit_enable(imu, TIME_STEP);
  wb_camera_enable(camera, TIME_STEP);
  // Number of lidar beams
  lidar_width = wb_lidar_get_horizontal_resolution(lidar);
  funcs.init_lidar(lidar_width);
  // Number of pixels in height and width
  camera_w = wb_camera_get_width(camera);
  camera_h = wb_camera_get_height(camera);
}

void mainloop(void)
{
  // Get data
  lidar_val = wb_lidar_get_range_image(lidar);
  imu_val = wb_inertial_unit_get_roll_pitch_yaw(imu);
  img_val = wb_camera_get_image(camera);
  // Get distance
  funcs.distance_get(lidar_val, pi, 0.15, 0);
  // Set speed
  Speed temp;
  temp = funcs.motion_keep(imu_val[2], Mobile.dir, L);
  temp.wheel_l += Mobile.speed;
  temp.wheel_r += Mobile.speed;
  set_speed(temp);
  // Get obstacle information
  Obj_center obstacle = funcs.obj_dis_info();
  float real_distance = sqrt(obstacle.obj_x * obstacle.obj_x + obstacle.obj_y * obstacle.obj_y);
  // float real_angle = acos(obstacle.obj_x / real_distance) - pi/2;
  
  // Main logic
  // Before the first stop sign
  if(bridge_seq[0] == 0)
  {
    Mobile.dir = -pi/2;
    if(real_distance < 1.3)
    {
      Mobile.dir = 0;
      // When the aim direction is setting to 0, seq0 is over
      bridge_seq[0] = 1;
    }
  }
  // Before the bridge
  else if(bridge_seq[1] == 0)
  {
    // Keep straight
    Mobile.dir = 0;
    if(abs(pi/6 - imu_val[0]) < 0.05)
    {
      // When start to climb, seq1 is over
      bridge_seq[1] = 1;
      // Slow down
      Mobile.speed = 4;
    }
  }
  // Before the tree
  else if(bridge_seq[2] == 0)
  {
    if(real_distance < 1 && abs(imu_val[0]) < 0.05)
    {
      Mobile.dir = -pi/2;
      // Max speed
      Mobile.speed = 6.3;
      // When after the bridge and
      // the direction is close to -pi/2, seq2 is over
      if(abs(-pi/2 - imu_val[2]) < 0.05)
        bridge_seq[2] = 1;
    }
  }
  // Before the second stop
  else if(bridge_seq[3] == 0)
  {
    if(real_distance < 1.5)
    {
      Mobile.dir = 0;
      // When the direction is close to 0, seq3 is over
      if(abs(0 - imu_val[2]) < 0.05)
        bridge_seq[3] = 1;
    }
  }
  // Before the arch
  else if(bridge_seq[4] == 0)
  {
    Mobile.speed = 5;
    if(real_distance < 1.5)
    {
      Mobile.dir = 0;
      bridge_seq[4] = 1;
    }
  }
}

void set_speed(Speed spe)
{
  // Limit the speed (-6.4,6.4)
  spe.wheel_l = funcs.val_limit(spe.wheel_l,-6.3,6.3);
  spe.wheel_r = funcs.val_limit(spe.wheel_r,-6.3,6.3);
  // Set speed
  wb_motor_set_velocity(front_left_wheel, spe.wheel_l);
  wb_motor_set_velocity(front_right_wheel, spe.wheel_r);
  wb_motor_set_velocity(back_left_wheel, spe.wheel_l);
  wb_motor_set_velocity(back_right_wheel, spe.wheel_r);
}