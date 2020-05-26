// File:          TDPS.cpp
// Date:          April/26/2020
// Description:   Course Project
// Author:        Kunyang Xie
// GitHub:        abnormal0

// Initial car coordinate = [-49 0.1 11.5]
// Corner car coordinate = [-31.5 0.1 11.5]
// Bridge car coordinate = [18 0.1 3.5]
// Cast car coordinate = [-1.7 0.1 3.5]
// Initial food coordinate = [-48.9373 0.64 11.5065]
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <arm.h>
#include <gripper.h>
#include <string.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/lidar.h>
#include <webots/camera.h>
#include <webots/inertial_unit.h>
#include <webots/position_sensor.h>
#include <funcs.hpp>
using namespace std;

#define north 0
#define south -pi
#define east -pi/2
#define west pi/2

// Work Frequency
// float TIME_STEP = 10.;
// Class declaration
Funcs funcs;
// Data variables
const float *lidar_val = NULL;
const double *imu_val = NULL;
const unsigned char *imag_val = NULL;
// Define the height and width of the camera
int camera_w = 0;
int camera_h = 0;
// Number of lidar beams
int lidar_width = 0;
// State machines
unsigned short cast_seq[5] = {0};
unsigned short bridge_seq[7] = {0};
unsigned short color_seq[2] = {0};
// The ith stop sign has been taken
// The tree can be regarded as a stop sign
bool taken[3] = {0};
// Speed-angle coefficeint
// This coefficient connects dirction difference and wheel speed
float L = 2.;
// Target status of car
struct Mobile
{
  // float dir = -pi/2;
  float dir = pi;
  float speed = 6.3;
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

static void step()
{
  if (wb_robot_step(TIME_STEP) == -1)
  {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}
  
static void passive_wait(double sec)
{
  double start_time = wb_robot_get_time();
  do
  {
    step();
  }while(start_time + sec >wb_robot_get_time());
}

void go(enum Height height1)
{ 
  arm_set_height(height1);
}

void back(enum Height height2)
{
  arm_set_height(height2);
}

int main(int argc, char **argv)
{
  // Initialize
  robot_init();
  arm_init();
  gripper_init();
  // Mainloop
  while (wb_robot_step(TIME_STEP) != -1)
    mainloop();
  // Clear memory
  wb_robot_cleanup();
  delete lidar_val;
  delete imu_val;
  delete imag_val;
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
  imag_val = wb_camera_get_image(camera);
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
  Obj_center lin_imag = funcs.imag_process(imag_val);
  float real_distance = sqrt(obstacle.obj_x * obstacle.obj_x + obstacle.obj_y * obstacle.obj_y);
  float real_angle = acos(obstacle.obj_x / real_distance) - pi/2;
  float line_dir = sqrt(lin_imag.obj_x * lin_imag.obj_x + lin_imag.obj_y * lin_imag.obj_y);
  float line_angle = acos(lin_imag.obj_x / line_dir) - pi/2;
  Mobile.dir = imu_val[2] + line_angle;
  // Main logic
  // Start
  if(cast_seq[0] == 0)
  {
    if(obstacle.obj_y < funcs.lidar_max_range)
      Mobile.speed = 10*(obstacle.obj_y - 0.9);
    if(obstacle.obj_y - 0.9 < 0.05)
      cast_seq[0] = 1;
  }
  else if(cast_seq[1] == 0)
  {
    Mobile.dir = south;
    if(fabs(imu_val[2] + pi < 0.2))
      Mobile.speed = 25*(obstacle.obj_y - 0.32);
    if(fabs(obstacle.obj_y - 0.32) < 0.03)
      cast_seq[1] = 1;
  }
  else if(cast_seq[2] == 0)
  {
    Mobile.dir = north;
    Mobile.speed = 0;
    if(fabs(imu_val[2] < 0.1))
    {
      wb_motor_set_velocity(front_left_wheel, 0);
      wb_motor_set_velocity(front_right_wheel, 0);
      wb_motor_set_velocity(back_left_wheel, 0);
      wb_motor_set_velocity(back_right_wheel, 0);
      passive_wait(0.5);
      wb_motor_set_velocity(front_left_wheel, -1);
      wb_motor_set_velocity(front_right_wheel, -1);
      wb_motor_set_velocity(back_left_wheel, -1);
      wb_motor_set_velocity(back_right_wheel, -1);
      passive_wait(1);
      wb_motor_set_velocity(front_left_wheel, 0);
      wb_motor_set_velocity(front_right_wheel, 0);
      wb_motor_set_velocity(back_left_wheel, 0);
      wb_motor_set_velocity(back_right_wheel, 0);
      go(ARM_FRONT_FLOOR);
      passive_wait(6.0);
      gripper_release();
      passive_wait(3);
      back(ARM_RESET);
      passive_wait(1.0);
      if (fabs(wb_position_sensor_get_value(arm_pos[ARM3]) + 2.635) <0.04)
        cast_seq[2] = 1;
    }
  }
  else if(cast_seq[3] == 0)
  {
    Mobile.dir = east + pi/8;
    Mobile.speed = 6.3;
    if (funcs.count > 500)
      cast_seq[3] = 1;
  }
  else if(cast_seq[4] == 0)
  {
    Mobile.dir = imu_val[2] + line_angle;
    if(funcs.count < 40)
    {
      Mobile.dir = east;
      cast_seq[4] = 1;
    }
  }
  // Before the first stop sign
  else if(bridge_seq[0] == 0)
  {
    Mobile.dir = east;
    if(real_distance < 1.3)
    {
      Mobile.dir = north;
      // When the aim direction is setting to 0, seq0 is over
      bridge_seq[0] = 1;
    }
  }
  // Before the bridge
  else if(bridge_seq[1] == 0)
  {
    // Keep straight
    Mobile.dir = north;
    // Slow down
    Mobile.speed = 3;
    if(abs(pi/6 - imu_val[0]) < 0.05)
      bridge_seq[1] = 1;
  }
  // Down
  else if(bridge_seq[2] == 0)
  {
    Mobile.dir = north;
    if(abs(pi/6 + imu_val[0]) < 0.05)
      bridge_seq[2] = 1;
  }
  // Finished downhill
  else if(bridge_seq[3] == 0)
  {
    Mobile.dir = north;
    if(abs(imu_val[0]) < 0.05)
      bridge_seq[3] = 1;
  }
  // Before the tree
  else if(bridge_seq[4] == 0)
  {
    Mobile.dir = north;
    Mobile.speed = 6.3;
    if(real_distance < 1)
    {
      Mobile.dir = east;
      // When after the bridge and
      // the direction is close to -pi/2, seq2 is over
      if(abs(-pi/2 - imu_val[2]) < 0.3)
        bridge_seq[4] = 1;
    }
  }
  // Before the second stop
  else if(bridge_seq[5] == 0)
  {
    Mobile.dir = east;
    if(real_distance < 1.5)
    {
      Mobile.dir = north;
      if(abs(0 - imu_val[2]) < 0.2)
        bridge_seq[5] = 1;
    }
  }
  // Before the arch
  else if(bridge_seq[6] == 0)
  {
    Mobile.dir = north + real_angle;
    Mobile.speed = 5;
    // Have Detected the white line
    if(funcs.count > 100)
    {
      Mobile.speed = 6.3;
      bridge_seq[6] = 1;
    }
  }
  // Before color
  else if(color_seq[0] == 0)
  {
    if(real_distance < 0.5 && imu_val[2] == west)
      color_seq[0] = 0;
    
  }
  else if(color_seq[1] == 0)
  {
    Mobile.speed = 5;
    if(funcs.c_r > funcs.c_y and funcs.c_r > funcs.c_b)
    {
      funcs.color_trace = RED;
      cout << "red" <<endl;
    }
    else if(funcs.c_b > funcs.c_y and funcs.c_b > funcs.c_r)
    {
      funcs.color_trace = BLUE;
      cout << "blue" <<endl;
    }
    else
    {
      funcs.color_trace = YELLOW;
      cout << "yellow" <<endl;
    }
    color_seq[1] = 1;
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