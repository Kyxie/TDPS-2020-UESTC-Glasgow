#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/position_sensor.h>
#include <math.h>
#include <stdio.h>

#define TIME_STEP 10

static WbDeviceTag arm_elements[5];
static WbDeviceTag arm_pos[5];

enum Height {
  ARM_FRONT_FLOOR,
  ARM_FRONT_PLATE,
  ARM_HANOI_PREPARE,
  ARM_FRONT_CARDBOARD_BOX,
  ARM_RESET,
  ARM_BACK_PLATE_HIGH,
  ARM_BACK_PLATE_LOW,
  ARM_MAX_HEIGHT
};

enum Orientation {
  ARM_BACK_LEFT,
  ARM_LEFT,
  ARM_FRONT_LEFT,
  ARM_FRONT,
  ARM_FRONT_RIGHT,
  ARM_RIGHT,
  ARM_BACK_RIGHT,
  ARM_MAX_SIDE
};

enum Arm { ARM1, ARM2, ARM3, ARM4, ARM5 };

static enum Height current_height = ARM_RESET;
static enum Orientation current_orientation = ARM_FRONT;

void arm_reset() {
  wb_motor_set_position(arm_elements[ARM1], 0.0);
  wb_motor_set_position(arm_elements[ARM2], 1.57);
  wb_motor_set_position(arm_elements[ARM3], -2.635);
  wb_motor_set_position(arm_elements[ARM4], 1.78);
  wb_motor_set_position(arm_elements[ARM5], 0.0);
}

void arm_set_height(enum Height h_height) {
  switch (h_height) {
    case ARM_FRONT_FLOOR:
      wb_motor_set_position(arm_elements[ARM2], -0.97);
      wb_motor_set_position(arm_elements[ARM3], -1.55);
      wb_motor_set_position(arm_elements[ARM4], -0.61);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_FRONT_PLATE:
      wb_motor_set_position(arm_elements[ARM2], -0.62);
      wb_motor_set_position(arm_elements[ARM3], -0.98);
      wb_motor_set_position(arm_elements[ARM4], -1.53);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_FRONT_CARDBOARD_BOX:
      wb_motor_set_position(arm_elements[ARM2], 0.0);
      wb_motor_set_position(arm_elements[ARM3], -0.77);
      wb_motor_set_position(arm_elements[ARM4], -1.21);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_RESET:
      wb_motor_set_position(arm_elements[ARM2], 1.57);
      wb_motor_set_position(arm_elements[ARM3], -2.635);
      wb_motor_set_position(arm_elements[ARM4], 1.78);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_BACK_PLATE_HIGH:
      wb_motor_set_position(arm_elements[ARM2], 0.678);
      wb_motor_set_position(arm_elements[ARM3], 0.682);
      wb_motor_set_position(arm_elements[ARM4], 1.74);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_BACK_PLATE_LOW:
      wb_motor_set_position(arm_elements[ARM2], 0.92);
      wb_motor_set_position(arm_elements[ARM3], 0.42);
      wb_motor_set_position(arm_elements[ARM4], 1.78);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_HANOI_PREPARE:
      wb_motor_set_position(arm_elements[ARM2], -0.4);
      wb_motor_set_position(arm_elements[ARM3], -1.2);
      wb_motor_set_position(arm_elements[ARM4], -M_PI_2);
      wb_motor_set_position(arm_elements[ARM5], M_PI_2);
      break;
    default:
      fprintf(stderr, "arm_height() called with a wrong argument\n");
      return;
  }
  current_height = h_height;
}

void arm_set_orientation(enum Orientation orientation) {
  switch (orientation) {
    case ARM_BACK_LEFT:
      wb_motor_set_position(arm_elements[ARM1], -2.949);
      break;
    case ARM_LEFT:
      wb_motor_set_position(arm_elements[ARM1], -M_PI_2);
      break;
    case ARM_FRONT_LEFT:
      wb_motor_set_position(arm_elements[ARM1], -0.2);
      break;
    case ARM_FRONT:
      wb_motor_set_position(arm_elements[ARM1], 0.0);
      break;
    case ARM_FRONT_RIGHT:
      wb_motor_set_position(arm_elements[ARM1], 0.2);
      break;
    case ARM_RIGHT:
      wb_motor_set_position(arm_elements[ARM1], M_PI_2);
      break;
    case ARM_BACK_RIGHT:
      wb_motor_set_position(arm_elements[ARM1], 2.949);
      break;
    default:
      fprintf(stderr, "arm_set_side() called with a wrong argument\n");
      return;
  }
  current_orientation = orientation;
}

void arm_init() {
  arm_elements[ARM1] = wb_robot_get_device("arm1");
  arm_elements[ARM2] = wb_robot_get_device("arm2");
  arm_elements[ARM3] = wb_robot_get_device("arm3");
  arm_elements[ARM4] = wb_robot_get_device("arm4");
  arm_elements[ARM5] = wb_robot_get_device("arm5");
  
  arm_pos[ARM1] = wb_motor_get_position_sensor(arm_elements[ARM1]);
  arm_pos[ARM2] = wb_motor_get_position_sensor(arm_elements[ARM2]);
  arm_pos[ARM3] = wb_motor_get_position_sensor(arm_elements[ARM3]);
  arm_pos[ARM4] = wb_motor_get_position_sensor(arm_elements[ARM4]);
  arm_pos[ARM5] = wb_motor_get_position_sensor(arm_elements[ARM5]);

  wb_position_sensor_enable(arm_pos[ARM1],TIME_STEP);
  wb_position_sensor_enable(arm_pos[ARM2],TIME_STEP);
  wb_position_sensor_enable(arm_pos[ARM3],TIME_STEP);
  wb_position_sensor_enable(arm_pos[ARM4],TIME_STEP);
  wb_position_sensor_enable(arm_pos[ARM5],TIME_STEP);

  wb_motor_set_velocity(arm_elements[ARM2], 0.5);

  arm_set_height(ARM_RESET);
  arm_set_orientation(ARM_FRONT);
}

