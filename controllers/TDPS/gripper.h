#include <webots/motor.h>
#include <webots/robot.h>

#define LEFT 0
#define RIGHT 1

#define MIN_POS 0.0
#define MAX_POS 0.025
#define OFFSET_WHEN_LOCKED 0.021

static WbDeviceTag fingers[2];

double bound(double v, double a, double b) {
  return (v > b) ? b : (v < a) ? a : v;
}

void gripper_init() {
  fingers[LEFT] = wb_robot_get_device("finger1");
  fingers[RIGHT] = wb_robot_get_device("finger2");

  wb_motor_set_velocity(fingers[LEFT], 0.03);
  wb_motor_set_velocity(fingers[RIGHT], 0.03);
}

void gripper_grip() {
  wb_motor_set_position(fingers[LEFT], MIN_POS);
  wb_motor_set_position(fingers[RIGHT], MIN_POS);
}

void gripper_release() {
  wb_motor_set_position(fingers[LEFT], MAX_POS);
  wb_motor_set_position(fingers[RIGHT], MAX_POS);
}

void gripper_set_gap(double gap) {
  double v = bound(0.5 * (gap - OFFSET_WHEN_LOCKED), MIN_POS, MAX_POS);
  wb_motor_set_position(fingers[LEFT], v);
  wb_motor_set_position(fingers[RIGHT], v);
}