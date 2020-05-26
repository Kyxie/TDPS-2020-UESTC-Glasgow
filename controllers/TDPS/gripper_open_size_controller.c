#include <webots/motor.h>
#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>

#include "arm.h"
#include "gripper.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define TIME_STEP 64

double leftspeed = 0.0;
double rightspeed = 0.0;

WbDeviceTag wheels[4];
  
void check_keyboard(int key){
    switch(key){
      
      case WB_KEYBOARD_UP:
          leftspeed = 4.0;
          rightspeed = 4.0;
          break;
        case WB_KEYBOARD_DOWN:
          leftspeed = -4.0;
          rightspeed = -4.0;
          break;
        case WB_KEYBOARD_RIGHT:
          leftspeed = 2.0;
          rightspeed = -2.0;
          break;
        case WB_KEYBOARD_LEFT:
          leftspeed = -2.0;
          rightspeed = 2.0;
          break;
        case '=':
        case 388:
        case 65585:
          printf("Grip\n");
          gripper_grip();
          break;
        case '-':
        case 390:
          printf("Ungrip\n");
          gripper_release();
          break;
        
        case 332:
        case WB_KEYBOARD_UP | WB_KEYBOARD_SHIFT:
          printf("Increase arm height\n");
          arm_increase_height();
          break;
        case 326:
        case WB_KEYBOARD_DOWN | WB_KEYBOARD_SHIFT:
          printf("Decrease arm height\n");
          arm_decrease_height();
          break;
        case 330:
        case WB_KEYBOARD_RIGHT | WB_KEYBOARD_SHIFT:
          printf("Increase arm orientation\n");
          arm_increase_orientation();
          break;
        case 328:
        case WB_KEYBOARD_LEFT | WB_KEYBOARD_SHIFT:
          printf("Decrease arm orientation\n");
          arm_decrease_orientation();
        
        default :
          leftspeed = 0;
          rightspeed = 0;
    }
}

void set_wheels_velocity(){
        wb_motor_set_velocity(wheels[0],leftspeed);
        wb_motor_set_velocity(wheels[1],rightspeed);
        wb_motor_set_velocity(wheels[2],leftspeed);
        wb_motor_set_velocity(wheels[3],rightspeed);
}


int main(int argc, char **argv){
    wb_robot_init();
    
    arm_init();
    gripper_init();

    wb_keyboard_enable(TIME_STEP); 
   
      
    char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
    for (int i=0; i<4; i++){
      wheels[i] = wb_robot_get_device(wheels_names[i]);
      wb_motor_set_position(wheels[i],INFINITY);
      wb_motor_set_velocity(wheels[i],0.0);
    }

    while (wb_robot_step(TIME_STEP) != -1){
      //drop();
      gripper_grip();
      int key = wb_keyboard_get_key();
      check_keyboard(key);
      set_wheels_velocity();

}
    wb_robot_cleanup();
    return 0;
}