#ifndef _FUNCS_HPP_
#define _FUNCS_HPP_
  #include <iostream>
  #include <string>
  #include <math.h>
  #include <vector>
  using namespace std;
  
  #define pi 3.1415926
  #define mat vector<vector<vector<unsigned char>>>
  
  // Speed of wheels
  struct Speed
  {
    float wheel_l = 0.;
    float wheel_r = 0.;
  };
  // Center of obstacle
  struct Obj_center
  {
    float obj_x = 0.;
    float obj_y = 0.;
  };
  
  class Funcs
  {
    private:
      // History error for PD control
      float err_yaw_last = 0.;
      float MAX_SPEED = 5.;
      
    public:
      // Cartesian coordinate
      float *lidar_dcrx = NULL;
      float *lidar_dcry = NULL;
      // Lidar effective or not
      short *lidar_effective = NULL;
      // Number of lidar beams (512)
      int lidar_num = 0;
      // Range of lidar = 2m
      float lidar_max_range = 2.;
      
      ~Funcs()
      {
        delete lidar_dcrx;
        delete lidar_dcry;
        delete lidar_effective;
      }
      
      // Init lidar in Funcs
      void init_lidar(int num)
      {
        lidar_dcrx = new float [num];
        lidar_dcry = new float [num];
        lidar_num = num;
        lidar_effective = new short [num];
      }
      
      // yaw: current value, yaw_d: target value, L: speed coefficient
      Speed motion_keep(double yaw, double yaw_d, float L)
      {
        Speed temp;
        double dis = yaw - yaw_d;
        // If dis < 0, turn left, else, turn right
        double yaw_w = pd_control(abs(dis), 20., 5.);
        double speed_d = val_limit(yaw_w*L, -MAX_SPEED, MAX_SPEED);
        if(dis < 0)
        {
          temp.wheel_l = -speed_d;
          temp.wheel_r = speed_d;
        }
        else if (dis == 0)
        {
          temp.wheel_l = speed_d;
          temp.wheel_r = speed_d;
        }
        else
        {
          temp.wheel_l = speed_d;
          temp.wheel_r = -speed_d;
        }
        return temp;
      }
      
      // Return the center information of obstacle
      Obj_center obj_dis_info(void)
      {
        Obj_center temp;
        float temp_x = 0.;
        float temp_y = 0.;
        short counter = 0;
        // Sum
        for(int i = 0; i < lidar_num; i++)
        {
          if(lidar_effective[i] == 1)
          {
            temp_x += lidar_dcrx[i];
            temp_y += lidar_dcry[i];
            counter += 1;
          }
        }
        // Defaut values
        if(counter == 0)
        {
          temp_x = 0.;
          temp_y = 100.;
          counter = 1;
        }
        // Average
        temp.obj_x = temp_x / counter;
        temp.obj_y = temp_y / counter;
        return temp;
      }
      
      // Transfer to cartesian coordinate
      // x_bias and z_bias are the difference between center of car and center of lidar module
      // In this project, ang_range is pi
      void distance_get(const float *lidar_val, float ang_range, float z_bias, float x_bias)
      {
        float piece = ang_range / lidar_num;
        for(int i = 0; i < lidar_num; i++)
        {
          double ang = ang_range + (pi - ang_range)/2 - piece * i;
          lidar_dcrx[i] = (float)(lidar_val[i] * cos(ang) + x_bias);
          lidar_dcry[i] = (float)(lidar_val[i] * sin(ang) + z_bias);
          // If no detection
          if (lidar_val[i]/lidar_max_range>0.95)
            lidar_effective[i] = 0;
          else
            lidar_effective[i] = 1;
        }
      }
      
      // Restrict function
      float val_limit(float val, float min, float max)
      {
        float temp = 0;
        if(val < min)
          temp = min;
        else if(val > max)
          temp = max;
        else
          temp = val;
        return temp;
      }
      
      // PD Control
      float pd_control(float dis, float k, float d)
      {
        float temp = 0.;
        float dis_2 = 0.;
        dis_2 = dis - err_yaw_last;
        temp = k * dis + d * abs(0-dis_2);
        err_yaw_last = dis;
        return temp;
      }
  };
#endif