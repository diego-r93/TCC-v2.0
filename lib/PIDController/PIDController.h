#ifndef PIDCONTROLLER
#define PIDCONTROLLER

#include <algorithm>
#include <string>

#include "stdint.h"

class Pid {
  public:
   enum class Action : uint8_t { direct,
                                 reverse };  // controller action

   Pid();
   float pid_control(float input, float target);

   void setKp(double value);
   void setKi(double value);
   void setKd(double value);

   void setControllerDirection(Action direction);
   void setControllerDirection(uint8_t direction);
   void setControllerDirection(const std::string& direction);

   double getKp();  // proportional gain
   double getKi();  // integral gain
   double getKd();  // derivative gain

   float get_lastError();
   float get_lastSpeed();
   float get_pid();

  private:
   double kp;  // (P)roportional Tuning Parameter
   double ki;  // (I)ntegral Tuning Parameter
   double kd;  // (D)erivative Tuning Parameter

   Action controller_direction;

   unsigned long last_run;
   float last_error;
   float last_speed;
   float proportional;
   float integral;
   float derivative;
};

#endif