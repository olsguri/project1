#include <cmath>
#ifndef POINT_H
    #define POINT_H
    #include <project1/point.h>
#endif

class PID{
public:
    PID();

    //this function makes control output using arguments which are the current value and the target setpoint.
    float get_control(point car_pose, point goal_pose);
private:
    float error;
    float error_sum;
    float error_diff;

    // you can use this private member variables or additionally define other member variables as you want.
/*    float error;
    float error_sum;
    float error_diff;*/
    float pre_x;
    float pre_y;
    float Kp;
    float Ki;
    float Kd;
};
