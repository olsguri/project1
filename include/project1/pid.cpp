#include <project1/pid.h>

PID::PID(){

    /* TO DO
     *
     * find appropriate value of the coefficients for PID control, and initialize them.
     *
    */

	Kp=1.0;
	Ki=1.0;
	Kd=1.0;
	error = 0.0;
	error_sum = 0.0;
	error_diff = 0.0;
	

}

float PID::get_control(point car_pose, point goal_pose){

    float ctrl;
	error_sum += error;      
	error_diff =car_pose.th - goal_pose.th - error;
        error = car_pose.th - goal_pose.th;

	ctrl = Kp* error + Ki*error_sum + Kd*error_diff;   
 /* TO DO
     *
     * implement pid algorithm
     *
    */

    return ctrl;
}
