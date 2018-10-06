#include <project1/pid.h>

PID::PID(){

    /* TO DO
     *
     * find appropriate value of the coefficients for PID control, and initialize them.
     *
    */

	Kp=0.5;
	Ki=0.0;
	Kd=0.0;
	error = 0.0;
	error_sum = 0.0;
	error_diff = 0.0;
	pre_x=0.0;
	pre_y=0.0;	

}

float PID::get_control(point car_pose, point goal_pose){

    float ctrl;
/*    float cartogoal_size=sqrt(pow(goal_pose.x-car_pose.x,2)+pow(goal_pose.y-car_pose.y,2));
    float heading_size=sqrt(pow(car_pose.x-pre_x,2)+pow(car_pose.y-pre_y,2));
    float cartogoal_x=(goal_pose.x-car_pose.x)/cartogoal_size;
    float cartogoal_y=(goal_pose.y-car_pose.y)/cartogoal_size;
    float heading_x=(car_pose.x-pre_x)/heading_size;
    float heading_y=(car_pose.y-pre_y)/heading_size;
  */  

    float angle_from_car_to_goal=atan2(goal_pose.y-car_pose.y,goal_pose.x-car_pose.x);
    float heading_angle = atan2(car_pose.y-pre_y,car_pose.x-pre_x);
	error_sum += error;
	error_diff =angle_from_car_to_goal - heading_angle  - error;
        error = angle_from_car_to_goal - heading_angle;
	if(error<= -M_PI) error += 2*M_PI;
	else if(error>=M_PI) error -= 2*M_PI;
	ctrl = Kp* error + Ki*error_sum + Kd*error_diff;  
	if(ctrl>=0.5) ctrl=0.5;
	else if(ctrl<=-0.5) ctrl=-0.5;
	pre_x=car_pose.x;
	pre_y=car_pose.y;
 /* TO DO
     *
     * implement pid algorithm
     *
    */

    return ctrl;

}
