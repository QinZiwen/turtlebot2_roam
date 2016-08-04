#ifndef _TURTLEBOTMOVE_H
#define _TURTLEBOTMOVE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

//using namespace std;

class move
{
public:
	move(int argc, char **argv, float _linear_speed = 0.045, float _angular_speed = 0.18);
	~move();

	void setDistance(float distance);
	void setAngular(float angular);
	void setLinearSpeed(float sp){ _linear_speed = sp; }
	void setAngularSpeed(float ag){ _angular_speed = ag; }
	float getLinearSpeed(){ return _linear_speed; }
	float getAngularSpeed(){ return _angular_speed; }

private:
	ros::Publisher _move_pub;

	float _linear_speed;
	float _angular_speed;
	float _loop_rate;
};

#endif