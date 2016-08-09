#include "turtlebotMove.h"

move::move(int argc, char **argv, float _linear_speed, float _angular_speed) : _linear_speed(_linear_speed), _angular_speed(_angular_speed)
{
	std::cout << ">>> turtlebot move create, _linear_speed = " << _linear_speed << ", _angular_speed = " << _angular_speed << std::endl;

	ros::init(argc, argv, "turtlebot_move");
	ros::NodeHandle n;
	//_move_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	_move_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1000);
	_loop_rate = 2;
}

move::~move()
{
	std::cout << ">>> turtlebot move destroy" << std::endl;

	geometry_msgs::Twist move_cmd;
	move_cmd.linear.x = 0;
	move_cmd.linear.y = 0;
	move_cmd.linear.z = 0;
	move_cmd.angular.x = 0;
	move_cmd.angular.y = 0;
	move_cmd.angular.z = 0;

	_move_pub.publish(move_cmd);
}

void move::setDistance(float distance)
{
	int direction = distance > 0 ? 1 : -1;
	int timeThreshold = direction * distance / _linear_speed;
	
	//timeThreshold *= _loop_rate;
	//timeThreshold += 5;
	//timeThreshold = timeThreshold * _loop_rate + 5;
	timeThreshold = timeThreshold * _loop_rate + 10;
	
	ros::Rate loop_rate(_loop_rate);
	geometry_msgs::Twist move_cmd;
	int count = 0;

	std::cout << ">>> turtelbotmove SetDistance = " << distance << ", timeThreshold = " << timeThreshold << std::endl;

	while (1)
	{
		move_cmd.linear.x = direction * _linear_speed;
		move_cmd.linear.y = 0;
		move_cmd.linear.z = 0;
		move_cmd.angular.x = 0;
		move_cmd.angular.y = 0;
		move_cmd.angular.z = 0;

		_move_pub.publish(move_cmd);

		ros::spinOnce();

		loop_rate.sleep();
		std::cout << ">>> turtelbotmove Distance time = " << count++ << std::endl;

		if(count >= timeThreshold)
		  	break;
	}	

	move_cmd.linear.x = 0;
	move_cmd.linear.y = 0;
	move_cmd.linear.z = 0;
	move_cmd.angular.x = 0;
	move_cmd.angular.y = 0;
	move_cmd.angular.z = 0;

	_move_pub.publish(move_cmd);
	ros::spinOnce();
}

void move::setAngular(float angular)
{
	int direction = angular > 0 ? 1 : -1;
	int timeThreshold = direction * angular / _angular_speed;
	
	//timeThreshold *= _loop_rate * 2 + 40;
	//timeThreshold += 5;
	//timeThreshold = timeThreshold * _loop_rate + 5;
	timeThreshold = timeThreshold * _loop_rate + 24;

	ros::Rate loop_rate(_loop_rate);
	geometry_msgs::Twist move_cmd;
	int count = 0;

	std::cout << ">>> turtelbotmove setAngular = " << angular << ", timeThreshold = " << timeThreshold << std::endl;

	while (1)
	{
		move_cmd.linear.x = 0;
		move_cmd.linear.y = 0;
		move_cmd.linear.z = 0;
		move_cmd.angular.x = 0;
		move_cmd.angular.y = 0;
		move_cmd.angular.z = direction * _angular_speed;

		_move_pub.publish(move_cmd);

		ros::spinOnce();

		loop_rate.sleep();
		std::cout << ">>> turtelbotmove angular time = " << count++ << std::endl;

		if(count >= timeThreshold)
			break;
  	}

	move_cmd.linear.x = 0;
	move_cmd.linear.y = 0;
	move_cmd.linear.z = 0;
	move_cmd.angular.x = 0;
	move_cmd.angular.y = 0;
	move_cmd.angular.z = 0;

	_move_pub.publish(move_cmd);
	ros::spinOnce();
}