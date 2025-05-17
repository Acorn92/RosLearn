#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>


// Данная программа осуществляет передачу данных о желаемом положении БЛА
// В рамках задания нужно будет выполнять полет на основе информации от этой программы.
// Целевая траектория окружность.
int main(int argc, char **argv)
{
	ros::init(argc, argv, "sunflower_controller_node");
	ros::NodeHandle n;
	geometry_msgs::PoseStamped		desPose;
	ros::Publisher					desPosePub = n.advertise<geometry_msgs::PoseStamped>("/vehicle/desPose", 1);
	double							counter = 0;
	double							counterStep = 0.01;	
	double							circleRadius = 5;	
	ros::Rate 						rate(20);
	desPose.pose.position.z = 3;
	while(ros::ok())
	{
		desPose.pose.position.x = circleRadius * std::cos(counter);
		desPose.pose.position.y = circleRadius * std::sin(counter);
		desPosePub.publish(desPose);
		counter += counterStep;
		rate.sleep();
	}

}
