#ifndef _PIONEER3AT_H
#define _PIONEER3AT_H

#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

class Pioneer3AT
{
public:
    Pioneer3AT(ros::NodeHandle &_n);
	~Pioneer3AT();

    //input should be m/s and rad/s
    void move(float forwardx, float rotatez);
    //enable motor and stop
    void takeOff();
    //stop and disable motor
    void land();
private:
    ros::NodeHandle nh;

    ros::Publisher vel_pub;

    ros::ServiceClient enable_srv;
    ros::ServiceClient disable_srv;

    void initPublisherSubscriber();
};

#endif
