#include "Pioneer3AT.h"

using namespace std;

Pioneer3AT::Pioneer3AT(ros::NodeHandle &_n):
    nh(_n)
{
    initPublisherSubscriber();
}

Pioneer3AT::~Pioneer3AT(){}

void Pioneer3AT::move(float forwardx, float rotatez)
{
    geometry_msgs::Twist _vel;
    _vel.linear.x = forwardx;
    _vel.linear.y = 0;
    _vel.linear.z = 0;
    _vel.angular.x = 0;
    _vel.angular.y = 0;
    _vel.angular.z = rotatez;

    vel_pub.publish(_vel);
}

void Pioneer3AT::takeOff()
{
    std_srvs::Empty _srv;
    enable_srv.call(_srv);
    move(0, 0);
}

void Pioneer3AT::land()
{
    move(0, 0);
    std_srvs::Empty _srv;
    disable_srv.call(_srv);
}

void Pioneer3AT::initPublisherSubscriber()
{
    //publisher
    vel_pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);

    //subscriber

    //service client
    enable_srv = nh.serviceClient<std_srvs::Empty>("RosAria/enable_motors");
    disable_srv = nh.serviceClient<std_srvs::Empty>("RosAria/disable_motors");
}
