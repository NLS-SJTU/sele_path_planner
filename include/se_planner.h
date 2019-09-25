/*
path planner for semantic-elevation map
*/


#ifndef SEPLANNER_H
#define SEPLANNER_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <math.h>

#include <Eigen/Core>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Joy.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>

#include "Pioneer3AT.h"

using namespace std;

class SEPlanner{
public:
    SEPlanner(ros::NodeHandle &_n);
    ~SEPlanner();
    void orderandGo();
    void reset();

private:
    ros::NodeHandle nh;
    ros::Subscriber semap_sub, elemap_sub, order_sub, joy_sub;
    ros::Publisher path_pub;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    Pioneer3AT rob_ctrl;
    double lastvx, lastrz, movecmd[2];
    bool moving_flag, test_flag;

    int order_hflrb;
    grid_map::GridMap semantic_GM, elevation_GM;
    vector<vector<Eigen::Vector2d> > dwa_path_points; //in base frame, first direction,second forward

    //params
    double max_turn_radius, resolution_turn_radius, Dheight,
        MAX_VX, MAX_RZ, resolution_step, dist_discount, step_discount;
    int dwa_total_steps, n_directions;
    vector<double> type_factor;
    string map_frame, base_frame;

    void readParam();
    void initSubPub();

    void fromOrderToTarget(int order, Eigen::Vector3d &target);
    void getInfoFromGM(grid_map::Position pos, double& height, int& segtype, double& prob);
    bool checkAround(grid_map::Position pos);

    // dwa
    void simpleDWA(Eigen::Vector3d target, double& vx, double& rz);
//    void simpleVFH(Eigen::Vector3d target, double& vx, double& rz);
    int scorePosiblePath(double& score, Eigen::Vector3d target, geometry_msgs::TransformStamped transformStamped, int i_rad);
    void calCmd(double& vx, double& rz, double rad, double gol);
    void getNextPos(Eigen::Vector2d& this_pos, int i_rad, int step);
    void getNextPos(grid_map::Position& this_pos, geometry_msgs::TransformStamped transformStamped, int i_rad, int step);
    bool calStepCost(double& cost, grid_map::Position last_pos, grid_map::Position this_pos);
    void pubDWAPath(int best_i_rad, int best_i_step);

    void elevationMapCB(const grid_map_msgs::GridMapConstPtr &msg);
    void semanticMapCB(const grid_map_msgs::GridMapConstPtr &msg);
    void orderCB(const std_msgs::Int16ConstPtr &msg);
    void joyCB(const sensor_msgs::JoyConstPtr &msg);
};

#endif
