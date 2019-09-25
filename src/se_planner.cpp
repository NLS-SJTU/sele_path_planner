#include "se_planner.h"

using namespace std;

SEPlanner::SEPlanner(ros::NodeHandle &_n):nh(_n)
  , rob_ctrl(_n), order_hflrb(0), tfListener(tfBuffer)
  , lastvx(0.), lastrz(0.)
{
    reset();
    readParam();
    initSubPub();
//    for(int i_rad=0; i_rad<n_directions; ++i_rad){
//        for(int step=0; step<dwa_total_steps; ++step){
//            ROS_INFO("(%f,%f)",dwa_path_points[i_rad][step][0],dwa_path_points[i_rad][step][1]);
//        }
//    }
}

SEPlanner::~SEPlanner(){
    rob_ctrl.move(0.,0.);
    elevation_GM.clearAll();
    semantic_GM.clearAll();
}

void SEPlanner::orderandGo(){
    Eigen::Vector3d target_pos;
    if(moving_flag){
        rob_ctrl.move(movecmd[0], movecmd[1]);
    }
    if(order_hflrb == 0){return;}
    else{
        fromOrderToTarget(order_hflrb, target_pos);
    }
    // find path with ??
    double vx = 0., rz = 0.;
    simpleDWA(target_pos, vx, rz);
    if(!test_flag){
        rob_ctrl.move(vx, rz);
    }
}

void SEPlanner::reset(){
    ROS_INFO("[SEPLANNER]reset");
    rob_ctrl.move(0.,0.);
    lastvx = 0;
    lastrz = 0;
    movecmd[0] = 0;
    movecmd[1] = 0;
}

void SEPlanner::readParam(){
    nh.param("seleplanner/max_turn_radius", max_turn_radius, 0.31415);
    nh.param("seleplanner/n_directions", n_directions, 21);
    nh.param("seleplanner/resolution_step", resolution_step, 0.2);
    nh.param("seleplanner/MAX_VX", MAX_VX, 0.5);
    nh.param("seleplanner/MAX_RZ", MAX_RZ, 0.78);
    nh.param("seleplanner/dwa_total_steps", dwa_total_steps, 10);
    nh.param("seleplanner/dist_discount", dist_discount, 1.);
    nh.param("seleplanner/Dheight", Dheight, 0.1);
    nh.param("seleplanner/map_frame", map_frame, string("slam"));
    nh.param("seleplanner/base_frame", base_frame, string("base_link"));
    nh.param("seleplanner/step_discount", step_discount, 1.0);
    nh.param("seleplanner/test_flag", test_flag, false);
    resolution_turn_radius = 2 * max_turn_radius / (n_directions - 1);
    //prepare points for dwa
    Eigen::Vector2d tmpP;

    for(int i_rad=0; i_rad<n_directions; ++i_rad){
        vector<Eigen::Vector2d> tmpV;
        for(int step=1; step<=dwa_total_steps; ++step){
            getNextPos(tmpP, i_rad, step);
            // ROS_INFO("%i rad, %i step, (%f, %f)",i_rad, step, tmpP[0], tmpP[1]);
            tmpV.push_back(tmpP);
        }
        dwa_path_points.push_back(tmpV);
    }
    //get type factor
    XmlRpc::XmlRpcValue param_list;
    nh.getParam("seleplanner/type_factor", param_list);
    for(int i=0; i<param_list.size(); ++i){
//        cout<<"--init:"<<param_list[i];
        type_factor.push_back(double(param_list[i]));
    }
}

void SEPlanner::initSubPub(){
    semap_sub = nh.subscribe("/semantic_map", 1, &SEPlanner::semanticMapCB, this);
    elemap_sub = nh.subscribe("/elevation_map", 1, &SEPlanner::elevationMapCB, this);
    order_sub = nh.subscribe("/se_order", 1, &SEPlanner::orderCB, this);
    joy_sub = nh.subscribe("/joy", 1, &SEPlanner::joyCB, this);
    path_pub = nh.advertise<nav_msgs::Path>("/dwa_path", 1);
}

void SEPlanner::fromOrderToTarget(int order, Eigen::Vector3d &target){
    if(order == 1){
        target[0] = 10;
        target[1] = 0;
    }
    else if(order == 2){
        target[0] = 10;
        target[1] = 7;
    }
    else if(order == 3){
        target[0] = 10;
        target[1] = -7;
    }
    else if(order == 4){
        target[0] = -10;
        target[1] = 0;
    }
}

//target in base frame
void SEPlanner::simpleDWA(Eigen::Vector3d target, double& vx, double& rz){
    // ROS_INFO("DWA START");
    // get pose from latest tf
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform(map_frame, base_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }
    // ROS_INFO("score all path");
    // score all posible path
    double highest_score = -100, score;
    int best_i_rad = -1, best_i_step, step;
    for(int i_rad=0; i_rad<n_directions; ++i_rad){
        score = 0;
        step = scorePosiblePath(score, target, transformStamped, i_rad);
        // ROS_INFO("rad %i score is %f",i_rad,score);
        if(highest_score < score){
            highest_score = score;
            best_i_rad = i_rad;
            best_i_step = step;
        }
    }
    // calculate vx rz with best turn_rad
    double turn_rad = (best_i_rad * resolution_turn_radius - max_turn_radius) / resolution_step,
           go_m = best_i_step * resolution_step;
    pubDWAPath(best_i_rad, best_i_step);
    calCmd(vx, rz, turn_rad, go_m);
    // ROS_INFO("DWA END");
}

void SEPlanner::pubDWAPath(int best_i_rad, int best_i_step){
    if(best_i_rad == -1) return;
    nav_msgs::Path msg;
    msg.header.frame_id = base_frame;
    msg.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped onepoint;
    onepoint.header.frame_id = base_frame;
    onepoint.pose.position.z = 0.5;
    for(int i=0; i<=best_i_step; ++i){
        onepoint.pose.position.x = dwa_path_points[best_i_rad][i][0];
        onepoint.pose.position.y = dwa_path_points[best_i_rad][i][1];
        onepoint.header.seq = i;
        msg.poses.push_back(onepoint);
    }
    path_pub.publish(msg);
}

// rad: when agv move 1m, it rotates x rad
int SEPlanner::scorePosiblePath(double& score, Eigen::Vector3d target, geometry_msgs::TransformStamped transformStamped, int i_rad){
    grid_map::Position last_pos, this_pos;
    //get first grid in map frame
    last_pos[0] = transformStamped.transform.translation.x;
    last_pos[1] = transformStamped.transform.translation.y;
    //cal sum cost of following grid
    double totalcost = 0, cost;
//    ROS_INFO("score rad %i",i_rad);
    int step=0;
    for(; step<dwa_total_steps; ++step){
        // ROS_INFO("score step %i",step);
        //cal pos of next step
        getNextPos(this_pos, transformStamped, i_rad, step);
//        ROS_INFO("get pos");
        //cal cost between last and this
        if(!calStepCost(cost, last_pos, this_pos)){
            break;
        }
//        ROS_INFO("cal cost");
        totalcost += cost;
        last_pos = this_pos;
    }
    //cal final reward about getting close to target
//    Eigen::Vector3d tar_mapframe;
//    tf2::doTransform(target, tar_mapframe, transformStamped);
//    double dist = sqrt(pow(tar_mapframe[0]-last_pos[0],2) + pow(tar_mapframe[1]-last_pos[1],2));
    --step;
    // ROS_INFO("localframe tar(%f,%f), end(%f,%f)(%i,%i)",target[0], target[1], dwa_path_points[i_rad][step][0],dwa_path_points[i_rad][step][1],i_rad,step);
    double dist = sqrt(pow((target[0]-dwa_path_points[i_rad][step][0]),2) + pow((target[1]-dwa_path_points[i_rad][step][1]),2));
    score = step_discount * step * resolution_step - dist_discount * dist - totalcost;  //to be confirmed
//    ROS_INFO("score of rad %i, step %i, dist %f, movecost %f, score %f",i_rad, step,dist, totalcost, score);
    return step;
}

// return pos in base frame
void SEPlanner::getNextPos(Eigen::Vector2d& this_pos, int i_rad, int step){
    if(i_rad == int(n_directions/2)){
        // go straight
        this_pos[0] = step * resolution_step;
        this_pos[1] = 0;
        return;
    }
    // turn
    //alpha: when go one step, agv turns alpha rad
    //rcircle: r of turning circle
    //theta: when go step steps, agv turns theta rad
    double alpha = (i_rad*resolution_turn_radius - max_turn_radius),
            rcircle = resolution_step / alpha, theta = alpha * step;
    double L = sqrt(2*(1-cos(theta))*pow(rcircle,2));
    this_pos[0] = L * sin((M_PI-fabs(theta))/2);
    this_pos[1] = L * cos((M_PI-fabs(theta))/2)*(theta<0 ? -1:1);
}

// return pos in map frame
void SEPlanner::getNextPos(grid_map::Position& this_pos, geometry_msgs::TransformStamped transformStamped, int i_rad, int step){
    Eigen::Vector3d pos_base, pos_map;
    pos_base[0] = dwa_path_points[i_rad][step][0];
    pos_base[1] = dwa_path_points[i_rad][step][1];
    tf2::doTransform(pos_base, pos_map, transformStamped);
    this_pos[0] = pos_map[0];
    this_pos[1] = pos_map[1];
}

// return false if not passable
bool SEPlanner::calStepCost(double& cost, grid_map::Position last_pos, grid_map::Position this_pos){
    double h_1, h_2, pro_1, pro_2;
    int segtype_1, segtype_2;
    getInfoFromGM(last_pos, h_1, segtype_1, pro_1);
    getInfoFromGM(this_pos, h_2, segtype_2, pro_2);
//    cout<<"segtype2"<<segtype_2<<endl;
//    ROS_INFO("get data from GM");
    double dh = 0;
    if(isnan(h_1) or isnan(h_2)){dh = Dheight/2;}
    else{dh = fabs(h_1 - h_2);}

    if(dh > Dheight || checkAround(this_pos)){
        cost = 100;
        return false;
    }
    //to be confirmed
    cost = (2 - pro_2) * (dh+resolution_step) * type_factor[segtype_2];
    return true;
}

//check whether near danger place, true is danger
bool SEPlanner::checkAround(grid_map::Position pos){
    grid_map::Index ind, indnear;
    if(elevation_GM.getIndex(pos,ind)){
        double hnear[4], height = elevation_GM.at("elevation", ind);
        grid_map::Position nearpos;
        nearpos(0) = pos(0) - resolution_step;
        nearpos(1) = pos(1);
        if(!elevation_GM.getIndex(nearpos, indnear))
            return true;
        hnear[1] = elevation_GM.at("elevation", indnear);
        nearpos(0) = pos(0) + resolution_step;
        if(!elevation_GM.getIndex(nearpos, indnear))
            return true;
        hnear[2] = elevation_GM.at("elevation", indnear);
        nearpos(0) = pos(0);
        nearpos(1) = pos(1) - resolution_step;
        if(!elevation_GM.getIndex(nearpos, indnear))
            return true;
        hnear[3] = elevation_GM.at("elevation", indnear);
        nearpos(1) = pos(1) + resolution_step;
        if(!elevation_GM.getIndex(nearpos, indnear))
            return true;
        hnear[0] = elevation_GM.at("elevation", indnear);
        if((!(hnear[0]==NAN) && fabs(hnear[0] - height) > Dheight) ||
           (!(hnear[1]==NAN) && fabs(hnear[1] - height) > Dheight) ||
           (!(hnear[2]==NAN) && fabs(hnear[2] - height) > Dheight) ||
           (!(hnear[3]==NAN) && fabs(hnear[3] - height) > Dheight)){
            return true;
        }
    }
    else{
        return true;
    }
    return false;
}

void SEPlanner::getInfoFromGM(grid_map::Position pos, double& height, int& segtype, double& prob){
    grid_map::Index ind;
    //get height from elevation map
    if(elevation_GM.exists("elevation") && elevation_GM.getIndex(pos,ind)){
        height = elevation_GM.at("elevation", ind);
    }
    else{
        height = NAN;
    }
//    cout<<"pos:"<<pos[0]<<","<<pos[1]<<", height:"<<height<<endl;
    //get label and prob from semantic map
    if(semantic_GM.exists("label") && semantic_GM.getIndex(pos,ind)){
        segtype = semantic_GM.at("label", ind);
        prob = semantic_GM.at("prob", ind);
    }
    else{
        segtype = 0; //set to other type, to be confirmed
        prob = 1 / type_factor.size();
    }
//    cout<<"semantic:"<<segtype<<",pro:"<<prob<<endl;
    //segtype = 1;//for debug
    //prob = 1;//for debug
}

void SEPlanner::calCmd(double& vx, double& rz, double rad, double gol){
    if(gol > 1.5)
        vx = lastvx + 0.1;
    else{
        vx = gol / 3;
    }
    if(vx > MAX_VX){
        vx = MAX_VX;
    }
    rz = vx * rad;
    if(fabs(rz) > MAX_RZ){
        vx = MAX_RZ / rad;
        rz = MAX_RZ;
    }
    lastvx = vx;
    lastrz = rz;
}

void SEPlanner::elevationMapCB(const grid_map_msgs::GridMapConstPtr &msg){
//    elevation_GM.clearAll();   //??necessary
    grid_map::GridMapRosConverter::fromMessage(*msg, elevation_GM);
}

void SEPlanner::semanticMapCB(const grid_map_msgs::GridMapConstPtr &msg){
    grid_map::GridMapRosConverter::fromMessage(*msg, semantic_GM);
}

/// 0:hold, 1:forward, 2:turn left, 3:right, 4:turn back
void SEPlanner::orderCB(const std_msgs::Int16ConstPtr &msg){
    order_hflrb = msg->data;
    if(order_hflrb == 0){
        reset();
    }
}

void SEPlanner::joyCB(const sensor_msgs::JoyConstPtr &msg){
    if(msg->buttons[0] > 0.9){
        //btn A
        order_hflrb = 0;
        reset();
    }
    else if(msg->axes[6] > 0.9){
        //cross left
        ROS_INFO("[SEPLANNER]turn left.");
        order_hflrb = 2;
    }
    else if(msg->axes[6] < -0.9){
        //cross right
        ROS_INFO("[SEPLANNER]turn right.");
        order_hflrb = 3;
    }
    else if(msg->axes[7] > 0.9){
        //cross up
        ROS_INFO("[SEPLANNER]go forward.");
        order_hflrb = 1;
    }
    else if(fabs(msg->axes[1]) > 0.05 || fabs(msg->axes[3]) > 0.05){
        movecmd[0] = msg->axes[1] * MAX_VX;
        movecmd[1] = msg->axes[3] *MAX_RZ;
        rob_ctrl.move(movecmd[0], movecmd[1]);
        moving_flag = true;
    }
    else if(moving_flag){
        rob_ctrl.move(0,0);
        moving_flag = false;
        movecmd[0] = 0;
        movecmd[1] = 0;
    }
}
