#include "se_planner.h"

using namespace std;

SEPlanner::SEPlanner(ros::NodeHandle &_n):nh(_n)
  , rob_ctrl(_n), order_hflrb(0), tfListener(tfBuffer), lock_moving(false)
  , lastvx(0.), lastrz(0.), moving_flag(false), showdwa(false), turnback_cnt(0)
{
    reset();
    readParam();
    initSubPub();
}

SEPlanner::~SEPlanner(){
    rob_ctrl.move(0.,0.);
    elevation_GM.clearAll();
    semantic_GM.clearAll();
}

void SEPlanner::orderandGo(){
    Eigen::Vector3d target_pos;
    if(moving_flag){
//        cout<<"movebyhand:"<<movecmd[0]<<","<<movecmd[1]<<endl;
        rob_ctrl.move(movecmd[0], movecmd[1]);
    }

    if(fromOrderToTarget(order_hflrb, target_pos)){
        double vx = 0., rz = 0.;
        if(turnback_cnt > 0){
            if(turnback_cnt == 30){
                ROS_INFO("[SEPLANNER]moving back");
            }       
            --turnback_cnt;
            vx = -MAX_VX / 2;
            int accstep = int(MAX_VX / 2 / 0.1);
            if(rz > lastrz + 0.1){
                rz = lastrz + 0.1;
            }
            else if(rz < lastrz - 0.1){
                rz = lastrz - 0.1;
            }
            if(turnback_cnt > 30 - accstep){
                vx = lastvx + 0.1;
            }
            else if(lastvx > vx){
                vx = lastvx - 0.1;
            }
            lastvx = vx;
            lastrz = rz;
        }
        else{
            // find path with dwa
            bool forwardok = simpleDWA(target_pos, vx, rz);
            // cout<<"dwactrl:"<<vx<<","<<rz<<endl;
            if(not forwardok){
                turnback_cnt = 30;
            }
        }        
        if(!test_flag && !lock_moving){
            rob_ctrl.move(vx, rz);
        }
    }
}

void SEPlanner::reset(){
    ROS_INFO("[SEPLANNER]reset");
    rob_ctrl.move(0.,0.);
    order_hflrb = 0;
    lastvx = 0;
    lastrz = 0;
    movecmd[0] = 0;
    movecmd[1] = 0;
    vec_target_map.clear();
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
    nh.param("seleplanner/target_frame", target_frame, string("target"));
    nh.param("seleplanner/step_discount", step_discount, 1.0);
    nh.param("seleplanner/test_flag", test_flag, false);
    nh.param("seleplanner/height_factor", height_factor, 1.0);
    nh.param("seleplanner/turn_tune", turn_tune, 1.0);
    nh.param("seleplanner/showdwa", showdwa, true);
    nh.param("seleplanner/robot_radius", robot_radius, 0.35);
    nh.param("seleplanner/unknowgrid_factor", unknowgrid_factor, 0.9);
    nh.param("seleplanner/num_stored_target", num_stored_target, 1);
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

    if(showdwa){
        cv::namedWindow("dwa", cv::WINDOW_NORMAL);
    }
}

void SEPlanner::initSubPub(){
    semap_sub = nh.subscribe("/semantic_map", 1, &SEPlanner::semanticMapCB, this);
    elemap_sub = nh.subscribe("/elevation_map", 1, &SEPlanner::elevationMapCB, this);
    order_sub = nh.subscribe("/se_order", 1, &SEPlanner::orderCB, this);
    joy_sub = nh.subscribe("/joy", 1, &SEPlanner::joyCB, this);
    target_sub = nh.subscribe("/targetP", 1, &SEPlanner::targetCB, this);
    targetodom_sub = nh.subscribe("/targetP_odom", 1, &SEPlanner::targetodomCB, this);
    path_pub = nh.advertise<nav_msgs::Path>("/dwa_path", 1);
    crossingtype_pub = nh.advertise<std_msgs::Int16>("/crossing_type", 1);
    totarget_pub = nh.advertise<std_msgs::Bool>("/totarget", 1);
}

bool SEPlanner::fromOrderToTarget(int order, Eigen::Vector3d &target){
    geometry_msgs::TransformStamped tf_map_self;
    try{
      tf_self_map = tfBuffer.lookupTransform(map_frame, base_frame, ros::Time(0));
      tf_map_self = tfBuffer.lookupTransform(base_frame, map_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      return false;
    }
    if(order == 0){return false;}
    else if(order == 1){
        target[0] = 5;
        target[1] = 0;
    }
    else if(order == 2){
        target[0] = 3;
        target[1] = 5;
    }
    else if(order == 3){
        target[0] = 3;
        target[1] = -5;
    }
    else if(order == 4){
        target[0] = -5;
        target[1] = 0;
    }
    else if(order == 5){
        tf2::doTransform(target_map, target, tf_map_self);
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.child_frame_id = target_frame;
        transformStamped.header.frame_id = map_frame;
        transformStamped.transform.translation.x = target_map[0];
        transformStamped.transform.translation.y = target_map[1];
        transformStamped.transform.translation.z = target_map[2];
        transformStamped.transform.rotation.w = 1;
        tfBroadcaster.sendTransform(transformStamped);
//        cout<<target<<";\n";
        if( (pow(target[0],2) + pow(target[1],2)) < 0.09){  // target^2 < dist^2
            ROS_INFO("[SEPLANNER]to target...");
            std_msgs::Bool msg;
            totarget_pub.publish(msg);
            reset();
            return false; //to target
        }
    }
    return true;
}

//target in base frame
bool SEPlanner::simpleDWA(Eigen::Vector3d target, double& vx, double& rz){
    // ROS_INFO("DWA START");
    // get pose from latest tf
//    geometry_msgs::TransformStamped transformStamped;
    cv::Mat dwa_img = cv::Mat(300, 600, CV_8UC3, cv::Scalar(255,255,255)).clone();

    // ROS_INFO("score all path");
    // score all posible path
    double highest_score = -100, highest_score_b = -100, lowest_score_b = 100, score;
    int best_i_rad = -1, best_i_step, step;
    vector<double> before_fuse_vec, after_fuse_vec;  //score vector
    vector<int> step_vec;
    for(int i_rad=0; i_rad<n_directions; ++i_rad){
        score = 0;
        step = scorePosiblePath(score, target, tf_self_map, i_rad);
        // ROS_INFO("rad %i score is %f",i_rad,score);
        // cout<<"i:"<<i_rad<<", step:"<<step<<", score:"<<score<<endl;
        before_fuse_vec.push_back(score);
        after_fuse_vec.push_back(score);
        step_vec.push_back(step);
        if(lowest_score_b > score){
            lowest_score_b = score;
        }
        if(highest_score_b < score){
            highest_score_b = score;
        }
        //draw dwa
        if(showdwa){
            int startpr = 299, startpc, endpr, endpc;
            startpc = 550 - i_rad * 500 / n_directions;
            endpc = startpc - 350 / n_directions;
            endpr = startpr - 200 * (step) / (dwa_total_steps);
            cv::rectangle(dwa_img, cv::Point(startpc,startpr), cv::Point(endpc,endpr), cv::Scalar(230,2,250), 3);
        }
    }
    //smooth score to choose better path
    // ROS_INFO("smooth score and choose best path");
    for(int i_rad=0; i_rad<n_directions; ++i_rad){
        if(i_rad == 0){
            after_fuse_vec[0] = (before_fuse_vec[0] + before_fuse_vec[1]) / 2;
        }
        else if(i_rad == n_directions-1){
            after_fuse_vec[i_rad] = (before_fuse_vec[i_rad] + before_fuse_vec[i_rad-1]) / 2;
        }
        else{
            after_fuse_vec[i_rad] = (before_fuse_vec[i_rad] + before_fuse_vec[i_rad-1] + before_fuse_vec[i_rad+1]) / 3;
        }
        if(highest_score < after_fuse_vec[i_rad]){
            highest_score = after_fuse_vec[i_rad];
            best_i_rad = i_rad;
            best_i_step = step_vec[i_rad];
        }
        // cout<<"score: "<<after_fuse_vec[i_rad]<<", i:"<<i_rad<<endl;
        //draw dwa
        if(showdwa){
            int startpr = 299, startpc, endpr, endpc;
            startpc = 550 - i_rad * 500 / n_directions;
            endpc = startpc - 300 / n_directions;
            // endpr = startpr - 200 * (after_fuse_vec[i_rad] + 5) / (10);
            endpr = startpr - 200 * (after_fuse_vec[i_rad] - lowest_score_b) / (highest_score_b - lowest_score_b);
            cv::rectangle(dwa_img, cv::Point(startpc,startpr), cv::Point(endpc,endpr), cv::Scalar(123,222,0), -1);
        }
    }

    // calculate vx rz with best turn_rad
    // ROS_INFO("draw data");
    double turn_rad = (best_i_rad * resolution_turn_radius - max_turn_radius) / resolution_step,
           go_m = best_i_step * resolution_step;
    // cout<<"best i:"<<best_i_rad<<", turn_rad:"<<turn_rad<<", go_m:"<<go_m<<endl;
    if(showdwa){
        // legend
        cv::putText(dwa_img, "score", cv::Point(50,20), cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(123,222,0));
        cv::putText(dwa_img, "dist", cv::Point(50,50), cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(230,2,250));
        cv::putText(dwa_img, "chosen", cv::Point(50,80), cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(223,22,0));     

        int startpr = 299, startpc, endpr, endpc;
        startpc = 550 - best_i_rad * 500 / n_directions;
        endpc = startpc - 350 / n_directions;
        // endpr = startpr - 200 * (after_fuse_vec[best_i_rad] + 5) / (10);
        endpr = startpr - 200;
        cv::rectangle(dwa_img, cv::Point(startpc,startpr), cv::Point(endpc,endpr), cv::Scalar(223,22,0), -1);
        cv::line(dwa_img, cv::Point(300,0), cv::Point(300,299), cv::Scalar(2,2,0));
        cv::imshow("dwa", dwa_img);
        cv::waitKey(1);
    }
    pubDWAPath(best_i_rad, best_i_step);
    calCmd(vx, rz, turn_rad, go_m);
    return best_i_step > 0;
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
    double totalcost = 0, cost, dist_2;
    // ROS_INFO("score rad %i",i_rad);
    int step=0, fake_step = 0;
    for(; step<dwa_total_steps; ++step){
//         ROS_INFO("score step %i",step);
        //cal pos of next step
        getNextPos(this_pos, transformStamped, i_rad, step);
//        ROS_INFO("get pos");
        //cal cost between last and this
        if(!calStepCost(cost, last_pos, this_pos)){
            ++step;
            break;
        }
//        ROS_INFO("cal cost");
        totalcost += cost;
        last_pos = this_pos;
        dist_2 = (pow(target_map[0]-this_pos[0],2) + pow(target_map[1]-this_pos[1],2));
//        cout<<"dist to target:"<<dist_2<<";"<<this_pos<<","<<target<<endl;
        if(dist_2 < 0.04){
            // to target
            fake_step = dwa_total_steps;
            step += 2;
            break;
        }
        fake_step = step;
    }
    //cal final reward about getting close to target
    --step;
    --fake_step;
    // ROS_INFO("localframe tar(%f,%f), end(%f,%f)(%i,%i)",target[0], target[1], dwa_path_points[i_rad][step][0],dwa_path_points[i_rad][step][1],i_rad,step);
    double dist = sqrt(pow((target[0]-dwa_path_points[i_rad][step][0]),2) + pow((target[1]-dwa_path_points[i_rad][step][1]),2));
    if(false){//fake_step < step && step < 3){
        score = -5;
    }
    else{
        score = step_discount * fake_step * resolution_step - dist_discount * dist - totalcost;  //to be confirmed
    }
    if(score < -5){
        score = -5;
    }
    // ROS_INFO("score of rad %i, step %i(%i), dist %f, movecost %f, score %f",i_rad, step,fake_step, dist, totalcost, score);
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
    double dh = calDelHeight(h_1, h_2);

    if(dh > Dheight || not checkAroundAdvance(this_pos)){
        cost = 100;
        return false;
    }
    //to be confirmed
    //cout<<"prob2:"<<pro_2<<",segtype_2:"<<segtype_2<<",dh:"<<dh<<endl;
    cost = (2 - pro_2) * (height_factor*dh + resolution_step) * type_factor[segtype_2];
    return true;
}

double SEPlanner::calDelHeight(double h_1, double h_2){
    double dh;
    if(isnan(h_1) or isnan(h_2)){dh = Dheight * unknowgrid_factor;}
    else{dh = fabs(h_1 - h_2);}
    // cout<<h_1<<", "<<h_2<<", "<<dh<<"==="<<endl;
    return dh;
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

// similar to check around, with checking the area of radius of robot
// return true if safe
bool SEPlanner::checkAroundAdvance(grid_map::Position pos){
    grid_map::Index ind;
    grid_map::Length maplength = elevation_GM.getLength();
    if(elevation_GM.getIndex(pos,ind)){
        double nearheight, height = elevation_GM.at("elevation", ind);
        grid_map::Matrix& data = elevation_GM["elevation"];
        for(grid_map::CircleIterator iterator(elevation_GM, pos, robot_radius); !iterator.isPastEnd(); ++iterator){
            const grid_map::Index nearind(*iterator);
            // cout<<"checkAroundindex: "<<nearind(0)<<", "<<nearind(1)<<endl;
            // if(nearind(0) < 0 or nearind(0) > maplength(0) or nearind(1) < 0 or nearind(1) > maplength(1)){
            //     continue;
            // }
            double hnear = data(nearind(0), nearind(1));
            double dh = calDelHeight(hnear, height);
            // cout<<"checkAroundheight: "<<hnear<<endl;
            if(dh > Dheight){
                // cout<<hnear<<", "<<height<<", "<<dh<<"==="<<endl;
                return false;
            }
        }
    }
    else{
        // how to deal with unknown grid
        return true;
    }
    return true;
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
    //cout<<"labelexist:"<<semantic_GM.exists("label")<<",index:"<<semantic_GM.getIndex(pos,ind)<<endl;
    if(semantic_GM.exists("label") && semantic_GM.getIndex(pos,ind)){
        segtype = semantic_GM.at("label", ind);
        prob = semantic_GM.at("prob", ind);
        //cout<<"segtype:"<<segtype<<",prob:"<<prob<<endl;
        if(isnan(segtype) || isnan(prob) || !(segtype>=0 && segtype<type_factor.size())){
            segtype = 3; //to be confirmed
            prob = 0;
        }
    }
    else{
        segtype = 0; //set to other type, to be confirmed
        prob = 1. / type_factor.size();
    }
//    cout<<"semantic:"<<segtype<<",pro:"<<prob<<endl;
    //segtype = 1;//for debug
    //prob = 1;//for debug
}

void SEPlanner::calCmd(double& vx, double& rz, double rad, double gol){
    if(lastvx < 0){
        lastvx = 0;
        lastrz = 0;
    }
    if(gol > MAX_VX * 3)
        vx = MAX_VX;
    else{
        vx = gol / 3;
    }
    rz = vx * rad * turn_tune;
    if(fabs(rz) > MAX_RZ){
        vx = MAX_RZ / fabs(rad) / turn_tune;
        rz = MAX_RZ * rz / fabs(rz);
    }
    // smooth
    int minstepstovx = int(fabs(vx - lastvx) / 0.1)+1;
    int minstepstorz = int(fabs(rz - lastrz) / 0.1)+1;
    double delvx = 0.1, delrz = 0.1;
    if(minstepstovx > minstepstorz){
        delrz = (rz - lastrz) / minstepstovx;
        delvx = (vx - lastvx) / minstepstovx;
    }
    else{
        delrz = (rz - lastrz) / minstepstorz;
        delvx = (vx - lastvx) / minstepstorz;
    }

    vx = lastvx + delvx;
    rz = lastrz + delrz;
    //cout<<"vx:"<<vx<<" rz:"<<rz<<" gol:"<<gol<<" rad:"<<rad<<", delvx:"<<delvx<<", delrz:"<<delrz<<", minstepvx:"<<minstepstovx<<", minsteprz:"<<minstepstorz<<endl;

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
        lock_moving = true;
        ROS_INFO("[SEPLANNER]lock moving.");
        reset();
    }
    else if(msg->buttons[3] > 0.9){
        //btn Y
        lock_moving = false;
        ROS_INFO("[SEPLANNER]unlock moving.");
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
    else if(msg->buttons[4] > 0.9){
        // LB
        ROS_INFO("[SEPLANNER]set to crossing by manual.");
        std_msgs::Int16 msg;
        msg.data = 1;
        crossingtype_pub.publish(msg);
    }
    else if(msg->buttons[5] > 0.9){
        // RB
        ROS_INFO("[SEPLANNER]set to road by manual.");
        std_msgs::Int16 msg;
        msg.data = 0;
        crossingtype_pub.publish(msg);
    }
    else if(fabs(msg->axes[1]) > 0.05 || fabs(msg->axes[3]) > 0.05){
        movecmd[0] = msg->axes[1] * MAX_VX;
        movecmd[1] = msg->axes[3] * MAX_RZ;
//        rob_ctrl.move(movecmd[0], movecmd[1]);
        if(!moving_flag){
            ROS_INFO("[SEPLANNER]control by hand.");
        }
        moving_flag = true;
    }
    else if(moving_flag){
        rob_ctrl.move(0,0);
        moving_flag = false;
        movecmd[0] = 0;
        movecmd[1] = 0;
    }
}

void SEPlanner::targetCB(const geometry_msgs::PoseConstPtr &msg){
    // pub local target to tf map frame
    // in fact, this should be done by the publisher of local target
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform(map_frame, base_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }
    Eigen::Vector3d pos_base;
    pos_base[0] = msg->position.x;
    pos_base[1] = msg->position.y;
    pos_base[2] = msg->position.z;
    tf2::doTransform(pos_base, target_map, transformStamped);
    order_hflrb = 5;
}

void SEPlanner::targetodomCB(const geometry_msgs::PoseConstPtr &msg){
    Eigen::Vector3d tmptarmap(msg->position.x, msg->position.y, msg->position.z);
    if(vec_target_map.size() == 0){
        vec_target_map.push_back(tmptarmap);
    }
    else{
        // judge whether the same as the last one
        Eigen::Vector3d dellastthis = tmptarmap - vec_target_map[vec_target_map.size()-1];
        if(dellastthis.norm() > 0.0){
            // add this to vector
            vec_target_map.push_back(tmptarmap);
            if(vec_target_map.size() > num_stored_target){
                // only store last 5
                vec_target_map.erase(vec_target_map.begin());
            }
        }
    }
    target_map << 0,0,0;
    for(int i=0; i<vec_target_map.size(); ++i){
        target_map += vec_target_map[i];
    }
    target_map = target_map / vec_target_map.size();
    order_hflrb = 5;
    cout<<"get odom target:"<<tmptarmap[0]<<", "<<tmptarmap[1]<<"; smoothed odom target: "<<target_map[0]<<", "<<target_map[1]<<endl;
}