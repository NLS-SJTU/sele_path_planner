

#include "se_planner.h"

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv,"se_planner");
    ros::NodeHandle nh;
    SEPlanner sepl(nh);
    ros::Rate loop(20);
    ROS_INFO("[SEPLANNER]SEPlanner start running!");
    while (ros::ok()){
        sepl.orderandGo();
        ros::spinOnce();
        loop.sleep();
    }
    ros::shutdown();
    return 0;
}
