#include "pchandle.h"

int main(int argc, char** argv){
    ros::init(argc, argv,"turnpc");
    ros::NodeHandle nh;
    pchandle pch(nh);
    ROS_INFO("start");
    ros::Rate rate = ros::Rate(20);
    while(ros::ok()){
        pch.render();
        ros::spinOnce();
    }
    ROS_INFO("quit");
    ros::shutdown();
    return 0;
}
