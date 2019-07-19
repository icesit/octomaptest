#include "pchandle.h"

pchandle::pchandle(ros::NodeHandle &_n):
    nh(_n), getpose(false)
{
    initPubSub();
}

pchandle::~pchandle(){}

void pchandle::initPubSub(){
    pc_sub = nh.subscribe("/inpoints", 1, &pchandle::pcCB, this);
    odom_sub = nh.subscribe("/odom", 1, &pchandle::odomCB, this);

    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/outpoints", 1);
}

void pchandle::pcCB(const sensor_msgs::PointCloud2ConstPtr& msg){
    if(!getpose){
        return;
    }

    //turn points from body frame to "world" frame
    pcl::PointCloud<pcl::PointXYZ> oripc, dstpc;
    pcl::fromROSMsg(*msg, oripc);
    pcl::transformPointCloud(oripc, dstpc, _t, _q);

    //publish it to octomap
    sensor_msgs::PointCloud2 outmsg;
    pcl::toROSMsg(dstpc, outmsg);
    outmsg.header.frame_id = "odom";
    pc_pub.publish(outmsg);
}

void pchandle::odomCB(const nav_msgs::OdometryConstPtr & msg){
    getpose = true;
    _t(0) = msg->pose.pose.position.x;
    _t(1) = msg->pose.pose.position.y;
    _t(2) = msg->pose.pose.position.z;
    _q.w() = msg->pose.pose.orientation.w;
    _q.x() = msg->pose.pose.orientation.x;
    _q.y() = msg->pose.pose.orientation.y;
    _q.z() = msg->pose.pose.orientation.z;
}
