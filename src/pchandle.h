#ifndef PCHANDLE_H
#define PCHANDLE_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <math.h>

#include "ros/ros.h"
#include "ros/time.h"

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>


class pchandle{
public:
    pchandle(ros::NodeHandle &_n);
    ~pchandle();
private:
    ros::NodeHandle nh;
    ros::Subscriber pc_sub;
    ros::Subscriber odom_sub;
    ros::Publisher pc_pub;

    bool getpose;
    Eigen::Vector3f _t;
    Eigen::Quaternionf _q;

    void initPubSub();

    void pcCB(const sensor_msgs::PointCloud2ConstPtr& msg);
    void odomCB(const nav_msgs::OdometryConstPtr & msg);
};


#endif
