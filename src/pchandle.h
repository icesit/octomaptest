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
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

using namespace std;

class pchandle{
public:
    pchandle(ros::NodeHandle &_n);
    ~pchandle();

    void render();
private:
    ros::NodeHandle nh;
    ros::Subscriber pc_sub;
    ros::Subscriber odom_sub, worldpose_sub;
    ros::Publisher pc_pub;
    ros::Publisher m_binaryMapPub,m_fullMapPub;

    bool getpose;
    //q and t of body frame in odom frame
    Eigen::Vector3d _twb;
    Eigen::Quaterniond _qwb;
    //q and t of sensor frame in body frame
    Eigen::Vector3d _tbs;
    Eigen::Quaterniond _qbs;

    octomap::OcTree* m_octree;
    std::string m_worldFrameId;
    double pointcloud_min_x,pointcloud_max_x,pointcloud_max_z,pointcloud_min_z,pointcloud_max_y,pointcloud_min_y;

    //2d map
    double detect_resolution, detect_minx, detect_maxx, detect_miny, detect_maxy, detect_minz, detect_maxz;
    cv::Mat height_im, passable_im;
    int pass_thresh;

    bool firstsubpc;
    int v1,v2;

    void initPubSub();
    void initParam();

    void publishBinaryOctoMap(const ros::Time& rostime = ros::Time::now()) const;
    void publishFullOctoMap(const ros::Time& rostime = ros::Time::now()) const;

    void pcCB(const sensor_msgs::PointCloud2ConstPtr& msg);
    void odomCB(const nav_msgs::OdometryConstPtr & msg);
    void worldCB(const geometry_msgs::PoseStampedConstPtr & msg);

    void project2Dheightmap();
};


#endif
