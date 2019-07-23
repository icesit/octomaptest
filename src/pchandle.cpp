#include "pchandle.h"

pchandle::pchandle(ros::NodeHandle &_n):
    nh(_n), getpose(0)
  ,m_worldFrameId("/odom")
{
    initParam();
    initPubSub();
}

pchandle::~pchandle(){
    if (m_octree){
        delete m_octree;
        m_octree = NULL;
    }
}

void pchandle::initPubSub(){
    pc_sub = nh.subscribe("/inpoints", 1, &pchandle::pcCB, this);
    if(m_worldFrameId == "/odom"){
        odom_sub = nh.subscribe("/odom", 1, &pchandle::odomCB, this);
    }
    else{
        worldpose_sub = nh.subscribe("/world", 1, &pchandle::worldCB, this);
    }

    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/outpoints", 1);
    m_binaryMapPub = nh.advertise<octomap_msgs::Octomap>("/octomap_binary", 1);
    m_fullMapPub = nh.advertise<octomap_msgs::Octomap>("/octomap_full", 1);
}

void pchandle::initParam(){
    std::string nodename = "octotest/";
    nh.param(nodename+"qbs_w", _qbs.w(), 1.0);
    nh.param(nodename+"qbs_x", _qbs.x(), 0.0);
    nh.param(nodename+"qbs_y", _qbs.y(), 0.0);
    nh.param(nodename+"qbs_z", _qbs.z(), 0.0);
    nh.param(nodename+"tbs_x", _tbs.x(), 0.0);
    nh.param(nodename+"tbs_y", _tbs.y(), 0.0);
    nh.param(nodename+"tbs_z", _tbs.z(), 0.0);
    std::cout<<"from body to sensor q:"<<_qbs.w()<<","<<_qbs.x()
            <<","<<_qbs.y()<<","<<_qbs.z()
           <<";t:"<<_tbs.x()<<","<<_tbs.y()<<","<<_tbs.z()<<std::endl;

    double m_res,probHit,probMiss,thresMin,thresMax;
    nh.param(nodename+"resolution", m_res, 0.1);
    nh.param(nodename+"sensor_model/hit", probHit, 0.7);
    nh.param(nodename+"sensor_model/miss", probMiss, 0.4);
    nh.param(nodename+"sensor_model/min", thresMin, 0.12);
    nh.param(nodename+"sensor_model/max", thresMax, 0.97);
    nh.param(nodename+"world_frame_id", m_worldFrameId, m_worldFrameId);
    nh.param(nodename+"pointcloud_min_x", pointcloud_min_x, 0.0);
    nh.param(nodename+"pointcloud_max_x", pointcloud_max_x, 20.0);
    nh.param(nodename+"pointcloud_min_z", pointcloud_min_z, -10.0);
    nh.param(nodename+"pointcloud_max_z", pointcloud_max_z, 10.0);
    m_octree = new octomap::OcTree(m_res);
    m_octree->setProbHit(probHit);
    m_octree->setProbMiss(probMiss);
    m_octree->setClampingThresMin(thresMin);
    m_octree->setClampingThresMax(thresMax);

    nh.param(nodename+"detect_minx", detect_minx, 0.0);
    nh.param(nodename+"detect_maxx", detect_maxx, 3.0);
    nh.param(nodename+"detect_miny", detect_miny, -3.0);
    nh.param(nodename+"detect_maxy", detect_maxy, 3.0);
    nh.param(nodename+"detect_minz", detect_minz, -1.0);
    nh.param(nodename+"detect_maxz", detect_maxz, 0.0);
    nh.param(nodename+"detect_resolution", detect_resolution, 0.1);
    int row = int((detect_maxx - detect_minx) / detect_resolution)+1,
        col = int((detect_maxy - detect_miny) / detect_resolution)+1;
    height_im = cv::Mat(row, col, CV_8UC3, cv::Scalar(0,0,0)).clone();
    cv::namedWindow("height_im", 0);
}

// point cloud in sensor frame
void pchandle::pcCB(const sensor_msgs::PointCloud2ConstPtr& msg){
    if(!getpose){
        return;
    }

    // turn pointcloud to world frame
    Eigen::Vector3d _tws = _qwb*_tbs + _twb;
    Eigen::Quaterniond _qws = _qwb*_qbs;

    //turn to base frame
    pcl::PointCloud<pcl::PointXYZ> oripc, dstpc;
    pcl::fromROSMsg(*msg, oripc);
    pcl::transformPointCloud(oripc, dstpc, _tbs, _qbs);

    //filter points
    pcl::PassThrough<pcl::PointXYZ> pass_x, pass_z;
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(pointcloud_min_x, pointcloud_max_x);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(pointcloud_min_z, pointcloud_max_z);
    pass_x.setInputCloud(dstpc.makeShared());
    pass_x.filter(dstpc);
    pass_z.setInputCloud(dstpc.makeShared());
    pass_z.filter(dstpc);

    //turn to world frame
    pcl::transformPointCloud(dstpc, dstpc, _twb, _qwb);

    //insert to octree
    octomap::point3d sensorOrigin(float(_tws(0)), float(_tws(1)), float(_tws(2)));
    sensor_msgs::PointCloud2 tmp;
    octomap::Pointcloud octpointcloud;
    pcl::toROSMsg(dstpc, tmp);
    octomap::pointCloud2ToOctomap(tmp, octpointcloud);
    m_octree->insertPointCloud(octpointcloud, sensorOrigin);

    project2Dheightmap();

    //publish
    ros::Time rostime = ros::Time::now();
    publishBinaryOctoMap(rostime);
    publishFullOctoMap(rostime);
}

void pchandle::project2Dheightmap(){
    //turn octomap into 2d grid map in base frame
    int depth = m_octree->getTreeDepth();
    bool occu_flag = false;

    Eigen::Vector3d pt, pt_w;
    octomap::OcTreeKey octk;
    octomap::OcTreeNode* node;
    int im_c, im_r;
    //cout<<"get pc"<<endl;
    for(double _x=detect_minx; _x<detect_maxx;){
        pt(0) = _x;
        // real coord in base frame to image coord
        im_r = height_im.rows-1 - int((_x+0.01-detect_minx)/detect_resolution);
        //cout<<_x<<","<<im_r<<endl;
        for(double _y=detect_miny; _y<detect_maxy;){
            pt(1) = _y;
            // real coord in base frame to image coord
            im_c = height_im.cols-1 - int((_y-detect_miny)/detect_resolution);
            //cout<<"imcr:"<<im_c<<","<<im_r<<endl;
            height_im.at<cv::Vec3b>(im_r,im_c) = cv::Vec3b(0,0,0);
            for(double _z=detect_maxz; _z>detect_minz;){
                pt(2) = _z;
                // find occupied node from top to down
                // if this grid is occupied, record the height on 2d map
                //cout<<"ori:"<<pt(0)<<","<<pt(1)<<","<<pt(2)<<";";
                pt_w = _qwb * pt +_twb;
                //cout<<"dst:"<<pt_w(0)<<","<<pt_w(1)<<","<<pt_w(2)<<endl;
                octk = m_octree->coordToKey(pt_w(0),pt_w(1),pt_w(2),depth);
                node = m_octree->search(octk);
                if(node){
                    occu_flag = m_octree->isNodeOccupied(node);
                    if(occu_flag){
                        //cout<<"occupied"<<endl;
                        int value = int((_z - detect_minz)/(detect_maxz - detect_minz)*255);
                        height_im.at<cv::Vec3b>(im_r,im_c) = cv::Vec3b(0,0,value);
                        break;
                    }
                }
                _z -= detect_resolution;
            }
            _y += detect_resolution;
        }
        _x += detect_resolution;
    }
    cv::imshow("height_im", height_im);
    cv::waitKey(1);
}

void pchandle::odomCB(const nav_msgs::OdometryConstPtr & msg){
    getpose = true;
    _twb(0) = msg->pose.pose.position.x;
    _twb(1) = msg->pose.pose.position.y;
    _twb(2) = msg->pose.pose.position.z;
    _qwb.w() = msg->pose.pose.orientation.w;
    _qwb.x() = msg->pose.pose.orientation.x;
    _qwb.y() = msg->pose.pose.orientation.y;
    _qwb.z() = msg->pose.pose.orientation.z;
}

void pchandle::worldCB(const geometry_msgs::PoseStampedConstPtr & msg){
    getpose = true;
    _twb(0) = msg->pose.position.x;
    _twb(1) = msg->pose.position.y;
    _twb(2) = msg->pose.position.z;
    _qwb.w() = msg->pose.orientation.w;
    _qwb.x() = msg->pose.orientation.x;
    _qwb.y() = msg->pose.orientation.y;
    _qwb.z() = msg->pose.orientation.z;
}

void pchandle::publishBinaryOctoMap(const ros::Time& rostime) const{

  octomap_msgs::Octomap map;
  map.header.frame_id = m_worldFrameId;
  map.header.stamp = rostime;

  if (octomap_msgs::binaryMapToMsg(*m_octree, map))
    m_binaryMapPub.publish(map);
  else
    ROS_ERROR("Error serializing OctoMap");
}

void pchandle::publishFullOctoMap(const ros::Time& rostime) const{

  octomap_msgs::Octomap map;
  map.header.frame_id = m_worldFrameId;
  map.header.stamp = rostime;

  if (octomap_msgs::fullMapToMsg(*m_octree, map))
    m_fullMapPub.publish(map);
  else
    ROS_ERROR("Error serializing OctoMap");

}
