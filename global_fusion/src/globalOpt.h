/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once
#include <vector>
#include <map>
#include <iostream>
#include <mutex>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "LocalCartesian.hpp"
#include "tic_toc.h"
#include <fstream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

//#include "lidarFactor.hpp"
#include "common.h"


using namespace std;




class GlobalOptimization
{
public:
	GlobalOptimization();
	~GlobalOptimization();
	void set_map_theta(double map_thetat,bool map_theta_optimizet);
	void inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy);
	void inputGPSviz(double t, double latitude, double longitude, double altitude, double posAccuracy);
     void inputGPSPR(double t, double latitude, double longitude, double altitude, double posAccuracy);
     void inputPPKviz(double t, double latitude, double longitude, double altitude, double posAccuracy);
     void inputFRLviz(double t, double latitude, double longitude, double altitude, double w, double x, double y, double z);
	void inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ);
	void inputLoam(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ);
	void getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ);
     void GPS2XYZ(double latitude, double longitude, double altitude, double* xyz);
     void pointAssociateToMap(PointType const *const pi, PointType *const po);
     void transformUpdate();
     void transformAssociateToMap();
     void inputRot(double t, double q_w, double q_x, double q_y, double q_z, double rotAccuracy);
     void inputMag(double t, double mag_x, double mag_y, double mag_z, double magAccuracy);
     void inputSurfnCorners(double t,pcl::PointCloud<PointType>::Ptr& laserCloudCornerLast ,pcl::PointCloud<PointType>::Ptr& laserCloudSurfLast);
     //void inputCloudFullRes(double t,pcl::PointCloud<PointType>::Ptr& laserCloudFullRes);
     bool isbusy();
     double getLoamTime();
     bool getLoamOdom(double time, Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ);
	nav_msgs::Path global_path;
     nav_msgs::Path gps_path; 
     nav_msgs::Path ppk_path;
     nav_msgs::Path frl_path;
     //pcl::PointCloud<PointType>::Ptr laserCloudFullRes2;
     //double timeLaserCloudFullResLast; //used to check if theres any new cloud to process
	double last_update_time;
     std::mutex mPoseMap;
	//double last_GPS=0; //rav

     // for lidar mapping
     int laserCloudCenWidth = 10;
	int laserCloudCenHeight = 10;
	int laserCloudCenDepth = 5;
	const int laserCloudWidth = 21;
	const int laserCloudHeight = 21;
	const int laserCloudDepth = 11;


	const int laserCloudNum = 4851; //4851


	//const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth; //4851
     //const int laserCloudNum;

	int laserCloudValidNum;
	int laserCloudValidInd[125];
	int laserCloudSurroundInd[125];

    double timeLaserCloud = 0.0;
	
	// input: from odom
	pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
	pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;

	// ouput: all visualble cube points
	pcl::PointCloud<PointType>::Ptr laserCloudSurround;

	// surround points in map to build tree
	int laserCloudCornerFromMapNum;
	int laserCloudSurfFromMapNum;
	pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
	pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;

	//input & output: points in one frame. local --> global
	pcl::PointCloud<PointType>::Ptr laserCloudFullRes;

	// points in every cube
	pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[4851];
	pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[4851];
	
	// points in the current scan
	int laserCloudCornerStackNum;
     int laserCloudSurfStackNum;
	pcl::PointCloud<PointType>::Ptr laserCloudCornerStack;
	pcl::PointCloud<PointType>::Ptr laserCloudSurfStack;

	//kd-tree
	pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
	pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;	

	Eigen::Quaterniond q_w_curr;
	Eigen::Vector3d t_w_curr;

	Eigen::Quaterniond q_wmap_wodom;
	Eigen::Vector3d t_wmap_wodom;

	Eigen::Quaterniond q_wodom_curr;
	Eigen::Vector3d t_wodom_curr;

    Eigen::Quaterniond q_I_L;
	Eigen::Vector3d t_I_L;

	double map_theta;
    bool map_theta_optimize;

	Eigen::Vector3d p_loam_pre;
	Eigen::Vector3d p_loam;
	Eigen::Quaterniond q_loam_pre;
	Eigen::Quaterniond q_loam;
    int pre_loam_i;
    bool initLoamMap;

    //downsample filters
	pcl::VoxelGrid<PointType> downSizeFilterCorner;
	pcl::VoxelGrid<PointType> downSizeFilterSurf;

	//point selections to support pointwise iteration	
	PointType pointOri, pointSel;
    std::vector<int> pointSearchInd;
	std::vector<float> pointSearchSqDis;


private:
	
	void optimize();
	void updateGlobalPath();
	void do_loam_mapping_prep();
	void do_loam_mapping_optimize();
	void do_loam_mapping_map_update();

	// format t, tx,ty,tz,qw,qx,qy,qz
	map<double, vector<double>> localPoseMap;
	map<double, vector<double>> globalPoseMap;
    map<double, vector<double>> globalRotMap;
	map<double, vector<double>> GPSPositionMap;
	map<double, vector<double>> GPSPositionMapViz;
    map<double, vector<double>> GPSPRPositionMap;
    map<double, vector<double>> PPKPositionMap;
    map<double, vector<double>> FRLPoseMap;
    map<double, vector<double>> magMap;
    map<double, vector<double>> loamPoseMap;
	bool initGPS;
	bool newGPS;
	bool newGPSPR;
     bool newRot;
     bool newMag;
     bool newCloudFullRes;
     bool newCloud;
     bool flag_lidar_sync;
     bool newLoam;
	
	GeographicLib::LocalCartesian geoConverter;
	Eigen::Matrix4d WGPS_T_WVIO;
        Eigen::Matrix4d WGPS_T_WVIO_viz;
        int update_count;
        int GTframeCount;
        std::ofstream outfileOdom;
        std::ofstream outfileGt;
	   std::ofstream outfileVINS;   //rav
	   std::ofstream outfileGPS;    //rav
	   std::ofstream outfileFusion; //rav
	Eigen::Vector3d lastP;
	Eigen::Quaterniond lastQ;
	std::thread threadOpt;
};
