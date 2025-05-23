// VI-LOAM: A system level integration of LOAM and VINS algorithms, April 2022. 

// Modifier: Didula Dissanayaka - Intelligent Systems Lab, MUN - dissanayakadidula@gmail.com


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <math.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>

#include <sensor_msgs/NavSatFix.h>
#include "loam/geodetic_conv.hpp"
#include <geometry_msgs/PoseWithCovarianceStamped.h>



#include "loam/lidarFactor.hpp"
#include "loam/common.h"
#include "loam/tic_toc.h"
#include "loam/utility.h"

#include "nmea_msgs/Sentence.h"
#include <fstream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>


int frameCount = 0;
int GTframeCount = 0;    //skip frames when saving to files to compare with GPS update //D

double timeLaserCloudCornerLast = 0;
double timeLaserCloudSurfLast = 0;
double timeLaserCloudFullRes = 0;
double timeLaserOdometry = 0;


int laserCloudCenWidth = 10;
int laserCloudCenHeight = 10;
int laserCloudCenDepth = 5;
const int laserCloudWidth = 21;
const int laserCloudHeight = 21;
const int laserCloudDepth = 11;


const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth; //4851


int laserCloudValidInd[125];
int laserCloudSurroundInd[125];

// input: from odom
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());

// ouput: all visualble cube points
pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());

// surround points in map to build tree
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>());

//input & output: points in one frame. local --> global
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());

// points in every cube
pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];
pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];

//kd-tree
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());

double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters);
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters + 4);

// wmap_T_odom * odom_T_curr = wmap_T_curr;
// transformation between odom's world and map's world frame
Eigen::Quaterniond q_wmap_wodom(1, 0, 0, 0);
Eigen::Vector3d t_wmap_wodom(0, 0, 0);

Eigen::Quaterniond q_wodom_curr(1, 0, 0, 0);
Eigen::Vector3d t_wodom_curr(0, 0, 0);


std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLastBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLastBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
std::mutex mBuf;

pcl::VoxelGrid<PointType> downSizeFilterCorner;
pcl::VoxelGrid<PointType> downSizeFilterSurf;

std::vector<int> pointSearchInd;
std::vector<float> pointSearchSqDis;

PointType pointOri, pointSel;

ros::Publisher pubLaserCloudSurround, pubLaserCloudMap, pubLaserCloudFullRes, pubOdomAftMapped, pubOdomAftMappedHighFrec, pubLaserAfterMappedPath, pub_ppk;

nav_msgs::Path laserAfterMappedPath;

//-----------------------------//D
bool VinsInited=false;
bool VO_LO_Comb=true;
bool GPS_Comb=true;

bool VO_fail=false;
bool GPS_fail=false;
	
double timeVinsCurrent;
double timeVinsBuffer;          
std::queue<nav_msgs::Odometry::ConstPtr> visualOdometryBuf;

Eigen::Vector3d t_world_vocurr;
Eigen::Quaterniond q_world_vocurr;   	
Eigen::Vector3d q_world_vocurr_vec;
Eigen::Vector3d q_wodom_vocurr_vec;       

Eigen::Vector3d t_wodom_locurr;
Eigen::Quaterniond q_wodom_locurr;
Eigen::Quaterniond q_world_locurr;   	   	
Eigen::Vector3d q_wodom_locurr_vec;
Eigen::Vector3d q_world_locurr_vec; 


Eigen::Quaterniond q_w_curr_vo(1, 0, 0, 0);
Eigen::Vector3d t_w_curr_vo(0, 0, 0);

Eigen::Quaterniond q_w_prev(1, 0, 0, 0);
Eigen::Vector3d t_w_prev(0, 0, 0);

Eigen::Quaterniond vins_w_q(1, 0, 0, 0);
Eigen::Vector3d vins_w_t(0, 0, 0);

Eigen::Quaterniond lidar_w_q(1, 0, 0, 0);
Eigen::Vector3d lidar_w_t(0, 0, 0);

Eigen::Quaterniond vins_w_q_prev(1, 0, 0, 0);
Eigen::Vector3d vins_w_t_prev(0, 0, 0);     

Eigen::Quaterniond vins_w_q_init(1, 0, 0, 0);
Eigen::Vector3d vins_w_t_init(0, 0, 0);  	

Eigen::Matrix4d VELO_T_CAM;
Eigen::Matrix4d CAM_T_VELO;
Eigen::Matrix4d CAM_T_IMU;
Eigen::Matrix4d IMU_T_CAM;
Eigen::Matrix4d VELO_T_IMU;
Eigen::Matrix4d VELO_T_GPS;
Eigen::Matrix4d VELO_T_GPSx;
Eigen::Matrix4d VELO_T_GPSy;
Eigen::Matrix4d VELO_T_GPSyy;
Eigen::Matrix4d VELO_T_GPSz;

Eigen::Quaterniond q_vo_ff;             	
Eigen::Vector3d t_vo_ff;
Eigen::Quaterniond q_lo_ff; 
Eigen::Vector3d t_lo_ff;

Eigen::Vector3d q_vo_ff_vec;
Eigen::Vector3d q_lo_ff_vec;    //!D


//D - variables for GPS integration
geodetic_converter::GeodeticConverter g_geodetic_converter;
std::queue<sensor_msgs::NavSatFixConstPtr> GPSbuff;
double latitude, longitude, altitude;
bool initGPS=false;
double x, y, z;
Eigen::Vector3d GPS_position;
Eigen::Vector3d GPS_corrected;
double GPS_time=0;
double GPS_accuracy;
#define PI 3.14159265
double angle=-130;
Eigen::Matrix4d WL_T_WGPS = Eigen::Matrix4d::Identity();
int GPScount = 0; 
int GPS_status=2;

// O - init for ppk call back
bool rtk_unreliable = true;  //TODO  : send to config file
bool use_ppk = true;  //TODO  : send to config file
std::string ppk_pos_file = "/storage_ssd/bell412Dataset1/bell412_dataset1_ppk.pos"; //TODO  : send to config file
std::string nmeaSentence = {};
std::string nmea_time;
boost::posix_time::time_duration nmea_time_pval;
boost::posix_time::time_duration ppk_time_pval;
boost::posix_time::ptime my_posix_time;
boost::gregorian::date my_posix_date;
boost::posix_time::time_duration my_time_of_day;
std::ifstream myfile1 (ppk_pos_file);
std::string line;
bool skip_read = false;
sensor_msgs::NavSatFix fix_ppk_msg;


// set initial guess
void transformAssociateToMap()
{
	q_w_curr = q_wmap_wodom * q_wodom_curr;
	t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
}

void transformUpdate()
{
	q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
	t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
}

void pointAssociateToMap(PointType const *const pi, PointType *const po)
{
	Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
	Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
	po->x = point_w.x();
	po->y = point_w.y();
	po->z = point_w.z();
	po->intensity = pi->intensity;
	//po->intensity = 1.0;

}

/*void VOtransformAssociateToMap()     //D - visual odometry update on the map
{
	q_w_curr_vo = q_w_prev * q_lo_ff;
	t_w_curr_vo = q_w_prev * t_lo_ff + t_w_prev;
}*/



void pointAssociateTobeMapped(PointType const *const pi, PointType *const po)
{
	Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
	Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
	po->x = point_curr.x();
	po->y = point_curr.y();
	po->z = point_curr.z();
	po->intensity = pi->intensity;
}


//Remove yaw from quarternion   //D
/*Eigen::Quaterniond removeYawfromQuat(Eigen::Quaterniond q)
{
   	Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);	

    	Eigen::Matrix3d R_w_c = lidar_w_q.toRotationMatrix();
	Eigen::Matrix3d R= VELO_T_IMU.block<3, 3>(0, 0).inverse()*R_w_c*VELO_T_IMU.block<3, 3>(0, 0);
	q_vo_ff = Eigen::Quaterniond(R);
	q_vo_ff = q_vo_ff.normalized();
   	
	Eigen::Vector3d euler_lo = q_vo_ff.toRotationMatrix().eulerAngles(2, 1, 0);
	
	euler[0] = 0.0;
	q = euler2Quaternion(euler[0] ,euler[1] ,euler[2] );

	return q;
}*/


void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudCornerLast2)
{
	mBuf.lock();
	cornerLastBuf.push(laserCloudCornerLast2);
	mBuf.unlock();
}

void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudSurfLast2)
{
	mBuf.lock();
	surfLastBuf.push(laserCloudSurfLast2);
	mBuf.unlock();
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
	mBuf.lock();
	fullResBuf.push(laserCloudFullRes2);
	mBuf.unlock();
}

//receive synced visual odometry
void VinsOdometryHandler(const nav_msgs::Odometry::ConstPtr &pose_msg)  //D
{
    	mBuf.lock();
    	visualOdometryBuf.push(pose_msg);
    	mBuf.unlock();

   	timeVinsCurrent = pose_msg->header.stamp.toSec(); 
   	//ROS_INFO("\n -------------Mapping Vins_Header_time =, %f", timeVinsCurrent); 


	// high frequence publish
	Eigen::Quaterniond q_wodom_curr;
	Eigen::Vector3d t_wodom_curr;
	q_wodom_curr.x() = pose_msg->pose.pose.orientation.x;
	q_wodom_curr.y() = pose_msg->pose.pose.orientation.y;
	q_wodom_curr.z() = pose_msg->pose.pose.orientation.z;
	q_wodom_curr.w() = pose_msg->pose.pose.orientation.w;
	t_wodom_curr.x() = pose_msg->pose.pose.position.x;
	t_wodom_curr.y() = pose_msg->pose.pose.position.y;
	t_wodom_curr.z() = pose_msg->pose.pose.position.z;
}


void GPSHandler(const sensor_msgs::NavSatFixConstPtr& msg)
{
  if(use_ppk) return;

  if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) 
  {
    	ROS_WARN_STREAM_THROTTLE(1, "No GPS fix");
    	return;
  }

  latitude=msg->latitude;
  longitude=msg->longitude;
  altitude=msg->altitude;
  
  if (!initGPS) 
  {
	g_geodetic_converter.initialiseReference(latitude, longitude, altitude);
	initGPS=true;
        ROS_INFO("\n ------------------------------------GPS Mapping------------------------------------------------"); 
	return;
  }

  else 
  {
    mBuf.lock();
    GPSbuff.push(msg);
    mBuf.unlock();
  }
}


//receive lidar odometry
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry)
{
	mBuf.lock();
	odometryBuf.push(laserOdometry);
	mBuf.unlock();

	// high frequence publish
	Eigen::Quaterniond q_wodom_curr;
	Eigen::Vector3d t_wodom_curr;
	q_wodom_curr.x() = laserOdometry->pose.pose.orientation.x;
	q_wodom_curr.y() = laserOdometry->pose.pose.orientation.y;
	q_wodom_curr.z() = laserOdometry->pose.pose.orientation.z;
	q_wodom_curr.w() = laserOdometry->pose.pose.orientation.w;
	t_wodom_curr.x() = laserOdometry->pose.pose.position.x;
	t_wodom_curr.y() = laserOdometry->pose.pose.position.y;
	t_wodom_curr.z() = laserOdometry->pose.pose.position.z;

	Eigen::Quaterniond q_w_curr = q_wmap_wodom * q_wodom_curr;
	Eigen::Vector3d t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom; 

	nav_msgs::Odometry odomAftMapped;
	odomAftMapped.header.frame_id = "/camera_init";
	odomAftMapped.child_frame_id = "/aft_mapped";
	odomAftMapped.header.stamp = laserOdometry->header.stamp;
	odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
	odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
	odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
	odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
	odomAftMapped.pose.pose.position.x = t_w_curr.x();
	odomAftMapped.pose.pose.position.y = t_w_curr.y();
	odomAftMapped.pose.pose.position.z = t_w_curr.z();
	pubOdomAftMappedHighFrec.publish(odomAftMapped);
	
	GTframeCount++;
	if(GTframeCount==2)
	{
		GTframeCount = 0;
		Eigen::Matrix3d R_curr = q_w_curr.toRotationMatrix();   

		std::ofstream foutC("/home/didula/GVLOAM/src/Results/VLOAM.txt", std::ios::app);             //Delete the previous file -ToDO --- check how to replace
	    	foutC.setf(std::ios::fixed, std::ios::floatfield);
	    	foutC.precision(0);
		//foutC << header.stamp.toSec() * 1e9 << ",";
				foutC.precision(6);
			    	foutC << R_curr(0,0) << " "
				    << R_curr(0,1) << " "
				    << R_curr(0,2) << " "
				    << t_w_curr.x() << " "
				    << R_curr(1,0) << " "
				    << R_curr(1,1) << " "
				    << R_curr(1,2) << " "
				    << t_w_curr.y() << " "
				    << R_curr(2,0) << " "
				    << R_curr(2,1) << " "
				    << R_curr(2,2) << " "
				    << t_w_curr.z()<< std::endl;
			    	foutC.close();
	}
}

void process()
{
	while(1)
	{

		//KITTI 
		/*CAM_T_VELO <<  7.027555e-03, -9.999753e-01, 2.599616e-05, -7.137748e-03,                    //ToDo: Read from config file
		  		-2.254837e-03, -4.184312e-05, -9.999975e-01, -7.482656e-02,
		   		9.999728e-01, 7.027479e-03, -2.255075e-03, -3.336324e-01,
		                0,             0,             0,             1; 

		VELO_T_IMU = CAM_T_VELO.inverse(); */                              //should be VELO_T_CAM but kept like this for payload implementation 

        	

		/*
		//LIV-SAM Dataset
		VELO_T_IMU <<  -1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, -1, 0,
              		0, 0, 0, 1;


		//Payload-Down
	
		VELO_T_IMU <<   0, 0, 1, 0,
				0, -1, 0, 0,
				1, 0, 0, 0,
              			0, 0, 0, 1;
                //Payload-Front
        	VELO_T_IMU <<   1, 0, 0, 0,
				0, -1, 0, 0,
				0, 0, -1, 0,
              			0, 0, 0, 1;*/
		/*VELO_T_IMU <<   0, 0, -1, 0,
				0, 1, 0, 0,
				1, 0, 0, 0,
              			0, 0, 0, 1;*/

		
		//D - read the calibration Lidar to IMU from config
		ParamServer Param;                       
        
		VELO_T_IMU=Param.VELO_T_IMU;

		VELO_T_GPSz<< cos(angle*PI/180), -sin(angle*PI/180), 0, 0,
			     sin(angle*PI/180), cos(angle*PI/180), 0, 0,
			     0, 0, 1, 0,
			     0, 0, 0, 1;
		VELO_T_GPSy<< cos(180*PI/180), 0, sin(180*PI/180), 0,
			     0, 1, 0, 0,	
			     -sin(180*PI/180), 0, cos(180*PI/180), 0,
			     0, 0, 0, 1;
		VELO_T_GPSyy<< cos(-90*PI/180), 0, sin(-90*PI/180), 0,
			     0, 1, 0, 0,	
			     -sin(-90*PI/180), 0, cos(-90*PI/180), 0,
			     0, 0, 0, 1;

		//LVI-SAM and Payload front
		//VELO_T_GPS=VELO_T_GPSz*VELO_T_GPSy;
	
		//Payload down
		VELO_T_GPS=VELO_T_GPSyy*VELO_T_GPSz;


		//std::cout <<"-------VELO_T_IMU lidar Mapping-------- "<<VELO_T_IMU<< std::endl;

        	/*VELO_T_GPS << cos(angle*PI/180), -sin(angle*PI/180), 0, 0,
		     sin(angle*PI/180), cos(angle*PI/180), 0, 0,
	             0, 0, 1, 0,
		     0, 0, 0, 1;*/

		/*if(timeLaserCloudFullRes>1642447415.3 && timeLaserCloudFullRes<1642447510.5)
		{		
		    	ROS_INFO("\n ----------------------Mapping LOAM only------------------------");	
			VO_LO_Comb=false;

		}
		else if(timeLaserCloudFullRes>1642447554.3 && timeLaserCloudFullRes<1642447571.5)
		{		
		    	ROS_INFO("\n ----------------------Mapping LOAM only------------------------");	
			VO_LO_Comb=false;

		}
		else if(timeLaserCloudFullRes>1642447595.3 && timeLaserCloudFullRes<1642447615.5)
		{
			ROS_INFO("\n ----------------------Mapping LOAM only------------------------");	
			VO_LO_Comb=false;
		}
		else
		{
			VO_LO_Comb=true;				
		}*/
                  
		while (!cornerLastBuf.empty() && !surfLastBuf.empty() && !fullResBuf.empty() && !odometryBuf.empty() && (!visualOdometryBuf.empty() || !VO_LO_Comb))
		{  	
			mBuf.lock();
			while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
				odometryBuf.pop();
			if (odometryBuf.empty())
			{
				mBuf.unlock();
				break;
			}

			while (!surfLastBuf.empty() && surfLastBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
				surfLastBuf.pop();
			if (surfLastBuf.empty())
			{
				mBuf.unlock();
				break;
			}

			while (!fullResBuf.empty() && fullResBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
				fullResBuf.pop();
			if (fullResBuf.empty())
			{
				mBuf.unlock();
				break;
			}
			
			if (VO_LO_Comb)
	    		{
				while (!visualOdometryBuf.empty() && visualOdometryBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())                 //D
					visualOdometryBuf.pop();
				if (visualOdometryBuf.empty())
				{
					mBuf.unlock();
					break;
				}
			}
			timeLaserCloudCornerLast = cornerLastBuf.front()->header.stamp.toSec();
			timeLaserCloudSurfLast = surfLastBuf.front()->header.stamp.toSec();
			timeLaserCloudFullRes = fullResBuf.front()->header.stamp.toSec();
			timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();
			//timeVinsBuffer=visualOdometryBuf.front()->header.stamp.toSec();                //D

			if (timeLaserCloudCornerLast != timeLaserOdometry ||
				timeLaserCloudSurfLast != timeLaserOdometry ||
				timeLaserCloudFullRes != timeLaserOdometry)
			{
				printf("time corner %f surf %f full %f odom %f \n", timeLaserCloudCornerLast, timeLaserCloudSurfLast, timeLaserCloudFullRes, timeLaserOdometry);
				printf("unsync message!");
				mBuf.unlock();
				break;
			}

			laserCloudCornerLast->clear();
			pcl::fromROSMsg(*cornerLastBuf.front(), *laserCloudCornerLast);
			cornerLastBuf.pop();

			laserCloudSurfLast->clear();
			pcl::fromROSMsg(*surfLastBuf.front(), *laserCloudSurfLast);
			surfLastBuf.pop();

			laserCloudFullRes->clear();
			pcl::fromROSMsg(*fullResBuf.front(), *laserCloudFullRes);
			fullResBuf.pop();


			//Correcting Lidar odometry 
			q_wodom_curr.x() = odometryBuf.front()->pose.pose.orientation.x;
			q_wodom_curr.y() = odometryBuf.front()->pose.pose.orientation.y;
			q_wodom_curr.z() = odometryBuf.front()->pose.pose.orientation.z;
			q_wodom_curr.w() = odometryBuf.front()->pose.pose.orientation.w;
			t_wodom_curr.x() = odometryBuf.front()->pose.pose.position.x;
			t_wodom_curr.y() = odometryBuf.front()->pose.pose.position.y;
			t_wodom_curr.z() = odometryBuf.front()->pose.pose.position.z;
			odometryBuf.pop();

			//std::cout <<"-------Lidar odometry translation "<<t_wodom_curr<< std::endl; 

			/*lidar_w_q.x() = odometryBuf.front()->pose.pose.orientation.x;
			lidar_w_q.y() = odometryBuf.front()->pose.pose.orientation.y;
			lidar_w_q.z() = odometryBuf.front()->pose.pose.orientation.z;
			lidar_w_q.w() = odometryBuf.front()->pose.pose.orientation.w;
			lidar_w_t.x() = odometryBuf.front()->pose.pose.position.x;
			lidar_w_t.y() = odometryBuf.front()->pose.pose.position.y;
			lidar_w_t.z() = odometryBuf.front()->pose.pose.position.z;
			odometryBuf.pop();*/


			/*if (!VinsInited)                                                                              //D - VIO initialization
			{

				VinsInited = true;		
		  		vins_w_t_prev.x() = visualOdometryBuf.front()->pose.pose.position.x;                   //First update from VIO stored as the initial previoud odometry
				vins_w_t_prev.y() = visualOdometryBuf.front()->pose.pose.position.y;                    
				vins_w_t_prev.z() = visualOdometryBuf.front()->pose.pose.position.z;
				vins_w_q_prev.w() = visualOdometryBuf.front()->pose.pose.orientation.w;
				vins_w_q_prev.x() = visualOdometryBuf.front()->pose.pose.orientation.x;
				vins_w_q_prev.y() = visualOdometryBuf.front()->pose.pose.orientation.y;
				vins_w_q_prev.z() = visualOdometryBuf.front()->pose.pose.orientation.z;
			}*/

	    		if (VO_LO_Comb)
	    		{
			//Visual Odometry 
			     	vins_w_t.x() = visualOdometryBuf.front()->pose.pose.position.x;
				vins_w_t.y() = visualOdometryBuf.front()->pose.pose.position.y;
				vins_w_t.z() = visualOdometryBuf.front()->pose.pose.position.z;
			    	vins_w_q.x() = visualOdometryBuf.front()->pose.pose.orientation.x;
				vins_w_q.y() = visualOdometryBuf.front()->pose.pose.orientation.y;
				vins_w_q.z() = visualOdometryBuf.front()->pose.pose.orientation.z;
			    	vins_w_q.w() = visualOdometryBuf.front()->pose.pose.orientation.w;
			}
			ROS_INFO("\n -------------Mapping Lidar_time =, %f", timeLaserOdometry);

			if(!GPSbuff.empty())
			{

			  for(int i = 0; i < (signed)GPSbuff.size(); ++i)
     			  {

			    	GPS_time=GPSbuff.front()->header.stamp.toSec();
				ROS_INFO("\n -------------GPS_time =, %f", GPS_time);
				if(fabs(GPS_time-timeLaserOdometry)<0.1	)
				{
					GPS_fail=false;		
					latitude=GPSbuff.front()->latitude;
					longitude=GPSbuff.front()->longitude;
					altitude=GPSbuff.front()->altitude;
					GPS_accuracy = GPSbuff.front()->position_covariance[0];
					GPS_status=GPSbuff.front()->status.status;

					g_geodetic_converter.geodetic2Enu(latitude, longitude, altitude, &x, &y, &z);
					GPS_position<<x,y,z; 
					std::cout <<"-------GPS_position----- "<<GPS_position<<"buffsize --- "<< GPSbuff.size() << "latlon:"<< latitude <<","<< longitude <<","<< altitude <<  std::endl; 
					//ROS_INFO("\n -------------GPS correction---------");
					GPSbuff.pop();
					break;
				}
	    			else if(GPS_time-timeLaserOdometry>0.1)
				{
					GPS_fail=true;
					break;		
				}
				else
				{	
					GPSbuff.pop();
					GPS_fail=true;		
				}
			  }
			}

			GPS_corrected=VELO_T_GPS.block<3, 3>(0, 0) * GPS_position + VELO_T_GPS.block<3, 1>(0, 3);
			std::cout <<"-------GPS_corrected----- "<<GPS_corrected<< std::endl; 

			/*std::cout <<"-------vins_w_q.x "<<vins_w_q.x()<< std::endl;        
			std::cout <<"-------vins_w_q.y "<<vins_w_q.y()<< std::endl; 
			std::cout <<"-------vins_w_q.z "<<vins_w_q.z()<< std::endl;					
			std::cout <<"-------vins_w_q.w "<<vins_w_q.w()<< std::endl;			

			vins_w_q_init= removeYawfromQuat(vins_w_q);

			std::cout <<"-------yaw removed vins_w_q.x "<<vins_w_q_init.x()<< std::endl;        
			std::cout <<"-------yaw removed vins_w_q.y "<<vins_w_q_init.y()<< std::endl; 
			std::cout <<"-------yaw removed vins_w_q.z "<<vins_w_q_init.z()<< std::endl;					
			std::cout <<"-------yaw removed vins_w_q.w "<<vins_w_q_init.w()<< std::endl;*/

			/*Eigen::Matrix3d R_vw_c = vins_w_q.toRotationMatrix();
			Eigen::Matrix3d R_L= VELO_T_IMU.block<3, 3>(0, 0)*R_vw_c*VELO_T_IMU.block<3, 3>(0, 0).inverse();
		    	t_wodom_curr = VELO_T_IMU.block<3, 3>(0, 0)*vins_w_t+VELO_T_IMU.block<3, 1>(0, 3);
			q_wodom_curr = Eigen::Quaterniond(R_L);*/

			/*t_world_vocurr=vins_w_q_init.inverse()*(vins_w_t-vins_w_t_init);                              //world - vo world frame and wodom - lidar world frame
			q_world_vocurr=vins_w_q_init.inverse()*vins_w_q;
			q_world_vocurr_vec<<q_world_vocurr.x(),q_world_vocurr.y(), q_world_vocurr.z();

				
			//Transforming to lidar frame
	    		t_wodom_curr = VELO_T_IMU.block<3, 3>(0, 0)*t_world_vocurr+VELO_T_IMU.block<3, 1>(0, 3);
			q_wodom_vocurr_vec=VELO_T_IMU.block<3, 3>(0, 0)*q_world_vocurr_vec;

		    	q_wodom_curr.x()=q_wodom_vocurr_vec[0];
			q_wodom_curr.y()=q_wodom_vocurr_vec[1];
			q_wodom_curr.z()=q_wodom_vocurr_vec[2];
		    	q_wodom_curr.w()=q_world_vocurr.w();*/

			
			//D - VINS restart failure case
			/*if ((vins_w_t - vins_w_t_prev).norm() > 10.0)
			{
				ROS_ERROR("---------- VINS big translation, restart estimator in lidar mapping -------!");
				VO_fail=true;
			}
			else
			{
				VO_fail=false;
			}*/


			while(!cornerLastBuf.empty())
			{
				cornerLastBuf.pop();
				printf("drop lidar frame in mapping for real time performance \n");
			}

			mBuf.unlock();

			TicToc t_whole;

			transformAssociateToMap();

			TicToc t_shift;
			int centerCubeI = int((t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth;
			int centerCubeJ = int((t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight;
			int centerCubeK = int((t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth;

			if (t_w_curr.x() + 25.0 < 0)
				centerCubeI--;
			if (t_w_curr.y() + 25.0 < 0)
				centerCubeJ--;
			if (t_w_curr.z() + 25.0 < 0)
				centerCubeK--;

			while (centerCubeI < 3)
			{
				for (int j = 0; j < laserCloudHeight; j++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{ 
						int i = laserCloudWidth - 1;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k]; 
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; i >= 1; i--)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeI++;
				laserCloudCenWidth++;
			}

			while (centerCubeI >= laserCloudWidth - 3)
			{ 
				for (int j = 0; j < laserCloudHeight; j++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{
						int i = 0;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; i < laserCloudWidth - 1; i++)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeI--;
				laserCloudCenWidth--;
			}

			while (centerCubeJ < 3)
			{
				for (int i = 0; i < laserCloudWidth; i++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{
						int j = laserCloudHeight - 1;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; j >= 1; j--)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeJ++;
				laserCloudCenHeight++;
			}

			while (centerCubeJ >= laserCloudHeight - 3)
			{
				for (int i = 0; i < laserCloudWidth; i++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{
						int j = 0;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; j < laserCloudHeight - 1; j++)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeJ--;
				laserCloudCenHeight--;
			}

			while (centerCubeK < 3)
			{
				for (int i = 0; i < laserCloudWidth; i++)
				{
					for (int j = 0; j < laserCloudHeight; j++)
					{
						int k = laserCloudDepth - 1;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; k >= 1; k--)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeK++;
				laserCloudCenDepth++;
			}

			while (centerCubeK >= laserCloudDepth - 3)
			{
				for (int i = 0; i < laserCloudWidth; i++)
				{
					for (int j = 0; j < laserCloudHeight; j++)
					{
						int k = 0;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; k < laserCloudDepth - 1; k++)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeK--;
				laserCloudCenDepth--;
			}

			int laserCloudValidNum = 0;
			int laserCloudSurroundNum = 0;

			for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
			{
				for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
				{
					for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
					{
						if (i >= 0 && i < laserCloudWidth &&
							j >= 0 && j < laserCloudHeight &&
							k >= 0 && k < laserCloudDepth)
						{ 
							laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
							laserCloudValidNum++;
							laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
							laserCloudSurroundNum++;
						}
					}
				}
			}

			laserCloudCornerFromMap->clear();
			laserCloudSurfFromMap->clear();
			for (int i = 0; i < laserCloudValidNum; i++)
			{
				*laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
				*laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
			}
			int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
			int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();


			pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
			downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
			downSizeFilterCorner.filter(*laserCloudCornerStack);
			int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

			pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
			downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
			downSizeFilterSurf.filter(*laserCloudSurfStack);
			int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

			////printf("map prepare time %f ms\n", t_shift.toc());
			////printf("map corner num %d  surf num %d \n", laserCloudCornerFromMapNum, laserCloudSurfFromMapNum);
				TicToc t_opt;
				TicToc t_tree;
				kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
				kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);
				////printf("build tree time %f ms \n", t_tree.toc());

				for (int iterCount = 0; iterCount < 2; iterCount++)
				{
					//ceres::LossFunction *loss_function = NULL;
					ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
					ceres::LocalParameterization *q_parameterization =
						new ceres::EigenQuaternionParameterization();
					ceres::Problem::Options problem_options;

					ceres::Problem problem(problem_options);
					problem.AddParameterBlock(parameters, 4, q_parameterization);
					problem.AddParameterBlock(parameters + 4, 3);

					TicToc t_data;
					int corner_num = 0;
					int surf_num = 0;

					if (VO_LO_Comb && !VO_fail)
					{
						/*Eigen::Matrix3d R_w_p = vins_w_q_init.toRotationMatrix();
						Eigen::Matrix3d R_w_c = vins_w_q.toRotationMatrix();
						Eigen::Matrix3d R_p_c = R_w_p.inverse()*R_w_c;

						Eigen::Matrix3d Rff_L= VELO_T_IMU.block<3, 3>(0, 0)*R_p_c;	
						q_lo_ff = Eigen::Quaterniond(Rff_L);
						q_lo_ff = q_lo_ff.normalized();  */


						Eigen::Matrix3d R_w_c = vins_w_q.toRotationMatrix();
						Eigen::Matrix3d R= VELO_T_IMU.block<3, 3>(0, 0)*R_w_c*VELO_T_IMU.block<3, 3>(0, 0).inverse();
					    	t_lo_ff = VELO_T_IMU.block<3, 3>(0, 0)*vins_w_t+VELO_T_IMU.block<3, 1>(0, 3);
						q_lo_ff = Eigen::Quaterniond(R);
						q_lo_ff = q_lo_ff.normalized(); 

						/*std::cout <<"-------t_lo_ff.x "<<t_lo_ff.x()<< std::endl;        
						std::cout <<"-------t_lo_ff.y "<<t_lo_ff.y()<< std::endl; 
						std::cout <<"-------t_lo_ff.z "<<t_lo_ff.z()<< std::endl;		
	
						std::cout <<"-------q_lo_ff.x "<<q_lo_ff.x()<< std::endl;        
						std::cout <<"-------q_lo_ff.y "<<q_lo_ff.y()<< std::endl; 
						std::cout <<"-------q_lo_ff.z "<<q_lo_ff.z()<< std::endl;					
						std::cout <<"-------q_lo_ff.w "<<q_lo_ff.w()<< std::endl;*/

						//ceres::CostFunction* vo_function = GlobalRotationError::Create(q_vo_ff.w(), q_lo_ff_vec[0], q_lo_ff_vec[1], q_lo_ff_vec[2], 0.001);
						ceres::CostFunction* vo_function = GlobalRotationError::Create(q_lo_ff.w(), q_lo_ff.x(), q_lo_ff.y(), q_lo_ff.z(), 0.001);
						problem.AddResidualBlock(vo_function, NULL, parameters);

					}


					if (GPS_Comb && !GPS_fail && (GPS_status<2 || use_ppk))
					{

						ceres::CostFunction* gps_function = TError::Create(GPS_corrected.x(), -GPS_corrected.y(), GPS_corrected.z(), 0.01);
						problem.AddResidualBlock(gps_function, loss_function, parameters+4);
						ROS_INFO("\n -------------GPS correction---------");

					}



					/*if (VO_LO_Comb)
					{
						Eigen::Matrix3d R_w_p = vins_w_q_prev.toRotationMatrix();
						Eigen::Matrix3d R_w_c = vins_w_q.toRotationMatrix();
						Eigen::Matrix3d R_w_L= VELO_T_IMU.block<3, 3>(0, 0)*R_w_c;
						q_lo_ff = Eigen::Quaterniond(R_w_L);
						q_lo_ff = q_lo_ff.normalized();
	
						t_vo_ff=R_w_p.inverse()*(vins_w_t-vins_w_t_prev);
					    	t_lo_ff = VELO_T_IMU.block<3, 3>(0, 0)*t_vo_ff+VELO_T_IMU.block<3, 1>(0, 3);  

						//std::cout <<"-------R_w_p "<<R_w_p<< std::endl;   

						VOtransformAssociateToMap();

						ceres::CostFunction* vo_function = RelativeRTError::Create(t_lo_ff.x(), t_lo_ff.y(), t_lo_ff.z(),
											                                q_lo_ff.w(), q_lo_ff.x(), q_lo_ff.y(), q_lo_ff.z(),
											                                10, 0.01);


						problem.AddResidualBlock(vo_function, NULL, parameters, parameters + 4);

					}*/




					/*if (VO_LO_Comb)
					{

						//ROS_INFO("\n -----------------------------------VO Pose constraint added --------------------------------------- ");
						//ROS_INFO("\n -------------mapping lidar frame time --------------%f", timeLaserCloudFullRes); 	
						//ROS_INFO("\n -------------mapping visual frame time --------------%f", timeVinsBuffer); 	
						

						//Eigen::Matrix3d R_w_p;
						//Eigen::Matrix3d R_w_c;


						R_w_p <<  0, -1, 0,
							  1, 0, 0,
							  0, 0, 1;

						R_w_c <<  -1, 0, 0,
							  0, -1, 0,
							  0, 0, 1;

						vins_w_t_prev <<  1, 0, 2;
						vins_w_t <<  2, 1, 4;

						Eigen::Matrix3d R_w_p = vins_w_q_prev.toRotationMatrix();
						Eigen::Matrix3d R_w_c = vins_w_q.toRotationMatrix();
						Eigen::Matrix3d R_p_c = R_w_p.inverse()*R_w_c;

						Eigen::Matrix3d Rff_L= VELO_T_IMU.block<3, 3>(0, 0)*R_p_c*VELO_T_IMU.block<3, 3>(0, 0).inverse();

						t_vo_ff=R_w_p.inverse()*(vins_w_t-vins_w_t_prev);
						q_lo_ff = Eigen::Quaterniond(Rff_L);
						q_lo_ff = q_lo_ff.normalized();  
					    	t_lo_ff = VELO_T_IMU.block<3, 3>(0, 0)*t_vo_ff+VELO_T_IMU.block<3, 1>(0, 3);  

						//std::cout <<"-------R_w_p "<<R_w_p<< std::endl;   

						VOtransformAssociateToMap();

						ceres::CostFunction* vo_function = RelativeRTError::Create(t_w_curr_vo.x(), t_w_curr_vo.y(), t_w_curr_vo.z(),
											                                q_w_curr_vo.w(), q_w_curr_vo.x(), q_w_curr_vo.y(), q_w_curr_vo.z(),
											                                0.1, 0.01);


						problem.AddResidualBlock(vo_function, NULL, parameters, parameters + 4);

					}*/



					/*if (VO_LO_Comb)
					{

						//ROS_INFO("\n -----------------------------------VO Pose constraint added --------------------------------------- ");
						//ROS_INFO("\n -------------mapping lidar frame time --------------%f", timeLaserCloudFullRes); 	
						//ROS_INFO("\n -------------mapping visual frame time --------------%f", timeVinsBuffer); 	

                      

						t_vo_ff=vins_w_q_prev.inverse()*(vins_w_t-vins_w_t_prev);
						q_vo_ff=vins_w_q_prev.inverse()*vins_w_q;
						q_vo_ff = q_vo_ff.normalized();
						q_vo_ff_vec<<q_vo_ff.x(),q_vo_ff.y(), q_vo_ff.z();


						//Transforming to lidar frame
						q_lo_ff_vec=VELO_T_IMU.block<3, 3>(0, 0)*q_vo_ff_vec;
						q_lo_ff.x()= q_lo_ff_vec[0];
						q_lo_ff.y()= q_lo_ff_vec[1];
						q_lo_ff.z()= q_lo_ff_vec[2];
						q_lo_ff.w()= q_vo_ff.w();
					    	t_lo_ff = VELO_T_IMU.block<3, 3>(0, 0)*t_vo_ff+VELO_T_IMU.block<3, 1>(0, 3);


						VOtransformAssociateToMap();

						std::cout <<"-------t_w_curr_vo.x "<<t_w_curr_vo.x()<< std::endl;        
			    			std::cout <<"-------t_w_curr_vo.y "<<t_w_curr_vo.y()<< std::endl; 
			    			std::cout <<"-------t_w_curr_vo.z "<<t_w_curr_vo.z()<< std::endl;		
					
						std::cout <<"-------q_w_curr_vo.x "<<q_w_curr_vo.x()<< std::endl;        
			    			std::cout <<"-------q_w_curr_vo.y "<<q_w_curr_vo.y()<< std::endl; 
			    			std::cout <<"-------q_w_curr_vo.z "<<q_w_curr_vo.z()<< std::endl;					
			    			std::cout <<"-------q_w_curr_vo.w "<<q_w_curr_vo.w()<< std::endl;



						ceres::CostFunction* vo_function = RelativeRTError::Create(t_w_curr_vo.x(), t_w_curr_vo.y(), t_w_curr_vo.z(),
											                                q_w_curr_vo.w(), q_w_curr_vo.x(), q_w_curr_vo.y(), q_w_curr_vo.z(),
											                                0.1, 0.01);


						problem.AddResidualBlock(vo_function, NULL, parameters, parameters + 4);

					}*/

			        //std::cout <<"-------point association translation "<<t_w_curr<< std::endl; 
				if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50 && laserCloudCornerStackNum>50 && laserCloudSurfStackNum>50)
				{
					
					for (int i = 0; i < laserCloudCornerStackNum; i++)
					{
						pointOri = laserCloudCornerStack->points[i];
						//double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
						pointAssociateToMap(&pointOri, &pointSel);
						kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis); 

						if (pointSearchSqDis[4] < 1.0)
						{ 
							std::vector<Eigen::Vector3d> nearCorners;
							Eigen::Vector3d center(0, 0, 0);
							for (int j = 0; j < 5; j++)
							{
								Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
													laserCloudCornerFromMap->points[pointSearchInd[j]].y,
													laserCloudCornerFromMap->points[pointSearchInd[j]].z);
								center = center + tmp;
								nearCorners.push_back(tmp);
							}
							center = center / 5.0;

							Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
							for (int j = 0; j < 5; j++)
							{
								Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
								covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
							}

							Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

							// if is indeed line feature
							// note Eigen library sort eigenvalues in increasing order
							Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
							Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
							if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
							{ 
								Eigen::Vector3d point_on_line = center;
								Eigen::Vector3d point_a, point_b;
								point_a = 0.1 * unit_direction + point_on_line;
								point_b = -0.1 * unit_direction + point_on_line;

								ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
								problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
								corner_num++;	
							}							
						}
						/*
						else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
						{
							Eigen::Vector3d center(0, 0, 0);
							for (int j = 0; j < 5; j++)
							{
								Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
													laserCloudCornerFromMap->points[pointSearchInd[j]].y,
													laserCloudCornerFromMap->points[pointSearchInd[j]].z);
								center = center + tmp;
							}
							center = center / 5.0;	
							Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
							ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
							problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
						}
						*/
					}

					for (int i = 0; i < laserCloudSurfStackNum; i++)
					{
						pointOri = laserCloudSurfStack->points[i];
						//double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
						pointAssociateToMap(&pointOri, &pointSel);
						kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

						Eigen::Matrix<double, 5, 3> matA0;
						Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
						if (pointSearchSqDis[4] < 1.0)
						{
							
							for (int j = 0; j < 5; j++)
							{
								matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
								matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
								matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
								//printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
							}
							// find the norm of plane
							Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
							double negative_OA_dot_norm = 1 / norm.norm();
							norm.normalize();

							// Here n(pa, pb, pc) is unit norm of plane
							bool planeValid = true;
							for (int j = 0; j < 5; j++)
							{
								// if OX * n > 0.2, then plane is not fit well
								if (fabs(norm(0) * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
										 norm(1) * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
										 norm(2) * laserCloudSurfFromMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
								{
									planeValid = false;
									break;
								}
							}
							Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
							if (planeValid)
							{
								ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
								problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
								surf_num++;
							}
						}

						/*
						else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
						{
							Eigen::Vector3d center(0, 0, 0);
							for (int j = 0; j < 5; j++)
							{
								Eigen::Vector3d tmp(laserCloudSurfFromMap->points[pointSearchInd[j]].x,
													laserCloudSurfFromMap->points[pointSearchInd[j]].y,
													laserCloudSurfFromMap->points[pointSearchInd[j]].z);
								center = center + tmp;
							}
							center = center / 5.0;	
							Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
							ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
							problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
						}
						*/
					}
				}
				else
				{
					ROS_WARN("Not enough corner and surf features");
				}
					////printf("corner num %d surface num %d \n", laserCloudCornerStackNum, laserCloudSurfStackNum);
					printf("corner num %d used corner num %d \n", laserCloudCornerStackNum, corner_num);
					printf("surf num %d used surf num %d \n", laserCloudSurfStackNum, surf_num);

					////printf("mapping data assosiation time %f ms \n", t_data.toc());

					TicToc t_solver;
					ceres::Solver::Options options;
					options.linear_solver_type = ceres::DENSE_QR;
					options.max_num_iterations = 5;
					options.minimizer_progress_to_stdout = false;
					options.check_gradients = false;
					options.gradient_check_relative_precision = 1e-4;
					ceres::Solver::Summary summary;
					ceres::Solve(options, &problem, &summary);
					////printf("mapping solver time %f ms \n", t_solver.toc());

     			                //std::cout << summary.FullReport() << "\n";   //D - Solver details

					//printf("time %f \n", timeLaserOdometry);
					//printf("corner factor num %d surf factor num %d\n", corner_num, surf_num);
					//printf("result q %f %f %f %f result t %f %f %f\n", parameters[3], parameters[0], parameters[1], parameters[2],
					//	   parameters[4], parameters[5], parameters[6]);
				}
				////printf("mapping optimization time %f \n", t_opt.toc());


				/*if(GPScount<2000)
				{
					Eigen::Matrix4d WL_T_body = Eigen::Matrix4d::Identity(); 
            				Eigen::Matrix4d WGPS_T_body = Eigen::Matrix4d::Identity();

		    	    		WL_T_body.block<3, 3>(0, 0) = q_w_curr.toRotationMatrix();
		    	    		WL_T_body.block<3, 1>(0, 3) = t_w_curr;
		    	    		//WGPS_T_body.block<3, 3>(0, 0) = q_w_curr.toRotationMatrix();
		    	    		WGPS_T_body.block<3, 1>(0, 3) = GPS_position;
		    	    		WL_T_WGPS = WL_T_body * WGPS_T_body.inverse();
					GPScount++;
				}

				std::cout <<"-------Lidar_position----- "<<t_w_curr<< std::endl; */
				//std::cout <<"-------Lidar mapping translation "<<t_w_curr<< std::endl; 

			  	t_w_prev.x() = t_w_curr.x();                   
				t_w_prev.y() = t_w_curr.y();                    
				t_w_prev.z() = t_w_curr.z();
				q_w_prev.w() = q_w_curr.w();
				q_w_prev.x() = q_w_curr.x();
				q_w_prev.y() = q_w_curr.y();
				q_w_prev.z() = q_w_curr.z();
				
				if (VO_LO_Comb)
	    			{
				  	vins_w_t_prev.x() = visualOdometryBuf.front()->pose.pose.position.x;                   
					vins_w_t_prev.y() = visualOdometryBuf.front()->pose.pose.position.y;                    
					vins_w_t_prev.z() = visualOdometryBuf.front()->pose.pose.position.z;
					vins_w_q_prev.w() = visualOdometryBuf.front()->pose.pose.orientation.w;
					vins_w_q_prev.x() = visualOdometryBuf.front()->pose.pose.orientation.x;
					vins_w_q_prev.y() = visualOdometryBuf.front()->pose.pose.orientation.y;
					vins_w_q_prev.z() = visualOdometryBuf.front()->pose.pose.orientation.z;
					visualOdometryBuf.pop();
				}

			transformUpdate();

			TicToc t_add;
			for (int i = 0; i < laserCloudCornerStackNum; i++)
			{
				pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);

				int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
				int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
				int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

				if (pointSel.x + 25.0 < 0)
					cubeI--;
				if (pointSel.y + 25.0 < 0)
					cubeJ--;
				if (pointSel.z + 25.0 < 0)
					cubeK--;

				if (cubeI >= 0 && cubeI < laserCloudWidth &&
					cubeJ >= 0 && cubeJ < laserCloudHeight &&
					cubeK >= 0 && cubeK < laserCloudDepth)
				{
					int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
					laserCloudCornerArray[cubeInd]->push_back(pointSel);
				}
			}

			for (int i = 0; i < laserCloudSurfStackNum; i++)
			{
				pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

				int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
				int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
				int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

				if (pointSel.x + 25.0 < 0)
					cubeI--;
				if (pointSel.y + 25.0 < 0)
					cubeJ--;
				if (pointSel.z + 25.0 < 0)
					cubeK--;

				if (cubeI >= 0 && cubeI < laserCloudWidth &&
					cubeJ >= 0 && cubeJ < laserCloudHeight &&
					cubeK >= 0 && cubeK < laserCloudDepth)
				{
					int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
					laserCloudSurfArray[cubeInd]->push_back(pointSel);
				}
			}
			////printf("add points time %f ms\n", t_add.toc());

			
			TicToc t_filter;
			for (int i = 0; i < laserCloudValidNum; i++)
			{
				int ind = laserCloudValidInd[i];

				pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
				downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
				downSizeFilterCorner.filter(*tmpCorner);
				laserCloudCornerArray[ind] = tmpCorner;

				pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
				downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
				downSizeFilterSurf.filter(*tmpSurf);
				laserCloudSurfArray[ind] = tmpSurf;
			}
			////printf("filter time %f ms \n", t_filter.toc());
			
			TicToc t_pub;
			//publish surround map for every 5 frame
			if (frameCount % 5 == 0)
			{
				laserCloudSurround->clear();
				for (int i = 0; i < laserCloudSurroundNum; i++)
				{
					int ind = laserCloudSurroundInd[i];
					*laserCloudSurround += *laserCloudCornerArray[ind];
					*laserCloudSurround += *laserCloudSurfArray[ind];
				}

				sensor_msgs::PointCloud2 laserCloudSurround3;
				pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
				laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				laserCloudSurround3.header.frame_id = "/odom";
				pubLaserCloudSurround.publish(laserCloudSurround3);
			}

			if (frameCount % 20 == 0)
			{
				pcl::PointCloud<PointType> laserCloudMap;
				for (int i = 0; i < 4851; i++)
				{
					laserCloudMap += *laserCloudCornerArray[i];
					laserCloudMap += *laserCloudSurfArray[i];
				}
				sensor_msgs::PointCloud2 laserCloudMsg;
				pcl::toROSMsg(laserCloudMap, laserCloudMsg);
				laserCloudMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				laserCloudMsg.header.frame_id = "/odom";
				// pubLaserCloudMap.publish(laserCloudMsg); //rav
			}

			int laserCloudFullResNum = laserCloudFullRes->points.size();
			for (int i = 0; i < laserCloudFullResNum; i++)
			{
				pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
			}

			sensor_msgs::PointCloud2 laserCloudFullRes3;
			pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
			laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
			laserCloudFullRes3.header.frame_id = "/odom";
			pubLaserCloudFullRes.publish(laserCloudFullRes3); 

			////printf("mapping pub time %f ms \n", t_pub.toc());

			printf("whole mapping time %f ms +++++\n", t_whole.toc());

			nav_msgs::Odometry odomAftMapped;
			odomAftMapped.header.frame_id = "/odom";
			odomAftMapped.child_frame_id = "/aft_mapped";
			odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
			odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
			odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
			odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
			odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
			odomAftMapped.pose.pose.position.x = t_w_curr.x();
			odomAftMapped.pose.pose.position.y = t_w_curr.y();
			odomAftMapped.pose.pose.position.z = t_w_curr.z();
			pubOdomAftMapped.publish(odomAftMapped);

			geometry_msgs::PoseStamped laserAfterMappedPose;
			laserAfterMappedPose.header = odomAftMapped.header;
			laserAfterMappedPose.pose = odomAftMapped.pose.pose;
			laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
			laserAfterMappedPath.header.frame_id = "/odom";
			laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
			pubLaserAfterMappedPath.publish(laserAfterMappedPath);

			static tf::TransformBroadcaster br;
			tf::Transform transform;
			tf::Quaternion q;
			transform.setOrigin(tf::Vector3(t_w_curr(0),
											t_w_curr(1),
											t_w_curr(2)));
			q.setW(q_w_curr.w());
			q.setX(q_w_curr.x());
			q.setY(q_w_curr.y());
			q.setZ(q_w_curr.z());
			transform.setRotation(q);
			//br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "/camera_init", "/aft_mapped"));
			br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "/odom", "/aft_mapped"));
                        static tf::Transform t_odom_odom_inv = tf::Transform(tf::createQuaternionFromRPY(0, 3.141, 0), tf::Vector3(0, 0, 0));
			br.sendTransform(tf::StampedTransform(t_odom_odom_inv, odomAftMapped.header.stamp, "odom", "odom_invert"));
			frameCount++;

		}
		std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
	}
}

float GpsToDecimalDegrees(const char* nmeaPos, char quadrant)
{
  float v= 0;
  if(strlen(nmeaPos)>5)
  {
    char integerPart[3+1];
    int digitCount= (nmeaPos[4]=='.' ? 2 : 3);
    memcpy(integerPart, nmeaPos, digitCount);
    integerPart[digitCount]= 0;
    nmeaPos+= digitCount;
    v= atoi(integerPart) + atof(nmeaPos)/60.;
    if(quadrant=='W' || quadrant=='S')
      v= -v;
  }
  return v;
}

ros::Time findRosTimeFromNmea(ros::Time stamp, std::string substr){
				my_posix_date   =    stamp.toBoost().date();
				//my_posix_date   =    boost::gregorian::date(2022,1,16);
	  			my_time_of_day  =    boost::posix_time::hours(std::stod(substr.substr(0,2))) +
						     boost::posix_time::minutes(std::stod(substr.substr(2,2))) + 
						     boost::posix_time::seconds(std::stod(substr.substr(4,2))) +
						     boost::posix_time::millisec(std::stod(substr.substr(7,2))*10); 
				
				my_posix_time   =    boost::posix_time::ptime(my_posix_date, my_time_of_day);
				
				

				//UTC 00:00 handling
				// get the difference in calculated clock and header clock
				boost::posix_time::time_duration time_diff = my_posix_time - stamp.toBoost();
				//ROS_INFO_STREAM("diff: "<< time_diff ) ; //ok
				//if the clock is off more than 24 hrs throw an error -this is not currently handled
				if (time_diff >= boost::posix_time::hours(24) || time_diff <= -boost::posix_time::hours(24)){
					ROS_INFO_STREAM("Clocks are off more than 24 Hrs (not handled in code) :"<< time_diff) ; //ok
					ROS_ERROR("Update code");
					ros::shutdown();
				}
				// if it is larger than 12hrs substract a day
				else if (time_diff >= boost::posix_time::hours(12)){
					my_posix_time = my_posix_time - boost::posix_time::hours(24);
					ROS_INFO_STREAM("UTC 00:00 corrected :"<< my_posix_time ) ; //ok
				}
				else if (time_diff <= -boost::posix_time::hours(12)){
					my_posix_time = my_posix_time + boost::posix_time::hours(24);
					ROS_INFO_STREAM("UTC 00:00 corrected :"<< my_posix_time ) ; //ok
					// if it is lees than -12hrs add a day
				}
			
				//perform clock correction
				return ros::Time::fromBoost(my_posix_time);

}

void nmeaCallback(const nmea_msgs::Sentence::ConstPtr& msg)
{  
    nmeaSentence = msg->sentence;
    
    //cout << nmeaSentence << endl;
    std::istringstream ss5(nmeaSentence);
    string token;
    std::getline(ss5,token, ','); string nmea_type = token;
    if (nmea_type.compare("$GNGGA") != 0)
	return;
    std::getline(ss5,token, ','); nmea_time = token;
    std::getline(ss5,token, ','); string nmea_lat_str = token; 
    std::getline(ss5,token, ','); double nmea_lat = GpsToDecimalDegrees(nmea_lat_str.c_str(), token.at(0));
    std::getline(ss5,token, ','); string nmea_lon_str = token; 
    std::getline(ss5,token, ','); double nmea_lon = GpsToDecimalDegrees(nmea_lon_str.c_str(), token.at(0));
    std::getline(ss5,token, ','); int nmea_fix = stoi(token);
    std::getline(ss5,token, ',');
    std::getline(ss5,token, ',');
    std::getline(ss5,token, ','); double nmea_alt = stod(token);


    std::stringstream nmea_time_concat;
    nmea_time_concat << nmea_time.substr(0,2) << ":"  << nmea_time.substr(2,2) << ":" << nmea_time.substr(4,5) << "0";
    std::string nmea_time_formatted_to_string = nmea_time_concat.str();

    
  
    //cout << "nmea received time:" << nmea_time << endl; //k
    
    //define the variables 
    std::string tempppk,templat,templon,tempalt,fix_ppk_time;
    //open file
    if (myfile1.is_open())
    {
     	
 	bool ppk_synced = false;
	while (!ppk_synced)
	{
		//0. read the next line
		if (!skip_read){
		std::getline(myfile1, line);} 
		
    		//1. remove the header info in pos file
		if(line.at(0) == '%') continue;
		//cout<<line<<endl;
 
                //2. extract the position message line
                //2022/05/25 20:43:10.200   45.325064906  -75.664896317    86.7785   1  28   0.0035   0.0025   0.0064  -0.0005  -0.0015  -0.0021   0.01    7.9
		std::string tempdate,temptime,templat,templon,tempalt,tempQ,tempnumsat,tempsdn,tempsde,tempsdu,tempsdne,tempsdeu,tempsdun,tempage,tempratio;
                std::istringstream iss1(line);
    		iss1 >> tempdate >> temptime >> templat >> templon >> tempalt >> tempQ >> tempnumsat >> tempsdn >> tempsde >> tempsdu >> tempsdne >> tempsdeu >> tempsdun >> tempage >> tempratio;
		
		//cout<<"ppk time string:"<<temptime<< " Nmea time: "<<nmea_time_formatted_to_string <<endl;//k

		// TODO:quick check of the UTC date to warn possible errors
		// convert ros time of nmea stamp to UTC and and check with PPK time
		
		// 3.1 Convert the two times to seconds
		//time value conversions
  		nmea_time_pval  =    boost::posix_time::hours(std::stod(nmea_time_formatted_to_string.substr(0,2))) +
				     boost::posix_time::minutes(std::stod(nmea_time_formatted_to_string.substr(3,2))) + 
			             boost::posix_time::seconds(std::stod(nmea_time_formatted_to_string.substr(6,2))) +
		                     boost::posix_time::millisec(std::stod(nmea_time_formatted_to_string.substr(9,2))*10);

		ppk_time_pval  =     boost::posix_time::hours(std::stod(temptime.substr(0,2))) +
				     boost::posix_time::minutes(std::stod(temptime.substr(3,2))) + 
			             boost::posix_time::seconds(std::stod(temptime.substr(6,2))-18) + //TODO:GPST to UTC correction
		                     boost::posix_time::millisec(std::stod(temptime.substr(9,2))*10);
		boost::posix_time::time_duration delta = ppk_time_pval-nmea_time_pval;
		double diff = delta.total_microseconds();
		//cout<<"ppk time val:"<< ppk_time_pval << " Nmea time val: "<< nmea_time_pval << "|" << diff << endl;//k

		//3.keep reading the file if the nmea time is ahead
                if (diff<0){
		     skip_read = false;
		     ppk_synced = false;
                     //cout<<"NMEA ahead:"<< ppk_time_pval << " Nmea time val: "<< nmea_time_pval << "|" << diff << endl;
		     continue;
		}

		//4.do not read the next line if the ppk time is ahead
		if (diff>0){
		     skip_read = true;
		     ppk_synced = true; 
		     continue;
		}

                //5.if they match input the GPS data to the optimizer
                if (diff==0){
		     skip_read = false;
		     ppk_synced = true; 


                     ros::Time nmea_ros_time=findRosTimeFromNmea(msg->header.stamp,nmea_time);
		     //cout<<"** SYNCED: ppk time val:"<< ppk_time_pval << " Nmea time val: "<< nmea_time_pval << "|" << diff << "|" << nmea_lat << "|" << nmea_lon << "|" << nmea_alt << "|" <<  stof(templat) << "|" <<  stof(templon) << "|" <<  stof(tempalt) << endl;
		     //cout << nmea_ros_time << "|" << msg->header.stamp << endl;
  		     // Create a ppk GPS message
	             fix_ppk_msg.header = msg->header;
		     fix_ppk_msg.status.status = 2;
		     fix_ppk_msg.status.service = 1;
	             fix_ppk_msg.latitude = stof(templat); // deg
		     fix_ppk_msg.longitude = stof(templon); // deg
		     fix_ppk_msg.altitude = stof(tempalt); // m
		     float sdn = std::stof(tempsdn)*10; // ppk covariances are too tight
	    	     float sde = std::stof(tempsde)*10;
	             float sdu = std::stof(tempsdu)*10;
                     float sdne = std::stof(tempsdne)*10;
	    	     float sdeu = std::stof(tempsdeu)*10;
	             float sdun = std::stof(tempsdun)*10;
		     fix_ppk_msg.position_covariance = {sdn, sdne, sdun, sdne, sde, sdeu, sdun, sdeu, sdu};
		     fix_ppk_msg.position_covariance_type = 3;

		     

                     // append to the ppk path and publish
                     // visualization only
                     //globalEstimator.inputPPKviz(fix_ppk_msg.header.stamp.toSec(), stof(templat), stof(templon), stof(tempalt), sdn);
                     //if(nmea_fix==5){   //5 is RTK
                     //globalEstimator.inputPPKviz(fix_ppk_msg.header.stamp.toSec(), nmea_lat, nmea_lon, nmea_alt, sdn);}
		     pub_ppk.publish(fix_ppk_msg);		     

                     if(use_ppk) {


		    if (!initGPS) 
  			{
				g_geodetic_converter.initialiseReference(latitude, longitude, altitude);
				initGPS=true;
        			ROS_INFO("\n ------------------------------------GPS                Mapping------------------------------------------------"); 
				return;
  			}
			// include in the GPS q
                     sensor_msgs::NavSatFixConstPtr fix_ppk_msg_const_pointer( new sensor_msgs::NavSatFix(fix_ppk_msg) );
                     mBuf.lock();
    		     GPSbuff.push(fix_ppk_msg_const_pointer);
                     mBuf.unlock();
		     cout << "GPS buffer size:" << GPSbuff.size() << endl;

		     }	
		} 
		 		
	}
    }
    else std::cout << "Unable to open ppk position file";
    cout<<"---------------------------"<<endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laserMapping");
	ros::NodeHandle nh;

	float lineRes = 0;
	float planeRes = 0;
	nh.param<float>("mapping_line_resolution", lineRes, 0.4);
	nh.param<float>("mapping_plane_resolution", planeRes, 0.8);
	printf("line resolution %f plane resolution %f \n", lineRes, planeRes);
	downSizeFilterCorner.setLeafSize(lineRes, lineRes,lineRes);
	downSizeFilterSurf.setLeafSize(planeRes, planeRes, planeRes);

	ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100, laserCloudCornerLastHandler);

	ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100, laserCloudSurfLastHandler);

	ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 100, laserOdometryHandler);

	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100, laserCloudFullResHandler);

    	ros::Subscriber subVinsOdometry = nh.subscribe<nav_msgs::Odometry>("visual_odom_last", 100, VinsOdometryHandler);        //Didula - synced visual odometry
        
	ros::Subscriber sub_GPS = nh.subscribe("/fix", 100, GPSHandler); 	//D - Subscribe to GPS messages

        ros::Subscriber sub_nmea = nh.subscribe("/nmea_sentence", 100, nmeaCallback); //O - for ppk call back
        pub_ppk = nh.advertise<sensor_msgs::NavSatFix>("/fix_ppk", 100);

    	//ros::Subscriber sub_GPS = nh.subscribe("/gps/fix", 100, GPSHandler);     //D - LVI-SAM GPS topic

	pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 100);

	pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 100);

	pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100);

	pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);

	pubOdomAftMappedHighFrec = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100);

	pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);

	for (int i = 0; i < laserCloudNum; i++)
	{
		laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
		laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
	}

	std::thread mapping_process{process};

	ros::spin();

	return 0;
}
