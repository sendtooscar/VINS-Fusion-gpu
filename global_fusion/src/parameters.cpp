/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;
std::vector<Eigen::Matrix3d> RCL;
std::vector<Eigen::Vector3d> TCL;


double L_C_tx;
double L_C_ty;
double L_C_tz;
double L_C_rx;
double L_C_ry;
double L_C_rz;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string OUTPUT_FOLDER;
std::string IMU_TOPIC;
std::string POINT_CLOUD_TOPIC;
int ROW, COL;
double TD;
int NUM_OF_CAM;
int STEREO;
int USE_IMU;
int USE_LIDAR;
int USE_LOAM;
int LIDAR_SKIP;
int USE_DENSE_CLOUD;
int MULTIPLE_THREAD;
int USE_GPU;
int USE_GPU_ACC_FLOW;
int PUB_RECTIFY;
Eigen::Matrix3d rectify_R_left;
Eigen::Matrix3d rectify_R_right;
map<int, Eigen::Vector3d> pts_gt;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::string FISHEYE_MASK;
std::vector<std::string> CAM_NAMES;
int MAX_CNT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;


double MAP_THETA;
int MAP_THETA_OPTIMIZE;
std::string PPK_PATH;
std::string FRL_PATH;
int VIZ_PPK;
int VIZ_FRL;
int USE_MAG;
int USE_GPS;
int USE_FRL_ROT;
int USE_FRL;
int USE_PPK;

/*map_theta: 0   # so that gps path has the correct rotation to visualize
map_theta_optimize : false # automatically optimizes mapped theta
ppk_path: "test"  # this is the ppk file path
frl_path: "test"  # this is the path of the frl file having all the data
viz_ppk: false    # this visualizes ppk
viz_frl: false    # this visualizes ppk
use_mag: false    # to enable yaw updates
use_gps: false    # to do gps update
use_frl_rot : false # updates using the rotation of the FRL file*/


template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;          
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image0_topic"] >> IMAGE0_TOPIC;
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    FLOW_BACK = fsSettings["flow_back"];

    fsSettings["point_cloud_topic"] >> POINT_CLOUD_TOPIC;
    USE_LIDAR = fsSettings["use_lidar"];
    USE_LOAM = fsSettings["use_loam"];
    USE_DENSE_CLOUD = fsSettings["use_dense_cloud"];
    LIDAR_SKIP = fsSettings["lidar_skip"];
  
    MULTIPLE_THREAD = fsSettings["multiple_thread"];

    USE_GPU = fsSettings["use_gpu"];
    USE_GPU_ACC_FLOW = fsSettings["use_gpu_acc_flow"];

    USE_IMU = fsSettings["imu"];
    printf("USE_IMU: %d\n", USE_IMU);
    if(USE_IMU)
    {
        fsSettings["imu_topic"] >> IMU_TOPIC;
        printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
        ACC_N = fsSettings["acc_n"];
        ACC_W = fsSettings["acc_w"];
        GYR_N = fsSettings["gyr_n"];
        GYR_W = fsSettings["gyr_w"];
        G.z() = fsSettings["g_norm"];
    }

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    fsSettings["output_path"] >> OUTPUT_FOLDER;
    VINS_RESULT_PATH = OUTPUT_FOLDER + "/vio.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_T;
        fsSettings["body_T_cam0"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    } 
    
    NUM_OF_CAM = fsSettings["num_of_cam"];
    printf("camera number %d\n", NUM_OF_CAM);

    if(NUM_OF_CAM != 1 && NUM_OF_CAM != 2)
    {
        printf("num_of_cam should be 1 or 2\n");
        assert(0);
    }

    if(USE_LIDAR == 1 || USE_LOAM == 1)
    {
        cv::Mat cv_T;
        fsSettings["cam0_T_lidar"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RCL.push_back(T.block<3, 3>(0, 0));
        TCL.push_back(T.block<3, 1>(0, 3));

        fsSettings["lidar_to_cam_tx"] >> L_C_tx;
	fsSettings["lidar_to_cam_ty"] >> L_C_ty;
	fsSettings["lidar_to_cam_tz"] >> L_C_tz;
	fsSettings["lidar_to_cam_rx"] >> L_C_rx;
	fsSettings["lidar_to_cam_ry"] >> L_C_ry;
	fsSettings["lidar_to_cam_rz"] >> L_C_rz;
    }

    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    CAM_NAMES.push_back(cam0Path);

    if(NUM_OF_CAM == 2)
    {
        STEREO = 1;
        std::string cam1Calib;
        fsSettings["cam1_calib"] >> cam1Calib;
        std::string cam1Path = configPath + "/" + cam1Calib; 
        //printf("%s cam1 path\n", cam1Path.c_str() );
        CAM_NAMES.push_back(cam1Path);
        
        cv::Mat cv_T;
        fsSettings["body_T_cam1"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
        fsSettings["publish_rectify"] >> PUB_RECTIFY;
    }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %d COL: %d ", ROW, COL);

    if(!USE_IMU)
    {
        ESTIMATE_EXTRINSIC = 0;
        ESTIMATE_TD = 0;
        printf("no imu, fix extrinsic param; no time offset calibration\n");
    }
    if(PUB_RECTIFY)
    {
        cv::Mat rectify_left;
        cv::Mat rectify_right;
        fsSettings["cam0_rectify"] >> rectify_left;
        fsSettings["cam1_rectify"] >> rectify_right;
        cv::cv2eigen(rectify_left, rectify_R_left);
        cv::cv2eigen(rectify_right, rectify_R_right);

    }

    
    MAP_THETA = fsSettings["map_theta"];
    MAP_THETA_OPTIMIZE = fsSettings["map_theta_optimize"];
    fsSettings["ppk_path"] >>  PPK_PATH;
    fsSettings["frl_path"] >>  FRL_PATH;
    VIZ_PPK = fsSettings["viz_ppk"];
    VIZ_FRL = fsSettings["viz_frl"];
    USE_MAG = fsSettings["use_mag"];
    USE_GPS = fsSettings["use_gps"];
    USE_FRL_ROT = fsSettings["use_frl_rot"];
    USE_FRL = fsSettings["use_frl"];



    fsSettings.release();
}


float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

void publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    if (thisPub->getNumSubscribers() == 0)
        return;
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    thisPub->publish(tempCloud); 
}
