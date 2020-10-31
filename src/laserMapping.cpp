// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

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
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <Exp_mat.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <opencv/cv.h>
#include <common_lib.h>
#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <opencv2/core/eigen.hpp>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <fast_lio/States.h>
#include <geometry_msgs/Vector3.h>


#ifndef DEPLOY
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

#define INIT_TIME           (1.0)
#define LASER_POINT_COV     (0.0015)
#define NUM_MATCH_POINTS    (5)

std::string root_dir = ROOT_DIR;

int iterCount = 0;
int NUM_MAX_ITERATIONS  = 0;
int laserCloudCenWidth  = 20;
int laserCloudCenHeight = 10;
int laserCloudCenDepth  = 20;
int laserCloudValidInd[250];
int laserCloudSurroundInd[250];
int laserCloudValidNum    = 0;
int laserCloudSurroundNum = 0;

const int laserCloudWidth  = 42;
const int laserCloudHeight = 22;
const int laserCloudDepth  = 42;
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;//4851

/// IMU relative variables
std::mutex mtx_buffer;
std::condition_variable sig_buffer;
bool lidar_pushed = false;
bool b_exit = false;
bool b_reset = false;
bool b_first = true;

/// Buffers for measurements
double cube_len = 0.0;
double lidar_end_time = 0.0;
double last_timestamp_lidar = -1;
double last_timestamp_imu   = -1;
double HALF_FOV_COS = 0.0;

std::deque<sensor_msgs::PointCloud2::ConstPtr> lidar_buffer;
std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

//surf feature in map
PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());

std::deque< fast_lio::States > rot_kp_imu_buff;

//all points
PointCloudXYZI::Ptr laserCloudFullRes2(new PointCloudXYZI());
PointCloudXYZI::Ptr featsArray[laserCloudNum];
pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudFullResColor(new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());

//estimator inputs and output;
MeasureGroup Measures;
StatesGroup  state;

void SigHandle(int sig)
{
  b_exit = true;
  ROS_WARN("catch sig %d", sig);
  sig_buffer.notify_all();
}

//project lidar frame to world
void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(state.rot_end * (p_body + Lidar_offset_to_IMU) + state.pos_end);
    
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyToWorld(PointType const * const pi, pcl::PointXYZRGB * const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(state.rot_end * (p_body + Lidar_offset_to_IMU) + state.pos_end);
    
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    //po->intensity = pi->intensity;

    float intensity = pi->intensity;
    intensity = intensity - std::floor(intensity);

    int reflection_map = intensity*10000;

    //std::cout<<"DEBUG reflection_map "<<reflection_map<<std::endl;

    if (reflection_map < 30)
    {
        int green = (reflection_map * 255 / 30);
        po->r = 0;
        po->g = green & 0xff;
        po->b = 0xff;
    }
    else if (reflection_map < 90)
    {
        int blue = (((90 - reflection_map) * 255) / 60);
        po->r = 0x0;
        po->g = 0xff;
        po->b = blue & 0xff;
    }
    else if (reflection_map < 150)
    {
        int red = ((reflection_map-90) * 255 / 60);
        po->r = red & 0xff;
        po->g = 0xff;
        po->b = 0x0;
    }
    else
    {
        int green = (((255-reflection_map) * 255) / (255-150));
        po->r = 0xff;
        po->g = green & 0xff;
        po->b = 0;
    }
}

void lasermap_fov_segment()
{
    laserCloudValidNum    = 0;
    laserCloudSurroundNum = 0;
    PointType pointOnYAxis;
    pointOnYAxis.x = LIDAR_SP_LEN;
    pointOnYAxis.y = 0.0;
    pointOnYAxis.z = 0.0;
    pointBodyToWorld(&pointOnYAxis, &pointOnYAxis);
    // std::cout<<"special point:"<<pointOnYAxis.x<<" "<<pointOnYAxis.y<<" "<<pointOnYAxis.z<<" "<<std::endl;

    int centerCubeI = int((state.pos_end(0) + 0.5 * cube_len) / cube_len) + laserCloudCenWidth;
    int centerCubeJ = int((state.pos_end(1) + 0.5 * cube_len) / cube_len) + laserCloudCenHeight;
    int centerCubeK = int((state.pos_end(2) + 0.5 * cube_len) / cube_len) + laserCloudCenDepth;

    if (state.pos_end(0) + 0.5 * cube_len < 0) centerCubeI--;
    if (state.pos_end(1) + 0.5 * cube_len < 0) centerCubeJ--;
    if (state.pos_end(2) + 0.5 * cube_len < 0) centerCubeK--;

    while (centerCubeI < 3)
    {
        for (int j = 0; j < laserCloudHeight; j++)
        {
            for (int k = 0; k < laserCloudDepth; k++)
            {
                int i = laserCloudWidth - 1;

                PointCloudXYZI::Ptr laserCloudCubeSurfPointer =
                        featsArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

                for (; i >= 1; i--) {
                    featsArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            featsArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                }
                featsArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeSurfPointer;
                laserCloudCubeSurfPointer->clear();
            }
        }
        centerCubeI++;
        laserCloudCenWidth++;
    }

    while (centerCubeI >= laserCloudWidth - 3) {
        for (int j = 0; j < laserCloudHeight; j++) {
            for (int k = 0; k < laserCloudDepth; k++) {
                int i = 0;
                PointCloudXYZI::Ptr laserCloudCubeSurfPointer =
                        featsArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

                for (; i < laserCloudWidth - 1; i++)
                {
                    featsArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            featsArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                }

                featsArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeSurfPointer;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeI--;
        laserCloudCenWidth--;
    }

    while (centerCubeJ < 3) {
        for (int i = 0; i < laserCloudWidth; i++) {
            for (int k = 0; k < laserCloudDepth; k++) {
                int j = laserCloudHeight - 1;
                PointCloudXYZI::Ptr laserCloudCubeSurfPointer =
                        featsArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

                for (; j >= 1; j--) {
                    featsArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            featsArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight*k];
                }
                featsArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeSurfPointer;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeJ++;
        laserCloudCenHeight++;
    }

    while (centerCubeJ >= laserCloudHeight - 3) {
        for (int i = 0; i < laserCloudWidth; i++) {
            for (int k = 0; k < laserCloudDepth; k++) {
                int j = 0;
                PointCloudXYZI::Ptr laserCloudCubeSurfPointer =
                        featsArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

                for (; j < laserCloudHeight - 1; j++) {
                    featsArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            featsArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight*k];
                }
                featsArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeSurfPointer;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeJ--;
        laserCloudCenHeight--;
    }

    while (centerCubeK < 3) {
        for (int i = 0; i < laserCloudWidth; i++) {
            for (int j = 0; j < laserCloudHeight; j++) {
                int k = laserCloudDepth - 1;
                PointCloudXYZI::Ptr laserCloudCubeSurfPointer =
                        featsArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

                for (; k >= 1; k--) {
                    featsArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            featsArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k - 1)];
                }
                featsArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeSurfPointer;
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
                PointCloudXYZI::Ptr laserCloudCubeSurfPointer =
                        featsArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            
                for (; k < laserCloudDepth - 1; k++)
                {
                    featsArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            featsArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k + 1)];
                }

                featsArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeSurfPointer;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeK--;
        laserCloudCenDepth--;
    }

    for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++) 
    {
        for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++) 
        {
            for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++) 
            {
                if (i >= 0 && i < laserCloudWidth &&
                        j >= 0 && j < laserCloudHeight &&
                        k >= 0 && k < laserCloudDepth) 
                {

                    float centerX = cube_len * (i - laserCloudCenWidth);
                    float centerY = cube_len * (j - laserCloudCenHeight);
                    float centerZ = cube_len * (k - laserCloudCenDepth);

                    float check1, check2;
                    float squaredSide1, squaredSide2;
                    float ang_cos = 1;

                    bool isInLaserFOV = false;

                    for (int ii = -1; ii <= 1; ii += 2) 
                    {
                        for (int jj = -1; jj <= 1; jj += 2) 
                        {
                            for (int kk = -1; (kk <= 1) && (!isInLaserFOV); kk += 2) 
                            {

                                float cornerX = centerX + 0.5 * cube_len * ii;
                                float cornerY = centerY + 0.5 * cube_len * jj;
                                float cornerZ = centerZ + 0.5 * cube_len * kk;

                                squaredSide1 = (state.pos_end(0) - cornerX)
                                        * (state.pos_end(0) - cornerX)
                                        + (state.pos_end(1) - cornerY)
                                        * (state.pos_end(1) - cornerY)
                                        + (state.pos_end(2) - cornerZ)
                                        * (state.pos_end(2) - cornerZ);

                                squaredSide2 = (pointOnYAxis.x - cornerX) * (pointOnYAxis.x - cornerX)
                                        + (pointOnYAxis.y - cornerY) * (pointOnYAxis.y - cornerY)
                                        + (pointOnYAxis.z - cornerZ) * (pointOnYAxis.z - cornerZ);

                                ang_cos = fabs(squaredSide1 <= 3) ? 1.0 :
                                    (LIDAR_SP_LEN * LIDAR_SP_LEN + squaredSide1 - squaredSide2) / (2 * LIDAR_SP_LEN * sqrt(squaredSide1));
                                
                                if(ang_cos > HALF_FOV_COS) isInLaserFOV = true;
                            }
                        }
                    }
                    
                    if(!isInLaserFOV)
                    {
                        float cornerX = centerX;
                        float cornerY = centerY;
                        float cornerZ = centerZ;

                        squaredSide1 = (state.pos_end(0) - cornerX)
                                * (state.pos_end(0) - cornerX)
                                + (state.pos_end(1) - cornerY)
                                * (state.pos_end(1) - cornerY)
                                + (state.pos_end(2) - cornerZ)
                                * (state.pos_end(2) - cornerZ);

                        if(squaredSide1 <= 0.4 * cube_len * cube_len)
                        {
                            isInLaserFOV = true;
                        }

                        squaredSide2 = (pointOnYAxis.x - cornerX) * (pointOnYAxis.x - cornerX)
                                + (pointOnYAxis.y - cornerY) * (pointOnYAxis.y - cornerY)
                                + (pointOnYAxis.z - cornerZ) * (pointOnYAxis.z - cornerZ);
                        
                        ang_cos = fabs(squaredSide2 <= 0.5 * cube_len) ? 1.0 :
                            (LIDAR_SP_LEN * LIDAR_SP_LEN + squaredSide1 - squaredSide2) / (2 * LIDAR_SP_LEN * sqrt(squaredSide1));
                        
                        if(ang_cos > HALF_FOV_COS) isInLaserFOV = true;
                    }

                    // std::cout<<"cent point: "<<centerX<<" "<<centerY<<" "<<centerZ<<" infov? "<<isInLaserFOV<<std::endl;
                    
                    if (isInLaserFOV)
                    {
                        
                        laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j
                                + laserCloudWidth * laserCloudHeight * k;
                        laserCloudValidNum ++;
                    }
                    laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j
                            + laserCloudWidth * laserCloudHeight * k;
                    laserCloudSurroundNum ++;
                    
                }
            }
        }
    }

    featsFromMap->clear();
    
    for (int i = 0; i < laserCloudValidNum; i++)
    {
        *featsFromMap += *featsArray[laserCloudValidInd[i]];
    }
}

void feat_points_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    mtx_buffer.lock();
    // std::cout<<"got feature"<<std::endl;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
    lidar_buffer.push_back(msg);
    last_timestamp_lidar = msg->header.stamp.toSec();

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_ERROR("imu loop back, clear buffer");
        imu_buffer.clear();
        b_reset = true;
        b_first = true;
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    // std::cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<std::endl;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push lidar frame ***/
    if(!lidar_pushed)
    {
        meas.lidar.reset(new PointCloudXYZI());
        pcl::fromROSMsg(*(lidar_buffer.front()), *(meas.lidar));
        meas.lidar_beg_time = lidar_buffer.front()->header.stamp.toSec();
        lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    lidar_pushed = false;
    // std::cout<<"[IMU Sycned]: "<<imu_time<<" "<<lidar_end_time<<std::endl;
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    ros::Subscriber sub_pcl = nh.subscribe("/laser_cloud_flat", 20000, feat_points_cbk);
    ros::Subscriber sub_imu = nh.subscribe("/livox/imu", 20000, imu_cbk);
    // ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>
    //         ("/livox_undistort", 100, featsLastHandler);
    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 100);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 100);
    // ros::Publisher pubSolvedPose6D = nh.advertise<fast_lio::States>
    //         ("/States_updated", 100);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> 
            ("/aft_mapped_to_init", 10);

#ifdef DEPLOY
    ros::Publisher mavros_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    geometry_msgs::PoseStamped msg_body_pose;
#endif
    
    /*** variables definition ***/
    bool dense_map_en, Need_Init = true;
    std::string map_file_path;
    int effect_feat_num = 0, frame_num = 0;
    double filter_size_corner_min, filter_size_surf_min, filter_size_map_min, fov_deg,\
           deltaT, deltaR, aver_time_consu = 0, first_lidar_time = 0;
    Eigen::Matrix<double,DIM_OF_STATES,DIM_OF_STATES> G, H_T_H, I_STATE;
    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();

    nav_msgs::Odometry odomAftMapped;

    cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

    PointType pointOri, pointSel, coeff;
    PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
    PointCloudXYZI::Ptr feats_down(new PointCloudXYZI());
    PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI());
    PointCloudXYZI::Ptr coeffSel(new PointCloudXYZI());
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterMap;

    /*** variables initialize ***/
    ros::param::get("~dense_map_enable",dense_map_en);
    ros::param::get("~max_iteration",NUM_MAX_ITERATIONS);
    ros::param::get("~map_file_path",map_file_path);
    ros::param::get("~fov_degree",fov_deg);
    ros::param::get("~filter_size_corner",filter_size_corner_min);
    ros::param::get("~filter_size_surf",filter_size_surf_min);
    ros::param::get("~filter_size_map",filter_size_map_min);
    ros::param::get("~cube_side_length",cube_len);

    HALF_FOV_COS = std::cos((fov_deg + 10.0) * 0.5 * PI_M / 180.0);

    for (int i = 0; i < laserCloudNum; i++)
    {
        featsArray[i].reset(new PointCloudXYZI());
    }

    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

    std::shared_ptr<ImuProcess> p_imu(new ImuProcess());

    /*** debug record ***/
    std::ofstream fout_pre, fout_out;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),std::ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),std::ios::out);
    if (fout_pre && fout_out)
        std::cout << "~~~~"<<ROOT_DIR<<" file opened" << std::endl;
    else
        std::cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << std::endl;
    std::vector<double> T1, s_plot, s_plot2, s_plot3;

//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    while (status)
    {
        if (b_exit) break;
        ros::spinOnce();
        while(sync_packages(Measures)) 
        {
            if (b_reset)
            {
                ROS_WARN("reset when rosbag play back");
                p_imu->Reset();
                b_reset = false;
                continue;
            }

            double t1,t2,t3,t4,match_start, match_time, solve_start, solve_time, pca_time, svd_time;
            match_time = 0;
            solve_time = 0;
            pca_time   = 0;
            svd_time   = 0;
            t1 = omp_get_wtime();

            p_imu->Process(Measures, state, feats_undistort);

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                first_lidar_time = Measures.lidar_beg_time;
                std::cout<<"not ready for odometry"<<std::endl;
                continue;
            }

            if ((Measures.lidar_beg_time - first_lidar_time) < INIT_TIME)
            {
                Need_Init = true;
                std::cout<<"||||||||||Initiallizing LiDar||||||||||"<<std::endl;
            }
            else
            {
                Need_Init = false;
            }
            
            /*** Compute the euler angle ***/
            Eigen::Vector3d euler_cur = RotMtoEuler(state.rot_end);
            fout_pre << std::setw(10) << Measures.lidar_beg_time << " " << euler_cur.transpose()*57.3 << " " << state.pos_end.transpose() << " " << state.vel_end.transpose() \
            <<" "<<state.bias_g.transpose()<<" "<<state.bias_a.transpose()<< std::endl;
            #ifdef DEBUG_PRINT
            std::cout<<"current lidar time "<<Measures.lidar_beg_time<<" "<<"first lidar time "<<first_lidar_time<<std::endl;
            std::cout<<"pre-integrated states: "<<euler_cur.transpose()*57.3<<" "<<state.pos_end.transpose()<<" "<<state.vel_end.transpose()<<" "<<state.bias_g.transpose()<<" "<<state.bias_a.transpose()<<std::endl;
            #endif
            
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();
            
            /*** downsample the features and maps ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down);
            downSizeFilterMap.setInputCloud(featsFromMap);
            downSizeFilterMap.filter(*featsFromMap);

            int featsFromMapNum = featsFromMap->points.size();
            int feats_down_size = feats_down->points.size();
            std::cout<<"[ mapping ]: Raw feature num: "<<feats_undistort->points.size()<<" downsamp num "<<feats_down_size<<" Map num: "<<featsFromMapNum<<" laserCloudValidNum "<<laserCloudValidNum<<std::endl;

            /*** ICP and iterated Kalman filter update ***/
            PointCloudXYZI::Ptr coeffSel_tmpt (new PointCloudXYZI(*feats_down));
            PointCloudXYZI::Ptr feats_down_updated (new PointCloudXYZI(*feats_down));

            if (featsFromMapNum > 100)
            {
                kdtreeSurfFromMap->setInputCloud(featsFromMap);
                std::vector<bool> point_selected_surf(feats_down_size, true);
                std::vector<std::vector<int>> pointSearchInd_surf(feats_down_size);
                
                int  rematch_num = 0;
                bool rematch_en = 0;

                t2 = omp_get_wtime();
                
                for (iterCount = 0; iterCount < NUM_MAX_ITERATIONS; iterCount++) 
                {
                    match_start = omp_get_wtime();
                    laserCloudOri->clear();
                    coeffSel->clear();

                    /** closest surface search and residual computation **/
                    omp_set_num_threads(4);
                    #pragma omp parallel for
                    for (int i = 0; i < feats_down_size; i++)
                    {
                        PointType &pointOri_tmpt = feats_down->points[i];
                        PointType &pointSel_tmpt = feats_down_updated->points[i];

                        /* transform to world frame */
                        pointBodyToWorld(&pointOri_tmpt, &pointSel_tmpt);

                        std::vector<float> pointSearchSqDis_surf;
                        auto &points_near = pointSearchInd_surf[i];
                        if (iterCount == 0 || rematch_en)
                        {
                            /** Find the closest surface/line in the map **/
                            kdtreeSurfFromMap->nearestKSearch(pointSel_tmpt, NUM_MATCH_POINTS, points_near, pointSearchSqDis_surf);
                            if (pointSearchSqDis_surf[NUM_MATCH_POINTS - 1] < 3.0)
                            {
                                point_selected_surf[i] = true;
                            }
                        }
                        
                        if (! point_selected_surf[i]) continue;

                        double pca_start = omp_get_wtime();

                        /// PCA (using minimum square method)
                        cv::Mat matA0(NUM_MATCH_POINTS, 3, CV_32F, cv::Scalar::all(0));
                        cv::Mat matB0(NUM_MATCH_POINTS, 1, CV_32F, cv::Scalar::all(-1));
                        cv::Mat matX0(NUM_MATCH_POINTS, 1, CV_32F, cv::Scalar::all(0));

                        for (int j = 0; j < NUM_MATCH_POINTS; j++)
                        {
                            matA0.at<float>(j, 0) = featsFromMap->points[points_near[j]].x;
                            matA0.at<float>(j, 1) = featsFromMap->points[points_near[j]].y;
                            matA0.at<float>(j, 2) = featsFromMap->points[points_near[j]].z;
                        }

                        //matA0*matX0=matB0
                        //AX+BY+CZ+D = 0 <=> AX+BY+CZ=-D <=> (A/D)X+(B/D)Y+(C/D)Z = -1
                        //(X,Y,Z)<=>mat_a0
                        //A/D, B/D, C/D <=> mat_x0
            
                        cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);  //TODO

                        float pa = matX0.at<float>(0, 0);
                        float pb = matX0.at<float>(1, 0);
                        float pc = matX0.at<float>(2, 0);
                        float pd = 1;

                        //ps is the norm of the plane norm_vec vector
                        //pd is the distance from point to plane
                        float ps = sqrt(pa * pa + pb * pb + pc * pc);
                        pa /= ps;
                        pb /= ps;
                        pc /= ps;
                        pd /= ps;

                        bool planeValid = true;
                        for (int j = 0; j < NUM_MATCH_POINTS; j++)
                        {
                            if (fabs(pa * featsFromMap->points[points_near[j]].x +
                                        pb * featsFromMap->points[points_near[j]].y +
                                        pc * featsFromMap->points[points_near[j]].z + pd) > 0.1)
                            {
                                planeValid = false;
                                point_selected_surf[i] = false;
                                break;
                            }
                        }

                        if (planeValid) 
                        {
                            //loss fuction
                            float pd2 = pa * pointSel_tmpt.x + pb * pointSel_tmpt.y + pc * pointSel_tmpt.z + pd;
                            //if(fabs(pd2) > 0.1) continue;
                            float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel_tmpt.x * pointSel_tmpt.x + pointSel_tmpt.y * pointSel_tmpt.y + pointSel_tmpt.z * pointSel_tmpt.z));

                            if (s > 0.1)
                            {
                                point_selected_surf[i] = true;
                                coeffSel_tmpt->points[i].x = s * pa;
                                coeffSel_tmpt->points[i].y = s * pb;
                                coeffSel_tmpt->points[i].z = s * pc;
                                coeffSel_tmpt->points[i].intensity = s * pd2;
                            }
                            else
                            {
                                point_selected_surf[i] = false;
                            }
                        }

                        pca_time += omp_get_wtime() - pca_start;
                    }

                    double total_residual = 0.0;
                    for (int i = 0; i < coeffSel_tmpt->points.size(); i++)
                    {
                        float error_abs = std::abs(coeffSel_tmpt->points[i].intensity);
                        if (point_selected_surf[i]  && (error_abs < 0.5))
                        {
                            laserCloudOri->push_back(feats_down->points[i]);
                            coeffSel->push_back(coeffSel_tmpt->points[i]);
                            total_residual += error_abs;
                        }
                    }

                    int laserCloudSelNum = laserCloudOri->points.size();
                    double ave_residual = total_residual / laserCloudSelNum;
                    // ave_res_last 

                    effect_feat_num = coeffSel->size();
                    if(iterCount == 1) std::cout << "[ mapping ]: Effective feature num: "<<effect_feat_num<<std::endl;

                    match_time += omp_get_wtime() - match_start;
                    solve_start = omp_get_wtime();

                    if (laserCloudSelNum < 50) {
                        continue;
                    }
                    
                    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
                    Eigen::MatrixXd Hsub(laserCloudSelNum, 6);
                    Eigen::VectorXd meas_vec(laserCloudSelNum);
                    Hsub.setZero();

                    omp_set_num_threads(4);
                    #pragma omp parallel for
                    for (int i = 0; i < laserCloudSelNum; i++)
                    {
                        const PointType &laser_p  = laserCloudOri->points[i];
                        Eigen::Vector3d point_this(laser_p.x, laser_p.y, laser_p.z);
                        point_this += Lidar_offset_to_IMU;
                        Eigen::Matrix3d point_crossmat;
                        point_crossmat<<SKEW_SYM_MATRX(point_this);

                        /*** get the normal vector of closest surface/corner ***/
                        const PointType &norm_p = coeffSel->points[i];
                        Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);

                        /*** calculate the Measuremnt Jacobian matrix H ***/
                        Eigen::Vector3d A(point_crossmat * state.rot_end.transpose() * norm_vec);
                        Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z;

                        /*** Measuremnt: distance to the closest surface/corner ***/
                        meas_vec(i) = - norm_p.intensity;
                    }

                    Eigen::Vector3d rot_add, t_add, v_add, bg_add, ba_add, g_add;
                    Eigen::VectorXd solution(DIM_OF_STATES);
                    Eigen::MatrixXd K(DIM_OF_STATES, laserCloudSelNum);
                    
                    /*** Iterative Kalman Filter Update ***/
                    if (Need_Init)
                    {
                        /*** only run in initialization period ***/
                        Eigen::MatrixXd H_init(Eigen::Matrix<double, 9, DIM_OF_STATES>::Zero());
                        Eigen::MatrixXd z_init(Eigen::Matrix<double, 9, 1>::Zero());
                        H_init.block<3,3>(0,0)  = Eigen::Matrix3d::Identity();
                        H_init.block<3,3>(3,3)  = Eigen::Matrix3d::Identity();
                        H_init.block<3,3>(6,15) = Eigen::Matrix3d::Identity();
                        z_init.block<3,1>(0,0)  = - Log(state.rot_end);
                        z_init.block<3,1>(0,0)  = - state.pos_end;

                        auto H_init_T = H_init.transpose();
                        auto &&K_init = state.cov * H_init_T * (H_init * state.cov * H_init_T + 0.0001 * Eigen::Matrix<double,9,9>::Identity()).inverse();
                        solution      = K_init * z_init;

                        solution.block<9,1>(0,0).setZero();
                        state += solution;
                        state.cov = (Eigen::MatrixXd::Identity(DIM_OF_STATES, DIM_OF_STATES) - K_init * H_init) * state.cov;
                    }
                    else
                    {
                        auto &&Hsub_T = Hsub.transpose();
                        H_T_H.block<6,6>(0,0) = Hsub_T * Hsub;
                        Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> &&K_1 = (H_T_H + (state.cov / LASER_POINT_COV).inverse()).inverse();
                        K = K_1.block<DIM_OF_STATES,6>(0,0) * Hsub_T;
                        solution = K * meas_vec;
                        state += solution;

                        rot_add = solution.block<3,1>(0,0);
                        t_add   = solution.block<3,1>(3,0);

                        deltaR = rot_add.norm() * 57.3;
                        deltaT = t_add.norm() * 100.0;
                    }

                    euler_cur = RotMtoEuler(state.rot_end);

                    #ifdef DEBUG_PRINT
                    // std::cout<<"solution: "<<solution.transpose()<<std::endl;
                    std::cout<<"update: R"<<euler_cur.transpose()*57.3<<" p "<<state.pos_end.transpose()<<" v "<<state.vel_end.transpose()<<" bg"<<state.bias_g.transpose()<<" ba"<<state.bias_a.transpose()<<std::endl;
                    std::cout<<"dR & dT: "<<deltaR<<" "<<deltaT<<" res norm:"<<ave_residual<<std::endl;
                    #endif

                    /*** Rematch Judgement ***/
                    rematch_en = false;
                    if ((deltaR < 0.07 && deltaT < 0.015))
                    {
                        rematch_en = true;
                        rematch_num ++;
                    }

                    /*** Convergence Judgements and Covariance Update ***/
                    if (rematch_num >= 2 || (iterCount == NUM_MAX_ITERATIONS - 1))
                    {
                        if (!Need_Init)
                        {
                            /*** Covariance Update ***/
                            G.block<DIM_OF_STATES,6>(0,0) = K * Hsub;
                            state.cov = (I_STATE - G) * state.cov;
                            // std::cout<<"propagated cov: "<<state.cov.diagonal().transpose()<<std::endl;
                        }
                        solve_time += omp_get_wtime() - solve_start;
                        break;
                    }
                    solve_time += omp_get_wtime() - solve_start;
                }
                
                std::cout<<"[ mapping ]: iteration count: "<<iterCount+1<<std::endl;
            }
            
            bool if_cube_updated[laserCloudNum] = {0};
            for (int i = 0; i < feats_down_size; i++)
            {
                PointType &pointSel = feats_down_updated->points[i];

                int cubeI = int((pointSel.x + 0.5 * cube_len) / cube_len) + laserCloudCenWidth;
                int cubeJ = int((pointSel.y + 0.5 * cube_len) / cube_len) + laserCloudCenHeight;
                int cubeK = int((pointSel.z + 0.5 * cube_len) / cube_len) + laserCloudCenDepth;

                if (pointSel.x + 0.5 * cube_len < 0) cubeI--;
                if (pointSel.y + 0.5 * cube_len < 0) cubeJ--;
                if (pointSel.z + 0.5 * cube_len < 0) cubeK--;

                if (cubeI >= 0 && cubeI < laserCloudWidth &&
                        cubeJ >= 0 && cubeJ < laserCloudHeight &&
                        cubeK >= 0 && cubeK < laserCloudDepth) {
                    int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                    featsArray[cubeInd]->push_back(pointSel);
                    if_cube_updated[cubeInd] = true;
                }
            }
            for (int i = 0; i < laserCloudValidNum; i++)
            {
                int ind = laserCloudValidInd[i];

                if(if_cube_updated[ind])
                {
                    downSizeFilterMap.setInputCloud(featsArray[ind]);
                    downSizeFilterMap.filter(*featsArray[ind]);
                }
            }
            t3 = omp_get_wtime();

            /******* Publish current frame points in world coordinates:  *******/            
            laserCloudFullRes2->clear();
            *laserCloudFullRes2 = dense_map_en ? (*feats_undistort) : (* feats_down);
            // *laserCloudFullRes2 = dense_map_en ? (*laserCloudFullRes) : (* feats_down);

            int laserCloudFullResNum = laserCloudFullRes2->points.size();

            pcl::PointXYZRGB temp_point;
            laserCloudFullResColor->clear();
            for (int i = 0; i < laserCloudFullResNum; i++)
            {
                RGBpointBodyToWorld(&laserCloudFullRes2->points[i], &temp_point);
                laserCloudFullResColor->push_back(temp_point);
            }

            sensor_msgs::PointCloud2 laserCloudFullRes3;
            pcl::toROSMsg(*laserCloudFullResColor, laserCloudFullRes3);
            laserCloudFullRes3.header.stamp = ros::Time::now();//.fromSec(last_timestamp_lidar);
            laserCloudFullRes3.header.frame_id = "/camera_init";
            pubLaserCloudFullRes.publish(laserCloudFullRes3);

            /******* Publish Maps:  *******/
            // sensor_msgs::PointCloud2 laserCloudMap;
            // pcl::toROSMsg(*featsFromMap, laserCloudMap);
            // laserCloudMap.header.stamp = ros::Time::now();//ros::Time().fromSec(last_timestamp_lidar);
            // laserCloudMap.header.frame_id = "/camera_init";
            // pubLaserCloudMap.publish(laserCloudMap);

            /******* Publish Odometry ******/
            geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                    (euler_cur(2), - euler_cur(0), - euler_cur(1));
            odomAftMapped.header.frame_id = "/camera_init";
            odomAftMapped.child_frame_id = "/aft_mapped";
            odomAftMapped.header.stamp = ros::Time::now();//ros::Time().fromSec(last_timestamp_lidar);
            odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
            odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
            odomAftMapped.pose.pose.orientation.z = geoQuat.x;
            odomAftMapped.pose.pose.orientation.w = geoQuat.w;
            odomAftMapped.pose.pose.position.x = state.pos_end(0);
            odomAftMapped.pose.pose.position.y = state.pos_end(1);
            odomAftMapped.pose.pose.position.z = state.pos_end(2);

            pubOdomAftMapped.publish(odomAftMapped);

            static tf::TransformBroadcaster br;
            tf::Transform                   transform;
            tf::Quaternion                  q;
            transform.setOrigin( tf::Vector3( odomAftMapped.pose.pose.position.x,
                                                odomAftMapped.pose.pose.position.y,
                                                odomAftMapped.pose.pose.position.z ) );
            q.setW( odomAftMapped.pose.pose.orientation.w );
            q.setX( odomAftMapped.pose.pose.orientation.x );
            q.setY( odomAftMapped.pose.pose.orientation.y );
            q.setZ( odomAftMapped.pose.pose.orientation.z );
            transform.setRotation( q );
            br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "/camera_init", "/aft_mapped" ) );

            #ifdef DEPLOY
            msg_body_pose.header.stamp = ros::Time::now();
            msg_body_pose.header.frame_id = "/camera_odom_frame";
            msg_body_pose.pose.position.x = state.pos_end(0);
            msg_body_pose.pose.position.y = - state.pos_end(1);
            msg_body_pose.pose.position.z = - state.pos_end(2);
            msg_body_pose.pose.orientation.x = - geoQuat.y;
            msg_body_pose.pose.orientation.y = - geoQuat.z;
            msg_body_pose.pose.orientation.z = geoQuat.x;
            msg_body_pose.pose.orientation.w = geoQuat.w;
            mavros_pose_publisher.publish(msg_body_pose);
            #endif

            /*** save debug variables ***/
            t4 = omp_get_wtime();
            frame_num ++;
            aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t3 - t1) / frame_num;
            // aver_time_consu = aver_time_consu * 0.5 + (t4 - t1) * 0.5;
            T1.push_back(Measures.lidar_beg_time);
            s_plot.push_back(aver_time_consu);
            // s_plot2.push_back(double(deltaR));
            // s_plot3.push_back(double(deltaT));

            std::cout<<"[ mapping ]: time: selection "<<t2-t1 <<" match "<<match_time<<" pca "<<pca_time<<" solve "<<solve_time<<" total "<<t4 - t1<<std::endl;
            fout_out << std::setw(10) << Measures.lidar_beg_time << " " << euler_cur.transpose()*57.3 << " " << state.pos_end.transpose() << " " << state.vel_end.transpose() \
            <<" "<<state.bias_g.transpose()<<" "<<state.bias_a.transpose()<< std::endl;
            // fout_out << std::setw(10) << Measures.lidar_beg_time << " " << t3-t1 << " " << effect_feat_num << std::endl;
        }
        status = ros::ok();
        rate.sleep();
    }
    //--------------------------save map---------------
    std::string surf_filename(map_file_path + "/surf.pcd");
    std::string corner_filename(map_file_path + "/corner.pcd");
    std::string all_points_filename(map_file_path + "/all_points.pcd");

    PointCloudXYZI surf_points, corner_points;
    surf_points = *featsFromMap;
    fout_out.close();
    fout_pre.close();
    if (surf_points.size() > 0 && corner_points.size() > 0) 
    {
    pcl::PCDWriter pcd_writer;
    std::cout << "saving...";
    pcd_writer.writeBinary(surf_filename, surf_points);
    pcd_writer.writeBinary(corner_filename, corner_points);
    }
    else
    {
        // #ifdef DEBUG_PRINT
        #ifndef DEPLOY
        if (!T1.empty())
        {
            plt::named_plot("time consumed",T1,s_plot);
            // plt::named_plot("R_residual",T1,s_plot2);
            // plt::named_plot("T_residual",T1,s_plot3);
            plt::legend();
            plt::show();
            plt::pause(0.5);
            plt::close();
            // plt::save("/home/xw/catkin_like_loam/src/LIEK_LOAM/a.png");
        }
        std::cout << "no points saved";
        // #endif
        #endif
    }
    //--------------------------
    //  loss_output.close();
    return 0;
}
