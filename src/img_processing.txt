#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

//new
#include <sensor_msgs/Image.h>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Core>
#include <deque>
#include <vector>
#include <pcl/impl/point_types.hpp>



void IMU_TO_CAMERA(PointType const * const pi, PointType * const po)
{
    Eigen::Matrix4d extrinsic_matrix;
    extrinsic_matrix<< 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
           0.999557249008, 0.0149672133247, 0.025715529948,  -0.064676986768,
           -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
           0, 0, 0, 1;
    Eigen::Vector4d p_body(pi->x, pi->y, pi->z,1);
    Eigen::Vector4d c_body;
    c_body = extrinsic_matrix*p_body;

    po->x=c_body(0);
    po->y=c_body(1);
    po->z=c_body(2);
    po->intensity=pi->intensity;

}

void Camera_to_IMU(pcl::PointXYZRGB const * const pi, pcl::PointXYZRGB const * const po)
{
    Eigen::Matrix4d extrinsic_matrix;
    extrinsic_matrix<< 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
           0.999557249008, 0.0149672133247, 0.025715529948,  -0.064676986768,
           -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
           0, 0, 0, 1;

    Eigen::Matrix4d inverse_extrinsic = extrinsic_matrix.inverse();

    Eigen::Vector4d c_body(pi->x, pi->y, pi->z);
    Eigen::Vector4d b_body;
    b_body=inverse_extrinsic*c_body;

    po->x=b_body(0);
    po->y=b_body(1);
    po->z=b_body(2);
    po->r=pi->r;
    po->g=pi->g;
    po->b=pi->b;
      
}

void 2d_projection(PointType const * const pi, cv::Mat image, pcl::PointXYZRGB const * const po)
{
    Eigen::Matrix4d camera_projection_matrix;
    double gamma1 = 2.1387619122017772e+03;
    double gamma2 = 2.1315886210259278e+03;
    double u0 = 3.6119856633263799e+02;
    double v0 = 2.4827644773395667e+02;
    
    
    camera_projection_matrix<< gamma1, 0, u0, 0, 
                                0, gamma2, v0, 0,
                                0, 0, 1, 0;    


    Eigen::Vector4d p_body(pi->x, pi->y, pi->z,1);
    V3D c_body(camera_projection_matrix*p_body);

    double homo =1.0/c_body(2);      

    double x_coord = c_body(0)*homo;
    double y_coord = c_body(1)*homo;

    int u=static_cast<int>(x_coord);
    int v=static_cast<int>(y_coord);

    double alpha = x_coord-u;
    double beta  = y_coord-v; 

    if (u >= 0 && u < image.cols && v >= 0 && v < image.rows)
    {
        cv::Vec3b pixel = image.at<cv::Vec3b>(v,u);
        po->r = static_Cast<int>(pixel[2]);
        po->g = static_Cast<int>(pixel[1]);
        po->b = static_Cast<int>(pixel[0]);
    }

    po->x=pi->x;
    po->y=pi->y;
    po->z=pi->z;
                          
}




