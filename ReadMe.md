## FAST_LIO_COLOR

This code is coloring the point cloud based on FAST_LIO. It is coupled with camera.

**Required**

1. OpenCV >= 4.0
2. [ws_all_publisher](https://github.com/CVLab-DSR-Project/ws_all_publisher.git): To publish pointcloud, compressed image, IMU
3. [Fast_LIO] Requirements(https://github.com/hku-mars/FAST_LIO.git): Same as Fast_lio requirements


### Build from source

Clone the repository and catkin_make

'''
     cd ~/catkin_ws/src
     git clone https://github.com/hyeongaa/FAST_LIO_COLOR.git
     cd ..
     catkin_make
'''

### Run the rosbag dataset

Datasets can me find here().

'''
    source devel/setup.bash
    roslaunch fast_lio maaping_mid360_cam.launch

    rosbag play your_bag.bag
'''


### Parameters and config

To change the extrinsic parameter of camera and imu, or to change the intrinsic parameter of camera, check the config file.

'''
    cd src/FAST_LIO_COLOR/config
'''

mid360_cam.yaml contains the info of the parameters.


