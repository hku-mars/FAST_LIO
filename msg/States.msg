Header header          # timestamp of the first lidar in a frame
float64[] rot_end      # the estimated attitude (rotation matrix) at the end lidar point
float64[] pos_end      # the estimated position at the end lidar point (world frame)
float64[] vel_end      # the estimated velocity at the end lidar point (world frame)
float64[] bias_gyr     # gyroscope bias
float64[] bias_acc     # accelerator bias
float64[] gravity      # the estimated gravity acceleration
float64[] cov          # states covariance
# Pose6D[] IMUpose        # 6D pose at each imu measurements