# the preintegrated Lidar states at the time of IMU measurements in a frame
float64  offset_time # the offset time of IMU measurement w.r.t the first lidar point
float64[3] acc       # the preintegrated total acceleration (global frame) at the Lidar origin
float64[3] gyr       # the unbiased angular velocity (body frame) at the Lidar origin
float64[3] vel       # the preintegrated velocity (global frame) at the Lidar origin
float64[3] pos       # the preintegrated position (global frame) at the Lidar origin
float64[9] rot       # the preintegrated rotation (global frame) at the Lidar origin