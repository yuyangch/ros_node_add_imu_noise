# ros_node_add_imu_noise
this is a ros package that add noise to ground truth imu message (such as the one outputs from gazebo_ros_imu.cpp plugin, adopted from gazebo_imu_plugin.cpp included in PX4's sitl_gazebo. the noise parameters available are:
```
  /// Gyroscope noise density (two-sided spectrum) [rad/s/sqrt(Hz)]
  double gyroscope_noise_density;
  /// Gyroscope bias random walk [rad/s/s/sqrt(Hz)]
  double gyroscope_random_walk;
  /// Gyroscope bias correlation time constant [s]
  double gyroscope_bias_correlation_time;
  /// Gyroscope turn on bias standard deviation [rad/s]
  double gyroscope_turn_on_bias_sigma;
  /// Accelerometer noise density (two-sided spectrum) [m/s^2/sqrt(Hz)]
  double accelerometer_noise_density;
  /// Accelerometer bias random walk. [m/s^2/s/sqrt(Hz)]
  double accelerometer_random_walk;
  /// Accelerometer bias correlation time constant [s]
  double accelerometer_bias_correlation_time;
  /// Accelerometer turn on bias standard deviation [m/s^2]
  double accelerometer_turn_on_bias_sigma;
  /// Norm of the gravitational acceleration [m/s^2]
  double gravity_magnitude;
```


The gazebo_imu_plugin.cpp is a gazebo plugin. It is developed by the following authors 
```
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 ```

 An explanation of the nose parameters can be found at 
 https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model


 with the following references:

 [1] "IEEE Standard Specification Format Guide and Test Procedure for Single-Axis Interferometric Fiber Optic Gyros", IEEE Std 952-1997, 1998 http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=660628&isnumber=14457

[2] "Indirect Kalman Filter for 3D Attitude Estimation", Nikolas Trawny and Stergios I. Roumeliotis, MARS Lab Tech. Report Nr. 2005-002, Rev. 57 http://www-users.cs.umn.edu/~trawny/Publications/Quaternions_3D.pdf

[3] "Sigma-Point Kalman Filtering for Integrated GPS and Inertial Navigation", John L. Crassidis, University at Buffalo http://www.acsu.buffalo.edu/~johnc/gpsins_gnc05.pdf