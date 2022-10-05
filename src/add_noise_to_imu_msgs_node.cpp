#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include <Eigen/Core>
#include <random>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
static constexpr double kDefaultAdisGyroscopeNoiseDensity =
    2.0 * 35.0 / 3600.0 / 180.0 * M_PI;
static constexpr double kDefaultAdisGyroscopeRandomWalk =
    2.0 * 4.0 / 3600.0 / 180.0 * M_PI;
static constexpr double kDefaultAdisGyroscopeBiasCorrelationTime =
    1.0e+3;
static constexpr double kDefaultAdisGyroscopeTurnOnBiasSigma =
    0.5 / 180.0 * M_PI;
static constexpr double kDefaultAdisAccelerometerNoiseDensity =
    2.0 * 2.0e-3;
static constexpr double kDefaultAdisAccelerometerRandomWalk =
    2.0 * 3.0e-3;
static constexpr double kDefaultAdisAccelerometerBiasCorrelationTime =
    300.0;
static constexpr double kDefaultAdisAccelerometerTurnOnBiasSigma =
    20.0e-3 * 9.8;
// Earth's gravity in Zurich (lat=+47.3667degN, lon=+8.5500degE, h=+500m, WGS84)
static constexpr double kDefaultGravityMagnitude = 9.8068;





class imu_parameters{
public:
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

  imu_parameters(float g_bct, float g_nd, float g_rw,float g_tobs,float a_bct ,float a_nd, float a_rw,float  a_tobs, float g)
   { // Constructor with parameters
    gyroscope_bias_correlation_time = g_bct;
    gyroscope_noise_density = g_nd;
    gyroscope_random_walk = g_rw;
    gyroscope_turn_on_bias_sigma=g_tobs;

    accelerometer_bias_correlation_time = a_bct;
    accelerometer_noise_density = a_nd;
    accelerometer_random_walk = a_rw;
    accelerometer_turn_on_bias_sigma = a_tobs;

    gravity_magnitude=g;
  }


};

imu_parameters imu_parameters_(kDefaultAdisGyroscopeBiasCorrelationTime,kDefaultAdisGyroscopeNoiseDensity,kDefaultAdisGyroscopeRandomWalk,kDefaultAdisGyroscopeTurnOnBiasSigma,kDefaultAdisAccelerometerBiasCorrelationTime,kDefaultAdisAccelerometerNoiseDensity,kDefaultAdisAccelerometerRandomWalk,kDefaultAdisAccelerometerTurnOnBiasSigma,kDefaultGravityMagnitude);

Eigen::Vector3d gyroscope_bias_;
Eigen::Vector3d accelerometer_bias_; 

std::default_random_engine random_generator_;
std::normal_distribution<double> standard_normal_distribution_;



template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}

float current_time;
float last_time_;

ros::Publisher imu_pub ;



void addNoise(Eigen::Vector3d* linear_acceleration,
                               Eigen::Vector3d* angular_velocity,
                               const double dt) {
  // CHECK(linear_acceleration);
  // CHECK(angular_velocity);




  assert(dt > 0.0);

  // Gyrosocpe
  double tau_g = imu_parameters_.gyroscope_bias_correlation_time;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_g_d = 1 / sqrt(dt) * imu_parameters_.gyroscope_noise_density;
  double sigma_b_g = imu_parameters_.gyroscope_random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_g_d =
      sqrt( - sigma_b_g * sigma_b_g * tau_g / 2.0 *
      (exp(-2.0 * dt / tau_g) - 1.0));
  // Compute state-transition.
  double phi_g_d = exp(-1.0 / tau_g * dt);
  // Simulate gyroscope noise processes and add them to the true angular rate.
  for (int i = 0; i < 3; ++i) {
    gyroscope_bias_[i] = phi_g_d * gyroscope_bias_[i] +
        sigma_b_g_d * standard_normal_distribution_(random_generator_);
    (*angular_velocity)[i] = (*angular_velocity)[i] +
        gyroscope_bias_[i] +
        sigma_g_d * standard_normal_distribution_(random_generator_);
  }

  // Accelerometer
  double tau_a = imu_parameters_.accelerometer_bias_correlation_time;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_a_d = 1 / sqrt(dt) * imu_parameters_.accelerometer_noise_density;
  double sigma_b_a = imu_parameters_.accelerometer_random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_a_d =
      sqrt( - sigma_b_a * sigma_b_a * tau_a / 2.0 *
      (exp(-2.0 * dt / tau_a) - 1.0));
  // Compute state-transition.
  double phi_a_d = exp(-1.0 / tau_a * dt);
  // Simulate accelerometer noise processes and add them to the true linear
  // acceleration.
  for (int i = 0; i < 3; ++i) {
    accelerometer_bias_[i] = phi_a_d * accelerometer_bias_[i] +
        sigma_b_a_d * standard_normal_distribution_(random_generator_);
    (*linear_acceleration)[i] = (*linear_acceleration)[i] +
        accelerometer_bias_[i] +
        sigma_a_d * standard_normal_distribution_(random_generator_);
  }

}

void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
{

  std::cout<<"---------------------start of message---------------------"<<std::endl;

  sensor_msgs::Imu imu_;
  imu_.header=msg->header;
  imu_.orientation=msg->orientation;
  imu_.linear_acceleration=msg->linear_acceleration;
  imu_.angular_velocity=msg->angular_velocity;


  current_time=ROS_TIME(msg);
  //std::cout<<"current_time is"<<current_time<<std::endl;

  double dt = (current_time - last_time_);

  //std::cout<<"last_time_ is"<<current_time<<std::endl;
  last_time_ = current_time;

  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;

  linear_acceleration<<imu_.linear_acceleration.x,imu_.linear_acceleration.y,imu_.linear_acceleration.z;
  angular_velocity<<imu_.angular_velocity.x,imu_.angular_velocity.y,imu_.angular_velocity.z;
  //std::cout<<"dt is"<<dt<<std::endl;
  std::cout<<"before linear_acceleration =" <<linear_acceleration<<std::endl;
  std::cout<<"before angular_velocity =" <<angular_velocity<<std::endl;
  addNoise(&linear_acceleration,&angular_velocity,dt);
  std::cout<<"after noise linear_acceleration =" <<linear_acceleration<<std::endl;
  std::cout<<"after noise angular_velocity =" <<angular_velocity<<std::endl;



  imu_.linear_acceleration.x=linear_acceleration(0);
  imu_.linear_acceleration.y=linear_acceleration(1);
  imu_.linear_acceleration.z=linear_acceleration(2);
  imu_.angular_velocity.x=angular_velocity(0);
  imu_.angular_velocity.y=angular_velocity(1);
  imu_.angular_velocity.z=angular_velocity(2);

  imu_.angular_velocity.z+=imu_parameters_.gravity_magnitude;
  //std::cout<<"heard!"<<std::endl;
  //ROS_INFO("I heard orientation x y z w: [%f %f %f %f]", msg->orientation.x, msg->orientation.y,msg->orientation.z,msg->orientation.w);

  imu_pub.publish(imu_);

}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

  standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);

  double sigma_bon_g = imu_parameters_.gyroscope_turn_on_bias_sigma;
  double sigma_bon_a = imu_parameters_.accelerometer_turn_on_bias_sigma;
  for (int i = 0; i < 3; ++i) {
      gyroscope_bias_[i] =
          sigma_bon_g * standard_normal_distribution_(random_generator_);
      accelerometer_bias_[i] =
          sigma_bon_a * standard_normal_distribution_(random_generator_);
  }
  imu_pub= n.advertise<sensor_msgs::Imu>("/imu_raw", 1);
  ros::Subscriber sub = n.subscribe("/gazebo_ros_imu", 1, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}