#pragma once

#include <mutex>
#include <deque>
#include <string>
#include <array>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace mavtech_filter
{

class QuadEsekfNode final : public rclcpp::Node
{
public:
  explicit QuadEsekfNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  static constexpr int kErrDim = 15;

  struct State
  {
    Eigen::Vector3d p_WB = Eigen::Vector3d::Zero();  // position, NED world
    Eigen::Vector3d v_WB = Eigen::Vector3d::Zero();  // velocity, NED world
    Eigen::Quaterniond q_WB = Eigen::Quaterniond::Identity();  // body -> NED world
    Eigen::Vector3d b_g = Eigen::Vector3d::Zero();  // gyro bias, body frame
    Eigen::Vector3d b_a = Eigen::Vector3d::Zero();  // accel bias, body frame
  };

  struct Snapshot
  {
    rclcpp::Time stamp;
    State state;
    Eigen::Matrix<double, kErrDim, kErrDim> covariance;
  };

  // ROS callbacks.
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void vioCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publishTimerCallback();

  // ESEKF core.
  void initializeFromVio(const nav_msgs::msg::Odometry & msg);
  void propagateImu(const sensor_msgs::msg::Imu & msg);
  void updateVioPose(const nav_msgs::msg::Odometry & msg);
  void updateVioMeasurementAtCurrentState(const nav_msgs::msg::Odometry & msg);
  void saveStateSnapshot(const rclcpp::Time & stamp);
  void pruneBuffers(const rclcpp::Time & newest_stamp);
  int findSnapshotIndexBeforeOrAt(const rclcpp::Time & stamp) const;
  void eraseStateSnapshotsAfter(const rclcpp::Time & stamp);
  void applyErrorCorrection(const Eigen::Matrix<double, kErrDim, 1> & dx);
  void publishOdometry(const rclcpp::Time & stamp);

  // Math utilities.
  static Eigen::Matrix3d skew(const Eigen::Vector3d & v);
  static Eigen::Quaterniond smallAngleQuaternion(const Eigen::Vector3d & dtheta);
  static Eigen::Vector3d quaternionLog(const Eigen::Quaterniond & q);
  static double getVarianceOrDefault(const std::array<double, 36> & cov, int idx, double fallback);
  static Eigen::Matrix3d makeDiagonalNoise(double x, double y, double z);

  // Parameters.
  std::string imu_topic_;
  std::string vio_topic_;
  std::string odom_topic_;
  std::string world_frame_id_;
  std::string body_frame_id_;

  double publish_rate_hz_ = 30.0;
  double gravity_mps2_ = 9.80665;
  bool use_vio_velocity_ = false;
  bool publish_until_initialized_ = false;
  bool enable_delayed_vio_repropagation_ = true;
  double vio_delay_s_ = 0.0;
  double history_buffer_s_ = 2.0;

  // Continuous-time process noise densities.
  double gyro_noise_density_ = 1.0e-3;
  double accel_noise_density_ = 5.0e-2;
  double gyro_bias_random_walk_ = 1.0e-5;
  double accel_bias_random_walk_ = 1.0e-4;

  // Measurement fallback standard deviations.
  double vio_pos_std_ = 0.05;
  double vio_ori_std_rad_ = 0.03;
  double vio_vel_std_ = 0.20;

  // Transforms from raw 6-axis IMU messages to VIO/NED-aligned body frame.
  Eigen::Matrix3d R_imu_to_vio_ned_;
  Eigen::Quaterniond q_imu_to_vio_ned_;

  // State and covariance.
  State x_;
  Eigen::Matrix<double, kErrDim, kErrDim> P_ =
      Eigen::Matrix<double, kErrDim, kErrDim>::Identity();

  bool initialized_ = false;
  rclcpp::Time last_imu_stamp_;
  rclcpp::Time last_state_stamp_;

  std::deque<sensor_msgs::msg::Imu> imu_buffer_;
  std::deque<Snapshot> state_buffer_;

  std::mutex mtx_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace mavtech_filter