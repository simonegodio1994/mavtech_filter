\
#pragma once

#include <array>
#include <deque>
#include <mutex>
#include <string>

#include <Eigen/Dense>

#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace mavtech_filter
{

class QuadEsekfNode final : public rclcpp::Node
{
public:
  explicit QuadEsekfNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct State
  {
    Eigen::Vector3d p_WB = Eigen::Vector3d::Zero();            // position in NED/world
    Eigen::Vector3d v_WB = Eigen::Vector3d::Zero();            // velocity in NED/world
    Eigen::Quaterniond q_WB = Eigen::Quaterniond::Identity();  // body -> NED/world
  };

  struct ImuDelta
  {
    Eigen::Vector3d dp_W = Eigen::Vector3d::Zero();
    Eigen::Vector3d dv_W = Eigen::Vector3d::Zero();
    Eigen::Quaterniond dq = Eigen::Quaterniond::Identity();
    double dt = 0.0;
    bool valid = false;
  };

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void vioCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publishTimerCallback();

  void initializeFromVio(const nav_msgs::msg::Odometry & msg, const rclcpp::Time & measurement_stamp);
  void updateAnchorFromVio(const nav_msgs::msg::Odometry & msg, const rclcpp::Time & measurement_stamp);

  State predictFromAnchorTo(const rclcpp::Time & target_stamp) const;
  Eigen::Vector3d computeVioTwistRampDelta(double horizon_s) const;
  ImuDelta integrateImuDelta(
      const State & anchor,
      const rclcpp::Time & t0,
      const rclcpp::Time & t1) const;

  void publishOdometry(const State & state, const rclcpp::Time & stamp);

  rclcpp::Time getVioMeasurementStamp(const nav_msgs::msg::Odometry & msg) const;
  rclcpp::Time getPredictionTargetStamp() const;
  void pruneImuBuffer(const rclcpp::Time & newest_stamp);

  static Eigen::Quaterniond smallAngleQuaternion(const Eigen::Vector3d & dtheta);
  static Eigen::Vector3d transformImuVector(
      const Eigen::Matrix3d & R_imu_to_vio_ned,
      const geometry_msgs::msg::Vector3 & v);
  static bool isFiniteQuaternion(const Eigen::Quaterniond & q);
  static bool isFiniteVector(const Eigen::Vector3d & v);

  // Topics / frames.
  std::string imu_topic_;
  std::string vio_topic_;
  std::string odom_topic_;
  std::string world_frame_id_;
  std::string body_frame_id_;

  // Timing.
  double publish_rate_hz_ = 50.0;
  double vio_delay_s_ = 0.0;
  double max_prediction_horizon_s_ = 0.12;
  double extra_prediction_s_ = 0.0;
  double imu_buffer_s_ = 2.0;

  // Dynamics.
  double gravity_mps2_ = 9.80665;
  bool use_imu_gyro_for_orientation_prediction_ = true;
  bool use_imu_accel_for_velocity_prediction_ = true;
  bool use_imu_accel_for_position_prediction_ = true;

  // IMU contribution is intentionally bounded. This is not free inertial dead
  // reckoning; it is only a short look-ahead from the last VIO anchor.
  double imu_velocity_prediction_gain_ = 0.70;
  double imu_position_prediction_gain_ = 0.70;
  double max_imu_prediction_horizon_s_ = 0.08;
  double max_imu_velocity_delta_xy_mps_ = 0.12;
  double max_imu_velocity_delta_z_mps_ = 0.16;
  double max_imu_position_delta_xy_m_ = 0.025;
  double max_imu_position_delta_z_m_ = 0.035;

  double imu_accel_lpf_tau_s_ = 0.04;
  double imu_accel_deadband_mps2_ = 0.08;

  // VIO twist ramp prediction.
  // This removes takeoff stair-step on Vz without relying too much on
  // accelerometer integration.
  bool use_vio_twist_ramp_prediction_ = true;
  bool use_vio_twist_ramp_for_z_only_ = true;
  double vio_twist_ramp_gain_ = 0.85;
  double vio_twist_ramp_accel_lpf_tau_s_ = 0.22;
  double max_vio_twist_ramp_horizon_s_ = 0.10;
  double max_vio_twist_ramp_delta_xy_mps_ = 0.00;
  double max_vio_twist_ramp_delta_z_mps_ = 0.16;

  // Optional axis-specific gain for the IMU contribution to velocity.
  // For takeoff, keep IMU contribution moderate and let VIO ramp prediction
  // shape the vertical velocity.
  double imu_velocity_prediction_gain_xy_ = 0.15;
  double imu_velocity_prediction_gain_z_ = 0.25;

  // Velocity anchor.
  bool use_vio_twist_velocity_ = true;
  bool smooth_vio_anchor_velocity_ = false;
  double vio_velocity_alpha_ = 1.0;

  // Final output smoothing. Keep disabled by default for delay compensation.
  bool enable_pose_output_smoothing_ = false;
  bool enable_twist_output_smoothing_ = false;
  double pose_output_smoothing_tau_s_ = 0.04;
  double twist_output_smoothing_tau_s_ = 0.025;
  double max_output_position_error_m_ = 0.05;

  // Raw IMU -> VIO/NED-aligned body frame.
  Eigen::Matrix3d R_imu_to_vio_ned_;

  State anchor_;
  State previous_vio_;

  Eigen::Vector3d last_vio_twist_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d prev_vio_twist_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d vio_twist_accel_lpf_ = Eigen::Vector3d::Zero();
  bool has_last_vio_twist_ = false;
  bool has_vio_twist_accel_lpf_ = false;
  double last_vio_twist_dt_s_ = 0.10;

  bool has_previous_vio_ = false;

  State output_state_;
  bool output_initialized_ = false;
  rclcpp::Time last_output_stamp_;

  rclcpp::Time anchor_stamp_;
  rclcpp::Time previous_vio_stamp_;
  rclcpp::Time latest_imu_stamp_;

  std::array<double, 36> latest_pose_covariance_{};
  std::array<double, 36> latest_twist_covariance_{};

  bool initialized_ = false;
  std::deque<sensor_msgs::msg::Imu> imu_buffer_;

  mutable std::mutex mtx_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace mavtech_filter