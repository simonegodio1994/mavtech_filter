#include "mavtech_filter/esekf_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <deque>

namespace mavtech_filter
{

namespace
{
constexpr double kMinDt = 1.0e-5;
constexpr double kMaxDt = 0.05;  // Protect against bag jumps/stalls.
constexpr double kDegenerateCovThreshold = 1.0e-12;
}  // namespace

QuadEsekfNode::QuadEsekfNode(const rclcpp::NodeOptions & options)
: Node("esekf_node", options)
{
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
  vio_topic_ = declare_parameter<std::string>("vio_topic", "/okvis/okvis_odometry_ned");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/mavtech_filter/odom_ned");

  world_frame_id_ = declare_parameter<std::string>("world_frame_id", "world_ned");
  body_frame_id_ = declare_parameter<std::string>("body_frame_id", "base_link");

  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
  gravity_mps2_ = declare_parameter<double>("gravity_mps2", 9.80665);
  use_vio_velocity_ = declare_parameter<bool>("use_vio_velocity", false);
  publish_until_initialized_ = declare_parameter<bool>("publish_until_initialized", false);
  enable_delayed_vio_repropagation_ =
      declare_parameter<bool>("enable_delayed_vio_repropagation", true);
  vio_delay_s_ = declare_parameter<double>("vio_delay_s", 0.0);
  history_buffer_s_ = declare_parameter<double>("history_buffer_s", 2.0);

  gyro_noise_density_ = declare_parameter<double>("gyro_noise_density", 1.0e-3);
  accel_noise_density_ = declare_parameter<double>("accel_noise_density", 5.0e-2);
  gyro_bias_random_walk_ = declare_parameter<double>("gyro_bias_random_walk", 1.0e-5);
  accel_bias_random_walk_ = declare_parameter<double>("accel_bias_random_walk", 1.0e-4);

  vio_pos_std_ = declare_parameter<double>("vio_pos_std", 0.05);
  vio_ori_std_rad_ = declare_parameter<double>("vio_ori_std_rad", 0.03);
  vio_vel_std_ = declare_parameter<double>("vio_vel_std", 0.20);

  R_imu_to_vio_ned_ =
      (Eigen::Matrix3d() << 0.0, 1.0, 0.0,
                           1.0, 0.0, 0.0,
                           0.0, 0.0, -1.0)
          .finished();

  q_imu_to_vio_ned_ = Eigen::Quaterniond(
      0.0,
      M_SQRT1_2,
      M_SQRT1_2,
      0.0);
  q_imu_to_vio_ned_.normalize();

  // Conservative initial uncertainty.
  P_.setZero();
  P_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1.0;    // position
  P_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1.0;    // velocity
  P_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 0.25;   // attitude
  P_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 0.01;   // gyro bias
  P_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 0.10; // accel bias

  rclcpp::SensorDataQoS imu_qos;
  imu_qos.keep_last(1500);
  imu_qos.best_effort();

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, imu_qos,
      std::bind(&QuadEsekfNode::imuCallback, this, std::placeholders::_1));

  rclcpp::QoS vio_odom_qos(rclcpp::KeepLast(1));
  vio_odom_qos.best_effort();
  vio_odom_qos.durability_volatile();

  vio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      vio_topic_, vio_odom_qos,
      std::bind(&QuadEsekfNode::vioCallback, this, std::placeholders::_1));

  rclcpp::QoS fused_odom_qos(rclcpp::KeepLast(1));
  fused_odom_qos.best_effort();
  fused_odom_qos.durability_volatile();

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, fused_odom_qos);

  const double safe_rate = std::max(1.0, publish_rate_hz_);
  publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / safe_rate)),
      std::bind(&QuadEsekfNode::publishTimerCallback, this));

  RCLCPP_INFO(get_logger(), "esekf_node started");
  RCLCPP_INFO(get_logger(), "  IMU topic: %s", imu_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  VIO topic: %s", vio_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  output topic: %s @ %.1f Hz", odom_topic_.c_str(), safe_rate);
  RCLCPP_INFO(get_logger(), "  convention: NED world, gravity = [0, 0, +%.5f] m/s^2", gravity_mps2_);
  RCLCPP_INFO(
      get_logger(),
      "  delayed VIO repropagation: %s, vio_delay_s=%.3f, history_buffer_s=%.2f",
      enable_delayed_vio_repropagation_ ? "enabled" : "disabled",
      vio_delay_s_,
      history_buffer_s_);
}

void QuadEsekfNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);

  if (!initialized_) {
    return;
  }

  const rclcpp::Time stamp(msg->header.stamp);
  if (stamp.nanoseconds() == 0) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Received IMU message with zero timestamp; skipping");
    return;
  }

  // Keep the raw IMU stream so that delayed VIO corrections can be applied
  // in the past and then repropagated back to the newest IMU time.
  if (imu_buffer_.empty() ||
      rclcpp::Time(imu_buffer_.back().header.stamp) < stamp) {
    imu_buffer_.push_back(*msg);
  } else {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Received non-monotonic IMU timestamp; skipping buffer insertion");
  }

  propagateImu(*msg);
  saveStateSnapshot(last_state_stamp_);
  pruneBuffers(last_state_stamp_);
}

void QuadEsekfNode::vioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);

  if (!initialized_) {
    initializeFromVio(*msg);
    return;
  }

  updateVioPose(*msg);
}

void QuadEsekfNode::publishTimerCallback()
{
  std::lock_guard<std::mutex> lock(mtx_);

  if (!initialized_ && !publish_until_initialized_) {
    return;
  }

  const rclcpp::Time stamp =
      initialized_ ? last_state_stamp_ : this->get_clock()->now();

  publishOdometry(stamp);
}

void QuadEsekfNode::initializeFromVio(const nav_msgs::msg::Odometry & msg)
{
  x_.p_WB = Eigen::Vector3d(
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      msg.pose.pose.position.z);

  x_.v_WB = Eigen::Vector3d(
      msg.twist.twist.linear.x,
      msg.twist.twist.linear.y,
      msg.twist.twist.linear.z);

  x_.q_WB = Eigen::Quaterniond(
      msg.pose.pose.orientation.w,
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z);
  x_.q_WB.normalize();

  x_.b_g.setZero();
  x_.b_a.setZero();

  last_state_stamp_ = rclcpp::Time(msg.header.stamp);
  last_imu_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

  imu_buffer_.clear();
  state_buffer_.clear();

  initialized_ = true;
  saveStateSnapshot(last_state_stamp_);

  RCLCPP_INFO(
      get_logger(),
      "ESEKF initialized from VIO at t=%.9f, p=[%.3f %.3f %.3f]",
      last_state_stamp_.seconds(), x_.p_WB.x(), x_.p_WB.y(), x_.p_WB.z());
}

void QuadEsekfNode::propagateImu(const sensor_msgs::msg::Imu & msg)
{
  const rclcpp::Time stamp(msg.header.stamp);

  if (last_imu_stamp_.nanoseconds() == 0) {
    last_imu_stamp_ = stamp;
    last_state_stamp_ = stamp;
    return;
  }

  double dt = (stamp - last_imu_stamp_).seconds();
  last_imu_stamp_ = stamp;

  if (!std::isfinite(dt) || dt < kMinDt) {
    return;
  }

  if (dt > kMaxDt) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Large IMU dt %.4f s clamped to %.4f s", dt, kMaxDt);
    dt = kMaxDt;
  }

  Eigen::Vector3d omega_raw(
      msg.angular_velocity.x,
      msg.angular_velocity.y,
      msg.angular_velocity.z);

  Eigen::Vector3d accel_raw(
      msg.linear_acceleration.x,
      msg.linear_acceleration.y,
      msg.linear_acceleration.z);

  // Raw IMU 6-axis vectors -> VIO/NED-aligned body frame.
  const Eigen::Vector3d omega = R_imu_to_vio_ned_ * omega_raw;
  const Eigen::Vector3d accel = R_imu_to_vio_ned_ * accel_raw;

  const Eigen::Vector3d omega_unbiased = omega - x_.b_g;
  const Eigen::Vector3d accel_unbiased = accel - x_.b_a;

  const Eigen::Matrix3d R_WB = x_.q_WB.toRotationMatrix();
  const Eigen::Vector3d g_W(0.0, 0.0, gravity_mps2_);

  // Nominal propagation.
  const Eigen::Vector3d a_W = R_WB * accel_unbiased + g_W;
  x_.p_WB += x_.v_WB * dt + 0.5 * a_W * dt * dt;
  x_.v_WB += a_W * dt;
  x_.q_WB = (x_.q_WB * smallAngleQuaternion(omega_unbiased * dt)).normalized();

  // Error-state propagation.
  // Error vector: [dp, dv, dtheta, dbg, dba].
  Eigen::Matrix<double, kErrDim, kErrDim> F =
      Eigen::Matrix<double, kErrDim, kErrDim>::Identity();

  F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  F.block<3, 3>(3, 6) = -R_WB * skew(accel_unbiased) * dt;
  F.block<3, 3>(3, 12) = -R_WB * dt;
  F.block<3, 3>(6, 6) =
      Eigen::Matrix3d::Identity() - skew(omega_unbiased) * dt;
  F.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity() * dt;

  Eigen::Matrix<double, kErrDim, kErrDim> Q =
      Eigen::Matrix<double, kErrDim, kErrDim>::Zero();

  const double gyro_var = gyro_noise_density_ * gyro_noise_density_;
  const double accel_var = accel_noise_density_ * accel_noise_density_;
  const double gyro_bias_var = gyro_bias_random_walk_ * gyro_bias_random_walk_;
  const double accel_bias_var = accel_bias_random_walk_ * accel_bias_random_walk_;

  Q.block<3, 3>(3, 3) = R_WB * Eigen::Matrix3d::Identity() * accel_var * R_WB.transpose() * dt * dt;
  Q.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * gyro_var * dt * dt;
  Q.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * gyro_bias_var * dt;
  Q.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * accel_bias_var * dt;

  P_ = F * P_ * F.transpose() + Q;
  P_ = 0.5 * (P_ + P_.transpose());

  last_state_stamp_ = stamp;
}

void QuadEsekfNode::updateVioPose(const nav_msgs::msg::Odometry & msg)
{
  rclcpp::Time measurement_stamp(msg.header.stamp);

  if (measurement_stamp.nanoseconds() == 0) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Received VIO odometry with zero timestamp; skipping update");
    return;
  }

  // If the VIO odometry timestamp represents publication time rather than the
  // original image/estimation time, compensate a known fixed delay here.
  // If your VIO header.stamp is already the image/state time, keep vio_delay_s=0.
  if (vio_delay_s_ > 0.0) {
    measurement_stamp =
        measurement_stamp - rclcpp::Duration::from_seconds(vio_delay_s_);
  }

  if (!enable_delayed_vio_repropagation_) {
    updateVioMeasurementAtCurrentState(msg);
    return;
  }

  if (state_buffer_.empty() || imu_buffer_.empty()) {
    updateVioMeasurementAtCurrentState(msg);
    saveStateSnapshot(last_state_stamp_);
    return;
  }

  const rclcpp::Time newest_state_stamp = last_state_stamp_;

  // If the measurement is newer than the current propagated state, just update
  // the current state. This can happen at startup or with unusual timestamps.
  if (measurement_stamp >= newest_state_stamp) {
    updateVioMeasurementAtCurrentState(msg);
    saveStateSnapshot(last_state_stamp_);
    return;
  }

  const int snapshot_idx = findSnapshotIndexBeforeOrAt(measurement_stamp);
  if (snapshot_idx < 0) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "VIO measurement too old for history buffer: vio_t=%.6f oldest_state_t=%.6f",
        measurement_stamp.seconds(),
        state_buffer_.empty() ? 0.0 : state_buffer_.front().stamp.seconds());
    return;
  }

  const Snapshot snapshot = state_buffer_[static_cast<size_t>(snapshot_idx)];

  // Restore the filter at the closest historical state before/at the VIO
  // measurement time, apply the VIO correction there, then repropagate all IMU
  // samples that occurred after that historical state.
  x_ = snapshot.state;
  P_ = snapshot.covariance;
  last_state_stamp_ = snapshot.stamp;
  last_imu_stamp_ = snapshot.stamp;

  updateVioMeasurementAtCurrentState(msg);

  // Drop snapshots after the correction time. They are no longer valid because
  // the state history has changed after the delayed update.
  eraseStateSnapshotsAfter(snapshot.stamp);
  saveStateSnapshot(last_state_stamp_);

  size_t repropagated_imu_count = 0;
  for (const auto & imu_msg : imu_buffer_) {
    const rclcpp::Time imu_stamp(imu_msg.header.stamp);
    if (imu_stamp <= snapshot.stamp) {
      continue;
    }

    // Repropagate only up to the newest state we had before applying the delayed
    // update. Newer IMUs will arrive normally through imuCallback.
    if (imu_stamp > newest_state_stamp) {
      break;
    }

    propagateImu(imu_msg);
    saveStateSnapshot(last_state_stamp_);
    ++repropagated_imu_count;
  }

  pruneBuffers(last_state_stamp_);

  RCLCPP_DEBUG(
      get_logger(),
      "Delayed VIO update applied at %.6f using snapshot %.6f and repropagated %zu IMUs to %.6f",
      measurement_stamp.seconds(),
      snapshot.stamp.seconds(),
      repropagated_imu_count,
      last_state_stamp_.seconds());
}

void QuadEsekfNode::updateVioMeasurementAtCurrentState(const nav_msgs::msg::Odometry & msg)
{
  Eigen::Vector3d p_meas(
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      msg.pose.pose.position.z);

  Eigen::Quaterniond q_meas(
      msg.pose.pose.orientation.w,
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z);
  q_meas.normalize();

  const int meas_dim = use_vio_velocity_ ? 9 : 6;

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(meas_dim, kErrDim);
  Eigen::VectorXd r = Eigen::VectorXd::Zero(meas_dim);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(meas_dim, meas_dim);

  // Position residual.
  r.segment<3>(0) = p_meas - x_.p_WB;
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

  const double px_var = getVarianceOrDefault(msg.pose.covariance, 0, vio_pos_std_ * vio_pos_std_);
  const double py_var = getVarianceOrDefault(msg.pose.covariance, 7, vio_pos_std_ * vio_pos_std_);
  const double pz_var = getVarianceOrDefault(msg.pose.covariance, 14, vio_pos_std_ * vio_pos_std_);
  R.block<3, 3>(0, 0) = makeDiagonalNoise(px_var, py_var, pz_var);

  // Orientation residual. Compatible with left-multiplicative correction:
  // q_corrected = Exp(dtheta) * q_nominal.
  Eigen::Quaterniond dq = q_meas * x_.q_WB.conjugate();
  if (dq.w() < 0.0) {
    dq.coeffs() *= -1.0;
  }

  r.segment<3>(3) = quaternionLog(dq);
  H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();

  const double ox_var = getVarianceOrDefault(msg.pose.covariance, 21, vio_ori_std_rad_ * vio_ori_std_rad_);
  const double oy_var = getVarianceOrDefault(msg.pose.covariance, 28, vio_ori_std_rad_ * vio_ori_std_rad_);
  const double oz_var = getVarianceOrDefault(msg.pose.covariance, 35, vio_ori_std_rad_ * vio_ori_std_rad_);
  R.block<3, 3>(3, 3) = makeDiagonalNoise(ox_var, oy_var, oz_var);

  if (use_vio_velocity_) {
    Eigen::Vector3d v_meas(
        msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.linear.z);

    r.segment<3>(6) = v_meas - x_.v_WB;
    H.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity();

    const double vx_var = getVarianceOrDefault(msg.twist.covariance, 0, vio_vel_std_ * vio_vel_std_);
    const double vy_var = getVarianceOrDefault(msg.twist.covariance, 7, vio_vel_std_ * vio_vel_std_);
    const double vz_var = getVarianceOrDefault(msg.twist.covariance, 14, vio_vel_std_ * vio_vel_std_);
    R.block<3, 3>(6, 6) = makeDiagonalNoise(vx_var, vy_var, vz_var);
  }

  const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  const Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

  const Eigen::Matrix<double, kErrDim, 1> dx = K * r;

  const Eigen::Matrix<double, kErrDim, kErrDim> I =
      Eigen::Matrix<double, kErrDim, kErrDim>::Identity();

  // Joseph form for numerical stability.
  P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * R * K.transpose();
  P_ = 0.5 * (P_ + P_.transpose());

  applyErrorCorrection(dx);
}


void QuadEsekfNode::saveStateSnapshot(const rclcpp::Time & stamp)
{
  if (stamp.nanoseconds() == 0) {
    return;
  }

  if (!state_buffer_.empty() && state_buffer_.back().stamp == stamp) {
    state_buffer_.back().state = x_;
    state_buffer_.back().covariance = P_;
    return;
  }

  if (!state_buffer_.empty() && state_buffer_.back().stamp > stamp) {
    // Keep the buffer time-ordered. This should only happen after a restore;
    // remove invalid future snapshots and append the corrected one.
    eraseStateSnapshotsAfter(stamp);
  }

  Snapshot snapshot;
  snapshot.stamp = stamp;
  snapshot.state = x_;
  snapshot.covariance = P_;
  state_buffer_.push_back(snapshot);
}

void QuadEsekfNode::pruneBuffers(const rclcpp::Time & newest_stamp)
{
  if (newest_stamp.nanoseconds() == 0 || history_buffer_s_ <= 0.0) {
    return;
  }

  const rclcpp::Time oldest_allowed =
      newest_stamp - rclcpp::Duration::from_seconds(history_buffer_s_);

  while (!imu_buffer_.empty() &&
         rclcpp::Time(imu_buffer_.front().header.stamp) < oldest_allowed) {
    imu_buffer_.pop_front();
  }

  while (state_buffer_.size() > 2 && state_buffer_.front().stamp < oldest_allowed) {
    state_buffer_.pop_front();
  }
}

int QuadEsekfNode::findSnapshotIndexBeforeOrAt(const rclcpp::Time & stamp) const
{
  if (state_buffer_.empty()) {
    return -1;
  }

  int best_idx = -1;
  for (size_t i = 0; i < state_buffer_.size(); ++i) {
    if (state_buffer_[i].stamp <= stamp) {
      best_idx = static_cast<int>(i);
    } else {
      break;
    }
  }

  return best_idx;
}

void QuadEsekfNode::eraseStateSnapshotsAfter(const rclcpp::Time & stamp)
{
  while (!state_buffer_.empty() && state_buffer_.back().stamp > stamp) {
    state_buffer_.pop_back();
  }
}


void QuadEsekfNode::applyErrorCorrection(
    const Eigen::Matrix<double, kErrDim, 1> & dx)
{
  x_.p_WB += dx.segment<3>(0);
  x_.v_WB += dx.segment<3>(3);
  x_.q_WB = (smallAngleQuaternion(dx.segment<3>(6)) * x_.q_WB).normalized();
  x_.b_g += dx.segment<3>(9);
  x_.b_a += dx.segment<3>(12);

  // Reset Jacobian is approximated as identity for this first implementation.
}

void QuadEsekfNode::publishOdometry(const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = world_frame_id_;
  odom.child_frame_id = body_frame_id_;

  odom.pose.pose.position.x = x_.p_WB.x();
  odom.pose.pose.position.y = x_.p_WB.y();
  odom.pose.pose.position.z = x_.p_WB.z();

  odom.pose.pose.orientation.w = x_.q_WB.w();
  odom.pose.pose.orientation.x = x_.q_WB.x();
  odom.pose.pose.orientation.y = x_.q_WB.y();
  odom.pose.pose.orientation.z = x_.q_WB.z();

  odom.twist.twist.linear.x = x_.v_WB.x();
  odom.twist.twist.linear.y = x_.v_WB.y();
  odom.twist.twist.linear.z = x_.v_WB.z();

  // Fill pose covariance from error-state covariance.
  for (int i = 0; i < 36; ++i) {
    odom.pose.covariance[i] = 0.0;
    odom.twist.covariance[i] = 0.0;
  }

  // ROS covariance order: x, y, z, rot_x, rot_y, rot_z.
  odom.pose.covariance[0] = P_(0, 0);
  odom.pose.covariance[7] = P_(1, 1);
  odom.pose.covariance[14] = P_(2, 2);
  odom.pose.covariance[21] = P_(6, 6);
  odom.pose.covariance[28] = P_(7, 7);
  odom.pose.covariance[35] = P_(8, 8);

  odom.twist.covariance[0] = P_(3, 3);
  odom.twist.covariance[7] = P_(4, 4);
  odom.twist.covariance[14] = P_(5, 5);

  odom_pub_->publish(odom);
}

Eigen::Matrix3d QuadEsekfNode::skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d S;
  S << 0.0, -v.z(), v.y(),
       v.z(), 0.0, -v.x(),
       -v.y(), v.x(), 0.0;
  return S;
}

Eigen::Quaterniond QuadEsekfNode::smallAngleQuaternion(const Eigen::Vector3d & dtheta)
{
  const double angle = dtheta.norm();
  if (angle < 1.0e-12) {
    return Eigen::Quaterniond(
        1.0,
        0.5 * dtheta.x(),
        0.5 * dtheta.y(),
        0.5 * dtheta.z()).normalized();
  }

  const Eigen::Vector3d axis = dtheta / angle;
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)).normalized();
}

Eigen::Vector3d QuadEsekfNode::quaternionLog(const Eigen::Quaterniond & q_in)
{
  Eigen::Quaterniond q = q_in.normalized();
  if (q.w() < 0.0) {
    q.coeffs() *= -1.0;
  }

  const Eigen::Vector3d v(q.x(), q.y(), q.z());
  const double v_norm = v.norm();

  if (v_norm < 1.0e-12) {
    return 2.0 * v;
  }

  const double angle = 2.0 * std::atan2(v_norm, q.w());
  return angle * v / v_norm;
}

double QuadEsekfNode::getVarianceOrDefault(
    const std::array<double, 36> & cov,
    int idx,
    double fallback)
{
  if (idx < 0 || idx >= 36) {
    return fallback;
  }

  const double value = cov[static_cast<size_t>(idx)];
  if (!std::isfinite(value) || value < kDegenerateCovThreshold) {
    return fallback;
  }

  return value;
}

Eigen::Matrix3d QuadEsekfNode::makeDiagonalNoise(double x, double y, double z)
{
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  R(0, 0) = std::max(x, kDegenerateCovThreshold);
  R(1, 1) = std::max(y, kDegenerateCovThreshold);
  R(2, 2) = std::max(z, kDegenerateCovThreshold);
  return R;
}

}  // namespace mavtech_filter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mavtech_filter::QuadEsekfNode>());
  rclcpp::shutdown();
  return 0;
}