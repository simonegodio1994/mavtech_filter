\
#include "mavtech_filter/esekf_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mavtech_filter
{

namespace
{
constexpr double kMinDt = 1.0e-6;
constexpr double kMaxImuDt = 0.02;
}  // namespace

QuadEsekfNode::QuadEsekfNode(const rclcpp::NodeOptions & options)
: Node("esekf_node", options)
{
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
  vio_topic_ = declare_parameter<std::string>("vio_topic", "/okvis/okvis_odometry_ned");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/mavtech_filter/odom_ned");

  world_frame_id_ = declare_parameter<std::string>("world_frame_id", "world_ned");
  body_frame_id_ = declare_parameter<std::string>("body_frame_id", "base_link");

  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 200.0);
  vio_delay_s_ = declare_parameter<double>("vio_delay_s", 0.0);
  max_prediction_horizon_s_ = declare_parameter<double>("max_prediction_horizon_s", 0.12);
  extra_prediction_s_ = declare_parameter<double>("extra_prediction_s", 0.0);
  imu_buffer_s_ = declare_parameter<double>("imu_buffer_s", 2.0);

  gravity_mps2_ = declare_parameter<double>("gravity_mps2", 9.80665);
  use_imu_gyro_for_orientation_prediction_ =
      declare_parameter<bool>("use_imu_gyro_for_orientation_prediction", true);
  use_imu_accel_for_velocity_prediction_ =
      declare_parameter<bool>("use_imu_accel_for_velocity_prediction", true);
  use_imu_accel_for_position_prediction_ =
      declare_parameter<bool>("use_imu_accel_for_position_prediction", true);

  imu_velocity_prediction_gain_ =
      declare_parameter<double>("imu_velocity_prediction_gain", 0.70);
  imu_position_prediction_gain_ =
      declare_parameter<double>("imu_position_prediction_gain", 0.70);

  imu_velocity_prediction_gain_xy_ =
      declare_parameter<double>("imu_velocity_prediction_gain_xy", 0.15);
  imu_velocity_prediction_gain_z_ =
      declare_parameter<double>("imu_velocity_prediction_gain_z", 0.25);
  max_imu_prediction_horizon_s_ =
      declare_parameter<double>("max_imu_prediction_horizon_s", 0.08);
  max_imu_velocity_delta_xy_mps_ =
      declare_parameter<double>("max_imu_velocity_delta_xy_mps", 0.12);
  max_imu_velocity_delta_z_mps_ =
      declare_parameter<double>("max_imu_velocity_delta_z_mps", 0.16);
  max_imu_position_delta_xy_m_ =
      declare_parameter<double>("max_imu_position_delta_xy_m", 0.025);
  max_imu_position_delta_z_m_ =
      declare_parameter<double>("max_imu_position_delta_z_m", 0.035);

  imu_accel_lpf_tau_s_ =
      declare_parameter<double>("imu_accel_lpf_tau_s", 0.04);
  imu_accel_deadband_mps2_ =
      declare_parameter<double>("imu_accel_deadband_mps2", 0.08);

  use_vio_twist_ramp_prediction_ =
      declare_parameter<bool>("use_vio_twist_ramp_prediction", true);
  use_vio_twist_ramp_for_z_only_ =
      declare_parameter<bool>("use_vio_twist_ramp_for_z_only", true);
  vio_twist_ramp_gain_ =
      declare_parameter<double>("vio_twist_ramp_gain", 0.85);
  vio_twist_ramp_accel_lpf_tau_s_ =
      declare_parameter<double>("vio_twist_ramp_accel_lpf_tau_s", 0.22);
  max_vio_twist_ramp_horizon_s_ =
      declare_parameter<double>("max_vio_twist_ramp_horizon_s", 0.10);
  max_vio_twist_ramp_delta_xy_mps_ =
      declare_parameter<double>("max_vio_twist_ramp_delta_xy_mps", 0.00);
  max_vio_twist_ramp_delta_z_mps_ =
      declare_parameter<double>("max_vio_twist_ramp_delta_z_mps", 0.16);

  use_vio_twist_velocity_ =
      declare_parameter<bool>("use_vio_twist_velocity", true);
  smooth_vio_anchor_velocity_ =
      declare_parameter<bool>("smooth_vio_anchor_velocity", false);
  vio_velocity_alpha_ = declare_parameter<double>("vio_velocity_alpha", 1.0);

  // Backward compatible name used in older configs.
  if (!has_parameter("use_vio_twist_velocity")) {
    use_vio_twist_velocity_ =
        declare_parameter<bool>("use_vio_velocity", use_vio_twist_velocity_);
  }

  enable_pose_output_smoothing_ =
      declare_parameter<bool>("enable_pose_output_smoothing", false);
  enable_twist_output_smoothing_ =
      declare_parameter<bool>("enable_twist_output_smoothing", false);
  pose_output_smoothing_tau_s_ =
      declare_parameter<double>("pose_output_smoothing_tau_s", 0.04);
  twist_output_smoothing_tau_s_ =
      declare_parameter<double>("twist_output_smoothing_tau_s", 0.025);
  max_output_position_error_m_ =
      declare_parameter<double>("max_output_position_error_m", 0.05);

  publish_rate_hz_ = std::max(1.0, publish_rate_hz_);
  max_prediction_horizon_s_ = std::max(0.0, max_prediction_horizon_s_);
  extra_prediction_s_ = std::max(0.0, extra_prediction_s_);
  imu_buffer_s_ = std::max(0.1, imu_buffer_s_);

  imu_velocity_prediction_gain_ = std::clamp(imu_velocity_prediction_gain_, 0.0, 1.0);
  imu_position_prediction_gain_ = std::clamp(imu_position_prediction_gain_, 0.0, 1.0);
  max_imu_prediction_horizon_s_ = std::max(0.0, max_imu_prediction_horizon_s_);
  max_imu_velocity_delta_xy_mps_ = std::max(0.0, max_imu_velocity_delta_xy_mps_);
  max_imu_velocity_delta_z_mps_ = std::max(0.0, max_imu_velocity_delta_z_mps_);
  max_imu_position_delta_xy_m_ = std::max(0.0, max_imu_position_delta_xy_m_);
  max_imu_position_delta_z_m_ = std::max(0.0, max_imu_position_delta_z_m_);
  imu_accel_lpf_tau_s_ = std::max(1.0e-4, imu_accel_lpf_tau_s_);
  imu_accel_deadband_mps2_ = std::max(0.0, imu_accel_deadband_mps2_);

  vio_twist_ramp_gain_ = std::clamp(vio_twist_ramp_gain_, 0.0, 1.5);
  vio_twist_ramp_accel_lpf_tau_s_ = std::max(1.0e-4, vio_twist_ramp_accel_lpf_tau_s_);
  max_vio_twist_ramp_horizon_s_ = std::max(0.0, max_vio_twist_ramp_horizon_s_);
  max_vio_twist_ramp_delta_xy_mps_ = std::max(0.0, max_vio_twist_ramp_delta_xy_mps_);
  max_vio_twist_ramp_delta_z_mps_ = std::max(0.0, max_vio_twist_ramp_delta_z_mps_);
  vio_velocity_alpha_ = std::clamp(vio_velocity_alpha_, 0.0, 1.0);
  pose_output_smoothing_tau_s_ = std::max(1.0e-4, pose_output_smoothing_tau_s_);
  twist_output_smoothing_tau_s_ = std::max(1.0e-4, twist_output_smoothing_tau_s_);

  R_imu_to_vio_ned_ =
      (Eigen::Matrix3d() << 0.0, 1.0, 0.0,
                           1.0, 0.0, 0.0,
                           0.0, 0.0, -1.0)
          .finished();

  latest_pose_covariance_.fill(0.0);
  latest_twist_covariance_.fill(0.0);

  rclcpp::SensorDataQoS imu_qos;
  imu_qos.keep_last(1500);
  imu_qos.best_effort();

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, imu_qos,
      std::bind(&QuadEsekfNode::imuCallback, this, std::placeholders::_1));

  rclcpp::QoS vio_qos(rclcpp::KeepLast(1));
  vio_qos.best_effort();
  vio_qos.durability_volatile();

  vio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      vio_topic_, vio_qos,
      std::bind(&QuadEsekfNode::vioCallback, this, std::placeholders::_1));

  rclcpp::QoS out_qos(rclcpp::KeepLast(1));
  out_qos.best_effort();
  out_qos.durability_volatile();

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, out_qos);

  publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      std::bind(&QuadEsekfNode::publishTimerCallback, this));

  RCLCPP_INFO(get_logger(), "esekf_node started: direct VIO-anchor + IMU preintegration predictor");
  RCLCPP_INFO(
      get_logger(),
      "  imu_pred: pos=%s vel=%s gyro=%s gain_p=%.2f gain_v=%.2f horizon=%.3f extra=%.3f",
      use_imu_accel_for_position_prediction_ ? "true" : "false",
      use_imu_accel_for_velocity_prediction_ ? "true" : "false",
      use_imu_gyro_for_orientation_prediction_ ? "true" : "false",
      imu_position_prediction_gain_,
      imu_velocity_prediction_gain_,
      max_imu_prediction_horizon_s_,
      extra_prediction_s_);
}

void QuadEsekfNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);

  const rclcpp::Time stamp(msg->header.stamp);
  if (stamp.nanoseconds() == 0) {
    return;
  }

  if (imu_buffer_.empty() || rclcpp::Time(imu_buffer_.back().header.stamp) < stamp) {
    imu_buffer_.push_back(*msg);
    latest_imu_stamp_ = stamp;
    pruneImuBuffer(stamp);
  }
}

void QuadEsekfNode::vioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);

  const rclcpp::Time measurement_stamp = getVioMeasurementStamp(*msg);
  if (measurement_stamp.nanoseconds() == 0) {
    return;
  }

  if (!initialized_) {
    initializeFromVio(*msg, measurement_stamp);
  } else {
    updateAnchorFromVio(*msg, measurement_stamp);
  }
}

void QuadEsekfNode::publishTimerCallback()
{
  std::lock_guard<std::mutex> lock(mtx_);

  if (!initialized_) {
    return;
  }

  const rclcpp::Time target_stamp = getPredictionTargetStamp();
  State out = predictFromAnchorTo(target_stamp);

  if (enable_pose_output_smoothing_ || enable_twist_output_smoothing_) {
    if (!output_initialized_ ||
        last_output_stamp_.nanoseconds() == 0 ||
        target_stamp <= last_output_stamp_) {
      output_state_ = out;
      output_initialized_ = true;
      last_output_stamp_ = target_stamp;
    } else {
      double dt = (target_stamp - last_output_stamp_).seconds();
      dt = std::clamp(dt, 0.0, 0.05);

      if (enable_pose_output_smoothing_) {
        const double a = std::clamp(1.0 - std::exp(-dt / pose_output_smoothing_tau_s_), 0.0, 1.0);
        output_state_.p_WB += a * (out.p_WB - output_state_.p_WB);
        output_state_.q_WB =
            output_state_.q_WB.normalized().slerp(a, out.q_WB.normalized()).normalized();

        const Eigen::Vector3d e = out.p_WB - output_state_.p_WB;
        const double en = e.norm();
        if (max_output_position_error_m_ > 0.0 && en > max_output_position_error_m_) {
          output_state_.p_WB = out.p_WB - e / en * max_output_position_error_m_;
        }
      } else {
        output_state_.p_WB = out.p_WB;
        output_state_.q_WB = out.q_WB;
      }

      if (enable_twist_output_smoothing_) {
        const double a = std::clamp(1.0 - std::exp(-dt / twist_output_smoothing_tau_s_), 0.0, 1.0);
        output_state_.v_WB += a * (out.v_WB - output_state_.v_WB);
      } else {
        output_state_.v_WB = out.v_WB;
      }

      last_output_stamp_ = target_stamp;
    }

    out = output_state_;
  }

  publishOdometry(out, target_stamp);
}

void QuadEsekfNode::initializeFromVio(
    const nav_msgs::msg::Odometry & msg,
    const rclcpp::Time & measurement_stamp)
{
  anchor_.p_WB = Eigen::Vector3d(
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      msg.pose.pose.position.z);

  anchor_.q_WB = Eigen::Quaterniond(
      msg.pose.pose.orientation.w,
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z);
  if (!isFiniteQuaternion(anchor_.q_WB)) {
    anchor_.q_WB = Eigen::Quaterniond::Identity();
  } else {
    anchor_.q_WB.normalize();
  }

  anchor_.v_WB = Eigen::Vector3d(
      msg.twist.twist.linear.x,
      msg.twist.twist.linear.y,
      msg.twist.twist.linear.z);
  if (!isFiniteVector(anchor_.v_WB)) {
    anchor_.v_WB.setZero();
  }

  last_vio_twist_ = anchor_.v_WB;
  prev_vio_twist_ = anchor_.v_WB;
  vio_twist_accel_lpf_.setZero();
  has_last_vio_twist_ = true;
  has_vio_twist_accel_lpf_ = true;
  last_vio_twist_dt_s_ = 0.10;

  previous_vio_ = anchor_;
  previous_vio_stamp_ = measurement_stamp;
  has_previous_vio_ = true;

  latest_pose_covariance_ = msg.pose.covariance;
  latest_twist_covariance_ = msg.twist.covariance;

  anchor_stamp_ = measurement_stamp;
  output_state_ = anchor_;
  output_initialized_ = true;
  last_output_stamp_ = measurement_stamp;
  initialized_ = true;

  RCLCPP_INFO(
      get_logger(),
      "Initialized direct predictor at t=%.9f p=[%.3f %.3f %.3f] v=[%.3f %.3f %.3f]",
      anchor_stamp_.seconds(),
      anchor_.p_WB.x(), anchor_.p_WB.y(), anchor_.p_WB.z(),
      anchor_.v_WB.x(), anchor_.v_WB.y(), anchor_.v_WB.z());
}

void QuadEsekfNode::updateAnchorFromVio(
    const nav_msgs::msg::Odometry & msg,
    const rclcpp::Time & measurement_stamp)
{
  State vio;

  vio.p_WB = Eigen::Vector3d(
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      msg.pose.pose.position.z);

  vio.q_WB = Eigen::Quaterniond(
      msg.pose.pose.orientation.w,
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z);
  if (!isFiniteQuaternion(vio.q_WB)) {
    vio.q_WB = anchor_.q_WB;
  } else {
    vio.q_WB.normalize();
  }

  Eigen::Vector3d raw_v = anchor_.v_WB;

  if (use_vio_twist_velocity_) {
    raw_v = Eigen::Vector3d(
        msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.linear.z);
    if (!isFiniteVector(raw_v)) {
      raw_v = anchor_.v_WB;
    }
  } else if (has_previous_vio_) {
    const double dt = (measurement_stamp - previous_vio_stamp_).seconds();
    if (std::isfinite(dt) && dt > 1.0e-4 && dt < 1.0) {
      raw_v = (vio.p_WB - previous_vio_.p_WB) / dt;
    }
  }

  // Critical: the anchor velocity should not be low-pass filtered if we are
  // trying to remove delay. Smoothing here directly creates phase lag.
  if (smooth_vio_anchor_velocity_) {
    vio.v_WB =
        (1.0 - vio_velocity_alpha_) * anchor_.v_WB +
        vio_velocity_alpha_ * raw_v;
  } else {
    vio.v_WB = raw_v;
  }

  previous_vio_ = vio;
  previous_vio_stamp_ = measurement_stamp;
  has_previous_vio_ = true;

  anchor_ = vio;
  anchor_stamp_ = measurement_stamp;

  latest_pose_covariance_ = msg.pose.covariance;
  latest_twist_covariance_ = msg.twist.covariance;
}


Eigen::Vector3d QuadEsekfNode::computeVioTwistRampDelta(double horizon_s) const
{
  Eigen::Vector3d dv = Eigen::Vector3d::Zero();

  if (!use_vio_twist_ramp_prediction_ ||
      !has_vio_twist_accel_lpf_ ||
      !std::isfinite(horizon_s) ||
      horizon_s <= 0.0) {
    return dv;
  }

  const double h = std::clamp(horizon_s, 0.0, max_vio_twist_ramp_horizon_s_);

  dv = vio_twist_ramp_gain_ * vio_twist_accel_lpf_ * h;

  dv.x() = std::clamp(dv.x(), -max_vio_twist_ramp_delta_xy_mps_, max_vio_twist_ramp_delta_xy_mps_);
  dv.y() = std::clamp(dv.y(), -max_vio_twist_ramp_delta_xy_mps_, max_vio_twist_ramp_delta_xy_mps_);
  dv.z() = std::clamp(dv.z(), -max_vio_twist_ramp_delta_z_mps_, max_vio_twist_ramp_delta_z_mps_);

  if (use_vio_twist_ramp_for_z_only_) {
    dv.x() = 0.0;
    dv.y() = 0.0;
  }

  return dv;
}

QuadEsekfNode::State QuadEsekfNode::predictFromAnchorTo(const rclcpp::Time & target_stamp) const
{
  State pred = anchor_;

  if (target_stamp.nanoseconds() == 0 || anchor_stamp_.nanoseconds() == 0 || target_stamp <= anchor_stamp_) {
    return pred;
  }

  double total_dt = (target_stamp - anchor_stamp_).seconds();
  if (!std::isfinite(total_dt) || total_dt <= 0.0) {
    return pred;
  }

  total_dt = std::min(total_dt, max_prediction_horizon_s_);

  const double imu_dt = std::min(total_dt, max_imu_prediction_horizon_s_);
  const rclcpp::Time imu_target =
      anchor_stamp_ + rclcpp::Duration::from_seconds(imu_dt);

  ImuDelta delta = integrateImuDelta(anchor_, anchor_stamp_, imu_target);

  const Eigen::Quaterniond q_pred =
      use_imu_gyro_for_orientation_prediction_ && delta.valid
          ? (anchor_.q_WB * delta.dq).normalized()
          : anchor_.q_WB;

  Eigen::Vector3d dv = Eigen::Vector3d::Zero();
  Eigen::Vector3d dp_accel = Eigen::Vector3d::Zero();

  if (delta.valid) {
    dv = delta.dv_W;
    dv.x() = std::clamp(dv.x(), -max_imu_velocity_delta_xy_mps_, max_imu_velocity_delta_xy_mps_);
    dv.y() = std::clamp(dv.y(), -max_imu_velocity_delta_xy_mps_, max_imu_velocity_delta_xy_mps_);
    dv.z() = std::clamp(dv.z(), -max_imu_velocity_delta_z_mps_, max_imu_velocity_delta_z_mps_);

    dp_accel = delta.dp_W;
    dp_accel.x() = std::clamp(dp_accel.x(), -max_imu_position_delta_xy_m_, max_imu_position_delta_xy_m_);
    dp_accel.y() = std::clamp(dp_accel.y(), -max_imu_position_delta_xy_m_, max_imu_position_delta_xy_m_);
    dp_accel.z() = std::clamp(dp_accel.z(), -max_imu_position_delta_z_m_, max_imu_position_delta_z_m_);
  }

  pred.q_WB = q_pred;

  pred.v_WB = anchor_.v_WB;

  if (use_imu_accel_for_velocity_prediction_ && delta.valid) {
    pred.v_WB.x() += imu_velocity_prediction_gain_ * imu_velocity_prediction_gain_xy_ * dv.x();
    pred.v_WB.y() += imu_velocity_prediction_gain_ * imu_velocity_prediction_gain_xy_ * dv.y();
    pred.v_WB.z() += imu_velocity_prediction_gain_ * imu_velocity_prediction_gain_z_ * dv.z();
  }

  // Add a VIO twist-ramp delta. This is the main fix for takeoff Vz:
  // it avoids sample-and-hold velocity while staying attached to the VIO trend.
  pred.v_WB += computeVioTwistRampDelta(total_dt);

  // Position prediction uses the anchor velocity over the full horizon plus a
  // bounded IMU double-integral over the short IMU horizon. This avoids the old
  // "pose follows VIO only, twist predicted separately" inconsistency.
  pred.p_WB = anchor_.p_WB + anchor_.v_WB * total_dt;
  if (use_imu_accel_for_position_prediction_ && delta.valid) {
    pred.p_WB += imu_position_prediction_gain_ * dp_accel;
  }

  return pred;
}

QuadEsekfNode::ImuDelta QuadEsekfNode::integrateImuDelta(
    const State & anchor,
    const rclcpp::Time & t0,
    const rclcpp::Time & t1) const
{
  ImuDelta out;

  if (imu_buffer_.empty() || t1 <= t0) {
    return out;
  }

  Eigen::Quaterniond q_delta = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond q_WB = anchor.q_WB.normalized();

  Eigen::Vector3d dv = Eigen::Vector3d::Zero();
  Eigen::Vector3d dp = Eigen::Vector3d::Zero();

  Eigen::Vector3d a_filt = Eigen::Vector3d::Zero();
  bool a_filt_initialized = false;

  rclcpp::Time prev_stamp = t0;

  for (const auto & imu_msg : imu_buffer_) {
    const rclcpp::Time imu_stamp(imu_msg.header.stamp);

    if (imu_stamp <= t0) {
      continue;
    }
    if (imu_stamp > t1) {
      break;
    }

    double dt = (imu_stamp - prev_stamp).seconds();
    if (!std::isfinite(dt) || dt <= kMinDt) {
      prev_stamp = imu_stamp;
      continue;
    }
    dt = std::min(dt, kMaxImuDt);

    const Eigen::Vector3d omega =
        transformImuVector(R_imu_to_vio_ned_, imu_msg.angular_velocity);
    const Eigen::Vector3d accel =
        transformImuVector(R_imu_to_vio_ned_, imu_msg.linear_acceleration);

    if (isFiniteVector(omega)) {
      const Eigen::Quaterniond dq = smallAngleQuaternion(omega * dt);
      q_delta = (q_delta * dq).normalized();
      q_WB = (q_WB * dq).normalized();
    }

    if (isFiniteVector(accel)) {
      Eigen::Vector3d a_W =
          q_WB.toRotationMatrix() * accel + Eigen::Vector3d(0.0, 0.0, gravity_mps2_);

      for (int i = 0; i < 3; ++i) {
        if (std::abs(a_W[i]) < imu_accel_deadband_mps2_) {
          a_W[i] = 0.0;
        }
      }

      if (!a_filt_initialized) {
        a_filt = a_W;
        a_filt_initialized = true;
      } else {
        const double alpha =
            std::clamp(1.0 - std::exp(-dt / imu_accel_lpf_tau_s_), 0.0, 1.0);
        a_filt += alpha * (a_W - a_filt);
      }

      dp += dv * dt + 0.5 * a_filt * dt * dt;
      dv += a_filt * dt;
      out.valid = true;
      out.dt += dt;
    }

    prev_stamp = imu_stamp;
  }

  out.dp_W = dp;
  out.dv_W = dv;
  out.dq = q_delta.normalized();

  return out;
}

void QuadEsekfNode::publishOdometry(const State & state, const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = world_frame_id_;
  odom.child_frame_id = body_frame_id_;

  odom.pose.pose.position.x = state.p_WB.x();
  odom.pose.pose.position.y = state.p_WB.y();
  odom.pose.pose.position.z = state.p_WB.z();

  const Eigen::Quaterniond q = state.q_WB.normalized();
  odom.pose.pose.orientation.w = q.w();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();

  odom.twist.twist.linear.x = state.v_WB.x();
  odom.twist.twist.linear.y = state.v_WB.y();
  odom.twist.twist.linear.z = state.v_WB.z();

  odom.pose.covariance = latest_pose_covariance_;
  odom.twist.covariance = latest_twist_covariance_;

  odom_pub_->publish(odom);
}

rclcpp::Time QuadEsekfNode::getVioMeasurementStamp(const nav_msgs::msg::Odometry & msg) const
{
  rclcpp::Time stamp(msg.header.stamp);

  if (vio_delay_s_ > 0.0) {
    stamp = stamp - rclcpp::Duration::from_seconds(vio_delay_s_);
  }

  return stamp;
}

rclcpp::Time QuadEsekfNode::getPredictionTargetStamp() const
{
  rclcpp::Time target = anchor_stamp_;

  if (latest_imu_stamp_.nanoseconds() > 0 && latest_imu_stamp_ > target) {
    target = latest_imu_stamp_;
  }

  if (extra_prediction_s_ > 0.0) {
    target = target + rclcpp::Duration::from_seconds(extra_prediction_s_);
  }

  if (anchor_stamp_.nanoseconds() > 0 && max_prediction_horizon_s_ > 0.0) {
    const rclcpp::Time max_target =
        anchor_stamp_ + rclcpp::Duration::from_seconds(max_prediction_horizon_s_);
    if (target > max_target) {
      target = max_target;
    }
  }

  return target;
}

void QuadEsekfNode::pruneImuBuffer(const rclcpp::Time & newest_stamp)
{
  const rclcpp::Time oldest_allowed =
      newest_stamp - rclcpp::Duration::from_seconds(imu_buffer_s_);

  while (!imu_buffer_.empty() &&
         rclcpp::Time(imu_buffer_.front().header.stamp) < oldest_allowed) {
    imu_buffer_.pop_front();
  }
}

Eigen::Quaterniond QuadEsekfNode::smallAngleQuaternion(const Eigen::Vector3d & dtheta)
{
  const double angle = dtheta.norm();

  if (!std::isfinite(angle) || angle < 1.0e-12) {
    return Eigen::Quaterniond(
        1.0,
        0.5 * dtheta.x(),
        0.5 * dtheta.y(),
        0.5 * dtheta.z()).normalized();
  }

  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, dtheta / angle)).normalized();
}

Eigen::Vector3d QuadEsekfNode::transformImuVector(
    const Eigen::Matrix3d & R_imu_to_vio_ned,
    const geometry_msgs::msg::Vector3 & v)
{
  return R_imu_to_vio_ned * Eigen::Vector3d(v.x, v.y, v.z);
}

bool QuadEsekfNode::isFiniteQuaternion(const Eigen::Quaterniond & q)
{
  return std::isfinite(q.w()) &&
         std::isfinite(q.x()) &&
         std::isfinite(q.y()) &&
         std::isfinite(q.z()) &&
         q.norm() > 1.0e-9;
}

bool QuadEsekfNode::isFiniteVector(const Eigen::Vector3d & v)
{
  return std::isfinite(v.x()) &&
         std::isfinite(v.y()) &&
         std::isfinite(v.z());
}

}  // namespace mavtech_filter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mavtech_filter::QuadEsekfNode>());
  rclcpp::shutdown();
  return 0;
}