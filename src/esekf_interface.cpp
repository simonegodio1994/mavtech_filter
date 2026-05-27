// src/odom_ned_to_px4_vehicle_odometry.cpp

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

class OdomNedToPx4VehicleOdometry final : public rclcpp::Node
{
public:
  OdomNedToPx4VehicleOdometry()
  : Node("odom_ned_to_px4_vehicle_odometry")
  {
    // ------------------------------------------------------------
    // Parameters
    // ------------------------------------------------------------
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/mavtech_filter/odom_ned");

    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/fmu/in/vehicle_visual_odometry");

    velocity_in_body_frame_ = declare_parameter<bool>(
      "velocity_in_body_frame", true);

    send_velocity_in_ned_ = declare_parameter<bool>(
      "send_velocity_in_ned", true);

    use_header_stamp_for_sample_ = declare_parameter<bool>(
      "use_header_stamp_for_sample", true);

    timestamp_offset_ms_ = declare_parameter<double>(
      "timestamp_offset_ms", 0.0);

    reset_counter_ = declare_parameter<int>(
      "reset_counter", 0);

    quality_ = declare_parameter<int>(
      "quality", 100);

    debug_orientation_ = declare_parameter<bool>(
      "debug_orientation", false);

    debug_velocity_ = declare_parameter<bool>(
      "debug_velocity", false);

    debug_timing_ = declare_parameter<bool>(
      "debug_timing", false);

    debug_every_n_ = declare_parameter<int>(
      "debug_every_n", 200);

    if (debug_every_n_ < 1) {
      debug_every_n_ = 1;
    }

    // ------------------------------------------------------------
    // QoS
    // PX4 side usually expects best effort + transient local.
    // Input odometry should stay low-latency: keep only latest sample.
    // ------------------------------------------------------------
    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    px4_qos.best_effort();
    px4_qos.transient_local();

    auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    odom_qos.best_effort();
    odom_qos.durability_volatile();

    publisher_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
      output_topic_, px4_qos);

    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_,
      odom_qos,
      std::bind(&OdomNedToPx4VehicleOdometry::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Converting Odometry NED '%s' to VehicleOdometry '%s'",
      input_topic_.c_str(),
      output_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Input velocity frame: %s",
      velocity_in_body_frame_ ? "BODY_FRD" : "NED");

    RCLCPP_INFO(
      get_logger(),
      "Output velocity frame sent to PX4: %s",
      send_velocity_in_ned_ ? "NED" : "BODY_FRD if input is BODY_FRD, otherwise NED");

    RCLCPP_INFO(
      get_logger(),
      "use_header_stamp_for_sample=%s, timestamp_offset_ms=%.3f",
      use_header_stamp_for_sample_ ? "true" : "false",
      timestamp_offset_ms_);
  }

private:
  using Vec3 = std::array<double, 3>;
  using Quat = std::array<double, 4>;  // w, x, y, z

  static inline uint64_t stampToUs(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<uint64_t>(stamp.sec) * 1000000ULL +
           static_cast<uint64_t>(stamp.nanosec) / 1000ULL;
  }

  inline uint64_t nowUs()
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000LL);
  }

  static inline double wrapPi(const double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static inline double clamp(const double v, const double lo, const double hi)
  {
    return std::max(lo, std::min(v, hi));
  }

  static inline Quat normalizeQuaternion(
    const double w,
    const double x,
    const double y,
    const double z)
  {
    const double n = std::sqrt(w * w + x * x + y * y + z * z);

    if (!std::isfinite(n) || n < 1e-12) {
      return {1.0, 0.0, 0.0, 0.0};
    }

    const double inv_n = 1.0 / n;
    return {w * inv_n, x * inv_n, y * inv_n, z * inv_n};
  }

  static inline Vec3 quaternionToRpy(
    const double w,
    const double x,
    const double y,
    const double z)
  {
    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    const double roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (w * y - z * x);
    sinp = clamp(sinp, -1.0, 1.0);
    const double pitch = std::asin(sinp);

    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);

    return {roll, pitch, yaw};
  }

  static inline Quat rpyToQuaternion(
    const double roll,
    const double pitch,
    const double yaw)
  {
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);

    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);

    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);

    const double w = cr * cp * cy + sr * sp * sy;
    const double x = sr * cp * cy - cr * sp * sy;
    const double y = cr * sp * cy + sr * cp * sy;
    const double z = cr * cp * sy - sr * sp * cy;

    return normalizeQuaternion(w, x, y, z);
  }

  static inline Quat quaternionWithWrappedRpy(
    const double w,
    const double x,
    const double y,
    const double z)
  {
    const auto q = normalizeQuaternion(w, x, y, z);
    const auto rpy = quaternionToRpy(q[0], q[1], q[2], q[3]);

    return rpyToQuaternion(
      wrapPi(rpy[0]),
      wrapPi(rpy[1]),
      wrapPi(rpy[2]));
  }

  static inline Vec3 cross(const Vec3 & a, const Vec3 & b)
  {
    return {
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0]
    };
  }

  static inline Vec3 quaternionRotateVector(
    const double q_w,
    const double q_x,
    const double q_y,
    const double q_z,
    const Vec3 & v)
  {
    const auto q = normalizeQuaternion(q_w, q_x, q_y, q_z);
    const Vec3 q_vec = {q[1], q[2], q[3]};

    Vec3 t = cross(q_vec, v);
    t[0] *= 2.0;
    t[1] *= 2.0;
    t[2] *= 2.0;

    const Vec3 q_cross_t = cross(q_vec, t);

    return {
      v[0] + q[0] * t[0] + q_cross_t[0],
      v[1] + q[0] * t[1] + q_cross_t[1],
      v[2] + q[0] * t[2] + q_cross_t[2]
    };
  }

  static inline Vec3 quaternionInverseRotateVector(
    const double q_w,
    const double q_x,
    const double q_y,
    const double q_z,
    const Vec3 & v)
  {
    const auto q = normalizeQuaternion(q_w, q_x, q_y, q_z);
    return quaternionRotateVector(q[0], -q[1], -q[2], -q[3], v);
  }

  inline std::pair<uint64_t, uint64_t> getMessageTimestampsUs(
    const nav_msgs::msg::Odometry & odom)
  {
    const uint64_t timestamp_us = nowUs();

    if (!use_header_stamp_for_sample_) {
      return {timestamp_us, timestamp_us};
    }

    if (odom.header.stamp.sec == 0 && odom.header.stamp.nanosec == 0) {
      return {timestamp_us, timestamp_us};
    }

    const uint64_t raw_sample_us = stampToUs(odom.header.stamp);

    const int64_t corrected_sample_us_signed =
      static_cast<int64_t>(raw_sample_us) -
      static_cast<int64_t>(timestamp_offset_ms_ * 1000.0);

    uint64_t corrected_sample_us = 0;

    if (corrected_sample_us_signed > 0) {
      corrected_sample_us = static_cast<uint64_t>(corrected_sample_us_signed);
    }

    // Importantissimo per PX4:
    // timestamp_sample non deve mai essere nel futuro rispetto a timestamp.
    if (corrected_sample_us > timestamp_us) {
      corrected_sample_us = timestamp_us;
    }

    return {timestamp_us, corrected_sample_us};
  }

  static inline bool finite3(const Vec3 & v)
  {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    px4_msgs::msg::VehicleOdometry msg{};

    const auto timestamps = getMessageTimestampsUs(*odom);
    msg.timestamp = timestamps.first;
    msg.timestamp_sample = timestamps.second;

    // ------------------------------------------------------------
    // Pose frame
    // Input odometry pose assumed already expressed in NED.
    // ------------------------------------------------------------
    msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

    const Vec3 position = {
      odom->pose.pose.position.x,
      odom->pose.pose.position.y,
      odom->pose.pose.position.z
    };

    if (!finite3(position)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Skipping odometry: non-finite position");
      return;
    }

    msg.position[0] = static_cast<float>(position[0]);  // North
    msg.position[1] = static_cast<float>(position[1]);  // East
    msg.position[2] = static_cast<float>(position[2]);  // Down

    // ------------------------------------------------------------
    // Orientation
    //
    // ROS geometry_msgs order: x, y, z, w
    // PX4 VehicleOdometry order: w, x, y, z
    //
    // Assumption:
    // q rotates BODY_FRD vectors into NED.
    // ------------------------------------------------------------
    const auto & q_ros = odom->pose.pose.orientation;

    const auto q = quaternionWithWrappedRpy(
      q_ros.w,
      q_ros.x,
      q_ros.y,
      q_ros.z);

    msg.q[0] = static_cast<float>(q[0]);
    msg.q[1] = static_cast<float>(q[1]);
    msg.q[2] = static_cast<float>(q[2]);
    msg.q[3] = static_cast<float>(q[3]);

    const auto rpy = quaternionToRpy(q[0], q[1], q[2], q[3]);

    // ------------------------------------------------------------
    // Linear velocity
    // ------------------------------------------------------------
    const Vec3 v_in = {
      odom->twist.twist.linear.x,
      odom->twist.twist.linear.y,
      odom->twist.twist.linear.z
    };

    if (!finite3(v_in)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Skipping odometry: non-finite linear velocity");
      return;
    }

    Vec3 v_body_frd{};
    Vec3 v_ned{};

    if (velocity_in_body_frame_) {
      v_body_frd = v_in;
      v_ned = quaternionRotateVector(q[0], q[1], q[2], q[3], v_body_frd);
    } else {
      v_ned = v_in;
      v_body_frd = quaternionInverseRotateVector(q[0], q[1], q[2], q[3], v_ned);
    }

    if (send_velocity_in_ned_) {
      msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
      msg.velocity[0] = static_cast<float>(v_ned[0]);
      msg.velocity[1] = static_cast<float>(v_ned[1]);
      msg.velocity[2] = static_cast<float>(v_ned[2]);
    } else {
      if (velocity_in_body_frame_) {
        msg.velocity_frame =
          px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;

        msg.velocity[0] = static_cast<float>(v_body_frd[0]);
        msg.velocity[1] = static_cast<float>(v_body_frd[1]);
        msg.velocity[2] = static_cast<float>(v_body_frd[2]);
      } else {
        msg.velocity_frame =
          px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;

        msg.velocity[0] = static_cast<float>(v_ned[0]);
        msg.velocity[1] = static_cast<float>(v_ned[1]);
        msg.velocity[2] = static_cast<float>(v_ned[2]);
      }
    }

    // ------------------------------------------------------------
    // Angular velocity
    // PX4 expects body-fixed FRD.
    // If not available/reliable, NaN is safer than fake zeros.
    // ------------------------------------------------------------
    msg.angular_velocity[0] = std::numeric_limits<float>::quiet_NaN();
    msg.angular_velocity[1] = std::numeric_limits<float>::quiet_NaN();
    msg.angular_velocity[2] = std::numeric_limits<float>::quiet_NaN();

    // ------------------------------------------------------------
    // Variances
    // Same values as your Python bridge.
    // Tune these carefully because PX4 EKF2 trust is very sensitive here.
    // ------------------------------------------------------------
    msg.position_variance[0] = 0.0003F;
    msg.position_variance[1] = 0.0003F;
    msg.position_variance[2] = 0.00005F;

    msg.velocity_variance[0] = 0.0015F;
    msg.velocity_variance[1] = 0.0015F;
    msg.velocity_variance[2] = 0.0015F;

    msg.orientation_variance[0] = 0.00001F;
    msg.orientation_variance[1] = 0.00001F;
    msg.orientation_variance[2] = 0.00001F;

    msg.reset_counter = static_cast<uint8_t>(std::max(0, reset_counter_));
    msg.quality = static_cast<int8_t>(std::max(-1, std::min(100, quality_)));

    publisher_->publish(msg);

    // ------------------------------------------------------------
    // Throttled debug
    // ------------------------------------------------------------
    ++debug_counter_;

    if (debug_counter_ % debug_every_n_ == 0) {
      if (debug_timing_) {
        const double delay_ms =
          static_cast<double>(msg.timestamp - msg.timestamp_sample) / 1000.0;

        RCLCPP_INFO(
          get_logger(),
          "Timing | timestamp=%lu us | timestamp_sample=%lu us | effective_delay_ms=%.2f",
          msg.timestamp,
          msg.timestamp_sample,
          delay_ms);
      }

      if (debug_orientation_) {
        RCLCPP_INFO(
          get_logger(),
          "Orientation sent to PX4 | roll=%+.4f rad (%+.2f deg), "
          "pitch=%+.4f rad (%+.2f deg), yaw=%+.4f rad (%+.2f deg)",
          rpy[0], rpy[0] * 180.0 / M_PI,
          rpy[1], rpy[1] * 180.0 / M_PI,
          rpy[2], rpy[2] * 180.0 / M_PI);
      }

      if (debug_velocity_) {
        RCLCPP_INFO(
          get_logger(),
          "Velocity conversion | "
          "v_body_frd=[%+.4f, %+.4f, %+.4f] | "
          "v_ned=[%+.4f, %+.4f, %+.4f] | sent_frame=%u",
          v_body_frd[0], v_body_frd[1], v_body_frd[2],
          v_ned[0], v_ned[1], v_ned[2],
          msg.velocity_frame);
      }
    }
  }

private:
  std::string input_topic_;
  std::string output_topic_;

  bool velocity_in_body_frame_{true};
  bool send_velocity_in_ned_{true};
  bool use_header_stamp_for_sample_{true};

  double timestamp_offset_ms_{0.0};

  int reset_counter_{0};
  int quality_{100};

  bool debug_orientation_{false};
  bool debug_velocity_{false};
  bool debug_timing_{false};

  int debug_every_n_{200};
  uint64_t debug_counter_{0};

  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<OdomNedToPx4VehicleOdometry>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}