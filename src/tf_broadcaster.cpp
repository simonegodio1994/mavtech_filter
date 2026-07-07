#include <array>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/vehicle_odometry.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


class Px4OdomTfBroadcaster : public rclcpp::Node
{
public:
  Px4OdomTfBroadcaster()
  : Node("px4_odom_tf_broadcaster")
  {
    odom_topic_ = this->declare_parameter<std::string>(
      "odom_topic",
      "/fmu/out/vehicle_odometry"
    );

    parent_frame_id_ = this->declare_parameter<std::string>(
      "parent_frame_id",
      "map"
    );

    body_frame_id_ = this->declare_parameter<std::string>(
      "body_frame_id",
      "body"
    );

    depth_frame_id_ = this->declare_parameter<std::string>(
      "depth_frame_id",
      "body"
    );

    convert_ned_to_enu_ = this->declare_parameter<bool>(
      "convert_ned_to_enu",
      true
    );

    publish_depth_alias_ = this->declare_parameter<bool>(
      "publish_depth_alias",
      false
    );

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    auto qos = rclcpp::SensorDataQoS();

    odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      odom_topic_,
      qos,
      std::bind(&Px4OdomTfBroadcaster::odomCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "PX4 odom TF broadcaster started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing: %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing TF: %s -> %s",
                parent_frame_id_.c_str(), body_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "convert_ned_to_enu: %s",
                convert_ned_to_enu_ ? "true" : "false");

    if (publish_depth_alias_) {
      RCLCPP_INFO(this->get_logger(), "Publishing depth alias TF: %s -> %s",
                  body_frame_id_.c_str(), depth_frame_id_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Depth frame assumed coincident with body, no extra TF published.");
    }
  }

private:
  void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    if (!isFiniteOdom(*msg)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Received VehicleOdometry with NaN/Inf. Skipping TF."
      );
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = this->now();
    tf_msg.header.frame_id = parent_frame_id_;
    tf_msg.child_frame_id = body_frame_id_;

    const double px = static_cast<double>(msg->position[0]);
    const double py = static_cast<double>(msg->position[1]);
    const double pz = static_cast<double>(msg->position[2]);

    // PX4 quaternion order is q[0]=w, q[1]=x, q[2]=y, q[3]=z.
    const double qw = static_cast<double>(msg->q[0]);
    const double qx = static_cast<double>(msg->q[1]);
    const double qy = static_cast<double>(msg->q[2]);
    const double qz = static_cast<double>(msg->q[3]);

    tf2::Quaternion q_in(qx, qy, qz, qw);
    q_in.normalize();

    tf2::Vector3 p_out;
    tf2::Quaternion q_out;

    if (convert_ned_to_enu_) {
      convertPoseNedFrdToEnuFlu(px, py, pz, q_in, p_out, q_out);
    } else {
      p_out = tf2::Vector3(px, py, pz);
      q_out = q_in;
      q_out.normalize();
    }

    tf_msg.transform.translation.x = p_out.x();
    tf_msg.transform.translation.y = p_out.y();
    tf_msg.transform.translation.z = p_out.z();

    tf_msg.transform.rotation.x = q_out.x();
    tf_msg.transform.rotation.y = q_out.y();
    tf_msg.transform.rotation.z = q_out.z();
    tf_msg.transform.rotation.w = q_out.w();

    tf_broadcaster_->sendTransform(tf_msg);

    if (publish_depth_alias_ && depth_frame_id_ != body_frame_id_) {
      publishDepthAlias(tf_msg.header.stamp);
    }
  }

  static bool isFiniteOdom(const px4_msgs::msg::VehicleOdometry & msg)
  {
    for (int i = 0; i < 3; ++i) {
      if (!std::isfinite(msg.position[i])) {
        return false;
      }
    }

    for (int i = 0; i < 4; ++i) {
      if (!std::isfinite(msg.q[i])) {
        return false;
      }
    }

    const double q_norm =
      std::sqrt(
        msg.q[0] * msg.q[0] +
        msg.q[1] * msg.q[1] +
        msg.q[2] * msg.q[2] +
        msg.q[3] * msg.q[3]
      );

    return q_norm > 1.0e-6;
  }

  static void convertPoseNedFrdToEnuFlu(
    const double px_ned,
    const double py_ned,
    const double pz_ned,
    const tf2::Quaternion & q_ned_frd,
    tf2::Vector3 & p_enu,
    tf2::Quaternion & q_enu_flu)
  {
    // Position:
    //
    // NED:
    //   x = north
    //   y = east
    //   z = down
    //
    // ENU:
    //   x = east
    //   y = north
    //   z = up
    //
    // Therefore:
    //   ENU = [y_ned, x_ned, -z_ned]
    p_enu = tf2::Vector3(
      py_ned,
      px_ned,
      -pz_ned
    );

    // Orientation:
    //
    // PX4 body frame is usually FRD:
    //   x = forward
    //   y = right
    //   z = down
    //
    // ROS body frame is usually FLU:
    //   x = forward
    //   y = left
    //   z = up
    //
    // We want:
    //   R_enu_flu = R_enu_ned * R_ned_frd * R_frd_flu
    //
    // where:
    //   R_enu_ned =
    //      [0  1  0
    //       1  0  0
    //       0  0 -1]
    //
    //   R_frd_flu =
    //      [1  0  0
    //       0 -1  0
    //       0  0 -1]

    tf2::Matrix3x3 R_ned_frd(q_ned_frd);

    tf2::Matrix3x3 R_enu_ned(
      0.0, 1.0,  0.0,
      1.0, 0.0,  0.0,
      0.0, 0.0, -1.0
    );

    tf2::Matrix3x3 R_frd_flu(
      1.0,  0.0,  0.0,
      0.0, -1.0,  0.0,
      0.0,  0.0, -1.0
    );

    tf2::Matrix3x3 R_enu_flu = R_enu_ned * R_ned_frd * R_frd_flu;

    R_enu_flu.getRotation(q_enu_flu);
    q_enu_flu.normalize();
  }

  void publishDepthAlias(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped depth_tf;

    depth_tf.header.stamp = stamp;
    depth_tf.header.frame_id = body_frame_id_;
    depth_tf.child_frame_id = depth_frame_id_;

    depth_tf.transform.translation.x = 0.0;
    depth_tf.transform.translation.y = 0.0;
    depth_tf.transform.translation.z = 0.0;

    depth_tf.transform.rotation.x = 0.0;
    depth_tf.transform.rotation.y = 0.0;
    depth_tf.transform.rotation.z = 0.0;
    depth_tf.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(depth_tf);
  }

private:
  std::string odom_topic_;
  std::string parent_frame_id_;
  std::string body_frame_id_;
  std::string depth_frame_id_;

  bool convert_ned_to_enu_;
  bool publish_depth_alias_;

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Px4OdomTfBroadcaster>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}