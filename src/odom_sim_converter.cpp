#include <array>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

class OdomSimConverter : public rclcpp::Node
{
public:
  OdomSimConverter()
  : Node("odom_sim_converter")
  {
    input_topic_ = this->declare_parameter<std::string>(
      "input_topic", "/ground_truth/odometry_enu");

    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "/ground_truth/odometry");

    output_frame_id_ = this->declare_parameter<std::string>(
      "output_frame_id", "map");

    output_child_frame_id_ = this->declare_parameter<std::string>(
      "output_child_frame_id", "body");

    pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic_, rclcpp::QoS(10));

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&OdomSimConverter::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Sim odometry converter started");
    RCLCPP_INFO(this->get_logger(), "Input ENU : %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output NWU: %s frame=%s",
                output_topic_.c_str(), output_frame_id_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    nav_msgs::msg::Odometry out = *msg;
    out.header.frame_id = output_frame_id_;
    out.child_frame_id = output_child_frame_id_;

    // Gazebo ENU -> ROS map NWU
    // ENU = [East, North, Up]
    // NWU = [North, West, Up]
    // x_nwu =  y_enu
    // y_nwu = -x_enu
    // z_nwu =  z_enu
    out.pose.pose.position.x = msg->pose.pose.position.y;
    out.pose.pose.position.y = -msg->pose.pose.position.x;
    out.pose.pose.position.z = msg->pose.pose.position.z;

    const tf2::Quaternion q_enu_body(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);

    tf2::Matrix3x3 R_enu_body(q_enu_body);

    const tf2::Matrix3x3 R_nwu_enu(
       0.0,  1.0, 0.0,
      -1.0,  0.0, 0.0,
       0.0,  0.0, 1.0);

    const tf2::Matrix3x3 R_nwu_body = R_nwu_enu * R_enu_body;

    tf2::Quaternion q_nwu_body;
    R_nwu_body.getRotation(q_nwu_body);
    q_nwu_body.normalize();

    out.pose.pose.orientation.x = q_nwu_body.x();
    out.pose.pose.orientation.y = q_nwu_body.y();
    out.pose.pose.orientation.z = q_nwu_body.z();
    out.pose.pose.orientation.w = q_nwu_body.w();

    // nav_msgs/Odometry twist is expressed in child_frame_id (body),
    // therefore it stays unchanged when only the world frame changes.
    out.twist = msg->twist;

    rotatePoseCovariance(msg->pose.covariance, out.pose.covariance);

    pub_->publish(out);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "[SIM ODOM DEBUG] ENU p=[%.3f %.3f %.3f] -> NWU/map p=[%.3f %.3f %.3f]",
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z,
      out.pose.pose.position.x,
      out.pose.pose.position.y,
      out.pose.pose.position.z);
  }

  static void rotatePoseCovariance(
    const std::array<double, 36> & input,
    std::array<double, 36> & output)
  {
    const double R[3][3] = {
      { 0.0,  1.0, 0.0},
      {-1.0,  0.0, 0.0},
      { 0.0,  0.0, 1.0}
    };

    double T[6][6] = {};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        T[i][j] = R[i][j];
        T[i + 3][j + 3] = R[i][j];
      }
    }

    double tmp[6][6] = {};
    double res[6][6] = {};

    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        for (int k = 0; k < 6; ++k) {
          tmp[i][j] += T[i][k] * input[k * 6 + j];
        }
      }
    }

    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        for (int k = 0; k < 6; ++k) {
          res[i][j] += tmp[i][k] * T[j][k];
        }
      }
    }

    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        output[i * 6 + j] = res[i][j];
      }
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  std::string output_child_frame_id_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomSimConverter>());
  rclcpp::shutdown();
  return 0;
}
