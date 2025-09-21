// flight_controller.cpp (ROS2 C++)
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

class FlightController : public rclcpp::Node {
public:
  FlightController(): Node("flight_controller") {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&FlightController::odom_cb, this, std::placeholders::_1));
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "target_pose", 10, std::bind(&FlightController::target_cb, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&FlightController::control_loop, this));
    Kp_lin_ = 1.0; Kp_yaw_ = 1.0;
  }

private:
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg){ odom_ = *msg; have_odom_ = true; }
  void target_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg){ target_ = *msg; have_target_ = true; }

  void control_loop() {
    if(!have_odom_ || !have_target_) return;
    double dx = target_.pose.position.x - odom_.pose.pose.position.x;
    double dy = target_.pose.position.y - odom_.pose.pose.position.y;
    double dz = target_.pose.position.z - odom_.pose.pose.position.z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    geometry_msgs::msg::Twist cmd;
    // simple proportional velocity command toward target
    cmd.linear.x = Kp_lin_ * dx;
    cmd.linear.y = Kp_lin_ * dy;
    cmd.linear.z = Kp_lin_ * dz;
    // yaw control (very simple)
    double yaw_err = 0.0; // compute from quaternion if needed
    cmd.angular.z = Kp_yaw_ * yaw_err;
    cmd_pub_->publish(cmd);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Odometry odom_;
  geometry_msgs::msg::PoseStamped target_;
  bool have_odom_{false}, have_target_{false};
  double Kp_lin_, Kp_yaw_;
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlightController>());
  rclcpp::shutdown();
  return 0;
}
