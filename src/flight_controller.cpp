#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

class FlightController : public rclcpp::Node {
public:
  FlightController() : Node("flight_controller") {
    // Publishers
    force_pub_ = create_publisher<geometry_msgs::msg::Vector3>("/desired_force", 10);

    // Subscribers
    target_sub_ = create_subscription<visualization_msgs::msg::Marker>(
        "target_marker", 10,
        std::bind(&FlightController::target_callback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&FlightController::odom_callback, this, std::placeholders::_1));

    // Control loop timer (50Hz)
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&FlightController::control_loop, this));

    RCLCPP_INFO(get_logger(), "Dynamic flight controller initialized");
  }

private:
  void target_callback(const visualization_msgs::msg::Marker::SharedPtr msg) {
      target_pose_.pose.position = msg->pose.position;
      target_pose_.pose.orientation = msg->pose.orientation;
      have_target_ = true;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_ = msg->pose.pose;
    current_vel_ = msg->twist.twist.linear;
    have_odom_ = true;

  }

  void control_loop() {
    if (!have_target_ || !have_odom_) {
        RCLCPP_WARN(get_logger(), "Waiting for target or odom");
        return;
    }
    // Position errors
    double dx = target_pose_.pose.position.x - current_pose_.position.x;
    double dy = target_pose_.pose.position.y - current_pose_.position.y;
    double dz = target_pose_.pose.position.z - current_pose_.position.z;

    // Velocity errors (derivative term)
    double dvx = -current_vel_.x;
    double dvy = -current_vel_.y;
    double dvz = -current_vel_.z;

    // PID-style gains
    double Kp_xy = 3;   // horizontal proportional
    double Kd_xy = 2;   // horizontal damping
    double Kp_z = 3;   // proportional
    double Kd_z = 2;   // damping

    // Mass and gravity
    double mass = 5.0;      // bird mass in kg
    double g = 9.81;        // gravity

    // Compute desired force
    geometry_msgs::msg::Vector3 force;
    force.x = mass*(Kp_xy * dx + Kd_xy * dvx);
    force.y = mass*(Kp_xy * dy + Kd_xy * dvy);
    force.z = mass*(g + Kp_z * dz + Kd_z * dvz);  // gravity compensated
    // Clamp force values
    force.x = std::clamp(force.x, -50.0, 50.0);
    force.y = std::clamp(force.y, -50.0, 50.0);
    force.z = std::clamp(force.z, 0.0, 100.0);
    RCLCPP_INFO(get_logger(), "force.z = %.3f, dz = %.3f, dvz = %.3f", force.z, dz, dvz);

    // Publish force
    force_pub_->publish(force);
  }

  // Publishers & Subscribers
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr force_pub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  geometry_msgs::msg::PoseStamped target_pose_;
  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Vector3 current_vel_;
  bool have_target_ = false;
  bool have_odom_ = false;
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlightController>());
  rclcpp::shutdown();
  return 0;
}
