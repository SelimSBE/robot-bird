// flight_controller.cpp (ROS2 C++)
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

class FlightController : public rclcpp::Node {
public:
  FlightController() : Node("flight_controller") {
    // Publishers
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Subscribers
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "target_pose", 10,
      std::bind(&FlightController::target_callback, this, std::placeholders::_1));
      
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&FlightController::odom_callback, this, std::placeholders::_1));

    // Control loop timer (20Hz)
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&FlightController::control_loop, this));

    RCLCPP_INFO(get_logger(), "Flight controller initialized");
  }

private:
  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    target_pose_ = *msg;
    have_target_ = true;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_ = msg->pose.pose;
    have_odom_ = true;
  }

  void control_loop() {
    if (!have_target_ || !have_odom_) {
      RCLCPP_DEBUG(get_logger(), "Waiting for target or odom data...");
      return;
    }

    // Calculate error
    double dx = target_pose_.pose.position.x - current_pose_.position.x;
    double dy = target_pose_.pose.position.y - current_pose_.position.y;
    double dz = target_pose_.pose.position.z - current_pose_.position.z;

    // Log position error
    RCLCPP_INFO_THROTTLE(get_logger(), 
                        *get_clock(),
                        1000, // Log every second
                        "Position error: dx=%.2f, dy=%.2f, dz=%.2f",
                        dx, dy, dz);

    // Simple proportional control
    double k_px = 1.0;  // Increased gain for more responsive movement
    double k_py = 1.0;  // Increased gain for more responsive movement
    double k_pz = 1.0;  // Increased gain for more responsive movement
    
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = k_px * dx;
    cmd_vel.linear.y = k_py * dy;
    cmd_vel.linear.z = k_pz * dz;

    // Limit maximum velocity
    double max_vel = 10.0;  // Increased max velocity
    double vel_magnitude = sqrt(pow(cmd_vel.linear.x, 2) + 
                              pow(cmd_vel.linear.y, 2) + 
                              pow(cmd_vel.linear.z, 2));
    
    if (vel_magnitude > max_vel) {
      cmd_vel.linear.x *= max_vel / vel_magnitude;
      cmd_vel.linear.y *= max_vel / vel_magnitude;
      cmd_vel.linear.z *= max_vel / vel_magnitude;
    }

    // Log commanded velocity
    RCLCPP_INFO_THROTTLE(get_logger(),
                        *get_clock(),
                        1000,
                        "Cmd vel: vx=%.2f, vy=%.2f, vz=%.2f",
                        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z);

    cmd_vel_pub_->publish(cmd_vel);
  }

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  
  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // State
  geometry_msgs::msg::PoseStamped target_pose_;
  geometry_msgs::msg::Pose current_pose_;
  bool have_target_ = false;
  bool have_odom_ = false;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlightController>());
  rclcpp::shutdown();
  return 0;
}
