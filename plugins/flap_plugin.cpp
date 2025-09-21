#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace gazebo {

class FlapPlugin : public ModelPlugin {
public:
  FlapPlugin() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  ~FlapPlugin() {
    if (node_) {
      rclcpp::shutdown();
    }
  }

  void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override {
    std::cerr << "FlapPlugin: Load called" << std::endl;
    
    model_ = _model;
    if (!model_) {
        std::cerr << "FlapPlugin: Model pointer is null!" << std::endl;
        return;
    }

    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    node_ = std::make_shared<rclcpp::Node>("flap_plugin_node");
    std::cerr << "FlapPlugin: Created node" << std::endl;
    
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    std::cerr << "FlapPlugin: Created publisher" << std::endl;
    
    cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&FlapPlugin::cmdVelCallback, this, std::placeholders::_1));

    last_update_time_ = node_->now();
    
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&FlapPlugin::OnUpdate, this));

    RCLCPP_INFO(node_->get_logger(), "FlapPlugin initialization complete");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    vel_x_ = msg->linear.x;
    vel_y_ = msg->linear.y;
    vel_z_ = msg->linear.z;
    RCLCPP_DEBUG(node_->get_logger(), "Received cmd_vel: [%f, %f, %f]", 
                 vel_x_, vel_y_, vel_z_);
  }

  void OnUpdate() {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), 
                        *node_->get_clock(),
                        5000, // Log every 5 seconds
                        "Plugin is running, model pose: x=%f, y=%f, z=%f",
                        model_->WorldPose().Pos().X(),
                        model_->WorldPose().Pos().Y(),
                        model_->WorldPose().Pos().Z());
    
    if (!model_ || !node_ || !odom_pub_) {
      return;
    }

    // Handle ROS 2 callbacks
    rclcpp::spin_some(node_);

    // Update at 50Hz
    auto current_time = node_->now();
    if ((current_time - last_update_time_).seconds() < 0.02) {
      return;
    }
    last_update_time_ = current_time;

    // Apply forces
    auto link = model_->GetLink("base_link");
    if (link) {
      ignition::math::Vector3d force(vel_x_, vel_y_, vel_z_);
      link->AddForce(force);
    }

    // Publish odometry
    auto pose = model_->WorldPose();
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    
    // Position
    odom_msg.pose.pose.position.x = pose.Pos().X();
    odom_msg.pose.pose.position.y = pose.Pos().Y();
    odom_msg.pose.pose.position.z = pose.Pos().Z();
    
    // Orientation
    odom_msg.pose.pose.orientation.x = pose.Rot().X();
    odom_msg.pose.pose.orientation.y = pose.Rot().Y();
    odom_msg.pose.pose.orientation.z = pose.Rot().Z();
    odom_msg.pose.pose.orientation.w = pose.Rot().W();

    odom_pub_->publish(odom_msg);
  }

  physics::ModelPtr model_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  event::ConnectionPtr update_connection_;
  rclcpp::Time last_update_time_;
  double vel_x_{0}, vel_y_{0}, vel_z_{0};
};

GZ_REGISTER_MODEL_PLUGIN(FlapPlugin)

} // namespace gazebo