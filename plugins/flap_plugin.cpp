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

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
    std::cerr << "FlapPlugin: Load called" << std::endl;
    
    model_ = _model;
    
    // Get link name from SDF
    std::string link_name = "base_link";
    if (_sdf->HasElement("link_name")) {
        link_name = _sdf->Get<std::string>("link_name");
    }
    
    // Get link
    link_ = model_->GetLink(link_name);
    if (!link_) {
        RCLCPP_ERROR(rclcpp::get_logger("flap_plugin"), 
                    "Could not find link [%s]!", link_name.c_str());
        return;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("flap_plugin"), 
                "Successfully found link [%s]", link_name.c_str());
    
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
    desired_vel_ = *msg;
    RCLCPP_INFO(node_->get_logger(), "Received cmd_vel: [%f, %f, %f]",
                msg->linear.x, msg->linear.y, msg->linear.z);
  }

  void OnUpdate() {
    auto current_time = node_->now();
    double dt = (current_time - last_update_time_).seconds();
    
    if (dt < 0.01) return;  // Limit update rate to 100Hz

    // Apply forces proportional to desired velocity
    if (link_) {
        double force_magnitude = 50.0;
        double lift_multiplier = 2.0;

        // Get current orientation
        auto pose = link_->WorldPose();
        auto rot = pose.Rot();

        // Create force vector in world frame
        ignition::math::Vector3d world_force(
            desired_vel_.linear.x * force_magnitude,
            desired_vel_.linear.y * force_magnitude,
            desired_vel_.linear.z * force_magnitude * lift_multiplier + 49.05 // Added base lift
        );

        // Transform force to local frame
        auto local_force = rot.RotateVectorReverse(world_force);
        
        // Apply force at the link's center of mass in local frame
        link_->AddRelativeForce(local_force);
        
        RCLCPP_INFO_THROTTLE(node_->get_logger(),
                          *node_->get_clock(),
                          1000,
                          "World force: [%f, %f, %f], Local force: [%f, %f, %f]",
                          world_force.X(), world_force.Y(), world_force.Z(),
                          local_force.X(), local_force.Y(), local_force.Z());
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

    last_update_time_ = current_time;
    rclcpp::spin_some(node_);
  }

  physics::ModelPtr model_;
  physics::LinkPtr link_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  event::ConnectionPtr update_connection_;
  rclcpp::Time last_update_time_;
  geometry_msgs::msg::Twist desired_vel_;
};

GZ_REGISTER_MODEL_PLUGIN(FlapPlugin)

} // namespace gazebo