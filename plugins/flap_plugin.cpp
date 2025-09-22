#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp> // desired force
#include <nav_msgs/msg/odometry.hpp>

namespace gazebo {

class FlapPlugin : public ModelPlugin {
public:
  FlapPlugin() {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  }

  ~FlapPlugin() {
    if (node_) rclcpp::shutdown();
  }

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
    model_ = _model;

    // Get link
    std::string link_name = "body_link";
    if (_sdf->HasElement("link_name"))
        link_name = _sdf->Get<std::string>("link_name");

    link_ = model_->GetLink(link_name);
    if (!link_) {
      RCLCPP_ERROR(rclcpp::get_logger("flap_plugin"), 
                  "Could not find link [%s]!", link_name.c_str());
      return;
    }

    node_ = std::make_shared<rclcpp::Node>("flap_plugin_node");

    // Subscribers
    force_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
      "/desired_force", 10,
      std::bind(&FlapPlugin::forceCallback, this, std::placeholders::_1));

    // Odometry publisher
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    last_update_time_ = node_->now();

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&FlapPlugin::OnUpdate, this));

    RCLCPP_INFO(node_->get_logger(), "FlapPlugin initialized");
  }

private:
  void forceCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    desired_force_.X() = msg->x;
    desired_force_.Y() = msg->y;
    desired_force_.Z() = msg->z;
    RCLCPP_DEBUG(node_->get_logger(), "Received desired force [%f,%f,%f]",
                 msg->x, msg->y, msg->z);
  }

  void OnUpdate() {
    auto current_time = node_->now();
    double dt = (current_time - last_update_time_).seconds();

    if (link_) {
      // Transform world force into link local frame
      auto pose = link_->WorldPose();
      auto rot = pose.Rot();

      // Apply force at COM
      RCLCPP_INFO(node_->get_logger(), "Force Z: %f", desired_force_.Z());
      link_->AddForce(desired_force_);

      RCLCPP_INFO(node_->get_logger(),
                  "Mass=%.3f, Gravity=%.3f, Applied=%.3f",
                  link_->GetInertial()->Mass(),
                  (link_->GetInertial()->Mass() * link_->GetWorld()->Gravity()).Z(),
                  desired_force_.Z());
            
      // debug log
      // Net force at the link’s center of mass
      ignition::math::Vector3d force = link_->WorldForce();

      // Net torque
      ignition::math::Vector3d torque = link_->WorldTorque();

      RCLCPP_INFO(node_->get_logger(),
                  "Net force=[%.3f, %.3f, %.3f], torque=[%.3f, %.3f, %.3f]",
                  force.X(), force.Y(), force.Z(),
                  torque.X(), torque.Y(), torque.Z());
    }

    // Publish odometry
    auto pose = model_->WorldPose();
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "body_link";
    odom_msg.pose.pose.position.x = pose.Pos().X();
    odom_msg.pose.pose.position.y = pose.Pos().Y();
    odom_msg.pose.pose.position.z = pose.Pos().Z();
    odom_msg.pose.pose.orientation.x = pose.Rot().X();
    odom_msg.pose.pose.orientation.y = pose.Rot().Y();
    odom_msg.pose.pose.orientation.z = pose.Rot().Z();
    odom_msg.pose.pose.orientation.w = pose.Rot().W();

    auto linear_vel = link_->WorldLinearVel();
    odom_msg.twist.twist.linear.x = linear_vel.X();
    odom_msg.twist.twist.linear.y = linear_vel.Y();
    odom_msg.twist.twist.linear.z = linear_vel.Z();

    auto angular_vel = link_->WorldAngularVel();
    odom_msg.twist.twist.angular.x = angular_vel.X();
    odom_msg.twist.twist.angular.y = angular_vel.Y();
    odom_msg.twist.twist.angular.z = angular_vel.Z();

    odom_pub_->publish(odom_msg);

    last_update_time_ = current_time;
    rclcpp::spin_some(node_);
  }

  physics::ModelPtr model_;
  physics::LinkPtr link_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr force_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  event::ConnectionPtr update_connection_;
  rclcpp::Time last_update_time_;
  ignition::math::Vector3d desired_force_;
};

GZ_REGISTER_MODEL_PLUGIN(FlapPlugin)

} // namespace gazebo
