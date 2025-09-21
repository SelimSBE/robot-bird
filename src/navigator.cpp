#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

class Navigator : public rclcpp::Node {
public:
  Navigator(): Node("navigator") {
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("target_marker", 10);
    timer_ = create_wall_timer(std::chrono::seconds(2), std::bind(&Navigator::send_target, this));
  }

private:
  void send_target() {
    auto msg = geometry_msgs::msg::PoseStamped();
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.pose.position.x = 100.0;
    msg.pose.position.y = 100.0;
    msg.pose.position.z = 100.0;
    pose_pub_->publish(msg);

    // Publish visualization marker
    auto marker = visualization_msgs::msg::Marker();
    marker.header = msg.header;
    marker.ns = "target";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = msg.pose;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker_pub_->publish(marker);

    RCLCPP_INFO(get_logger(), "Publishing target (%f,%f,%f)", 
                msg.pose.position.x, 
                msg.pose.position.y, 
                msg.pose.position.z);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigator>());
  rclcpp::shutdown();
  return 0;
}
