#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class Navigator : public rclcpp::Node {
public:
  Navigator(): Node("navigator") {
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
    timer_ = create_wall_timer(std::chrono::seconds(2), std::bind(&Navigator::send_target, this));
  }
private:
  void send_target() {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.pose.position.x = 100.0;
    msg.pose.position.y = 100.0;
    msg.pose.position.z = 100.0;
    msg.pose.orientation.w = 1.0;
    RCLCPP_INFO(get_logger(), "Publishing target (100,100,100)");
    pub_->publish(msg);
  }
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigator>());
  rclcpp::shutdown();
  return 0;
}
