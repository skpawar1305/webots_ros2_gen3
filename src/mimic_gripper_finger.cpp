#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class GripperMimicNode : public rclcpp::Node {
public:
  GripperMimicNode() : Node("gripper_mimic_node") {
    // Change these topic names if your setup uses different ones
    right_joint_cmd_topic_ = "/right_finger_controller/commands";
    left_joint_name_ = "left_finger_joint";
    left_joint_position_ = 0.0;
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&GripperMimicNode::joint_state_callback, this, std::placeholders::_1));
    right_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(right_joint_cmd_topic_, 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&GripperMimicNode::publish_right_cmd, this));
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == left_joint_name_) {
        left_joint_position_ = msg->position[i];
        break;
      }
    }
  }

  void publish_right_cmd() {
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.resize(1);
    cmd.data[0] = left_joint_position_;
    right_cmd_pub_->publish(cmd);
  }

  std::string right_joint_cmd_topic_;
  std::string left_joint_name_;
  double left_joint_position_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperMimicNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}