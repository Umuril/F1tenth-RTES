#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

#define PACKAGE_NAME "latency_cpp"

#define PING_TOPIC_NAME PACKAGE_NAME "_pings"
#define PONG_TOPIC_NAME PACKAGE_NAME "_pongs"
#define QOS_HISTORY_DEPTH 10

class PongNode : public rclcpp::Node
{
public:
  PongNode() : Node(PACKAGE_NAME "_pong") {
    _subscription = this->create_subscription<std_msgs::msg::String>(PING_TOPIC_NAME, QOS_HISTORY_DEPTH, std::bind(&PongNode::receive_ping, this, _1));
    _publisher = this->create_publisher<std_msgs::msg::String>(PONG_TOPIC_NAME, QOS_HISTORY_DEPTH);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscription;
  
  void receive_ping(const std_msgs::msg::String::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    auto message = std_msgs::msg::String();
    message.data = msg->data;
    _publisher->publish(message);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PongNode>());
  rclcpp::shutdown();
  return 0;
}
