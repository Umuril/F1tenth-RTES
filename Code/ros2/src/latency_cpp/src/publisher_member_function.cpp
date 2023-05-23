#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

#define PACKAGE_NAME "latency_cpp"

#define TOPIC_NAME PACKAGE_NAME "_topic"
#define QOS_HISTORY_DEPTH 10

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node(PACKAGE_NAME "_pub_node") {
    _count = 0;
    _publisher = this->create_publisher<std_msgs::msg::String>(TOPIC_NAME, QOS_HISTORY_DEPTH);
    _timer = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
  size_t _count;

  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(_count++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    _publisher->publish(message);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
