#include <cstring>
#include <memory>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

#define PACKAGE_NAME "latency_cpp"

#define PING_TOPIC_NAME PACKAGE_NAME "_pings"
#define PONG_TOPIC_NAME PACKAGE_NAME "_pongs"
#define QOS_HISTORY_DEPTH 10

#define MAX_PAYLOAD_LENGTH 511 * 1024

class PingNode : public rclcpp::Node
{
public:
  PingNode() : Node(PACKAGE_NAME "_ping") {
    RCLCPP_INFO(this->get_logger(), "PING TOPIC: %s", PING_TOPIC_NAME);
    _publisher = this->create_publisher<std_msgs::msg::String>(PING_TOPIC_NAME, QOS_HISTORY_DEPTH);
    _subscription = this->create_subscription<std_msgs::msg::String>(PONG_TOPIC_NAME, QOS_HISTORY_DEPTH, std::bind(&PingNode::receive_pong, this, _1));

    _clk = new rclcpp::Clock();
    _timer = this->create_wall_timer(500ms, std::bind(&PingNode::start_up, this));
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscription;
  rclcpp::Clock * _clk;
  rclcpp::TimerBase::SharedPtr _timer;

  void start_up() {
    this->_timer->cancel();
    this->send_ping();
  }

  void send_ping() {
    rclcpp::Time start_time = _clk->now();
    unsigned long long start_ns = (unsigned long long) start_time.nanoseconds();
    
    auto message = std_msgs::msg::String();
    int len = rand() % MAX_PAYLOAD_LENGTH;
    std::string payload(len, '*');
    message.data = std::to_string(start_ns) + " " + std::to_string(len) + " " + payload;
    RCLCPP_INFO(this->get_logger(), "Publishing payload of '%d'", len);
    _publisher->publish(message);
  }

  void receive_pong(const std_msgs::msg::String::SharedPtr msg) {
    char * response = &msg->data[0];
    char * time = strtok(response, " ");
    long start_ns = atol(time);
    char * length = strtok(NULL, " ");
    int len = atoi(length);

    rclcpp::Time end_time = _clk->now();
    unsigned long long end_ns = (unsigned long long) end_time.nanoseconds();

    RCLCPP_INFO(this->get_logger(), "[__TIMER__] %llu %d", end_ns - start_ns, len);
    this->send_ping();
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PingNode>());
  rclcpp::shutdown();
  return 0;
}
