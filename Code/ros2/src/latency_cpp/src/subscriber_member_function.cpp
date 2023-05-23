// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

#define PACKAGE_NAME "latency_cpp"

#define TOPIC_NAME PACKAGE_NAME "_topic"
#define QOS_HISTORY_DEPTH 10

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber() : Node(PACKAGE_NAME "_sub_node") {
    auto callback = std::bind(&MinimalSubscriber::topic_callback, this, _1);
    _subscription = this->create_subscription<std_msgs::msg::String>(TOPIC_NAME, QOS_HISTORY_DEPTH, callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscription;
  
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
