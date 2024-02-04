#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <cmath>
#include <bits/stdc++.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.h"
#include <std_msgs/msg/header.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "cslam_common_interfaces/msg/keyframe_odom.hpp"
#include "cslam_common_interfaces/msg/inter_robot_loop_closure.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class LoopClosurePublisher : public rclcpp::Node
{  
  public:
    LoopClosurePublisher()
    : Node("loop_closure_publisher"), count_(0)
    {

// SUBSCRIPTIONS

      // Subscription to the Blockchain topic
      subscription_blockchain_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/blockchain_approved_transformation", 100, std::bind(&LoopClosurePublisher::topic_blockchain_callback, this, _1));
      // Subscription to the Blockchain topic
      //subscription_no_blockchain_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/blockchain_transformation_C0", 100, std::bind(&LoopClosurePublisher::topic_no_blockchain_callback, this, _1));


// PUBLISHERS

      // Loop Closure Publisher
      publisher_ = this->create_publisher<cslam_common_interfaces::msg::InterRobotLoopClosure>("/cslam/inter_robot_loop_closure", 100);

    }

// SUBSCRIPTIONS FUNCTIONS

  private:
    void topic_blockchain_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const
    {
      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();

      message.robot0_keyframe_id = msg->data[0];
      message.robot0_id = msg->data[1] - 1;
      message.robot1_keyframe_id = msg->data[2];
      message.robot1_id = msg->data[3] - 1;
      message.transform.translation.x = msg->data[4];
      message.transform.translation.y = msg->data[5];
      message.success = true;

      publisher_-> publish(message);

    }

    private:
    void topic_no_blockchain_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const
    {
      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();

      message.robot0_keyframe_id = msg->data[8];
      message.robot0_id = msg->data[5] - 1;
      message.robot1_keyframe_id = msg->data[4];
      message.robot1_id = msg->data[1] - 1;
      message.transform.translation.x = msg->data[9];
      message.transform.translation.y = msg->data[10];
      message.success = true;

      publisher_-> publish(message);

    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_blockchain_;
    //rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_no_blockchain_;
    rclcpp::Publisher<cslam_common_interfaces::msg::InterRobotLoopClosure>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoopClosurePublisher>());
  rclcpp::shutdown();
  return 0;
}

