#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "elmo_motor_bridge/msg/motor_cmd.hpp" // No error, IDE include path problem.

class JoyCtrl : public rclcpp::Node
{
  public:
    JoyCtrl() : Node("joy_ctrl_node")
    {
      // Rigest the subscriber.
      joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoyCtrl::joy_callback, this, std::placeholders::_1));
      
      // Rigest the publisher.
      cmd_publisher_ = this->create_publisher<elmo_motor_bridge::msg::MotorCmd>("motor_cmd", 10);

    }

  private:
    // Define the subscriber.
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;

    // Define the publisher. 
    rclcpp::Publisher<elmo_motor_bridge::msg::MotorCmd>::SharedPtr cmd_publisher_;


    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
        elmo_motor_bridge::msg::MotorCmd mc;
        // mc.current = msg->axes[4] * 1000;
        mc.current = 800;
        mc.op_mode = 10;

        // Publish the cmd.
        cmd_publisher_->publish(mc); 
    }

};