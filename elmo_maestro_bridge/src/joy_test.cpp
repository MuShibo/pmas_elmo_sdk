/**
 * @file joy_test.cpp
 * @program FreeMan
 * @author Shibo Mu
 * @brief Single motor test with joystick data input.
 * @version 1.0
 * @date 2024
 */


#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <stdint.h>

#include "elmo_maestro_bridge/joy_test.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyCtrl>());
    rclcpp::shutdown();
    return 0;
}