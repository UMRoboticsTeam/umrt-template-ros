//
// Created by ea on 2026-29-05.
//

#include <rclcpp/rclcpp.hpp>
#include "umrt-ros-poe-cam/foxglove_republisher.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FoxgloveRepublisher>());
    rclcpp::shutdown();
    return 0;
}