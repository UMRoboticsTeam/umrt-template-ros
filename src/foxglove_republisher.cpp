/*
 * Copyright 2024 Edcel Abanto, University of Manitoba Robotics Team
 * 
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 * 
 * Created on 2026-29-05 by ea.
 */

#include <rclcpp/rclcpp.hpp>
#include "umrt-ros-poe-cam/foxglove_republisher.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FoxgloveRepublisher>());
    rclcpp::shutdown();
    return 0;
}