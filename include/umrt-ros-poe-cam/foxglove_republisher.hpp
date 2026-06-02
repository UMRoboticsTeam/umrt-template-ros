/*
 * Copyright 2024 Edcel Abanto, University of Manitoba Robotics Team
 * 
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 * 
 * Created on 2026-29-05 by ea.
 */

#ifndef UMRT_ROS_POE_CAM__FOXGLOVE_REPUBLISHER_HPP
#define UMRT_ROS_POE_CAM__FOXGLOVE_REPUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

/**
 * @class FoxgloveRepublisher
 * @brief A ROS2 node that decompresses Foxglove-encoded video streams into Raw Images.
 *
 * @details This node utilizes image_transport to subscribe to an incoming compressed 
 * "foxglove" transport stream, automatically decodes it, and republishes the 
 * resulting raw sensor_msgs::msg::Image. It explicitly enforces Best Effort 
 * (sensor_data) Quality of Service (QoS) on both the subscriber and publisher 
 * to ensure minimum latency for downstream Visual Inertial Odometry (VIO) 
 * and mapping algorithms.
 *
 * @note This C++ implementation bypasses command-line remapping bugs associated with 
 * the default ros2 image_transport republish tool when handling custom QoS profiles.
 */
class FoxgloveRepublisher : public rclcpp::Node
{

public:

    /**
     * Initializes a FoxgloveRepublisher.
     */
    FoxgloveRepublisher();

private:

    /**
     * Image Transport Suscriber, which subscribes to get the image frames to decode and republish
     */
    image_transport::Subscriber sub_;


    /**
     * Image Transport Suscriber,whcih publishes the intended image frames
     */
    image_transport::Publisher pub_;
};

#endif //UMRT_ROS_POE_CAM__FOXGLOVE_REPUBLISHER_HPP