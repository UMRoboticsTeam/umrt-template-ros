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
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "umrt-ros-poe-cam/foxglove_republisher.hpp"


FoxgloveRepublisher::FoxgloveRepublisher() : Node("foxglove_republisher")
{
    // Configure the Best Effort QoS for the Subscriber and Publisher
    rmw_qos_profile_t sub_qos = rmw_qos_profile_sensor_data; 
    sub_qos.depth = 1;
    rmw_qos_profile_t pub_qos = rmw_qos_profile_sensor_data;
    pub_qos.depth = 1;

    // Create the Subscription using the 'Foxglove' compressed video transport
    // image_transport will transparently decode the stream into a raw sensor_msgs::msg::Image
    sub_ = image_transport::create_subscription(
        this,
        "in", // Input Topic 
        [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
            pub_.publish(msg);
        },
        "foxglove",
        sub_qos
    );

    // Create the Publisher
    pub_ = image_transport::create_publisher(
        this,
        "out",  //  Output Topic
        pub_qos
    );

    RCLCPP_INFO(this->get_logger(), "Foxglove Compressed Video to Raw Image Republisher running.");
}