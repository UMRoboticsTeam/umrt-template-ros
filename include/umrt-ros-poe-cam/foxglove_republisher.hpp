//
// Created by ea on 2026-29-05.
//

#ifndef UMRT_ROS_POE_CAM_HPP
#define UMRT_ROS_POE_CAM_HPP

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

class FoxgloveRepublisher : public rclcpp::Node
{

public:
    FoxgloveRepublisher();

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;
};

#endif