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

    /**
     * Initializes a FoxgloveRepublisher.
     */
    FoxgloveRepublisher();

private:

    /**
     * Publishes the raw image message.
     *
     * @param msg raw image message 
     */
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    /**
     * Image Transport Suscriber, which subscribes to get the image frames to decode and republish
     */
    image_transport::Subscriber sub_;


    /**
     * Image Transport Suscriber,whcih publishes the intended image frames
     */
    image_transport::Publisher pub_;
};

#endif