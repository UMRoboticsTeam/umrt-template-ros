//
// Created by ea on 2026-29-05.
//

#ifndef UMRT_MOBILENET_PUBLISHER_HPP
#define UMRT_MOBILENET_PUBLISHER_HPP

#include <cstdio>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <foxglove_msgs/msg/compressed_video.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "depthai/depthai.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/TransformDataConverter.hpp"

class MobileNetPublisherNode : public rclcpp::Node
{

public:
    MobileNetPublisherNode();

private:

    void setupCameraPipeline(dai::Pipeline& pipeline);
    void setupImuPipeline(dai::Pipeline& pipeline);
    void setupDepthPipeline(dai::Pipeline& pipeline);
    void setupOdomPipeline(dai::Pipeline& pipeline);

    void setupVideoPublishers();
    void setupTelemetryPublishers();

    std::shared_ptr<dai::Device> device_;
    std::shared_ptr<dai::Pipeline> pipeline_;

    std::shared_ptr<dai::node::Camera> color_cam_;
    std::shared_ptr<dai::node::VideoEncoder> video_enc_;
    std::shared_ptr<dai::node::IMU> imu_node_;
    std::shared_ptr<dai::node::Camera> mono_left;
    std::shared_ptr<dai::node::Camera> mono_right;
    std::shared_ptr<dai::node::StereoDepth> stereo_depth_;
    std::shared_ptr<dai::node::BasaltVIO> odom_;

    std::shared_ptr<dai::MessageQueue> encoded_q_;
    std::shared_ptr<dai::MessageQueue> imu_q_;
    std::shared_ptr<dai::MessageQueue> depth_q_;
    std::shared_ptr<dai::MessageQueue> odom_q_;

    rclcpp::Publisher<foxglove_msgs::msg::CompressedVideo>::SharedPtr encoded_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_; 
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    sensor_msgs::msg::CameraInfo depth_info_msg_;

    std::shared_ptr<depthai_bridge::ImuConverter> imu_conv_;
    std::shared_ptr<depthai_bridge::ImageConverter> depth_conv_;
    std::shared_ptr<depthai_bridge::TransformDataConverter> odom_conv_;

    int imu_warmup_count_ = 0;
    int depth_warmup_count_ = 0;
    const int MAX_WARMUP_COUNT = 50;

};

#endif