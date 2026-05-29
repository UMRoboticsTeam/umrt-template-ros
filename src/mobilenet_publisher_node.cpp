//
// Created by ea on 2026-29-05.
//

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
#include "umrt-ros-poe-cam/mobilenet_publisher.hpp"


MobileNetPublisherNode::MobileNetPublisherNode() : Node("mobilenet_publisher_node") {
    // 1. Initialize core hardware connection
    device_ = std::make_shared<dai::Device>();
    pipeline_ = std::make_shared<dai::Pipeline>(device_);

    // 2. Build the hardware graph via modular functions
    setupCameraPipeline(*pipeline_);
    setupImuPipeline(*pipeline_);
    setupDepthPipeline(*pipeline_);
    // setupOdomPipeline(*pipeline_);

    // 3. Fire up the physical device
    pipeline_->start();

    RCLCPP_INFO(this->get_logger(), "Pipeline running: %s", pipeline_->isRunning() ? "yes" : "no");

    // 4. Bind the hardware queues to ROS 2 bridge publishers
    setupVideoPublishers();
    setupTelemetryPublishers();

}

void MobileNetPublisherNode::setupCameraPipeline(dai::Pipeline& pipeline) {
    color_cam_ = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    video_enc_ = pipeline.create<dai::node::VideoEncoder>();

    // auto script_node = pipeline.create<dai::node::Script>();

    video_enc_->setDefaultProfilePreset(30, dai::VideoEncoderProperties::Profile::H264_MAIN);

    //  For Encoder Stream
    auto cam_out = color_cam_->requestOutput({1920, 1080}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 30);
    cam_out->link(video_enc_->input);

    encoded_q_ = video_enc_->out.createOutputQueue(30, false);
    
}

void MobileNetPublisherNode::setupImuPipeline(dai::Pipeline& pipeline) {
    imu_node_ = pipeline.create<dai::node::IMU>();
    imu_node_->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 200);
    imu_node_->setBatchReportThreshold(1);
    imu_node_->setMaxBatchReports(10);
    
    imu_q_ = imu_node_->out.createOutputQueue(8, false);

}

void MobileNetPublisherNode::setupDepthPipeline(dai::Pipeline& pipeline) {

    stereo_depth_ = pipeline.create<dai::node::StereoDepth>();

    stereo_depth_->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::ROBOTICS);

    // mono_left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    // mono_right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    // Throttle the VIO/Depth cameras to 15 FPS to guarantee bandwidth for the 200Hz IMU
    mono_left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, 15);
    mono_right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::nullopt, 15);

    auto left_out = mono_left->requestOutput({640, 400}, dai::ImgFrame::Type::RAW8, dai::ImgResizeMode::CROP, 15);
    auto right_out = mono_right->requestOutput({640, 400}, dai::ImgFrame::Type::RAW8, dai::ImgResizeMode::CROP, 15);

    left_out->link(stereo_depth_->left);
    right_out->link(stereo_depth_->right);

    depth_q_ = stereo_depth_->depth.createOutputQueue(2, false);

}

void MobileNetPublisherNode::setupOdomPipeline(dai::Pipeline& pipeline) {
 
    odom_ = pipeline.create<dai::node::BasaltVIO>();

    odom_->setImuUpdateRate(200);

    // mono_left->requestOutput({640, 400}, dai::ImgFrame::Type::RAW8, dai::ImgResizeMode::CROP, 30)->link(odom_->left);
    // mono_right->requestOutput({640, 400}, dai::ImgFrame::Type::RAW8, dai::ImgResizeMode::CROP, 30)->link(odom_->right);
    stereo_depth_->syncedLeft.link(odom_->left);
    stereo_depth_->syncedRight.link(odom_->right);
    imu_node_->out.link(odom_->imu); 

    odom_q_ = odom_->transform.createOutputQueue(8, false);

}

void MobileNetPublisherNode::setupVideoPublishers() {

    rclcpp::QoS best_effort_qos(10);
    best_effort_qos.best_effort();
    best_effort_qos.durability_volatile();

    encoded_pub_ = this->create_publisher<foxglove_msgs::msg::CompressedVideo>("encoded_video", best_effort_qos);

    encoded_q_->addCallback([this](std::shared_ptr<dai::ADatatype> data) {
        auto daiMsg = std::dynamic_pointer_cast<dai::EncodedFrame>(data);
        if (daiMsg) {
            foxglove_msgs::msg::CompressedVideo msg;
            msg.timestamp = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(daiMsg->getTimestamp().time_since_epoch()).count());
            msg.frame_id = "oakd_camera";
            msg.format = "h264";
            msg.data.assign(daiMsg->getData().begin(), daiMsg->getData().end());
            encoded_pub_->publish(std::move(msg));
        }
    });

}

void MobileNetPublisherNode::setupTelemetryPublishers() {

    //  Initialize Publishers
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 30);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("stereo/depth", 10);
    depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("stereo/camera_info", 10);

    //  Initialize the Bridge Converter to handle the math and sync
    depthai_bridge::ImuSyncMethod imuMode = depthai_bridge::ImuSyncMethod::COPY;
    imu_conv_ = std::make_shared<depthai_bridge::ImuConverter>("oakd_camera", imuMode);
    depth_conv_ = std::make_shared<depthai_bridge::ImageConverter>("oakd_camera", false);

    //  Depth Cam Info
    dai::CalibrationHandler calibData = device_->readCalibration();
    depth_info_msg_ = depth_conv_->calibrationToCameraInfo(calibData, dai::CameraBoardSocket::CAM_C, 640, 400);
    depth_info_msg_.header.frame_id = "oakd_camera";

    //  IMU Callback - Pushes data through the converter
    imu_q_->addCallback([this](std::shared_ptr<dai::ADatatype> data) {
        auto daiMsg = std::dynamic_pointer_cast<dai::IMUData>(data);
        if (daiMsg) {

            if (imu_warmup_count_ < MAX_WARMUP_COUNT) {
                imu_warmup_count_++;
                return; // Skip
            }

            std::deque<sensor_msgs::msg::Imu> rosMsgs;            
            // Let the bridge do the complex synchronization and math
            imu_conv_->toRosMsg(daiMsg, rosMsgs);

            // Publish the resulting valid ROS messages
            for (auto& msg : rosMsgs) {
                imu_pub_->publish(std::move(msg));
            }
        }
    });

    //  Depth Callback - Pushes data through the converter
    depth_q_->addCallback([this](std::shared_ptr<dai::ADatatype> data) {
        auto daiMsg = std::dynamic_pointer_cast<dai::ImgFrame>(data);
        if (daiMsg) {

            if (depth_warmup_count_ < MAX_WARMUP_COUNT) {
                depth_warmup_count_++;
                return; // Skip
            }

            std::deque<sensor_msgs::msg::Image> rosMsgs;
            // Convert to ROS Image message
            depth_conv_->toRosMsg(daiMsg, rosMsgs);
            
            // Publish
            for (auto& msg : rosMsgs) {
                depth_pub_->publish(std::move(msg));
                
                depth_info_msg_.header.stamp = msg.header.stamp;
                depth_info_pub_->publish(depth_info_msg_);
            }
        }
    });

}
