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

/**
 *  MobileNetPublsherNode Class
 */
class MobileNetPublisherNode : public rclcpp::Node {
public:
    MobileNetPublisherNode() : Node("mobilenet_publisher_node") {
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

private: 

    // --- PIPELINE SETUP FUNCTIONS ---
    void setupCameraPipeline(dai::Pipeline& pipeline);
    void setupImuPipeline(dai::Pipeline& pipeline);
    void setupDepthPipeline(dai::Pipeline& pipeline);
    void setupOdomPipeline(dai::Pipeline& pipeline);

    // --- BRIDGE PUBLISHER SETUP FUNCTIONS ---
    void setupVideoPublishers();
    void setupTelemetryPublishers();

    // --- SHARED HARDWARE POINTERS ---
    std::shared_ptr<dai::Device> device_;
    std::shared_ptr<dai::Pipeline> pipeline_; 
    
    // Nodes that need to be tracked across setup scopes
    std::shared_ptr<dai::node::Camera> color_cam_;
    std::shared_ptr<dai::node::VideoEncoder> video_enc_;
    std::shared_ptr<dai::node::IMU> imu_node_;
    std::shared_ptr<dai::node::Camera> mono_left;
    std::shared_ptr<dai::node::Camera> mono_right;
    std::shared_ptr<dai::node::StereoDepth> stereo_depth_;
    std::shared_ptr<dai::node::BasaltVIO> odom_;

    // Queues
    std::shared_ptr<dai::MessageQueue> encoded_q_;
    std::shared_ptr<dai::MessageQueue> raw_q_;
    std::shared_ptr<dai::MessageQueue> imu_q_;
    std::shared_ptr<dai::MessageQueue> depth_q_;
    std::shared_ptr<dai::MessageQueue> odom_q_;

    // Bridge Publishers (Using generic definitions to match your structures)
    // std::unique_ptr<depthai_bridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData>> imu_pub_;
    // std::unique_ptr<depthai_bridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>> depth_pub_;
    rclcpp::Publisher<foxglove_msgs::msg::CompressedVideo>::SharedPtr encoded_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_; 
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    sensor_msgs::msg::CameraInfo depth_info_msg_;

    // The math engine from the bridge
    std::shared_ptr<depthai_bridge::ImuConverter> imu_conv_;
    std::shared_ptr<depthai_bridge::ImageConverter> depth_conv_;
    std::shared_ptr<depthai_bridge::TransformDataConverter> odom_conv_;

};

void MobileNetPublisherNode::setupCameraPipeline(dai::Pipeline& pipeline) {
    color_cam_ = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    video_enc_ = pipeline.create<dai::node::VideoEncoder>();

    auto script_node = pipeline.create<dai::node::Script>();

    video_enc_->setDefaultProfilePreset(30, dai::VideoEncoderProperties::Profile::H264_MAIN);

    //  For Encoder Stream
    auto cam_out = color_cam_->requestOutput({1920, 1080}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 30);
    cam_out->link(video_enc_->input);

    //  For Raw Stream
    // auto raw_out = color_cam_->requestOutput({640, 360}, dai::ImgFrame::Type::BGR888i, dai::ImgResizeMode::CROP, 30);
    cam_out->link(script_node->inputs["in"]);

    script_node->setScript(R"(
        import time
        frame_count = 0
        while True:
            frame = node.io['in'].get()

            # Forwave for ever 3rd frame = 10 FPS
            if frame_count % 3 == 0:
                node.io['out'].send(frame)

            frame_count += 1
    )");

    encoded_q_ = video_enc_->out.createOutputQueue(30, false);
    raw_q_ = script_node->outputs["out"].createOutputQueue(2, false);
    // raw_q_ = raw_out->createOutputQueue(2, false);
}

void MobileNetPublisherNode::setupImuPipeline(dai::Pipeline& pipeline) {
    imu_node_ = pipeline.create<dai::node::IMU>();
    imu_node_->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 200);
    // imu_node_->setBatchReportThreshold(1);
    // imu_node_->setMaxBatchReports(10);
    imu_node_->setBatchReportThreshold(5);
    imu_node_->setMaxBatchReports(20);
    
    imu_q_ = imu_node_->out.createOutputQueue(8, false);
}

void MobileNetPublisherNode::setupDepthPipeline(dai::Pipeline& pipeline) {

    stereo_depth_ = pipeline.create<dai::node::StereoDepth>();
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

    raw_pub_ = this->create_publisher<sensor_msgs::msg::Image>("raw_video", best_effort_qos);

    raw_q_->addCallback([this](std::shared_ptr<dai::ADatatype> data) {
        auto daiMsg = std::dynamic_pointer_cast<dai::ImgFrame>(data);
        if (daiMsg) {
            sensor_msgs::msg::Image msg;
            msg.header.stamp = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(daiMsg->getTimestamp().time_since_epoch()).count());
            msg.header.frame_id = "oakd_camera";
            msg.height = daiMsg->getHeight();
            msg.width = daiMsg->getWidth();
            msg.encoding = "nv12"; // Note: Ensure daiMsg data matches this format
            msg.step = msg.width;
            msg.data.assign(daiMsg->getData().begin(), daiMsg->getData().end());
            raw_pub_->publish(std::move(msg));
        }
    });
}

void MobileNetPublisherNode::setupTelemetryPublishers() {

    //  Initialize Publishers
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 30);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("stereo/depth", 10);
    // odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 30);
    depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("stereo/camera_info", 10);

    //  Initialize the Bridge Converter to handle the math and sync
    depthai_bridge::ImuSyncMethod imuMode = depthai_bridge::ImuSyncMethod::COPY;
    imu_conv_ = std::make_shared<depthai_bridge::ImuConverter>("oakd_camera", imuMode);
    depth_conv_ = std::make_shared<depthai_bridge::ImageConverter>("oakd_camera", false);
    // Explicitly provide parent frame, child frame, and the optional timestamp flag
    // odom_conv_ = std::make_shared<depthai_bridge::TransformDataConverter>("odom", "oakd_camera", false);

    //  Depth Cam Info
    dai::CalibrationHandler calibData = device_->readCalibration();
    depth_info_msg_ = depth_conv_->calibrationToCameraInfo(calibData, dai::CameraBoardSocket::CAM_C, 640, 400);
    depth_info_msg_.header.frame_id = "oakd_camera";

    //  IMU Callback - Pushes data through the converter
    imu_q_->addCallback([this](std::shared_ptr<dai::ADatatype> data) {
        auto daiMsg = std::dynamic_pointer_cast<dai::IMUData>(data);
        if (daiMsg) {
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

    //  Odometry Callback - Pushes data through converter 
    // odom_q_->addCallback([this](std::shared_ptr<dai::ADatatype> data) {
    //     auto daiMsg = std::dynamic_pointer_cast<dai::TransformData>(data);
    //     if (daiMsg) {
    //         std::deque<nav_msgs::msg::Odometry> rosMsgs;

    //         //  Convert to ROS Odom message 
    //         odom_conv_->toRosMsg(daiMsg, rosMsgs);

    //         //  Publish
    //         for (auto& msg : rosMsgs) {
    //             odom_pub_->publish(std::move(msg));
    //         }
    //     }
    // });

}

//  Main - Spin the Node
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MobileNetPublisherNode>());
    rclcpp::shutdown();
    return 0;
}