#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

class FoxgloveRepublisher : public rclcpp::Node
{
public:
    FoxgloveRepublisher() : Node("foxglove_republisher")
    {
        // 1. Configure the Best Effort QoS for the Subscriber
        rmw_qos_profile_t sub_qos = rmw_qos_profile_sensor_data; // Enforces BEST_EFFORT
        sub_qos.depth = 1;

        // 2. Configure the QoS for the raw output Publisher 
        // We set this to best-effort too so your downstream VIO node gets the lowest latency possible
        rmw_qos_profile_t pub_qos = rmw_qos_profile_sensor_data;
        pub_qos.depth = 1;

        // 3. Create the Subscription using the 'foxglove' transport hint
        // image_transport will transparently decode the stream into a raw sensor_msgs::msg::Image
        sub_ = image_transport::create_subscription(
            this,
            "in", // Base topic
            std::bind(&FoxgloveRepublisher::imageCallback, this, std::placeholders::_1),
            "foxglove",
            sub_qos
        );

        // 4. Create the Publisher
        pub_ = image_transport::create_publisher(
            this,
            "out", 
            pub_qos
        );

        RCLCPP_INFO(this->get_logger(), "Custom Best-Effort Foxglove Republisher running.");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        // Republish the freshly decoded raw image
        pub_.publish(msg);
    }

    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FoxgloveRepublisher>());
    rclcpp::shutdown();
    return 0;
}