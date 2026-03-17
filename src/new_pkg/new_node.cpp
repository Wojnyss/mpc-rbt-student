#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <chrono>
#include <functional>
#include <algorithm>

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_node")
    {
        // Parameters with default values from previous task
        this->declare_parameter("battery_max_voltage", 42.0);
        this->declare_parameter("battery_min_voltage", 32.0);

        name_publisher_ =
            this->create_publisher<std_msgs::msg::String>("node_name", 10);

        battery_publisher_ =
            this->create_publisher<std_msgs::msg::Float32>("battery_percentage", 10);

        battery_subscriber_ =
            this->create_subscription<std_msgs::msg::Float32>(
                "battery_voltage",
                10,
                std::bind(&MyNode::battery_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MyNode::publish_name, this));
    }

private:
    void publish_name()
    {
        std_msgs::msg::String msg;
        msg.data = this->get_name();
        name_publisher_->publish(msg);
    }

    void battery_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        const float voltage = msg->data;

        double max_v = this->get_parameter("battery_max_voltage").as_double();
        double min_v = this->get_parameter("battery_min_voltage").as_double();

        float percentage = 0.0f;

        if (max_v > min_v) {
            percentage = static_cast<float>((voltage - min_v) / (max_v - min_v) * 100.0);
            percentage = std::clamp(percentage, 0.0f, 100.0f);
        } else {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid parameters: battery_max_voltage must be greater than battery_min_voltage.");
        }

        std_msgs::msg::Float32 out_msg;
        out_msg.data = percentage;
        battery_publisher_->publish(out_msg);

        RCLCPP_INFO(this->get_logger(),
                    "Voltage: %.2f V, min: %.2f V, max: %.2f V -> %.1f %%",
                    voltage, min_v, max_v, percentage);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr name_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}
