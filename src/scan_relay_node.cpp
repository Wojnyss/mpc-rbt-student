#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanRelayNode : public rclcpp::Node {
public:
    ScanRelayNode() : Node("scan_relay_node") {
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/tiago_base/Hokuyo_URG_04LX_UG01",
            10,
            std::bind(&ScanRelayNode::callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Scan relay node started.");
    }

private:
    void callback(const sensor_msgs::msg::LaserScan & msg) {
        publisher_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanRelayNode>());
    rclcpp::shutdown();
    return 0;
}
