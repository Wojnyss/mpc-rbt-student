#include "Localization.hpp"
#include "mpc_rbt_simulator/RobotConfig.hpp"

#include <cmath>

LocalizationNode::LocalizationNode()
    : rclcpp::Node("localization_node"),
      last_time_(this->get_clock()->now()) {

    odometry_.header.frame_id = "odom";
    odometry_.child_frame_id = "base_link";

    odometry_.pose.pose.position.x = -0.5;
    odometry_.pose.pose.position.y = 0.0;
    odometry_.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    odometry_.pose.pose.orientation = tf2::toMsg(q);

    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        10,
        std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1)
    );

    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "Localization node started.");
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    if (msg.velocity.size() < 2) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "JointState does not contain at least two wheel velocities."
        );
        return;
    }

    auto current_time = this->get_clock()->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if (dt <= 0.0) {
        return;
    }

    updateOdometry(msg.velocity[0], msg.velocity[1], dt);
    publishOdometry();
    publishTransform();
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {
    // Uprav název WHEEL_RADIUS podle skutečného názvu v RobotConfig.hpp,
    // pokud by build hlásil, že taková konstanta neexistuje.
    double linear =
        robot_config::WHEEL_RADIUS * (right_wheel_vel + left_wheel_vel) / 2.0;

    double angular =
        robot_config::WHEEL_RADIUS * (-right_wheel_vel + left_wheel_vel) /
        (2.0 * robot_config::HALF_DISTANCE_BETWEEN_WHEELS);

    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);

    double roll, pitch, theta;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta);

    theta = std::atan2(std::sin(theta), std::cos(theta));

    odometry_.pose.pose.position.x += linear * std::cos(theta) * dt;
    odometry_.pose.pose.position.y += linear * std::sin(theta) * dt;

    theta += angular * dt;
    theta = std::atan2(std::sin(theta), std::cos(theta));

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    odometry_.pose.pose.orientation = tf2::toMsg(q);

    odometry_.twist.twist.linear.x = linear;
    odometry_.twist.twist.linear.y = 0.0;
    odometry_.twist.twist.linear.z = 0.0;
    odometry_.twist.twist.angular.x = 0.0;
    odometry_.twist.twist.angular.y = 0.0;
    odometry_.twist.twist.angular.z = angular;
}

void LocalizationNode::publishOdometry() {
    odometry_.header.stamp = this->get_clock()->now();
    odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publishTransform() {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    t.transform.translation.x = odometry_.pose.pose.position.x;
    t.transform.translation.y = odometry_.pose.pose.position.y;
    t.transform.translation.z = odometry_.pose.pose.position.z;
    t.transform.rotation = odometry_.pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
}
