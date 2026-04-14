#ifndef MOTIONCTRL_HPP
#define MOTIONCTRL_HPP

#include <memory>
#include <vector>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class MotionControlNode : public rclcpp::Node
{
public:
    MotionControlNode();

private:
    // Methods
    void checkCollision();
    void updateTwist();
    void execute();

    // Action callbacks
    rclcpp_action::GoalResponse navHandleGoal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal);

    rclcpp_action::CancelResponse navHandleCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle);

    void navHandleAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle);

    // Other callbacks
    void pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future);
    void odomCallback(const nav_msgs::msg::Odometry & msg);
    void lidarCallback(const sensor_msgs::msg::LaserScan & msg);

    // Client
    rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr plan_client_;

    // Action server
    rclcpp_action::Server<nav2_msgs::action::NavigateToPose>::SharedPtr action_server_;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;

    // Action handle
    std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle_;

    // Data
    nav_msgs::msg::Odometry odom_;
    nav_msgs::msg::Path planned_path_;
    sensor_msgs::msg::LaserScan laser_scan_;

    bool collision_detected_;
    bool collision_latched_ = false;   // nová proměnná
    bool goal_reached_;
    bool odom_received_;
    bool rotate_in_place_mode_;
    double rotate_exit_threshold_;
    double rotate_enter_threshold_;
};

#endif // MOTIONCTRL_HPP