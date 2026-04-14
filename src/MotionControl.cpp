#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

using std::placeholders::_1;
using std::placeholders::_2;

MotionControlNode::MotionControlNode()
    : rclcpp::Node("motion_control_node"),
      collision_detected_(false),
      collision_latched_(false),
      odom_received_(false),
      goal_reached_(false),
      rotate_in_place_mode_(false),
      rotate_enter_threshold_(1.2),
      rotate_exit_threshold_(0.35)
{
    // Subscribers for odometry and laser scans
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        10,
        std::bind(&MotionControlNode::odomCallback, this, _1));

    lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        10,
        std::bind(&MotionControlNode::lidarCallback, this, _1));

    // Publisher for robot control
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10);

    // Client for path planning
    plan_client_ = this->create_client<nav_msgs::srv::GetPlan>("/plan_path");

    // Action server
    action_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
        this,
        "go_to_goal",
        std::bind(&MotionControlNode::navHandleGoal, this, _1, _2),
        std::bind(&MotionControlNode::navHandleCancel, this, _1),
        std::bind(&MotionControlNode::navHandleAccepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "Motion control node started.");

    odom_.header.frame_id = "map";
    odom_.child_frame_id = "base_link";

    odom_.pose.pose.position.x = -0.5;
    odom_.pose.pose.position.y = 0.0;
    odom_.pose.pose.position.z = 0.0;

    odom_.pose.pose.orientation.x = 0.0;
    odom_.pose.pose.orientation.y = 0.0;
    odom_.pose.pose.orientation.z = 0.0;
    odom_.pose.pose.orientation.w = 1.0;


    // Connect to path planning service server
    while (rclcpp::ok() && !plan_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Waiting for path planning service...");
    }

    if (!rclcpp::ok()) {
        return;
    }
}

void MotionControlNode::checkCollision()
{
    bool obstacle_now = false;

    if (laser_scan_.ranges.empty()) {
        return;
    }

    const double forward_angle = 20.0 * M_PI / 180.0;
    const double stop_threshold = 0.25; // klidně snížit

    for (size_t i = 0; i < laser_scan_.ranges.size(); ++i) {
        const double angle =
            laser_scan_.angle_min + i * laser_scan_.angle_increment;

        if (std::abs(angle) > forward_angle) {
            continue;
        }

        float r = laser_scan_.ranges[i];

        if (!std::isfinite(r)) continue;
        if (r < laser_scan_.range_min || r > laser_scan_.range_max) continue;

        if (r < stop_threshold) {
            obstacle_now = true;
            break;
        }
    }

    // detekce hrany (jen při změně)
    if (obstacle_now && !collision_detected_) {
        RCLCPP_WARN(this->get_logger(), "Obstacle detected -> STOP");
    }

    if (!obstacle_now && collision_detected_) {
        RCLCPP_INFO(this->get_logger(), "Obstacle cleared");
    }

    collision_detected_ = obstacle_now;
}

void MotionControlNode::updateTwist()
{
    if (!goal_handle_) {
        return;
    }

    if (planned_path_.poses.empty()) {
        return;
    }

    if (goal_reached_) {
        geometry_msgs::msg::Twist stop;
        twist_publisher_->publish(stop);
        return;
    }

    // Aktualni poloha robotu
    const double x = odom_.pose.pose.position.x;
    const double y = odom_.pose.pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(odom_.pose.pose.orientation, q);
    const double yaw = tf2::getYaw(q);

    // Najdeme nejblizsi bod na ceste
    size_t nearest_index = 0;
    double nearest_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < planned_path_.poses.size(); ++i) {
        const double dx = planned_path_.poses[i].pose.position.x - x;
        const double dy = planned_path_.poses[i].pose.position.y - y;
        const double dist = std::hypot(dx, dy);

        if (dist < nearest_dist) {
            nearest_dist = dist;
            nearest_index = i;
        }
    }

    const size_t lookahead_offset = 12;
    const size_t target_index =
        std::min(nearest_index + lookahead_offset, planned_path_.poses.size() - 1);

    const auto &target_pose = planned_path_.poses[target_index].pose;

    const double dx = target_pose.position.x - x;
    const double dy = target_pose.position.y - y;

    // Transformace do souradneho systemu robotu
    const double xr = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double yr = -std::sin(yaw) * dx + std::cos(yaw) * dy;

    // Vzdalenost ke koncovemu bodu
    const double goal_dx = planned_path_.poses.back().pose.position.x - x;
    const double goal_dy = planned_path_.poses.back().pose.position.y - y;
    const double dist_to_goal = std::hypot(goal_dx, goal_dy);

    if (dist_to_goal < 0.15) {
        geometry_msgs::msg::Twist stop;
        twist_publisher_->publish(stop);
        goal_reached_ = true;
        return;
    }

    // Uhel k cilovemu bodu v souradnicich robotu
    const double heading_error = std::atan2(yr, xr);

    geometry_msgs::msg::Twist twist;

    // Prvni vstup do rezimu rotace na miste:
    // pokud je prekazka pred robotem a cil je hodne bokem / za robotem.
    if (!rotate_in_place_mode_ && collision_detected_ &&
        std::abs(heading_error) >= rotate_enter_threshold_) {
        rotate_in_place_mode_ = true;
        RCLCPP_WARN(this->get_logger(), "Entering rotate-in-place mode.");
    }

    // Pokud jsme v rezimu rotace na miste, ignorujeme prekazku pred robotem
    // a dodelame otoceni, dokud neni heading error dost maly.
    if (rotate_in_place_mode_) {
        double w = 1.2 * heading_error;
        const double w_max = 1.0;
        w = std::clamp(w, -w_max, w_max);

        twist.linear.x = 0.0;
        twist.angular.z = w;
        twist_publisher_->publish(twist);

        if (std::abs(heading_error) < rotate_exit_threshold_) {
            rotate_in_place_mode_ = false;
            RCLCPP_INFO(this->get_logger(), "Leaving rotate-in-place mode.");
        }

        return;
    }

    // Pokud neni povolena rotace na miste a prekazka je stale pred robotem, zastav
    if (collision_detected_) {
        geometry_msgs::msg::Twist stop;
        twist_publisher_->publish(stop);
        return;
    }

    // Standardni pure pursuit-like regulace
    const double lookahead = std::max(std::hypot(xr, yr), 0.05);
    const double kappa = 2.0 * yr / (lookahead * lookahead);

    double v = 0.50;
    double w = kappa * v;

    const double v_max = 0.70;
    const double w_max = 1.50;

    v = std::clamp(v, 0.0, v_max);
    w = std::clamp(w, -w_max, w_max);

    if (std::abs(w) > 1.0) {
        v = 0.10;
    }

    twist.linear.x = v;
    twist.angular.z = w;

    twist_publisher_->publish(twist);
}

rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal)
{
    (void)uuid;

    if (goal_handle_) {
        RCLCPP_WARN(this->get_logger(), "Another goal is already active.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Received navigation goal: x=%.3f y=%.3f",
        goal->pose.pose.position.x,
        goal->pose.pose.position.y);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle)
{
    (void)goal_handle;

    geometry_msgs::msg::Twist stop;
    twist_publisher_->publish(stop);

    RCLCPP_WARN(this->get_logger(), "Navigation goal canceled.");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionControlNode::navHandleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle)
{
    goal_handle_ = goal_handle;
    goal_reached_ = false;
    collision_detected_ = false;
    collision_latched_ = false;
    rotate_in_place_mode_ = false;
    planned_path_.poses.clear();

    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();

    request->start.header.frame_id = "map";
    request->start.header.stamp = this->now();

    request->start.pose = odom_.pose.pose;

    request->goal = goal_handle->get_goal()->pose;
    request->tolerance = 0.1;

    plan_client_->async_send_request(
        request,
        std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1)
    );
}

void MotionControlNode::execute()
{
    auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
    auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();

    rclcpp::Rate loop_rate(10.0);

    while (rclcpp::ok()) {
        if (!goal_handle_) {
            return;
        }

        if (goal_handle_->is_canceling()) {
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);
            goal_handle_->canceled(result);
            goal_handle_.reset();
            planned_path_.poses.clear();

            RCLCPP_WARN(this->get_logger(), "Goal canceled during execution.");
            return;
        }

        if (collision_detected_ && !collision_latched_) {
          if (rotate_in_place_mode_) {
            loop_rate.sleep();
            continue;
          }

          collision_latched_ = true;

          geometry_msgs::msg::Twist stop;
          twist_publisher_->publish(stop);

          RCLCPP_ERROR(this->get_logger(), "Aborting goal due to obstacle");

          auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
          goal_handle_->abort(result);
          goal_handle_.reset();
          planned_path_.poses.clear();
          return;
        }

        if (goal_reached_) {
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);
            goal_handle_->succeed(result);
            goal_handle_.reset();
            planned_path_.poses.clear();

            RCLCPP_INFO(this->get_logger(), "Goal reached.");
            return;
        }

        if (!planned_path_.poses.empty()) {
            const double dx =
                planned_path_.poses.back().pose.position.x - odom_.pose.pose.position.x;
            const double dy =
                planned_path_.poses.back().pose.position.y - odom_.pose.pose.position.y;

            feedback->distance_remaining = std::hypot(dx, dy);
            goal_handle_->publish_feedback(feedback);
        }

        loop_rate.sleep();
    }
}

void MotionControlNode::pathCallback(
    rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future)
{
    auto response = future.get();

    if (response && !response->plan.poses.empty()) {
        planned_path_ = response->plan;

        RCLCPP_INFO(
            this->get_logger(),
            "Received path with %zu poses.",
            planned_path_.poses.size());

        std::thread(&MotionControlNode::execute, this).detach();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Planner returned empty path.");

        if (goal_handle_) {
            auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
            goal_handle_->abort(result);
            goal_handle_.reset();
        }
    }
}

void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry &msg)
{
    odom_ = msg;
    odom_received_ = true;


    updateTwist();
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan &msg)
{
    checkCollision();
    laser_scan_ = msg;
}