#include <chrono>
#include <functional>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>

#include "KeyboardControl.hpp"

using namespace std::chrono_literals;

KeyboardControlNode::KeyboardControlNode()
    : rclcpp::Node("keyboard_control_node")
{
    // Parametry rychlosti
    this->declare_parameter("linear_speed", 0.5);
    this->declare_parameter("angular_speed", 0.5);


    // Publisher na cmd_vel
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Nastaveni terminalu na non-blocking rezim
    tcgetattr(STDIN_FILENO, &old_termios_);
    struct termios new_termios = old_termios_;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

    // Timer
    timer_ = this->create_wall_timer(10ms, std::bind(&KeyboardControlNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Keyboard Control node started.");
    RCLCPP_INFO(this->get_logger(), "Use arrow keys to control the robot. Press Ctrl+C to quit.");
    RCLCPP_INFO(this->get_logger(), "linear_speed = %.2f, angular_speed = %.2f", linear_speed_, angular_speed_);
}

KeyboardControlNode::~KeyboardControlNode()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
}

void KeyboardControlNode::timerCallback()
{
    linear_speed_ = this->get_parameter("linear_speed").as_double();
    angular_speed_ = this->get_parameter("angular_speed").as_double();

    geometry_msgs::msg::Twist twist{};
    char c;

    fd_set readfds;
    struct timeval timeout;

    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int retval = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);

    if (retval > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
        if (read(STDIN_FILENO, &c, 1) == 1) {
            if (c == '\033') { // ESC sekvence pro sipky
                char seq[2];
                if (read(STDIN_FILENO, &seq, 2) != 2)
                    return;

                if (seq[0] == '[') {
                    switch (seq[1]) {
                        case 'A': // sipka nahoru
                            twist.linear.x = linear_speed_;
                            break;
                        case 'B': // sipka dolu
                            twist.linear.x = -linear_speed_;
                            break;
                        case 'C': // sipka doprava
                            twist.angular.z = -angular_speed_;
                            break;
                        case 'D': // sipka doleva
                            twist.angular.z = angular_speed_;
                            break;
                        default:
                            return;
                    }

                    twist_publisher_->publish(twist);
                }
            }
        }
    }
}
