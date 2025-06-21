#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdlib>  // For std::atoi

class PIDMazeSolver : public rclcpp::Node {
public:
    PIDMazeSolver(int scene_number) : Node("pid_maze_solver"), scene_number_(scene_number) {
        selectWaypoints();

        vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&PIDMazeSolver::odomCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PIDMazeSolver::controlLoop, this));
    }

private:
    struct Goal {
        float x;
        float y;
        float theta;
    };

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Goal> relative_goals;
    Goal current_target;

    std::size_t current_goal_index = 0;
    bool initialized = false;
    bool goal_active = false;

    float current_x = 0.0, current_y = 0.0;

    float prev_error_x = 0.0, integral_x = 0.0;
    float prev_error_y = 0.0, integral_y = 0.0;

    float kp = 0.5;
    float ki = 0.01;
    float kd = 0.08;

    float max_linear_speed = 0.5;   //0.5
    float goal_tolerance = 0.02;  //0.04

    int scene_number_;

    void selectWaypoints() {
        switch (scene_number_) {
            case 1:
                RCLCPP_INFO(this->get_logger(), "Scene 1: Simulation waypoints selected.");
                relative_goals = {
                    {0.35, 0.0, 0.0}, {0.2, -0.2, 0.0}, {0.0, -1.1, 0.0}, {0.5, 0.0, 0.0},
                    {0.0, 0.45, 0.0}, {0.4, 0.0, 0.0}, {0.0, 0.5, 0.0}, {0.6, 0.0, 0.0},
                    {0.0, 0.9, 0.0}, {-0.5, 0.0, 0.0}, {0.0, -0.3, 0.0}, {-0.4, 0.0, 0.0}, {-0.3, 0.3, 0.0}, {-0.6, 0.0, 0.0}
                };
                break;

            case 2:
                RCLCPP_INFO(this->get_logger(), "Scene 2: CyberWorld waypoints selected.");
                relative_goals = {
                    {1.0, 0.0, 0.0}, {0.0, -0.6, 0.0}, {0.0, 0.6, 0.0}, {-1.0, 0.0, 0.0}
                };
                break;

            default:
                RCLCPP_ERROR(this->get_logger(), "Invalid scene number: %d", scene_number_);
                rclcpp::shutdown();
                break;
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x = msg->pose.pose.position.x;
        current_y = msg->pose.pose.position.y;

        if (!initialized) {
            RCLCPP_INFO(this->get_logger(), "Initial position: (%.3f, %.3f)", current_x, current_y);
            initialized = true;
        }
    }

    void controlLoop() {
        if (!initialized || current_goal_index >= relative_goals.size())
            return;

        if (!goal_active) {
            current_target.x = current_x + relative_goals[current_goal_index].x;
            current_target.y = current_y + relative_goals[current_goal_index].y;
            goal_active = true;
            RCLCPP_INFO(this->get_logger(), "New target #%zu set to (%.3f, %.3f)",
                        current_goal_index, current_target.x, current_target.y);
        }

        float error_x = current_target.x - current_x;
        float error_y = current_target.y - current_y;
        float distance = std::sqrt(error_x * error_x + error_y * error_y);

        RCLCPP_INFO(this->get_logger(), "delta X: %.3f, delta Y: %.3f, Distance: %.3f",
                    error_x, error_y, distance);

        integral_x += error_x;
        float derivative_x = (error_x - prev_error_x) / 0.1;
        float control_x = kp * error_x + ki * integral_x + kd * derivative_x;

        integral_y += error_y;
        float derivative_y = (error_y - prev_error_y) / 0.1;
        float control_y = kp * error_y + ki * integral_y + kd * derivative_y;

        control_x = std::clamp(control_x, -max_linear_speed, max_linear_speed);
        control_y = std::clamp(control_y, -max_linear_speed, max_linear_speed);

        geometry_msgs::msg::Twist vel;
        vel.linear.x = control_x;
        vel.linear.y = control_y;
        vel_pub->publish(vel);

        if (distance < goal_tolerance) {
            RCLCPP_INFO(this->get_logger(), "Relative Goal %zu reached", current_goal_index);

            geometry_msgs::msg::Twist stop;
            for (int i = 0; i < 2; ++i) {
                vel_pub->publish(stop);
                rclcpp::sleep_for(std::chrono::milliseconds(500));
            }

            current_goal_index++;
            goal_active = false;
            integral_x = 0.0;
            integral_y = 0.0;

            if (current_goal_index >= relative_goals.size()) {
                vel_pub->publish(stop);
                timer_->cancel();
                rclcpp::shutdown();
            }
        }

        prev_error_x = error_x;
        prev_error_y = error_y;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int scene_number = 1; // Default: Simulation

    if (argc > 1) {
        scene_number = std::atoi(argv[1]);
    }

    auto node = std::make_shared<PIDMazeSolver>(scene_number);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
