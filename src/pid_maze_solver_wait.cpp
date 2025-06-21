#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <tuple>

class PIDMazeSolver : public rclcpp::Node {
public:
    PIDMazeSolver() : Node("pid_maze_solver") {
        relative_goals = {
            {0.35, 0.0, 0.0},
            {0.2, -0.2, -0.785},
            {0.0, -1.1, -0.785},
            {0.5, 0.0, 1.57},
            {0.0, 0.45, 1.57},
            {0.4, 0.0, 0.0},
            {0.0, 0.5, 0.0},
            {0.6, 0.0, 0.0},
            {0.0, 0.9, 0.0},
            {-0.5, 0.0, 1.57},
            {0.0, -0.3, 0.0},
            {-0.4, 0.0, 0.0},
            {-0.3, 0.3, -0.785},
            {-0.6, 0.0, 0.785},
            {0.0, 0.0, 3.145}
        };

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
        float dx;
        float dy;
        float dtheta;
    };

    enum class State { TURNING, MOVING };

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Goal> relative_goals;
    std::size_t current_goal_index = 0;

    bool initialized = false;
    bool goal_active = false;

    float current_x = 0.0, current_y = 0.0, current_yaw = 0.0;
    float target_x = 0.0, target_y = 0.0, target_yaw = 0.0;

    float prev_error_x = 0.0, integral_x = 0.0;
    float prev_error_y = 0.0, integral_y = 0.0;
    float prev_error_yaw = 0.0, integral_yaw = 0.0;

    float kp = 0.5, ki = 0.01, kd = 0.08;
    float kp_yaw = 2.5, ki_yaw = 0.0, kd_yaw = 0.2;

    float max_linear_speed = 0.5;
    float max_angular_speed = 1.0;
    float goal_tolerance = 0.03;
    float yaw_tolerance = 0.05;

    State current_state = State::TURNING;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x = msg->pose.pose.position.x;
        current_y = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        current_yaw = static_cast<float>(yaw);

        if (!initialized) {
            target_x = current_x;
            target_y = current_y;
            target_yaw = current_yaw;
            initialized = true;
        }
    }

    float normalizeAngle(float angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    std::tuple<double, double, double> velocity2twist(double dphi, double dx, double dy) {
        double cos_phi = std::cos(current_yaw);
        double sin_phi = std::sin(current_yaw);
        double wz = dphi;
        double vx = cos_phi * dx + sin_phi * dy;
        double vy = -sin_phi * dx + cos_phi * dy;
        RCLCPP_INFO(this->get_logger(), "velocity2twist: %.2f , %.2f , %.2f ", wz, vx, vy);
        return {wz, vx, vy};
    }

    void controlLoop() {
        if (!initialized || current_goal_index >= relative_goals.size())
            return;

        const auto& goal = relative_goals[current_goal_index];

        if (!goal_active) {
            float dx = goal.dx;
            float dy = goal.dy;
            float dtheta = goal.dtheta;

            float cos_theta = std::cos(target_yaw);
            float sin_theta = std::sin(target_yaw);

            target_x += dx * cos_theta - dy * sin_theta;
            target_y += dx * sin_theta + dy * cos_theta;
            target_yaw = normalizeAngle(target_yaw + dtheta);

            goal_active = true;
            current_state = State::TURNING;

            RCLCPP_INFO(this->get_logger(), "New Goal %zu: (%.3f, %.3f, %.3f)",
                        current_goal_index, target_x, target_y, target_yaw);
        }

        float yaw_error = normalizeAngle(target_yaw - current_yaw);

        if (current_state == State::TURNING) {
            integral_yaw += yaw_error;
            float derivative_yaw = (yaw_error - prev_error_yaw) / 0.1;
            float control_yaw = kp_yaw * yaw_error + ki_yaw * integral_yaw + kd_yaw * derivative_yaw;
            control_yaw = std::clamp(control_yaw, -max_angular_speed, max_angular_speed);

            auto [wz, vx_unused, vy_unused] = velocity2twist(control_yaw, 0.0, 0.0);
            geometry_msgs::msg::Twist vel;
            vel.angular.z = wz;
            vel_pub->publish(vel);

            RCLCPP_INFO(this->get_logger(), "[TURNING] Goal #%zu | Current Yaw: %.3f | Target Yaw: %.3f | Error: %.3f",
                        current_goal_index, current_yaw, target_yaw, yaw_error);

            if (std::fabs(yaw_error) < yaw_tolerance) {
                current_state = State::MOVING;
                integral_yaw = 0.0;
                prev_error_yaw = 0.0;
            } else {
                prev_error_yaw = yaw_error;
                return;
            }
        }

        float error_x = target_x - current_x;
        float error_y = target_y - current_y;
        float distance = std::sqrt(error_x * error_x + error_y * error_y);

        integral_x += error_x;
        float derivative_x = (error_x - prev_error_x) / 0.1;
        float control_x = kp * error_x + ki * integral_x + kd * derivative_x;

        integral_y += error_y;
        float derivative_y = (error_y - prev_error_y) / 0.1;
        float control_y = kp * error_y + ki * integral_y + kd * derivative_y;

        control_x = std::clamp(control_x, -max_linear_speed, max_linear_speed);
        control_y = std::clamp(control_y, -max_linear_speed, max_linear_speed);

        auto [wz_unused, vx, vy] = velocity2twist(0.0, control_x, control_y);
        geometry_msgs::msg::Twist vel;
        vel.linear.x = vx;
        vel.linear.y = vy;
        vel_pub->publish(vel);

        RCLCPP_INFO(this->get_logger(),
                    "[MOVING] Goal #%zu | delta X: %.3f, delta Y: %.3f, Distance: %.3f, Yaw Error: %.3f",
                    current_goal_index, error_x, error_y, distance, yaw_error);

        if (distance < goal_tolerance) {
            RCLCPP_INFO(this->get_logger(), "Goal %zu reached ✅", current_goal_index);
            geometry_msgs::msg::Twist stop;
            vel_pub->publish(stop);

            current_goal_index++;
            goal_active = false;
            integral_x = integral_y = 0.0;
            prev_error_x = prev_error_y = 0.0;
        } else {
            prev_error_x = error_x;
            prev_error_y = error_y;
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDMazeSolver>());
    rclcpp::shutdown();
    return 0;
}
