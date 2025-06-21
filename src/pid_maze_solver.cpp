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

    enum class Phase { TURNING, MOVING };
    Phase phase = Phase::TURNING;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Goal> relative_goals;
    Goal current_target;

    std::size_t current_goal_index = 0;
    bool initialized = false;
    bool goal_active = false;

    double current_x = 0.0, current_y = 0.0, current_yaw = 0.0;
    double start_x = 0.0, start_y = 0.0, start_yaw = 0.0;

    double prev_error_x = 0.0, integral_x = 0.0;
    double prev_error_y = 0.0, integral_y = 0.0;

    // PID gains
    float kp = 0.5;
    float ki = 0.01;
    float kd = 0.08;

    // float max_linear_speed = 0.5;
    double max_linear_speed = 0.5;
    float goal_tolerance = 0.02;
    float yaw_tolerance = 0.052;  // ~3 degrees

    void selectWaypoints() {
        RCLCPP_INFO(this->get_logger(), "Simulation waypoints loaded.");
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
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x = msg->pose.pose.position.x;
        current_y = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double roll, pitch;
        tf2::Matrix3x3(q).getRPY(roll, pitch, current_yaw);

        if (!initialized) {
            RCLCPP_INFO(this->get_logger(), "Initial pose: (%.3f, %.3f), yaw: %.3f", current_x, current_y, current_yaw);
            initialized = true;
        }
    }

    double normalizeAngle(double angle) {
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
        if (!initialized || current_goal_index >= relative_goals.size()) return;

        const auto& goal = relative_goals[current_goal_index];

        if (!goal_active) {
            start_yaw = current_yaw;
            start_x = current_x;
            start_y = current_y;
            current_target = goal;
            goal_active = true;
            phase = Phase::TURNING;

            RCLCPP_INFO(this->get_logger(), "New goal #%zu: turn %.3f rad, then move Δx=%.3f, Δy=%.3f",
                        current_goal_index, goal.theta, goal.x, goal.y);
        }

        if (phase == Phase::TURNING) {
            double target_yaw = normalizeAngle(start_yaw + goal.theta);
            double yaw_error = normalizeAngle(target_yaw - current_yaw);

            geometry_msgs::msg::Twist twist;
            if (std::fabs(yaw_error) > yaw_tolerance) {
                twist.angular.z = (yaw_error > 0) ? 0.3 : -0.3;
                vel_pub->publish(twist);
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Turn complete. Starting linear motion.");
            phase = Phase::MOVING;
            return;
        }

        if (phase == Phase::MOVING) {
            double cos_phi = std::cos(start_yaw + goal.theta);
            double sin_phi = std::sin(start_yaw + goal.theta);
            double target_x = start_x + cos_phi * goal.x - sin_phi * goal.y;
            double target_y = start_y + sin_phi * goal.x + cos_phi * goal.y;

            double error_x = target_x - current_x;
            double error_y = target_y - current_y;
            double distance = std::sqrt(error_x * error_x + error_y * error_y);

            integral_x += error_x;
            integral_y += error_y;

            double control_x = kp * error_x + ki * integral_x + kd * (error_x - prev_error_x) / 0.1;
            double control_y = kp * error_y + ki * integral_y + kd * (error_y - prev_error_y) / 0.1;

            control_x = std::clamp(control_x, -max_linear_speed, max_linear_speed);
            control_y = std::clamp(control_y, -max_linear_speed, max_linear_speed);

            auto [wz, vx, vy] = velocity2twist(0.0, control_x, control_y);

            geometry_msgs::msg::Twist vel;
            vel.linear.x = vx;
            vel.linear.y = vy;
            vel.angular.z = wz;
            vel_pub->publish(vel);

            RCLCPP_INFO(this->get_logger(), "Published velocity: linear.x = %.3f, linear.y = %.3f", vx, vy);

            if (distance < goal_tolerance) {
                RCLCPP_INFO(this->get_logger(), "Goal #%zu reached", current_goal_index);

                geometry_msgs::msg::Twist stop;
                vel_pub->publish(stop);
                rclcpp::sleep_for(std::chrono::milliseconds(500));

                current_goal_index++;
                goal_active = false;
                integral_x = 0.0;
                integral_y = 0.0;
                prev_error_x = 0.0;
                prev_error_y = 0.0;

                if (current_goal_index >= relative_goals.size()) {
                    vel_pub->publish(stop);
                    timer_->cancel();
                    rclcpp::shutdown();
                }
            }

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
