#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
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

        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PIDMazeSolver::scanCallback, this, std::placeholders::_1));

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
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    bool wall_too_close = false;

    float front_range_ = std::numeric_limits<float>::infinity();
    float left_range_  = std::numeric_limits<float>::infinity();
    float right_range_ = std::numeric_limits<float>::infinity();
    float back_range_  = std::numeric_limits<float>::infinity();

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

    // PID gains for move
    float kp = 0.5;
    float ki = 0.01;
    float kd = 0.08;

    // PID variables for turning
    double prev_yaw_error = 0.0;
    double integral_yaw = 0.0;

    float kp_turn = 1.2;
    float ki_turn = 0.0;
    float kd_turn = 0.1;

    // float max_linear_speed = 0.5;
    double max_linear_speed = 0.5;
    float goal_tolerance = 0.02;
    float yaw_tolerance = 0.052;  // ~3 degrees

    double initial_yaw = 0.0;

    void selectWaypoints() {
        RCLCPP_INFO(this->get_logger(), "Simulation waypoints loaded.");
        relative_goals = {
            {0.35, 0.0, 0.0},
            {0.2, -0.2, -0.785},
            {0.0, -1.1, -0.785},
            {0.5, 0.0, 1.57},
            {0.0, 0.45, 1.57},
            {0.4, 0.0, 0.0},
            {0.0, 0.55, 0.0},
            {0.6, 0.0, 0.0},
            {0.0, 0.85, 0.0},
            {-0.5, 0.0, 1.57},
            {0.0, -0.3, 0.0},
            {-0.45, 0.0, 0.0},
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
        double roll, pitch, raw_yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, raw_yaw);

        if (!initialized) {
            initial_yaw = raw_yaw;
            initialized = true;
            RCLCPP_INFO(this->get_logger(), "Initial pose: (%.3f, %.3f), yaw: %.3f", current_x, current_y, initial_yaw);
        }
        // Normalize yaw to be relative to initial yaw
        current_yaw = raw_yaw - initial_yaw;
    
        // Optional: wrap current_yaw to [-π, π]
        if (current_yaw > M_PI) current_yaw -= 2 * M_PI;
        if (current_yaw < -M_PI) current_yaw += 2 * M_PI;
        
    }
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        int total_ranges = static_cast<int>(msg->ranges.size());

        // Compute index helpers
        auto angleToIndex = [&](float angle_deg) -> int {
            float angle_rad = angle_deg * M_PI / 180.0;
            int index = static_cast<int>((angle_rad - msg->angle_min) / msg->angle_increment);
            return std::clamp(index, 0, total_ranges - 1);
        };

        int idx_front = angleToIndex(0.0);
        int idx_left  = angleToIndex(90.0);
        int idx_right = angleToIndex(-90.0);
        int idx_back  = angleToIndex(180.0);

        front_range_ = msg->ranges[idx_front];
        left_range_  = msg->ranges[idx_left];
        right_range_ = msg->ranges[idx_right];
        back_range_  = msg->ranges[idx_back];

        wall_too_close = (front_range_ < 0.4);  // retain if needed elsewhere
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



        // RCLCPP_INFO(this->get_logger(), 
        // "velocity2twist | yaw: %.3f rad | input dx=%.3f dy=%.3f | output vx=%.3f vy=%.3f wz=%.3f",
        // current_yaw, dx, dy, vx, vy, wz);
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

            // PID for turning
            integral_yaw += yaw_error;
            double derivative = (yaw_error - prev_yaw_error) / 0.1;  // assuming 100ms loop
            double angular_z = kp_turn * yaw_error + ki_turn * integral_yaw + kd_turn * derivative;
            angular_z = std::clamp(angular_z, -0.6, 0.6);

            geometry_msgs::msg::Twist twist;
            if (std::fabs(yaw_error) > yaw_tolerance) {
                twist.angular.z = angular_z;
                vel_pub->publish(twist);
                prev_yaw_error = yaw_error;

                // RCLCPP_INFO(this->get_logger(),
                //     "Turning PID | error=%.3f, integral=%.3f, derivative=%.3f, angular_z=%.3f",
                //     yaw_error, integral_yaw, derivative, angular_z);
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Turn complete. Starting linear motion.");
            integral_yaw = 0.0;
            prev_yaw_error = 0.0;

            phase = Phase::MOVING;
            return;
        }


        if (phase == Phase::MOVING) {
            double target_x = start_x + goal.x;
            double target_y = start_y + goal.y;

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

            if (left_range_ < 0.17) {
                RCLCPP_WARN(this->get_logger(), "Left wall too close! Shifting right.");
                vel.linear.y -= 0.1;  // shift right
            }
            if (front_range_ < 0.17) {
                RCLCPP_WARN(this->get_logger(), "Front wall too close! Backing up.");
                vel.linear.x -= 0.1;  // move backward
            }
            if (right_range_ < 0.17) {
                RCLCPP_WARN(this->get_logger(), "Right wall too close! Shifting left.");
                vel.linear.y += 0.1;  // move left
            }
            if (back_range_ < 0.17) {
                RCLCPP_WARN(this->get_logger(), "Back wall too close! Moving forward.");
                vel.linear.x += 0.1;
            }

            vel_pub->publish(vel);

            RCLCPP_INFO(this->get_logger(), "Published velocity: linear.x = %.3f, linear.y = %.3f", vx, vy);
            // double target_yaw = start_yaw + goal.theta;
            // RCLCPP_INFO(this->get_logger(), "Yaw Info | target_yaw: %.3f, start_yaw: %.3f, goal.theta: %.3f",
            // target_yaw, start_yaw, goal.theta);

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
        // if (wall_too_close) {
        //     RCLCPP_WARN(this->get_logger(), "Wall too close! Stopping.");
        //     geometry_msgs::msg::Twist stop;
        //     vel_pub->publish(stop);
        //     return;
        // }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDMazeSolver>());
    rclcpp::shutdown();
    return 0;
}
