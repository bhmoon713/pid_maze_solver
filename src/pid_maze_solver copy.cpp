#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <vector>
#include <cmath>
#include <algorithm>
#include <tuple>

class PIDMazeSolver : public rclcpp::Node {
public:
    PIDMazeSolver(int scene_number)
        : Node("pid_maze_solver"), scene_number_(scene_number) {
        
        relative_goals = readWaypointsYAML(scene_number_);

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
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Goal> relative_goals;
    Goal current_target;

    int scene_number_;
    std::size_t current_goal_index = 0;
    bool initialized = false;
    bool goal_active = false;

    double current_x = 0.0, current_y = 0.0, current_yaw = 0.0;
    double start_x = 0.0, start_y = 0.0, start_yaw = 0.0;

    double prev_error_x = 0.0, integral_x = 0.0;
    double prev_error_y = 0.0, integral_y = 0.0;
    double prev_yaw_error = 0.0, integral_yaw = 0.0;

    float kp = 0.5, ki = 0.01, kd = 0.08;
    float kp_turn = 1.2, ki_turn = 0.0, kd_turn = 0.1;
    double max_linear_speed = 0.1;
    float goal_tolerance = 0.02;
    float yaw_tolerance = 0.052;

    float front_range_ = std::numeric_limits<float>::infinity();
    float left_range_  = std::numeric_limits<float>::infinity();
    float right_range_ = std::numeric_limits<float>::infinity();
    float back_range_  = std::numeric_limits<float>::infinity();
    bool wall_too_close = false;

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

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        const auto &ranges = msg->ranges;

        // Fixed indices based on real robot LaserScan layout
        int idx_front = 0;
        int idx_left = 179;
        int idx_back = 359;
        int idx_right = 539;
        int idx_front_end = 719;

        // Use average of front (start and end) for better stability
        front_range_ = (ranges[idx_front] + ranges[idx_front_end]) / 2.0;
        left_range_  = ranges[idx_left];
        back_range_  = ranges[idx_back];
        right_range_ = ranges[idx_right];

        wall_too_close = (front_range_ < 0.4);
    }

    std::vector<Goal> readWaypointsYAML(int scene_number) {
        std::vector<Goal> waypoints;
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("pid_maze_solver");

        std::string waypoint_file_name;
        if (scene_number == 1) {
            waypoint_file_name = "waypoints_sim.yaml";
        } else if (scene_number == 2) {
            waypoint_file_name = "waypoints_real.yaml";
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid scene number: %d", scene_number);
            return waypoints;
        }

        std::string yaml_path = package_share_directory + "/waypoints/" + waypoint_file_name;
        RCLCPP_INFO(this->get_logger(), "Loading waypoints from: %s", yaml_path.c_str());

        try {
            YAML::Node config = YAML::LoadFile(yaml_path);
            if (config["waypoints"]) {
                for (const auto& wp : config["waypoints"]) {
                    waypoints.push_back({wp[0].as<float>(), wp[1].as<float>(), wp[2].as<float>()});
                }
            }
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YAML: %s", e.what());
        }

        return waypoints;
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

            integral_yaw += yaw_error;
            double derivative = (yaw_error - prev_yaw_error) / 0.1;
            double angular_z = kp_turn * yaw_error + ki_turn * integral_yaw + kd_turn * derivative;
            angular_z = std::clamp(angular_z, -0.6, 0.6);

            geometry_msgs::msg::Twist twist;
            if (std::fabs(yaw_error) > yaw_tolerance) {
                twist.angular.z = angular_z;
                vel_pub->publish(twist);
                prev_yaw_error = yaw_error;
                return;
            }

            integral_yaw = 0.0;
            prev_yaw_error = 0.0;
            RCLCPP_INFO(this->get_logger(), "Turn complete. Starting linear motion.");
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

            // if (left_range_ < 0.2) {
            //     vel.linear.y -= 0.05;
            //     RCLCPP_INFO(this->get_logger(), "Left too close");
            // }
            // if (right_range_ < 0.2) {
            //     vel.linear.y += 0.04;
            //     RCLCPP_INFO(this->get_logger(), "Right too close");
            // }
            // if (front_range_ < 0.18) {
            //     vel.linear.x -= 0.05;
            //     RCLCPP_INFO(this->get_logger(), "Front too close");
            // }
            // if (back_range_ < 0.18) {
            //     vel.linear.x += 0.05;
            //     RCLCPP_INFO(this->get_logger(), "Back too close");
            // }

            vel_pub->publish(vel);

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

    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("pid_maze_solver"), "Usage: ros2 run pid_maze_solver pid_maze_solver <scene_number>");
        return 1;
    }

    int scene_number = std::atoi(argv[1]);
    rclcpp::spin(std::make_shared<PIDMazeSolver>(scene_number));
    rclcpp::shutdown();
    return 0;
}
