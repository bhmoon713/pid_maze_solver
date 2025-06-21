#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

struct Waypoint {
    double x;
    double y;
    double yaw;  // in radians
};

class PIDMazeSolver : public rclcpp::Node {
public:
    PIDMazeSolver() : Node("pid_maze_solver"), current_index_(0), state_(State::TURNING) {
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&PIDMazeSolver::odomCallback, this, std::placeholders::_1));
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PIDMazeSolver::scanCallback, this, std::placeholders::_1));

        timer_ = create_wall_timer(50ms, std::bind(&PIDMazeSolver::controlLoop, this));

        // Define maze waypoints
        waypoints_ = {
            {1.0, 1.0, 1.57},
            {1.0, 0.5, -M_PI_2},
            {1.0, 0.0, M_PI_2},
            {0.0, 0.0, M_PI},
            {0.5, 0.5, 0.0}
        };
    }

private:
    enum class State { TURNING, MOVING, FINISHED };

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Waypoint> waypoints_;
    size_t current_index_;
    State state_;

    double current_x_, current_y_, current_yaw_;
    double front_distance_ = 1.0;

    // PID parameters (adjust as needed)
    double kp_turn = 1.5, ki_turn = 0.0, kd_turn = 0.3;
    double kp_move = 0.8, ki_move = 0.0, kd_move = 0.2;

    double error_sum_turn_ = 0.0, prev_error_turn_ = 0.0;
    double error_sum_move_ = 0.0, prev_error_move_ = 0.0;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        double roll, pitch;
        tf2::Matrix3x3(q).getRPY(roll, pitch, current_yaw_);
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        size_t center_index = msg->ranges.size() / 2;
        front_distance_ = msg->ranges[center_index];
    }

    void controlLoop() {
    if (current_index_ >= waypoints_.size()) {
        stopRobot();
        state_ = State::FINISHED;
        return;
    }

    const auto &goal = waypoints_[current_index_];

    // Compute distance to goal
    double dx = goal.x - current_x_;
    double dy = goal.y - current_y_;
    double distance = std::hypot(dx, dy);

    if (state_ == State::TURNING) {
        double angle_error = normalizeAngle(goal.yaw - current_yaw_);
        double angular_z = pid(angle_error, error_sum_turn_, prev_error_turn_, kp_turn, ki_turn, kd_turn);
        
        if (std::fabs(angle_error) < 0.05) {
            state_ = State::MOVING;
            resetPID(error_sum_turn_, prev_error_turn_);
            angular_z = 0.0;
        }

        publishCmd(0.0, angular_z);
    }
    else if (state_ == State::MOVING) {
        if (front_distance_ < 0.3) {
            stopRobot();
            RCLCPP_WARN(this->get_logger(), "Obstacle detected! Stopping.");
            return;
        }

        double linear_x = pid(distance, error_sum_move_, prev_error_move_, kp_move, ki_move, kd_move);
        linear_x = std::clamp(linear_x, -0.3, 0.3);

        if (distance < 0.1) {
            current_index_++;
            state_ = State::TURNING;
            resetPID(error_sum_move_, prev_error_move_);
            stopRobot();
            return;
        }

        publishCmd(linear_x, 0.0);
    }
}


    double pid(double error, double &sum, double &prev, double kp, double ki, double kd) {
        sum += error;
        double d = error - prev;
        prev = error;
        return kp * error + ki * sum + kd * d;
    }

    void resetPID(double &sum, double &prev) {
        sum = 0.0;
        prev = 0.0;
    }

    void stopRobot() {
        publishCmd(0.0, 0.0);
    }

    void publishCmd(double linear, double angular) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = linear;
        cmd.angular.z = angular;
        cmd_pub_->publish(cmd);
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDMazeSolver>());
    rclcpp::shutdown();
    return 0;
}
