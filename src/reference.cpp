#include "rclcpp/logging.hpp"
#include "rclcpp/rate.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

class MazeSolver : public rclcpp::Node {
public:
    MazeSolver() : Node("Maze_solver") {

        // Set up waypoints
        points_goals = {
            {0.46, -0.04},
            {1.402, -0.8},
            {1.97, -0.84},
            {2.33, -0.55},
            {2.76, -0.53},
            {3.09, -0.27},
            {3.60, -0.29},
            {4.2724,0.1939},//{4.3307, 0.238},
            {4.409, 0.694},// {4.42, 0.70}, {4.435, 0.686},
            {4.24, 0.9869},// {4.229, 0.991},
            {4.347, 1.457},
            {4.571, 1.812},
            {4.695, 2.326}, 

        };
        current_goal_index = 0;

        // Publisher for velocity commands
        vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscriber for odometry data
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rosbot_xl_base_controller/odom", 10, std::bind(&MazeSolver::odomCallback, this, std::placeholders::_1));

        // Make sure the loop execute within specific time
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
            std::bind(&MazeSolver::controlLoop, this));
    }

private:

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::pair<float, float>> points_goals;
    float rad_goals;

    int current_goal_index;

    float current_yaw_rad;
    float current_x_pos;
    float current_y_pos;

    float prev_error_dis;
    float prev_error_tn;

    float integral_dis;
    float integral_tn;
    
    float D_kp = 0.5, D_ki = 0.0125, D_kd = 0.0025;
    float T_kp = 0.7, T_ki = 0.0175, T_kd = 0.0035;

    float yaw_goal_tolerance = 0.02; 
    float dis_goal_tolerance = 0.03; 

    bool open_distance = false;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_yaw_rad = euler_degree_transform(msg);
        current_x_pos = msg->pose.pose.position.x;
        current_y_pos = msg->pose.pose.position.y;
    }

    float euler_degree_transform(const nav_msgs::msg::Odometry::SharedPtr msg){
        
        float x = msg->pose.pose.orientation.x;
        float y = msg->pose.pose.orientation.y;
        float z = msg->pose.pose.orientation.z;
        float w = msg->pose.pose.orientation.w; 

        return atan2(2 * (w * z + x * y),1 - 2 * (y * y + z * z));
    }

    float cartesian_transfrom(float x_set, float y_set, float x, float y){
        
        float goad_rad;
        goad_rad = atan2( y_set - y, x_set - x);

        return goad_rad;
    }

    void controlLoop() {
        if(open_distance == false) turn_controller();
        else distance_controller();
    }

    void distance_controller(){
        // Calculate error
        float x_error = points_goals[current_goal_index].first - current_x_pos;
        float y_error = points_goals[current_goal_index].second - current_y_pos;

        float error = std::sqrt( std::pow(x_error,2) + std::pow(y_error,2 ));
        
        RCLCPP_INFO(rclcpp::get_logger("Distance controller"), "Error of distance: %f" , error);

        // Update integral term
        integral_dis += error;

        // Calculate derivative term
        float derivative = (error - prev_error_dis) / 0.1;  // loop runs every 0.1 seconds

        // PID control
        float control_signal = D_kp * error + D_ki * integral_dis + D_kd * derivative;

        // Create velocity command
        geometry_msgs::msg::Twist vel;
        vel.linear.x = control_signal;
        vel_pub->publish(vel);

        // Check if the goal is reached
        if (std::abs(error) < dis_goal_tolerance) {
            RCLCPP_INFO(rclcpp::get_logger("Distance controller"), "Goal of position %i is completed" , current_goal_index);
            rclcpp::Rate r(1);
            for(int i=0; i<2;i++){
                vel.linear.x  = 0.0;
                vel_pub->publish(vel);
                r.sleep();
            }
            // Move to the next goal
            current_goal_index++;
            // Change to turn_controller by changed open_distance to false
            open_distance = false;
            // Reset integral term
            integral_dis = 0.0;

            // Stop the robot if all goals are reached
            if (current_goal_index >= points_goals.size()) {
                vel.linear.x  = 0.0;
                vel_pub->publish(vel);
                timer_->cancel();
            }
        }

        // Update previous error
        prev_error_dis = error;
    }
    void turn_controller(){
        
        rad_goals = cartesian_transfrom(
            points_goals[current_goal_index].first,
            points_goals[current_goal_index].second , 
                    current_x_pos, current_y_pos);
        // Calculate error
        float error = rad_goals - current_yaw_rad;

        // Update integral term
        integral_tn += error;

        // Calculate derivative term
        float derivative = (error - prev_error_tn) / 0.1;  // loop runs every 0.1 seconds

        // PID control
        float control_signal = T_kp * error + T_ki * integral_tn + T_kd * derivative;

        // Create velocity command
        geometry_msgs::msg::Twist vel;
        // velocity is equal to control signal
        vel.angular.z = control_signal;
        vel_pub->publish(vel);


        // Check if the goal is reached
        if (std::abs(error) < yaw_goal_tolerance) {
            RCLCPP_INFO(rclcpp::get_logger("Maze solver"), "Goal of yaw %i is completed" , current_goal_index);
            
            rclcpp::Rate r(1);
            for(int i=0; i<2;i++){
                vel.angular.z  = 0.0;
                vel_pub->publish(vel);
                r.sleep();
            }

            // Reset integral term
            integral_tn = 0.0;
            //Change to distance_controller by changed open_distance to true
            open_distance = true;

            // Stop the robot if all goals are reached
            if (current_goal_index >= points_goals.size()) {
                vel.angular.z = 0.0;
                vel_pub->publish(vel);
            }
        }

        // Update previous error
        prev_error_tn = error;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MazeSolver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}