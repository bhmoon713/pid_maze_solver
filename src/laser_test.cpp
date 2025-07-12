#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>

class LaserTestNode : public rclcpp::Node {
public:
  LaserTestNode() : Node("laser_test_node") {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&LaserTestNode::laserCallback, this, std::placeholders::_1)
    );
  }

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const auto &ranges = msg->ranges;

    std::vector<int> indices = {0, 179, 359, 539, 719};  // real robot
    std::vector<std::string> labels = {
      "Front", "Left", "Back", "Right", "Front (end)"
    };

    RCLCPP_INFO(this->get_logger(), "\n---- LaserScan Info ----");
    RCLCPP_INFO(this->get_logger(), "Frame: %s", msg->header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "Timestamp: %u.%09u",
                msg->header.stamp.sec, msg->header.stamp.nanosec);
    RCLCPP_INFO(this->get_logger(), "angle_min: %.4f rad | angle_max: %.4f rad",
                msg->angle_min, msg->angle_max);
    RCLCPP_INFO(this->get_logger(), "angle_increment: %.6f rad | total ranges: %zu",
                msg->angle_increment, ranges.size());

    for (size_t i = 0; i < indices.size(); ++i) {
      int idx = indices[i];
      if (idx >= 0 && idx < static_cast<int>(ranges.size())) {
        float angle_rad = msg->angle_min + idx * msg->angle_increment;
        float angle_deg = angle_rad * 180.0 / M_PI;
        float range_val = ranges[idx];
        RCLCPP_INFO(this->get_logger(),
                    "ranges[%3d] (%-11s) @ %+7.2f° = %.3f m",
                    idx, labels[i].c_str(), angle_deg, range_val);
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "Index %d (%s) out of range (total: %zu)",
                    idx, labels[i].c_str(), ranges.size());
      }
    }

    // === Additional average region calculations ===
    auto calc_avg = [&](int start_idx, int end_idx) {
      float sum = 0.0f;
      int count = 0;
      for (int i = start_idx; i <= end_idx && i < static_cast<int>(ranges.size()); ++i) {
        if (std::isfinite(ranges[i])) {
          sum += ranges[i];
          count++;
        }
      }
      return count > 0 ? sum / count : -1.0f;  // return -1 if no valid data
    };

    float right_up_avg = calc_avg(540, 629);
    float right_down_avg = calc_avg(449, 538);
    float left_down_avg = calc_avg(180, 269);
    float left_up_avg = calc_avg(89, 178);

    RCLCPP_INFO(this->get_logger(), "\n---- Region Averages ----");
    RCLCPP_INFO(this->get_logger(), "Right Up Avg   [540, 629]  : %.3f m", right_up_avg);
    RCLCPP_INFO(this->get_logger(), "Right Down Avg [449, 538] : %.3f m", right_down_avg);
    RCLCPP_INFO(this->get_logger(), "Left Down Avg  [180, 269] : %.3f m", left_down_avg);
    RCLCPP_INFO(this->get_logger(), "Left Up Avg    [89, 178] : %.3f m", left_up_avg);
    RCLCPP_INFO(this->get_logger(), "----------------------------\n");
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserTestNode>());
  rclcpp::shutdown();
  return 0;
}
