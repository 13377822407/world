#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class ImuPublisher : public rclcpp::Node {
public:
  ImuPublisher()
  : Node("imu_publisher") {
    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&ImuPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_link";
    // minimal valid message, zeros are acceptable for placeholder
    pub_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPublisher>());
  rclcpp::shutdown();
  return 0;
}
