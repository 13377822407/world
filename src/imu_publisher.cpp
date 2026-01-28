#include <chrono>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class ImuPublisher : public rclcpp::Node {
public:
  ImuPublisher()
  : Node("imu_publisher") {
    publish_rate_ = this->declare_parameter<double>("publish_rate", 100.0);
    linear_accel_noise_ = this->declare_parameter<double>("linear_accel_noise", 0.02);
    angular_vel_noise_ = this->declare_parameter<double>("angular_vel_noise", 0.01);

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

    std::random_device rd;
    gen_.seed(rd());
    accel_noise_dist_ = std::normal_distribution<double>(0.0, linear_accel_noise_);
    ang_noise_dist_ = std::normal_distribution<double>(0.0, angular_vel_noise_);
    timer_ = this->create_wall_timer(100ms, std::bind(&ImuPublisher::timer_callback, this));

    RCLCPP_INFO(get_logger(), "sim_imu_publisher started (rate=%.1f Hz)", publish_rate_);
  }

private:
  void timer_callback() {
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    
    msg.linear_acceleration.x = accel_noise_dist_(gen_);
    msg.linear_acceleration.y = accel_noise_dist_(gen_);
    msg.linear_acceleration.z = 9.81 + accel_noise_dist_(gen_);

    msg.angular_velocity.x = ang_noise_dist_(gen_);
    msg.angular_velocity.y = ang_noise_dist_(gen_);
    msg.angular_velocity.z = ang_noise_dist_(gen_);

    // covariance: -1 means unknown in some conventions; here we fill small values
    for (int i = 0; i < 9; ++i) msg.orientation_covariance[i] = -1.0;
    for (int i = 0; i < 9; ++i) msg.angular_velocity_covariance[i] = 0.01;
    for (int i = 0; i < 9; ++i) msg.linear_acceleration_covariance[i] = 0.04;
    imu_pub_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double publish_rate_;
  double linear_accel_noise_;
  double angular_vel_noise_;

  std::mt19937 gen_;
  std::normal_distribution<double> accel_noise_dist_;
  std::normal_distribution<double> ang_noise_dist_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPublisher>());
  rclcpp::shutdown();
  return 0;
}
