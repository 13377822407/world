#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class odom_publisher : public rclcpp::Node {
private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Time last_time;

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double linear_velocity;
    double angular_velocity;
    double pub_rate;
    std::string frame_id;
    std::string child_frame_id;

public:
    odom_publisher()
    : Node("odom_publisher") {
        pub_rate = this->declare_parameter("pub_rate", 10.0);
        frame_id = this->declare_parameter("frame_id", std::string("odom"));
        child_frame_id = this->declare_parameter("child_frame_id", std::string("base_link"));

        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom_world", 10);
        cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", 10, std::bind(&odom_publisher::cmd_vel_callback, this, std::placeholders::_1));
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        last_time = this->now();

        // initialize velocities
        linear_velocity = 0.0;
        angular_velocity = 0.0;

        auto period = std::chrono::duration<double>(1.0 / pub_rate);
        timer = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),std::bind(&odom_publisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Odometry publisher node has been started(linear=%.2f)",linear_velocity);
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        linear_velocity = msg->linear.x;
        angular_velocity = msg->angular.z;
    }

    void timer_callback() {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time).seconds();

        if (std::isfinite(linear_velocity) && std::isfinite(angular_velocity)) { // Check if the velocities are finite
            double delta_x = linear_velocity * std::cos(theta) * dt;
            double delta_y = linear_velocity * std::sin(theta) * dt;
            double delta_theta = angular_velocity * dt;

            x += delta_x;
            y += delta_y;
            theta += delta_theta;
        }

        nav_msgs::msg::Odometry odom;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta);

        odom.header.stamp = current_time;
        odom.header.frame_id = frame_id;
        odom.child_frame_id = child_frame_id;
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.linear.x = linear_velocity;
        odom.twist.twist.angular.z = angular_velocity;

        // set reasonable covariances
        for (int i = 0; i < 36; ++i) {
            odom.pose.covariance[i] = 0.0;
            odom.twist.covariance[i] = 0.0;
        }
        odom.pose.covariance[0] = 0.05;
        odom.pose.covariance[7] = 0.05;
        odom.pose.covariance[35] = 0.02;
        odom.twist.covariance[0] = 0.1;
        odom.twist.covariance[7] = 0.1;
        odom.twist.covariance[35] = 0.05;

        odom_pub->publish(odom);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = frame_id;
        t.child_frame_id = child_frame_id;
        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster->sendTransform(t);
        last_time = current_time;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<odom_publisher>());
    rclcpp::shutdown();
    return 0;
}
