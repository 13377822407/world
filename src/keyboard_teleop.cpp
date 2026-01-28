/**
 * @file teleop_keyboard.cpp
 * @brief 键盘遥控节点 - 通过键盘控制机器人移动
 * 
 * 功能说明：
 * 1. 监听键盘输入
 * 2. 根据按键发送相应的速度命令到/cmd_vel话题
 * 3. 支持加速、减速、转向、紧急停止等功能
 * 
 * 按键映射：
 * - w/s : 增加/减少前进速度
 * - a/d : 增加/减少转向速度（左转/右转）
 * - x   : 后退
 * - 空格: 紧急停止
 * - q   : 退出程序
 * 
 * 学习要点：
 * - 如何处理Linux终端的键盘输入
 * - 如何在循环中发布消息
 * - 速度控制的平滑增减逻辑
 * - 安全停止机制
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <termios.h>  // Linux终端控制
#include <unistd.h>   // UNIX标准函数
#include <fcntl.h>    // 文件控制

/**
 * @brief 获取单个按键输入（非阻塞）
 * 
 * 在Linux系统中，终端默认是行缓冲模式，需要按回车才能读取输入。
 * 这个函数将终端设置为原始模式（raw mode），可以立即读取单个字符。
 * 
 * @return char 按下的键，如果没有按键则返回0
 */
char getch()
{
    char buf = 0;
    struct termios old = {0};
    
    // 保存当前终端设置
    if (tcgetattr(STDIN_FILENO, &old) < 0) {
        perror("tcgetattr()");
    }
    
    // 设置终端为原始模式
    struct termios raw = old;
    raw.c_lflag &= ~(ICANON | ECHO);  // 关闭规范模式和回显
    raw.c_cc[VMIN] = 0;               // 非阻塞读取
    raw.c_cc[VTIME] = 1;              // 0.1秒超时
    
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) < 0) {
        perror("tcsetattr ICANON");
    }
    
    // 读取一个字符
    if (read(STDIN_FILENO, &buf, 1) < 0) {
        perror("read()");
    }
    
    // 恢复终端设置
    if (tcsetattr(STDIN_FILENO, TCSADRAIN, &old) < 0) {
        perror("tcsetattr ~ICANON");
    }
    
    return buf;
}

/**
 * @class TeleopKeyboard
 * @brief 键盘遥控节点类
 */
class TeleopKeyboard : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * 
     * 初始化节点、发布者和速度参数
     */
    TeleopKeyboard() : Node("teleop_keyboard")
    {
        // 创建发布者
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // 初始化速度参数
        linear_vel_ = 0.0;   // 当前线速度
        angular_vel_ = 0.0;  // 当前角速度
        
        // 速度增量（每次按键改变的速度）
        linear_step_ = 0.05;   // 线速度步长 0.05 m/s
        angular_step_ = 0.1;   // 角速度步长 0.1 rad/s
        
        // 速度限制（安全考虑）
        max_linear_vel_ = 1.0;   // 最大线速度 1.0 m/s
        max_angular_vel_ = 2.0;  // 最大角速度 2.0 rad/s
        
        RCLCPP_INFO(this->get_logger(), "===================================");
        RCLCPP_INFO(this->get_logger(), "  键盘遥控节点已启动");
        RCLCPP_INFO(this->get_logger(), "===================================");
        print_usage();
    }
    
    /**
     * @brief 运行主循环
     * 
     * 持续监听键盘输入并发布速度命令
     */
    void run()
    {
        char key;
        bool running = true;
        
        // 创建用于发布的Twist消息
        auto twist_msg = geometry_msgs::msg::Twist();
        
        // 创建定时器，确保以固定频率发布（即使没有按键）
        // 这对于某些机器人控制器很重要，它们需要持续接收命令
        auto timer = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz
            [this, &twist_msg]() {
                twist_msg.linear.x = linear_vel_;
                twist_msg.angular.z = angular_vel_;
                publisher_->publish(twist_msg);
            }
        );
        
        while (running && rclcpp::ok()) {
            // 读取按键
            key = getch();
            
            if (key != 0) {  // 有按键输入
                switch (key) {
                    case 'w':  // 加速前进
                    case 'W':
                        linear_vel_ += linear_step_;
                        linear_vel_ = std::min(linear_vel_, max_linear_vel_);
                        RCLCPP_INFO(this->get_logger(), "前进加速: %.2f m/s", linear_vel_);
                        break;
                    
                    case 's':  // 减速
                    case 'S':
                        linear_vel_ -= linear_step_;
                        linear_vel_ = std::max(linear_vel_, -max_linear_vel_);
                        RCLCPP_INFO(this->get_logger(), "减速: %.2f m/s", linear_vel_);
                        break;
                    
                    case 'x':  // 后退
                    case 'X':
                        linear_vel_ = -0.2;
                        RCLCPP_INFO(this->get_logger(), "后退: %.2f m/s", linear_vel_);
                        break;
                    
                    case 'a':  // 左转
                    case 'A':
                        angular_vel_ += angular_step_;
                        angular_vel_ = std::min(angular_vel_, max_angular_vel_);
                        RCLCPP_INFO(this->get_logger(), "左转: %.2f rad/s", angular_vel_);
                        break;
                    
                    case 'd':  // 右转
                    case 'D':
                        angular_vel_ -= angular_step_;
                        angular_vel_ = std::max(angular_vel_, -max_angular_vel_);
                        RCLCPP_INFO(this->get_logger(), "右转: %.2f rad/s", angular_vel_);
                        break;
                    
                    case ' ':  // 空格键 - 紧急停止
                        linear_vel_ = 0.0;
                        angular_vel_ = 0.0;
                        RCLCPP_WARN(this->get_logger(), "紧急停止！");
                        break;
                    
                    case 'q':  // 退出
                    case 'Q':
                        RCLCPP_INFO(this->get_logger(), "退出程序...");
                        linear_vel_ = 0.0;
                        angular_vel_ = 0.0;
                        // 发送停止命令
                        twist_msg.linear.x = 0.0;
                        twist_msg.angular.z = 0.0;
                        publisher_->publish(twist_msg);
                        running = false;
                        break;
                    
                    case 'h':  // 显示帮助
                    case 'H':
                        print_usage();
                        break;
                    
                    default:
                        // 忽略未定义的按键
                        break;
                }
                
                // 显示当前速度
                if (key != 'h' && key != 'H' && key != 'q' && key != 'Q') {
                    LCPP_INFO(this->get_logger(), 
                               "当前速度 -> 线速度: %.2f m/s, 角速度: %.2f rad/s",
                               linear_vel_, angular_vel_);
                }
            }
            
            // 处理ROS2回调（包括定时器）
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }

private:
    /**
     * @brief 打印使用说明
     */
    void print_usage()
    {
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "控制说明:");
        RCLCPP_INFO(this->get_logger(), "-----------------------------------");
        RCLCPP_INFO(this->get_logger(), "  W : 前进加速");
        RCLCPP_INFO(this->get_logger(), "  S : 减速");
        RCLCPP_INFO(this->get_logger(), "  X : 后退");
        RCLCPP_INFO(this->get_logger(), "  A : 左转");
        RCLCPP_INFO(this->get_logger(), "  D : 右转");
        RCLCPP_INFO(this->get_logger(), "  空格 : 紧急停止");
        RCLCPP_INFO(this->get_logger(), "  H : 显示此帮助");
        RCLCPP_INFO(this->get_logger(), "  Q : 退出程序");
        RCLCPP_INFO(this->get_logger(), "-----------------------------------");
        RCLCPP_INFO(this->get_logger(), "速度限制:");
        RCLCPP_INFO(this->get_logger(), "  最大线速度: %.2f m/s", max_linear_vel_);
        RCLCPP_INFO(this->get_logger(), "  最大角速度: %.2f rad/s", max_angular_vel_);
        RCLCPP_INFO(this->get_logger(), "-----------------------------------");
        RCLCPP_INFO(this->get_logger(), "");
    }
    
    // 成员变量
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    
    // 当前速度
    double linear_vel_;   ///< 当前线速度 (m/s)
    double angular_vel_;  ///< 当前角速度 (rad/s)
    
    // 速度控制参数
    double linear_step_;       ///< 线速度增量
    double angular_step_;      ///< 角速度增量
    double max_linear_vel_;    ///< 最大线速度
    double max_angular_vel_;   ///< 最大角速度
};

/**
 * @brief 主函数
 */
int main(int argc, char * argv[])
{
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建键盘遥控节点
    auto node = std::make_shared<TeleopKeyboard>();
    
    // 运行主循环
    node->run();
    
    // 清理资源
    rclcpp::shutdown();
    
    return 0;
}
