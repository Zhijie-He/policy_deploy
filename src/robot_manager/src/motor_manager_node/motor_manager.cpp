#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include "motor_manager.h"

MotorManagerNode::MotorManagerNode(const std::string &node_name): rclcpp::Node(node_name)
{
	// RCLCPP_INFO 用于输出信息级别的日志
	RCLCPP_INFO(this->get_logger(), "MotorManagerNode Start, Versions V1.0.1 !");

	// 创建一个发布者，发布到 "motor_status" 话题
	motor_state_publisher_ = this->create_publisher<robot_message::msg::MotorStateData>("motor_status", 10);
	
	// 定时器，定期发布消息（例如每秒发布一次）
	//timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MotorManagerNode::Timer_Callback, this));
}

MotorManagerNode::~MotorManagerNode() {
    RCLCPP_INFO(this->get_logger(), "MotorManagerNode is being destroyed!");
}

int main(int argc, char *argv[])
{
	// 这一步是 ROS 2 的初始化过程，必须在任何 ROS 2 功能之前调用
	rclcpp::init(argc, argv);
	// 创建一个 MotorManagerNode 类的实例，并通过 rclcpp::spin 运行该节点
	rclcpp::spin(std::make_shared<MotorManagerNode>("motor_manager"));
	// 当程序退出时，调用 rclcpp::shutdown 来清理 ROS 2 环境
	rclcpp::shutdown();
	// 返回 0，表示程序正常结束
	return 0;
}

