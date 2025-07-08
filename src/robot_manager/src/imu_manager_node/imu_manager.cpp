#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include "imu_manager.h"

/**
  * @brief  构造函数
  * @param  null  
  * @retval null
  * @usage  
  */
ImuManagerNode::ImuManagerNode(const std::string &node_name): rclcpp::Node(node_name)
{
	// RCLCPP_INFO 用于输出信息级别的日志
	RCLCPP_INFO(this->get_logger(), "ImuManagerNode Start, Versions V1.0.1 !");

	// 创建一个发布者，发布到 "imu_data" 话题
	imu_data_publisher_ = this->create_publisher<robot_message::msg::ImuData>("imu_data", 10);
}

// void ImuManagerNode::Imu_Data_Message_Push(robot_message::msg::ImuData &msg) 
// {
// 	// 发布消息
// 	if(imu_data_publisher_ != nullptr)
// 	{
// 		imu_data_publisher_->publish(msg);
// 	}
// }

ImuManagerNode::~ImuManagerNode() {
    RCLCPP_INFO(this->get_logger(), "ImuManagerNode is being destroyed!");
}


int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ImuManagerNode>("imu_manager"));
	rclcpp::shutdown();

	return 0;
}

