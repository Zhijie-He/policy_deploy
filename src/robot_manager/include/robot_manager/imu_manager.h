#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cstring>
#include <mutex>
#include <chrono>

#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"						    // 引入 ROS 2 的核心头文件，包含对 rclcpp 库（C++ 客户端库）和基础节点类的支持
#include "std_msgs/msg/string.hpp"  				    // 使用 ROS 2 标准消息类型 String
#include "robot_message/msg/imu_data.hpp" 	    // 引入自定义消息头文件

// #include "imu_low_communication.h"

class ImuManagerNode : public rclcpp::Node {

public:
	ImuManagerNode(const std::string &node_name);

    ~ImuManagerNode();

private:

	// 发布者对象，发布消息到 "motor_control" 话题
	rclcpp::Publisher<robot_message::msg::ImuData>::SharedPtr imu_data_publisher_;

	// void Imu_Data_Message_Push(robot_message::msg::ImuData &msg) override;
};
