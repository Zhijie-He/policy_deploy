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

#include "rclcpp/rclcpp.hpp"							// 引入 ROS 2 的核心头文件，包含对 rclcpp 库（C++ 客户端库）和基础节点类的支持
#include "std_msgs/msg/string.hpp"  					// 使用 ROS 2 标准消息类型 String
#include "robot_message/msg/motor_state_data.hpp" 		// 引入自定义消息头文件
#include "robot_message/msg/motor_control.hpp" 			// 引入自定义消息头文件
#include "robot_message/msg/joint_state_group.hpp" 		// 引入自定义消息头文件
#include "robot_message/msg/joint_control_group.hpp" 	// 引入自定义消息头文件

// #include "joint_message_dispose.h"

class JointDisposeNode : public rclcpp::Node {

public:
	JointDisposeNode(const std::string &node_name);

    ~JointDisposeNode();

private:
	// 订阅者对象，订阅 "motor_status" 话题
	rclcpp::Subscription<robot_message::msg::MotorStateData>::SharedPtr motor_state_subscription_;

	// 订阅者对象，订阅 "joint_control_group" 话题
	rclcpp::Subscription<robot_message::msg::JointControlGroup>::SharedPtr joint_control_subscription_;

	// 发布者对象，发布消息到 "motor_control" 话题
	rclcpp::Publisher<robot_message::msg::MotorControl>::SharedPtr motor_control_publisher_;

	// 发布者对象，发布消息到 "joint_state_group" 话题
	rclcpp::Publisher<robot_message::msg::JointStateGroup>::SharedPtr joint_state_publisher_;

	void Motor_State_Message_Callback(const robot_message::msg::MotorStateData::SharedPtr msg);

	void Joint_Control_Message_Callback(const robot_message::msg::JointControlGroup::SharedPtr msg);

	// void Joint_State_Message_Push(robot_message::msg::JointStateGroup &msg) override;

	// void Motor_Control_Message_Push(robot_message::msg::MotorControl &msg) override;
};
