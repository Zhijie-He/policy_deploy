#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include "joint_dispose.h"


JointDisposeNode::JointDisposeNode(const std::string &node_name): rclcpp::Node(node_name)
{
	// RCLCPP_INFO 用于输出信息级别的日志
	RCLCPP_INFO(this->get_logger(), "JointDisposeNode Start, Versions V1.0.1 !");

	// 创建一个发布者，发布到 "motor_control" 话题
	motor_control_publisher_ = this->create_publisher<robot_message::msg::MotorControl>("motor_control", 10);

	// 创建一个发布者，发布到 "joint_state_group" 话题
	joint_state_publisher_ = this->create_publisher<robot_message::msg::JointStateGroup>("joint_state_group", 10);

	// 创建一个订阅者，订阅 "motor_control" 话题
	motor_state_subscription_ = this->create_subscription<robot_message::msg::MotorStateData>(
		"motor_status", 10,
		std::bind(&JointDisposeNode::Motor_State_Message_Callback, this, std::placeholders::_1));

	// 创建一个订阅者，订阅 "joint_control_group" 话题
	joint_control_subscription_ = this->create_subscription<robot_message::msg::JointControlGroup>(
		"joint_control_group", 10,
		std::bind(&JointDisposeNode::Joint_Control_Message_Callback, this, std::placeholders::_1));

}


/**
  * @brief  MotorStateData消息订阅回调函数
  * @param  msg	传入的消息形参  
  * @retval null
  * @usage  
  */
void JointDisposeNode::Motor_State_Message_Callback(const robot_message::msg::MotorStateData::SharedPtr msg)
{
	/* 消息日志打印 */ 
	RCLCPP_INFO(this->get_logger(), "Received motor state, sequence: %d", msg->sequence);
	// for (int i = 0; i < 14; ++i)
	// {
	// 	RCLCPP_INFO(this->get_logger(), 
	// 		"Motor %d: Mode: %d, Pos: %.2f, W: %.2f, T: %.2f, kp: %.2f, kd: %.2f", 
	// 		i, msg->cmd[i].mode, msg->cmd[i].pos, msg->cmd[i].w, msg->cmd[i].t, msg->cmd[i].kp, msg->cmd[i].kd);
	// }

	// JointMessageDispose::Motor_State_Queue_Push(msg);

}



/**
  * @brief  JointControlGroup消息订阅回调函数
  * @param  msg	传入的消息形参  
  * @retval null
  * @usage  
  */
void JointDisposeNode::Joint_Control_Message_Callback(const robot_message::msg::JointControlGroup::SharedPtr msg)
{
	/* 消息日志打印 */ 
	RCLCPP_INFO(this->get_logger(), "Received motor control, sequence: %d", msg->sequence);
	// for (int i = 0; i < 14; ++i)
	// {
	// 	RCLCPP_INFO(this->get_logger(), 
	// 		"Motor %d: Mode: %d, Pos: %.2f, W: %.2f, T: %.2f, kp: %.2f, kd: %.2f", 
	// 		i, msg->cmd[i].mode, msg->cmd[i].pos, msg->cmd[i].w, msg->cmd[i].t, msg->cmd[i].kp, msg->cmd[i].kd);
	// }

	// JointMessageDispose::Joint_Control_Queue_Push(msg);

}

/**
  * @brief  JointStateGroup消息发布函数
  * @param  msg	传入的消息形参  
  * @retval null
  * @usage  
  */
// void JointDisposeNode::Joint_State_Message_Push(robot_message::msg::JointStateGroup &msg)
// {
// 	// 发布消息
// 	if(joint_state_publisher_ != nullptr)
// 	{
// 		joint_state_publisher_->publish(msg);

// 	}
// }


/**
  * @brief  MotorControl消息发布函数
  * @param  msg	传入的消息形参  
  * @retval null
  * @usage  
  */
// void JointDisposeNode::Motor_Control_Message_Push(robot_message::msg::MotorControl &msg) 
// {
// 	// 发布消息
// 	if(motor_control_publisher_ != nullptr)
// 	{
// 		motor_control_publisher_->publish(msg);
// 	}
// }


JointDisposeNode::~JointDisposeNode() {
    RCLCPP_INFO(this->get_logger(), "JointDisposeNode is being destroyed!");
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<JointDisposeNode>("joint_dispose"));
	rclcpp::shutdown();

	return 0;
}

