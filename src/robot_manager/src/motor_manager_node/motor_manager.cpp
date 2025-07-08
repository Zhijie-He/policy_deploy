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

	// 创建一个订阅者，订阅 "motor_control" 话题
	motor_control_subscription_ = this->create_subscription<robot_message::msg::MotorControl>("motor_control", 10, 
		std::bind(&MotorManagerNode::Motor_Control_Message_Callback, this, std::placeholders::_1));
	
	// 定时器，定期发布消息（例如每秒发布一次）
	timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MotorManagerNode::Timer_Callback, this));
}


/**
  * @brief  MotorControl消息订阅回调函数
  * @param  msg	传入的消息形参  
  * @retval null
  * @usage  
  */
void MotorManagerNode::Timer_Callback(void)
{
	static uint16_t sequence_ = 0;

	// 创建 MotorControl 消息
	// auto msg = robot_message::msg::MotorControl();

	auto msg = robot_message::msg::MotorStateData();

	msg.sequence = sequence_++;

	// 填充 motor_state 数据
	for (int i = 0; i < 14; ++i)
	{
		msg.state[i].mode = 1;  // 示例值，设置电机模式
		msg.state[i].pos = 0.0f;  // 示例值，设置电机位置
		msg.state[i].w = 0.0f;  // 示例值，设置电机速度
		msg.state[i].t = 0.0f;  // 示例值，设置电机力矩
		// msg.state[i].kp = 25.0f;  // 示例值，设置电机温度
		// msg.state[i].kd = 25.0f;  // 示例值，设置电机温度
	}

	// 发布消息
	// motor_control_subscription_test_->publish(msg);
	motor_state_publisher_->publish(msg);
	//输出发布的信息
	RCLCPP_INFO(this->get_logger(), "Published motor state data, sequence: %d", msg.sequence);
}


/**
  * @brief  MotorControl消息订阅回调函数
  * @param  msg	传入的消息形参  
  * @retval null
  * @usage  
  */
void MotorManagerNode::Motor_Control_Message_Callback(const robot_message::msg::MotorControl::SharedPtr msg)
{
	/* 消息日志打印 */ 
	RCLCPP_INFO(this->get_logger(), "Received motor control, sequence: %d", msg->sequence);
	for (int i = 0; i < 14; ++i)
	{
		RCLCPP_INFO(this->get_logger(), 
			"Motor %d: Mode: %d, Pos: %.2f, W: %.2f, T: %.2f, kp: %.2f, kd: %.2f", 
			i, msg->cmd[i].mode, msg->cmd[i].pos, msg->cmd[i].w, msg->cmd[i].t, msg->cmd[i].kp, msg->cmd[i].kd);
	}

	/* 电机控制消息push到控制消息处理队列 */
	// MotorLowCommunication::Motor_Control_Queue_Push(msg);
}


MotorManagerNode::~MotorManagerNode() {
    RCLCPP_INFO(this->get_logger(), "MotorManagerNode is being destroyed!");
}



/**
  * @brief  主函数：ROS 2 程序的入口
  * @param  null
  * @retval null
  * @usage  
  */
int main(int argc, char *argv[])
{
	// 这一步是 ROS 2 的初始化过程，必须在任何 ROS 2 功能之前调用
	rclcpp::init(argc, argv);

	// 创建一个 MotorManagerNode 类的实例，并通过 rclcpp::spin 运行该节点
	// rclcpp::spin 会进入循环，直到节点被关闭或者终止
	rclcpp::spin(std::make_shared<MotorManagerNode>("motor_manager")); // this node name will be replaced by launch file name

	// 当程序退出时，调用 rclcpp::shutdown 来清理 ROS 2 环境
	rclcpp::shutdown();

	// 返回 0，表示程序正常结束
	return 0;
}

