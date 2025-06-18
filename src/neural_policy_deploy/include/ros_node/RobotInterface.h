#pragma once

#include "rclcpp/rclcpp.hpp"						
#include "robot_message/msg/joint_state_group.hpp" 		// 引入自定义消息头文件
#include "robot_message/msg/joint_control_group.hpp" 	// 引入自定义消息头文件
#include "robot_message/msg/imu_data.hpp" 				// 引入自定义消息头文件
#include "sensor_msgs/msg/joy.hpp"
#include "state_machine/StateMachine.h"
#include "types/motor_data.h"

class RobotInterface : public rclcpp::Node{
public:
	RobotInterface(const std::string &node_name, StateMachine* controller_ptr);
	~RobotInterface();

private:
  	// Finite StateMachine that provides read and write destinations to buffers.
	StateMachine* controller_ptr_;

	// 发布者对象，发布消息到 "motor_status" 话题
	rclcpp::Publisher<robot_message::msg::JointControlGroup>::SharedPtr motor_cmd_publisher_;

	// 订阅者对象，订阅 "motor_control" 话题
	rclcpp::Subscription<robot_message::msg::JointStateGroup>::SharedPtr motor_state_subscriber_;
	rclcpp::Subscription<robot_message::msg::ImuData>::SharedPtr imu_state_subscriber_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;

	// 定时器对象
	rclcpp::TimerBase::SharedPtr timer_;

	//MotorControl消息订阅回调函数
	void Motor_State_Message_Callback(const robot_message::msg::JointStateGroup::SharedPtr msg);
	void IMU_State_Message_Callback(const robot_message::msg::ImuData::SharedPtr msg);
	void Joystick_Callback(const sensor_msgs::msg::Joy::SharedPtr msg);

	//motor_status消息发布函数
	void Motor_Cmd_Message_Publish(const std::shared_ptr<motor_cmd_data>& data);
	void Timer_Callback();
};

