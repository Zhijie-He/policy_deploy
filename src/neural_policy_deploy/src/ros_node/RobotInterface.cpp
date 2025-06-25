#include "ros_node/RobotInterface.h"
#include "utility/logger.h"

RobotInterface::RobotInterface(const std::string &node_name, StateMachine* controller_ptr):
	rclcpp::Node(node_name), controller_ptr_(controller_ptr) 
{
	FRC_INFO("[RobotInterface.Const] Robot Receiver Node Start, Versions V1.0.1.!");
	rclcpp::SensorDataQoS qos;
	qos.keep_last(1);

	// Create puvlisher to "joint_control_group", including joint level PD commands
	motor_cmd_publisher_ = this->create_publisher<robot_message::msg::JointControlGroup>("joint_control_group", 10);

	// Create subscriber to "joint_state_group" topic, receiving joint measurements.
	motor_state_subscriber_ = this->create_subscription<robot_message::msg::JointStateGroup>(
		"joint_state_group", qos, std::bind(&RobotInterface::Motor_State_Message_Callback, this, std::placeholders::_1));

	// Create subscriber to "imu_data" topic, receiving IMU measurements.
	imu_state_subscriber_ = this->create_subscription<robot_message::msg::ImuData>(
		"imu_data", qos, std::bind(&RobotInterface::IMU_State_Message_Callback, this, std::placeholders::_1));
	
	// Create subscriber to "joy" topic, receiving joystick data.
	joystick_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
			"joy", qos, std::bind(&RobotInterface::Joystick_Callback, this, std::placeholders::_1));

	// Timer for publish. 100Hz.
	timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&RobotInterface::Timer_Callback, this));
}

void RobotInterface::Motor_State_Message_Callback(const robot_message::msg::JointStateGroup::SharedPtr msg) {
	// 电机控制消息push到控制消息处理队列
	auto data = std::make_shared<motor_state_data>();
	data->sequence = msg->sequence;
	data->timestamp = msg->timestamp;

	// this urdf parse indices need to put in cfg?
	const int URDF_indices[14] = {7,8,9,10,11,12, 1,2,3,4,5,6, 0,13}; /// From Hardware order to URDF order
	for (int i = 0; i < 14; i++) {
		data->state[i].mode = msg->state[URDF_indices[i]].mode;
		data->state[i].pos = msg->state[URDF_indices[i]].pos;
		data->state[i].w = msg->state[URDF_indices[i]].w;
		data->state[i].t = msg->state[URDF_indices[i]].t;
	}
	controller_ptr_->setMotorStatePtr(data);
	/* 消息日志打印 */
	// auto t = rclcpp::Clock().now();
	// RCLCPP_INFO(this->get_logger(), "Got joint state, from: %ld, which is %.4fms ago", msg->timestamp, (t.nanoseconds()-msg->timestamp)/1e6);
	// RCLCPP_INFO(this->get_logger(), "Received motor state, sent: %ld, received: %ld", msg->timestamp, t.nanoseconds());
}

void RobotInterface::IMU_State_Message_Callback(const robot_message::msg::ImuData::SharedPtr msg) {
	auto data = std::make_shared<GyroData>();
	// data->timestamp = msg->timestamp;
	for (int i = 0; i < 3; i++) {
		data->gyro.group.rpy[i] = msg->rpy[i] * M_PI / 180.0;
		data->gyro.group.acc[i] = msg->accelerometer[i];
		data->gyro.group.rpy_rate[i] = msg->gyroscope[i];
	}
	controller_ptr_->setGyroPtr(data);

	/* 消息日志打印 */
	// auto t = rclcpp::Clock().now();
	// RCLCPP_INFO(this->get_logger(), "Received imu state, sent: %ld, received: %ld", msg->timestamp, t.nanoseconds());
}

void RobotInterface::Joystick_Callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
	// Extract joystick data
	std::vector<float> axes = msg->axes;
	std::vector<int32_t> buttons = msg->buttons;
	auto joystick_data = std::make_shared<JoystickData>(zeroJoystick());

	joystick_data->isAlive = true;
	joystick_data->JoyLeft << axes[XBOX_AXIS_LY], axes[XBOX_AXIS_LX];
	joystick_data->JoyRight << axes[XBOX_AXIS_RY], axes[XBOX_AXIS_RX];
	joystick_data->pad << axes[XBOX_AXIS_YY], axes[XBOX_AXIS_XX];
	joystick_data->triggerLeft = (1.f - axes[XBOX_AXIS_LT]) / 2.f;
	joystick_data->triggerRight = (1.f - axes[XBOX_AXIS_RT]) / 2.f;
	joystick_data->buttonA = buttons[XBOX_BUTTON_A];
	joystick_data->buttonB = buttons[XBOX_BUTTON_B];
	joystick_data->buttonX = buttons[XBOX_BUTTON_X];
	joystick_data->buttonY = buttons[XBOX_BUTTON_Y];
	joystick_data->shoulderLeft = buttons[XBOX_BUTTON_LB];
	joystick_data->shoulderRight = buttons[XBOX_BUTTON_RB];
	joystick_data->buttonStart = buttons[XBOX_BUTTON_START];

	controller_ptr_->setJoystickPtr(joystick_data);

	RCLCPP_INFO(this->get_logger(), "Axes: [%.2f, %.2f, %.2f, %.2f]", axes[0], axes[1], axes[2], axes[3]);
	RCLCPP_INFO(this->get_logger(), "Buttons: [%d, %d, %d, %d]", buttons[0], buttons[1], buttons[2], buttons[3]);
}

void RobotInterface::Motor_Cmd_Message_Publish(const std::shared_ptr<motor_cmd_data>& data) {
	if(data == nullptr) {
		FRC_INFO("[RobotInterface.cmd_publish] Failed to publish, got Nullptr for data.");
		return;
	}

	// Instantiate JointControl message.
	auto msg = robot_message::msg::JointControlGroup();
	msg.sequence = data->sequence;
	msg.timestamp = data->timestamp;

	// Fill data into message with loop.
	const int URDF_indices[14] = {7,8,9,10,11,12, 1,2,3,4,5,6, 0,13}; /// From Hardware order to URDF order
	for (int i = 0; i < 14; i++) {
		msg.cmd[URDF_indices[i]].mode = data->cmd[i].mode;
		msg.cmd[URDF_indices[i]].pos = data->cmd[i].pos;
		msg.cmd[URDF_indices[i]].w = data->cmd[i].w;
		msg.cmd[URDF_indices[i]].t = data->cmd[i].t;
		msg.cmd[URDF_indices[i]].kp = data->cmd[i].kp;
		msg.cmd[URDF_indices[i]].kd = data->cmd[i].kd;
	}

	motor_cmd_publisher_->publish(msg);
	
	// auto t = rclcpp::Clock().now();
	// RCLCPP_INFO(this->get_logger(), "Pub command, from: %ld, which is %.4fms ago", msg.timestamp, (t.nanoseconds()-msg.timestamp)/1e6);
}

void RobotInterface::Timer_Callback() {
	controller_ptr_->updateAction();
	const auto data = controller_ptr_->getMotorTargetPtr();
	Motor_Cmd_Message_Publish(data);
	if (!controller_ptr_->isRunning()) rclcpp::shutdown();
}

RobotInterface::~RobotInterface() {
	FRC_INFO("[RobotInterface.~RobotInterface] Destroying Robot Receiver Node!");
}
