#pragma once

#include <iostream>
#include <string>
#include <cstring>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <functional>
#include <thread> 
#include <atomic>

#include "rclcpp/rclcpp.hpp"						// 引入 ROS 2 的核心头文件，包含对 rclcpp 库（C++ 客户端库）和基础节点类的支持
#include "std_msgs/msg/string.hpp"  				// 使用 ROS 2 标准消息类型 String
#include "robot_message/msg/motor_control.hpp" 	    // 引入自定义消息头文件

#include "ipc_communication.h"
#include "circular_queue.hpp"
#include "motor_offset.h"

/*******************************************************************************
 * Macro definition
 ******************************************************************************/
#define BECKOFF_SERVER_IP_ADDR 	"192.168.93.2"

#define BECKOFF_SERVER_PORT 	5050

#define IPC_LOCAL_POART  5050

/*******************************************************************************
 * Class definition
 ******************************************************************************/
class MotorLowCommunication : public IpcCommunication, public MotorOffsetTransition {
public:
    // 构造函数，传递回调函数并创建线程
    MotorLowCommunication();

    //析构函数
    ~MotorLowCommunication();
 
    struct sockaddr_in beckoff_server_addr;

    void Motor_Control_Queue_Push(robot_message::msg::MotorControl::SharedPtr message);

private:

    std::thread ipc_recive_therd, motor_cmd_send_therd;         //线程对象

    std::atomic<bool> stop_flag;                                //线程终止标志

    CircularQueue<robot_message::msg::MotorControl> motor_control_queue;  // 创建消息队列，类型为 MotorControl

    void Motor_Cmd_Send_Task(void);                     //电机命令发送线程

    void IpcReceiveDispose(void) override;				//ICP接收线程  

    void Motor_State_Recive_Callback(motor_state_data* state_data);

protected:	//受保护成员，只能在类或派生类调用
	
    virtual void Motor_Stete_Message_Push(motor_state_data* state_data);

};

