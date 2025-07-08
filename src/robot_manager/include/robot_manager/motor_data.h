#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstring>
#include <mutex>
#include <chrono>

#include <thread>


#pragma pack(push, 1)  // 保存当前对齐状态，并设置为1字节对齐

struct motor_cmd {
  unsigned int mode;   //工作模式 0x00 0x0a
  float pos;      //Desired shaft position of motor【期望电机本身的位置（rad）】
  float w;        //Desired output speed of motor【期望电机本身的速度(rad/s)】
  float t;        //Desired output torque of motor【期望电机本身的输出力矩（Nm）】
  float kp;       //The position stiffness【电机本身的位置刚度系数】
  float kd;       //The speed stiffness【电机本身的速度刚度系数】
};

struct motor_cmd_data {
  unsigned char header1;     
  unsigned char header2; 

  unsigned short sequence;   

  unsigned int data_type;
  
  motor_cmd cmd[14];    //电机指令数据
  
  unsigned int crc_32;      //校验码

};

struct left_arm_cmd_data {
  unsigned char header1;     
  unsigned char header2; 

  unsigned short sequence;   

  unsigned int data_type;
  
  motor_cmd cmd[8];    //电机指令数据
  
  unsigned int crc_32;      //校验码

};

struct right_arm_cmd_data {
  unsigned char header1;     
  unsigned char header2; 

  unsigned short sequence;   

  unsigned int data_type;
  
  motor_cmd cmd[8];    //电机指令数据
  
  unsigned int crc_32;      //校验码

};

struct motor_state { 
  unsigned int mode;         //工作模式
  float pos;            //The motor shaft position(control board zero fixed)【当前电机位置】
  float w;              //The motor shaft speed【当前实际电机速度】
  float t;              //The output torque of motor【当前实际电机输出力矩】
  float temperature;  //电机温度
};

struct motor_state_data {
  unsigned char header1;
  unsigned char header2;
  unsigned short sequence;
  unsigned int data_type;
  
  // 电机顺序

  motor_state state[14]; //电机状态数据
  
  unsigned int crc_32;       //校验码

};

struct left_arm_state_data {
  unsigned char header1;
  unsigned char header2;
  unsigned short sequence;
  unsigned int data_type;
  
  // 电机顺序

  motor_state state[8]; //电机状态数据
  
  unsigned int crc_32;       //校验码

};

struct right_arm_state_data {
  unsigned char header1;
  unsigned char header2;
  unsigned short sequence;
  unsigned int data_type;
  
  // 电机顺序

  motor_state state[8]; //电机状态数据
  
  unsigned int crc_32;       //校验码

};

/*********** END *************/

struct Motor_Param_Curve {
	float motor_current_pos;

	float motor_current_w;

	float motor_current_t;

  float motor_current_temp;

  float motor_target_value;

  unsigned char tail[4];
  
};


#pragma pack(pop)  // 恢复之前的对齐状态
