#pragma once

#include <iostream>
#include <string>
#include <cstring>

#include "motor_data.h"

class MotorOffsetTransition {
public:
    // 构造函数，传递回调函数并创建线程
    MotorOffsetTransition();

    //析构函数
    ~MotorOffsetTransition();


protected:

    float motor_offset_value[14];

    float motor_polarity_value[14];

    void Motor_Congif_XML_Load(const std::string& file_path);

    float Motor_Angle_Offset_Add(float raw_data, float offest);

    float Motor_Angle_Offset_Minus(float raw_data, float offest);

    void Motor_State_Polarity_Offset_Dispose(motor_state_data * data);

    void Left_ARM_State_Polarity_Offset_Dispose(left_arm_state_data * data);

    void Right_ARM_State_Polarity_Offset_Dispose(right_arm_state_data * data);

    void Motor_Cmd_Polarity_Offset_Dispose(motor_cmd_data * cmd);

    void Left_ARM_Cmd_Polarity_Offset_Dispose(left_arm_cmd_data * cmd);

    void Right_ARM_Cmd_Polarity_Offset_Dispose(right_arm_cmd_data * cmd);
};
