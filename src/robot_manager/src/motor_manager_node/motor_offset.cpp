#include "motor_offset.h"

#include <libxml/parser.h>
#include <libxml/tree.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <cstring>

/*******************************************************************************
 * Variables
 ******************************************************************************/
const float arm_left_offset_value[8] = {0.0f, 0.0, 0.0, 0.0f,
                                        0.0f, 0.0f, 0.0f, 0.0f};

const float arm_left_polarity_value[8] = {1.0f, -1.0f, -1.0f, -1.0f,
                                          1.0f, -1.0f, -1.0f, 1.0f};

const float arm_right_offset_value[8] = {0.0, 0.0f, 0.0f, 0.0f,
                                        0.0f, 0.0f, 0.0f, 0.0f};

const float arm_right_polarity_value[8] = {-1.0f, -1.0f, -1.0f, 1.0f,
                                          1.0f, 1.0f, -1.0f, 1.0f};

/*******************************************************************************
 * Code
 ******************************************************************************/

/**
  * @brief  构造函数
  * @param  null  
  * @retval null
  * @usage  
  */
MotorOffsetTransition::MotorOffsetTransition() {
    // 初始化默认值
    float default_offset_value[14] = {1.05f, 0.174f, 0.349f, 1.57f,
                                        2.54f, 0.0f, 0.0f, 0.174f,
                                        0.349f, 0.873f, -0.0997f, 0.0f,
                                        0.0f, 0.0f};

    float default_polarity_value[14] = {-1.0f, 1.0f, -1.0f, 1.0f,
                                        -1.0f, -1.0f, 1.0f, 1.0f,
                                        -1.0f, -1.0f, 1.0f, 1.0f,
                                        -1.0f, 1.0f};

    std::memcpy(motor_offset_value, default_offset_value, sizeof(default_offset_value));
    std::memcpy(motor_polarity_value, default_polarity_value, sizeof(default_polarity_value));

    // 获取XML文件路径
    std::string package_share_path = ament_index_cpp::get_package_share_directory("robot_manager");
    std::string config_file_path = package_share_path + "/config/motor_config.xml";

    // 从XML文件中加载数据
    Motor_Congif_XML_Load(config_file_path);

    // 打印最终加载的参数
    std::cout << "Final motor offset values:" << std::endl;
    for (int i = 0; i < 14; ++i) {
        std::cout << "Motor " << i << ": " << motor_offset_value[i] << std::endl;
    }

    std::cout << "Final motor polarity values:" << std::endl;
    for (int i = 0; i < 14; ++i) {
        std::cout << "Motor " << i << ": " << motor_polarity_value[i] << std::endl;
    }
}

/**
  * @brief  析构函数
  * @param  null  
  * @retval null
  * @usage  
  */
MotorOffsetTransition::~MotorOffsetTransition() {
	;;
}

/**
  * @brief  电机xml配置加载函数
  * @param  file_path：文件路径  
  * @retval null
  * @usage  
  */
void MotorOffsetTransition::Motor_Congif_XML_Load(const std::string& file_path) 
{
    xmlDocPtr doc = xmlReadFile(file_path.c_str(), nullptr, 0);
    if (doc == nullptr) {
        std::cerr << "Failed to parse XML file: " << file_path << std::endl;
        return;
    }

    xmlNodePtr root = xmlDocGetRootElement(doc);
    if (root == nullptr) {
        std::cerr << "Empty XML document: " << file_path << std::endl;
        xmlFreeDoc(doc);
        return;
    }

    for (xmlNodePtr node = root->children; node != nullptr; node = node->next) {
        if (node->type == XML_ELEMENT_NODE && std::strcmp(reinterpret_cast<const char*>(node->name), "motor") == 0) {
            xmlChar* id_attr = xmlGetProp(node, reinterpret_cast<const xmlChar*>("id"));
            if (id_attr != nullptr) {
                int id = std::atoi(reinterpret_cast<const char*>(id_attr));
                if (id >= 0 && id < 14) {
                    xmlNodePtr child = node->children;
                    while (child != nullptr) {
                        if (child->type == XML_ELEMENT_NODE) {
                            if (std::strcmp(reinterpret_cast<const char*>(child->name), "offset") == 0) {
                                xmlChar* offset = xmlNodeGetContent(child);
                                if (offset != nullptr) {
                                    motor_offset_value[id] = std::atof(reinterpret_cast<const char*>(offset));
                                    xmlFree(offset);
                                }
                            } else if (std::strcmp(reinterpret_cast<const char*>(child->name), "polarity") == 0) {
                                xmlChar* polarity = xmlNodeGetContent(child);
                                if (polarity != nullptr) {
                                    motor_polarity_value[id] = std::atof(reinterpret_cast<const char*>(polarity));
                                    xmlFree(polarity);
                                }
                            }
                        }
                        child = child->next;
                    }
                }
                xmlFree(id_attr);
            }
        }
    }

    xmlFreeDoc(doc);
}

/**
  * @brief  电机角度零偏添加函数
  * @param  *data  
  * @retval null
  * @usage  
  */
float MotorOffsetTransition::Motor_Angle_Offset_Add(float raw_data, float offest)
{
  return raw_data-offest;
}

/**
  * @brief  电机角度零偏还原函数
  * @param  *data  
  * @retval null
  * @usage  
  */
float MotorOffsetTransition::Motor_Angle_Offset_Minus(float raw_data, float offest)
{
  return raw_data + offest;
}


/**
  * @brief  EtherCat电机状态零偏与极性处理函数
  * @param  *data  
  * @retval null
  * @usage  
  */
void MotorOffsetTransition::Motor_State_Polarity_Offset_Dispose(motor_state_data * data)
{
  for(volatile int i = 0; i < 14; i++)
  {
    data->state[i].pos = motor_polarity_value[i] * Motor_Angle_Offset_Add(data->state[i].pos, motor_offset_value[i]);

    data->state[i].t = motor_polarity_value[i] * data->state[i].t;

    data->state[i].w = motor_polarity_value[i] * data->state[i].w;
  }
}


/**
  * @brief  左手机械臂电机状态零偏与极性处理函数
  * @param  *data  
  * @retval null
  * @usage  
  */
void MotorOffsetTransition::Left_ARM_State_Polarity_Offset_Dispose(left_arm_state_data * data)
{
  for(volatile int i = 0; i < 8; i++)
  {
    data->state[i].pos = arm_left_polarity_value[i] * Motor_Angle_Offset_Add(data->state[i].pos, arm_left_offset_value[i]);

    data->state[i].t = arm_left_polarity_value[i] * data->state[i].t;

    data->state[i].w = arm_left_polarity_value[i] * data->state[i].w;
  }
}

/**
  * @brief  右手机械臂电机状态零偏与极性处理函数
  * @param  *data  
  * @retval null
  * @usage  
  */
void MotorOffsetTransition::Right_ARM_State_Polarity_Offset_Dispose(right_arm_state_data * data)
{
  for(volatile int i = 0; i < 8; i++)
  {
    data->state[i].pos = arm_right_polarity_value[i] * Motor_Angle_Offset_Add(data->state[i].pos, arm_right_offset_value[i]);

    data->state[i].t = arm_right_polarity_value[i] * data->state[i].t;

    data->state[i].w = arm_right_polarity_value[i] * data->state[i].w;
  }
}


/**
  * @brief  EtherCat电机命令零偏与极性处理函数
  * @param  *data  
  * @retval null
  * @usage  
  */
void MotorOffsetTransition::Motor_Cmd_Polarity_Offset_Dispose(motor_cmd_data * cmd)
{
  for(volatile int i = 0; i < 14; i++)
  {
    cmd->cmd[i].pos = Motor_Angle_Offset_Minus(motor_polarity_value[i] * cmd->cmd[i].pos, motor_offset_value[i]);

    cmd->cmd[i].t = motor_polarity_value[i] * cmd->cmd[i].t;

    cmd->cmd[i].w = motor_polarity_value[i] * cmd->cmd[i].w;
  }
}


/**
  * @brief  左手机械臂电机状态零偏与极性处理函数
  * @param  *data  
  * @retval null
  * @usage  
  */
void MotorOffsetTransition::Left_ARM_Cmd_Polarity_Offset_Dispose(left_arm_cmd_data * cmd)
{
  for(volatile int i = 0; i < 8; i++)
  {
    cmd->cmd[i].pos = Motor_Angle_Offset_Minus(arm_left_polarity_value[i] * cmd->cmd[i].pos, arm_left_offset_value[i]);

    cmd->cmd[i].t = arm_left_polarity_value[i] * cmd->cmd[i].t;

    cmd->cmd[i].w = arm_left_polarity_value[i] * cmd->cmd[i].w;
  }
}

/**
  * @brief  右手机械臂电机状态零偏与极性处理函数
  * @param  *data  
  * @retval null
  * @usage  
  */
void MotorOffsetTransition::Right_ARM_Cmd_Polarity_Offset_Dispose(right_arm_cmd_data * cmd)
{
  for(volatile int i = 0; i < 8; i++)
  {
    cmd->cmd[i].pos = Motor_Angle_Offset_Minus(arm_right_polarity_value[i] * cmd->cmd[i].pos, arm_right_offset_value[i]);

    cmd->cmd[i].t = arm_right_polarity_value[i] * cmd->cmd[i].t;

    cmd->cmd[i].w = arm_right_polarity_value[i] * cmd->cmd[i].w;
  }
}



