/*
 * system_defined.h
 *
 *  Created on: 2017-11-4
 *      Author: moxiaobo
 */

#pragma once

#define SLAVE_NUMBER 64
#define SLAVE_NUMBER_TRIPLE (SLAVE_NUMBER*3)
#define SCOPE_SIZE 200

#include <cinttypes>

// 定义低层机器人传感器/控制数据的共享结构体，用于通信或控制
typedef struct GyroData
{
  union
  {
    struct
    {
      float roll, pitch, yaw; // 欧拉角：姿态
      float x_acc, y_acc, z_acc; // 加速度
      float x_omega, y_omega, z_omega; // 角速度
      uint64_t timestamp;
    } data;
    struct
    {
      float rpy[3];  // 姿态角（Attitude Angles）= 机体在三维空间中的旋转状态
      float acc[3];   // 加速度
      float rpy_rate[3];  // 角速度
      uint64_t timestamp;
    };
    float buffer[3][3];
    unsigned char buffer_byte[3][12];
  } gyro;
} GyroData;
// Gyro 是 Gyroscope（陀螺仪） 的缩写。 “陀螺仪传感器的数据结构” / “惯性测量单元（IMU）数据结构”‘
// 🔁 通常你能从 IMU 里直接拿到：
// 加速度： [ax, ay, az]        ← X/Y/Z 方向的线性加速度
// 角速度： [ωx, ωy, ωz]       ← 绕 X/Y/Z 的角速度（陀螺仪）
// 地磁场（可选）：[mx, my, mz]← 用于航向估计（如北偏角）
// 姿态角（rpy）	从角速度积分得到，或融合加速度/磁力计	估算 roll/pitch/yaw  姿态角（roll, pitch, yaw） 是机器人/飞行器在三维空间中朝向和方向的角度表示方式，分别对应绕 X/Y/Z 轴的旋转。
// 这是一个嵌套了 union 的结构体，核心是这一段：
// union {
//   struct {...} data;
//  struct {...};
//  float buffer[3][3];
//  unsigned char buffer_byte[3][12];
// };
// 🚨 所有这些字段（结构体 data、匿名结构体、buffer、buffer_byte）共享同一块内存！


/// logical robot data
typedef union LegsData
{
  struct
  {
    float position[SLAVE_NUMBER]; // 各关节位置
    float velocity[SLAVE_NUMBER]; // 各关节速度
    float ampere[SLAVE_NUMBER];   // 各关节电流/力矩估计
    uint64_t timestamp;
  } data;
  float buffer[3][SLAVE_NUMBER];  // 批量访问
} jointStateData;

// jointTargetData
// 所有关节的：
// 控制目标（pos/vel/amp）
// 控制增益（kp/kd）

typedef union
{
  struct
  {
    float position[SLAVE_NUMBER];
    float velocity[SLAVE_NUMBER];
    float ampere[SLAVE_NUMBER];
    float kp[SLAVE_NUMBER];  // 每个关节的 P 控制增益
    float kd[SLAVE_NUMBER];   // 每个关节的 D 控制增益
    uint64_t timestamp;
  } data;
  float buffer[5][SLAVE_NUMBER];
} jointTargetData;

typedef union
{
  struct
  {
    float position[SLAVE_NUMBER];
    float velocity[SLAVE_NUMBER];
    float kp[SLAVE_NUMBER];  // 每个关节的 P 控制增益
    float kd[SLAVE_NUMBER];   // 每个关节的 D 控制增益
    uint64_t timestamp;
  } data;
  float buffer[4][SLAVE_NUMBER];
} jointCMD;


/// logical robot data
typedef union humanData
{
  struct
  {
    float position[SLAVE_NUMBER]; 
    float velocity[SLAVE_NUMBER]; 
    float jointTorques[SLAVE_NUMBER];
    float timestamp;
  } data;
  float buffer[3][SLAVE_NUMBER];  // 批量访问
} robotStatus;


