#pragma once

#include "cpp_types.h"

/*!
 * Mini Cheetah's IMU
 */
struct VectorNavData {
  Vec3<float> accelerometer;
  Vec3<float> gyro;
  Quat<float> quat; // convention: WXYZ
};

/*!
 * "Cheater" state sent to the robot from simulator
 */
template <typename T>
struct CheaterState {
  Quat<T> orientation;
  Vec3<T> position;
  Vec3<T> omegaBody;
  Vec3<T> vBody;
  Vec3<T> acceleration;
};


typedef struct GyroData {
  union {
    struct {
      float roll, pitch, yaw;
      float x_acc, y_acc, z_acc;
      float x_omega, y_omega, z_omega;
    } data;
    struct {
      float rpy[3];
      float acc[3];
      float rpy_rate[3];
    };
    float buffer[3][3];
    unsigned char buffer_byte[3][12];
  } gyro;

  uint64_t timestamp;
} GyroData;



