#pragma once

#include <yaml-cpp/yaml.h>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <memory>
#include <mutex>
#include <Eigen/Core>
#include <vector>

#include "types/system_defined.h"
#include "core/RobotConfig.h"
#include "types/joystickTypes.h"

class MujocoManager {
public:
  MujocoManager(std::shared_ptr<const RobotConfig> cfg,
                GyroData* gyroPtr,
                jointStateData* motorInputPtr,
                jointTargetData* motorOutputPtr);
  ~MujocoManager();

  void initWorld();
  void loadRobotModel();
  void initState();
  void setupRobotProperties();
  void launchServer();
  void run();
  void updateRobotState();
  void integrate();
  void reset();
  void getContactForce();
  void stop() { running_ = false; glfwSetWindowShouldClose(window_, GLFW_TRUE); }
  void setUserInputPtr(char* key, JoystickData* joy) { joyPtr_ = joy; keyPtr_ = key; }

private:
  std::shared_ptr<const RobotConfig> cfg_;

  std::string robotName_;
  float control_dt_ = 2e-3;
  float simulation_dt_ = 5e-4;

  bool running_ = true;
  bool isStatesReady = false;
  bool isFixedBase = false;
  bool isRopeHanging = false;
  float ropeHeight_ = 0.9f;

  int gcDim_ = 0, gvDim_ = 0;
  int jointDim_ = 0;

  jointStateData* motorReadingBuf_ = nullptr;
  jointTargetData* motorCommandBuf_ = nullptr;
  GyroData* gyro_data_ = nullptr;
  JoystickData* joyPtr_ = nullptr;
  char* keyPtr_ = nullptr;

  std::mutex state_lock_;
  std::mutex action_lock_;

  Eigen::VectorXd gc_init_, gv_init_;
  Eigen::VectorXd gc_, gv_, gv_prev_;
  Eigen::VectorXf pTarget, vTarget;
  Eigen::VectorXf tauCmd, tauCmd_joint;
  Eigen::VectorXf jointPGain, jointDGain;
  Eigen::VectorXd contactForce;

  std::vector<int> footBodyIds{0, 0};
  std::vector<int> footGeomIds{0, 0};

  // MuJoCo core members
  mjModel* mj_model_ = nullptr;
  mjData* mj_data_ = nullptr;
  GLFWwindow* window_ = nullptr;
  mjvCamera cam_;
  mjvOption opt_;
  mjvScene scn_;
  mjrContext con_;
  // mjModel*           mj_model_         = nullptr;
  // mjData*            mj_data_          = nullptr;
  // GLFWwindow*        glfw_window_      = nullptr;
  // mjvCamera          camera_;                  // 摄像机
  // mjvOption          viz_option_;              // 可视化选项
  // mjvScene           render_scene_;            // 渲染场景缓存
  // mjrContext         render_context_;          // OpenGL 渲染上下文
};