#pragma once
#include "core/BaseRobotConfig.h"
#include "types/system_defined.h"
#include "types/joystickTypes.h"
#include <memory>
#include "hardware/listener.h"
#include "types/CustomTypes.h"
#include "utility/data_buffer.h"
// unitree
// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

enum class Mode {
  PR = 0,  // Series Control for Ptich/Roll Joints
  AB = 1   // Parallel Control for A/B Joints
};

class G1Sim2RealEnv {
public:
  G1Sim2RealEnv(std::shared_ptr<const BaseRobotConfig> cfg, 
                const std::string& net_interface,
                std::shared_ptr<DataBuffer<jointCMD>> jointCMDBufferPtr,
                std::shared_ptr<DataBuffer<robotStatus>> robotStatusBufferPtr);

  G1Sim2RealEnv(std::shared_ptr<const BaseRobotConfig> cfg,
                const std::string& net_interface,
                std::shared_ptr<DataBuffer<jointCMD>> jointCMDBufferPtr,
                std::shared_ptr<DataBuffer<robotStatus>> robotStatusBufferPtr,
                const std::string& mode,
                const std::string& track,
                const std::vector<std::string>& track_list,
                std::shared_ptr<CustomTypes::MocapConfig> mocap_cfg,  
                std::shared_ptr<CustomTypes::VlaConfig> vla_cfg);    

  void setUserInputPtr(char* key, JoystickData* joy, std::shared_ptr<Listener> listener) { keyPtr_ = key; joyPtr_ = joy; listenerPtr_ = listener; }
  void stop() { running_ = false;}
  void LowStateHandler(const void *message);
  void initState();
  void updateRobotState();
  void waitForLowState();
  void simulateRobot();
  void sendCmd(LowCmd_& cmd);
  void zeroTorqueState();
  void moveToDefaultPos();
  void defaultPosState();
  void run();

private:
  // 对应构造参数的成员变量
  std::shared_ptr<const BaseRobotConfig> cfg_;
  std::string net_interface_;
  std::shared_ptr<DataBuffer<robotStatus>> robotStatusBufferPtr_;
  std::shared_ptr<DataBuffer<jointCMD>> jointCMDBufferPtr_;
  std::string mode_;
  std::string track_;
  std::vector<std::string> track_list_;
  std::shared_ptr<CustomTypes::MocapConfig> mocap_cfg_;
  std::shared_ptr<CustomTypes::VlaConfig> vla_cfg_;

  bool running_ = true;

  JoystickData* joyPtr_ = nullptr;
  char* keyPtr_ = nullptr;
  std::shared_ptr<Listener> listenerPtr_;

  float control_dt_;

  std::mutex state_lock_;
  std::mutex action_lock_;

  Eigen::VectorXd gc_, gv_;
  Eigen::VectorXf pTarget, vTarget;
  Eigen::VectorXf tauCmd;
  Eigen::VectorXf jointPGain, jointDGain;
  int gcDim_ = 0, gvDim_ = 0;
  int jointDim_ = 0;
  int counter_ = 0;

  // unitree sdk
  uint8_t mode_machine_;
  Mode mode_pr_;
  LowCmd_ low_cmd_;
  LowState_ low_state_;

  // publisher
  std::unique_ptr<ChannelPublisher<LowCmd_>> lowcmd_publisher_;
  // subscriber
  std::unique_ptr<ChannelSubscriber<LowState_>> lowstate_subscriber_;

  unitree::common::ThreadPtr command_writer_ptr_, control_thread_ptr_;
};
