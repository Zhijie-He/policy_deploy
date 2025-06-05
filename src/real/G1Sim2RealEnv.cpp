// G1Sim2RealEnv.cpp
#include "real/G1Sim2RealEnv.h"
#include "utility/logger.h"
#include <thread>

void print_lowcmd(const LowCmd_& cmd) {
    std::cout << "=== LowCmd_ ===" << std::endl;
    std::cout << "Mode PR: " << (int)cmd.mode_pr() << std::endl;
    std::cout << "Mode Machine: " << (int)cmd.mode_machine() << std::endl;

    for (size_t i = 0; i < cmd.motor_cmd().size(); ++i) {
        const auto& m = cmd.motor_cmd().at(i);
        std::cout << "Motor[" << i << "]"
                  << "  mode=" << (int)m.mode()
                  << "  tau=" << m.tau()
                  << "  q=" << m.q()
                  << "  dq=" << m.dq()
                  << "  kp=" << m.kp()
                  << "  kd=" << m.kd()
                  << std::endl;
    }
}

void init_cmd_hg(LowCmd_& cmd, uint8_t mode_machine, Mode mode_pr) {
  cmd.mode_machine() = mode_machine;
  cmd.mode_pr() = static_cast<uint8_t>(mode_pr);
  // here should follow num actions
  size_t size = cmd.motor_cmd().size();
  for (size_t i = 0; i < size; ++i) {
      cmd.motor_cmd()[i].mode() = 1;
      cmd.motor_cmd()[i].q() = 0;
      cmd.motor_cmd()[i].dq() = 0;
      cmd.motor_cmd()[i].kp() = 0;
      cmd.motor_cmd()[i].kd() = 0;
      cmd.motor_cmd()[i].tau() = 0;
  }
}

G1Sim2RealEnv::G1Sim2RealEnv(std::shared_ptr<const BaseRobotConfig> cfg, 
                            const std::string& net_interface,
                            std::shared_ptr<jointCMD> jointCMDPtr,
                            std::shared_ptr<robotStatus> robotStatusPtr)
    : cfg_(cfg),
      robotStatusPtr_(robotStatusPtr),
      jointCMDPtr_(jointCMDPtr),
      control_dt_(cfg->getPolicyDt()),
      mode_pr_(Mode::PR),
      mode_machine_(0)
{   
    FRC_INFO("[G1Sim2RealEnv.Const] net_interface: " << net_interface);

    // initialize DDS communication
    ChannelFactory::Instance()->Init(0); // net_interface.c_str());
    if(cfg_->msg_type == "hg"){
      // create publisher
      lowcmd_publisher_ = std::make_unique<ChannelPublisher<LowCmd_>>(cfg_->lowcmd_topic);
      lowcmd_publisher_->InitChannel();
      
      // create subscriber
      lowstate_subscriber_ = std::make_unique<ChannelSubscriber<LowState_>>(cfg_->lowstate_topic);
      lowstate_subscriber_->InitChannel(
        [this](const void *message) {
          this->LowStateHandler(message);
        }, 10
      );
    } else {
      FRC_ERROR("[G1Sim2RealEnv] Invalid msg_type" << cfg_->msg_type);
      throw std::invalid_argument("Invalid msg_type: " + cfg_->msg_type);
    }
    
    // wait for the subscriber to receive data
    // waitForLowState();
    init_cmd_hg(low_cmd_, mode_machine_, mode_pr_);
    print_lowcmd(low_cmd_);
}


void G1Sim2RealEnv::LowStateHandler(const void *message) {
  LowState_ low_state = *(const LowState_ *)message;
  const auto &rpy = low_state.imu_state().rpy();
  FRC_INFO("[LowState] imu rpy: " << std::fixed << std::setprecision(2) << rpy[0] << " " << rpy[1] << " " << rpy[2]);

  // const auto& remote = low_state.wireless_remote(); // std::array<uint8_t, 40>
  // for (int i = 0; i < 40; ++i) {
  //     printf("remote[%02d] = 0x%02X\n", i, remote[i]);
  // }

  // 手柄数据解析
  if (listenerPtr_) {
    listenerPtr_->remote.set(low_state.wireless_remote().data());
    // FRC_INFO("[Remote] Axis: " << listenerPtr_->remote.button[KeyMap::start]);
    FRC_INFO("[Remote] lx: " << listenerPtr_->remote.lx
          << " rx: " << listenerPtr_->remote.rx
          << " ry: " << listenerPtr_->remote.ry
          << " ly: " << listenerPtr_->remote.ly);
  }
}

void G1Sim2RealEnv::waitForLowState() {
    while (low_state_.tick() == 0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(cfg_->getPolicyDt()));
    }
    FRC_INFO("[G1Sim2RealEnv.waitForLowState] Successfully connected to the robot.");
}

void G1Sim2RealEnv::simulateRobot() {   
    // 创建 publisher
    auto state_pub = std::make_unique<ChannelPublisher<LowState_>>(cfg_->lowstate_topic);
    state_pub->InitChannel();

    // 构造并发布消息
    LowState_ fake_state;
    fake_state.imu_state().rpy()[0] = 0.1;
    fake_state.imu_state().rpy()[1] = 0.2;
    fake_state.imu_state().rpy()[2] = 0.3;

    while (true) {
        state_pub->Write(fake_state);  // 发布
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

void G1Sim2RealEnv::zeroTorqueState() {
  FRC_INFO("[G1Sim2RealEnv.zeroTorqueState] zeroTorqueState.");
}
