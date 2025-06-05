// G1Sim2RealEnv.cpp
#include "real/G1Sim2RealEnv.h"
#include "utility/logger.h"

G1Sim2RealEnv::G1Sim2RealEnv(std::shared_ptr<const BaseRobotConfig> cfg, 
                            const std::string& net_interface,
                            jointCMD* jointCMDPtr,
                            robotStatus* robotStatusPtr)
    : cfg_(cfg),
      robotStatusPtr_(robotStatusPtr),
      jointCMDPtr_(jointCMDPtr),
      control_dt_(cfg->getPolicyDt()),
      // mode_pr_(Mode::PR),
      mode_machine_(0)
{   
    FRC_INFO("[G1Sim2RealEnv.Const] net_interface: " << net_interface);

    // ChannelFactory::Instance()->Init(0, net_interface);
    // remote_controller_ = std::make_shared<RemoteController>();

    // if (cfg_->msg_type == "hg") {
    //     low_cmd_ = unitree_hg::msg::dds_::LowCmd_();
    //     low_state_ = unitree_hg::msg::dds_::LowState_();

    //     lowcmd_publisher_ = std::make_shared<ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(cfg_->lowcmd_topic);
    //     lowcmd_publisher_->InitChannel();

    //     lowstate_subscriber_ = std::make_shared<ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(cfg_->lowstate_topic);
    //     lowstate_subscriber_->InitChannel(std::bind(&G1Sim2RealEnv::LowStateHgHandler, this, std::placeholders::_1), 10);
    // } else {
    //     throw std::runtime_error("Unsupported msg_type in cfg: " + cfg_->msg_type);
    // }

    // waitForLowState();

    // if (cfg_->msg_type == "hg") {
    //     initLowCmdHg();  // C++ 等效实现
    // } else {
    //     throw std::runtime_error("Unsupported msg_type in cfg: " + cfg_->msg_type);
    // }
}