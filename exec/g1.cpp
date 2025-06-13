#include <algorithm>
#include <filesystem>
#include <csignal>
#include "state_machine/StateMachine.h"
#include "hardware/listener.h"
#include "sim2/real/g1_sim2real_env.h"
#include "sim2/simulator/g1_sim2mujoco_env.h"
#include "utility/tools.h"

#define LOG_USE_COLOR 1
#define LOG_USE_PREFIX 1
#include "utility/logger.h"

class G1Controller {
public:
  G1Controller(const std::string& net,
               std::shared_ptr<BaseRobotConfig> cfg,
               const std::string& config_name,
               const std::string& mode,
               const std::string& track,
               const std::vector<std::string>& track_list,
               std::shared_ptr<CustomTypes::MocapConfig> mocap_cfg,
               std::shared_ptr<CustomTypes::VlaConfig> vla_cfg)
      : cfg_(cfg), mode_(mode), track_(track), track_list_(track_list) ,mocap_cfg_(mocap_cfg), vla_cfg_(vla_cfg){
        state_machine_ = std::make_shared<StateMachine>(cfg, config_name);

        if (mode == "sim2mujoco") hu_env_ = std::make_shared<G1Sim2MujocoEnv>(cfg, state_machine_);
        else if(mode == "sim2real") hu_env_ = std::make_shared<G1Sim2RealEnv>(net, cfg, state_machine_);
        else throw std::runtime_error("Unsupported mode!");

        listener_ = std::make_shared<Listener>();
        hu_env_->setUserInputPtr(listener_, listener_->getKeyInputPtr(), nullptr);
        state_machine_->setInputPtr(listener_->getKeyInputPtr(), nullptr);

        threads_.emplace_back([&]() { listener_->listenKeyboard(); });             // start keyboard listener
        // threads_.emplace_back([&]() { state_machine_->run(); });                // start async policy
  }

  void zero_torque_state(){
    if(mode_=="sim2real") hu_env_->zeroTorqueState();
  }

  void move_to_default_pose(){
    hu_env_->moveToDefaultPos();
  }

  void default_pos_state(){
    if(mode_=="sim2real") hu_env_->defaultPosState();
  }

  void start_threads(){
    if(mode_ != "sim2mujoco") return;
    threads_.emplace_back([&]() { hu_env_->integrate(); }); 
    threads_.emplace_back([&]() { hu_env_->run(); });      
  }

  void run(){
    if(mode_ == "sim2mujoco"){
      start_threads();
      hu_env_->renderLoop();
    }else if(mode_=="sim2real"){
      hu_env_->run();
    }
  }

  ~G1Controller(){
    if(state_machine_) state_machine_->stop();
    if(listener_) listener_->stop();
    if(hu_env_) hu_env_->stop();

    for(auto& t : threads_){
      if(t.joinable()) t.join();
    }
  }

  std::shared_ptr<Listener> listener_ = nullptr;
  std::shared_ptr<StateMachine> state_machine_ = nullptr;
  std::shared_ptr<BaseEnv> hu_env_ = nullptr;

private:
  std::shared_ptr<BaseRobotConfig> cfg_ = nullptr;

  std::string mode_;
  std::string track_;
  std::vector<std::string> track_list_;
  std::shared_ptr<CustomTypes::MocapConfig> mocap_cfg_;
  std::shared_ptr<CustomTypes::VlaConfig> vla_cfg_;

  std::vector<std::thread> threads_;
};

std::shared_ptr<BaseRobotConfig> cfg = nullptr;
std::unique_ptr<G1Controller> controller = nullptr;

void close_all_threads(int signum) {
  FRC_INFO("Interrupted with SIGINT: " << signum << "\n");
  controller.reset();
  std::exit(0);
}

int main(int argc, char** argv) {
  std::string exec_name = std::filesystem::path(argv[0]).filename().string();
  std::string mode = argv[1];        // sim2mujoco or sim2real
  std::string config_name = argv[2]; // config name like g1_eman
  std::string net = (argc >= 4) ? argv[3] : "";

  // 参数num检查
  if (argc < 3) {
      FRC_ERROR("Usage: " << exec_name << " <mode> <config_name> [net]");
      FRC_ERROR("Example: " << exec_name << " sim2mujoco g1_eman");
      FRC_ERROR("         " << exec_name << " sim2real g1_eman net");
      return -1;
  }

  // 模式合法性检查
  if (mode != "sim2mujoco" && mode != "sim2real") {
    FRC_ERROR("Invalid mode: " << mode);
    FRC_ERROR("Available modes: sim2mujoco | sim2real");
    return -1;
  }

  // 配置合法性检查
  const std::vector<std::string> valid_configs = {"g1_unitree", "g1_eman", "h1", "h1_2"};
  if (std::find(valid_configs.begin(), valid_configs.end(), config_name) == valid_configs.end()) {
      std::ostringstream oss;
      oss << "Available config names: ";
      for (size_t i = 0; i < valid_configs.size(); ++i) {
          oss << valid_configs[i];
          if (i != valid_configs.size() - 1) oss << " | ";
      }
      FRC_ERROR("Invalid config name: " << config_name);
      FRC_ERROR(oss.str());
      return -1;
  }

  // sim2real 限制：必须带 net 参数.
  if (mode == "sim2real" && net.empty()) {
    FRC_ERROR("Missing <net> argument for mode 'sim2real'.");
    FRC_ERROR("Usage: " << exec_name << " sim2real g1_eman net");
    return -1;
  }

  // sim2real 限制：只支持 g1_eman，不支持 g1_unitree
  if (mode == "sim2real" && config_name != "g1_eman") {
    FRC_ERROR("Mode 'sim2real' is not supported with config " << config_name);
    FRC_ERROR("Only 'g1_eman' is supported in sim2real mode.");
    return -1;
  }

  signal(SIGINT, close_all_threads);

  // mode: sim2mujoco, sim2real
  // track: cmd, fpos, mocap, vla
  std::string track = "cmd";
  std::vector<std::string> track_list = {"cmd"};
  std::shared_ptr<CustomTypes::MocapConfig> mocap_cfg = nullptr;
  std::shared_ptr<CustomTypes::VlaConfig> vla_cfg = nullptr;

  // std::vector<std::string> track_list = {"cmd", "mocap"};
  // auto mocap_cfg = std::make_shared<CustomTypes::MocapConfig>("192.168.123.111", 7003, 30);

  // std::vector<std::string> track_list = {"cmd", "vla"};
  // auto vla_cfg = std::make_shared<CustomTypes::VlaConfig>("example_data.pkl");

  try {
    cfg = tools::loadConfig(config_name);
    
    controller = std::make_unique<G1Controller>(net, cfg, config_name, mode, track, track_list, mocap_cfg, vla_cfg);

    // Enter the zero torque state, press the start key to continue executing
    controller->zero_torque_state();
    // Move to the default position
    controller->move_to_default_pose();
    // Enter the default position state, press the A key to continue executing
    controller->default_pos_state();

    controller->run();

    close_all_threads(404);
    
  } catch (const std::exception& e) {
      FRC_ERROR("Failed to construct G1Controller: " << e.what());
      return -1;
  }

  return 0;
}
