#include <filesystem>
#include <csignal>
#include "state_machine/StateMachine.h"
#include "hardware/listener.h"
#ifdef USE_UNITREE_SDK
#include "sim2/real/g1_sim2real_env.h"
#endif
#include "sim2/simulator/g1_sim2mujoco_env.h"
#include "utility/tools.h"
#include "utility/cxxopts.hpp"

#define LOG_USE_COLOR 1
#define LOG_USE_PREFIX 1
#include "utility/logger.h"

class G1Controller {
public:
  G1Controller(const std::string& net,
               const std::string& mode,
               const std::vector<std::pair<std::string, char>>& registers,
               std::shared_ptr<BaseRobotConfig> cfg,
               const std::string& hands_type,
               torch::Device device,

               bool headless,
               const std::string& inference_engine_type,
               const std::string& precision)
  {
    state_machine_ = std::make_shared<StateMachine>(cfg, device, registers, hands_type, inference_engine_type, precision);
    
    if (mode == "sim2mujoco") hu_env_ = std::make_shared<G1Sim2MujocoEnv>(cfg, hands_type, state_machine_);
#ifdef USE_UNITREE_SDK
    else if(mode == "sim2real") hu_env_ = std::make_shared<G1Sim2RealEnv>(net, cfg, hands_type, state_machine_);
#endif 
    else throw std::runtime_error("Invalid mode: " + mode);
    
    listener_ = std::make_shared<Listener>();
    listener_->setKeyboardCallback(std::bind(&StateMachine::handleKeyboardInput, state_machine_.get(), std::placeholders::_1));
    listener_->start();

    hu_env_->setHeadless(headless); 
    hu_env_->setUserInputPtr(listener_, listener_->getKeyInputPtr(), nullptr);
    state_machine_->setInputPtr(listener_->getKeyInputPtr(), nullptr);
  }

  void zero_torque_state(){
    hu_env_->zeroTorqueState();
  }

  void move_to_default_pose(){
    hu_env_->moveToDefaultPos();
  }

  void default_pos_state(){
    hu_env_->defaultPosState();
  }

  void run(){
    hu_env_->run();
  }

  ~G1Controller(){
    if(hu_env_) hu_env_->stop();
    if(listener_) {
      listener_->stop();
      listener_->join();
    }
  }
  
private:
  std::shared_ptr<Listener> listener_ = nullptr;
  std::shared_ptr<StateMachine> state_machine_ = nullptr;
  std::shared_ptr<BaseEnv> hu_env_ = nullptr;
};

std::shared_ptr<BaseRobotConfig> config = nullptr;
std::unique_ptr<G1Controller> controller = nullptr;

void close_all_threads(int signum) {
  FRC_INFO("Interrupted with SIGINT: " << signum << "\n");
  controller.reset();
  std::exit(0);
}

int main(int argc, char** argv) {  
  std::string exec_name = std::filesystem::path(argv[0]).filename().string();
  std::string mode;
  std::string config_name;
  bool headless = false;
  std::string net;
  torch::Device device = torch::kCPU;
  std::string inference_engine_type = "libtorch";
  std::string precision = "fp32";
  std::string hands_type = "box";
  
  try {
    cxxopts::Options options(exec_name, "Run Mujoco-based simulation Or Real for Human Legged Robot");
    options.add_options()
      ("m,mode", "Mode: sim2mujoco or sim2real", cxxopts::value<std::string>())
      ("c,config", "Config name: g1_eman", cxxopts::value<std::string>())
      ("headless", "Run in headless mode (no GUI)", cxxopts::value<bool>()->default_value("false"))
      ("d,device", "Device to use: cpu or cuda", cxxopts::value<std::string>()->default_value("cpu"))
      ("n,net", "Network interface name for sim2real", cxxopts::value<std::string>()->default_value(""))
      ("engine", "Inference engine type: libtorch | tensorrt", cxxopts::value<std::string>()->default_value("libtorch"))
      ("precision", "Inference precision: fp32 | fp16 | int8", cxxopts::value<std::string>()->default_value("fp32"))
      ("hands_type", "hands_type type: box | dex3", cxxopts::value<std::string>()->default_value("box"))
      ("h,help", "Show help");

    auto result = options.parse(argc, argv);

    if (result.count("help") || !result.count("mode") || !result.count("config")) {
      FRC_INFO("\n" << options.help());
      return 0;
    }

    // 读取参数
    mode = result["mode"].as<std::string>();
    config_name = result["config"].as<std::string>();
    headless = result["headless"].as<bool>();
    net = result["net"].as<std::string>();
    inference_engine_type = result["engine"].as<std::string>();
    precision = result["precision"].as<std::string>();
    hands_type = result["hands_type"].as<std::string>();

    std::string device_str = result["device"].as<std::string>();
    std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);
    std::transform(device_str.begin(), device_str.end(), device_str.begin(), ::tolower);
    std::transform(inference_engine_type.begin(), inference_engine_type.end(), inference_engine_type.begin(), ::tolower);
    std::transform(precision.begin(), precision.end(), precision.begin(), ::tolower);
    std::transform(hands_type.begin(), hands_type.end(), hands_type.begin(), ::tolower);

    // 模式合法性检查
    if (mode != "sim2mujoco" && mode != "sim2real") {
      FRC_ERROR("Invalid mode: " << mode);
      FRC_ERROR("Available modes: sim2mujoco | sim2real");
      return -1;
    }

    // 配置合法性检查
    const std::vector<std::string> valid_configs = {"g1_eman"};
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

    // sim2real 限制：必须带 net
    if (mode == "sim2real" && net.empty()) {
      FRC_ERROR("Missing <net> argument for mode 'sim2real'.");
      FRC_ERROR("Usage: " << exec_name << " --mode sim2real --config g1_eman --net <interface>");
      return -1;
    }

    // sim2real 限制：只支持 g1_eman，不支持 other config
    if (mode == "sim2real" && config_name != "g1_eman") {
      FRC_ERROR("Only 'g1_eman' is supported in sim2real mode.");
      return -1;
    }

    // 设备检查
    if (device_str != "cpu" && device_str != "cuda") {
      FRC_ERROR("Invalid device: " << device_str);
      FRC_ERROR("Device must be 'cpu' or 'cuda'");
      return -1;
    }

    // engine type check
    const std::vector<std::string> valid_engines = {"libtorch", "tensorrt"};
    if (std::find(valid_engines.begin(), valid_engines.end(), inference_engine_type) == valid_engines.end()) {
      FRC_ERROR("Invalid inference engine type: " << inference_engine_type);
      FRC_ERROR("Available types: libtorch | tensorrt");
      return -1;
    }

    // precision check
    const std::vector<std::string> valid_precisions = {"fp32", "fp16", "int8"};
    if (std::find(valid_precisions.begin(), valid_precisions.end(), precision) == valid_precisions.end()) {
      FRC_ERROR("Invalid inference precision: " << precision);
      FRC_ERROR("Available precisions: fp32 | fp16 | int8");
      return -1;
    }

    // hands_type type check
    const std::vector<std::string> valid_hands = {"box", "dex3"};
    if (std::find(valid_hands.begin(), valid_hands.end(), hands_type) == valid_hands.end()) {
      FRC_ERROR("Invalid hands type: " << hands_type);
      FRC_ERROR("Available types: box | dex3");
      return -1;
    }

    // select device
    if (device_str == "cuda") {
      device = tools::getDefaultDevice();
      if (device.type() != torch::kCUDA) {
        FRC_WARN("CUDA requested, but not available. Falling back to CPU.");
        device_str = "cpu";
      }
    }
  } catch (const std::exception& e) {
    FRC_ERROR("[Argument Parsing Error] " << e.what());
    return 1;
  }

  signal(SIGINT, close_all_threads);

  try {
    config = tools::loadConfig(config_name);

    std::vector<std::pair<std::string, char>> registers = {
      {"CmdTask", '1'},
      // {"TeleopTask", '2'},
      // {"MocapTask", '3'}
    };
    
    controller = std::make_unique<G1Controller>(net, mode, registers, config, hands_type, device, 
                                                headless, inference_engine_type, precision);
                                                
    // Enter the zero torque state, press the start key to continue executing
    controller->zero_torque_state();
    // Move to the default position
    controller->move_to_default_pose();
    // Enter the default position state, press the A key to continue executing
    controller->default_pos_state();

    controller->run();

    close_all_threads(200);
  } catch (const std::exception& e) {
    FRC_ERROR("Failed to construct G1Controller: " << e.what());
    return -1;
  }
  return 0;
}
