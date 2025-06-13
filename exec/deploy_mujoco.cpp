#include <filesystem>
#include <csignal>
#include "controller/NeuralController.h"
#include "state_machine/StateMachine.h"
#include "hardware/listener.h"
#include "sim2/simulator/g1_sim2mujoco_env.h"
#include "utility/tools.h"

#define LOG_USE_COLOR 1
#define LOG_USE_PREFIX 1
#include "utility/logger.h"

std::shared_ptr<BaseRobotConfig> cfg = nullptr;
std::shared_ptr<Listener> listener = nullptr;
std::shared_ptr<StateMachine> state_machine = nullptr;
std::shared_ptr<G1Sim2MujocoEnv> env = nullptr; 

void close_all_threads(int signum) {
  FRC_INFO("Interrupted with SIGINT: " << signum << "\n");
  if (env != nullptr) env->stop();
  if (listener != nullptr) listener->stop();
  if (state_machine != nullptr) state_machine->stop();
  std::exit(0);
}


int main(int argc, char** argv) {
  std::string exec_name = std::filesystem::path(argv[0]).filename().string();
  if (argc < 2) {
    FRC_ERROR("Usage error: Please provide a config name, e.g., ./" << exec_name << " g1_unitree");
    return -1;
  }

  signal(SIGINT, close_all_threads);

  std::string config_name = argv[1]; 
  cfg = tools::loadConfig(config_name);
  listener = std::make_shared<Listener>();
  state_machine = std::make_shared<StateMachine>(cfg, config_name);
  env = std::make_shared<G1Sim2MujocoEnv>(cfg,
                                          state_machine->getJointCMDBufferPtr(),
                                          state_machine->getRobotStatusBufferPtr());
 
  env->setUserInputPtr(listener, listener->getKeyInputPtr(), nullptr);
  state_machine->setInputPtr(listener->getKeyInputPtr(), nullptr);
    
  std::thread keyboard_thread(&Listener::listenKeyboard, listener); // 监听键盘
  std::thread ctrl_thread(&StateMachine::run, state_machine);

  // Move to the default position
  env->moveToDefaultPos();

  std::thread comm_thread(&G1Sim2MujocoEnv::run, env);
  std::thread integrate_thread(&G1Sim2MujocoEnv::integrate, env);

  env->renderLoop();

  state_machine->stop();
  listener->stop();
  ctrl_thread.join();
  keyboard_thread.join();
  comm_thread.join();
  integrate_thread.join();

  return 0;
}
