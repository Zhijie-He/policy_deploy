#include <iostream>
#include <thread>
#include <mutex>
#include <csignal>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "state_machine/StateMachine.h"
#include "utility/MathUtilities.h"
#include "ros_node/RobotInterface.h"
#include "utility/timer.h"

#define LOG_USE_COLOR 1
#define LOG_USE_PREFIX 1
#include "utility/logger.h"

void close_all_threads(int signum);

class JointMonkey: public StateMachine{
public:
  explicit JointMonkey(const YAML::Node& cfg):
  StateMachine(cfg), loopTimer(-1), gains_coeff(1.0){
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("neural_policy_deploy");
    Eigen::MatrixXf RobotDef = readCSV(package_share_dir+cfg["homing_pos_path"].as<std::string>(), _jointNum, 6,1).cast<float>();
    const Eigen::VectorXf jointPosMax = RobotDef.col(4);
    const Eigen::VectorXf jointPosMin = RobotDef.col(3);
    jointMean = (jointPosMax + jointPosMin) / 2;
    jointScale = (jointPosMax - jointPosMin) / 2;
    jointPGain = RobotDef.col(1) * gains_coeff;
    jointDGain = RobotDef.col(2) * gains_coeff;
    jointSafePos =  RobotDef.col(5);

    FRC_INFO("[JMC.init] mean joint pos: " << jointMean.transpose());
  }

private:
 
  bool _onFinishing = false;
  int printFreq = 100;
  int printCounter = 0;
  Timer loopTimer;
  int jointIdx = 0;
  int timestep = 0;
  float amp = 0.5;
  float period = 5.0;
  float gains_coeff = 1.0;
  Eigen::VectorXf jointMean, jointScale, jointPGain, jointDGain, jointSafePos;
};

std::shared_ptr<JointMonkey> ctrl = nullptr;

void close_all_threads(int signum)
{
  FRC_INFO("SIGINT captured. closing all threads. " << signum);
  // if (ctrl != nullptr) ctrl->stop();
  rclcpp::shutdown();
}

int main(int argc, char *argv[]) {
  FRC_INFO("[Main] Joint Monkey Test v250617.");
  std::string package_share_dir = ament_index_cpp::get_package_share_directory("neural_policy_deploy");
  std::string yaml_file_path = package_share_dir + "/config/robot_config.yaml";
  YAML::Node cfg = YAML::LoadFile(yaml_file_path);
  FRC_INFO("[Main] Load config from : " << yaml_file_path);

  ctrl = std::make_shared<JointMonkey>(cfg);

  rclcpp::init(argc, argv);
  signal(SIGINT, close_all_threads);
  rclcpp::spin(std::make_shared<RobotInterface>("interface", ctrl.get()));

  return 0;
}

