#include "controller/ResetController.h"
#include "utility/logger.h"

ResetController::ResetController(Eigen::VectorXf finalMotorPosition, unsigned long max_timesteps, float dt):
        Controller(finalMotorPosition.rows()), // 初始化父类（设置关节数） 调用 Controller(dim)，其中 dim = finalMotorPosition.rows() 即“根据目标关节向量的大小，动态设置关节数”。
        _kMaxTimesteps(max_timesteps), 
        _kFinalMotorPosition(std::move(finalMotorPosition)),
        _dt(dt),
        _timestep(0),
        _isFirstVisit(true) {
  _trajectory.setDimension(kNumMotors); // 	初始化三次插值器维度
};

ResetController::~ResetController() = default;

void ResetController::reset() {
  _timestep = 0;
  _isFirstVisit = true;
}

void ResetController::reset(const Eigen::VectorXf &finalMotorPosition) {
  _kFinalMotorPosition = finalMotorPosition;
  reset();
}

void ResetController::reset(const Eigen::VectorXf &finalMotorPosition, unsigned long max_timesteps) {
  _kMaxTimesteps = max_timesteps;
  reset(finalMotorPosition);
}

bool ResetController::isComplete() const { return _timestep >= _kMaxTimesteps; }

void ResetController::_initializeTrajectory(const CustomTypes::State &robotState) {
  Eigen::VectorXf _initialMotorPosition = robotState.motorPosition;

  std::vector<float> initPos, finlPos, initVel, finlVel;
  initPos.resize(kNumMotors, 0.);
  finlPos.resize(kNumMotors, 0.);
  initVel.resize(kNumMotors, 0.);
  finlVel.resize(kNumMotors, 0.);

  FRC_INFO("Initializing Standup Traj: ");
  FRC_INFO("ini:" << robotState.motorPosition.transpose());
  FRC_INFO("fin:" << _kFinalMotorPosition.transpose());

  memcpy(initPos.data(), robotState.motorPosition.data(), kNumMotors*sizeof(float));
  memcpy(finlPos.data(), _kFinalMotorPosition.data(), kNumMotors*sizeof(float));
  memcpy(initVel.data(), robotState.motorVelocity.data(), kNumMotors*sizeof(float));

  _trajectory.SetParam(initPos.data(),
                       initVel.data(),
                       finlPos.data(),
                       finlVel.data(),
                       _kMaxTimesteps * _dt);

}

CustomTypes::Action ResetController::getControlAction(const CustomTypes::State &robotState) {
  if (_isFirstVisit) {
    DBG_INFO("[ResetCtrl] Initializing trajectory. with Pos: "<<robotState.motorPosition.transpose());
    _initializeTrajectory(robotState);
    _isFirstVisit = false;
  }

  Eigen::VectorXf _targetMotorPosition = Eigen::VectorXf::Zero(kNumMotors);
  Eigen::VectorXf _targetMotorVelocity = Eigen::VectorXf::Zero(kNumMotors);
  float currentTime = _timestep * _dt;
  if (_kMaxTimesteps <= _timestep) currentTime = _kMaxTimesteps * _dt;
  _trajectory.getCurvePoint(currentTime, _targetMotorPosition.data());

  _timestep += 1;
  // Ignore the robot state and just return the recorded position / velocity
  return CustomTypes::Action({_targetMotorPosition,
                              _targetMotorVelocity,
                              Eigen::VectorXf::Zero(kNumMotors),
                              Controller::_kP,
                              Controller::_kD,
                              robotState.timestamp
                             });
};
