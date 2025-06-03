#include "controller/ResetController.h"
#include "utility/logger.h"

ResetController::ResetController(std::shared_ptr<const RobotConfig> cfg):
        _jointNum(cfg->num_actions),
        _kP(cfg->homingKp),
        _kD(cfg->homingKd),
        _kMaxTimesteps(cfg->homing_timesteps), 
        _kFinalMotorPosition(std::move(cfg->homingPos)),
        _dt(cfg->getPolicyDt()),
        _timestep(0),
        _isFirstVisit(true) {
          _robotState = CustomTypes::zeroState(_jointNum);
          _trajectory.setDimension(_jointNum); // 设置插值器维度为电机数量
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
  assert(_kFinalMotorPosition.size() >= _jointNum);
  
  std::vector<float> initPos, finlPos, initVel, finlVel;
  initPos.resize(_jointNum, 0.);
  finlPos.resize(_jointNum, 0.);
  initVel.resize(_jointNum, 0.);
  finlVel.resize(_jointNum, 0.); // 通常设为 0

  FRC_INFO("[ResetController] Initializing Standup Traj: ");
  FRC_INFO("[ResetController] initial pos: " << robotState.motorPosition.transpose());
  FRC_INFO("[ResetController] final pos: " << _kFinalMotorPosition.transpose());
  
  memcpy(initPos.data(), robotState.motorPosition.data(), _jointNum*sizeof(float));
  memcpy(finlPos.data(), _kFinalMotorPosition.data(), _jointNum*sizeof(float));
  memcpy(initVel.data(), robotState.motorVelocity.data(), _jointNum*sizeof(float));

  // 构造一个“平滑站立”轨迹
  _trajectory.SetParam(initPos.data(),   // 初始位置
                     initVel.data(),   // 初始速度
                     finlPos.data(),   // 目标位置
                     finlVel.data(),   // 目标速度（为 0）
                     _kMaxTimesteps * _dt); // 总时长（秒）
}

CustomTypes::Action ResetController::getControlAction(const CustomTypes::State &robotState) {
  if (_isFirstVisit) {
    DBG_INFO("[ResetCtrl] Initializing trajectory. with Pos: "<<robotState.motorPosition.transpose());
    _initializeTrajectory(robotState);
    _isFirstVisit = false;
  }

  Eigen::VectorXf _targetMotorPosition = Eigen::VectorXf::Zero(_jointNum);
  Eigen::VectorXf _targetMotorVelocity = Eigen::VectorXf::Zero(_jointNum);

  float currentTime = _timestep * _dt;
  if (_kMaxTimesteps <= _timestep) currentTime = _kMaxTimesteps * _dt;
  _trajectory.getCurvePoint(currentTime, _targetMotorPosition.data());

  _timestep += 1;
  // Ignore the robot state and just return the recorded position / velocity
  return CustomTypes::Action({_targetMotorPosition,
                              _targetMotorVelocity,
                              Eigen::VectorXf::Zero(_jointNum),
                              _kP,
                              _kD,
                              robotState.timestamp
                             });
};
