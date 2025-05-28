#pragma once

#include <utility>
#include "types/CustomTypes.h"
#include "types/cpp_types.h"

/*  Abstract interface for all controllers */
class Controller
{
public:
  Controller(): kNumMotors(28) {initMembers();};
  explicit Controller(size_t dim): kNumMotors(dim) {initMembers();};
  virtual ~Controller() = default;;
  virtual void reset() = 0;
  virtual bool isComplete() const = 0;
  virtual void stop() {}
  virtual CustomTypes::Action getControlAction(const CustomTypes::State& robotState) = 0;
  size_t getCurrIdx() const {return _curr_idx;}
  void startLogging() {isLoggingFlag=true;}
  void stopLogging() {isLoggingFlag=false;}

  virtual void setBaseVelTarget(Vec3<float> lin, Vec3<float> ang) {
    baseLinVelTarget = std::move(lin);
    baseAngVelTarget = std::move(ang);
  }
  virtual void setPDGains(const Eigen::VectorXf& kp, const Eigen::VectorXf& kd) {
    _kP = kp;
    _kD = kd;
  }

  void initMembers() {
    _robotState = CustomTypes::zeroState(kNumMotors);
    _kP.setConstant(kNumMotors, 100);
    _kD.setConstant(kNumMotors, 2);
  }
  virtual void sendTime(double t){simulationTime = t;}

private:
    CustomTypes::State _robotState;
protected:
    bool isLoggingFlag = false;
    size_t _curr_idx{0};
    uint64_t simulationTime{0};
    Eigen::VectorXf _kP;
    Eigen::VectorXf _kD;
    Vec3<float> baseLinVelTarget, baseAngVelTarget;
    size_t kNumMotors = 28;
};


