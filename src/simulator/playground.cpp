//
// Created by willw on 24/3/23.
//

#include <string>
#include "raisim/World.hpp"
#include "yaml-cpp/yaml.h"
#include "types/cpp_types.h"
#include "simulator/playground.hpp"

using namespace std::chrono;

RandomHeightMap::RandomHeightMap(raisim::World* world, float fieldSizeX, float fieldSizeY,
                                 float terrainScale) : GenericPlayground(world)
{
  _type = "heightmap";
  _localUnitName = "heightmap";
  raisim::Vec<3> gravity{0, 0, -9.8113};
  _world->setGravity(gravity);

  _fieldSize << fieldSizeX, fieldSizeY;
  raisim::TerrainProperties terrainProperties;
  terrainProperties.frequency = 1.2;
  terrainProperties.zScale = terrainScale;
  terrainProperties.xSize = _fieldSize[0];
  terrainProperties.ySize = _fieldSize[1];
  terrainProperties.xSamples = size_t(_fieldSize[0]) * 10;
  terrainProperties.ySamples = size_t(_fieldSize[1]) * 10;
  terrainProperties.fractalOctaves = 2;
  terrainProperties.fractalLacunarity = 2.0;
  terrainProperties.fractalGain = 10.25;
  terrainProperties.seed = rd();
  _hm = _world->addHeightMap(_fieldCenter[0], _fieldCenter[1], terrainProperties, "ground");
  _hm->setName(_localUnitName);
  _hm->setAppearance("grey");
}

void RandomHeightMap::environmentalReset(raisim::ArticulatedSystem* robot)
{
  std::string ankleNames[4] = {"FR_foot_fixed", "FL_foot_fixed", "RR_foot_fixed", "RL_foot_fixed"};
  Eigen::VectorXd gc = robot->getGeneralizedCoordinate().e();
  Vec3<double> basePos{0, 0, 0};
  if (robot->getJointType(0) == raisim::Joint::Type::FLOATING) basePos = robot->getBasePosition().e();
  Mat3<double> baseRotMat = robot->getBaseOrientation().e();
  raisim::Vec<3> footPos_w;
  Vec4<float> groundHeight = Vec4<float>::Zero();

  Vec3<double> newPos{0, 0, 0};
  newPos[0] = (randUni(randEng) - 0.5) * _fieldSize[0] + _fieldCenter[0];
  newPos[1] = (randUni(randEng) - 0.5) * _fieldSize[1] + _fieldCenter[1];
  Vec3<double> newRPY{0, 0, std::atan2(_fieldCenter[1] - newPos[1], _fieldCenter[0] - newPos[0])};
  Mat3<double> newRotMat = ori::rpyToRotMat(newRPY).transpose().cast<double>();

  for (int i = 0; i < 4; i++)
  {
    robot->getFramePosition(robot->getFrameIdxByName(ankleNames[i]), footPos_w);
    Vec3<double> footPos_b = baseRotMat.transpose() * (footPos_w.e() - basePos);
    Vec3<double> newFootPos_w = newRotMat * footPos_b + newPos;
    groundHeight[i] = _hm->getHeight(newFootPos_w[0], newFootPos_w[1]);
  }
  newPos[2] = basePos[2] - footPos_w[2] + groundHeight.maxCoeff();
  robot->setBasePos_e(newPos);
  robot->setBaseOrientation_e(newRotMat);
}


MultiSkillField::MultiSkillField(raisim::World* world, const std::string& cfgPath) :
  GenericPlayground(world),
  _cfg(YAML::LoadFile(cfgPath))
{
  randEng.seed(time(nullptr));
  _world = world;

  Vec3<double> gravity = Eigen::Map<Vec3<double>>(_cfg["gravity"].as<std::vector<double>>().data());
  _world->setGravity(gravity);
  _fieldWidth = _cfg["width"].as<double>();
  _newUnitStartingPos = -2;
  addPaddingPlatform(_cfg["starting_pad_length"].as<double>() - _newUnitStartingPos, _newUnitStartingPos, "Starting");

  auto backWall = _world->addBox(0.01, _fieldWidth, 15, 1e-4,
                                 "default", raisim::RAISIM_STATIC_COLLISION_GROUP, raisim::CollisionGroup(-1));
  backWall->setPosition(-5, 0, 5);
  backWall->setName("BackWall");
  backWall->setBodyType(raisim::BodyType::STATIC);
  backWall->setAppearance("1,1,1,1");

  loadLengthVec("straight_track", _tracRegionLen);
  _regionStartingVec.push_back(_newUnitStartingPos);
  addPaddingPlatform(_tracRegionLen.sum(), _newUnitStartingPos, "RaceTrack");
  _regionEndingVec.push_back(_newUnitStartingPos);

  loadLengthVec("wavy_heightmap", _wavyRegionLen);
  _wavyZoneProp = raisim::TerrainProperties();
  _wavyZoneProp.frequency = _cfg["wavy_heightmap"]["frequency"].as<double>();
  _wavyZoneProp.fractalOctaves = _cfg["wavy_heightmap"]["fractal_octaves"].as<double>();
  _wavyZoneProp.fractalGain = _cfg["wavy_heightmap"]["fractal_gain"].as<double>();
  _wavyZoneProp.fractalLacunarity = _cfg["wavy_heightmap"]["fractal_lacunarity"].as<double>();
  _wavyZoneProp.zScale = _cfg["wavy_heightmap"]["z_scale"].as<double>();
  _wavyZoneProp.seed = rd();
  _wavyZoneProp.xSize = _wavyRegionLen[1];
  _wavyZoneProp.ySize = _fieldWidth;
  _wavyZoneProp.xSamples = (size_t)(_wavyRegionLen[1] * 20);
  _wavyZoneProp.ySamples = (size_t)(_fieldWidth * 20);
  _wavyZoneProp.heightOffset = -_wavyZoneProp.zScale / 2;
  _wavyZoneProp.stepSize = 0;
  addPaddingPlatform(_wavyRegionLen[0], _newUnitStartingPos, "WavyStart");
  _regionStartingVec.push_back(_newUnitStartingPos);
  auto wavyHm = _world->addHeightMap(_newUnitStartingPos + _wavyRegionLen[1] / 2, 0, _wavyZoneProp);
  wavyHm->setName("WavyZone");
  _newUnitStartingPos += _wavyRegionLen[1];
  _regionEndingVec.push_back(_newUnitStartingPos);
  addPaddingPlatform(_wavyRegionLen[2], _newUnitStartingPos, "WavyEnd");

  loadLengthVec("discrete_terrain", _dcrtRegionLen);
  addPaddingPlatform(_dcrtRegionLen[0], _newUnitStartingPos, "DcrtStart");
  _regionStartingVec.push_back(_newUnitStartingPos);

  _dcrtBlockFreq = _cfg["discrete_terrain"]["platform_frequency"].as<double>();
  _dcrtBlockLen = _cfg["discrete_terrain"]["platform_length"].as<double>();
  _dcrtBlockWidth = _cfg["discrete_terrain"]["platform_width"].as<double>();
  _dcrtBlockHeight = _cfg["discrete_terrain"]["z_scale"].as<double>();
  /// Add blocks
  int blockNum = (int)(_dcrtBlockFreq * (_dcrtRegionLen[1] - _dcrtBlockLen) * _fieldWidth);
  for (int i = 0; i < blockNum; i++)
  {
    _drctBlocks.push_back(_world->addBox(_dcrtBlockLen, _dcrtBlockWidth, _dcrtBlockHeight, 1e-4,
                                         "default", raisim::RAISIM_STATIC_COLLISION_GROUP, raisim::CollisionGroup(-1)));
    _drctBlocks.back()->setName(std::string("Block_") + std::to_string(i));
    _drctBlocks.back()->setBodyType(raisim::BodyType::STATIC);
    _drctBlocks.back()->setAppearance("cyan");
    double x, y, z;
    x = _newUnitStartingPos + _dcrtBlockLen / 2 + randUni(randEng) * (_dcrtRegionLen[1] - _dcrtBlockLen);
    y = (randUni(randEng) * 2 - 1) * _fieldWidth / 2;
    z = -_dcrtBlockHeight / 2 + (randUni(randEng) * _dcrtBlockHeight);
    _drctBlocks.back()->setPosition(x, y, z);
  }
  _regionEndingVec.push_back(_newUnitStartingPos + _dcrtRegionLen[1]);
  addPaddingPlatform(_dcrtRegionLen[1] + _dcrtRegionLen[2], _newUnitStartingPos, "DcrtEnd");

  loadLengthVec("slope", _slopRegionLen);
  _slopHeight = _cfg["slope"]["platform_height"].as<double>();
  _slopDownLength = _cfg["slope"]["downward_length"].as<double>();
  _slopUpLength = _cfg["slope"]["upward_length"].as<double>();
  double _slopPeakLength = _slopRegionLen[1] - _slopDownLength - _slopUpLength;
  addPaddingPlatform(_slopRegionLen[0], _newUnitStartingPos, "SlopStart");
  _regionStartingVec.push_back(_newUnitStartingPos);
  std::vector<double> vecUpSlop{0, _slopHeight, 0, _slopHeight};
  std::vector<double> vecDownSlop{_slopHeight, 0, _slopHeight, 0};
  std::vector<double> vecPeakSlop{_slopHeight, _slopHeight, _slopHeight, _slopHeight};
  auto upSlope = _world->addHeightMap(2, 2,
                                      _slopUpLength, _fieldWidth,
                                      _newUnitStartingPos + _slopUpLength / 2, 0,
                                      vecUpSlop);
  _newUnitStartingPos += _slopUpLength;

  auto peakSlope = _world->addHeightMap(2, 2,
                                        _slopPeakLength, _fieldWidth,
                                        _newUnitStartingPos + _slopPeakLength / 2, 0,
                                        vecPeakSlop);
  _newUnitStartingPos += _slopPeakLength;
  auto downSlope = _world->addHeightMap(2, 2,
                                        _slopDownLength, _fieldWidth,
                                        _newUnitStartingPos + _slopDownLength / 2, 0,
                                        vecDownSlop);
  _newUnitStartingPos += _slopDownLength;
  _regionEndingVec.push_back(_newUnitStartingPos);
  addPaddingPlatform(_slopRegionLen[2], _newUnitStartingPos, "SlopEnd");

  loadLengthVec("gap_terrain", _gapyRegionLen);
  addPaddingPlatform(_gapyRegionLen[0], _newUnitStartingPos, "Terminal-1");
  _regionStartingVec.push_back(_newUnitStartingPos);
  double gapEndPos = _gapyRegionLen[1] + _newUnitStartingPos;

  _gapyGapLen = _cfg["gap_terrain"]["gap_length"].as<double>();
  _gapyInterv = Eigen::Map<Vec2<double>>(_cfg["gap_terrain"]["gap_interval_range"].as<std::vector<double>>().data());
  for (int i = 0; _newUnitStartingPos < gapEndPos; i++)
  {
    /// Zone terrain composition: StartPad-(Gap-Tile)-(Gap-Tile)-...-(Gap-Tile)-EndingPad
    double nextGapLen = _gapyGapLen * randUni(randEng);
    double nextTileLen = _gapyInterv[0] + (_gapyInterv[1] - _gapyInterv[0]) * randUni(randEng);
    if (_newUnitStartingPos + nextGapLen < gapEndPos) _newUnitStartingPos += nextGapLen;
    else break;
    if (_newUnitStartingPos + nextTileLen < gapEndPos)
    {
      //      addPaddingPlatform(nextTileLen, _newUnitStartingPos, "Tile");
      addPaddingBox(nextTileLen, _newUnitStartingPos, "Tile");
    }
    else
    {
      //      addPaddingPlatform(gapEndPos-_newUnitStartingPos,_newUnitStartingPos,"Tile");
      addPaddingBox(gapEndPos - _newUnitStartingPos, _newUnitStartingPos, "Tile");
      break;
    }
  }
  _newUnitStartingPos = gapEndPos;
  _regionEndingVec.push_back(_newUnitStartingPos);
  addPaddingPlatform(_gapyRegionLen[2], _newUnitStartingPos, "Terminal-3");

  _world->setMaterialPairProp("foot", "varied_ground", 1.0, 0.0, 1e-3);

  _fricCoeffRange[0] = _cfg["friction_variation"]["min_friction"].as<double>();
  _fricCoeffRange[1] = _cfg["friction_variation"]["max_friction"].as<double>();
  _fricResampleRate = _cfg["friction_variation"]["resample_rate"].as<double>();

  loadLengthVec("friction_variation", _fricRegionLen);
  addPaddingPlatform(_fricRegionLen[0], _newUnitStartingPos, "FricStart");
  _regionStartingVec.push_back(_newUnitStartingPos);
  std::vector<double> vecFlatHeight{0, 0, 0, 0};
  auto frictionTile = _world->addHeightMap(2, 2,
                                           _fricRegionLen[1], _fieldWidth,
                                           _newUnitStartingPos + _fricRegionLen[1] / 2, 0,
                                           vecFlatHeight, "varied_ground");
  frictionTile->setName("FricZone");
  _newUnitStartingPos += _fricRegionLen[1];
  _regionEndingVec.push_back(_newUnitStartingPos);
  addPaddingPlatform(_fricRegionLen[2], _newUnitStartingPos, "FricEnd");

  loadLengthVec("payload_test_zone", _pyldRegionLen);
  _pyldMassRange[0] = _cfg["payload_test_zone"]["min_payload"].as<double>();
  _pyldMassRange[1] = _cfg["payload_test_zone"]["max_payload"].as<double>();
  _pyldResampleRate = _cfg["payload_test_zone"]["resample_rate"].as<double>();
  _pyldPosRadius = Eigen::Map<Vec3<double>>(
    _cfg["payload_test_zone"]["loading_point_range"].as<std::vector<double>>().data());
  addPaddingPlatform(_pyldRegionLen[0], _newUnitStartingPos, "PayloadStart");
  _regionStartingVec.push_back(_newUnitStartingPos);
  addPaddingPlatform(_pyldRegionLen[1], _newUnitStartingPos, "PayloadZone");
  _regionEndingVec.push_back(_newUnitStartingPos);
  addPaddingPlatform(_pyldRegionLen[2], _newUnitStartingPos, "PayloadEnd");

  loadLengthVec("shooting_range", _shotRegionLen);
  _shotBulletFreq = _cfg["shooting_range"]["fire_frequency"].as<double>();
  _shotBulletMass = _cfg["shooting_range"]["bullet_mass"].as<double>();
  _shotBulletSize = _cfg["shooting_range"]["bullet_size"].as<double>();
  _shotBulletVel << _cfg["shooting_range"]["bullet_init_vel"].as<double>(), 0, 0;
  _shotPrecision = _cfg["shooting_range"]["shooting_precision"].as<double>();
  _shotDistance = _cfg["shooting_range"]["fire_distance"].as<double>();
  addPaddingPlatform(_shotRegionLen[0], _newUnitStartingPos, "ShootingStart");
  _regionStartingVec.push_back(_newUnitStartingPos);
  addPaddingPlatform(_shotRegionLen[1], _newUnitStartingPos, "ShootingZone");
  _regionEndingVec.push_back(_newUnitStartingPos);
  addPaddingPlatform(_shotRegionLen[2], _newUnitStartingPos, "ShootingEnd");


  loadLengthVec("pushing_zone", _pushRegionLen);

  _pushFrequency = _cfg["pushing_zone"]["force_frequency"].as<double>();
  _pushEffectTime = _cfg["pushing_zone"]["effect_time"].as<double>();
  _pushFrcMax = Eigen::Map<Vec3<double>>(_cfg["pushing_zone"]["force_magnitude"].as<std::vector<double>>().data());
  _pushFrcMin = Eigen::Map<Vec3<double>>(_cfg["pushing_zone"]["force_minimal"].as<std::vector<double>>().data());
  _pushFrcPosRange = Eigen::Map<Vec3<double>>(
    _cfg["pushing_zone"]["apply_point_range"].as<std::vector<double>>().data());

  addPaddingPlatform(_pushRegionLen[0], _newUnitStartingPos, "PushingStart");
  _regionStartingVec.push_back(_newUnitStartingPos);
  addPaddingPlatform(_pushRegionLen[1], _newUnitStartingPos, "PushingZone");
  _regionEndingVec.push_back(_newUnitStartingPos);
  addPaddingPlatform(_pushRegionLen[2], _newUnitStartingPos, "PushingEnd");


  loadLengthVec("malfunction", _lameRegionLen);
  _lameLegIdx = _cfg["malfunction"]["disable_leg_index"].as<double>();
  _lameJntPos = Eigen::Map<Vec3<double>>(_cfg["malfunction"]["keep_position"].as<std::vector<double>>().data());
  addPaddingPlatform(_lameRegionLen[0], _newUnitStartingPos, "LameStart");
  _regionStartingVec.push_back(_newUnitStartingPos);
  addPaddingPlatform(_lameRegionLen[1], _newUnitStartingPos, "LameZone");
  _regionEndingVec.push_back(_newUnitStartingPos);
  addPaddingPlatform(_lameRegionLen[2], _newUnitStartingPos, "LameEnd");


  addPaddingPlatform(1, _newUnitStartingPos, "Terminal-1");

  auto termPoint = _world->addSphere(0.1, 1e-4);
  termPoint->setPosition(_newUnitStartingPos, 0, 4);
  termPoint->setAppearance("1,1,1,1");
  termPoint->setBodyType(raisim::BodyType::STATIC);
  termPoint->setName("TerminalSphere");


  for (int i = 0; i < _regionStartingVec.size(); i++)
  {
    std::cout << "[ChallTerr] region " << _regionNames[i] << ": [" << _regionStartingVec[i] << "," << _regionEndingVec[
      i] << "]\n";
  }
};

void MultiSkillField::environmentalCallback(raisim::ArticulatedSystem* robot)
{
  /// Recheck which region the robot is in;
  Vec3<double> robotPos{0, 0, 0};
  if (robot->getJointType(0) == raisim::Joint::Type::FLOATING)
  {
    robotPos = robot->getBasePosition().e();
  }
  _localUnitName = "plain";
  for (int i = 0; i < _regionStartingVec.size(); i++)
  {
    if (robotPos[0] > _regionStartingVec[i] && robotPos[0] < _regionEndingVec[i])
    {
      _localUnitName = _regionNames[i];
      break;
    }
  }

  /// If friction variation zone, change friction coefficient;
  if (_localUnitName == "friction_variation" && randUni(randEng) < _fricResampleRate)
  {
    double newFric = _fricCoeffRange[0] + (_fricCoeffRange[1] - _fricCoeffRange[0]) * randUni(randEng);
    _world->setMaterialPairProp("foot", "varied_ground", newFric, 0.0, 1e-3);

    std::cout << "[EnvCalbk] Mu changed to: " << newFric << ".\n";
  }

  /// If payload zone, apply payload;
  if (_localUnitName == "payload_test_zone" && randUni(randEng) < _pyldResampleRate)
  {
    raisim::Vec<3> desired_com;
    for (int i = 0; i < 3; i++)
    {
      double offset = randUni(randEng) * 2 - 1;
      desired_com[i] = _pyldOriginalCoM[i] + offset * _pyldPosRadius[i];
    }

    // Set the body COM to randomized value
    int trunkIdx = robot->getBodyIdx("TORSO");
    double offsetMass = randUni(randEng) * (_pyldMassRange[1] - _pyldMassRange[0]) + _pyldMassRange[0];
    std::vector<raisim::Vec<3>>& all_coms = robot->getBodyCOM_B();
    all_coms[0] = desired_com;
    robot->setMass(trunkIdx, _pyldOriginalMass + offsetMass);
    robot->updateMassInfo();
    _pyldIsChanged = true;

    std::cout << "[EnvCalbk] Mass increased: " << offsetMass << "kg.\n";
    std::cout << "[EnvCalbk] CoM Shifted: " << desired_com.e().transpose() << "m.\n";
  }
  else if (_pyldIsChanged && _localUnitName != "payload_test_zone")
  {
    int trunkIdx = robot->getBodyIdx("TORSO");
    raisim::Vec<3> desired_com{_pyldOriginalCoM};
    std::vector<raisim::Vec<3>>& all_coms = robot->getBodyCOM_B();
    all_coms[0] = desired_com;
    robot->setMass(trunkIdx, _pyldOriginalMass);
    robot->updateMassInfo();
    _pyldIsChanged = false;
    std::cout << "[EnvCalbk] CoM & Mass are now back to normal. \n";
  }

  /// If shooting range, start firing;
  if (_localUnitName == "shooting_range" && robot->getJointType(0) == raisim::Joint::Type::FLOATING)
  {
    auto timeInterv = 1e-6 * duration_cast<microseconds>((system_clock::now() - _lastShotTime)).count();
    if (timeInterv > 1 / _shotBulletFreq)
    {
      _lastShotTime = system_clock::now();
      _shootTarget = robot->getBasePosition().e();
      for (int i = 0; i < 3; i++) _shootTarget[i] += (randUni(randEng) * 2 - 1) * _shotPrecision;
      double pitch = randUni(randEng) * M_PI;
      double yaw = randUni(randEng) * 2 * M_PI;
      _shootBase << cos(pitch) * cos(yaw),
        cos(pitch) * sin(yaw),
        sin(yaw);
      Vec3<double> bulletInitVel = -_shootBase * _shotBulletVel[0];
      _shootBase = _shootBase * _shotDistance + _shootTarget;
      auto bullet = _world->addSphere(_shotBulletSize, _shotBulletMass);
      bullet->setPosition(_shootBase);
      bullet->setLinearVelocity(bulletInitVel);
    }
  }

  /// If pushing zone, apply external force;
  if (_localUnitName == "pushing_zone")
  {
    auto timeInterv = 1e-6 * duration_cast<microseconds>((system_clock::now() - _lastPushTime)).count();
    if (timeInterv > 1 / _pushFrequency)
    {
      _lastPushTime = system_clock::now();
      timeInterv = 0;
      for (int i = 0; i < 3; i++)
      {
        _pushFrcCur[i] = randUni(randEng) * 2 - 1;
        _pushFrcPos[i] = randUni(randEng) * 2 - 1;
      }
      _pushFrcCur = _pushFrcCur.cwiseProduct(_pushFrcMax - _pushFrcMin) + _pushFrcMin;
      _pushFrcPos = _pushFrcPos.cwiseProduct(_pushFrcPosRange);
      std::cout << "[EnvClbk] Applying external force: " << _pushFrcCur.transpose() << "at Time: " << _world->
        getWorldTime() << std::endl;
    }
    if (timeInterv < _pushEffectTime)
    {
      raisim::Vec<3> pos{_pushFrcPos}, frc{_pushFrcCur};
      robot->setExternalForce(robot->getBodyIdx("TORSO"), pos, frc);
    }
    else if (_pushFrcCur.any() != 0)
    {
      _pushFrcCur.setZero();
      _pushFrcPos.setZero();
      robot->clearExternalForcesAndTorques();
      std::cout << "[EnvClbk] Force cleared at Time: " << _world->getWorldTime() << std::endl;
    }
  }
  else if (_pushFrcCur.any() != 0)
  {
    _pushFrcCur.setZero();
    _pushFrcPos.setZero();
    robot->clearExternalForcesAndTorques();
    std::cout << "[EnvClbk] Out of region, Force cleared at Time: " << _world->getWorldTime() << std::endl;
  }
  /// If amputation zone, disable one leg (Or do it outside)
}


void MultiSkillField::addPaddingPlatform(double length, double& start, const char* name)
{
  double centerX = start + length / 2;
  std::vector<double> vecFlatHeight{0, 0, 0, 0};
  double colorBase = 0.5;
  auto platform = _world->addHeightMap(2, 2, length, _fieldWidth, centerX, 0, vecFlatHeight);
  platform->setName(name);
  std::string randomColor = "";
  for (int i = 0; i < 3; i++)
    randomColor += std::to_string(colorBase + (1 - colorBase) * randUni(randEng)) + ",";
  randomColor += "1";
  platform->setAppearance(randomColor);
  start += length;
}

void MultiSkillField::addPaddingBox(double length, double& start, const char* name)
{
  double centerX = start + length / 2;
  std::vector<double> vecFlatHeight{0, 0, 0, 0};
  double colorBase = 0.5;
  auto platform = _world->addBox(length, _fieldWidth, 1, 1e-8);
  platform->setPosition(length / 2 + start, 0, -0.5);
  platform->setBodyType(raisim::BodyType::STATIC);
  platform->setName(name);
  std::string randomColor = "";
  for (int i = 0; i < 3; i++)
    randomColor += std::to_string(colorBase + (1 - colorBase) * randUni(randEng)) + ",";
  randomColor += "1";
  platform->setAppearance(randomColor);
  start += length;
};

void MultiSkillField::loadLengthVec(const char* key, Vec3<double>& dest)
{
  dest[0] = _cfg[key]["starting_padding"].as<double>();
  dest[1] = _cfg[key]["length"].as<double>();
  dest[2] = _cfg[key]["ending_padding"].as<double>();
  std::string zoneName{key};
  _regionNames.push_back(zoneName);
};
