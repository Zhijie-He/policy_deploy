//
// Created by willw on 24/3/23.
//

#ifndef RAISIM_SOCKET_PLAYGROUND_H
#define RAISIM_SOCKET_PLAYGROUND_H

#include <string>
#include "raisim/World.hpp"
#include "yaml-cpp/yaml.h"
#include "types/cpp_types.h"
#include "utility/orientation_tools.h"

using namespace std::chrono;

class GenericPlayground{
public:
    virtual ~GenericPlayground() = default;

    explicit GenericPlayground(raisim::World* world):
    _type("blank"),
    _localUnitName("unknown"),
    _world(world){
      randEng.seed(rd());
      RSINFO("Blank world, not implemented.")
    }
    virtual void environmentalCallback(raisim::ArticulatedSystem* robot){
      RSINFO("Blank world, nothing to do.")
    }
    virtual void environmentalReset(raisim::ArticulatedSystem* robot){}
    std::string getLocalTerrainType(){return _localUnitName;}

protected:
    std::string _type="default";
    std::string _localUnitName;
    raisim::World* _world;

    std::random_device rd{};
    std::default_random_engine randEng;

    std::uniform_real_distribution<double> randUni = std::uniform_real_distribution<double>(0, 1);
    std::normal_distribution<double> randNorm = std::normal_distribution<double>(0,1e-4);
};

class PlainPlayground: public GenericPlayground{
public:
    explicit PlainPlayground(raisim::World *world) : GenericPlayground(world){
      _type = "plain";
      _localUnitName = "flat";
      const raisim::Vec<3> gravity{0, 0, -9.8113};
      _world->setGravity(gravity);
      _ground = _world->addGround(0,"default");
    }
    void environmentalCallback(raisim::ArticulatedSystem* robot)override{}
    void environmentalReset(raisim::ArticulatedSystem* robot)override{}
private:
    raisim::Ground* _ground;
};

class RandomHeightMap: public GenericPlayground{
public:
    explicit RandomHeightMap(raisim::World *world, float fieldSizeX, float fieldSizeY, float terrainScale);
    void environmentalReset(raisim::ArticulatedSystem* robot) override ;
    void environmentalCallback(raisim::ArticulatedSystem* robot) override{}
private:
    raisim::HeightMap* _hm;
    std::random_device rd;
    Vec2<float> _fieldSize{10,10};
    Vec2<float> _fieldCenter{0,0};
};

class MultiSkillField: public GenericPlayground{
public:
    explicit MultiSkillField(raisim::World *world, const std::string& cfgPath);
    void environmentalCallback(raisim::ArticulatedSystem* robot) override;
    void environmentalReset(raisim::ArticulatedSystem* robot) override {};
private:

    void addPaddingPlatform(double length, double& start, const char* name="");
    void addPaddingBox(double length, double& start, const char* name);
    void loadLengthVec(const char* key, Vec3<double>& dest);

    YAML::Node _cfg;

    std::vector<double> _regionStartingVec;
    std::vector<double> _regionEndingVec;
    std::vector<std::string> _regionNames;
    double _fieldWidth=1;
    double _newUnitStartingPos = 0.;

    Vec3<double> _tracRegionLen{1, 1,1};

    Vec3<double> _wavyRegionLen{1, 1,1};
    raisim::TerrainProperties _wavyZoneProp;

    Vec3<double> _dcrtRegionLen{1, 1,1};
    double _dcrtBlockFreq=2;
    double _dcrtBlockLen=2;
    double _dcrtBlockWidth=2;
    double _dcrtBlockHeight=2;
    std::vector<raisim::Box*> _drctBlocks;

    Vec3<double> _slopRegionLen{1, 1,1};
    double _slopHeight{};
    double _slopUpLength{};
    double _slopDownLength{};

    Vec3<double> _gapyRegionLen{1,1,1};
    Vec2<double> _gapyInterv{};
    double _gapyGapLen{};

    Vec3<double> _fricRegionLen{1,1,1};
    Vec2<double> _fricCoeffRange{};
    double _fricResampleRate=1e-4;

    Vec3<double> _pyldRegionLen{1,1,1};
    Vec2<double> _pyldMassRange{};
    Vec3<double> _pyldPosRadius{};
    bool _pyldIsChanged=false;
    double _pyldResampleRate=1e-4;
    double _pyldOriginalMass=4.713;
    Vec3<double> _pyldOriginalCoM{1e-2,2e-3,5e-4};

    Vec3<double> _shotRegionLen{1,1,1};
    double _shotBulletMass{};
    double _shotBulletFreq{};
    double _shotBulletSize{};
    Vec3<double> _shotBulletVel{};
    double _shotPrecision{};
    double _shotDistance{};
    std::chrono::time_point<std::chrono::system_clock> _lastShotTime{};
    Vec3<double> _shootTarget{};
    Vec3<double> _shootBase{};


    Vec3<double> _pushRegionLen{1,1,1};
    Vec3<double> _pushFrcMax{};
    Vec3<double> _pushFrcMin{};
    Vec3<double> _pushFrcPosRange{0.1,0.01,0.01};
    Vec3<double> _pushFrcPos{};
    Vec3<double> _pushFrcCur{};
    bool _isApplyingFrc = false;
    double _pushEffectTime{};
    double _pushFrequency{};
    std::chrono::time_point<std::chrono::system_clock> _lastPushTime{};

    Vec3<double> _lameRegionLen{1,1,1};
    Vec3<double> _lameJntPos{0,1.4,-2.6};
    double _lameLegIdx=0;
};

#endif //RAISIM_SOCKET_PLAYGROUND_H
