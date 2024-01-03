// Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
#pragma once
#include <vector>
#include <functional>
#include <future>
#include <mutex>
#include <pybind11/pybind11.h>
#include <Eigen/Core>
#include <Eigen/CXX11/Tensor>
#include "MathUtility.h"
#include "Controller.h"
#include "PhysicalAsset.h"
#include "Track.h"
namespace py=pybind11;
namespace nl=nlohmann;
class Fighter;
class MissileSensor;

DECLARE_CLASS_WITHOUT_TRAMPOLINE(PropNav,Controller)
    public:
    double gain;
    //constructors & destructor
    //PropNav();
    PropNav(const nl::json& modelConfig_,const nl::json& instanceConfig_);
    void control() override;
    std::pair<Eigen::Vector3d,Eigen::Vector3d> calc(const Eigen::Vector3d &rs,const Eigen::Vector3d &vs,const Eigen::Vector3d &rt,const Eigen::Vector3d &vt);
};

DECLARE_CLASS_WITH_TRAMPOLINE(Missile,PhysicalAsset)
    friend void makeRangeTableSub(std::mutex& m,std::promise<Eigen::VectorXd> p,const nl::json& modelConfig_,const nl::json& instanceConfig_,const Eigen::MatrixXd& args);
    public:
    enum class Mode{
        GUIDED,
        SELF,
        MEMORY
    };
    //configで指定するもの
    //基底クラスでは何も使わない

    //位置、姿勢等の運動状態に関する追加変数
    Eigen::Vector3d accel;
    double accelScalar;
    //その他の内部変数
    std::vector<Eigen::VectorXd> rangeTablePoints;
    Eigen::Tensor<double,6> rangeTable;
    Track3D target;
    double targetUpdatedTime;
    bool hasLaunched;
    Mode mode;
    double launchedT;
    Eigen::Vector3d estTPos,estTVel;
    //子要素
    std::weak_ptr<MissileSensor> sensor;
    public:
    //constructors & destructor
    Missile(const nl::json& modelConfig_,const nl::json& instanceConfig_);
    virtual ~Missile();
    //functions
    virtual void makeChildren();
    virtual void validate();
    virtual void setDependency();
    virtual void perceive(bool inReset);
    virtual void control();
    virtual void behave();
    virtual void kill();
    virtual void calcMotion(double tAftLaunch,double dt)=0;
    virtual bool hitCheck(const Eigen::Vector3d &tpos,const Eigen::Vector3d &tpos_prev)=0;
    virtual bool checkDeactivateCondition(double tAftLaunch)=0;
    virtual void calcQ();
    virtual double getRmax(const Eigen::Vector3d &rs,const Eigen::Vector3d &vs,const Eigen::Vector3d &rt,const Eigen::Vector3d &vt);
    virtual double getRmax(const Eigen::Vector3d &rs,const Eigen::Vector3d &vs,const Eigen::Vector3d &rt,const Eigen::Vector3d &vt,const double& aa);
    virtual double calcRange(double vs,double hs,double vt,double ht,double obs,double aa);
    virtual double calcRangeSub(double vs,double hs,double vt,double ht,double obs,double aa,double r);
    virtual void makeRangeTable(const std::string& dstPath);
};

DECLARE_TRAMPOLINE(Missile)
    virtual void calcMotion(double tAftLaunch,double dt) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE_PURE(void,Base,calcMotion,tAftLaunch,dt);
    }
    virtual bool hitCheck(const Eigen::Vector3d &tpos,const Eigen::Vector3d &tpos_prev) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE_PURE(bool,Base,hitCheck,tpos,tpos_prev);
    }
    virtual bool checkDeactivateCondition(double tAftLaunch) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE_PURE(bool,Base,checkDeactivateCondition,tAftLaunch);
    }
    virtual void calcQ() override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(void,Base,calcQ);
    }
    virtual double getRmax(const Eigen::Vector3d &rs,const Eigen::Vector3d &vs,const Eigen::Vector3d &rt,const Eigen::Vector3d &vt){
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(double,Base,getRmax,rs,vs,rt,vt);
    }
    virtual double getRmax(const Eigen::Vector3d &rs,const Eigen::Vector3d &vs,const Eigen::Vector3d &rt,const Eigen::Vector3d &vt,const double& aa){
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(double,Base,getRmax,rs,vs,rt,vt,aa);
    }
    virtual double calcRange(double vs,double hs,double vt,double ht,double obs,double aa) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(double,Base,calcRange,vs,hs,vt,ht,obs,aa);
    }
    virtual double calcRangeSub(double vs,double hs,double vt,double ht,double obs,double aa,double r) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(double,Base,calcRangeSub,vs,hs,vt,ht,obs,aa,r);
    }
    virtual void makeRangeTable(const std::string& dstPath){
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(void,Base,makeRangeTable,dstPath);
    }
};
void makeRangeTableSub(std::mutex& m,std::promise<Eigen::VectorXd> p,std::shared_ptr<Missile> msl,const Eigen::MatrixXd& args);

void exportMissile(py::module& m);
