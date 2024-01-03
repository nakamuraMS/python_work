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
#include "Missile.h"
namespace py=pybind11;
namespace nl=nlohmann;

DECLARE_CLASS_WITH_TRAMPOLINE(R5Missile,Missile)
    public:
    //configで指定するもの
    double tMax,tBurn,hitD,minV;
    double massI,massF,thrust,maxLoadG;
    double Sref,maxA,maxD,l,d,ln,lcgi,lcgf,lw,bw,bt,thicknessRatio,Sw,St;
    double frictionFactor;
    //内部変数
    double mass;//質量
    double lcg;//重心位置
    //constructors & destructor
    R5Missile(const nl::json& modelConfig_,const nl::json& instanceConfig_);
    virtual ~R5Missile();
    //functions
    virtual void calcMotion(double tAftLaunch,double dt);
    virtual bool hitCheck(const Eigen::Vector3d &tpos,const Eigen::Vector3d &tpos_prev);
    virtual bool checkDeactivateCondition(double tAftLaunch);
};

DECLARE_TRAMPOLINE(R5Missile)
    virtual void calcMotion(double tAftLaunch,double dt) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(void,Base,calcMotion,tAftLaunch,dt);
    }
    virtual bool hitCheck(const Eigen::Vector3d &tpos,const Eigen::Vector3d &tpos_prev) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(bool,Base,hitCheck,tpos,tpos_prev);
    }
    virtual bool checkDeactivateCondition(double tAftLaunch) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(bool,Base,checkDeactivateCondition,tAftLaunch);
    }
};

void exportR5Missile(py::module& m);
