// Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
#include "GUIDataFrameCreator.h"
#include "SimulationManager.h"
#include "Fighter.h"
#include "Missile.h"
#include "Sensor.h"
#include "Agent.h"
#include "Ruler.h"
using namespace util;

GUIDataFrameCreator::GUIDataFrameCreator(){
}
GUIDataFrameCreator::~GUIDataFrameCreator(){}
nl::json GUIDataFrameCreator::makeFrame(std::shared_ptr<SimulationManagerAccessorForCallback> manager){
    nl::json frame={
        {"time",manager->getTime()},
        {"tick",manager->getTickCount()},
        {"ruler",manager->getRuler().lock()->observables},
        {"scores",manager->scores()},
        {"totalRewards",manager->totalRewards()},
        {"agents",nl::json::object()},
        {"fighters",nl::json::object()},
        {"missiles",nl::json::object()}
    };
    for(auto&& e:manager->getAssets()){
        if(isinstance<Fighter>(e)){
            auto f=getShared<Fighter>(e);
            frame["fighters"][f->getFullName()]={
                {"isAlive",f->isAlive()},
                {"target",f->target},
                {"team",f->getTeam()},
                {"name",f->getName()},
                {"posI",f->posI()},
                {"velI",f->velI()},
                {"agent",f->agent.lock()->getFullName()},
                {"remMsls",f->remMsls},
                {"motion",f->motion},
                {"ex",f->relBtoI(Eigen::Vector3d(1,0,0))},
                {"ey",f->relBtoI(Eigen::Vector3d(0,1,0))},
                {"ez",f->relBtoI(Eigen::Vector3d(0,0,1))},
                {"radar",{{"Lref",f->radar.lock()->Lref}}},
                {"observables",f->observables}
            };
        }else if(isinstance<Missile>(e)){
            auto m=getShared<Missile>(e);
            frame["missiles"][m->getFullName()]={
                {"isAlive",m->isAlive()},
                {"team",m->getTeam()},
                {"name",m->getName()},
                {"posI",m->posI()},
                {"velI",m->velI()},
                {"motion",m->motion},
                {"ex",m->relBtoI(Eigen::Vector3d(1,0,0))},
                {"ey",m->relBtoI(Eigen::Vector3d(0,1,0))},
                {"ez",m->relBtoI(Eigen::Vector3d(0,0,1))},
                {"hasLaunched",m->hasLaunched},
                {"mode",enumToJson(m->mode)},
                {"estTPos",m->estTPos},
                {"sensor",{
                    {"isActive",m->sensor.lock()->isActive},
                    {"Lref",m->sensor.lock()->Lref},
                    {"thetaFOR",m->sensor.lock()->thetaFOR}
                }}
            };
        }
    }
    for(auto&& e:manager->getAgents()){
        auto a=e.lock();
        frame["agents"][a->getFullName()]={
            {"team",a->getTeam()},
            {"repr",a->repr()},
            {"parents",a->parents},
            {"observables",a->observables}
        };
    }
    return frame;
}

void exportGUIDataFrameCreator(py::module& m)
{
    using namespace pybind11::literals;
    py::class_<GUIDataFrameCreator,GUIDataFrameCreatorWrap<>,std::shared_ptr<GUIDataFrameCreator>>(m,"GUIDataFrameCreator")
    .def(py::init<>())
    DEF_FUNC(GUIDataFrameCreator,makeFrame)
    ;
}
