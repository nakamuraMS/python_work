// Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
#include "Viewer.h"
#include "Utility.h"
#include "SimulationManager.h"
#include "Asset.h"
#include "Agent.h"
using namespace util;

Viewer::Viewer(const nl::json& modelConfig_,const nl::json& instanceConfig_)
:Callback(modelConfig_,instanceConfig_){
    if(isDummy){return;}
    isValid=false;
}
Viewer::~Viewer(){
    close();
}
void Viewer::validate(){
    isValid=true;
}
void Viewer::display(){
}
void Viewer::close(){
}
void Viewer::onEpisodeBegin(){
    if(!isValid){
        validate();
    }
}
void Viewer::onInnerStepBegin(){
}
void Viewer::onInnerStepEnd(){
    if(isValid){
        display();
    }
}
std::shared_ptr<GUIDataFrameCreator> Viewer::getDataFrameCreator(){
    return std::make_shared<GUIDataFrameCreator>();
}
void exportViewer(py::module &m)
{
    using namespace pybind11::literals;
    EXPOSE_CLASS(Viewer)
    DEF_FUNC(Viewer,validate)
    DEF_FUNC(Viewer,display)
    DEF_FUNC(Viewer,close)
    DEF_FUNC(Viewer,onEpisodeBegin)
    DEF_FUNC(Viewer,onInnerStepBegin)
    DEF_FUNC(Viewer,onInnerStepEnd)
    DEF_FUNC(Viewer,getDataFrameCreator)
    DEF_READWRITE(Viewer,isValid)
    ;
}
