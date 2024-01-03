// Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
#pragma once
#include <iostream>
#include <vector>
#include <map>
#include <pybind11/pybind11.h>
#include "Callback.h"
#include "GUIDataFrameCreator.h"

namespace py=pybind11;
namespace nl=nlohmann;

DECLARE_CLASS_WITH_TRAMPOLINE(Viewer,Callback)
    public:
    bool isValid;
    std::string name;
    //constructors & destructor
    Viewer(const nl::json& modelConfig_,const nl::json& instanceConfig_);
    virtual ~Viewer();
    //functions
    virtual void validate();
    virtual void display();
    virtual void close();
    virtual void onEpisodeBegin();
    virtual void onInnerStepBegin();
    virtual void onInnerStepEnd();
    virtual std::shared_ptr<GUIDataFrameCreator> getDataFrameCreator();
};
DECLARE_TRAMPOLINE(Viewer)
    virtual void validate() override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(void,Base,validate);
    }
    virtual void display() override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(void,Base,display);
    }
    virtual void close() override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(void,Base,close);
    }
    virtual std::shared_ptr<GUIDataFrameCreator> getDataFrameCreator() override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(std::shared_ptr<GUIDataFrameCreator>,Base,getDataFrameCreator);
    }
};
void exportViewer(py::module &m);
