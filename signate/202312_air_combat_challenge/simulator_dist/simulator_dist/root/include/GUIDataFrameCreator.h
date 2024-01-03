// Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
#pragma once
#include <pybind11/pybind11.h>
#include "Utility.h"
namespace py=pybind11;
namespace nl=nlohmann;
class SimulationManagerAccessorForCallback;

class PYBIND11_EXPORT GUIDataFrameCreator{
    /*GUI描画のために必要なデータフレームを生成するための基底クラス。
    SimulationManager内部で動くViewerクラスであれば直接各インスタンスにアクセスしても差し支えないが、
    ログデータや別プロセスから描画する際に描画処理を再利用することを念頭に置いて、
    nl::json型でデータフレームを生成することを標準仕様とする。
    */
    public:
    //constructors & destructor
    GUIDataFrameCreator();
    virtual ~GUIDataFrameCreator();
    //functions
    virtual nl::json makeFrame(std::shared_ptr<SimulationManagerAccessorForCallback> manager);
};
template<class Base=GUIDataFrameCreator>
class GUIDataFrameCreatorWrap:public Base{
    public:
    using Base::Base;
    virtual nl::json makeFrame(std::shared_ptr<SimulationManagerAccessorForCallback> manager) override{
        PYBIND11_OVERRIDE(nl::json,Base,makeFrame,manager);
    }
};
void exportGUIDataFrameCreator(py::module& m);
