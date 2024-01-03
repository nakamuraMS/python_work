// Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
#define MY_MODULE_NAME libCore
#include "Common.h"
#include <cmath>
#include <iostream>
#include <pybind11/pybind11.h>
#include "Units.h"
#include "Utility.h"
#include "JsonMutableFromPython.h"
#include "ConfigDispatcher.h"
#include "Quaternion.h"
#include "MathUtility.h"
#include "CommunicationBuffer.h"
#include "Entity.h"
#include "Asset.h"
#include "MotionState.h"
#include "StaticCollisionAvoider2D.h"
#include "PhysicalAsset.h"
#include "Controller.h"
#include "Track.h"
#include "Propulsion.h"
#include "FlightControllerUtility.h"
#include "Fighter.h"
#include "MassPointFighter.h"
#include "CoordinatedFighter.h"
#include "SixDoFFighter.h"
#include "Missile.h"
#include "R5Missile.h"
#include "Sensor.h"
#include "Agent.h"
#include "R5InitialFighterAgent01.h"
#include "Callback.h"
#include "Ruler.h"
#include "R5AirToAirCombatRuler01.h"
#include "Reward.h"
#include "WinLoseReward.h"
#include "R5AirToAirCombatBasicReward01.h"
#include "GUIDataFrameCreator.h"
#include "Viewer.h"
#include "SimulationManager.h"
#include "Factory.h"
namespace py=pybind11;

PYBIND11_MODULE(MY_MODULE_NAME,m)
{    
    using namespace pybind11::literals;
    m.doc()="ASRCAISim1";
    exportCommon(m);
    exportUnits(m);
    exportUtility(m);
    exportJsonMutableFromPython(m);
    exportQuaternion(m);
    exportMathUtility(m);
    exportCommunicationBuffer(m);
    exportEntity(m);
    exportAsset(m);
    exportMotionState(m);
    exportStaticCollisionAvoider2D(m);
    exportPhysicalAsset(m);
    exportController(m);
    exportTrack(m);
    exportPropulsion(m);
    exportFlightControllerUtility(m);
    exportFighter(m);
    exportMassPointFighter(m);
    exportCoordinatedFighter(m);
    exportSixDoFFighter(m);
    exportMissile(m);
    exportR5Missile(m);
    exportSensor(m);
    exportAgent(m);
    exportR5InitialFighterAgent01(m);
    exportCallback(m);
    exportRuler(m);
    exportReward(m);
    exportR5AirToAirCombatRuler01(m);
    exportWinLoseReward(m);
    exportR5AirToAirCombatBasicReward01(m);
    exportGUIDataFrameCreator(m);
    exportViewer(m);
    exportSimulationManager(m);
    exportFactory(m);
    setupBuiltIns();
}
