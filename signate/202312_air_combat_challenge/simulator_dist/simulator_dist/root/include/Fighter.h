// Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
#pragma once
#include <deque>
#include <vector>
#include <functional>
#include <pybind11/pybind11.h>
#include <Eigen/Core>
#include "MathUtility.h"
#include "PhysicalAsset.h"
#include "Controller.h"
#include "Track.h"
#include "Propulsion.h"
namespace py=pybind11;
namespace nl=nlohmann;
class Missile;
class AircraftRadar;
class MWS;

DECLARE_CLASS_WITH_TRAMPOLINE(Fighter,PhysicalAsset)
    public:
    double m;//Mass in [kg]
    double rcsScale;//as a dimensionless value
    double fuelCapacity;//In [kg]
    std::weak_ptr<Propulsion> engine;
    std::weak_ptr<AircraftRadar> radar;
    std::weak_ptr<MWS> mws;
    std::vector<std::weak_ptr<Missile>> missiles;
    std::weak_ptr<Missile> dummyMissile;
    std::vector<std::pair<Track3D,bool>> missileTargets;
    int nextMsl,numMsls,remMsls;
    bool isDatalinkEnabled;
    std::vector<Track3D> track;
    std::vector<std::vector<std::string>> trackSource;
    std::string datalinkName;
    Track3D target;int targetID;
    bool enableThrustAfterFuelEmpty;
    double fuelRemaining;
    double optCruiseFuelFlowRatePerDistance;
    public:
    //constructors & destructor
    Fighter(const nl::json& modelConfig_,const nl::json& instanceConfig_);
    virtual ~Fighter();
    //functions
    virtual void makeChildren();
    virtual void validate();
    virtual void setDependency();
    virtual void perceive(bool inReset);
    virtual void control();
    virtual void behave();
    virtual void kill();
    virtual double getThrust();// In [N]
    virtual double calcThrust(const nl::json& args);//as generic interface
    virtual double calcOptimumCruiseFuelFlowRatePerDistance()=0;//Minimum fuel flow rate (per distance) in level steady flight
    virtual double getMaxReachableRange();//In optimized level steady flight
    virtual std::pair<bool,Track3D> isTracking(std::weak_ptr<PhysicalAsset> target_);
    virtual std::pair<bool,Track3D> isTracking(const Track3D& target_);
    virtual std::pair<bool,Track3D> isTracking(const boost::uuids::uuid& target_);
    virtual bool isLaunchable();
    virtual bool isLaunchableAt(const Track3D& target_);
    virtual void setFlightControllerMode(const std::string& ctrlName);
    virtual void calcMotion(double dt)=0;
    virtual Eigen::Vector3d toEulerAngle();
    virtual double getRmax(const Eigen::Vector3d &rs,const Eigen::Vector3d &vs,const Eigen::Vector3d &rt,const Eigen::Vector3d &vt);
    virtual double getRmax(const Eigen::Vector3d &rs,const Eigen::Vector3d &vs,const Eigen::Vector3d &rt,const Eigen::Vector3d &vt,const double& aa);
    //座標変換
    virtual Eigen::Vector3d relHtoI(const Eigen::Vector3d &v) const;
    virtual Eigen::Vector3d relItoH(const Eigen::Vector3d &v) const;
    virtual Eigen::Vector3d absHtoI(const Eigen::Vector3d &v) const;
    virtual Eigen::Vector3d absItoH(const Eigen::Vector3d &v) const;
    virtual std::shared_ptr<AssetAccessor> getAccessor();
    DECLARE_CLASS_WITHOUT_TRAMPOLINE(SensorDataSharer,Controller)
        public:
        SensorDataSharer(const nl::json& modelConfig_,const nl::json& instanceConfig_);
        virtual ~SensorDataSharer();
        virtual void perceive(bool inReset) override;
    };
    DECLARE_CLASS_WITHOUT_TRAMPOLINE(SensorDataSanitizer,Controller)
        public:
        std::map<std::string,double> lastSharedTime;
        SensorDataSanitizer(const nl::json& modelConfig_,const nl::json& instanceConfig_);
        virtual ~SensorDataSanitizer();
        virtual void perceive(bool inReset) override;
    };
    DECLARE_CLASS_WITHOUT_TRAMPOLINE(OtherDataSharer,Controller)
        public:
        OtherDataSharer(const nl::json& modelConfig_,const nl::json& instanceConfig_);
        virtual ~OtherDataSharer();
        virtual void perceive(bool inReset) override;
        virtual void control() override;
    };
    DECLARE_CLASS_WITHOUT_TRAMPOLINE(OtherDataSanitizer,Controller)
        public:
        std::map<std::string,double> lastSharedTimeOfAgentObservable;
        std::map<std::string,double> lastSharedTimeOfFighterObservable;
        double lastSharedTimeOfAgentCommand;
        OtherDataSanitizer(const nl::json& modelConfig_,const nl::json& instanceConfig_);
        virtual ~OtherDataSanitizer();
        virtual void perceive(bool inReset) override;
        virtual void control() override;
    };
    DECLARE_CLASS_WITH_TRAMPOLINE(HumanIntervention,Controller)
        //delay for shot command in order to simulate approval by human operator.
        public:
        int capacity;
        double delay,cooldown;//in seconds.
        double lastShotApprovalTime;//in seconds.
        std::deque<std::pair<double,nl::json>> recognizedShotCommands;//in seconds.
        HumanIntervention(const nl::json& modelConfig_,const nl::json& instanceConfig_);
        virtual ~HumanIntervention();
        virtual bool isLaunchable();
        virtual bool isLaunchableAt(const Track3D& target_);
        virtual void control() override;
    };
    DECLARE_CLASS_WITH_TRAMPOLINE(WeaponController,Controller)
        public:
        bool enable_loal;
        double launchRangeLimit;// in meters
        double offBoresightAngleLimit;// in degrees
        bool enable_utdc;
        bool enable_takeover_guidance;
        WeaponController(const nl::json& modelConfig_,const nl::json& instanceConfig_);
        virtual ~WeaponController();
        virtual bool isLaunchable();
        virtual bool isLaunchableAt(const Track3D& target_);
        virtual void control() override;
    };
    DECLARE_CLASS_WITH_TRAMPOLINE(FlightController,Controller)
        public:
        //constructors & destructor
        FlightController(const nl::json& modelConfig_,const nl::json& instanceConfig_);
        virtual ~FlightController();
        virtual void control() override;
        virtual nl::json getDefaultCommand();
        virtual nl::json calc(const nl::json &cmd);
        virtual void setMode(const std::string& mode_);
        protected:
        std::string mode;
    };
};
DECLARE_TRAMPOLINE(Fighter::HumanIntervention)
    virtual bool isLaunchable() override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(bool,Base,isLaunchable);
    }
    virtual bool isLaunchableAt(const Track3D& target_) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(bool,Base,isLaunchableAt,target_);
    }
};
DECLARE_TRAMPOLINE(Fighter::WeaponController)
    virtual bool isLaunchable() override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(bool,Base,isLaunchable);
    }
    virtual bool isLaunchableAt(const Track3D& target_) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(bool,Base,isLaunchableAt,target_);
    }
};
DECLARE_TRAMPOLINE(Fighter::FlightController)
    virtual nl::json getDefaultCommand() override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(nl::json,Base,getDefaultCommand);
    }
    virtual nl::json calc(const nl::json &cmd) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(nl::json,Base,calc,cmd);
    }
    virtual void setMode(const std::string& mode_) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(void,Base,setMode,mode_);
    }
};

DECLARE_TRAMPOLINE(Fighter)
    virtual double getThrust() override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(double,Base,getThrust);
    }
    virtual double calcThrust(const nl::json& args) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(double,Base,calcThrust,args);
    }
    virtual double calcOptimumCruiseFuelFlowRatePerDistance() override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE_PURE(double,Base,calcOptimumCruiseFuelFlowRatePerDistance);
    }
    virtual std::pair<bool,Track3D> isTracking(std::weak_ptr<PhysicalAsset> target_) override{
        typedef std::pair<bool,Track3D> retType;
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(retType,Base,isTracking,target_);
    }
    virtual std::pair<bool,Track3D> isTracking(const Track3D&  target_) override{
        typedef std::pair<bool,Track3D> retType;
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(retType,Base,isTracking,target_);
    }
    virtual bool isLaunchable() override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(bool,Base,isLaunchable);
    }
    virtual bool isLaunchableAt(const Track3D& target_) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(bool,Base,isLaunchableAt,target_);
    }
    virtual void setFlightControllerMode(const std::string& ctrlName) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(void,Base,setFlightControllerMode,ctrlName);
    }
    virtual void calcMotion(double dt) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE_PURE(void,Base,calcMotion,dt);
    }
    virtual Eigen::Vector3d toEulerAngle() override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(Eigen::Vector3d,Base,toEulerAngle);
    }
    virtual double getRmax(const Eigen::Vector3d &rs,const Eigen::Vector3d &vs,const Eigen::Vector3d &rt,const Eigen::Vector3d &vt){
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(double,Base,getRmax,rs,vs,rt,vt);
    }
    virtual double getRmax(const Eigen::Vector3d &rs,const Eigen::Vector3d &vs,const Eigen::Vector3d &rt,const Eigen::Vector3d &vt,const double& aa){
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(double,Base,getRmax,rs,vs,rt,vt,aa);
    }
    virtual Eigen::Vector3d relHtoI(const Eigen::Vector3d &v) const override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(Eigen::Vector3d,Base,relHtoI,v);
    }
    virtual Eigen::Vector3d relItoH(const Eigen::Vector3d &v) const override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(Eigen::Vector3d,Base,relItoH,v);
    }
    virtual Eigen::Vector3d absHtoI(const Eigen::Vector3d &v) const override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(Eigen::Vector3d,Base,absHtoI,v);
    }
    virtual Eigen::Vector3d absItoH(const Eigen::Vector3d &v) const override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(Eigen::Vector3d,Base,absItoH,v);
    }
};

DECLARE_CLASS_WITH_TRAMPOLINE(FighterAccessor,PhysicalAssetAccessor)
    friend class SimulationManager;
    public:
    FighterAccessor(std::shared_ptr<Fighter> a);
    virtual ~FighterAccessor();
    template<class T>
    bool isinstance(){
        return util::isinstance<T>(asset);
    }
    virtual void setFlightControllerMode(const std::string& ctrlName="");
    virtual double getMaxReachableRange();
    virtual double getRmax(const Eigen::Vector3d &rs,const Eigen::Vector3d &vs,const Eigen::Vector3d &rt,const Eigen::Vector3d &vt,const double& aa);
    virtual double getRmax(const Eigen::Vector3d &rs,const Eigen::Vector3d &vs,const Eigen::Vector3d &rt,const Eigen::Vector3d &vt);
    virtual bool isLaunchable();
    virtual bool isLaunchableAt(const Track3D& target_);
    protected:
    std::weak_ptr<Fighter> asset;
};
DECLARE_TRAMPOLINE(FighterAccessor)
    virtual void setFlightControllerMode(const std::string& ctrlName="") override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(void,Base,setFlightControllerMode,ctrlName);
    }
    virtual double getMaxReachableRange() override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(double,Base,getMaxReachableRange);
    }
    virtual double getRmax(const Eigen::Vector3d &rs,const Eigen::Vector3d &vs,const Eigen::Vector3d &rt,const Eigen::Vector3d &vt,const double& aa) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(double,Base,getRmax,rs,vs,rt,vt,aa);
    }
    virtual double getRmax(const Eigen::Vector3d &rs,const Eigen::Vector3d &vs,const Eigen::Vector3d &rt,const Eigen::Vector3d &vt) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(double,Base,getRmax,rs,vs,rt,vt);
    }
    virtual bool isLaunchable() override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(bool,Base,isLaunchable);
    }
    virtual bool isLaunchableAt(const Track3D& target_) override{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERRIDE(bool,Base,isLaunchableAt,target_);
    }
};

void exportFighter(py::module& m);