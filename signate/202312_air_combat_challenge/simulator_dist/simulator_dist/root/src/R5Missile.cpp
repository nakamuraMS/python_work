// Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
#include "R5Missile.h"
#include <boost/math/tools/roots.hpp>
#include "MathUtility.h"
#include "Units.h"
#include "SimulationManager.h"
#include "R5EkkerMDT.h"
using namespace util;

R5Missile::R5Missile(const nl::json& modelConfig_,const nl::json& instanceConfig_)
:Missile(modelConfig_,instanceConfig_){
    if(isDummy){return;}
    tMax=getValueFromJsonKR(modelConfig,"tMax",randomGen);
    tBurn=getValueFromJsonKR(modelConfig,"tBurn",randomGen);
    hitD=getValueFromJsonKR(modelConfig,"hitD",randomGen);
    minV=getValueFromJsonKR(modelConfig,"minV",randomGen);
    massI=getValueFromJsonKR(modelConfig,"massI",randomGen);
    massF=getValueFromJsonKR(modelConfig,"massF",randomGen);
    thrust=getValueFromJsonKR(modelConfig,"thrust",randomGen);
    maxLoadG=getValueFromJsonKR(modelConfig,"maxLoadG",randomGen);
    maxA=deg2rad(getValueFromJsonKR(modelConfig,"maxA",randomGen));
    maxD=deg2rad(getValueFromJsonKR(modelConfig,"maxD",randomGen));
    l=getValueFromJsonKR(modelConfig,"length",randomGen);
    d=getValueFromJsonKR(modelConfig,"diameter",randomGen);
    ln=getValueFromJsonKR(modelConfig,"lengthN",randomGen);
    lcgi=getValueFromJsonKR(modelConfig,"lcgi",randomGen);
    lcgf=getValueFromJsonKR(modelConfig,"lcgf",randomGen);
    lw=getValueFromJsonKR(modelConfig,"locationW",randomGen);
    bw=getValueFromJsonKR(modelConfig,"spanW",randomGen);
    bt=getValueFromJsonKR(modelConfig,"spanT",randomGen);
    thicknessRatio=getValueFromJsonKR(modelConfig,"thicknessRatio",randomGen);
    frictionFactor=getValueFromJsonKRD(modelConfig,"frictionFactor",randomGen,1.1);
    Sw=getValueFromJsonKR(modelConfig,"areaW",randomGen);
    St=getValueFromJsonKR(modelConfig,"areaT",randomGen);
    Sref=M_PI*d*d/4;
    mass=massI;
    lcg=lcgi;
}
R5Missile::~R5Missile(){}
void R5Missile::calcMotion(double tAftLaunch,double dt){
    pos_prev=motion.pos;
    vel_prev=motion.vel;
    double V=motion.vel.norm();
    auto propNav=getShared<PropNav>(controllers["Navigator"]);
    Eigen::Vector3d accelCmd=propNav->commands["accel"];
    Eigen::Vector3d omegaV=propNav->commands["omega"];
    double omega=omegaV.norm();
    double omegaMax=maxLoadG*gravity/V;
    if(omega>omegaMax){
        omegaV*=(omegaMax/omega);
        omega=omegaMax;
        accelCmd=omegaV.cross(motion.vel);
    }
    double alt=-motion.pos(2);
    bool pwroff=tAftLaunch>=tBurn;
    double thrust_;
    if(pwroff){
        thrust_ = 0.0;
        mass = massF;
        lcg = lcgf;
    }else{
        thrust_ = thrust;
        mass = massI+(massF-massI)*(tAftLaunch/tBurn-1);
        lcg = lcgi+(lcgf-lcgi)*(tAftLaunch/tBurn-1);
    }
    std::map<std::string,double> atm=atmosphere(alt);
    double M=V/atm["a"];
    double qd=0.5*atm["rho"]*V*V;
    Eigen::Vector3d ex=motion.vel/V;
    Eigen::Vector3d ey=(accelCmd-Eigen::Vector3d(0,0,gravity)).cross(ex)*mass;
    double desiredSideForce=ey.norm();
    Eigen::Vector3d sideAx;
    double aoa,CL,CD;
    if(desiredSideForce>0){
        ey/=desiredSideForce;
        sideAx=ex.cross(ey);
        std::pair<double,double> tmp=R5EkkerMDT::cldcmd(d,l,ln,lcg,bt,St,M);
        double cld=tmp.first;
        double cmd=tmp.second;
        auto func=[&](double aoa_){
            std::pair<double,double> tmp=R5EkkerMDT::clacma(d,l,ln,lcg,bw,Sw,lw,bt,St,M,aoa_);
            double cla=tmp.first;
            double cma=tmp.second;
            double delta=std::clamp<double>(-cma/cmd*aoa_,-maxD,maxD);
            return (cla*aoa_+cld*delta)*(qd*Sref)+thrust_*sin(aoa_)-desiredSideForce;
        };
        if(func(0.0)>=0){
            aoa=0.0;
        }else if(func(maxA)<=0){
            aoa=maxA;
        }else{
            try{
                boost::math::tools::eps_tolerance<double> tol(16);
                boost::uintmax_t max_iter=10;
                auto result=boost::math::tools::toms748_solve(func,0.0,maxA,tol,max_iter);
                aoa=(result.first+result.second)/2;
            }catch(std::exception& e){
                std::cout<<"exception at R5Missile::calcMotion"<<std::endl;
                DEBUG_PRINT_EXPRESSION(func(0.0))
                DEBUG_PRINT_EXPRESSION(func(maxA))
                DEBUG_PRINT_EXPRESSION(alt)
                DEBUG_PRINT_EXPRESSION(M)
                DEBUG_PRINT_EXPRESSION(desiredSideForce)
                DEBUG_PRINT_EXPRESSION(thrust_)
                throw e;
            }
        }
        tmp=R5EkkerMDT::clacma(d,l,ln,lcg,bw,Sw,lw,bt,St,M,aoa);
        double cla=tmp.first;
        double cma=tmp.second;
        double delta=std::clamp<double>(-cma/cmd*aoa,-maxD,maxD);
        //CL=(cla-cld*cma/cmd)*aoa;
        CL=cla*aoa+cld*delta;
        CD=R5EkkerMDT::cdtrim(d,l,ln,bw,Sw,thicknessRatio,bt,St,thicknessRatio,M,alt,aoa,delta,frictionFactor,pwroff);
    }else{
        sideAx<<0,0,0;
        CL=0.0;
        CD=R5EkkerMDT::cdtrim(d,l,ln,bw,Sw,thicknessRatio,bt,St,thicknessRatio,M,alt,0,0,frictionFactor,pwroff);
        aoa=0.0;
    }
    Eigen::Vector3d g(0,0,gravity);
    Eigen::Vector3d sideAccel=sideAx*(CL*qd*Sref+thrust_*sin(aoa))/mass+(g-ex*(g.dot(ex)));
	//attitude and deflection are assumed to be immidiately adjustable as desired.
	accelScalar=(-CD*Sref*qd+thrust_*cos(aoa))/mass+g.dot(ex);
	Eigen::Vector3d accel=ex*accelScalar;
    omegaV=motion.vel.cross(sideAccel)/(V*V);
    omega=omegaV.norm();
    if(omega*dt<1e-8){//ほぼ無回転
        motion.pos+=motion.vel*dt+accel*dt*dt/2.;
        motion.vel+=accel*dt;
    }else{
        Eigen::Vector3d ex0=motion.vel/V;
        Eigen::Vector3d ey0=omegaV.cross(ex0).normalized();
        double wt=omega*dt;
        double acc=accelScalar;
        Eigen::Vector3d vn=(ex0*cos(wt)+ey0*sin(wt)).normalized();
        motion.vel=(V+acc*dt)*vn;
        Eigen::Vector3d dr=V*dt*(ex0*sinc(wt)+ey0*oneMinusCos_x2(wt)*wt)+acc*dt*dt*(ex0*(sinc(wt)-oneMinusCos_x2(wt))+ey0*wt*(sincMinusOne_x2(wt)+oneMinusCos_x2(wt)));
        motion.pos+=dr;
        accel=(motion.vel-vel_prev)/dt;
    }
    motion.omega=omegaV;
    motion.time=manager->getTime()+dt;
    calcQ();
}
bool R5Missile::hitCheck(const Eigen::Vector3d &tpos,const Eigen::Vector3d &tpos_prev){
    Eigen::Vector3d r1=pos_prev;
    Eigen::Vector3d r2=tpos_prev;
    Eigen::Vector3d d1=posI()-r1;
    Eigen::Vector3d d2=tpos-r2;
    Eigen::Vector3d A=r1-r2;double a=A.norm();
    Eigen::Vector3d B=d1-d2;double b=B.norm();
    if(a<hitD){//初期位置であたり
        return true;
    }else if(b<1e-8){//相対速度がほぼ0}
        return a<hitD;
    }
    double tMin=std::min(1.,std::max(0.,-2*A.dot(B)/(b*b)));
    return (A+B*tMin).norm()<hitD;
}
bool R5Missile::checkDeactivateCondition(double tAftLaunch){
    //最大飛翔時間超過or地面に激突or燃焼後に速度が規定値未満
    bool ret=(tAftLaunch>=tMax ||
            motion.pos(2)>0 ||
            (tAftLaunch>=tBurn && motion.vel.norm()<minV)
    );
    return ret;
}

void exportR5Missile(py::module& m)
{
    using namespace pybind11::literals;
    auto cls=EXPOSE_CLASS(R5Missile);
    cls
    DEF_FUNC(R5Missile,calcMotion)
    DEF_FUNC(R5Missile,hitCheck)
    DEF_FUNC(R5Missile,checkDeactivateCondition)
    DEF_READWRITE(R5Missile,tMax)
    DEF_READWRITE(R5Missile,tBurn)
    DEF_READWRITE(R5Missile,hitD)
    DEF_READWRITE(R5Missile,minV)
    DEF_READWRITE(R5Missile,mass)
    DEF_READWRITE(R5Missile,massI)
    DEF_READWRITE(R5Missile,massF)
    DEF_READWRITE(R5Missile,thrust)
    DEF_READWRITE(R5Missile,maxLoadG)
    DEF_READWRITE(R5Missile,Sref)
    DEF_READWRITE(R5Missile,maxA)
    DEF_READWRITE(R5Missile,l)
    DEF_READWRITE(R5Missile,d)
    DEF_READWRITE(R5Missile,ln)
    DEF_READWRITE(R5Missile,lcg)
    DEF_READWRITE(R5Missile,lcgi)
    DEF_READWRITE(R5Missile,lcgf)
    DEF_READWRITE(R5Missile,lw)
    DEF_READWRITE(R5Missile,bw)
    DEF_READWRITE(R5Missile,bt)
    DEF_READWRITE(R5Missile,thicknessRatio)
    DEF_READWRITE(R5Missile,frictionFactor)
    DEF_READWRITE(R5Missile,Sw)
    DEF_READWRITE(R5Missile,St)
    ;
}
