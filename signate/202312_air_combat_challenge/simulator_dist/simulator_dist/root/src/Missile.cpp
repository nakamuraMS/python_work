// Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
#include "Missile.h"
#include <cmath>
#include <functional>
#include <thread>
#include <future>
#include <mutex>
#include <boost/math/special_functions/ellint_2.hpp>
#include <boost/math/tools/roots.hpp>
#include "MathUtility.h"
#include "Units.h"
#include "SimulationManager.h"
#include "Agent.h"
#include "Fighter.h"
#include "Sensor.h"
#include "CommunicationBuffer.h"
using namespace util;
PropNav::PropNav(const nl::json& modelConfig_,const nl::json& instanceConfig_)
:Controller(modelConfig_,instanceConfig_){
    if(isDummy){return;}
    manager=instanceConfig.at("manager");
    try{
        parent=Entity::from_json_weakref<Asset>(instanceConfig.at("parent"));
    }catch(...){
        //
    }
    //modelConfigで指定するもの
    gain=getValueFromJsonKR(modelConfig,"G",randomGen);
    commands={
        {"accel",Eigen::Vector3d(0,0,0)},
        {"omega",Eigen::Vector3d(0,0,0)}
    };
}
void PropNav::control(){
    auto msl=getShared<Missile>(parent);
    if(msl->hasLaunched && msl->isAlive()){
        nl::json pc=parent.lock()->commands.at("Navigator");
        auto ret=calc(pc.at("rs"),pc.at("vs"),pc.at("rt"),pc.at("vt"));
        commands={
            {"accel",ret.first},
            {"omega",ret.second}
        };
    }
}
std::pair<Eigen::Vector3d,Eigen::Vector3d> PropNav::calc(const Eigen::Vector3d &rs,const Eigen::Vector3d &vs,const Eigen::Vector3d &rt,const Eigen::Vector3d &vt){
    Eigen::Vector3d rr=rt-rs;
    Eigen::Vector3d vr=vt-vs;
    double R2=rr.squaredNorm();
    if(R2==0){
        //本来は1/|rr|のスケールで発散するが、rrの向きが不定となる|rr|=0のときに限り、rrとvsが平行とみなすことによって指令値を0とする。
        return std::make_pair(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,0));
    }
    double Vs2=vs.squaredNorm();
    if(Vs2==0){
        //vsの向きが不定となる|vs|=0のときに限り、vsとomegaが平行とみなすことによって指令値を0とする。
        //ただし、現状のMissileクラスの運動モデルの実装の方が|v|=0に対応していないためこのような状況下では使用されない。
        return std::make_pair(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,0));
    }
    Eigen::Vector3d omega=rr.cross(vr)/R2;
    Eigen::Vector3d accel=gain*omega.cross(vs);
    return std::make_pair(accel,vs.cross(accel)/Vs2);
}

Missile::Missile(const nl::json& modelConfig_,const nl::json& instanceConfig_)
:PhysicalAsset(modelConfig_,instanceConfig_){
    if(isDummy){return;}
    //位置、姿勢等の運動状態に関する変数の初期化
    if(!parent.expired()){
        auto p=parent.lock();
        motion.pos=p->posI();
        motion.vel=p->velI();
        motion.omega=p->omegaI();
        motion.q=p->qI();
        vel_prev=p->vel_prev;
        pos_prev=p->pos_prev;
    }else{
        motion.pos<<0,0,0;
        motion.vel<<0,0,0;
        motion.omega<<0,0,0;
        motion.q=Quaternion(1,0,0,0);
        vel_prev=motion.vel;
        if(!manager->expired()){
            pos_prev=motion.pos-motion.vel*interval[SimPhase::BEHAVE]*manager->getBaseTimeStep();
        }else{
            pos_prev=motion.pos;
        }
    }
    accel<<0,0,0;
    accelScalar=0.0;
    //その他の内部変数
    target=Track3D();
    hasLaunched=false;
    isAlive_=true;
    mode=Mode::MEMORY;
    estTPos=estTVel=Eigen::Vector3d(0,0,0);
    launchedT=0.0;
    observables["hasLaunched"]=hasLaunched;
    observables["mode"]=enumToJson(mode);
    observables["target"]=Track3D(target);
    observables["spec"]={
        {"sensor",{}}
    };
}
Missile::~Missile(){}
void Missile::makeChildren(){
    std::string snsrName=fullName+":Sensor";
    nl::json sub={
        {"fullName",fullName+":Sensor"},
        {"seed",randomGen()},
        {"parent",this->weak_from_this()},
        {"isBound",true}
    };
    sensor=manager->generateAsset<MissileSensor>(
        "PhysicalAsset",
        modelConfig.at("Sensor"),
        sub
    );
    observables["spec"]["sensor"]=sensor.lock()->observables["spec"];
    sub={
        {"fullName",fullName+":Navigator"},
        {"parent",this->weak_from_this()}
    };
    controllers["Navigator"]=manager->generateAsset<Controller>("Controller",modelConfig.at("Navigator"),sub);
}
void Missile::validate(){
    py::gil_scoped_acquire acquire;
    auto os=py::module_::import("os");
    std::string moduleName=py::cast<std::string>(this->restoreOriginalClassForPY(false).attr("__module__"));
    auto self=py::module_::import(moduleName.c_str());
    auto np=py::module_::import("numpy");
    py::object loaded;
    std::string fileName=modelConfig["rangeTable"];
    std::string filePath=fileName;
    if(py::cast<bool>(os.attr("path").attr("exists")(filePath))){
        loaded=np.attr("load")(filePath);
    }
    filePath=py::cast<std::string>(os.attr("path").attr("join")(os.attr("getcwd")(),fileName));
    if(py::cast<bool>(os.attr("path").attr("exists")(filePath))){
        loaded=np.attr("load")(filePath);
    }else{
        filePath=py::cast<std::string>(os.attr("path").attr("join")(
            os.attr("path").attr("dirname")(self.attr("__file__")),fileName));
        try{
            loaded=np.attr("load")(filePath);
        }catch(...){
            bool createRangeTableIfMissing=false;
            if(modelConfig.contains("createRangeTableIfMissing")){
                createRangeTableIfMissing=modelConfig.at("createRangeTableIfMissing").get<bool>();
            }
            if(createRangeTableIfMissing){
                std::cout<<fileName<<" is not found."<<std::endl;
                filePath=py::cast<std::string>(os.attr("path").attr("join")(os.attr("getcwd")(),fileName));
                makeRangeTable(filePath);
                loaded=np.attr("load")(filePath);
            }else{
                throw std::runtime_error("The range table file \""+fileName+"\"is not found.");
            }
        }
    }
    rangeTablePoints={
        py::cast<Eigen::VectorXd>(loaded["vs"]),
        py::cast<Eigen::VectorXd>(loaded["hs"]),
        py::cast<Eigen::VectorXd>(loaded["vt"]),
        py::cast<Eigen::VectorXd>(loaded["ht"]),
        py::cast<Eigen::VectorXd>(loaded["obs"]),
        py::cast<Eigen::VectorXd>(loaded["aa"])
    };
    rangeTable=py::cast<Eigen::Tensor<double,6>>(loaded["ranges"]);
    auto p=parent.lock();
    motion.pos=p->posI();
    motion.vel=p->velI();
    motion.omega=p->omegaI();
    motion.q=p->qI();
    vel_prev=p->vel_prev;
    pos_prev=p->pos_prev;
}
void Missile::setDependency(){
    dependencyChecker->addDependency(SimPhase::PERCEIVE,sensor.lock());
    controllers["Navigator"].lock()->dependencyChecker->addDependency(SimPhase::CONTROL,getShared<Asset>(this->shared_from_this()));
    for(auto&& asset:manager->getAssets([](std::shared_ptr<const Asset> asset)->bool{
            return isinstance<Fighter>(asset);
        })){
        dependencyChecker->addDependency(SimPhase::BEHAVE,getShared(asset));
    }
}
void Missile::perceive(bool inReset){
    PhysicalAsset::perceive(inReset);
    observables["hasLaunched"]=hasLaunched;
    observables["launchedT"]=hasLaunched ? launchedT : -1;
    if(hasLaunched){
        std::pair<bool,Track3D> ret;
        if(sensor.lock()->isActive){
            ret=sensor.lock()->isTracking(target);
        }else{
            ret=std::make_pair(false,Track3D());
        }
        if(ret.first){
            mode=Mode::SELF;
            target=ret.second.copy();
            targetUpdatedTime=manager->getTime();
            estTPos=target.posI();
            estTVel=target.velI();
        }else{
            auto data=communicationBuffers["MissileComm:"+getFullName()].lock()->receive("target");
            ret=std::dynamic_pointer_cast<Fighter>(parent.lock())->isTracking(target);
            if(data.first>=0 && data.first>=targetUpdatedTime && !data.second.get<Track3D>().is_none()){
                mode=Mode::GUIDED;
                target=data.second;
                targetUpdatedTime=manager->getTime();
                estTPos=target.posI();
                estTVel=target.velI();
            }else{
                mode=Mode::MEMORY;
                estTPos+=estTVel*interval[SimPhase::PERCEIVE]*manager->getBaseTimeStep();
            }
        }
        observables["mode"]=enumToJson(mode);
        observables["target"]=Track3D(target);
    }
}
void Missile::control(){
    auto launchFlag=communicationBuffers["MissileComm:"+getFullName()].lock()->receive("launch");
    if(!hasLaunched && (launchFlag.first>=0 && launchFlag.second)){
        target=communicationBuffers["MissileComm:"+getFullName()].lock()->receive("target").second;
        targetUpdatedTime=manager->getTime();
        launchedT=manager->getTime();
        calcQ();
        mode=Mode::GUIDED;
        estTPos=target.posI();
        estTVel=target.velI();
        hasLaunched=true;
    }
    if(hasLaunched){
        commands={
            {"Sensor",nl::json::array({
                {
                    {"name","steering"},
                    {"estTPos",estTPos},
                    {"estTVel",estTVel}
                }
            })},
            {"Navigator",{
                {"rs",posI()},
                {"vs",velI()},
                {"rt",estTPos},
                {"vt",estTVel}
            }}
        };
        double L=(estTPos-posI()).norm();
        if(L<sensor.lock()->Lref){
            commands["Sensor"].push_back({
                {"name","activate"},
                {"target",target}
            });
        }
    }
}
void Missile::behave(){
    if(hasLaunched){
        double tAftLaunch=manager->getTime()-launchedT;
        calcMotion(tAftLaunch,interval[SimPhase::BEHAVE]*manager->getBaseTimeStep());
        for(auto&& e:manager->getAssets([&](std::shared_ptr<const Asset> asset)->bool{
            return asset->getTeam()!=getTeam() && isinstance<Fighter>(asset);
        })){
            Eigen::Vector3d tpos=e.lock()->posI();
            Eigen::Vector3d tpos_prev=e.lock()->pos_prev;
            if(
                e.lock()->isAlive() &&
                hitCheck(tpos,tpos_prev)
            ){//命中
                manager->requestToKillAsset(e.lock());
                nl::json cmd={
                    {"wpn",this->shared_from_this()},
                    {"tgt",e.lock()}
                };
                manager->triggerEvent("Hit",cmd);
                manager->requestToKillAsset(getShared<Asset>(this->shared_from_this()));
            }
        }
        if(checkDeactivateCondition(tAftLaunch)){
            //命中以外の飛翔終了条件を満たした
            manager->requestToKillAsset(getShared<Asset>(this->shared_from_this()));
        }
    }else{
        std::shared_ptr<PhysicalAsset> parent_=parent.lock();
        motion.pos=parent_->posI();
        motion.vel=parent_->velI();
        motion.omega=parent_->omegaI();
        motion.q=parent_->qI();
        pos_prev=parent_->pos_prev;
        vel_prev=parent_->vel_prev;
    }
}
void Missile::kill(){
    nl::json spec=observables["spec"];
    sensor.lock()->kill();
    for(auto&& e:controllers){
        e.second.lock()->kill();
    }
    this->PhysicalAsset::kill();
    observables["spec"]=spec;
}
void Missile::calcQ(){
    Eigen::Vector3d ex=velI().normalized();
    Eigen::Vector3d ez=Eigen::Vector3d(0,0,1);
    Eigen::Vector3d ey=ez.cross(ex);
    double Y=ey.norm();
    if(Y<1e-8){
        ey=ex.cross(Eigen::Vector3d(1,0,0)).normalized();
    }else{
        ey/=Y;
    }
    ez=ex.cross(ey);
    motion.q=Quaternion::fromBasis(ex,ey,ez);
    motion.calcQh();
}
double Missile::calcRange(double vs,double hs,double vt,double ht,double obs,double aa){
    double r0=30000.0;
    double r1=300000.0;
    double minR=1.0;
    while(!calcRangeSub(vs,hs,vt,ht,obs,aa,r0)){
        r1=r0;
        r0*=0.5;
        if(r0<minR){
            return minR;
        }
    }
    while(calcRangeSub(vs,hs,vt,ht,obs,aa,r1)){
        r0=r1;
        r1*=2;
    }
    double rm;
    while((r1-r0)>1.0){
        rm=(r0+r1)*0.5;
        if(calcRangeSub(vs,hs,vt,ht,obs,aa,rm)){
            r0=rm;
        }else{
            r1=rm;
        }
    }
    return (r0+r1)*0.5;
}
double Missile::calcRangeSub(double vs,double hs,double vt,double ht,double obs,double aa,double r){
    Eigen::Vector3d posBef=motion.pos;
    Eigen::Vector3d velBef=motion.vel;
    Eigen::Vector3d omegaBef=motion.omega;
    Eigen::Vector3d pos_prevBef=pos_prev;
    Eigen::Vector3d vel_prevBef=vel_prev;
    double dt=1.0/10.0;
    motion.pos=Eigen::Vector3d(-r,0,-hs);
    motion.vel=Eigen::Vector3d(cos(obs),-sin(obs),0)*std::max(vs,1e-3);
    motion.omega<<0,0,0;
    calcQ();
    Eigen::Vector3d tpos=Eigen::Vector3d(0,0,-ht);
    Eigen::Vector3d tvel=Eigen::Vector3d(cos(aa),sin(aa),0)*vt;
    Eigen::Vector3d tpos_prev=tpos-dt*tvel;
    pos_prev=motion.pos-dt*motion.vel;
    vel_prev=motion.vel;
    double t=0;
    bool finished=false;
    bool hit=false;
    auto propNav=getShared<PropNav>(controllers["Navigator"]);
    bool hasLaunchedBef=hasLaunched;
    hasLaunched=true;
    while(!finished){
        commands={
            {"Navigator",{
                {"rs",motion.pos},
                {"vs",motion.vel},
                {"rt",tpos},
                {"vt",tvel}
            }}
        };
        propNav->control();
        calcMotion(t,dt);
        tpos_prev=tpos;
        tpos+=dt*tvel;
        t+=dt;
        if(hitCheck(tpos,tpos_prev)){
            finished=true;
            hit=true;
        }else if(checkDeactivateCondition(t)){
            finished=true;
            hit=false;
        }
    }
    motion.pos=posBef;
    motion.vel=velBef;
    motion.omega=omegaBef;
    pos_prev=pos_prevBef;
    vel_prev=vel_prevBef;
    calcQ();
    hasLaunched=hasLaunchedBef;
    return hit;
}
double Missile::getRmax(const Eigen::Vector3d &rs,const Eigen::Vector3d &vs,const Eigen::Vector3d &rt,const Eigen::Vector3d &vt){
    //When aa is omitted, current aa is used.
    double Vs=vs.norm();
    double hs=-rs(2);
    double Vt=vt.norm();
    double ht=-rt(2);
    Eigen::Vector3d dr=Eigen::Vector3d(rt(0)-rs(0),rt(1)-rs(1),0);
    Eigen::Vector3d vsh=Eigen::Vector3d(vs(0),vs(1),0);
    Eigen::Vector3d vth=Eigen::Vector3d(vt(0),vt(1),0);
    double Rh=dr.norm();
    double Vsh=vsh.norm();
    double Vth=vth.norm();
    double obs,aa;
    if(Vsh==0 || Rh==0){
        obs=0.0;
    }else{
        obs=acos(std::min(1.,std::max(-1.,dr.dot(vsh)/(Rh*Vsh))));
    }
    if(Vth==0 || Rh==0){
        aa=0.0;
    }else{
        aa=acos(std::min(1.,std::max(-1.,dr.dot(vth)/(Rh*Vth))));
    }
    if(obs>1e-8){
        bool sameSide=(vsh.cross(dr)).dot(vth.cross(dr))>=0;
        if(!sameSide){
            aa=-aa;
        }
    }
    Eigen::MatrixXd arg(1,6);
    arg<<Vs,hs,Vt,ht,obs,aa;
    return interpn<double>(rangeTablePoints,rangeTable,arg)(0);
}
double Missile::getRmax(const Eigen::Vector3d &rs,const Eigen::Vector3d &vs,const Eigen::Vector3d &rt,const Eigen::Vector3d &vt,const double& aa){
    double Vs=vs.norm();
    double hs=-rs(2);
    double Vt=vt.norm();
    double ht=-rt(2);
    Eigen::Vector3d dr=Eigen::Vector3d(rt(0)-rs(0),rt(1)-rs(1),0);
    Eigen::Vector3d vsh=Eigen::Vector3d(vs(0),vs(1),0);
    double Rh=dr.norm();
    double Vsh=vsh.norm();
    double obs;
    if(Vsh==0 || Rh==0){
        obs=0.0;
    }else{
        obs=acos(std::min(1.,std::max(-1.,dr.dot(vsh)/(Rh*Vsh))));
    }
    Eigen::MatrixXd arg(1,6);
    arg<<Vs,hs,Vt,ht,obs,aa;
    return interpn<double>(rangeTablePoints,rangeTable,arg)(0);
}
void Missile::makeRangeTable(const std::string& dstPath){
    py::gil_scoped_acquire acquire;
    std::cout<<"makeRangeTable start"<<std::endl;
    int nvs=7;
    int nhs=6;
    int nvt=7;
    int nht=6;
    int nobs=7;
    int naa=13;
    Eigen::VectorXd vs=Eigen::VectorXd::LinSpaced(nvs,0.0,600.0);
    Eigen::VectorXd hs=Eigen::VectorXd::LinSpaced(nhs,0.0,20000.0);
    Eigen::VectorXd vt=Eigen::VectorXd::LinSpaced(nvt,0.0,600.0);
    Eigen::VectorXd ht=Eigen::VectorXd::LinSpaced(nht,0.0,20000.0);
    Eigen::VectorXd obs=Eigen::VectorXd::LinSpaced(nobs,0.0,M_PI);
    Eigen::VectorXd aa=Eigen::VectorXd::LinSpaced(naa,-M_PI,M_PI);
    int numDataPoints=nvs*nhs*nht*nvt*nobs*naa;
    Eigen::MatrixXd indices=Eigen::MatrixXd::Zero(6,numDataPoints);//ColMajor
    Eigen::MatrixXd args=Eigen::MatrixXd::Zero(6,numDataPoints);//ColMajor
    std::cout<<"create indices..."<<std::endl;
    int idx=0;
    for(int aa_i=0;aa_i<naa;aa_i++){
        for(int obs_i=0;obs_i<nobs;obs_i++){
            for(int ht_i=0;ht_i<nht;ht_i++){
                for(int vt_i=0;vt_i<nvt;vt_i++){
                    for(int hs_i=0;hs_i<nhs;hs_i++){
                        for(int vs_i=0;vs_i<nvs;++vs_i){
                            args.block(0,idx,6,1)<<vs(vs_i),hs(hs_i),vt(vt_i),ht(ht_i),obs(obs_i),aa(aa_i);
                            idx++;
                        }
                    }
                }
            }
        }
    }
    std::cout<<"create indices done."<<std::endl;
    int numProcess=std::thread::hardware_concurrency();
    std::vector<std::thread> th;
    std::vector<std::future<Eigen::VectorXd>> f;
    std::vector<int> begins;
    int numPerProc=numDataPoints/numProcess;
    std::cout<<"run in "<<numProcess<<" processes. total="<<numDataPoints<<", perProc="<<numPerProc<<std::endl;
    std::mutex mtx;
    std::vector<std::shared_ptr<Missile>> msls;
    for(int i=0;i<numProcess;++i){
        msls.push_back(manager->generateUnmanagedChildByClassName<Missile>("PhysicalAsset",factoryClassName,modelConfig,instanceConfig));
        std::promise<Eigen::VectorXd> p;
        f.emplace_back(p.get_future());
        Eigen::MatrixXd subargs=args.block(0,numPerProc*i,6,(i!=numProcess-1 ? numPerProc : numDataPoints-numPerProc*i));
        th.emplace_back(makeRangeTableSub,std::ref(mtx),std::move(p),msls[i],subargs);
    }
    std::cout<<"waiting calculation"<<std::endl;
    std::vector<Eigen::VectorXd> rangeTablePoints_={vs,hs,vt,ht,obs,aa};
    Eigen::VectorXd returned(numDataPoints);
    for(int i=0;i<numProcess;++i){
        try{
            returned.block(numPerProc*i,0,(i!=numProcess-1 ? numPerProc : numDataPoints-numPerProc*i),1)=f[i].get();
        }catch(std::exception& ex){
            {
                std::lock_guard<std::mutex> lock(mtx);
                std::cout<<"exception in proc("<<i<<"): "<<ex.what()<<std::endl;
            }
        }
    }
    Eigen::Tensor<double,6> rangeTable_=Eigen::TensorMap<Eigen::Tensor<double,6>>(returned.data(),nvs,nhs,nvt,nht,nobs,naa);
    for(int i=0;i<numProcess;++i){
        th[i].join();
    }
    std::cout<<"exporting to npz file"<<std::endl;
    auto np=py::module_::import("numpy");
    np.attr("savez")(dstPath,py::arg("vs")=vs,py::arg("hs")=hs,py::arg("vt")=vt,py::arg("ht")=ht,py::arg("obs")=obs,py::arg("aa")=aa,py::arg("ranges")=rangeTable_);
    std::cout<<"makeRangeTable done."<<std::endl;
}
void makeRangeTableSub(std::mutex& m,std::promise<Eigen::VectorXd> p,std::shared_ptr<Missile> msl,const Eigen::MatrixXd& args){
    try{
        {
            std::lock_guard<std::mutex> lock(m);
            std::cout<<"subProc started. num="<<args.cols()<<std::endl;
        }
        auto mAcc=msl->manager->copy();
        auto dep=DependencyChecker::create(msl->dependencyChecker);
        nl::json sub={
            {"seed",msl->randomGen()},//Entity
            {"fullName",msl->getFullName()+":Navigator"},//Controller
            {"manager",mAcc},
            {"baseTimeStep",msl->manager->getBaseTimeStep()},
            {"dependencyChecker",dep},
            {"parent",msl->weak_from_this()}
        };
        auto propNav=msl->manager->generateUnmanagedChild<Controller>("Controller",msl->modelConfig.at("Navigator"),sub);
        msl->controllers["Navigator"]=propNav;
        Eigen::VectorXd ret(args.cols());
        for(int i=0;i<args.cols();++i){
            ret(i)=msl->calcRange(args(0,i),args(1,i),args(2,i),args(3,i),args(4,i),args(5,i));
            {
                std::lock_guard<std::mutex> lock(m);
                std::cout<<i<<"/"<<(int)(args.cols())<<": "<<args(0,i)<<","<<args(1,i)<<","<<args(2,i)<<","<<args(3,i)<<","<<args(4,i)<<","<<args(5,i)<<" range="<<ret(i)<<std::endl;
            }
        }
        p.set_value(ret);
    }catch(...){
        p.set_exception(std::current_exception());
    }
}

void exportMissile(py::module& m)
{
    using namespace pybind11::literals;
    auto cls=EXPOSE_CLASS(Missile);
    cls
    DEF_FUNC(Missile,makeChildren)
    DEF_FUNC(Missile,validate)
    DEF_FUNC(Missile,setDependency)
    DEF_FUNC(Missile,perceive)
    DEF_FUNC(Missile,control)
    DEF_FUNC(Missile,behave)
    DEF_FUNC(Missile,kill)
    DEF_FUNC(Missile,calcMotion)
    DEF_FUNC(Missile,calcQ)
    DEF_FUNC(Missile,hitCheck)
    DEF_FUNC(Missile,checkDeactivateCondition)
    .def("getRmax",py::overload_cast<const Eigen::Vector3d&,const Eigen::Vector3d&,const Eigen::Vector3d&,const Eigen::Vector3d&>(&Missile::getRmax))
    .def("getRmax",py::overload_cast<const Eigen::Vector3d&,const Eigen::Vector3d&,const Eigen::Vector3d&,const Eigen::Vector3d&,const double&>(&Missile::getRmax))
    DEF_FUNC(Missile,calcRange)
    DEF_FUNC(Missile,calcRangeSub)
    DEF_FUNC(Missile,makeRangeTable)
    DEF_READWRITE(Missile,accel)
    DEF_READWRITE(Missile,accelScalar)
    DEF_READWRITE(Missile,target)
    DEF_READWRITE(Missile,hasLaunched)
    DEF_READWRITE(Missile,mode)
    DEF_READWRITE(Missile,launchedT)
    DEF_READWRITE(Missile,estTPos)
    DEF_READWRITE(Missile,estTVel)
    DEF_READWRITE(Missile,sensor)
    ;
    py::enum_<Missile::Mode>(cls,"Mode")
    .value("GUIDED",Missile::Mode::GUIDED)
    .value("SELF",Missile::Mode::SELF)
    .value("MEMORY",Missile::Mode::MEMORY)
    ;
}
