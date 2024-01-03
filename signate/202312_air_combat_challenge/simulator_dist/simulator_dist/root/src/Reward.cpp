// Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
#include "Reward.h"
#include "Utility.h"
#include "SimulationManager.h"
#include "Asset.h"
#include "Agent.h"
#include "Ruler.h"
#include <regex>
using namespace util;

Reward::Reward(const nl::json& modelConfig_,const nl::json& instanceConfig_)
:Callback(modelConfig_,instanceConfig_){
    if(isDummy){return;}
    try{
        j_target=instanceConfig.at("target");
    }catch(...){
        j_target="All";
    }
    if(j_target.is_null()){
        j_target="All";
    }
    isTeamReward=false;
}
Reward::~Reward(){}
void Reward::onEpisodeBegin(){
    target.clear();
    reward.clear();
    totalReward.clear();
    if(j_target.is_string()){
        std::string s_target=j_target;
        setupTarget(s_target);
    }else if(j_target.is_array()){
        target=j_target.get<std::vector<std::string>>();
        for(auto &&t:target){
            setupTarget(t);
        }
    }else{
        std::cout<<"instanceConfig['target']="<<j_target<<std::endl;
        throw std::runtime_error("Invalid designation of 'target' for Reward. Use either string or array of string.");
    }
}
void Reward::onStepBegin(){
    //派生クラスでは最初に必ずこのクラスのonStepBeginを呼び出すか、同等の処理を記述すること。
    auto agentsActed=manager->agentsActed();
    if(isTeamReward){
        for(auto &&team:target){
            reward[team]=0;
        }
        for(auto && e:agentsActed){
            auto fullName=e.first;
            auto agent=e.second.lock();
            auto team=agent->getTeam();
            if(reward.find(team)!=reward.end()){
                reward[fullName]=0;
            }
        }
    }else{
        for(auto && e:agentsActed){
            auto fullName=e.first;
            auto agent=e.second.lock();
            if(reward.find(fullName)!=reward.end()){
                reward[fullName]=0;
            }
        }
    }
}
void Reward::onStepEnd(){
    //派生クラスでは最後に必ずこのクラスのonStepEndを呼び出すか、同等の処理を記述すること。
    auto agentsToAct=manager->agentsToAct();
    if(isTeamReward){
        for(auto &&team:target){
            totalReward[team]+=reward[team];
        }
        for(auto && e:agentsToAct){
            auto fullName=e.first;
            auto agent=e.second.lock();
            auto team=agent->getTeam();
            if(reward.find(team)!=reward.end()){
                reward[fullName]=totalReward[team]-totalReward[fullName];
                totalReward[fullName]=totalReward[team];
            }
        }
    }else{
        for(auto && e:agentsToAct){
            auto fullName=e.first;
            auto agent=e.second.lock();
            if(reward.find(fullName)!=reward.end()){
                totalReward[fullName]+=reward[fullName];
            }
        }
    }
}
void Reward::setupTarget(const std::string& query){
    std::regex re;
    if(query=="All"){
        //まずAgent単位の報酬は常に保持する。
        for(auto &&e:manager->getAgents()){
            auto agent=getShared(e);
            std::string agentFullName=agent->getFullName();
            if(reward.find(agentFullName)==reward.end()){
                if(!isTeamReward){
                    target.push_back(agentFullName);
                }
                reward[agentFullName]=0.0;
                totalReward[agentFullName]=0.0;
            }
        }
        //TeamRewardの場合は追加で陣営単位の報酬も保持する。
        if(isTeamReward){
            for(auto &&team:manager->getTeams()){
                if(reward.find(team)==reward.end()){
                    target.push_back(team);
                    reward[team]=0.0;
                    totalReward[team]=0.0;
                }
            }
        }
    }else{
        if(query.find("Team:")==0){
            re=std::regex(query.substr(5));
            //まずAgent単位の報酬は常に保持する。
            for(auto &&e:manager->getAgents()){
                auto agent=getShared(e);
                std::string team=agent->getTeam();
                std::string agentFullName=agent->getFullName();
                if(std::regex_match(team,re) && reward.find(agentFullName)==reward.end()){
                    if(!isTeamReward){
                        target.push_back(agentFullName);
                    }
                    reward[agentFullName]=0.0;
                    totalReward[agentFullName]=0.0;
                }
            }
            //TeamRewardの場合は追加で陣営単位の報酬も保持する。
            if(isTeamReward){
                for(auto &&team:manager->getTeams()){
                    if(std::regex_match(team,re) && reward.find(team)==reward.end()){
                        target.push_back(team);
                        reward[team]=0.0;
                        totalReward[team]=0.0;
                    }
                }
            }
        }else if(query.find("Agent:")==0){
            re=std::regex(query.substr(6));
            //まずAgent単位の報酬は常に保持する。
            for(auto &&e:manager->getAgents()){
                auto agent=getShared(e);
                std::string agentFullName=agent->getFullName();
                if(std::regex_match(agentFullName,re) && reward.find(agentFullName)==reward.end()){
                    if(!isTeamReward){
                        target.push_back(agentFullName);
                    }
                    reward[agentFullName]=0.0;
                    totalReward[agentFullName]=0.0;
                }
            }
            //TeamRewardの場合は追加で陣営単位の報酬も保持する。
            if(isTeamReward){
                for(auto &&e:manager->getAgents()){
                    auto agent=getShared(e);
                    std::string team=agent->getTeam();
                    std::string agentFullName=agent->getFullName();
                    if(std::regex_match(agentFullName,re) && reward.find(team)==reward.end()){
                        target.push_back(team);
                        reward[team]=0.0;
                        totalReward[team]=0.0;
                    }
                }
            }
        }else{
            throw std::runtime_error("Invalid reward target designation. Use either \"All\", \"Team:REGEX_PATTERN\", or \"Agent:REGEX_PATTERN\". The given query is \""+query+"\"");
        }
    }
}
double Reward::getReward(const std::string &key){
    if(reward.count(key)>0){
        return reward[key];
    }else{
        return 0.0;
    }
}
double Reward::getTotalReward(const std::string &key){
    if(totalReward.count(key)>0){
        return totalReward[key];
    }else{
        return 0.0;
    }
}
double Reward::getReward(const std::shared_ptr<Agent> key){
    return Reward::getReward(key->getFullName());
}
double Reward::getTotalReward(const std::shared_ptr<Agent> key){
    return Reward::getTotalReward(key->getFullName());
}
AgentReward::AgentReward(const nl::json& modelConfig_,const nl::json& instanceConfig_)
:Reward(modelConfig_,instanceConfig_){
    if(isDummy){return;}
    isTeamReward=false;
}
AgentReward::~AgentReward(){}

TeamReward::TeamReward(const nl::json& modelConfig_,const nl::json& instanceConfig_)
:Reward(modelConfig_,instanceConfig_){
    if(isDummy){return;}
    isTeamReward=true;
}
TeamReward::~TeamReward(){}

ScoreReward::ScoreReward(const nl::json& modelConfig_,const nl::json& instanceConfig_)
:TeamReward(modelConfig_,instanceConfig_){
    if(isDummy){return;}
}
ScoreReward::~ScoreReward(){}
void ScoreReward::onStepEnd(){
    for(auto &&team:target){
        reward[team]=manager->getRuler().lock()->getStepScore(team);
        totalReward[team]+=reward[team];
    }
    this->TeamReward::onStepEnd();
}

void exportReward(py::module &m)
{
    using namespace pybind11::literals;
    EXPOSE_CLASS(Reward)
    DEF_FUNC(Reward,onEpisodeBegin)
    DEF_FUNC(Reward,onStepBegin)
    DEF_FUNC(Reward,onStepEnd)
    DEF_FUNC(Reward,setupTarget)
    .def("getReward",py::overload_cast<const std::string&>(&Reward::getReward))
    .def("getReward",py::overload_cast<const std::shared_ptr<Agent>>(&Reward::getReward))
    .def("getTotalReward",py::overload_cast<const std::string&>(&Reward::getTotalReward))
    .def("getTotalReward",py::overload_cast<const std::shared_ptr<Agent>>(&Reward::getTotalReward))
    .def_property("j_target",[](const Reward& v){return v.j_target;},[](Reward& v,const py::object& obj){v.j_target=obj;})
    DEF_READWRITE(Reward,target)
    DEF_READWRITE(Reward,reward)
    DEF_READWRITE(Reward,totalReward)
    ;
    EXPOSE_CLASS(AgentReward)
    ;
    EXPOSE_CLASS(TeamReward)
    ;
    EXPOSE_CLASS(ScoreReward)
    DEF_FUNC(ScoreReward,onStepEnd)
    ;
}
