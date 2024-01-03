// Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
#define MY_MODULE_NAME libOriginalModelSample
#include <ASRCAISim1/Factory.h>
#include "R5AgentSample01S.h"
#include "R5AgentSample01M.h"
#include "R5RewardSample01.h"
#include "R5RewardSample02.h"
#include <iostream>
namespace py=pybind11;


PYBIND11_MODULE(MY_MODULE_NAME,m)
{    
    using namespace pybind11::literals;
    m.doc()="OriginalModelSample";
    exportR5AgentSample01S(m);
    exportR5AgentSample01M(m);
    exportR5RewardSample01(m);
    exportR5RewardSample02(m);
    FACTORY_ADD_CLASS(Agent,R5AgentSample01S)
    FACTORY_ADD_CLASS(Agent,R5AgentSample01M)
    FACTORY_ADD_CLASS(Reward,R5RewardSample01)
    FACTORY_ADD_CLASS(Reward,R5RewardSample02)
}
