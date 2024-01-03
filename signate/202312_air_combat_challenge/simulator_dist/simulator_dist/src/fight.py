from __future__ import print_function
import os
import json
import time
import copy
import random
import importlib
import numpy as np
import gymnasium as gym
from timeout_decorator import timeout, TimeoutError

from ASRCAISim1.libCore import Factory, Fighter
from ASRCAISim1.common import addPythonClass
from ASRCAISim1.GymManager import GymManager


def map_r(x, callback_fn=None):
    # recursive map function
    if isinstance(x, (list, tuple, set)):
        return type(x)(map_r(xx, callback_fn) for xx in x)
    elif isinstance(x, dict):
        return type(x)((key, map_r(xx, callback_fn)) for key, xx in x.items())
    return callback_fn(x) if callback_fn is not None else None


def initialStateReplacer1(config, teams, number, rand):
    #一定領域内でランダムかつ点対称な初期条件を生成する。
    def keepAngle(a):
        ret=a
        while ret>=360.0:
            ret-=360.0
        while ret<=-360.0:
            ret+=360.0
        return ret
    def invert(src):
        dst={
            "pos":[-src["pos"][0],-src["pos"][1],src["pos"][2]],
            "vel":src["vel"],
            "heading":keepAngle(src["heading"]+180.0)
        }
        return dst

    state=[{
        #teams[0](=Blue)側の初期条件の範囲
        "pos":gym.spaces.Box(low=np.array(rand['pos'][0]),high=np.array(rand['pos'][1]),dtype=np.float64).sample().tolist(),
        "vel":random.uniform(rand['vel'][0], rand['vel'][1]),
        "heading":random.uniform(rand['heading'][0], rand['heading'][1]),
    } for i in range(number)]
    states={
        "Blue": state,
        "Red": [invert(s) for s in state],
    }

    ret=copy.deepcopy(config)
    assetConfigDispatcher=ret["AssetConfigDispatcher"]
    for team in teams:
        for i in range(number):
            assetConfigDispatcher[team+"InitialState"]["elements"][i]["value"]["instanceConfig"]=states[team][i]
    return ret


def agentConfigMaker(team: str, userModelID: str, isSingle: bool, number: int) -> dict:
    # 青と赤のAgent指定部分のコンフィグを生成
    if(isSingle):
        # 1個のインスタンスで1機なので異なるnameを割り当てることで2個のインスタンスとし、portはどちらも"0"
        return {
            "type": "group",
            "order": "fixed",
            "elements": [
                {"type": "External", "model": "Agent_"+userModelID, 
                    "policy": "Policy_"+userModelID, "name": team+"_"+userModelID+"_"+str(i+1), "port": "0"} for i in range(number)
            ]
        }
    else:
        # 1個のインスタンスで2機分なので同じnameを割り当てることで1個のインスタンスとし、それぞれ異なるport("0"と"1")を割り当てる
        return {
            "type": "group",
            "order": "fixed",
            "elements": [
                {"type": "External", "model": "Agent_"+userModelID,
                    "policy": "Policy_"+userModelID, "name": team+"_"+userModelID, "port": str(i)} for i in range(number)
            ]
        }


def policyMapper(fullName):
    agentName, modelName, policyName = fullName.split(":")
    return policyName


def wrap_get_action_from_obs(args, time_out):
    @timeout(time_out)
    def get_action_from_obs(policies, rewards, terminateds, truncateds, infos, observation_space, action_space, policyMapper, obs, team):
        actions = {}
        for k, o in obs.items():
            suff_k = k.split('_')[0]
            if policyMapper(k) in policies:
                if team in suff_k:
                    actions[k] = policies[policyMapper(k)].step(o, rewards[k], terminateds[k] or truncateds[k], infos[k], k, observation_space[k], action_space[k])
        return actions
    
    return get_action_from_obs(*args)


def step_foward(func, args, time_out):
    """
    func: decorated with timeout
    """
    try:
        r = func(args, time_out)
        return r
    except TimeoutError:
        print('TimeOut')
        return args[6].sample()


def get_ticks(env, num_assets):
    ticks = {}
    red_fighter_names = [a().getFullName() for a in env.manager.getAgents() if 'Red' in a().getFullName()]
    blue_fighter_names = [a().getFullName() for a in env.manager.getAgents() if 'Blue' in a().getFullName()]
    ticks['Red'] = get_stepcount(env, red_fighter_names, num_assets)
    ticks['Blue'] = get_stepcount(env, blue_fighter_names, num_assets)

    return ticks


def get_stepcount(env, fighter_names, num_assets):
    tick = {}
    if len(fighter_names)>1:
        for fighter_name in fighter_names:
            agentName, modelName, policyName = fighter_name.split(":")
            team = agentName.split('_')[0]
            port = agentName.split('_')[-1]
            tick[int(port)] = env.manager.getAgentStepCount(fighter_name)
    elif len(fighter_names) == 1:
        for i in range(num_assets):
            tick[i+1] = env.manager.getAgentStepCount(fighter_names[0])
    return tick


def get_data(data, fighter_names, env):
    ptime = env.manager.getTime()
    data_f = {}
    i = 1
    for k, v in data.items():
        if k in fighter_names:
            data_f[i] = v
            i+=1
    out = {'ptime':ptime, 'data':data_f}
    n_data = len(data_f)
    return out, n_data


def update_logs(red_fighter_names, blue_fighter_names, logs, env, obs, infos, red_actions, blue_actions):
    def conv(src):
        return map_r(src, lambda d: d.tolist() if isinstance(d, np.ndarray) else d)

    a, n_data = get_data(conv(red_actions), red_fighter_names, env)
    if n_data:
        logs['Red']['actions'].append(a)
    o, n_data = get_data(conv(obs), red_fighter_names, env)
    if n_data:
        logs['Red']['observations'].append(o)
    info, n_data = get_data(infos, red_fighter_names, env)
    if n_data:
        logs['Red']['infos'].append(info)

    a, n_data = get_data(conv(blue_actions), blue_fighter_names, env)
    if n_data:
        logs['Blue']['actions'].append(a)
    o, n_data = get_data(conv(obs), blue_fighter_names, env)
    if n_data:
        logs['Blue']['observations'].append(o)
    info, n_data = get_data(infos, blue_fighter_names, env)
    if n_data:
        logs['Blue']['infos'].append(info)


def fight(matchInfo):
    seed=matchInfo.get("seed",None)
    if(seed is None):
        seed = np.random.randint(2**31)
    numAssets=2
    logName="_vs_".join([info.get("logName",info["userModelID"]) for info in matchInfo["teams"].values()])

    # ユーザーモジュールの読み込み
    importedUserModules={}
    for team,info in matchInfo["teams"].items():
        userModelID=info["userModelID"]
        userModuleID=info["userModuleID"]
        args=info.get("args",{})
        #args["userModelID"]=userModelID
        info["args"]=args
        if userModuleID not in importedUserModules:
            try:
                module = importlib.import_module(userModuleID)
                assert hasattr(module, "getUserAgentClass")
                assert hasattr(module, "getUserAgentModelConfig")
                assert hasattr(module, "isUserAgentSingleAsset")
                assert hasattr(module, "getUserPolicy")
            except Exception as e:
                raise e  # 読み込み失敗時の扱いは要検討
            importedUserModules[userModuleID]=module
        module=importedUserModules[userModuleID]
        agentClass = module.getUserAgentClass(args)
        if not matchInfo['registered'][team]:
            addPythonClass("Agent", "Agent_"+userModelID, agentClass)
            Factory.addDefaultModel(
                "Agent",
                "Agent_"+userModelID,
                {
                    "class": "Agent_"+userModelID,
                    "config": module.getUserAgentModelConfig(args)
                }
            )
        matchInfo['registered'][team] = True

    # コンフィグの生成
    agentConfig = {
        "Manager": {
            "AgentConfigDispatcher": {
                team+"Agents": agentConfigMaker(team,info["userModelID"], importedUserModules[info["userModuleID"]].isUserAgentSingleAsset(info["args"]),numAssets)
                for team,info in matchInfo["teams"].items()
            }
        }
    }
    loggerConfig = {}
    if "log_dir" in matchInfo:
        loggerConfig = {
            "MultiEpisodeLogger":{
                "class":"MultiEpisodeLogger",
                "config":{
                    "prefix":os.path.join(matchInfo["log_dir"],logName),
                    "episodeInterval":1,
                    "ratingDenominator":100
                }
            }
        }
        if matchInfo["replay"]:
            loggerConfig["GUIStateLogger"]={
                "class":"GUIStateLogger",
                "config":{
                    "prefix":os.path.join(matchInfo["log_dir"],logName),
                    "episodeInterval":1,
                    "innerInterval":10
                }
            }

    configs = [
        os.path.join(matchInfo["common_dir"], "R5_contest_mission_config.json"),
        os.path.join(matchInfo["common_dir"], "R5_contest_asset_models.json"),
        os.path.join(matchInfo["common_dir"], "R5_contest_agent_ruler_reward_models.json"),
        agentConfig,
        {
            "Manager": {
                "Rewards": [],
                "seed":seed,
                "ViewerType":"God" if matchInfo["visualize"] else "None",
                "Loggers": loggerConfig
            }
        }
    ]
    context = {
        "config": configs,
        "worker_index": 0,
        "vector_index": 0
    }

    # 環境の生成
    env = GymManager(context)

    # StandalonePolicyの生成
    policies = {
        "Policy_"+info["userModelID"]: importedUserModules[info["userModuleID"]].getUserPolicy(info["args"])
        for team,info in matchInfo["teams"].items()
    }


    # 生成状況の確認
    observation_space = env.observation_space
    action_space = env.action_space
    print("=====Agent classes=====")
    for team,info in matchInfo["teams"].items():
        print("Agent_"+info["userModelID"], " = ", importedUserModules[info["userModuleID"]].getUserAgentClass(info["args"]))
    print("=====Policies=====")
    for name, policy in policies.items():
        print(name, " = ", type(policy))
    print("=====Policy Map (at reset)=====")
    for fullName in action_space: # type: ignore
        print(fullName, " -> ", policyMapper(fullName))
    print("=====Agent to Asset map=====")
    for agent in [a() for a in env.manager.getAgents()]:
        print(agent.getFullName(), " -> ", "{")
        for port, parent in agent.parents.items():
            print("  ", port, " : ", parent.getFullName())
        print("}")

    # シミュレーションの実行
    print("=====running simulation(s)=====")
    if matchInfo['control_init']:
        print('Change initial states at random')
        with open(os.path.join(matchInfo["common_dir"], 'initial_states.json')) as f:
            initial_states = json.load(f)
        env.requestReconfigure(initialStateReplacer1(env.manager.getManagerConfig()(), matchInfo["teams"], numAssets, initial_states),{})

    time_out = matchInfo['time_out']
    start_time = time.time()
    obs, info = env.reset()
    rewards = {k: 0.0 for k in obs.keys()}
    terminateds = {k: False for k in obs.keys()}
    truncateds = {k: False for k in obs.keys()}
    infos = {k: None for k in obs.keys()}
    for p in policies.values():
        p.reset()
    terminateds["__all__"]=False
    truncateds["__all__"]=False
    red_error = False
    blue_error = False
    error = False
    logs = {'Red':{'observations': [], 'actions':[], 'infos': []}, 'Blue':{'observations': [], 'actions': [], 'infos': []}}
    red_fighter_names = sorted([a().getFullName() for a in env.manager.getAgents() if 'Red' in a().getFullName()])
    blue_fighter_names = sorted([a().getFullName() for a in env.manager.getAgents() if 'Blue' in a().getFullName()])
    if matchInfo['make_log']:
        print('Making logs')
    while not (terminateds["__all__"] or truncateds["__all__"]):
        #print(p_ticks)
        observation_space = env.get_observation_space()
        action_space = env.get_action_space()

        # get Red action
        red_actions = {}
        try:
            red_actions = step_foward(func = wrap_get_action_from_obs, args=(policies, rewards, terminateds, truncateds, infos, observation_space, action_space, policyMapper, obs, 'Red'), time_out=time_out)
        except Exception as e:
            print('Catched error in Red. {}'.format(e))
            red_error = True

        # get Blue action
        blue_actions = {}
        try:
            blue_actions = step_foward(func = wrap_get_action_from_obs, args=(policies, rewards, terminateds, truncateds, infos, observation_space, action_space, policyMapper, obs, 'Blue'), time_out=time_out)
        except Exception as e:
            print('Catched error in Blue. {}'.format(e))
            blue_error = True

        if red_error or blue_error:
            break
        if matchInfo['make_log']:
            update_logs(red_fighter_names, blue_fighter_names, logs, env, obs, infos, red_actions, blue_actions)
            #print(p_ticks)
        
        actions = dict(red_actions, **blue_actions)

        obs, rewards, terminateds, truncateds, infos = env.step(actions)



    print('Time elapsed: {}[s]'.format(time.time() - start_time))

    if red_error and blue_error:
        winner = ''
        error = True
    elif red_error:
        winner = 'Blue'
        error = True
    elif blue_error:
        winner = 'Red'
        error = True
    else:
        winner = env.manager.getRuler()().winner
    
    detail = {
        "finishedTime":float(env.manager.getTime()),
        "numAlives":{team:float(np.sum([f.isAlive() for f in [f() for f in env.manager.getAssets(lambda a:isinstance(a,Fighter) and a.getTeam()==team)]])) for team in env.manager.getRuler()().score},
        "endReason":env.manager.getRuler()().endReason.name,
        "error": error,
        "logs": logs,
        "config": env.manager.getManagerConfig()() # for testing
    }
    scores = {k: v for k, v in env.manager.scores.items()}
    detail['scores'] = scores
    print("winner=", winner)
    return winner, detail
