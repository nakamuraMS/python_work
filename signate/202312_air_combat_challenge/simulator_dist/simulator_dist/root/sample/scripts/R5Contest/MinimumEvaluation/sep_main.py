# Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
import os
import time
import importlib
import ray
import copy
import gymnasium as gym
import numpy as np
from ASRCAISim1.libCore import SimulationManager
from ASRCAISim1.common import addPythonClass
from ASRCAISim1.GymManager import GymManager,SimpleEvaluator
from ASRCAISim1.addons.AgentIsolation import PolicyDelegator

def agentConfigMaker(team: str, userModelID: str, isSingle: bool, number: int) -> dict:
    # 青と赤のAgent指定部分のコンフィグを生成
    if(isSingle):
        # 1個のインスタンスで1機なので異なるnameを割り当てることで2個のインスタンスとし、portはどちらも"0"
        return {
            "type": "group",
            "order": "fixed",
            "elements": [
                {"type": "External", "model": "Agent_"+userModelID, "policy": "Policy_" +
                    userModelID, "name": team+"_"+userModelID+"_"+str(i+1), "port": "0"} for i in range(number)
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

def initialStateReplacer1(config, teams, number):
    #一定領域内でランダムかつ点対称な初期条件を生成する。
    def invert(src):
        def keepAngle(a):
            ret=a
            while ret>=360.0:
                ret-=360.0
            while ret<=-360.0:
                ret+=360.0
            return ret
        dst={
            "pos":[-src["pos"][0],-src["pos"][1],src["pos"][2]],
            "vel":src["vel"],
            "heading":keepAngle(src["heading"]+180.0)
        }
        return dst

    state=[{
        #teams[0](=Blue)側の初期条件の範囲
        "pos":gym.spaces.Box(low=np.array([-10000.0,20000.0,-12000.0]),high=np.array([10000.0,25000.0,-6000.0]),dtype=np.float64).sample().tolist(),
        "vel":260.0+20.0*np.random.rand(),
        "heading": 225.0+90.0*np.random.rand(),
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

def postGlobalCommand(command,data,server,port):
    #終了処理(kill)や次エピソードへの準備(clear)のため
    import socket
    import pickle
    bufferSize=4096
    conn=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    conn.connect((server,port))
    msg=pickle.dumps(["",command,data])
    header="HEADER:{:16d}".format(len(msg)).encode("utf-8")
    conn.send(header)
    ack=conn.recv(bufferSize).decode("utf-8")
    assert(ack[4:]=="OK")
    conn.send(msg)
    header=conn.recv(bufferSize).decode("utf-8")
    msgLen=int(header[7:])
    conn.send("ACK:OK".encode("utf-8"))
    received=0
    ret=b""
    while received<msgLen:
        part=conn.recv(bufferSize)
        received+=len(part)
        if(len(part)>0):
            ret+=part
    assert(received==msgLen)
    conn.close()
    return pickle.loads(ret)

def run(sep_config,matchInfo):
    seed=matchInfo.get("seed",None)
    if(seed is None):
        import numpy as np
        seed = np.random.randint(2**31)
    numAssets=2
    logName="_vs_".join([info.get("logName",info["userModelID"]) for info in matchInfo["teams"].values()])
    
    #edge側にuserModelID等を伝達
    for team,info in matchInfo["teams"].items():
        userModelID=info["userModelID"]
        userModuleID=info["userModuleID"]
        args=info.get("args",{})
        args["userModelID"]=userModelID
        info["args"]=args
        postGlobalCommand("userModelID",(userModelID,userModuleID,args),sep_config[team]["server"],sep_config[team]["agentPort"])
        postGlobalCommand("userModelID",(userModelID,userModuleID,args),sep_config[team]["server"],sep_config[team]["policyPort"])
    #ユーザーモジュールの読み込み
    importedUserModules={}
    for team,info in matchInfo["teams"].items():
        userModelID=info["userModelID"]
        userModuleID=info["userModuleID"]
        args=info["args"]
        if(not userModuleID in importedUserModules):
            try:
                module = importlib.import_module(userModuleID)
                assert hasattr(module, "getUserAgentClass")
                assert hasattr(module, "getUserAgentModelConfig")
                assert hasattr(module, "isUserAgentSingleAsset")
                assert hasattr(module, "getUserPolicy")
            except Exception as e:
                raise e  # 読み込み失敗時の扱いは要検討
            importedUserModules[userModuleID]=module
    
    #コンフィグの生成
    agentConfig={
        "Factory":{
            "Agent":{
                "Agent_"+info["userModelID"]:{
                    "class":"AgentDelegator",
                    "config":{
                        "socketServer":sep_config[team]["server"],
                        "socketPort":sep_config[team]["agentPort"]
                    }
                }
                for team,info in matchInfo["teams"].items()
            }
        },
        "Manager":{
            "AgentConfigDispatcher":{
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

    configs=[
        os.path.join(os.path.dirname(__file__), "common/R5_contest_mission_config.json"),
        os.path.join(os.path.dirname(__file__), "common/R5_contest_asset_models.json"),
        os.path.join(os.path.dirname(__file__), "common/R5_contest_agent_ruler_reward_models.json"),
        agentConfig,
        {
            "Manager":{
                "Rewards": [],
                "seed":seed,
                "ViewerType":"God" if matchInfo["visualize"] else "None",
                "Loggers": loggerConfig if "log_dir" in matchInfo else {}
            }
        }
    ]
    context={
        "config":configs,
        "worker_index":0,
        "vector_index":0
    }
    #edge側にFactoryのconfigを伝達
    fullConfig=SimulationManager.parseConfig(configs)() #末尾に()をつけてnl::json→dictに変換
    for team,info in matchInfo["teams"].items():
        postGlobalCommand("factoryConfig",fullConfig.get("Factory",{}),sep_config[team]["server"],sep_config[team]["agentPort"])
    #edge側の事前準備を完了(AgentDelegatee,PolicyDelegateeの通信待受に移行)
    for team,info in matchInfo["teams"].items():
        postGlobalCommand("ready",None,sep_config[team]["server"],sep_config[team]["agentPort"])
        postGlobalCommand("ready",None,sep_config[team]["server"],sep_config[team]["policyPort"])

    #環境の生成
    env=GymManager(context)
    

    #StandalonePolicyの生成
    policies={
        "Policy_"+info["userModelID"]: importedUserModules[info["userModuleID"]].getUserPolicy(info["args"])
        for team,info in matchInfo["teams"].items()
    }
    #policyMapperの定義(基本はデフォルト通り)
    def policyMapper(fullName):
        agentName,modelName,policyName=fullName.split(":")
        return policyName

    #生成状況の確認
    observation_space=env.observation_space
    action_space=env.action_space
    print("=====Policy Map (at reset)=====")
    for fullName in action_space:
        print(fullName," -> ",policyMapper(fullName))
    print("=====Agent to Asset map=====")
    for agent in [a() for a in env.manager.getAgents()]:
        print(agent.getFullName(), " -> ","{")
        for port,parent in agent.parents.items():
            print("  ",port," : ",parent.getFullName())
        print("}")
    
    #シミュレーションの実行
    print("=====running simulation(s)=====")
    numEpisodes = matchInfo["num_episodes"]
    for episodeCount in range(numEpisodes):
        if matchInfo["initial_state"]=="random":
            env.requestReconfigure(initialStateReplacer1(env.manager.getManagerConfig()(), matchInfo["teams"], numAssets),{})
        obs, info=env.reset()
        rewards={k:0.0 for k in obs.keys()}
        terminateds = {k: False for k in obs.keys()}
        truncateds = {k: False for k in obs.keys()}
        infos={k:None for k in obs.keys()}
        for p in policies.values():
            p.reset()
        terminateds["__all__"]=False
        truncateds["__all__"]=False
        while not (terminateds["__all__"] or truncateds["__all__"]):
            observation_space=env.get_observation_space()
            action_space=env.get_action_space()
            actions={k:policies[policyMapper(k)].step(
                o,
                rewards[k],
                terminateds[k] or truncateds[k],
                infos[k],
                k,
                observation_space[k],
                action_space[k]
            ) for k,o in obs.items() if policyMapper(k) in policies}
            obs, rewards, terminateds, truncateds, infos = env.step(actions)
        for team,info in matchInfo["teams"].items():
            postGlobalCommand("clear",None,sep_config[team]["server"],sep_config[team]["agentPort"])
        print("episode(",episodeCount+1,"/",numEpisodes,"), winner=",env.manager.getRuler()().winner,", scores=",{k:v for k,v in env.manager.scores.items()})
    #終了処理
    for team,info in matchInfo["teams"].items():
        postGlobalCommand("kill",None,sep_config[team]["server"],sep_config[team]["agentPort"])
        postGlobalCommand("kill",None,sep_config[team]["server"],sep_config[team]["policyPort"])


if __name__ == "__main__":
    import json
    import argparse
    candidates=json.load(open("candidates.json","r"))
    parser=argparse.ArgumentParser()
    parser.add_argument("Blue",type=str,help="name of Blue team")
    parser.add_argument("Red",type=str,help="name of Red team")
    parser.add_argument("-n","--num_episodes",type=int,default=1,help="number of evaluation episodes")
    parser.add_argument("-l","--log_dir",type=str,help="log directory")
    parser.add_argument("-r", "--replay",action="store_true",help="use when you want to record episodes for later visualization")
    parser.add_argument("-v","--visualize",action="store_true",help="use when you want to visualize episodes")
    parser.add_argument("-i","--initial_state",type=str,default="fixed",help="initial state condition (random or fixed)")
    args=parser.parse_args()
    assert(args.Blue in candidates and args.Red in candidates)
    matchInfo={
        "teams":{
            "Blue":{"userModelID":args.Blue},
            "Red":{"userModelID":args.Red},
        },
        "num_episodes":args.num_episodes,
        "replay":args.replay,
        "visualize":args.visualize,
        "initial_state":args.initial_state,
    }
    for team,value in matchInfo["teams"].items():
        matchInfo["teams"][team].update(candidates[value["userModelID"]])
    if(args.log_dir is not None):
        matchInfo["log_dir"]=args.log_dir
    sep_config=json.load(open("sep_config.json","r"))
    run(sep_config,matchInfo)
