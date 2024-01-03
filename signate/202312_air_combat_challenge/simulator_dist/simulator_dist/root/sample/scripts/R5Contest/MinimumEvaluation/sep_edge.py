# Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
import os
import time
import importlib
import ray
from ASRCAISim1.libCore import Factory
from ASRCAISim1.common import addPythonClass
from ASRCAISim1.GymManager import GymManager,SimpleEvaluator
from ASRCAISim1.addons.AgentIsolation import AgentDelegatee,PolicyDelegatee,SimulationManagerForIsolation

def preparation(server,port):
    import socket
    import pickle
    #対戦開始まで待機
    s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    s.bind((server,port))
    s.listen(1)
    isRunning=True
    bufferSize=4096
    userModelID=None
    userModuleID=None
    userArgs=None
    factoryConfig=None
    while isRunning:
        conn,addr=s.accept()
        header=conn.recv(bufferSize).decode("utf-8")
        msgLen=int(header[7:])
        conn.send("ACK:OK".encode("utf-8"))
        received=0
        msg=b""
        while received<msgLen:
            part=conn.recv(bufferSize)
            received+=len(part)
            if(len(part)>0):
                msg+=part
        assert(received==msgLen)
        _,command,data=pickle.loads(msg)
        if(command=="userModelID"):
            userModelID,userModuleID,userArgs=data
            ret=pickle.dumps(None)
        elif(command=="factoryConfig"):
            factoryConfig=data
            factoryConfig.pop("Agent")
            ret=pickle.dumps(None)
        elif(command=="ready"):
            isRunning=False
            ret=ret=pickle.dumps(None)
        elif(command=="kill"):
            userModelID=None
            userModuleID=None
            isRunning=False
            ret=ret=pickle.dumps(None)
        else:
            raise ValueError("Unknown command. command=",command)
        header="HEADER:{:16d}".format(len(ret)).encode("utf-8")
        conn.send(header)
        ack=conn.recv(bufferSize).decode("utf-8")
        assert(ack[4:]=="OK")
        conn.send(ret)
        conn.close()
    s.close()
    return userModelID,userModuleID,userArgs,factoryConfig

def agentServer(server,port):
    importedUserModules={}
    while True:
        userModelID,userModuleID,userArgs,factoryConfig=preparation(server,port)
        if(userModelID is None or userModuleID is None):
            return
        if(not userModuleID in importedUserModules):
            try:
                userModule = importlib.import_module(userModuleID)
                assert hasattr(userModule, "getUserAgentClass")
                assert hasattr(userModule, "getUserAgentModelConfig")
                assert hasattr(userModule, "isUserAgentSingleAsset")
                assert hasattr(userModule, "getUserPolicy")
            except Exception as e:
                raise e  # 読み込み失敗時の扱いは要検討
            importedUserModules[userModuleID]=userModule
        userModule=importedUserModules[userModuleID]
        userAgentClass = userModule.getUserAgentClass(userArgs)
        addPythonClass("Agent", "Agent_"+userModelID, userAgentClass)
        #コンフィグの生成
        factoryConfig["Agent"]={
            "Agent_"+userModelID:{
                "class":"Agent_"+userModelID,
                "config":userModule.getUserAgentModelConfig(userArgs)
            }
        }
        manager=SimulationManagerForIsolation(factoryConfig)
        delegatee=AgentDelegatee(manager,server,port)
        print("=====Agent class=====")
        print("Agent_"+userModelID," = ",userAgentClass)
        delegatee.run()

def policyServer(server,port):
    importedUserModules={}
    while True:
        userModelID,userModuleID,userArgs,factoryConfig=preparation(server,port)
        if(userModelID is None or userModuleID is None):
            return
        if(not userModuleID in importedUserModules):
            try:
                userModule = importlib.import_module(userModuleID)
                assert hasattr(userModule, "getUserAgentClass")
                assert hasattr(userModule, "getUserAgentModelConfig")
                assert hasattr(userModule, "isUserAgentSingleAsset")
                assert hasattr(userModule, "getUserPolicy")
            except Exception as e:
                raise e  # 読み込み失敗時の扱いは要検討
            importedUserModules[userModuleID]=userModule
        userModule=importedUserModules[userModuleID]
        policies={
            "Policy_"+userModelID:userModule.getUserPolicy(userArgs)
        }
        print("=====Policy=====")
        for name,policy in policies.items():
            print(name," = ",type(policy))
        delegatee=PolicyDelegatee(policies,server,port)
        delegatee.run()

def run(sep_config):
    server=sep_config["server"]
    agentPort=sep_config["agentPort"]
    policyPort=sep_config["policyPort"]
    import multiprocessing
    ctx=multiprocessing.get_context("spawn")
    agentProcess=ctx.Process(target=agentServer,args=(server,agentPort))
    policyProcess=ctx.Process(target=policyServer,args=(server,policyPort))
    agentProcess.start()
    policyProcess.start()
    agentProcess.join()
    policyProcess.join()

if __name__ == "__main__":
    import sys,json
    sep_config=json.load(open("sep_config.json","r"))
    assert(len(sys.argv)>1 and sys.argv[1] in ["Blue","Red"])
    run(sep_config[sys.argv[1]])