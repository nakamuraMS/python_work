# Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
import sys
import time
import json
import copy
import numpy as np
import ray
from ray.rllib.env.env_context import EnvContext
from ray._private.ray_constants import NODE_DEFAULT_IP
import OriginalModelSample #Factoryへの登録が必要なものは直接使用せずともインポートしておく必要がある
from ASRCAISim1.libCore import Fighter
from ASRCAISim1.addons.rayUtility.RayManager import RayManager

def initialStateReplacer(config, lower, upper, teams, number):
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
		"pos":[l+(h-l)*np.random.rand() for l,h in zip(lower["pos"],upper["pos"])],
		"vel":lower["vel"]+(upper["vel"]-lower["vel"])*np.random.rand(),
		"heading": lower["heading"]+(upper["heading"]-lower["heading"])*np.random.rand(),
	} for i in range(number)]
	states={
		teams[0]: state,
		teams[1]: [invert(s) for s in state],
	}

	ret=copy.deepcopy(config)
	assetConfigDispatcher=ret["AssetConfigDispatcher"]
	for team in teams:
		for i in range(number):
			assetConfigDispatcher[team+"InitialState"]["elements"][i]["value"]["instanceConfig"]=states[team][i]
	return ret

@ray.remote
def worker(config,worker_index):
	from ASRCAISim1.addons.rayUtility.RayManager import RayManager
	import OriginalModelSample #Factoryへの登録のためにこのファイルで直接使用せずとも必須
	env=RayManager(EnvContext(
		config["env_config"],
		worker_index,
		0
	))
	for i in range(config["num_episodes_per_worker"]):
		startT=time.time()
		if "initial_state_lower" in config and "initial_state_upper" in config:
			lower=config["initial_state_lower"]
			upper=config["initial_state_upper"]
			teams=env.manager.getTeams()
			numAssets=len(list(env.manager.getAssets(lambda a: isinstance(a,Fighter) and a.getTeam()==teams[0]))) #同数を仮定
			env.requestReconfigure(initialStateReplacer(env.manager.getManagerConfig()(), lower, upper, teams, numAssets),{})
		if i==0:
			ob,info=env.reset(seed=config["seed"]+worker_index)
		else:
			ob,info=env.reset()
		action_space=env.get_action_space()
		action=action_space.sample()
		dones={"__all__":False}
		while not dones["__all__"]:
			ob, reward, terminateds, truncateds, info = env.step(action)
			dones={k: terminateds[k] or truncateds[k] for k in terminateds}
		endT=time.time()
		print("Episode(",worker_index,",",i+1,") ended in ",endT-startT," seconds.")
	return True
class ExpertTrajectoryGatherer:
	def __init__(self,config):
		self.config=json.load(open(config,'r'))
		#ダミー環境を一度生成し、完全な形のenv_configを取得する。
		dummyEnv=RayManager(EnvContext(
			self.config["env_config"],
			-1,
			-1
		))
		self.completeEnvConfig={
			key:value for key,value in self.config["env_config"].items() if key!="config"
		}
		self.completeEnvConfig["config"]={
				"Manager":dummyEnv.getManagerConfig()(),
				"Factory":dummyEnv.getFactoryModelConfig()()
			}
		self.config["env_config"]=self.completeEnvConfig
		writerConfig={
			"class":"ExpertTrajectoryWriter",
			"config":{
				"prefix":self.config["save_dir"]+("" if self.config["save_dir"][-1]=="/" else "/"),
				"_enable_rl_module_api": self.config.get("_enable_rl_module_api", False),
				"_disable_preprocessor_api": self.config.get("_disable_preprocessor_api", False),
				"preprocessor_options": self.config.get("preprocessor_options", None),
			}
		}
		if("Loggers" in self.config["env_config"]["config"]["Manager"]):
			self.config["env_config"]["config"]["Manager"]["Loggers"]["ExpertTrajectoryWriter"]=writerConfig
		else:
			self.config["env_config"]["config"]["Manager"]["Loggers"]={"ExpertTrajectoryWriter":writerConfig}
	def run(self):
		if(not ray.is_initialized()):
			#rayの初期化がされていない場合、ここで初期化する。
			#既存のRay Clusterがあればconfigに基づき接続し、既存のClusterに接続できなければ、新たにClusterを立ち上げる。
			#強制的に新しいClusterを立ち上げる場合は、"head_ip_address"にnull(None)を指定する。
			#既存のClusterに接続する場合は、"head_ip_address"にHead nodeのIPアドレスとポートを指定する。rayの機能で自動的に接続先を探す場合は"auto"を指定する。
			head_ip_address=self.config.get("head_ip_address","auto")
			entrypoint_ip_address=self.config.get("entrypoint_ip_address",NODE_DEFAULT_IP)
			try:
				ray.init(address=head_ip_address,_node_ip_address=entrypoint_ip_address)
			except:
				print("Warning: Failed to init with the given head_ip_address. A new cluster will be launched instead.")
				ray.init(_node_ip_address=entrypoint_ip_address)
		import signal
		original=signal.getsignal(signal.SIGINT)
		res=[worker.remote(self.config,i) for i in range(self.config["num_workers"])]
		def sig_handler(sig,frame):
			for r in res:
				ray.kill(r)
			signal.signal(signal.SIGINT,original)
		signal.signal(signal.SIGINT,sig_handler)
		ray.get(res)
		return res
		

if __name__ == "__main__":
	if(len(sys.argv)>1):
		gatherer=ExpertTrajectoryGatherer(sys.argv[1])
		gatherer.run()
