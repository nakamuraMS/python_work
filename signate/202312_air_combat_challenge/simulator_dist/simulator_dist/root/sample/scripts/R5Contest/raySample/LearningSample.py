# Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
"""RayLeagueLearnerを用いて分散学習を行うためのサンプルスクリプト。
RayLeagueLearnerのコンフィグの大半をコマンドライン引数でjsonファイルを指定することで設定し、
一部のjson化しにくい部分を本スクリプトで設定している。
以下のようにコマンドライン引数としてjsonファイルを与えることで学習が行われる。
python LearningSample.py config.json
"""
import sys
import json
import ray
from ray.rllib.env.env_context import EnvContext
from ASRCAISim1.addons.rayUtility.RayManager import RayManager
from RayLeagueLearner import RayLeagueLearner
from ASRCAISim1.addons.MatchMaker.TwoTeamCombatMatchMaker import TwoTeamCombatMatchMaker, TwoTeamCombatMatchMonitor, wrapEnvForTwoTeamCombatMatchMaking
import OriginalModelSample #Factoryへの登録が必要なものは直接使用せずともインポートしておく必要がある
from OriginalModelSample.R5TorchNNSampleForRay import R5TorchNNSampleForRay
from ASRCAISim1.addons.rayUtility.extension.policy import DummyInternalRayPolicy
from ray.rllib.algorithms.impala import Impala, ImpalaTF1Policy, ImpalaTF2Policy, ImpalaTorchPolicy
from ray.rllib.algorithms.appo import APPO, APPOTF1Policy, APPOTF2Policy, APPOTorchPolicy

class DummyInternalRayPolicyForAPPO(DummyInternalRayPolicy):
	#APPOの場合、self.update_targetとして引数なしメソッドを持っていなければならない。
	#
	def __init__(self,observation_space,action_space,config):
		super().__init__(observation_space,action_space,config)
		def do_update_dummy():
			return
		self.update_target=do_update_dummy


availableTrainers={
	"IMPALA":{
		"trainer":Impala,
		"tf":ImpalaTF1Policy,
		"tf2":ImpalaTF2Policy,
		"torch":ImpalaTorchPolicy
	},
	"APPO":{
		"trainer":APPO,
		"tf":APPOTF1Policy,
		"tf2":APPOTF2Policy,
		"torch":APPOTorchPolicy,
		"internal":DummyInternalRayPolicyForAPPO
	},
}
matchMakerClasses={
	"TwoTeamCombatMatchMaker": TwoTeamCombatMatchMaker,
}
matchMonitorClasses={
	"TwoTeamCombatMatchMonitor": TwoTeamCombatMatchMonitor,
}

def envCreator(config: EnvContext):
	import OriginalModelSample #Factoryへの登録が必要なものは直接使用せずともインポートしておく必要がある
	return wrapEnvForTwoTeamCombatMatchMaking(RayManager)(config)

if __name__ == "__main__":
	if(len(sys.argv)>1):
		config=json.load(open(sys.argv[1],'r'))
		#既存のRay Clusterに接続する場合は以下を指定。(jsonで指定してもよい)
		if(not "head_ip_address" in config):
			config["head_ip_address"]="auto"
		#本スクリプトを実行するノードがClusterに接続する際のIPアドレスがデフォルト値("127.0.0.1")でない場合、以下で指定。(jsonで指定してもよい)
		if(not "entrypoint_ip_address" in config):
			config["entrypoint_ip_address"]="127.0.0.1"
		config["envCreator"]=envCreator
		learner=RayLeagueLearner(
			config,
			availableTrainers,
			matchMakerClasses.get(config.get("match_maker_class","TwoTeamCombatMatchMaker")),
			matchMonitorClasses.get(config.get("match_monitor_class","TwoTeamCombatMatchMonitor")),
			teams=config.get("match_maker",{}).get("teams",["Blue","Red"])
		)
		learner.run()

