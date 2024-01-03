# Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
import os
import time
import datetime
import numpy as np
import gymnasium as gym
from collections import defaultdict
import copy
import cloudpickle
from ASRCAISim1.libCore import Fighter
from ASRCAISim1.addons.MatchMaker.MatchMaker import MatchMaker,MatchMonitor
from ASRCAISim1.addons.MatchMaker.PayOff import PayOff

"""2陣営による戦闘を対象とした基本的なMatchMaker/MatchMonitorのサンプルクラス。
<使用時の前提>
・陣営名は"Blue"と"Red"を基準とする。SimulationManagerのRulerの設定もこれに合わせること。
・学習対象のPolicyは"Learner"とし、初期行動判断モデルは"Initial"とする。
　SimulationManagerのAgentConfigDispatcherはサンプルに示すような形式で各Policy名に対応するaliasを登録しておくこと。
・一つの陣営を操作するPolicyは一種類とする。つまり、SimulationManagerの対応するAgentも一種類となる。
・各Policyは1体で1陣営分を動かしても、1体で1機を動かしてもよく、対戦カードにはその設定を表すbool型の"MultiPort"キーを追加している。
・使用する環境クラス(GymManager又はその派生クラス)をwrapEnvForTwoTeamCombatMatchMakingでラップしたものを使用し、
　学習プログラムにおいてenv.setMatch関数を呼び出して対戦カードの反映を行うこと。
・対戦カードのSuffixは、学習中重みの場合は""(空文字列)、過去の重みをBlue側で使用する場合は"_Past_Blue"、過去の重みをRed側で使用する場合は"_Past_Red"とする。
　これらのSuffixを付加した名称に対応するPolicyインスタンスを学習プログラム側で用意すること。
<ログの記録>
このサンプルでは対戦ログの記録についても実装例を示している。
学習の進捗チェック用に、AlphaStar[1]で用いられているPayOffとイロレーティングを算出しており、
それらをget_metricsでTensorboardログとして使用できる形式で出力するとともに、各エピソードの結果をcsvファイルに出力する。
[1] Vinyals, Oriol, et al. "Grandmaster level in StarCraft II using multi-agent reinforcement learning." Nature 575.7782 (2019): 350-354.
<対戦カードの生成方式>
・Blue側：学習中の"Learner"又は一定確率(expert_ratio)で教師役の"Expert"
・Red側：一定エピソード数(warm_up_episodes)の経過前は常に初期行動判断モデル("Initial")とし、
    経過後は(1)学習中重みの直近のコピー、(2)初期行動判断モデル("Initial")、(3)過去の保存された重みから一様分布で選択
    の3種類を4:4:2の割合で選択
<configの書式>
config={
    # 基底クラスで指定されたもの
    "restore": None or str, #チェックポイントを読み込む場合、そのパスを指定。
    "weight_pool": str, #重みの保存先ディレクトリを指定。<policy>-<id>.datのようにPolicy名と重み番号を示すファイル名で保存される。
    "policy_config": { #Policyに関する設定。Policy名をキーとしたdictで与える。
        <Policy's name>: {
            "multi_port": bool, #1体で1陣営分を動かすタイプのPolicyか否か。デフォルトはFalse。
            "active_limit": int, #保存された過去の重みを使用する数の上限を指定する。
            "is_internal": bool, #SimulationManagerクラスにおけるInternalなPolicyかどうか。
            "populate": None or { #重み保存条件の指定
                "firstPopulation": int, # 初回の保存を行うエピソード数。0以下の値を指定すると一切保存しない。
                "interval": int, # 保存間隔。0以下の値を指定すると一切保存しない。
                "on_start": bool, # 開始時の初期重みで保存するかどうか。省略時はFalseとなる。
                "reset"; float, #重み保存時の重みリセット確率(0〜1)
            },
            "rating_initial": float, #初期レーティング
            "rating_fixed": bool, #レーティングを固定するかどうか。
            "initial_weight": None or str, #初期重み(リセット時も含む)のパス。
        },
        ...
    },
    "match_config": {#対戦カードの生成に関する指定。このサンプルでは以下のキーを指定可能。
        "expert_ratio": float, #摸倣学習を行う際にLearnerの代わりに教師役(Expert)が選択される確率。省略時は0。
        "warm_up_episodes": int, #学習初期に対戦相手を"Initial"に固定するエピソード数。デフォルトは1000。
        "initial_state_lower": { #Blue(teams[0])だった場合に設定する初期条件の下限値。省略時は置換しない。
            "pos:": 3-dim list[float], #初期位置
            "vel": float, #初期速度
            "heading": float, #初期方位(真北を0とし東回りを正とする)
        },
        "initial_state_upper": { #Blue(teams[0])だった場合に設定する初期条件の上限値。省略時は置換しない。
            "pos:": 3-dim list[float], #初期位置
            "vel": float, #初期速度
            "heading": float, #初期方位(真北を0とし東回りを正とする)
        },
    }, 
    # このクラスで追加されたもの
    "seed": None or int, #MatchMakerとしての乱数シードを指定。
    "log_prefix": str, #全対戦結果をcsv化したログの保存場所。
    "teams": list[str], #登場する陣営名のリスト。省略時は["Blue","Red"]となる。必要がない限り指定しない。
}
"""

def managerConfigReplacer(config,matchInfo):
    """対戦カードを表すmatchInfoに従い、SimulationManagerのコンフィグを置き換える関数。
    configはSimulationManager::getManagerConfig()で得られるものであり、
    Simulationmanagerのコンストラクタに渡すenv_configのうち"Manager"キー以下の部分となる。
    """
    ret=copy.deepcopy(config)
    agentConfigDispatcher=ret["AgentConfigDispatcher"]
    assetConfigDispatcher=ret["AssetConfigDispatcher"]
    for team, info in matchInfo.items():
        num=len(agentConfigDispatcher[team+"Agents"]["overrider"][0]["elements"])
        agentConfigDispatcher[team+"Agents"]["alias"]=matchInfo[team]["Policy"]
        if(matchInfo[team]["MultiPort"]):
            #中央集権型のAgent
            agentConfigDispatcher[team+"Agents"]["overrider"][0]["elements"]=[
                {"type":"direct","value":{"name":team,"port":str(i),"policy":matchInfo[team]["Policy"]+matchInfo[team]["Suffix"]}}
                for i in range(num)
            ]
        else:
            #SingleAssetAgent
            agentConfigDispatcher[team+"Agents"]["overrider"][0]["elements"]=[
                {"type":"direct","value":{"name":team+str(i+1),"policy":matchInfo[team]["Policy"]+matchInfo[team]["Suffix"]}}
                for i in range(num)
            ]
        #指定されていれば、初期条件を置換
        if "InitialState" in info:
            for i in range(num):
                assetConfigDispatcher[team+"InitialState"]["elements"][i]["value"]["instanceConfig"]=info["InitialState"][i]
    return ret

def wrapEnvForTwoTeamCombatMatchMaking(base_class):
    """MatchMakerの付帯情報をgym.Envに付加するラッパー
    """
    class WrappedEnvForTwoTeamCombatMatchMaking(base_class):
        def setMatch(self,matchInfo):
            self.matchInfo=matchInfo
            self.requestReconfigure(managerConfigReplacer(self.manager.getManagerConfig()(),matchInfo),{})
    return WrappedEnvForTwoTeamCombatMatchMaking

class TwoTeamCombatMatchMaker(MatchMaker):
    def initialize(self,config):
        """初期化を行う。
        """
        super().initialize(config)
        self.teams=config.get("teams",["Blue","Red"])
        self.episodeCounter=0
        self.random=np.random.RandomState()
        if("seed" in self.config):
            self.random.seed(self.config["seed"])
        self.file=None
        self.log_prefix=self.config["log_prefix"]
        self.logpath=self.log_prefix+"_"+datetime.datetime.now().strftime("%Y%m%d%H%M%S")+".csv"
        self.policyInfo={name:{
                    "activePast":set(), #候補となる過去の重み番号の一覧。setとして保持
                    "latestPast":0, #最新の重み番号
                    "numEpisodes":0, #これまでに行ったエピソード数
                    "numEpisodesForTrain":0, #これまでに訓練用として行ったエピソード数
                    "lastPopulated":0, #最後に重みが保存されたときの訓練用エピソード数
                    "numWins":0, #これまでの勝利数
                    "numDraws":0, #これまでの引き分け数
                    "numLosses":0, #これまでの敗北数
                    "rating":float(self.policy_config[name].get("rating_initial",1500.0)),
                    "rating_last":float(self.policy_config[name].get("rating_initial",1500.0))
                } for name in self.policy_config}
        self.payoff=PayOff()
    def load(self,path,configOverrider={}):
        """チェックポイントを保存する。その際、configOverriderで与えた項目はconfigを上書きする。
        """
        with open(path,"rb") as f:
            obj=cloudpickle.load(f)
            config=copy.deepcopy(obj["config"])
            config.update(configOverrider)
            self.initialize(config)
            self.resumed=True
            self.episodeCounter=obj["episodeCounter"]
            self.random=obj["random"]
            self.policyInfo=obj["policyInfo"]
            self.payoff=obj["payoff"]
            if(not "log_prefix" in configOverrider):
                self.logpath=obj["logpath"]
                if(os.path.exists(self.logpath)):
                    self.file=open(self.logpath,'a')
        print("MatchMaker has been loaded from ",path)
    def save(self,path):
        """チェックポイントを読み込む。
        """
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path,"wb") as f:
            obj={
                "config":self.config,
                "episodeCounter":self.episodeCounter,
                "random":self.random,
                "policyInfo":self.policyInfo,
                "payoff":self.payoff,
                "logpath":self.logpath
            }
            cloudpickle.dump(obj,f)
    def makeNextMatch(self,matchType):
        """対戦カードを生成する。
        このサンプルでは、以下のような対戦カードが生成される。
        第一陣営(Blue)側：学習中の"Learner"又は一定確率(expert_ratio)で教師役の"Expert"
        第二陣営(Red)側：一定エピソード数(warm_up_episodes)の経過前は常に初期行動判断モデル("Initial")とし、
            経過後は(1)学習中重みの直近のコピー、(2)初期行動判断モデル("Initial")、(3)過去の保存された重みから一様分布で選択
            の3種類を4:4:2の割合で選択(ただし、(3)に該当する重みが存在しない場合は(1)とする。)
        expert_ratioとwarm_up_episodesはmatch_configで指定する。
        初期条件はmatch_configのinitial_state_lowerとinitial_state_upperが指定されていればその領域内でランダムかつ点対称な条件に設定する。
        また、matchTypeは特に使用しない
        """
        ret={
            self.teams[0]:{
                "Policy": "Learner",
                "Weight": -1,
                "Suffix": ""
            }
        }
        expert_ratio = self.match_config.get("expert_ratio",0.0)
        if "Expert" in self.policyInfo and expert_ratio > 0:
            p=np.random.random()
            if(p<expert_ratio):
                ret[self.teams[0]]={
                    "Policy": "Expert",
                    "Weight": -1,
                    "Suffix": ""
                }

        numEpisodesForTrain=self.policyInfo["Learner"]["numEpisodesForTrain"]
        warm_up_episodes=self.match_config.get("warm_up_episodes",1000)
        if(numEpisodesForTrain<warm_up_episodes):
            ret[self.teams[1]]={
                "Policy": "Initial",
                "Weight": -1,
                "Suffix": ""
            }
        else:
            p=np.random.random()
            if(p<0.4):
                ret[self.teams[1]]={
                    "Policy": "Learner",
                    "Weight": 0,
                    "Suffix": "_Past_"+self.teams[1]
                }
            elif(p<0.8):
                ret[self.teams[1]]={
                    "Policy": "Initial",
                    "Weight": -1,
                    "Suffix": ""
                }
            else:
                candidates=np.array(list(self.policyInfo["Learner"]["activePast"]))
                if(len(candidates)>0):
                    wid=self.random.choice(candidates)
                    ret[self.teams[1]]={
                        "Policy": "Learner",
                        "Weight": wid,
                        "Suffix": "_Past_"+self.teams[1]
                    }
                else:
                    ret[self.teams[1]]={
                        "Policy": "Learner",
                        "Weight": 0,
                        "Suffix": "_Past_"+self.teams[1]
                    }
        for team in self.teams:
            ret[team]["MultiPort"]=self.policy_config[ret[team]["Policy"]].get("multi_port",False) is True

        #初期条件の設定(別途最適化してもよい)
        def keepAngle(a):
            ret=a
            while ret>=360.0:
                ret-=360.0
            while ret<=-360.0:
                ret+=360.0
            return ret
        def invert(src):
            #初期条件の非対称性による有利不利を無くしたい場合はこれで点対称にする。
            dst={
                "pos":[-src["pos"][0],-src["pos"][1],src["pos"][2]],
                "vel":src["vel"],
                "heading":keepAngle(src["heading"]+180.0)
            }
            return dst
        if "initial_state_lower" in self.match_config and "initial_state_upper" in self.match_config:
            lower=self.match_config["initial_state_lower"]
            upper=self.match_config["initial_state_upper"]
            state=[{
                "pos":[l+(h-l)*np.random.rand() for l,h in zip(lower["pos"],upper["pos"])],
                "vel":lower["vel"]+(upper["vel"]-lower["vel"])*np.random.rand(),
                "heading": lower["heading"]+(upper["heading"]-lower["heading"])*np.random.rand(),
            } for i in range(2)]
            ret[self.teams[0]]["InitialState"]=state
            ret[self.teams[1]]["InitialState"]=[invert(s) for s in state]

        return ret
    #
    # 対戦結果の処理
    #
    def onEpisodeEnd(self,match,result):
        """いずれかのSimulationManagerインスタンスにおいて対戦が終わったときに呼ばれ、
        そのインスタンスにおける次の対戦カードを生成して返す関数。
        このサンプルでは、以下の情報がMatchMonitor経由で得られるものとしている。
        result={
            "matchType": Any, #この対戦を生成した対戦グループを表す変数
            "winner": str, #直前の対戦の勝者陣営
            "finishedTime": float, #直前の対戦の終了時刻(シミュレーション時刻)
            "numSteps": int, #直前の対戦の終了ステップ数(Agentのステップ数)
            "calcTime": float, #直前の対戦の実行時間(現実の処理時間)
            "scores": dict[str,float], #直前の対戦の各陣営の得点
            "totalRewards": dict[str,float], #直前の対戦の各Agentの合計報酬
            "numAlives": dict[str,int], #直前の対戦終了時の各陣営の生存機数
            "endReason": str, #直前の対戦の終了理由
        }
        """
        if(self.file is None):
            os.makedirs(os.path.dirname(self.logpath),exist_ok=True)
            self.file=open(self.logpath,'w')
            self.makeHeader(match,result)
        self.episodeCounter+=1
        matchTeams=list(match.keys())
        oppos={matchTeams[0]:matchTeams[1],matchTeams[1]:matchTeams[0]}
        names=[match[team]["Policy"]+("" if match[team]["Weight"]<=0 else str(match[team]["Weight"])) for team in matchTeams]
        for team in matchTeams:
            policy=match[team]["Policy"]
            weight=match[team]["Weight"]
            name=policy+("" if weight<=0 else str(weight))#Not same as the name of Policy object. Results from both of the weight -1 and 0 are used to update stats for same "current" policy.
            match[team]["Name"]=name
            if(not name in self.policyInfo):
                self.policyInfo[name]={
                    "activePast":set(),
                    "latestPast":0,
                    "numEpisodes":0,
                    "numEpisodesForTrain":0,
                    "lastPopulated":0,
                    "numWins":0,
                    "numDraws":0,
                    "numLosses":0,
                    "rating":float(self.policy_config[policy].get("rating_initial",1500.0)),
                    "rating_last":float(self.policy_config[policy].get("rating_initial",1500.0))
                }
            self.policyInfo[name]["numEpisodes"]+=1
            if(weight<0):
                self.policyInfo[name]["numEpisodesForTrain"]+=1
            if(result["winner"]==team):
                self.policyInfo[name]["numWins"]+=1
            elif(result["winner"]==oppos[team]):
                self.policyInfo[name]["numLosses"]+=1
            else:
                self.policyInfo[name]["numDraws"]+=1
            self.policyInfo[name]["rating_last"]=self.policyInfo[name]["rating"]
        if(result["winner"]==matchTeams[0]):
            winloss="win"
        elif(result["winner"]==matchTeams[1]):
            winloss="loss"
        else:
            winloss="draw"
        self.payoff.update(names[0],names[1],winloss)
        #Update Elo rating
        for team in matchTeams:
            policy=match[team]["Policy"]
            name=match[team]["Name"]
            o_name=match[oppos[team]]["Name"]
            if(not self.policy_config[policy].get("rating_fixed",False)):
                if(result["winner"]==team):
                    self.policyInfo[name]["rating"]+=16/(10**((self.policyInfo[name]["rating_last"]-self.policyInfo[o_name]["rating_last"])/400.0)+1.0)
                elif(result["winner"]==oppos[team]):
                    self.policyInfo[name]["rating"]-=16/(10**((self.policyInfo[o_name]["rating_last"]-self.policyInfo[name]["rating_last"])/400.0)+1.0)
                else:
                    pass
        #Add extra info whether policies should be populated or not
        populate_config={}
        for team in matchTeams:
            policy=match[team]["Policy"]
            weight=match[team]["Weight"]
            if(weight<0):
                needPopulate=self.policyPopulateChecker(policy)
                if(needPopulate):
                    resetProb=np.clip(self.policy_config[policy]["populate"].get("reset",0.0),0,1)
                    needReset=self.random.uniform()<=resetProb
                else:
                    needReset=False
                if(needPopulate):
                    #保存対象だった場合、保存に伴う内部状態の更新も実施
                    populate_config[policy]=self.populate(policy,needReset)
        self.makeFrame(match,result)
        totalTeamRewards={
            team: np.mean([v for k,v in result["totalRewards"].items() if k.startswith(team)]) for team in matchTeams
        }
        summary="MatchType:{}".format(result["matchType"])
        summary+=" Episode {}".format(self.episodeCounter)
        summary+="("+" vs ".join([match[team]["Name"]+("*" if match[team]["Weight"]==0 else "") for team in matchTeams])+")"
        summary+=" Winner:{}".format(result["winner"] if result["winner"]!="" else "Draw")
        summary+=", Reason:{}".format(result["endReason"])
        summary+=", Score: "+" vs ".join(["{:.2f}".format(result["scores"][team]) for team in matchTeams])
        summary+=", Avg. Total Rewards: "+" vs ".join(["{:.2f}".format(totalTeamRewards[team]) for team in matchTeams])
        summary+=", Steps:{:.2f}".format(result["numSteps"])
        summary+=", Sim.time:{:.2f}".format(result["finishedTime"])
        summary+=", Calc.time:{:.2f}".format(result["calcTime"])
        summary+=", PayOff:{:.2f}".format(self.payoff[names][0])
        summary+=", Rating: ("+",".join(["{:.2f}".format(self.policyInfo[match[team]["Name"]]["rating_last"]) for team in matchTeams])
        summary+=")->("+",".join(["{:.2f}".format(self.policyInfo[match[team]["Name"]]["rating"]) for team in matchTeams])+")"
        print(summary)
        return populate_config
    def get_metrics(self,match,result):
        """SummaryWriter等でログとして記録すべき値をdictで返す。
        """
        ret={}
        for policyBaseName in self.policy_config:
            ret["rating/"+policyBaseName]=self.policyInfo[policyBaseName]["rating"]
            for oppo in self.policyInfo:
                if(policyBaseName!=oppo and self.payoff._games[policyBaseName,oppo]>0):
                    ret["payoff/"+policyBaseName+"/"+oppo]=self.payoff[policyBaseName,oppo]
        return ret
    def makeHeader(self,match,result):
        """MatchMakerのログファイル(csv)のヘッダを作成する。
        """
        row=["Episode","MatchType"]+self.teams+["Winner"]
        row.extend(["score[{}]".format(team) for team in self.teams])
        row.extend(["Avg. totalReward[{}]".format(team) for team in self.teams])
        row.extend(["finishedTime[s]","numSteps","calcTime[s]"])
        row.extend(["numAlives[{}]".format(team) for team in self.teams])
        row.extend("endReason")
        row.extend(["Rating(Before)[{}]".format(team) for team in self.teams])
        row.extend(["Rating(After)[{}]".format(team) for team in self.teams])
        row.extend(["Rating[{}]".format(p) for p in self.policy_config])
        row.extend(["PayOff["+"-".join(self.teams)+"]"])
        self.file.write(','.join(row)+"\n")
        self.file.flush()
    def makeFrame(self,match,result):
        """MatchMakerのログファイル(csv)の１エピソード分のデータ行を作成する。
        """
        names={
            team: match[team]["Policy"]+("" if match[team]["Weight"]<=0 else str(match[team]["Weight"]))
            for team in self.teams
        }
        winnerName=names.get(result["winner"],"Draw")
        row=[
            str(self.episodeCounter),
            result["matchType"]
        ]+[
            names[team]+("*" if match[team]["Weight"] else "") for team in self.teams
        ]+[
            winnerName
        ]
        row.extend([format(result["scores"][team],'+0.16e') for team in self.teams])
        totalTeamRewards={
            team: np.mean([v for k,v in result["totalRewards"].items() if k.startswith(team)]) for team in self.teams
        }
        row.extend([format(r,'+0.16e') for r in totalTeamRewards.values()])
        row.extend([
            format(result["finishedTime"],'+0.16e'),
            str(result["numSteps"]),
            format(result["calcTime"],'+0.16e')])
        row.extend([format(result["numAlives"][team],'+0.16e') for team in self.teams])
        row.extend([result["endReason"]])
        row.extend([format(self.policyInfo[names[team]]["rating_last"],'+0.16e') for team in self.teams])
        row.extend([format(self.policyInfo[names[team]]["rating"],'+0.16e') for team in self.teams])
        row.extend([format(self.policyInfo[p]["rating"],'+0.16e') for p in self.policy_config])
        row.extend([format(self.payoff[names[self.teams[0]],names[self.teams[1]]][0],'+0.16e')])
        self.file.write(','.join(row)+"\n")
        self.file.flush()
    #
    # 重み保存
    #
    def populate(self,policy,needReset):
        """policyをpopulateする。
        内部状態を更新しつつ、対応するpopulate_configを返す。
        """
        #保存対象だった場合、保存に伴う内部状態の更新も実施
        self.policyInfo[policy]["latestPast"]+=1
        self.policyInfo[policy]["activePast"].add(self.policyInfo[policy]["latestPast"])
        self.policyInfo[policy+str(self.policyInfo[policy]["latestPast"])]=self.policyInfo[policy].copy()
        self.policyInfo[policy+str(self.policyInfo[policy]["latestPast"])].pop("latestPast")
        self.policyInfo[policy+str(self.policyInfo[policy]["latestPast"])].pop("activePast")
        self.policyInfo[policy]["lastPopulated"]=self.policyInfo[policy]["numEpisodesForTrain"]
        self.selection(policy)
        return {
            "weight_id": self.policyInfo[policy]["latestPast"],
            "reset": needReset
        }
    def checkInitialPopulation(self):
        """開始時の初期重みをpopulateするかどうかを判定する。
        このサンプルでは、policy_configにおいてpolicy_config[policy]["populate"]["on_start"]=TrueとしたPolicyを保存対象とする。
        policy_config[policy]["populate"]={
            "firstPopulation": int, # 初回の保存を行うエピソード数。この関数では使用しない。
            "interval": int, # 保存間隔。この関数では使用しない。
            "on_start": bool, # 初期重みを保存するかどうか。この関数では使用しない。
            "reset": bool, #重み保存時に重みの初期化を行うかどうか。初期重みの保存時には適用しない。
        }
        """
        populate_config={}
        for policy in self.policy_config:
            config=self.policy_config[policy].get("populate",None) or {}
            if(config.get("on_start",False)):
                populate_config[policy]=self.populate(policy,False)
        return populate_config
    def selection(self,policy):
        """各Policyについて、場に存在するコピーの数(重みの数)を制限するために、残すものを選択する。
        このサンプルでは、policy_configで各ポリシーの"active_limit"キーに上限値を指定することで制限を有効化し、
        上限を越えた場合は古いものから取り除くものとする。
        """
        limit=self.policy_config[policy].get("active_limit",0)
        if(limit is None or limit<=0 or len(self.policyInfo[policy]["activePast"])>limit):
            return
        else:
            newActives=set()
            idx=self.policyInfo[policy]["latestPast"]
            cnt=0
            while(cnt<limit and idx>0):
                if(not idx in newActives):
                    newActives.add(idx)
                    cnt+=1
                idx-=1
            self.policyInfo[policy]["activePast"]=newActives
    #
    # 重み保存の判定用関数
    #
    def policyPopulateChecker(self,policy):
        """重みのコピーを保存するかどうかを判定する関数を生成するための関数。
        このサンプルでは、「前回の保存時から経験した(学習用の)エピソード数」に基づいて判定を行う。
        判定の閾値はpolicy_config[policy]["populate"]で設定するものとしている。
        policy_config[policy]["populate"]={
            "firstPopulation": int, # 初回の保存を行うエピソード数。0以下の値を指定すると一切保存しない。
            "interval": int, # 保存間隔。0以下の値を指定すると一切保存しない。
            "on_start": bool, # 初期重みを保存するかどうか。この関数では使用しない。
            "reset"; float, #重み保存時の重みリセット確率(0〜1)であり、この関数では使用しない。
        }
        """
        count = self.policyInfo[policy]["numEpisodesForTrain"]-self.policyInfo[policy]["lastPopulated"]
        config = self.policy_config[policy].get("populate",None) or {}
        first = config.get("first_population",0)
        if(first>0 and self.policyInfo[policy]["numEpisodesForTrain"]>=first):
            count = self.policyInfo[policy]["numEpisodesForTrain"]-self.policyInfo[policy]["lastPopulated"]
            interval = config.get("interval",0)
            if(interval>0 and count>=interval):
                return True
        return False

class TwoTeamCombatMatchMonitor(MatchMonitor):
    def __init__(self, env):
        self.env=env #GymManagr
    def onEpisodeBegin(self):
        self.startT=time.time()
    def onEpisodeEnd(self,matchType):
        """TwoTeamCombatMatchMakerのonEpisodeEndで使用するresultを生成して返す。
        このサンプルでは、以下の情報をGymManager(SimulationManager)から抽出するものとしている。
        result={
            "matchType": Any, #この対戦を生成した対戦グループを表す変数
            "winner": str, #直前の対戦の勝者陣営
            "finishedTime": float, #直前の対戦の終了時刻(シミュレーション時刻)
            "numSteps": int, #直前の対戦の終了ステップ数(Agentのステップ数)
            "calcTime": float, #直前の対戦の実行時間(現実の処理時間)
            "scores": dict[str,float], #直前の対戦の各陣営の得点
            "totalRewards": dict[str,float], #直前の対戦の各Agentの合計報酬
            "numAlives": dict[str,int], #直前の対戦終了時の各陣営の生存機数
            "endReason": str, #直前の対戦の終了理由
        }
        """
        endT=time.time()
        manager=self.env.manager
        ruler=manager.getRuler()()
        return {
            "matchType":matchType,
            "winner": ruler.winner,
            "finishedTime": manager.getTime(),
            "numSteps": manager.getExposedStepCount(),
            "calcTime":endT-self.startT,
            "scores": manager.scores,
            "totalRewards": manager.totalRewards,
            "numAlives": {team:np.sum([f.isAlive()
                for f in [f() for f in manager.getAssets(lambda a:isinstance(a,Fighter) and a.getTeam()==team)]
                ]) for team in manager.scores},
            "endReason": ruler.endReason.name
        }
