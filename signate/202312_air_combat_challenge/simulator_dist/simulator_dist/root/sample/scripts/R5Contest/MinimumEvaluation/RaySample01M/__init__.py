# Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
#ray.RLlibを用いた学習サンプルで学習したモデルの登録方法例
import os
import json
import glob
import ASRCAISim1
from ASRCAISim1.addons.rayUtility.extension.evaluation import StandaloneRayPolicy

#①Agentクラスオブジェクトを返す関数を定義
"""
以下はサンプルのAgentクラスを借りてくる場合の例
"""
def getUserAgentClass(args={}):
    from OriginalModelSample import R5AgentSample01M
    return R5AgentSample01M

#②Agentモデル登録用にmodelConfigを表すjsonを返す関数を定義
"""
なお、modelConfigとは、Agentクラスのコンストラクタに与えられる二つのjson(dict)のうちの一つであり、設定ファイルにおいて
{
    "Factory":{
        "Agent":{
            "modelName":{
                "class":"className",
                "config":{...}
            }
        }
    }
}の"config"の部分に記載される{...}のdictが該当する。
"""    
def getUserAgentModelConfig(args={}):
    return json.load(open(os.path.join(os.path.dirname(__file__),"config.json"),"r"))

#③Agentの種類(一つのAgentインスタンスで1機を操作するのか、陣営全体を操作するのか)を返す関数を定義
"""AgentがAssetとの紐付けに使用するportの名称は本来任意であるが、
　簡単のために1機を操作する場合は"0"、陣営全体を操作する場合は"0"〜"機数-1"で固定とする。
"""
def isUserAgentSingleAsset(args={}):
    #1機だけならばTrue,陣営全体ならばFalseを返すこと。
    return False

#④StandalonePolicyを返す関数を定義
def getUserPolicy(args={}):
    from ray.rllib.algorithms.appo.appo import APPO, APPOConfig
    from ray.rllib.algorithms.appo.appo_torch_policy import APPOTorchPolicy
    import gymnasium as gym
    tc=APPOConfig(APPO).update_from_dict(
        json.load(open(os.path.join(os.path.dirname(__file__),"trainer_config.json"),"r"))
    )

    #カスタムクラスを使用する場合はrayのCatalogに登録する必要があるが、
    #評価時の異なる行動判断モデル間での名称衝突を回避するために、固有の名前で登録し直すことを推奨する。
    from ray.rllib.models import ModelCatalog
    model_config=tc["model"]
    if "custom_model" in model_config:
        from OriginalModelSample.R5TorchNNSampleForRay import R5TorchNNSampleForRay
        ModelCatalog.register_custom_model(args["userModelID"]+":custom_model",R5TorchNNSampleForRay)
        model_config["custom_model"]=args["userModelID"]+":custom_model"

    policyClass=APPOTorchPolicy
    weightPath=args.get("weightPath",None)
    if(weightPath is None):
        cwdWeights=glob.glob(os.path.join(os.path.dirname(__file__),"*.dat"))
        weightPath=cwdWeights[0] if len(cwdWeights)>0 else None
    else:
        weightPath=os.path.join(os.path.dirname(__file__),weightPath)
    policyConfig={
        "trainer_config":tc,
        "policy_class":policyClass,
        "policy_spec_config":{},
        "weight":weightPath
    }
    isDeterministic=False #決定論的に行動させたい場合はTrue、確率論的に行動させたい場合はFalseとする。
    return StandaloneRayPolicy("my_policy",policyConfig,False,isDeterministic)
