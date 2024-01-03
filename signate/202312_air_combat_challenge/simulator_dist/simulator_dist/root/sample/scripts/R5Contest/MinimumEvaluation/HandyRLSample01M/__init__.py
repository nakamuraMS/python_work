# Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
#HandyRLを用いた学習サンプルで学習したモデルの登録方法例
import os,json
import yaml
import ASRCAISim1
from ASRCAISim1.addons.HandyRLUtility.StandaloneHandyRLPolicy import StandaloneHandyRLPolicy
from ASRCAISim1.addons.HandyRLUtility.distribution import getActionDistributionClass
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
    return json.load(open(os.path.join(os.path.dirname(__file__),"agent_config.json"),"r"))

#③Agentの種類(一つのAgentインスタンスで1機を操作するのか、陣営全体を操作するのか)を返す関数を定義
"""AgentがAssetとの紐付けに使用するportの名称は本来任意であるが、
　簡単のために1機を操作する場合は"0"、陣営全体を操作する場合は"0"〜"機数-1"で固定とする。
"""
def isUserAgentSingleAsset(args={}):
    #1機だけならばTrue,陣営全体ならばFalseを返すこと。
    return False

#④StandalonePolicyを返す関数を定義
def getUserPolicy(args={}):
    from OriginalModelSample.R5TorchNNSampleForHandyRL import R5TorchNNSampleForHandyRL
    import glob
    model_config=yaml.safe_load(open(os.path.join(os.path.dirname(__file__),"model_config.yaml"),"r"))
    weightPath=None
    if(args is not None):
        weightPath=args.get("weightPath",None)
    if weightPath is None:
        cwdWeights=glob.glob(os.path.join(os.path.dirname(__file__),"*.pth"))
        weightPath=cwdWeights[0] if len(cwdWeights)>0 else None
    else:
        weightPath=os.path.join(os.path.dirname(__file__),weightPath)
    isDeterministic=False #決定論的に行動させたい場合はTrue、確率論的に行動させたい場合はFalseとする。
    return StandaloneHandyRLPolicy(R5TorchNNSampleForHandyRL,model_config,weightPath,getActionDistributionClass,isDeterministic)
