# Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
import os,json
import ASRCAISim1
from ASRCAISim1.libCore import Factory
from ASRCAISim1.common import addPythonClass
if(os.name=="nt"):
    libName="OriginalModelSample"
    pyd=os.path.join(os.path.dirname(__file__),"lib"+libName+".pyd")
    if(not os.path.exists(pyd) or os.path.getsize(pyd)==0):
        print("Info: Maybe the first run after install. A hardlink to a dll will be created.")
        if(os.path.exists(pyd)):
            os.remove(pyd)
        dll=os.path.join(os.path.dirname(__file__),"lib"+libName+".dll")
        if(not os.path.exists(dll)):
            dll=os.path.join(os.path.dirname(__file__),""+libName+".dll")
        if(not os.path.exists(dll)):
            raise FileNotFoundError("There is no lib"+libName+".dll or "+libName+".dll.")
        import subprocess
        subprocess.run([
            "fsutil",
            "hardlink",
            "create",
            pyd,
            dll
        ])
try:
    from OriginalModelSample.libOriginalModelSample import *
except ImportError as e:
    if(os.name=="nt"):
        print("Failed to import the module. If you are using Windows, please make sure that: ")
        print('(1) If you are using conda, CONDA_DLL_SEARCH_MODIFICATION_ENABLE should be set to 1.')
        print('(2) dll dependencies (such as nlopt) are located appropriately.')
    raise e

from OriginalModelSample.R5PyAgentSample01S import R5PyAgentSample01S
from OriginalModelSample.R5PyAgentSample01M import R5PyAgentSample01M
from OriginalModelSample.R5PyRewardSample01 import R5PyRewardSample01
from OriginalModelSample.R5PyRewardSample02 import R5PyRewardSample02

addPythonClass('Agent','R5PyAgentSample01S',R5PyAgentSample01S)
addPythonClass('Agent','R5PyAgentSample01M',R5PyAgentSample01M)
addPythonClass('Reward','R5PyRewardSample01',R5PyRewardSample01)
addPythonClass('Reward','R5PyRewardSample02',R5PyRewardSample02)

Factory.addDefaultModelsFromJsonFile(os.path.join(os.path.dirname(__file__),"./config/sampleConfig.json"))
