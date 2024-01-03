# Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
import bz2
import datetime
import numpy as np
import os
import pickle
from ASRCAISim1.libCore import *


class GUIStateLogger(Callback):
	"""GUI表示に必要なシミュレーション情報をログとして保存するためのLogger。
	保存されたログをPygameViewerLoaderの派生クラスで読み込むことで表示が可能。
	"""
	def __init__(self,modelConfig,instanceConfig):
		super().__init__(modelConfig,instanceConfig)
		self.prefix=getValueFromJsonKRD(self.modelConfig,"prefix",self.randomGen,"")
		self.episodeInterval=getValueFromJsonKRD(self.modelConfig,"episodeInterval",self.randomGen,1)
		self.innerInterval=getValueFromJsonKRD(self.modelConfig,"innerInterval",self.randomGen,1)
		self.episodeCounter=0
		self.innerCounter=0
		self.timeStamp=datetime.datetime.now().strftime("%Y%m%d%H%M%S")
		self.fo=None
		self.pickler=None

	def onEpisodeBegin(self):
		"""エピソードの開始時(reset関数の最後)に呼ばれる。
		"""
		self.innerCounter=0
		self.episodeCounter+=1
		self.isFileCreated=False
		self.initialFrame=self.getDataFrameCreator().makeFrame(self.manager)

	def onInnerStepEnd(self):
		"""#インナーループの各ステップの最後(perceiveの後)に呼ばれる。
		"""
		self.innerCounter+=1
		if(self.episodeCounter%self.episodeInterval==0):
			if(self.innerCounter%self.innerInterval==0):
				if(not self.isFileCreated):
					path=self.prefix+"_"+self.timeStamp+"_e{:04d}.dat".format(self.episodeCounter)
					os.makedirs(os.path.dirname(path),exist_ok=True)
					self.fo=bz2.BZ2File(path,"wb",compresslevel=9)
					self.pickler=pickle.Pickler(self.fo,protocol=4)
					self.isFileCreated=True
					self.pickler.dump(self.initialFrame)
					self.pickler.clear_memo()
					self.initialFrame=None
				self.pickler.dump(self.getDataFrameCreator().makeFrame(self.manager))
				self.pickler.clear_memo()

	def onEpisodeEnd(self):
		"""エピソードの終了時(step関数の最後でdone==Trueの場合)に呼ばれる
		"""
		self.fo.close()
		self.fo=None
		self.pickler=None

	def getDataFrameCreator(self):
		"""データフレームの生成用オブジェクトを返す。
		"""
		return GUIDataFrameCreator()
