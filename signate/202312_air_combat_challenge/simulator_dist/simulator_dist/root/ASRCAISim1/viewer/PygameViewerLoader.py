# Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)

from math import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import os
import pygame
from pygame.locals import QUIT
import cv2
import numpy as np
from ASRCAISim1.utility.GraphicUtility import *
from ASRCAISim1.libCore import nljson
import pickle
import bz2
import glob

class PygameViewerLoader:
	"""PygameViewerのGUI表示を、GUIStateLoggerにより保存されたログを読み込んで行いつつ、
	連番画像又は動画として保存するためのクラス。(保存の有無は選択可能。)
	インターフェースはCallbackに準じているが、SimulationManagerとは独立に動くものである。
	PygameViewerクラスと同様に、実際の描画処理を行うPanelオブジェクトをmakePanelメソッドのオーバーライドにより指定する。
	<使い方の例>
	```python
	loader=GodViewLoader({
		"globPattern":"~/logs/hoge_*.dat",
		"outputPrefix":"~/movies/movie",
		"asVideo":True,
		"fps":60
	})
	loader.run()
	```
	"""
	def __init__(self,modelConfig={},instanceConfig={}):
		self.globPattern=modelConfig.get("globPattern","") #読み込み対象のファイルをglobパターンで指定する。		
		self.outputDir=modelConfig.get("outputDir",None) #保存先のprefixを指定する。未指定の場合は保存せず表示のみとなる。
		self.outputFileNamePrefix=modelConfig.get("outputFileNamePrefix",None) #保存先のprefixを指定する。未指定の場合は保存せず表示のみとなる。
		self.dataPaths=sorted(glob.glob(self.globPattern,recursive=True))
		self.asVideo=modelConfig.get("asVideo",True) #動画として保存するか連番画像として保存するか。
		self.skipRate=modelConfig.get("skipRate",1) #保存するフレームの間隔
		self.fps=modelConfig.get("fps",10) #動画として保存する際のフレームレート
		print("self.globPattern",self.dataPaths)
		self.episodeCounter=0
		self.width=1280#1920
		self.height=720#1080
		self.isValid=False
		self.manualDone=False
		self.unpickler=None

	def validate(self):
		self.isValid=True
		pygame.init()
		self.font=pygame.font.Font('freesansbold.ttf',12)
		self.window=pygame.display.set_mode(
			(self.width,self.height),pygame.DOUBLEBUF | pygame.OPENGL | pygame.OPENGLBLIT)
		self.clock=pygame.time.Clock()
		glEnable(GL_DEPTH_TEST)
		glEnable(GL_BLEND)
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
		self.panel=self.makePanel()

	def close(self):
		pygame.quit()

	def run(self):
		if(not self.isValid):
			self.validate()
		self.episodeCounter=0
		for path in self.dataPaths:
			self.episodeCounter += 1
			self.currentFileName=os.path.splitext(os.path.basename(path))[0]
			print("current: ",self.currentFileName)
			with bz2.BZ2File(path,"rb") as fin:
				self.onEpisodeBegin()
				unpickler=pickle.Unpickler(fin)
				while True:
					try:
						self.currentFrame=unpickler.load()
						if(isinstance(self.currentFrame,nljson)):
							self.currentFrame=self.currentFrame()
						self.onInnerStepBegin()
						self.onInnerStepEnd()
					except EOFError:
						break
				self.onEpisodeEnd()
				del unpickler

	def saveImage(self):
		try:
			if(not self.asVideo):
				path=os.path.join(
					self.outputDir,
					self.outputFileNamePrefix+"e{:04d}_f{:06.1f}.png".format(self.episodeCounter,self.frameCounter)
				)
				os.makedirs(os.path.dirname(path),exist_ok=True)
			width=self.width
			height=self.height
			img=np.zeros((height,width,3),dtype=np.uint8)
			glReadPixels(0,0,width,height,GL_BGR,GL_UNSIGNED_BYTE,img.data)
			img=np.ascontiguousarray(img[::-1])
			if(self.asVideo):
				self.video.write(img)
			else:
				cv2.imwrite(path,img)
		except Exception as e:
			print("failed to save an image:")
			raise e

	def onEpisodeBegin(self):
		"""エピソードの開始時(reset関数の最後)に呼ばれる。
		"""
		if(not self.isValid):
			self.validate()

		self.frameCounter=0

		if(self.asVideo):
			fourcc=cv2.VideoWriter_fourcc('m','p','4','v')
			path=os.path.join(
				self.outputDir,
				self.outputFileNamePrefix+"{}.mp4".format(self.currentFileName)
			)
			os.makedirs(os.path.dirname(path),exist_ok=True)
			self.video=cv2.VideoWriter(
				path,fourcc,self.fps,(self.width,self.height))

	def onStepBegin(self):
		"""gym.Envとしてのstepの開始時に呼ばれる。
		"""
		pass

	def onStepEnd(self):
		"""gym.Envとしてのstepの最後に呼ばれる。
		Managerは得点計算⇛報酬計算⇛その他コールバック⇛画面表示の順で呼ぶ。
		"""
		pass

	def onInnerStepBegin(self):
		"""インナーループの各ステップの開始時(controlの前)に呼ばれる。
		"""
		for e in pygame.event.get():
			if(e.type==QUIT):
				self.manualDone=True

	def onInnerStepEnd(self):
		"""#インナーループの各ステップの最後(perceiveの後)に呼ばれる。
		"""
		if(self.isValid):
			self.frameCounter+=1
			if (self.frameCounter-1)%self.skipRate==0:
				self.display(self.currentFrame)
				if(self.outputFileNamePrefix is not None):
					self.saveImage()

	def onEpisodeEnd(self):
		"""エピソードの終了時(step関数の最後でdone==Trueの場合)に呼ばれる
		"""
		if(self.asVideo):
			self.video.release()
		if(self.manualDone):
			pygame.quit()

	def display(self,frame):
		"""画面描画を行う。実際の描画処理はPanelに投げる。
		"""
		self.panel.display(frame)

	def makePanel(self):
		"""画面描画を行うPanelオブジェクトを返す。派生クラスでオーバーライドする。
		"""
		raise NotImplementedError
