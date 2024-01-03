# Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)

from math import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import sys
import pygame
from pygame.locals import QUIT
from ASRCAISim1.utility.GraphicUtility import *
from ASRCAISim1.libCore import *

class PygameViewer(Viewer):
	"""pygameを用いて、戦況を描画するための基底クラス。
	"""
	def __init__(self,modelConfig,instanceConfig):
		Viewer.__init__(self,modelConfig,instanceConfig)
		self.width=1280#1920
		self.height=720#1080
	def validate(self):
		self.fps=60#フレームレート。現在は無効にしている。
		self.isValid=True
		pygame.init()
		self.window=pygame.display.set_mode((self.width,self.height),pygame.DOUBLEBUF|pygame.OPENGL|pygame.OPENGLBLIT)
		self.clock=pygame.time.Clock()
		glEnable(GL_DEPTH_TEST)
		glEnable(GL_BLEND)
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
		self.panel=self.makePanel()
	def close(self):
		pygame.quit()
	def onEpisodeBegin(self):
		"""エピソードの開始時(reset関数の最後)に呼ばれる。
		"""
		if(not self.isValid):
			self.validate()
		frame=self.getDataFrameCreator().makeFrame(self.manager)()
		self.display(frame)
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
				self.manager.manualDone=True
	def onInnerStepEnd(self):
		"""#インナーループの各ステップの最後(perceiveの後)に呼ばれる。
		"""
		if(self.isValid):
			frame=self.getDataFrameCreator().makeFrame(self.manager)()
			self.display(frame)
	def onEpisodeEnd(self):
		"""エピソードの終了時(step関数の最後でdone==Trueの場合)に呼ばれる
		"""
		if(self.manager.manualDone):
			pygame.quit()
	def display(self,frame):
		"""画面描画を行う。実際の描画処理はPanelに投げる。
		"""
		self.panel.display(frame)
	def makePanel(self):
		"""画面描画を行うPanelオブジェクトを返す。派生クラスでオーバーライドする。
		"""
		raise NotImplementedError
