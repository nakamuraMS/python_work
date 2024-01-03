# Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)

from math import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import pygame
import numpy as np
from ASRCAISim1.utility.GraphicUtility import *
from ASRCAISim1.libCore import *
from ASRCAISim1.viewer.PygameViewer import PygameViewer

class GodView(PygameViewer):
	"""pygameを用いて、画面の上側に戦域を上から見た図を、下側に戦域を南から見た図を描画する例。
	機体、誘導弾、センサの覆域、防衛ラインの描画に加え、現在の時刻、得点、報酬、Agentの情報等を表示する。
	"""
	def makePanel(self):
		"""画面描画を行うPanelオブジェクトを返す。
		"""
		return GodViewPanel({
			"width": self.width,
			"height": self.height,
		})

class GodViewPanel:
	"""pygameを用いて、画面の上側に戦域を上から見た図を、下側に戦域を南から見た図を描画する例。
	機体、誘導弾、センサの覆域、防衛ラインの描画に加え、現在の時刻、得点、報酬、Agentの情報等を表示する。
	"""
	def __init__(self,config):
		self.width=config["width"]
		self.height=config["height"]
		self.fieldMargin=np.array([0.15,0.1,0.1])#各軸の表示範囲の余裕。
		self.xyScaleType='same'#x軸とy軸の縮尺。sameとしたら同じになる。fitとしたら個別に拡大・縮小される。
		self.regionType=['fix','fix','fix']#各軸の表示範囲。fixとしたら固定。fitとしたら全機が入るように拡大・縮小される。
		self.slideRegion=False
		self.w_margin=0.01
		self.h_margin=0.01
		self.h_xy=0.64
		self.h_yz=0.33
		self.font=pygame.font.Font('freesansbold.ttf',12)
	def calcRegion(self,frame):
		"""x,y,z軸の描画対象範囲を計算する。
		"""
		ruler=frame["ruler"]
		self.fgtrRegion=[
			np.min(np.r_[[f["posI"] for f in frame["fighters"].values()]],0),
			np.max(np.r_[[f["posI"] for f in frame["fighters"].values()]],0)
		]
		ruler=frame["ruler"]
		self.fieldRegion=[
			np.array([-ruler["dOut"],-ruler["dLine"],-ruler["hLim"]]),
			np.array([ruler["dOut"],ruler["dLine"],0])
		]
		self.bothRegion=[
			np.min(np.r_[[self.fgtrRegion[i],self.fieldRegion[i]]],0) for i in range(2)
		]
		xyAspect=self.h_xy*Draw2D.height/(Draw2D.width*(1-self.w_margin*2))
		minR=np.array([(self.bothRegion[0][i] if self.regionType[i]=='fit' else self.fieldRegion[0][i]) for i in range(3)])
		maxR=np.array([(self.bothRegion[1][i] if self.regionType[i]=='fit' else self.fieldRegion[1][i]) for i in range(3)])
		mid=(minR+maxR)/2.0
		delta=(maxR-minR)/2.0
		minR=mid-delta*(self.fieldMargin+np.array([1,1,1]))
		maxR=mid+delta*(self.fieldMargin+np.array([1,1,1]))
		if(self.xyScaleType=='same'):
			if(xyAspect*(maxR[1]-minR[1])>(maxR[0]-minR[0])):
				mid=(maxR[0]+minR[0])/2.0
				delta=xyAspect*(maxR[1]-minR[1])/2.0
				minR[0]=mid-delta
				maxR[0]=mid+delta
			elif(xyAspect*(maxR[1]-minR[1])<(maxR[0]-minR[0])):
				mid=(maxR[1]+minR[1])/2.0
				delta=(maxR[0]-minR[0])/xyAspect/2.0
				minR[1]=mid-delta
				maxR[1]=mid+delta
		#slide
		slide=np.array([0.0,0.0,0.0])
		if self.slideRegion:
			for i in range(3):
				if self.fgtrRegion[0][i] < minR[i]:
					slide[i] -= minR[i] - self.fgtrRegion[0][i]
				if self.fgtrRegion[1][i] > maxR[i]:
					slide[i] += self.fgtrRegion[1][i] - maxR[i]	
		self.region=[minR+slide,maxR+slide]
	def simToReg(self,sim):#returns (xy,yz)
		"""シミュレーション中の慣性座標系で表された位置simを、
		x,y,z各軸を描画範囲で正規化し0〜1にして返す。
		"""
		return (sim-self.region[0])/(self.region[1]-self.region[0])
	def regToSur(self,reg):#'xy' or 'yz'
		"""各軸正規化された位置regを、
		xy図、yz図上で対応する位置に表示されるような、画面全体のxy座標に変換して返す。
		"""
		xy=np.array([
			Draw2D.width*(self.w_margin+reg[1]*(1.0-2*self.w_margin)),
			Draw2D.height*((self.h_margin*2+self.h_yz)+reg[0]*(self.h_xy))
		])
		yz=np.array([
			Draw2D.width*(self.w_margin+reg[1]*(1.0-2*self.w_margin)),
			Draw2D.height*(self.h_margin+(1.0-reg[2])*self.h_yz)
		])
		return np.array([xy,yz])
	def makeGrid(self,interval):
		"""戦域の区切り線を描く。
		"""
		lower=np.ceil(self.region[0]/interval)
		upper=np.floor(self.region[1]/interval)
		cnt=upper-lower
		for i in range(int(lower[0]),int(upper[0])+1):
			d=(i*interval[0]-self.region[0][0])/(self.region[1][0]-self.region[0][0])
			p=[self.regToSur(x)[0] for x in [[d,0,0],[d,1,1]]]
			drawLine2D(p[0][0],p[0][1],p[1][0],p[1][1])
		for i in range(int(lower[1]),int(upper[1])+1):
			d=(i*interval[1]-self.region[0][1])/(self.region[1][1]-self.region[0][1])
			p=[self.regToSur(x) for x in [[0,d,0],[1,d,1]]]
			drawLine2D(p[0][0][0],p[0][0][1],p[1][0][0],p[1][0][1])
			drawLine2D(p[0][1][0],p[0][1][1],p[1][1][0],p[1][1][1])
		for i in range(int(lower[2]),int(upper[2])+1):
			d=(i*interval[2]-self.region[0][2])/(self.region[1][2]-self.region[0][2])
			p=[self.regToSur(x)[1] for x in [[0,0,d],[1,1,d]]]
			drawLine2D(p[0][0],p[0][1],p[1][0],p[1][1])	
	def display(self,frame):
		"""画面描画を行う。実際の描画処理はdisplayImplに記述する。
		"""
		glClearColor(0.8, 0.8, 0.8, 1.0)
		glClear(GL_COLOR_BUFFER_BIT |GL_DEPTH_BUFFER_BIT)
		glMatrixMode(GL_MODELVIEW)
		Draw2D.setShape(self.width,self.height)
		Draw2D.begin()
		self.displayImpl(frame)
		Draw2D.end()
		glFlush()
		pygame.display.flip()
		#pygame.display.update()
		#self.clock.tick(self.fps)
	def displayImpl(self,frame):
		"""実際の描画を処理を記述する。
		"""
		self.calcRegion(frame)
		ruler=frame["ruler"]
		glColor4f(1.0,1.0,1.0,1)
		p1,p2=self.regToSur([0,0,0]),self.regToSur([1,1,1])
		fillRect2D(p1[0][0],p1[0][1],p2[0][0]-p1[0][0],p2[0][1]-p1[0][1])
		fillRect2D(p1[1][0],p1[1][1],p2[1][0]-p1[1][0],p2[1][1]-p1[1][1])
		glColor4f(0.6,0.6,0.6,1)
		self.makeGrid([5000,5000,4000])
		glLineWidth(3.0)
		#防衛ライン、南北境界線
		nw=self.regToSur(self.simToReg(np.array([+ruler["dOut"],-ruler["dLine"],0])))
		ne=self.regToSur(self.simToReg(np.array([+ruler["dOut"],+ruler["dLine"],0])))
		sw=self.regToSur(self.simToReg(np.array([-ruler["dOut"],-ruler["dLine"],0])))
		se=self.regToSur(self.simToReg(np.array([-ruler["dOut"],+ruler["dLine"],0])))
		uw=self.regToSur(self.simToReg(np.array([+ruler["dOut"],-ruler["dLine"],-ruler["hLim"]])))
		ue=self.regToSur(self.simToReg(np.array([+ruler["dOut"],+ruler["dLine"],-ruler["hLim"]])))
		glColor4f(0,0,0,1)
		d=(ruler["dLine"]-self.region[0][1])/(self.region[1][1]-self.region[0][1])
		p=[self.regToSur(x) for x in [[0,d,0],[1,d,1]]]
		drawLine2D(ne[0][0],ne[0][1],se[0][0],se[0][1])
		drawLine2D(ne[1][0],ne[1][1],ue[1][0],ue[1][1])
		drawLine2D(nw[0][0],nw[0][1],sw[0][0],sw[0][1])
		drawLine2D(nw[1][0],nw[1][1],uw[1][0],uw[1][1])
		drawLine2D(nw[0][0],nw[0][1],ne[0][0],ne[0][1])
		drawLine2D(nw[1][0],nw[1][1],ne[1][0],ne[1][1])
		drawLine2D(sw[0][0],sw[0][1],se[0][0],se[0][1])
		drawLine2D(uw[1][0],uw[1][1],ue[1][0],ue[1][1])
		glLineWidth(1.0)
		txt="Time:{:6.2f}s".format(frame["time"])
		pos=(30,self.height-24-10)
		drawText2D(txt,pygame.font.Font('freesansbold.ttf',16),pos,(0,180,50,255),(255,255,255,255))
		txt="Score: "+", ".join([team+"={:.2f}".format(score) for team,score in frame["scores"].items()])
		pos=(30,self.height-24-40)
		drawText2D(txt,pygame.font.Font('freesansbold.ttf',16),pos,(0,180,50,255),(255,255,255,255))

		westPolicy=""
		for agentFullName,agent in frame["agents"].items():
			if agent["team"]==ruler["westSider"]:
				_,_,policy=agentFullName.split(":")
				if policy.startswith("Policy_"):
					westPolicy=policy[7:]
				else:
					westPolicy=policy
				break
		eastPolicy=""
		for agentFullName,agent in frame["agents"].items():
			if agent["team"]==ruler["eastSider"]:
				_,_,policy=agentFullName.split(":")
				if policy.startswith("Policy_"):
					eastPolicy=policy[7:]
				else:
					eastPolicy=policy
			break
		txt=westPolicy
		img=pygame.font.Font('freesansbold.ttf',16).render(txt,True,(0,180,50,255),None)
		pos=(self.width/2-20-img.get_width(),self.height-24-10)
		drawText2D(westPolicy,pygame.font.Font('freesansbold.ttf',16),pos,(255,0,0,255),(255,255,255,255))
		txt="vs"
		img=pygame.font.Font('freesansbold.ttf',16).render(txt,True,(0,180,50,255),None)
		pos=(self.width/2-img.get_width()/2,self.height-24-10)
		drawText2D(txt,pygame.font.Font('freesansbold.ttf',16),pos,(0,0,0,255),(255,255,255,255))
		txt=eastPolicy
		img=pygame.font.Font('freesansbold.ttf',16).render(txt,True,(0,180,50,255),None)
		pos=(self.width/2+20,self.height-24-10)
		drawText2D(eastPolicy,pygame.font.Font('freesansbold.ttf',16),pos,(0,0,255,255),(255,255,255,255))

		#報酬
		west=["{:.1f}".format(frame["totalRewards"][agentFullName]) for agentFullName,agent in frame["agents"].items() if agent["team"]==ruler["westSider"]]
		east=["{:.1f}".format(frame["totalRewards"][agentFullName]) for agentFullName,agent in frame["agents"].items() if agent["team"]==ruler["eastSider"]]
		txt="Total reward of West( "+ruler["westSider"]+" ):"+str(west)
		pos=(30,self.height-24-130)
		drawText2D(txt,pygame.font.Font('freesansbold.ttf',14),pos,(255,50,50,255),(255,255,255,255))
		txt="Total reward of East( "+ruler["eastSider"]+" ):"+str(east)
		pos=(30,self.height-24-150)
		drawText2D(txt,pygame.font.Font('freesansbold.ttf',14),pos,(50,50,255,255),(255,255,255,255))
		#機体
		leading={ruler["westSider"]:-ruler["dLine"],ruler["eastSider"]:-ruler["dLine"]}
		for fullName,f in frame["fighters"].items():
			if(f["team"]==ruler["eastSider"] and f["isAlive"]):
				leading[ruler["eastSider"]]=max(np.dot(ruler["forwardAx"][ruler["eastSider"]],f["posI"][0:2]),leading[ruler["eastSider"]])
				pf=self.regToSur(self.simToReg(np.array(f["posI"])))
				V=int(round(np.linalg.norm(np.array(f["velI"]))))
				vn=self.regToSur(self.simToReg(np.array(f["posI"])+np.array(f["velI"])/V*1000.0))-pf
				vn[0]/=np.linalg.norm(vn[0])
				vn[1]/=np.linalg.norm(vn[1])
				if int(f["name"][-1])<3: #Large
					glColor4f(0.0,0.353,1.0,1)
					n=4
				else: #Small
					glColor4f(0.25,1.0,0.5,1)
					n=3
				fw1=pf+vn*8
				fw2=pf+vn*18
				drawCircle2D(pf[0],4,n,vn[0])
				drawCircle2D(pf[1],4,n,vn[1])
				drawCircle2D(pf[0],5,n,vn[0])
				drawCircle2D(pf[1],5,n,vn[1])
				drawCircle2D(pf[0],6,n,vn[0])
				drawCircle2D(pf[1],6,n,vn[1])
				drawCircle2D(pf[0],7,n,vn[0])
				drawCircle2D(pf[1],7,n,vn[1])
				drawCircle2D(pf[0],8,n,vn[0])
				drawCircle2D(pf[1],8,n,vn[1])
				glColor4f(0,0,0,1)
				drawCircle2D(pf[0],3,n,vn[0])
				drawCircle2D(pf[1],3,n,vn[1])
				drawCircle2D(pf[0],9,n,vn[0])
				drawCircle2D(pf[1],9,n,vn[1])
				drawLine2D(fw1[0][0],fw1[0][1],fw2[0][0],fw2[0][1])
				drawLine2D(fw1[1][0],fw1[1][1],fw2[1][0],fw2[1][1])

				fTxt=f["name"]+": v="+str(V)+", m="+str(f["remMsls"])
				drawText2D(fTxt,self.font,(pf[0][0]+10,pf[0][1]+0),(0,0,0,255))
				drawText2D(fTxt,self.font,(pf[1][0]+10,pf[1][1]+0),(0,0,0,255))

				#誘導弾ロックオン範囲
				glColor4f(0.4,0.4,1.0,0.1)
				ex=np.array(f["ex"])
				ey=np.array(f["ey"])
				ez=np.array(f["ez"])
				xy=sqrt(ex[0]*ex[0]+ex[1]*ex[1])
				pitchAngle=atan2(-ex[2],xy)
				eyHorizontal=np.cross(np.array([0.,0.,1.]),ex)
				sn=np.linalg.norm(eyHorizontal)
				if(abs(sn)<1e-6):
					eyHorizontal=np.array([0.,1.,0.])
				else:
					eyHorizontal/=sn
				exHorizontal=np.cross(eyHorizontal,np.array([0.,0.,1.]))
				exHorizontal/=np.linalg.norm(exHorizontal)
				cs=np.dot(ex,exHorizontal)
				m=f["observables"]["weapon"]["missiles"][0]
				L=m["spec"]["sensor"]["Lref"]
				angle=m["spec"]["sensor"]["thetaFOR"]
				N=16
				#XY平面プラス側
				psxy=[self.simToReg(f["posI"])]+[self.simToReg(
					f["posI"]+L*(cos(-angle+angle*2*i/N)*exHorizontal+sin(-angle+angle*2*i/N)*eyHorizontal))
					for i in range(N+1)]
				psxy=polygonRegionCut(psxy,[0.0,1.0],0)
				glBegin(GL_TRIANGLE_FAN)
				for p in psxy:
					tmp = Draw2D.convertPos(self.regToSur(p)[0])
					glVertex2d(tmp[0], tmp[1])
				glEnd()
				#XY平面マイナス側
				psxy=[self.simToReg(f["posI"])]+[self.simToReg(
					f["posI"]-L*(cos(-angle+angle*2*i/N)*exHorizontal*abs(sin(pitchAngle))+sin(-angle+angle*2*i/N)*eyHorizontal))
					for i in range(N+1)]
				psxy=polygonRegionCut(psxy,[0.0,1.0],0)
				glBegin(GL_TRIANGLE_FAN)
				for p in psxy:
					tmp = Draw2D.convertPos(self.regToSur(p)[0])
					glVertex2d(tmp[0], tmp[1])
				glEnd()
				exInYZ=(ex*np.array([0,1,1]))
				sn=np.linalg.norm(exInYZ)
				if(sn<1e-6):
					exInYZ=np.array([0.,1.,0.])
					absSinYaw=1.0
					eyz=np.array([0.,0.,1.])
				else:
					exInYZ=exInYZ/sn
					eyz=np.cross(exInYZ,np.array([1.,0.,0.]))
					absSinYaw=abs(asin(ex[0]))
				#YZ平面プラス側
				psyz=[self.simToReg(f["posI"])]+[self.simToReg(
					f["posI"]+L*(cos(-angle+angle*2*i/N)*exInYZ+sin(-angle+angle*2*i/N)*eyz))
					for i in range(N+1)]
				psyz=polygonRegionCut(psyz,[0.0,1.0],2)
				glBegin(GL_TRIANGLE_FAN)
				for p in psyz:
					tmp = Draw2D.convertPos(self.regToSur(p)[1])
					glVertex2d(tmp[0], tmp[1])
				glEnd()
				#YZ平面マイナス側
				psyz=[self.simToReg(f["posI"])]+[self.simToReg(
					f["posI"]-L*(cos(-angle+angle*2*i/N)*exInYZ*absSinYaw+sin(-angle+angle*2*i/N)*eyz))
					for i in range(N+1)]
				psyz=polygonRegionCut(psyz,[0.0,1.0],2)
				glBegin(GL_TRIANGLE_FAN)
				for p in psyz:
					tmp = Draw2D.convertPos(self.regToSur(p)[1])
					glVertex2d(tmp[0], tmp[1])
				glEnd()

		glColor4f(1,0,0,1)
		for fullName,f in frame["fighters"].items():
			if(f["team"]==ruler["westSider"] and f["isAlive"]):
				leading[ruler["westSider"]]=max(np.dot(ruler["forwardAx"][ruler["westSider"]],f["posI"][0:2]),leading[ruler["westSider"]])
				pf=self.regToSur(self.simToReg(np.array(f["posI"])))
				V=int(round(np.linalg.norm(np.array(f["velI"]))))
				vn=self.regToSur(self.simToReg(np.array(f["posI"])+np.array(f["velI"])/V*1000.0))-pf
				vn[0]/=np.linalg.norm(vn[0])
				vn[1]/=np.linalg.norm(vn[1])
				if int(f["name"][-1])<3: #Large
					glColor4f(1,0.294,0.0,1)
					n=4
				else: #Small
					glColor4f(1.0,0.25,1.0,1)
					n=3
				fw1=pf+vn*8
				fw2=pf+vn*18
				drawCircle2D(pf[0],4,n,vn[0])
				drawCircle2D(pf[1],4,n,vn[1])
				drawCircle2D(pf[0],5,n,vn[0])
				drawCircle2D(pf[1],5,n,vn[1])
				drawCircle2D(pf[0],6,n,vn[0])
				drawCircle2D(pf[1],6,n,vn[1])
				drawCircle2D(pf[0],7,n,vn[0])
				drawCircle2D(pf[1],7,n,vn[1])
				drawCircle2D(pf[0],8,n,vn[0])
				drawCircle2D(pf[1],8,n,vn[1])
				glColor4f(0,0,0,1)
				drawCircle2D(pf[0],3,n,vn[0])
				drawCircle2D(pf[1],3,n,vn[1])
				drawCircle2D(pf[0],9,n,vn[0])
				drawCircle2D(pf[1],9,n,vn[1])
				drawLine2D(fw1[0][0],fw1[0][1],fw2[0][0],fw2[0][1])
				drawLine2D(fw1[1][0],fw1[1][1],fw2[1][0],fw2[1][1])

				fTxt=f["name"]+": v="+str(V)+", m="+str(f["remMsls"])
				drawText2D(fTxt,self.font,(pf[0][0]+10,pf[0][1]+0),(0,0,0,255))
				drawText2D(fTxt,self.font,(pf[1][0]+10,pf[1][1]+0),(0,0,0,255))

				#誘導弾ロックオン範囲
				glColor4f(1.0,0.4,0.4,0.1)
				ex=np.array(f["ex"])
				ey=np.array(f["ey"])
				ez=np.array(f["ez"])
				xy=sqrt(ex[0]*ex[0]+ex[1]*ex[1])
				pitchAngle=atan2(-ex[2],xy)
				eyHorizontal=np.cross(np.array([0.,0.,1.]),ex)
				sn=np.linalg.norm(eyHorizontal)
				if(abs(sn)<1e-6):
					eyHorizontal=np.array([0.,1.,0.])
				else:
					eyHorizontal/=sn
				exHorizontal=np.cross(eyHorizontal,np.array([0.,0.,1.]))
				exHorizontal/=np.linalg.norm(exHorizontal)
				cs=np.dot(ex,exHorizontal)
				m=f["observables"]["weapon"]["missiles"][0]
				L=m["spec"]["sensor"]["Lref"]
				angle=m["spec"]["sensor"]["thetaFOR"]
				N=16
				#XY平面プラス側
				psxy=[self.simToReg(f["posI"])]+[self.simToReg(
					f["posI"]+L*(cos(-angle+angle*2*i/N)*exHorizontal+sin(-angle+angle*2*i/N)*eyHorizontal))
					for i in range(N+1)]
				psxy=polygonRegionCut(psxy,[0.0,1.0],0)
				glBegin(GL_TRIANGLE_FAN)
				for p in psxy:
					tmp = Draw2D.convertPos(self.regToSur(p)[0])
					glVertex2d(tmp[0], tmp[1])
				glEnd()
				#XY平面マイナス側
				psxy=[self.simToReg(f["posI"])]+[self.simToReg(
					f["posI"]-L*(cos(-angle+angle*2*i/N)*exHorizontal*abs(sin(pitchAngle))+sin(-angle+angle*2*i/N)*eyHorizontal))
					for i in range(N+1)]
				psxy=polygonRegionCut(psxy,[0.0,1.0],0)
				glBegin(GL_TRIANGLE_FAN)
				for p in psxy:
					tmp = Draw2D.convertPos(self.regToSur(p)[0])
					glVertex2d(tmp[0], tmp[1])
				glEnd()
				exInYZ=(ex*np.array([0,1,1]))
				sn=np.linalg.norm(exInYZ)
				if(sn<1e-6):
					exInYZ=np.array([0.,1.,0.])
					absSinYaw=1.0
					eyz=np.array([0.,0.,1.])
				else:
					exInYZ=exInYZ/sn
					eyz=np.cross(exInYZ,np.array([1.,0.,0.]))
					absSinYaw=abs(asin(ex[0]))
				#YZ平面プラス側
				psyz=[self.simToReg(f["posI"])]+[self.simToReg(
					f["posI"]+L*(cos(-angle+angle*2*i/N)*exInYZ+sin(-angle+angle*2*i/N)*eyz))
					for i in range(N+1)]
				psyz=polygonRegionCut(psyz,[0.0,1.0],2)
				glBegin(GL_TRIANGLE_FAN)
				for p in psyz:
					tmp = Draw2D.convertPos(self.regToSur(p)[1])
					glVertex2d(tmp[0], tmp[1])
				glEnd()
				#YZ平面マイナス側
				psyz=[self.simToReg(f["posI"])]+[self.simToReg(
					f["posI"]-L*(cos(-angle+angle*2*i/N)*exInYZ*absSinYaw+sin(-angle+angle*2*i/N)*eyz))
					for i in range(N+1)]
				psyz=polygonRegionCut(psyz,[0.0,1.0],2)
				glBegin(GL_TRIANGLE_FAN)
				for p in psyz:
					tmp = Draw2D.convertPos(self.regToSur(p)[1])
					glVertex2d(tmp[0], tmp[1])
				glEnd()

		#誘導弾
		for m in frame["missiles"].values():
			if(m["team"]==ruler["eastSider"] and m["isAlive"] and m["hasLaunched"]):
				if(m["mode"]==Missile.Mode.SELF.name):
					glColor4f(1.0,1.0,0.0,1.0)
				elif(m["mode"]==Missile.Mode.GUIDED.name):
					if(m["sensor"]["isActive"]):
						glColor4f(0.0,0.8,1,1)
					else:
						glColor4f(0,0,1,1)
				else:
					if(m["sensor"]["isActive"]):
						glColor4f(0.0,0.8,1,1)
					else:
						glColor4f(0.7,0.7,1,1)
				pm=self.regToSur(self.simToReg(m["posI"]))
				fillCircle2D(pm[0],4,8)
				fillCircle2D(pm[1],4,8)
				#目標
				tm=self.regToSur(self.simToReg(m["estTPos"]))
				fillCircle2D(tm[0],2,8)
				fillCircle2D(tm[1],2,8)
				drawLine2D(pm[0][0],pm[0][1],tm[0][0],tm[0][1])
				drawLine2D(pm[1][0],pm[1][1],tm[1][0],tm[1][1])

		glColor4f(1,0,0,1)
		for m in frame["missiles"].values():
			if(m["team"]==ruler["westSider"] and m["isAlive"] and m["hasLaunched"]):
				if(m["mode"]==Missile.Mode.SELF.name):
					glColor4f(1,1,0,1)
				elif(m["mode"]==Missile.Mode.GUIDED.name):
					if(m["sensor"]["isActive"]):
						glColor4f(1.0,0.,0.8,1)
					else:
						glColor4f(1,0,0,1)
				else:
					if(m["sensor"]["isActive"]):
						glColor4f(1.0,0.,0.8,1)
					else:
						glColor4f(1,0.7,0.7,1)
				pm=self.regToSur(self.simToReg(m["posI"]))
				fillCircle2D(pm[0],4,8)
				fillCircle2D(pm[1],4,8)
				#目標
				tm=self.regToSur(self.simToReg(m["estTPos"]))
				fillCircle2D(tm[0],2,8)
				fillCircle2D(tm[1],2,8)
				drawLine2D(pm[0][0],pm[0][1],tm[0][0],tm[0][1])
				drawLine2D(pm[1][0],pm[1][1],tm[1][0],tm[1][1])
