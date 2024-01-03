# Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)

from ASRCAISim1.viewer.PygameViewerLoader import PygameViewerLoader
from ASRCAISim1.viewer.GodView import GodViewPanel

class GodViewLoader(PygameViewerLoader):
	"""GodViewと同等の表示を、GUIStateLoggerにより保存されたログを読み込んで行いつつ、
	連番画像又は動画として保存するためのクラス。(保存の有無は選択可能。)
	インターフェースはCallbackに準じているが、SimulationManagerとは独立に動くものである。
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
	def makePanel(self):
		"""画面描画を行うPanelオブジェクトを返す。
		"""
		return GodViewPanel({
			"width": self.width,
			"height": self.height,
		})
