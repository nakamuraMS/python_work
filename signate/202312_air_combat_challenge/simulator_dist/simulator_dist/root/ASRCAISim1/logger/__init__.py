# Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
import json
from ASRCAISim1.common import addPythonClass

#Loggerの登録

from ASRCAISim1.logger.LoggerSample import BasicLogger
addPythonClass('Callback','BasicLogger',BasicLogger)

from ASRCAISim1.logger.MultiEpisodeLogger import MultiEpisodeLogger
addPythonClass('Callback','MultiEpisodeLogger',MultiEpisodeLogger)

try:
	from ASRCAISim1.logger.GodViewLogger import GodViewLogger
	addPythonClass('Callback','GodViewLogger',GodViewLogger)
except:
	pass

from ASRCAISim1.logger.GUIStateLogger import GUIStateLogger
addPythonClass('Callback','GUIStateLogger',GUIStateLogger)
