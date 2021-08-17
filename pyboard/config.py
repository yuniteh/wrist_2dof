# This file contains hardcoded variables that are shared amognst various
# scripts in the overall HargrovePR enivronment.
#
# Author: Kevin Brenner

class Config:
	def __init__(self):
		self.num_classes = 10
		self.emg_chan_list = [0,1,2,3,4,5,6,7]
		self.total_num_feats = 4*len(self.emg_chan_list)