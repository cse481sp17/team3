#!/usr/bin/env python
import pickle
import rospy
from demonstration_program import DemonstrationProgram

class ProgramHandler(object):
	def __init__(self, load_file):
		try:
			with open(load_file, "r") as f:
				self.program_info = pickle.load(f)
		except EOFError:
			print('Got EOFError loading program handler')
			self.program_info = {}

	def get_program(self, program_name):
		return self.program_info[program_name]
