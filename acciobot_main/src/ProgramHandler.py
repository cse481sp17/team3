#!/usr/bin/env python
import pickle
import rospy
from demonstration_program import DemonstrationProgram

# DON'T EDIT THIS FILE LOL IT'S IN DEMO PROGRAM INSTEAD

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

	def get_drop(self):
		return self.program_info["drop"]

	def get_tuck(self):
		return self.program_info["tuck"]

	def get_shelf(self, shelf):
		if shelf == 2:
			return self.get_program("b4secondshelf")
		if shelf == 3:
			return self.get_program("b4thirdshelf")
		print("you did something wrong to get program")

	def get_raise(self):
		return self.get_program("raise")
