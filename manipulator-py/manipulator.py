
from mujoco_py import load_model_from_xml, load_model_from_path, MjSim, MjViewer
import math
import os
import numpy as np

from mujoco_py.modder import TextureModder

class manipulator():
	def __init__(self):
		pass
		self.model = load_model_from_path("multi-links_v2.xml")
		self.state_dim = 10+5+3
		self.action_dim = 5

	def reset(self):
		self.rx = 0.2
		self.ry = -0.2
		self.rz = np.random.rand()*0.1+0.05
		self.sim = MjSim(self.model)
		# self.viewer = MjViewer(self.sim)
		self.qvel=np.zeros(9)

		s = np.array( [self.sim.data.qvel[0], self.sim.data.qvel[1], self.sim.data.qvel[2], self.sim.data.qvel[3], self.sim.data.qvel[4],\
		 self.sim.data.qpos[0], self.sim.data.qpos[1], self.sim.data.qpos[2], self.sim.data.qpos[3], self.sim.data.qpos[4],\
		  self.qvel[0], self.qvel[1], self.qvel[2], self.qvel[3], self.qvel[4], self.rx, self.ry, self.rz] )
		# print("s: ", s)
		return s
	def step(self, a):
		pass
		# print("a: ", a)

		for i in [0, 1, 2, 3, 4]:
			self.qvel[i] += a[i]
			if self.qvel[i]>1.0:
				self.qvel[i] = 1.0
			if self.qvel[i]<-1.0:
				self.qvel[i] = -1.0

			self.sim.data.qvel[i] = self.qvel[i]		
				
		# Fix another joints.
		for i in [5,6,7,8]:
			self.sim.data.qpos[i] = 0
			self.sim.data.qvel[i] = 0

		self.sim.step()
		# self.viewer.render()

		# print("self.sim.data.sensordata: ", self.sim.data.sensordata.tolist())

		dis = np.linalg.norm(self.sim.data.sensordata[3:]-[self.rx, self.ry, self.rz])

		s = np.array( [self.sim.data.qvel[0], self.sim.data.qvel[1], self.sim.data.qvel[2], self.sim.data.qvel[3], self.sim.data.qvel[4],\
		 self.sim.data.qpos[0], self.sim.data.qpos[1], self.sim.data.qpos[2], self.sim.data.qpos[3], self.sim.data.qpos[4],\
		  self.qvel[0], self.qvel[1], self.qvel[2], self.qvel[3], self.qvel[4], self.rx, self.ry, self.rz] )

		d = 0

		r = - dis

		return s, r, d, 'info'
