
from mujoco_py import load_model_from_xml, load_model_from_path, MjSim, MjViewer
import math
import os
import numpy as np

from mujoco_py.modder import TextureModder

class manipulator():
	def __init__(self):
		pass
		self.model = load_model_from_path("multi-links_v2.xml")
		self.state_dim = 4
		self.action_dim = 2

	def reset(self):
		pass
		self.sim = MjSim(self.model)
		# self.viewer = MjViewer(self.sim)
		self.qvel=np.zeros(9)

		s = np.array( [self.sim.data.qvel[0], self.sim.data.qvel[2], self.sim.data.qpos[0], self.sim.data.qpos[2]] )
		# print("s: ", s)
		return s
	def step(self, a):
		pass
		# print("a: ", a)
		self.qvel[0] += a[0]
		self.qvel[2] += a[1]

		for i in [0, 2]:
			if self.qvel[i]>1.0:
				self.qvel[i] = 1.0
			if self.qvel[i]<-1.0:
				self.qvel[i] = -1.0

			self.sim.data.qvel[i] = self.qvel[i]		
				
		# self.sim.data.ctrl[:]=0.005

		# Fix another joints.
		for i in [1,3,4,5,6,7,8]:
			self.sim.data.qpos[i] = 0
			self.sim.data.qvel[i] = 0

		self.sim.step()
		# self.viewer.render()

		# print("self.sim.data.sensordata: ", self.sim.data.sensordata.tolist())

		# rx, ry, rz = self.set_pos(0.1, 0.1, 0.08, r=0.05)
		rx, ry, rz = -0.2, 0.1, 0.08

		dis = np.linalg.norm(self.sim.data.sensordata[3:]-[rx, ry, rz])

		# print("dis: ", dis)

		s = np.array( [self.sim.data.qvel[0], self.sim.data.qvel[2], self.sim.data.qpos[0], self.sim.data.qpos[2]] )
		# print("s: ", s)
		d = 0
		# print("dis: ", dis)
		r = - dis
		# print("r = - dis: ", r)

		return s, r, d, 'info'

	def set_pos(self, cx, cy, cz, r):
		th=np.random.randint(0,360) *np.pi/180
		phi=np.random.randint(0,360) *np.pi/180
		rx = cx + r*np.cos(phi)*np.cos(th)
		ry = cy + r*np.cos(phi)*np.sin(th)
		rz = cz + r*np.sin(phi)

		return rx, ry, rz

