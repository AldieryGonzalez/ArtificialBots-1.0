from robot import ROBOT
from world import WORLD
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import constants as c
import numpy
import time


class SIMULATION:
    def __init__(self):
        self.physicsClient = p.connect(p.GUI)
        self.world = WORLD()
        self.robot = ROBOT()
        p.loadSDF("generated/world.sdf")
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -19.6)
        self.planeId = p.loadURDF("plane.urdf")

    def __del__(self):
        p.disconnect()

    def Run(self):
        for i in range(c.steps):
            p.stepSimulation()

            self.robot.Sense(i)
            self.robot.Act(i)

            time.sleep(1 / 60)
