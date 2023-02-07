from robot import ROBOT
from world import WORLD
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import constants as c
import numpy
import time


class SIMULATION:
    def __init__(self, directOrGUI, solutionID):
        if (directOrGUI == "DIRECT"):
            self.directOrGUI = directOrGUI
            self.physicsClient = p.connect(p.DIRECT)
        else:
            self.directOrGUI = "GUI"
            self.physicsClient = p.connect(p.GUI)
        self.directOrGUI = directOrGUI
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -19.6)
        self.world = WORLD()
        self.robot = ROBOT(solutionID)

    def __del__(self):
        p.disconnect()

    def Get_Fitness(self):
        self.robot.Get_Fitness()

    def Run(self):
        for i in range(c.steps):
            p.stepSimulation()

            self.robot.Sense(i)
            self.robot.Think()
            self.robot.Act(i)

            if (self.directOrGUI == "GUI"):
                time.sleep(1 / 60)
