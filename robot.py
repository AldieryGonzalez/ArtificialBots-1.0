import pybullet as p
import pyrosim.pyrosim as pyrosim
from motor import MOTOR
from sensor import SENSOR
from pyrosim.neuralNetwork import NEURAL_NETWORK


class ROBOT:
    def __init__(self):
        self.robotId = p.loadURDF("generated/body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)  # self.robotId
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
        self.nn = NEURAL_NETWORK("generated/brain.nndf")

    def Prepare_To_Sense(self):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Prepare_To_Act(self):
        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)

    def Sense(self, t):
        for sensorName in self.sensors:
            self.sensors[sensorName].Get_Value(t)

    def Act(self, t):
        for jointName in self.motors:
            self.motors[jointName].Set_Value(t, self)

    def Think(self):
        self.nn.Update()
        self.nn.Print()
