import pybullet as p
import numpy
import os
import pyrosim.pyrosim as pyrosim
from motor import MOTOR
from sensor import SENSOR
from pyrosim.neuralNetwork import NEURAL_NETWORK
import constants as c


class ROBOT:
    def __init__(self, id):
        self.id = id
        self.robotId = p.loadURDF("generated/body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)  # self.robotId
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
        self.nn = NEURAL_NETWORK(f"brain{id}.nndf")
        os.system(f"del brain{id}.nndf")

    def Get_Fitness(self):
        stateOfLinkZero = p.getLinkState(self.robotId, 0)
        positionOfLinkZero = stateOfLinkZero[0]
        xCoordinateOfLinkZero = positionOfLinkZero[0]
        f = open(f"tmp{self.id}.txt", "w")
        f.write(str(xCoordinateOfLinkZero))
        f.close()
        os.rename(f"tmp{self.id}.txt",
                  f"fitness{self.id}.txt")

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
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(
                    neuronName) * c.motorJointRange
                self.motors[jointName].Set_Value(desiredAngle, self)

    def Think(self):
        self.nn.Update()
