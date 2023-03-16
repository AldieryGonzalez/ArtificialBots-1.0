import numpy
from termcolor import cprint
import pyrosim.pyrosim as pyrosim
from randomBody import RandomBody
import os
import random
import time
import constants as c


length, width, height = 1, 1, 1
x, y, z = 0, 0, 1

goalSize = c.goalSize
goalPos = c.goalPos
crossWidth = 1
sideCrossSize = [crossWidth, crossWidth, goalSize[2]]
crossSize = [goalSize[0]+2*crossWidth, crossWidth, crossWidth]
leftCrossPos = [goalPos[0]-goalSize[0]/2, goalPos[1]-crossWidth, goalSize[2]/2]
rightCrossPos = [goalPos[0]+goalSize[0]/2,
                 goalPos[1]-crossWidth, goalSize[2]/2]
crossPos = [goalPos[0], goalPos[1]-crossWidth, goalSize[2]+crossWidth/2]


class SOLUTION:
    def __init__(self, myID):
        self.myID = myID
        self.body = self.Create_Body(RandomBody)
        self.history = []

    def Evaluate(self, directOrGui):
        pass

    def Start_Simulation(self, directOrGui, eval=True):
        self.Create_World()
        self.body.Generate()
        os.system("start /B python3 simulate.py " +
                  directOrGui + " " + str(self.myID) + " " + str(eval))

    def Wait_For_Simulation_To_End(self):
        while not os.path.exists(f"fitness{str(self.myID)}.txt"):
            time.sleep(0.01)
        try:
            f = open(f"fitness{str(self.myID)}.txt", "r")
            value = float(f.read())
            self.fitness = value
            # print(f"\n\nRobot Number {self.myID}: {self.fitness}\n")
            f.close()
            os.system(f"del fitness{self.myID}.txt")
            if (len(self.history) == 0):
                self.history.append(self.fitness)
        except:
            time.sleep(0.1)
            print("error")
            f = open(f"fitness{str(self.myID)}.txt", "r")
            value = float(f.read())
            self.fitness = value
            # print(f"\n\nRobot Number {self.myID}: {self.fitness}\n")
            f.close()
            os.system(f"del fitness{self.myID}.txt")
            if (len(self.history) == 0):
                self.history.append(self.fitness)

    def Mutate(self):
        self.body.Mutate()
        randRow = random.randrange(0, len(self.body.sensors))
        randCol = random.randrange(0, len(self.body.joints))
        randValue = random.random() * 2 - 1
        self.body.weights[randRow, randCol] = randValue
        self.body.Generate()

    def Set_ID(self, newID):
        self.myID = newID
        self.body.Set_ID(newID)

    def Print(self):
        cprint(
            f'ID#: {self.myID}, fitness: {self.fitness}', "black", "on_white", attrs=["bold"])

    def Create_World(self):
        pyrosim.Start_SDF("generated/world.sdf")
        pyrosim.Send_Sphere(name=f'Ball', pos=[15, 1.5, 0.5])
        pyrosim.Send_Cube(name='goalBack', pos=[goalPos[0], goalPos[1], goalPos[2]],
                          size=[goalSize[0], goalSize[1], goalSize[2]], mass=999)
        pyrosim.Send_Cube(name='crossbar', pos=crossPos,
                          size=crossSize, mass=999)
        pyrosim.Send_Cube(name='leftSidebar',
                          pos=leftCrossPos, size=sideCrossSize, mass=999)
        pyrosim.Send_Cube(name='rightSidebar', pos=rightCrossPos,
                          size=sideCrossSize, mass=999)
        pyrosim.End()

    def Create_Empty_World(self):
        pyrosim.Start_SDF("generated/world.sdf")
        pyrosim.End()

    def Create_Body(self, bodyConstructor):
        return bodyConstructor(self.myID)

    def Generate_Biped(self):
        pyrosim.Start_URDF("generated/body.urdf")
        pyrosim.Send_Cube(name=f'Torso', pos=[0, 0, 1.5], size=[
            length, width, height], colorName="Red", rgbaStr="1 0 0 1")

        pyrosim.Send_Joint(name="Torso_LeftLeg", parent="Torso", child="LeftLeg",
                           type="revolute", position=[-0.6, 0, 1.5], jointAxis="1 0 0")
        pyrosim.Send_Cube(name=f'LeftLeg', pos=[0, 0, 0], size=[
            0.2, 0.2, 0.7])

        pyrosim.Send_Joint(name="LeftLeg_LeftLower", parent="LeftLeg", child="LeftLower",
                           type="revolute", position=[0, 0, -0.35], jointAxis="1 0 0")
        pyrosim.Send_Cube(name=f'LeftLower', pos=[0, 0, -0.35], size=[
            0.2, 0.2, 0.7])
        pyrosim.Send_Joint(name="LeftLower_LeftFoot", parent="LeftLower", child="LeftFoot",
                           type="revolute", position=[0, 0, -0.35], jointAxis="1 0 0")
        pyrosim.Send_Cube(name=f'LeftFoot', pos=[0, 0, -0.35], size=[
            0.5, 0.5, 0.1])

        pyrosim.Send_Joint(name="Torso_RightLeg", parent="Torso", child="RightLeg",
                           type="revolute", position=[0.6, 0, 1.5], jointAxis="1 0 0")
        pyrosim.Send_Cube(name=f'RightLeg', pos=[0.0, 0, 0], size=[
            0.2, 0.2, 0.7])

        pyrosim.Send_Joint(name="RightLeg_RightLower", parent="RightLeg", child="RightLower",
                           type="revolute", position=[0, 0, -0.35], jointAxis="1 0 0")
        pyrosim.Send_Cube(name=f'RightLower', pos=[0.0, 0, -0.35], size=[
            0.2, 0.2, 0.7])
        pyrosim.Send_Joint(name="RightLower_RightFoot", parent="RightLower", child="RightFoot",
                           type="revolute", position=[0, 0, -0.35], jointAxis="1 0 0")
        pyrosim.Send_Cube(name=f'RightFoot', pos=[0.0, 0, -0.35], size=[
            0.5, 0.5, 0.1])

        pyrosim.End()

    def Generate_Bi_Brain(self):
        pyrosim.Start_NeuralNetwork(f"brain{self.myID}.nndf")

        pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
        pyrosim.Send_Sensor_Neuron(name=1, linkName="LeftFoot")
        pyrosim.Send_Sensor_Neuron(name=2, linkName="RightFoot")

        pyrosim.Send_Motor_Neuron(name=3, jointName="Torso_LeftLeg")
        pyrosim.Send_Motor_Neuron(name=4, jointName="Torso_RightLeg")
        pyrosim.Send_Motor_Neuron(name=5, jointName="LeftLeg_LeftLower")
        pyrosim.Send_Motor_Neuron(name=6, jointName="RightLeg_RightLower")

        rows = c.numSensorNeurons
        cols = c.numMotorNeurons
        for currentRow in range(rows):
            for currentColumn in range(cols):
                index = currentRow*cols + currentColumn
                pyrosim.Send_Synapse(sourceNeuronName=currentRow,
                                     targetNeuronName=currentColumn+rows, weight=self.body.weights.item(index))

        pyrosim.End()

    def Generate_Quadruped(self):
        pyrosim.Start_URDF("generated/body.urdf")
        pyrosim.Send_Cube(name=f'Torso', pos=[0, y, z], size=[
            length, width, height])

        pyrosim.Send_Joint(name="Torso_FrontLeg", parent="Torso", child="FrontLeg",
                           type="revolute", position=[0, 0.5, 1], jointAxis="1 0 0")
        pyrosim.Send_Cube(name=f'FrontLeg', pos=[0, 0.5, 0], size=[
            0.2, width, 0.2])

        pyrosim.Send_Joint(name="FrontLeg_FrontLower", parent="FrontLeg", child="FrontLower",
                           type="revolute", position=[0, width, 0], jointAxis="1 0 0")
        pyrosim.Send_Cube(name=f'FrontLower', pos=[0, 0, -0.5], size=[
            0.2, 0.2, width])

        pyrosim.Send_Joint(name="Torso_BackLeg", parent="Torso", child="BackLeg",
                           type="revolute", position=[0, -0.5, 1], jointAxis="1 0 0")
        pyrosim.Send_Cube(name=f'BackLeg', pos=[0, -0.5, 0], size=[
            0.2, width, 0.2])

        pyrosim.Send_Joint(name="BackLeg_BackLower", parent="BackLeg", child="BackLower",
                           type="revolute", position=[0, -width, 0], jointAxis="1 0 0")
        pyrosim.Send_Cube(name=f'BackLower', pos=[0, 0, -0.5], size=[
            0.2, 0.2, width])

        pyrosim.Send_Joint(name="Torso_LeftLeg", parent="Torso", child="LeftLeg",
                           type="revolute", position=[-0.5, 0, 1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name=f'LeftLeg', pos=[-0.5, 0, 0], size=[
            width, 0.2, 0.2])

        pyrosim.Send_Joint(name="LeftLeg_LeftLower", parent="LeftLeg", child="LeftLower",
                           type="revolute", position=[-width, 0, 0], jointAxis="0 1 0")
        pyrosim.Send_Cube(name=f'LeftLower', pos=[0, 0, -0.5], size=[
            0.2, 0.2, width])

        pyrosim.Send_Joint(name="Torso_RightLeg", parent="Torso", child="RightLeg",
                           type="revolute", position=[0.5, 0, 1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name=f'RightLeg', pos=[0.5, 0, 0], size=[
            width, 0.2, 0.2])

        pyrosim.Send_Joint(name="RightLeg_RightLower", parent="RightLeg", child="RightLower",
                           type="revolute", position=[width, 0, 0], jointAxis="0 1 0")
        pyrosim.Send_Cube(name=f'RightLower', pos=[0, 0, -0.5], size=[
            0.2, 0.2, width])

        pyrosim.End()

    def Generate_Quad_Brain(self):
        pyrosim.Start_NeuralNetwork(f"brain{self.myID}.nndf")

        pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
        pyrosim.Send_Sensor_Neuron(name=1, linkName="BackLeg")
        pyrosim.Send_Sensor_Neuron(name=2, linkName="FrontLeg")
        pyrosim.Send_Sensor_Neuron(name=3, linkName="LeftLeg")
        pyrosim.Send_Sensor_Neuron(name=4, linkName="RightLeg")
        pyrosim.Send_Sensor_Neuron(name=5, linkName="BackLower")
        pyrosim.Send_Sensor_Neuron(name=6, linkName="FrontLower")
        pyrosim.Send_Sensor_Neuron(name=7, linkName="LeftLower")
        pyrosim.Send_Sensor_Neuron(name=8, linkName="RightLower")

        pyrosim.Send_Motor_Neuron(name=9, jointName="Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name=10, jointName="Torso_FrontLeg")
        pyrosim.Send_Motor_Neuron(name=11, jointName="Torso_LeftLeg")
        pyrosim.Send_Motor_Neuron(name=12, jointName="Torso_RightLeg")
        pyrosim.Send_Motor_Neuron(name=13, jointName="BackLeg_BackLower")
        pyrosim.Send_Motor_Neuron(name=14, jointName="FrontLeg_FrontLower")
        pyrosim.Send_Motor_Neuron(name=15, jointName="LeftLeg_LeftLower")
        pyrosim.Send_Motor_Neuron(name=16, jointName="RightLeg_RightLower")

        rows = c.numSensorNeurons
        cols = c.numMotorNeurons
        for currentRow in range(rows):
            for currentColumn in range(cols):
                index = currentRow*cols + currentColumn
                pyrosim.Send_Synapse(sourceNeuronName=currentRow,
                                     targetNeuronName=currentColumn+rows, weight=self.body.weights.item(index))

        pyrosim.End()
