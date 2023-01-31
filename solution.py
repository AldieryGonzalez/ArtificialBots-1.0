import numpy
import pyrosim.pyrosim as pyrosim
import os
import random
import time


length, width, height = 1, 1, 1
x, y, z = 0, 0, (3 * height)/2


class SOLUTION:
    def __init__(self, myID):
        self.myID = myID
        self.weights = numpy.matrix([[numpy.random.rand(), numpy.random.rand()],
                                     [numpy.random.rand(), numpy.random.rand()],
                                     [numpy.random.rand(), numpy.random.rand()]]) * 2 - 1

    def Evaluate(self, directOrGui):
        self.Create_World()
        self.Generate_Body()
        self.Generate_Brain()
        os.system("start /B python3 simulate.py " +
                  directOrGui + " " + str(self.myID))
        while not os.path.exists(f"data/fitness{str(self.myID)}.txt"):
            time.sleep(0.01)
        f = open(f"data/fitness{str(self.myID)}.txt", "r")
        value = float(f.read())
        self.fitness = value
        print(self.fitness)
        f.close()

    def Mutate(self):
        randRow = random.randint(0, 2)
        randCol = random.randint(0, 1)
        randValue = random.random() * 2 - 1
        self.weights[randRow, randCol] = randValue

    def Set_ID(self, newID):
        self.myID = newID

    def Create_World(self):
        pyrosim.Start_SDF("generated/world.sdf")
        pyrosim.Send_Cube(name=f'Box', pos=[
            x + (2 * length), y + width, z], size=[length, width, height])
        pyrosim.End()

    def Generate_Body(self):
        pyrosim.Start_URDF("generated/body.urdf")
        pyrosim.Send_Cube(name=f'Torso', pos=[x, y, z], size=[
            length, width, height])

        pyrosim.Send_Joint(name="Torso_FrontLeg", parent="Torso", child="FrontLeg",
                           type="revolute", position=[x - length/2, 0, height])
        pyrosim.Send_Cube(name=f'FrontLeg', pos=[-length/2, 0, -height/2], size=[
            length, width, height])

        pyrosim.Send_Joint(name="Torso_BackLeg", parent="Torso", child="BackLeg",
                           type="revolute", position=[x + length/2, 0, height])
        pyrosim.Send_Cube(name=f'BackLeg', pos=[length/2, 0, -height/2], size=[
            length, width, height])
        pyrosim.End()

    def Generate_Brain(self):
        pyrosim.Start_NeuralNetwork(f"brain{self.myID}.nndf")

        pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
        pyrosim.Send_Sensor_Neuron(name=1, linkName="BackLeg")
        pyrosim.Send_Sensor_Neuron(name=2, linkName="FrontLeg")

        pyrosim.Send_Motor_Neuron(name=3, jointName="Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name=4, jointName="Torso_FrontLeg")

        rows = 3
        cols = 2
        for currentRow in range(rows):
            for currentColumn in range(cols):
                index = currentRow*2 + currentColumn
                pyrosim.Send_Synapse(sourceNeuronName=currentRow,
                                     targetNeuronName=currentColumn+rows, weight=self.weights.item(index))

        pyrosim.End()
