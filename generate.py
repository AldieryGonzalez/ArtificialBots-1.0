import pyrosim.pyrosim as pyrosim
import random

length, width, height = 1, 1, 1
x, y, z = 0, 0, (3 * height)/2


def Create_World():
    pyrosim.Start_SDF("generated/world.sdf")
    pyrosim.Send_Cube(name=f'Box', pos=[
                      x + (2 * length), y + width, z], size=[length, width, height])
    pyrosim.End()


def Create_Robot():
    Generate_Body()
    Generate_Brain()


def Generate_Body():
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


def Generate_Brain():
    pyrosim.Start_NeuralNetwork("brain.nndf")

    pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
    pyrosim.Send_Sensor_Neuron(name=1, linkName="BackLeg")
    pyrosim.Send_Sensor_Neuron(name=2, linkName="FrontLeg")

    pyrosim.Send_Motor_Neuron(name=3, jointName="Torso_BackLeg")
    pyrosim.Send_Motor_Neuron(name=4, jointName="Torso_FrontLeg")

    for sensorName in range(3):
        for motorName in range(3, 5):
            w = (random.random() * 2) - 1
            pyrosim.Send_Synapse(sourceNeuronName=sensorName,
                                 targetNeuronName=motorName, weight=w)

    pyrosim.End()


Create_World()
Create_Robot()
