import pyrosim.pyrosim as pyrosim
import random
import constants as c


def Generate_Snake(self):
    # Now to Generate the Body of the Snake
    pyrosim.Start_URDF("generated/body.urdf")
    for i, isSensor in enumerate(self.seedArray):
        # Create the Random Size of the Link
        linkWidth = (random.random() * (c.maxSide - c.minSide)) + c.minSide
        linkHeight = (random.random() *
                      (c.maxSide - c.minSide)) + c.minSide
        linkLength = (random.random() *
                      (c.maxSide - c.minSide)) + c.minSide
        colorName = "Cyan"
        rgbaStr = "0 1 1 1"
        if (isSensor):
            colorName = "Green"
            rgbaStr = "0 1 0 1"

        if (i == 0):
            pyrosim.Send_Cube(name=f'link{i}', pos=[0, 0, 0.5], size=[
                linkLength, linkWidth, linkHeight], colorName=colorName, rgbaStr=rgbaStr)
            pyrosim.Send_Joint(name=f"link{i}_link{i+1}", parent=f"link{i}", child=f"link{i+1}",
                               type="revolute", position=[linkLength/2, 0, 0.5], jointAxis="0 1 0")

        elif (i + 1 < len(self.seedArray)):
            # Make the Link
            pyrosim.Send_Cube(name=f'link{i}', pos=[linkLength/2, 0, 0], size=[
                linkLength, linkWidth, linkHeight], rgbaStr=rgbaStr, colorName=colorName)
            pyrosim.Send_Joint(name=f"link{i}_link{i+1}", parent=f"link{i}", child=f"link{i+1}",
                               type="revolute", position=[linkLength, 0, 0], jointAxis="0 1 0")
        else:
            # Make the Link
            pyrosim.Send_Cube(name=f'link{i}', pos=[linkLength/2, 0, 0], size=[
                linkLength, linkWidth, linkHeight], colorName=colorName, rgbaStr=rgbaStr)
    pyrosim.End()
    self.Generate_Snake_Brain()


def Generate_Snake_Brain(self):
    pyrosim.Start_NeuralNetwork(f"brain{self.myID}.nndf")
    nameIter = 0
    for i, isSensor in enumerate(self.seedArray):
        if isSensor:
            pyrosim.Send_Sensor_Neuron(name=nameIter, linkName=f"link{i}")
            nameIter += 1
    for i, isSensor in enumerate(self.seedArray):
        if i + 1 < len(self.seedArray):
            pyrosim.Send_Motor_Neuron(
                name=nameIter, jointName=f"link{i}_link{i+1}")
            nameIter += 1

    # pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
    # pyrosim.Send_Sensor_Neuron(name=1, linkName="LeftFoot")
    # pyrosim.Send_Sensor_Neuron(name=2, linkName="RightFoot")

    # pyrosim.Send_Motor_Neuron(name=3, jointName="Torso_LeftLeg")
    # pyrosim.Send_Motor_Neuron(name=4, jointName="Torso_RightLeg")
    # pyrosim.Send_Motor_Neuron(name=5, jointName="LeftLeg_LeftLower")
    # pyrosim.Send_Motor_Neuron(name=6, jointName="RightLeg_RightLower")

    rows = self.numSensors
    cols = self.numMotors
    for currentRow in range(rows):
        for currentColumn in range(cols):
            index = currentRow*cols + currentColumn
            pyrosim.Send_Synapse(sourceNeuronName=currentRow,
                                 targetNeuronName=currentColumn+rows, weight=self.weights.item(index))

    pyrosim.End()
