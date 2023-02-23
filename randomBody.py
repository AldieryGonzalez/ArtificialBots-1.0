import random
import constants as c
import pyrosim.pyrosim as pyrosim
import pybullet as p
import numpy


class Vector:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def __iter__(self):
        return iter((self.x, self.y, self.z))

    def __add__(self, other):
        if (isinstance(other, Vector)):
            return Vector(self.x + other.x, self.y + other.y, self.z + other.z)
        elif (isinstance(other, float)):
            num = other
            return Vector(self.x + num, self.y + num, self.z + num)
        elif (isinstance(other, int)):
            num = other
            return Vector(self.x + num, self.y + num, self.z + num)
        else:
            return self

    def __sub__(self, other):
        if (isinstance(other, Vector)):
            return Vector(self.x - other.x, self.y - other.y, self.z - other.z)
        elif (isinstance(other, float)):
            num = other
            return Vector(self.x - num, self.y - num, self.z - num)
        elif (isinstance(other, int)):
            num = other
            return Vector(self.x - num, self.y - num, self.z - num)
        else:
            return self

    def __mul__(self, other):
        if (isinstance(other, Vector)):
            return Vector(self.x * other.x, self.y * other.y, self.z * other.z)
        elif (isinstance(other, float)):
            num = other
            return Vector(self.x * num, self.y * num, self.z * num)
        elif (isinstance(other, int)):
            num = other
            return Vector(self.x * num, self.y * num, self.z * num)
        else:
            return self

    def __truediv__(self, other):
        if (isinstance(other, Vector)):
            return Vector(self.x / other.x, self.y / other.y, self.z / other.z)
        elif (isinstance(other, float)):
            num = other
            return Vector(self.x / num, self.y / num, self.z / num)
        elif (isinstance(other, int)):
            num = other
            return Vector(self.x / num, self.y / num, self.z / num)
        else:
            return self


class Box:
    def __init__(self, center: Vector, dimensions: Vector):
        self.center = center
        self.width = dimensions.x
        self.depth = dimensions.y
        self.height = dimensions.z
        self.dimensions = dimensions
        self.minPoint = Vector(
            center.x - self.width/2, center.y - self.depth/2, center.z - self.height/2)
        self.maxPoint = Vector(
            center.x + self.width/2, center.y + self.depth/2, center.z + self.height/2)

    def set_z(self, val):
        self.center = Vector(self.center.x, self.center.y, val)
        self.minPoint = Vector(self.center.x - self.width/2,
                               self.center.y - self.depth/2, self.center.z - self.height/2)
        self.maxPoint = Vector(self.center.x + self.width/2,
                               self.center.y + self.depth/2, self.center.z + self.height/2)

    def collides(self, other):
        b: Box = other
        return (
            self.minPoint.x <= b.maxPoint.x and
            self.maxPoint.x >= b.minPoint.x and
            self.minPoint.z <= b.maxPoint.y and
            self.maxPoint.z >= b.minPoint.y and
            self.minPoint.z <= b.maxPoint.z and
            self.maxPoint.z >= b.minPoint.z
        )


class Torso:
    def __init__(self, id=0, parent=None, parentPoint=None) -> None:
        self.id = f"torso{id}"
        self.senses = random.random() < c.sensorChance
        self.limbs = []  # each limb tree of less cube like objects
        # ID: (Torso, Point) | (Limb, Point)
        self.children: dict[int, tuple[Torso, Vector]] = {}
        self.Generate_Dimensions(parent, parentPoint)

    def createConnectedTorso(self, idNumber: int, restOfBody):
        newPoint = Vector(*tuple(random.randint(-1, 1) for i in range(3)))
        newTorso = Torso(idNumber, self, newPoint)
        retries = 0

        while newTorso.clipsWithBody(restOfBody):
            if retries > 3:
                return False
            newPoint = Vector(*tuple(random.randint(-1, 1) for i in range(3)))
            newTorso = Torso(idNumber, self, newPoint)
            retries = retries + 1

        # how im choosing to connect torsos

        self.children[idNumber] = (newTorso, newPoint)
        return newTorso

    def clipsWithBody(self, restOfBody: list[Box]):
        for box in restOfBody:
            if box.collides(self.box):
                return True
        return False

    def Generate_Dimensions(self, p, pp):
        width = random.uniform(c.minTorsoSize, c.maxTorsoSize)
        depth = random.uniform(c.minTorsoSize, c.maxTorsoSize)
        height = random.uniform(c.minTorsoSize, c.maxTorsoSize)
        biggest = max(width, depth, height)
        smallest = min(width, depth, height)
        while (smallest/biggest < c.torsoRatio):
            width = random.uniform(c.minTorsoSize, c.maxTorsoSize)
            depth = random.uniform(c.minTorsoSize, c.maxTorsoSize)
            height = random.uniform(c.minTorsoSize, c.maxTorsoSize)
            biggest = max(width, depth, height)
            smallest = min(width, depth, height)

        dim = Vector(width, depth, height)

        pos = Vector(0, 0, 0)
        if (p != None and pp != None):
            parent: Torso = p
            pointVector: Vector = pp
            parentConnectionPoint = (
                (parent.box.dimensions/2) * pointVector) + parent.box.center
            newCenter = ((dim/2) * pointVector) + parentConnectionPoint

            pos = newCenter
        self.box = Box(pos, dim)


class RandomBody:
    def __init__(self, bodyID):
        self.root = Torso()
        self.collisionBoxes: list[Box] = [self.root.box]
        self.Define_Body()
        self.bodyID = bodyID

    def Set_ID(self, newID):
        self.bodyID = newID
        self.Generate()

    def Generate(self):
        self.Generate_Body()
        self.Generate_Brain()

    def Define_Body(self):
        numberOfTorsos = random.randint(2, c.maxTorsos)
        torsos = [self.root]
        nextAvailableID = 1
        body = [self.root]
        sensors: list[Torso] = []
        joints: list[tuple[Torso, Torso]] = []

        while nextAvailableID < numberOfTorsos:
            parentTorso = random.choice(torsos)
            newTorso = parentTorso.createConnectedTorso(
                nextAvailableID, self.collisionBoxes)
            if newTorso == False:
                continue
            nextAvailableID = nextAvailableID + 1
            self.collisionBoxes.append(newTorso.box)
            torsos.append(newTorso)
            body.append(newTorso)
            joints.append((parentTorso, newTorso))

        lowestPoint = 0
        for box in self.collisionBoxes:
            lowestPoint = min(lowestPoint, box.minPoint.z)
        for link in body:
            link.box.set_z(link.box.center.z - lowestPoint)
            if link.senses:
                sensors.append(link)
        self.sensors = sensors
        self.joints = joints
        self.weights = numpy.matrix([[numpy.random.rand() for i in range(
            len(self.joints))] for j in range(len(self.sensors))]) * 2 - 1

    def Generate_Body(self):
        pyrosim.Start_URDF(f"generated/body{self.bodyID}.urdf")
        curTorso = self.root
        colorName, rgbaStr = "Cyan", "0 1 1 1"
        if (curTorso.senses):
            colorName, rgbaStr = "Green", "0 1 0 1"

        ###
        # Now Generating the Body
        ###
        pyrosim.Send_Cube(name=curTorso.id, pos=[*curTorso.box.center], size=[
            *curTorso.box.dimensions], colorName=colorName, rgbaStr=rgbaStr)

        def genNode(parentLink: Torso, vector: Vector):
            colorName, rgbaStr = "Cyan", "0 1 1 1"
            if (parentLink.senses):
                colorName, rgbaStr = "Green", "0 1 0 1"
            newCenter = (parentLink.box.dimensions / 2) * vector

            pyrosim.Send_Cube(name=parentLink.id,
                              pos=[*newCenter], size=[*parentLink.box.dimensions], colorName=colorName, rgbaStr=rgbaStr)
            for linkID in parentLink.children:
                childLink: Torso = parentLink.children[linkID][0]
                point = parentLink.children[linkID][1]
                jointPos = ((curTorso.box.dimensions / 2) * point)

                pyrosim.Send_Joint(name=f"{parentLink.id}_{childLink.id}", parent=parentLink.id, child=childLink.id,
                                   type="revolute", position=[*jointPos], jointAxis="0 1 0")
                genNode(childLink, point)

        for linkID in curTorso.children:
            childLink = curTorso.children[linkID][0]
            point = curTorso.children[linkID][1]

            jointPos = ((curTorso.box.dimensions / 2)
                        * point) + curTorso.box.center

            pyrosim.Send_Joint(name=f"{curTorso.id}_{childLink.id}", parent=curTorso.id, child=childLink.id,
                               type="revolute", position=[*jointPos], jointAxis="0 1 0")
            genNode(childLink, point)
        pyrosim.End()

    def Generate_Brain(self):
        pyrosim.Start_NeuralNetwork(f"generated/brain{self.bodyID}.nndf")
        nameIter = 0
        for sensorLink in self.sensors:
            pyrosim.Send_Sensor_Neuron(name=nameIter, linkName=sensorLink.id)
            nameIter += 1
        for parent, child in self.joints:
            pyrosim.Send_Motor_Neuron(
                name=nameIter, jointName=f"{parent.id}_{child.id}")
            nameIter += 1

        rows = len(self.sensors)
        cols = len(self.joints)
        for currentRow in range(rows):
            for currentColumn in range(cols):
                index = currentRow*cols + currentColumn
                pyrosim.Send_Synapse(sourceNeuronName=currentRow,
                                     targetNeuronName=currentColumn+rows, weight=self.weights.item(index))

        pyrosim.End()
