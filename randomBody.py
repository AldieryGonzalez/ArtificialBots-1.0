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

    def toString(self, invert=False):
        if invert:
            return f"{abs(abs(self.x) - 1)} {abs(abs(self.y) - 1)} {abs(abs(self.z) - 1)}"
        return f"{abs(self.x)} {abs(self.y)} {abs(self.z)}"

    def Mutate(self):
        choice = random.randint(1, 3)
        if choice == 1:
            temp = self.x + random.uniform(-0.5, 0.5)
            if temp > c.minLimbSize and temp < c.maxLimbSize:
                self.x = temp
        elif choice == 2:
            temp = self.y + random.uniform(-0.5, 0.5)
            if temp > c.minLimbSize and temp < c.maxLimbSize:
                self.y = temp
        else:
            temp = self.z + random.uniform(-0.5, 0.5)
            if temp > c.minLimbSize and temp < c.maxLimbSize:
                self.z = temp

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

    def __eq__(self, other) -> bool:
        if (isinstance(other, Vector)):
            return self.toString() == other.toString()
        else:
            return False


def randomBetweenRounded(min, max, decimalPlaces):
    randomNum = random.uniform(min, max)
    return round(randomNum, decimalPlaces)


def createValidVector() -> Vector:
    points = [1, 0, 0]
    random.shuffle(points)
    return Vector(*points)


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

    def set_x(self, val):
        self.center = Vector(val, self.center.y, self.center.z)
        self.minPoint = Vector(self.center.x - self.width/2,
                               self.center.y - self.depth/2, self.center.z - self.height/2)
        self.maxPoint = Vector(self.center.x + self.width/2,
                               self.center.y + self.depth/2, self.center.z + self.height/2)

    def set_y(self, val):
        self.center = Vector(self.center.x, val, self.center.z)
        self.minPoint = Vector(self.center.x - self.width/2,
                               self.center.y - self.depth/2, self.center.z - self.height/2)
        self.maxPoint = Vector(self.center.x + self.width/2,
                               self.center.y + self.depth/2, self.center.z + self.height/2)

    def get_ratio(self):
        biggest = max(self.width, self.depth, self.height)
        smallest = min(self.width, self.depth, self.height)
        return smallest/biggest

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


class BodyPart:
    def __init__(self, id, parent, parentPoint, ):
        self.id = id

    def Generate_Box(self, parent, parentPoint):
        pass


class Limb():
    def __init__(self, id, parent, parentPoint: Vector, depth):
        self.id = f'limb{id}'
        self.limbDepth = depth
        self.senses = True
        self.children: dict[str, tuple[Limb, Vector, Vector]] = {}
        self.usedPoints: list[Vector] = []
        self.Generate_Box(parent, parentPoint)
        self.parent = parent

    # def createConnectedLimb(self, idNumber: int, restOfBody: list[Box]):
    #     newPoint = Vector(*tuple(random.randint(-1, 1) for i in range(3)))
    #     # newLimb = Limb(idNumber, newPoint)
    #     retries = 0

    def createConnectedLimb(self, idNumber, restOfBody):
        if self.limbDepth >= c.limbTreeLimit:
            return False
        newPoint = createValidVector()
        newLimb = Limb(idNumber, self, newPoint, self.limbDepth + 1)
        retries = 4
        while ((newPoint in self.usedPoints) or newLimb.clipsWithBody(restOfBody)):
            if retries < 1:
                return False
            newPoint = createValidVector()
            newLimb = Limb(idNumber, self, newPoint, self.limbDepth + 1)
            retries = retries - 1

        self.children[f"limb{idNumber}"] = (
            newLimb, newPoint, createValidVector())
        self.usedPoints.append(newPoint)
        newLimb.usedPoints.append(newPoint * -1)
        return newLimb

    def clipsWithBody(self, restOfBody: list[Box]):
        for box in restOfBody:
            if box.collides(self.box):
                return True
        return False

    def Generate_Box(self, parentLimb, parentPoint):
        width = randomBetweenRounded(c.minLimbSize, c.maxLimbSize, 4)
        depth = randomBetweenRounded(c.minLimbSize, c.maxLimbSize, 4)
        height = randomBetweenRounded(c.minLimbSize, c.maxLimbSize, 4)
        biggest = max(width, depth, height)
        smallest = min(width, depth, height)
        while (smallest/biggest > c.limbRatio):
            width = randomBetweenRounded(c.minLimbSize, c.maxLimbSize, 4)
            depth = randomBetweenRounded(c.minLimbSize, c.maxLimbSize, 4)
            height = randomBetweenRounded(c.minLimbSize, c.maxLimbSize, 4)
            biggest = max(width, depth, height)
            smallest = min(width, depth, height)

        dim = Vector(width, depth, height)

        pos = Vector(0, 0, 0)
        if (parentLimb != None and parentPoint != None):
            parent: Torso = parentLimb
            pointVector: Vector = parentPoint
            parentConnectionPoint = (
                (parent.box.dimensions/2) * pointVector) + parent.box.center
            newCenter = ((dim/2) * pointVector) + parentConnectionPoint

            pos = newCenter
        self.box = Box(pos, dim)


class Torso():
    def __init__(self, id=0, parent=None, parentPoint=None) -> None:
        self.id = f"torso{id}"
        self.senses = False
        self.children = {}
        self.usedPoints = []
        self.Generate_Dimensions(parent, parentPoint)
        self.parent = parent

    def createConnectedTorso(self, idNumber, restOfBody):
        newPoint = createValidVector()
        newTorso = Torso(idNumber, self, newPoint)
        retries = 3

        while ((newPoint in self.usedPoints) or newTorso.clipsWithBody(restOfBody)):

            if retries < 1:
                return False
            newPoint = createValidVector()
            newTorso = Torso(idNumber, self, newPoint)
            retries = retries - 1

        self.children[f"torso{idNumber}"] = (newTorso, newPoint, newPoint)
        self.usedPoints.append(newPoint)
        newTorso.usedPoints.append(newPoint * -1)
        return newTorso

    def createConnectedLimb(self, idNumber, restOfBody):
        newPoint = createValidVector()
        newLimb = Limb(idNumber, self, newPoint, 1)
        retries = 4
        while ((newPoint in self.usedPoints) or newLimb.clipsWithBody(restOfBody)):
            if retries < 1:
                return False
            newPoint = createValidVector()
            newLimb = Limb(idNumber, self, newPoint, 1)
            retries = retries - 1

        self.children[f"limb{idNumber}"] = (
            newLimb, newPoint, createValidVector())
        self.usedPoints.append(newPoint)
        newLimb.usedPoints.append(newPoint * -1)
        return newLimb

    def clipsWithBody(self, restOfBody: list[Box]):
        for box in restOfBody:
            if box.collides(self.box):
                return True
        return False

    def Generate_Dimensions(self, parentLimb, parentPoint):
        width = randomBetweenRounded(c.minTorsoSize, c.maxTorsoSize, 1)
        depth = randomBetweenRounded(c.minTorsoSize, c.maxTorsoSize, 1)
        height = randomBetweenRounded(c.minTorsoSize, c.maxTorsoSize, 1)
        biggest = max(width, depth, height)
        smallest = min(width, depth, height)
        while (smallest/biggest < c.torsoRatio):
            width = randomBetweenRounded(c.minTorsoSize, c.maxTorsoSize, 1)
            depth = randomBetweenRounded(c.minTorsoSize, c.maxTorsoSize, 1)
            height = randomBetweenRounded(c.minTorsoSize, c.maxTorsoSize, 1)
            biggest = max(width, depth, height)
            smallest = min(width, depth, height)

        dim = Vector(width, depth, height)

        pos = Vector(0, 0, 0)
        if (parentLimb != None and parentPoint != None):
            parent: Torso = parentLimb
            pointVector: Vector = parentPoint
            parentConnectionPoint = (
                (parent.box.dimensions/2) * pointVector) + parent.box.center
            newCenter = ((dim/2) * pointVector) + parentConnectionPoint

            pos = newCenter
        self.box = Box(pos, dim)


def doWithXChance(func, chance):
    if random.random() < chance:
        func()


class RandomBody:
    def __init__(self, bodyID):
        self.bodyID = bodyID
        random.seed(c.seed + bodyID)
        self.root = Torso()
        self.collisionBoxes: list[Box] = [self.root.box]
        self.Define_Body()

    def Set_ID(self, newID):
        self.bodyID = newID

    def Mutate(self):
        doWithXChance(self.Mutate_Body_Part_Size, 0.1)
        doWithXChance(self.Add_Remove_Body_Part, 0.1)
        doWithXChance(self.Mutate_Brain_Structure, 0.8)
        # self.Mutate_Joints()

    def Mutate_Body_Part_Size(self):
        randomLink = random.choice(self.body)
        randomLink.box.dimensions.Mutate()

    def Add_Remove_Body_Part(self):
        randomLink = random.choice(self.body[1:])
        add = random.choice([True, False])

        if add:
            retries = 3
            while retries > 0:
                if isinstance(randomLink, Torso):
                    if random.choice([True, False]):
                        newLink = randomLink.createConnectedTorso(
                            self.nextID, self.collisionBoxes)
                        if newLink:
                            self.nextID = self.nextID + 1
                            self.collisionBoxes.append(newLink.box)
                            self.body.append(newLink)
                            self.Add_Weights(joint=True, sensor=False)
                            self.joints.append((randomLink, newLink))
                            self.Adjust_Height()
                            return newLink
                        else:
                            retries = retries - 1
                            continue
                newLink = randomLink.createConnectedLimb(
                    self.nextID, self.collisionBoxes)
                if newLink:
                    self.nextID = self.nextID + 1
                    self.collisionBoxes.append(newLink.box)
                    self.body.append(newLink)
                    self.Add_Weights(joint=True, sensor=newLink.senses)
                    self.joints.append((randomLink, newLink))
                    self.sensors.append(newLink)
                    self.Adjust_Height()
                    return newLink
                else:
                    retries = retries - 1
                    continue
        else:
            if (randomLink.parent != None) and self.Remove_Link(randomLink):
                del randomLink.parent.children[randomLink.id]
            # self.Adjust_Sensors()

    def Add_Weights(self, joint, sensor):
        numJoints = len(self.joints)
        if (joint):
            arr = numpy.insert(self.weights, numJoints,
                               [numpy.random.rand() for i in range(len(self.sensors))], 1)
            self.weights = arr
            numJoints += 1
        if (sensor):
            arr = numpy.insert(self.weights, len(self.sensors),
                               [numpy.random.rand() for i in range(numJoints)], 0)
            self.weights = arr

    def Remove_Weights(self, jInd, sInd):
        if (jInd != -1):
            arr = numpy.delete(self.weights, jInd, 1)
            self.weights = arr
        if (sInd != -1):
            arr = numpy.delete(self.weights, sInd, 0)
            self.weights = arr

    def Remove_Link(self, link):
        if (link.parent != None) and (link.parent.parent != None):
            for child in link.children.values():
                self.Remove_Link(child[0])
            if (len(self.sensors) < 2 or len(self.joints) < 2):
                return False
            self.collisionBoxes.remove(link.box)
            self.body.remove(link)
            ji = -1
            si = -1
            try:
                ji = self.joints.index((link.parent, link))
            except:
                pass
            try:
                si = self.sensors.index(link)
            except:
                pass
            self.Remove_Weights(ji, si)
            self.joints.remove((link.parent, link))
            try:
                self.sensors.remove(link)
            except:
                pass
            return True
        return False

    def Adjust_Sensors(self):
        if (len(self.sensors) < 1):
            part = random.choice(self.body)
            part.senses = True
            self.sensors.append(part)

    def Mutate_Joints(self):
        pass

    def Mutate_Brain_Structure(self):
        randomLink = random.choice(self.body[1:])
        if (randomLink.senses):
            if (len(self.sensors) > 2):
                randomLink.senses = False
                self.Remove_Weights(
                    jInd=-1, sInd=self.sensors.index(randomLink))
                self.sensors.remove(randomLink)
        else:
            randomLink.senses = True
            self.Add_Weights(joint=False, sensor=True)
            self.sensors.append(randomLink)

    def Generate(self):
        sensors = list(map(lambda s: s.id, self.sensors))
        joints = list(map(lambda j: (j[0].id, j[1].id), self.joints))
        self.Generate_Body()
        self.Generate_Brain()

    def Redefine_Body(self):
        pass

    def Adjust_Height(self):
        lowestPoint = 0
        for box in self.collisionBoxes:
            lowestPoint = min(lowestPoint, box.minPoint.z)
        for link in self.body:
            link.box.set_z(link.box.center.z - lowestPoint)

    def Define_Body(self):
        numberOfTorsos = random.randint(2, c.maxTorsos)
        torsos = []
        torsos.append(self.root)
        nextAvailableID = 1
        body = []
        body.append(self.root)
        sensors = []
        joints = []

        while nextAvailableID < numberOfTorsos:
            parentPart = random.choice(torsos)
            newTorso = parentPart.createConnectedTorso(
                nextAvailableID, self.collisionBoxes)
            if newTorso == False:
                numberOfTorsos = numberOfTorsos - 1
                continue
            nextAvailableID = nextAvailableID + 1
            self.collisionBoxes.append(newTorso.box)
            torsos.append(newTorso)
            body.append(newTorso)
            joints.append((parentPart, newTorso))

        numOfLimbsWant = random.randint(1, c.baseLimbMax)
        numOfLimbs = 0
        retries = 100
        while numOfLimbs < numOfLimbsWant:
            parentPart = random.choice(body)
            newLimb = parentPart.createConnectedLimb(
                nextAvailableID, self.collisionBoxes)
            if newLimb == False:
                retries = retries - 1
                if retries < 1:
                    retries = 100
                    numOfLimbsWant = numOfLimbsWant - 1
                continue
            nextAvailableID = nextAvailableID + 1
            numOfLimbs = numOfLimbs + 1
            self.collisionBoxes.append(newLimb.box)
            body.append(newLimb)
            joints.append((parentPart, newLimb))

        lowestPoint = 0
        for box in self.collisionBoxes:
            lowestPoint = min(lowestPoint, box.minPoint.z)
        for link in body:
            link.box.set_z(link.box.center.z - lowestPoint)
            if link.senses:
                sensors.append(link)
        self.nextID = nextAvailableID
        self.sensors = sensors
        self.joints = joints
        self.body = body
        self.weights = numpy.array([[numpy.random.rand() for i in range(
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
        # for debugging
        # ID_to_color = {"torso0": ("Black", "0 0 0 1"),
        #                "torso1": ("Blue", "0 0 1 1"),
        #                "torso2": ("Green", "0 1 0 1"),
        #                "torso3": ("Cyan", "0 1 1 1")}
        # colorName, rgbaStr = ID_to_color[curTorso.id]
        pyrosim.Send_Cube(name=curTorso.id, pos=[*curTorso.box.center], size=[
            *curTorso.box.dimensions], colorName=colorName, rgbaStr=rgbaStr)

        def genNode(newLink, vector: Vector):
            colorName, rgbaStr = "Cyan", "0 1 1 1"
            if (newLink.senses):
                colorName, rgbaStr = "Green", "0 1 0 1"
            newCenter = (newLink.box.dimensions / 2) * vector

            # colorName, rgbaStr = ID_to_color[newLink.id]

            pyrosim.Send_Cube(name=newLink.id,
                              pos=[*newCenter], size=[*newLink.box.dimensions], colorName=colorName, rgbaStr=rgbaStr)
            for linkID in newLink.children:
                childLink = newLink.children[linkID][0]
                childPoint = newLink.children[linkID][1]
                axis = newLink.children[linkID][2]
                jointPos = ((newLink.box.dimensions / 2)
                            * childPoint) + newCenter
                jointAxis = axis.toString()
                jointType = "revolute"

                pyrosim.Send_Joint(name=f"{newLink.id}_{childLink.id}", parent=newLink.id, child=childLink.id,
                                   type=jointType, position=[*jointPos], jointAxis=jointAxis)
                genNode(childLink, childPoint)

        for linkID in curTorso.children:
            childLink = curTorso.children[linkID][0]
            point = curTorso.children[linkID][1]
            axis = curTorso.children[linkID][2]
            jointPos = ((curTorso.box.dimensions / 2)
                        * point) + curTorso.box.center
            jointAxis = axis.toString()
            jointType = "revolute"

            pyrosim.Send_Joint(name=f"{curTorso.id}_{childLink.id}", parent=curTorso.id, child=childLink.id,
                               type=jointType, position=[*jointPos], jointAxis=jointAxis)
            genNode(childLink, point)
        pyrosim.End()

    def Generate_Brain(self):
        pyrosim.Start_NeuralNetwork(f"brain{self.bodyID}.nndf")
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
