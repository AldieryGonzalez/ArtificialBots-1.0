import random
import constants as c
import pyrosim.pyrosim as pyrosim
import pybullet as p


class Torso:
    def __init__(self, id, usedConnections: list[tuple[int]]) -> None:
        self.id = id
        self.senses = random.random() < c.sensorChance
        self.usedPoints = usedConnections
        self.limbs = []  # each limb tree of less cube like objects
        self.connections = {}  # ID: (Torso, Point) | (Limb, Point)
        self.Generate_Dimensions()

    def createConnectedTorso(self, id):
        newPoint = tuple(random.randint(-1, 1) for i in range(3))

        while newPoint in self.usedPoints:
            newPoint = tuple(random.randint(-1, 1) for i in range(3))

        # how im choosing to connect torsos
        newTorso = Torso(id, [newPoint] * -1)
        self.usedPoints.append(newPoint)
        self.connections[id] = {"torso": newTorso, "point": newPoint}
        newPoint = tuple(-1*x for x in newPoint)
        newTorso.connections[self.id] = {"torso": self, "point": newPoint}
        return newTorso

    def Generate_Dimensions(self):
        width = random.uniform(c.minTorsoSize, c.maxTorsoSize)
        length = random.uniform(c.minTorsoSize, c.maxTorsoSize)
        height = random.uniform(c.minTorsoSize, c.maxTorsoSize)
        biggest = max(width, length, height)
        smallest = min(width, length, height)
        while (smallest/biggest < c.torsoRatio):
            width = random.uniform(c.minTorsoSize, c.maxTorsoSize)
            length = random.uniform(c.minTorsoSize, c.maxTorsoSize)
            height = random.uniform(c.minTorsoSize, c.maxTorsoSize)
            biggest = max(width, length, height)
            smallest = min(width, length, height)
        self.width = width
        self.length = length
        self.height = height


class RandomBody:
    def __init__(self):
        self.torsos: list[Torso] = []
        self.torsos.append(Torso(0, []))
        self.body = self.Define_Body()

    def Generate(self):
        self.Generate_Body()
        self.Generate_Brain()

    def Define_Body(self):
        body = {}

        for i in range(1, c.maxTorsos):
            torsoNumber = random.randrange(0, i)
            newTorso = self.torsos[torsoNumber].createConnectedTorso(i)
            self.torsos.append(newTorso)

    def Generate_Body(self):
        pyrosim.Start_URDF("generated/body.urdf")
        linkNumber = 0
        curTorso = self.torsos[0]
        colorName, rgbaStr = "Cyan", "0 1 1 1"
        if (curTorso.senses):
            colorName, rgbaStr = "Green", "0 1 0 1"

        ###
        # Now Generating the Body
        ###
        pyrosim.Send_Cube(name=f'torso0', pos=[0, 0, curTorso.height/2], size=[
            curTorso.length, curTorso.width, curTorso.height], colorName=colorName, rgbaStr=rgbaStr)

        def genNode(node: Torso, vector, prevID):
            colorName, rgbaStr = "Cyan", "0 1 1 1"
            if (node.senses):
                colorName, rgbaStr = "Green", "0 1 0 1"
            x = node.length / (2 * vector[0])
            y = node.width / (2 * vector[1])
            z = node.height / (2 * vector[2])
            pyrosim.Send_Cube(name=f'torso{node.id}', pos=[x, y, z], size=[
                node.length, node.width, node.height], colorName=colorName, rgbaStr=rgbaStr)
            for torsoID in node.connections:
                if torsoID == prevID:
                    continue
                torso: Torso = node.connections[torsoID]["torso"]
                point = node.connections[torsoID]["point"]
                pyrosim.Send_Joint(name=f"torso0_torso{torsoID}", parent=f"torso0", child=f"torso{torsoID}",
                                   type="revolute", position=[x, y, z], jointAxis="0 1 0")
                genNode(torso, point, node.id)

        for torsoID in curTorso.connections:
            torso: Torso = curTorso.connections[torsoID]["torso"]
            point = curTorso.connections[torsoID]["point"]
            x = torso.length / (2 * point[0])
            y = torso.width / (2 * point[1])
            z = (torso.height/2) * point[2] + 3 + curTorso.height/2
            pyrosim.Send_Joint(name=f"torso0_torso{torsoID}", parent=f"torso0", child=f"torso{torsoID}",
                               type="revolute", position=[x, y, z], jointAxis="0 1 0")
            genNode(torso, point, curTorso.id)
        pyrosim.End()

    def Generate_Brain(self):
        pass


RandomBody()
