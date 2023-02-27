import pybullet as p
import numpy as np
import os


class WORLD:
    def __init__(self):
        self.planeId = p.loadURDF("plane.urdf")
        self.objects = p.loadSDF("generated/world.sdf")

    def Get_Ball_Fitness(self, id):
        ballPosAndOrientation = p.getBasePositionAndOrientation(
            self.objects[0])
        goalPosAndOrientation = p.getBasePositionAndOrientation(
            self.objects[1])
        ballPosition = np.array(ballPosAndOrientation[0])
        goalPosition = np.array(goalPosAndOrientation[0])
        sumSQ = np.sum(np.square(ballPosition - goalPosition))
        dist = np.sqrt(sumSQ)

        f = open(f"tmp{id}.txt", "w")
        f.write(str(dist))
        f.close()
        os.rename(f"tmp{id}.txt",
                  f"fitness{id}.txt")
