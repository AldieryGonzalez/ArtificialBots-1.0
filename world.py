import pybullet as p


class WORLD:
    def __init__(self):
        p.loadSDF("generated/world.sdf")
        self.planeId = p.loadURDF("plane.urdf")
