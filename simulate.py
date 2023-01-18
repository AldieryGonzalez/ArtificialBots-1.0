import pybullet as p
import pybullet_data
import time
import numpy
import pyrosim.pyrosim as pyrosim

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")

pyrosim.Prepare_To_Simulate(robotId)
steps = 200
backLegSensorValues = numpy.zeros(steps)

for i in range(steps):
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    time.sleep(1 / 60)

print(backLegSensorValues)
numpy.save("data/init-data.npy", backLegSensorValues)
p.disconnect()
