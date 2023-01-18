import pybullet as p
import pybullet_data
import time
import numpy
import pyrosim.pyrosim as pyrosim

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -19.6)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("generated/body.urdf")
p.loadSDF("generated/world.sdf")

pyrosim.Prepare_To_Simulate(robotId)
steps = 600
backLegSensorValues = numpy.zeros(steps)
frontLegSensorValues = numpy.zeros(steps)

for i in range(steps):
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link(
        "FrontLeg")

    pyrosim.Set_Motor_For_Joint(
        bodyIndex=robotId,
        jointName='Torso_BackLeg',
        controlMode=p.POSITION_CONTROL,
        targetPosition=0.0,
        maxForce=500)

    time.sleep(1 / 60)

print(backLegSensorValues)
numpy.save("data/BackLeg-Data.npy", backLegSensorValues)
numpy.save("data/FrontLeg-Data.npy", frontLegSensorValues)
p.disconnect()
