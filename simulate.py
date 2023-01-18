import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import time
import numpy
import random


def createRandAngle(intensity):
    legIntScale = intensity
    movementIntensity = (numpy.pi - numpy.pi/2) * legIntScale * 2
    legAngle = (random.random() * movementIntensity) - movementIntensity/2
    return legAngle


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -19.6)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("generated/body.urdf")
p.loadSDF("generated/world.sdf")

pyrosim.Prepare_To_Simulate(robotId)
steps = 1000

backLegSensorValues = numpy.zeros(steps)
frontLegSensorValues = numpy.zeros(steps)

fl_amplitude, fl_frequency, fl_phaseOffset = numpy.pi/4, 10, 0
bl_amplitude, bl_frequency, bl_phaseOffset = numpy.pi/4, 10, numpy.pi/8

targetAngles = (0, numpy.pi * 2)
piArr = numpy.linspace(targetAngles[0], targetAngles[1], steps)
fl_sinArr = numpy.sin((piArr * fl_frequency) + fl_phaseOffset) * fl_amplitude
bl_sinArr = numpy.sin((piArr * bl_frequency) + bl_phaseOffset) * bl_amplitude

# numpy.save("data/bl_sin.npy", bl_sinArr)
# numpy.save("data/fl_sin.npy", fl_sinArr)
# print(sinArr)
# exit()
for i in range(steps):
    backLegAngle = createRandAngle(0.5)
    frontLegAngle = createRandAngle(0.1)

    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link(
        "FrontLeg")

    pyrosim.Set_Motor_For_Joint(
        bodyIndex=robotId,
        jointName='Torso_BackLeg',
        controlMode=p.POSITION_CONTROL,
        targetPosition=bl_sinArr[i],
        maxForce=500)

    pyrosim.Set_Motor_For_Joint(
        bodyIndex=robotId,
        jointName='Torso_FrontLeg',
        controlMode=p.POSITION_CONTROL,
        targetPosition=fl_sinArr[i],
        maxForce=500)

    time.sleep(1 / 60)

numpy.save("data/BackLeg-Data.npy", backLegSensorValues)
numpy.save("data/FrontLeg-Data.npy", frontLegSensorValues)
p.disconnect()
