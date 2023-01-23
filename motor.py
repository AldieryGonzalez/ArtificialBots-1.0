import pyrosim.pyrosim as pyrosim
import pybullet as p
import numpy
import constants as c


class MOTOR:
    def __init__(self, jointName):
        self.jointName = jointName
        # self.Prepare_To_Act()

    # def Prepare_To_Act(self):
    #     self.amplitude = 1
    #     self.frequency = 1 if self.jointName == "Torso_FrontLeg" else 0.5
    #     self.offset = 0
    #     targetAngles = (0, numpy.pi * 2)
    #     piArr = numpy.linspace(targetAngles[0], targetAngles[1], c.steps)
    #     self.motorValues = numpy.sin((piArr * self.frequency) +
    #                                  self.offset) * self.amplitude

    def Set_Value(self, desiredAngle, robot):
        pyrosim.Set_Motor_For_Joint(
            bodyIndex=robot.robotId,
            jointName=self.jointName,
            controlMode=p.POSITION_CONTROL,
            targetPosition=desiredAngle,
            maxForce=500)

    # def Save_Values(self):
    #     numpy.save(f"data/{self.jointName}_MOTOR.npy", self.motorValues)
