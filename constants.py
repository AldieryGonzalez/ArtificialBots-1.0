import numpy

steps = 1200
numberOfGenerations = 10
populationSize = 10
goalSize = (8, 0.5, 2)
goalPos = (0, 12, 1)

# DO NOT MODIFY
fl_amplitude, fl_frequency, fl_phaseOffset = numpy.pi/4, 10, 0
bl_amplitude, bl_frequency, bl_phaseOffset = numpy.pi/4, 10, numpy.pi/8
numSensorNeurons = 3
numMotorNeurons = 4
motorJointRange = 1.8
