import numpy

minSide = 0.3
maxSide = 1

pickleEveryXGens = 50

# Random Body Constants
maxTorsoSize = 1
minTorsoSize = 0.5
maxTorsos = 3  # Max amount of Torsos ON GENERATION

minLimbSize = 0.1
maxLimbSize = 1.9
limbTreeLimit = 3
baseLimbMax = 3

torsoRatio = 0.5
limbRatio = 0.3

motorJointRange = 1.2
sensorChance = 0.5
seed = 100

# Fitness for playing football
steps = 1000
numberOfGenerations = 500
populationSize = 10
goalSize = (8, 0.5, 2)
goalPos = (0, 12, 1)

# DO NOT MODIFY
fl_amplitude, fl_frequency, fl_phaseOffset = numpy.pi/4, 10, 0
bl_amplitude, bl_frequency, bl_phaseOffset = numpy.pi/4, 10, numpy.pi/8
numSensorNeurons = 3
numMotorNeurons = 4
