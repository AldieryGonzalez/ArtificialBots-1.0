import numpy

minSide = 0.3
maxSide = 1

# Snake Constants
minSensors = 2  # MUST BE LESS THAN MIN_LINKS
minLinks = 3  # MUST BE GREATER THAN MIN_SENSORS
maxLinks = 4


# Random Body Constants
maxTorsoSize = 1
minTorsoSize = 0.5
maxTorsos = 4

minLimbSize = 0.1
maxLimbSize = 1
limbTreeLimit = 3

torsoRatio = 0.5
limbRatio = 0.1

sensorChance = 0.5

# Fitness for playing football
steps = 2000
numberOfGenerations = 1
populationSize = 1
goalSize = (8, 0.5, 2)
goalPos = (0, 12, 1)

# DO NOT MODIFY
fl_amplitude, fl_frequency, fl_phaseOffset = numpy.pi/4, 10, 0
bl_amplitude, bl_frequency, bl_phaseOffset = numpy.pi/4, 10, numpy.pi/8
numSensorNeurons = 3
numMotorNeurons = 4
motorJointRange = 1.8
