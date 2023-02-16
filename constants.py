import numpy

# Snake Constants
minSensors = 2  # MUST BE LESS THAN MIN_LINKS
minLinks = 4  # MUST BE GREATER THAN MIN_SENSORS
maxLinks = 8
minSide = 0.3
maxSide = 1

# Fitness for playing football
steps = 1200
numberOfGenerations = 3
populationSize = 3
goalSize = (8, 0.5, 2)
goalPos = (0, 12, 1)

# DO NOT MODIFY
fl_amplitude, fl_frequency, fl_phaseOffset = numpy.pi/4, 10, 0
bl_amplitude, bl_frequency, bl_phaseOffset = numpy.pi/4, 10, numpy.pi/8
numSensorNeurons = 3
numMotorNeurons = 4
motorJointRange = 1.8
