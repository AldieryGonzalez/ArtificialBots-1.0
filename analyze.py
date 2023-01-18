import numpy
import matplotlib.pyplot as pyplot

backLegSensorValues = numpy.load("data/BackLeg-Data.npy")
frontLegSensorValues = numpy.load("data/FrontLeg-Data.npy")

pyplot.plot(backLegSensorValues, label="Back", linewidth=4)
pyplot.plot(frontLegSensorValues, label="Front")

pyplot.legend()

pyplot.show()
