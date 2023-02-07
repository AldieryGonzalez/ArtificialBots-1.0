import numpy
import matplotlib.pyplot as pyplot

backLegSensorValues = numpy.load("data/BackLeg-Data.npy")
frontLegSensorValues = numpy.load("data/FrontLeg-Data.npy")
sinValues = numpy.load("data/sin.npy")

# pyplot.plot(backLegSensorValues, label="Back", linewidth=4)
# pyplot.plot(frontLegSensorValues, label="Front")
pyplot.plot(sinValues, label="Sin")

pyplot.legend()

pyplot.show()
