import numpy
import matplotlib.pyplot as pyplot

backLegSensorValues = numpy.load("data/init-data.npy")

pyplot.plot(backLegSensorValues)
pyplot.show()
