import numpy
import matplotlib.pyplot

valueBack = numpy.load('data/backLegSensorValues.npy')
valueFront = numpy.load('data/frontLegSensorValues.npy')
matplotlib.pyplot.plot(valueBack,linewidth=2)#linewidth=2, can be used but I think its unneeded
matplotlib.pyplot.plot(valueFront,linewidth=2)#same here
matplotlib.pyplot.legend(['Back Leg','Front Leg'])
matplotlib.pyplot.show()