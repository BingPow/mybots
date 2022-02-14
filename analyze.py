import numpy
import matplotlib.pyplot as plt

#valueBack = numpy.load('data/backLegSensorValues.npy')
#valueFront = numpy.load('data/frontLegSensorValues.npy')
#plt.plot(valueBack,linewidth=2)#linewidth=2, can be used but I think its unneeded
#plt.plot(valueFront,linewidth=2)#same here
#plt.legend(['Back Leg','Front Leg'])
#plt.show()

valueAngles = numpy.load('data/targetAngles.npy')
plt.plot(valueAngles, (numpy.pi/4)*numpy.sin(10*valueAngles))
plt.show()
