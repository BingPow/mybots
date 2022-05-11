import numpy
import matplotlib.pyplot as plt

valueBackRight = numpy.load('data/backRightLegSensorValues.npy')
valueFrontRight = numpy.load('data/frontRightLegSensorValues.npy')
valueBackLeft = numpy.load('data/backLeftLegSensorValues.npy')
valueFrontLeft = numpy.load('data/frontLeftLegSensorValues.npy')
plt.plot(valueBackRight,linewidth=1)#linewidth=2, can be used but I think its unneeded
plt.plot(valueBackLeft,linewidth=1)#same here
plt.plot(valueFrontRight,linewidth=1)#linewidth=2, can be used but I think its unneeded
plt.plot(valueFrontLeft,linewidth=1)#same here
plt.legend(['Back Right Leg','Back Left Leg','Front Right Leg','Front Left Leg'])
plt.show()

# valueAngles = numpy.load('data/targetAngles.npy')
# plt.plot(valueAngles, (numpy.pi/4)*numpy.sin(10*valueAngles))
# plt.show()
