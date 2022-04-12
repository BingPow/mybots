from math import pi

#GRAVITY
grav = -9.8

#Neurons
numSensorNeurons = 11
numMotorNeurons = 8

#NUMBER OF STEPS, VERY IMPORTANT for FOR LOOP, SENSORS AND TARGET ANGLES
#usually 1000
steps = 1000

numberOfGenerations = 1 # could be 10

populationSize = 1

maxForce = 125

sleep = 1 / 240

motorJointRange = 0.75

#Amplitudes and Front Legs
amplitude = pi/4
frequency = 7
phaseOffSet = pi/4

length = 1
width = 1
height = 1
