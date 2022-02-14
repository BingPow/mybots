from cmath import sin
import time
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy
import random
from math import pi
import matplotlib.pylab as plt

#NUMBER OF STEPS, VERY IMPORTANT for FOR LOOP, SENSORS AND TARGET ANGLES
steps = 1000

amplitudeBL = pi/4
frequencyBL = 7
phaseOffSetBL = pi/4
amplitudeFL = pi/4
frequencyFL = 1
phaseOffSetFL = pi/8
#amplitude * sin(frequency * i + phaseOffset)
#targetAngles = numpy.linspace(0,2*pi,steps)
#numpy.save('data/targetAngles.npy',targetAngles)
#exit()
targetAnglesFL = amplitudeBL*numpy.sin(frequencyBL*numpy.linspace(0,2*pi, steps) + phaseOffSetBL)
targetAnglesBL = amplitudeFL*numpy.sin(frequencyFL*numpy.linspace(0,2*pi, steps) + phaseOffSetFL)
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")
pyrosim.Prepare_To_Simulate(robotId)
backLegSensorValues = numpy.zeros(steps)
frontLegSensorValues = numpy.zeros(steps)

for i in range (steps): #should be 1000
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    #Back Leg
    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = "Torso_BackLeg",
        controlMode = p.POSITION_CONTROL,
        targetPosition = targetAnglesBL[i],
        maxForce = 10)
    #Front Leg
    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = "Torso_FrontLeg",
        controlMode = p.POSITION_CONTROL,
        targetPosition = targetAnglesFL[i],
        maxForce = 10)
    time.sleep(1/60)

numpy.save('data/backLegSensorValues.npy',backLegSensorValues)
numpy.save('data/frontLegSensorValues.npy',frontLegSensorValues)
#numpy.save('data/targetAngles.npy',targetAngles)

p.disconnect()

