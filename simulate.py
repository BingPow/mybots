from cmath import sin
import time
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy
import random
from math import pi
import matplotlib.pylab as plt
import constants as c
#amplitude * sin(frequency * i + phaseOffset)
#targetAngles = numpy.linspace(0,2*pi,steps)
#numpy.save('data/targetAngles.npy',targetAngles)
#exit()
targetAnglesFL = c.amplitudeBL*numpy.sin(c.frequencyBL*numpy.linspace(0,2*pi, c.steps) + c.phaseOffSetBL)
targetAnglesBL = c.amplitudeFL*numpy.sin(c.frequencyFL*numpy.linspace(0,2*pi, c.steps) + c.phaseOffSetFL)
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,c.grav)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")
pyrosim.Prepare_To_Simulate(robotId)
backLegSensorValues = numpy.zeros(c.steps)
frontLegSensorValues = numpy.zeros(c.steps)

for i in range (c.steps): #should be 1000
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    #Back Leg
    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = "Torso_BackLeg",
        controlMode = p.POSITION_CONTROL,
        targetPosition = targetAnglesBL[i],
        maxForce = c.maxForce)
    #Front Leg
    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = "Torso_FrontLeg",
        controlMode = p.POSITION_CONTROL,
        targetPosition = targetAnglesFL[i],
        maxForce = c.maxForce)
    time.sleep(c.sleep)

numpy.save('data/backLegSensorValues.npy',backLegSensorValues)
numpy.save('data/frontLegSensorValues.npy',frontLegSensorValues)
#numpy.save('data/targetAngles.npy',targetAngles)

p.disconnect()

