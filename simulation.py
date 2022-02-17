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

from world import WORLD
from robot import ROBOT

class SIMULATION:
    def __init__(self):
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,c.grav)
        self.world = WORLD()
        self.robot = ROBOT()
        pyrosim.Prepare_To_Simulate(self.robot.robotId)
        self.robot.Prepare_To_Sense()

    def Run(self):
        for i in range (c.steps): #should be 1000
            print(i)
            p.stepSimulation()
            self.robot.Sense(i)
            '''
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
            '''
            time.sleep(c.sleep)

    def __del__(self):

        p.disconnect()  