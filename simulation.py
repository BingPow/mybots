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
    def __init__(self, G, ID):
        self.frontRightLegSensorValues = numpy.zeros(1500)
        self.frontLeftLegSensorValues = numpy.zeros(1500)
        self.backRightLegSensorValues = numpy.zeros(1500)
        self.backLeftLegSensorValues = numpy.zeros(1500)

        self.directOrGUI = G

        if (G == "DIRECT"):
            self.physicsClient = p.connect(p.DIRECT)
        else:
            self.physicsClient = p.connect(p.GUI)
    
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,c.grav)
        self.world = WORLD()
        self.robot = ROBOT(ID)
        
        pyrosim.Prepare_To_Simulate(self.robot.robot)
        self.robot.Prepare_To_Sense()
        self.robot.Prepare_To_Act()

    def Run(self):
        for i in range (c.steps): #should be 1000
            p.stepSimulation()
            self.frontRightLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontRightLowerArm")   
            self.frontLeftLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeftLowerArm")   
            self.backRightLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackRightLowerArm")   
            self.backLeftLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeftLowerArm")   

            self.robot.Sense(i)
            self.robot.Think()
            self.robot.Act(i)
            self.robot.Get_Z_Fitness()
            if (self.directOrGUI == 'GUI'):
                time.sleep(c.sleep)
                
        numpy.save('data/frontRightLegSensorValues.npy',self.frontRightLegSensorValues)
        numpy.save('data/frontLeftLegSensorValues.npy',self.frontLeftLegSensorValues)
        numpy.save('data/backRightLegSensorValues.npy',self.backRightLegSensorValues)
        numpy.save('data/backLeftLegSensorValues.npy',self.backLeftLegSensorValues)
    
    def Get_Fitness(self):
        self.robot.Get_Fitness()

    def __del__(self):
        p.disconnect()  