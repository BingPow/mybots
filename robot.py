import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import constants as c
import numpy

from sensor import SENSOR
from motor import MOTOR

class ROBOT:
    def __init__(self):
        self.sensors = {}
        self.motors = {}
        self.robot = p.loadURDF("body.urdf")
    #Removed robotId with robot
    def Prepare_To_Sense(self):
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    #some how get the dictionary to update the lists by link name
    def Sense(self,t):
        self.linkNamesList = list(self.sensors.keys())
        for i in range(len(self.sensors)):
            self.sensors[self.linkNamesList[i]].Get_Value(t)
        
    def Prepare_To_Act(self):
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)

    def Act(self,t):
        self.jointNamesList = list(self.motors.keys())
        for i in range(len(self.motors)):
            self.motors[self.jointNamesList[i]].Set_Value(self.robot,t)

    def Save_Values(self):
        pass
        #numpy.save('data/backLegSensorValues.npy',self.sensors"")
        #numpy.save('data/frontLegSensorValues.npy',frontLegSensorValues)
        #numpy.save('data/targetAngles.npy',targetAngles)