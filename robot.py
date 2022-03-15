import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
from pyrosim.neuralNetwork import NEURAL_NETWORK
import constants as c
import numpy

from sensor import SENSOR
from motor import MOTOR

class ROBOT:
    def __init__(self):
        self.sensors = {}
        self.motors = {}
        self.robot = p.loadURDF("body.urdf")
        self.nn = NEURAL_NETWORK("brain.nndf")

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
        '''
        for neuronName in self.nn.Get_Neuron_Names():
            if (self.nn.Is_Motor_Neuron(neuronName)):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName)
                self.motors[jointName].Set_Value(self.robot,desiredAngle)
        '''
        for i in range(len(self.motors)):
            self.motors[self.jointNamesList[i]].Set_Value(self.robot,t)
        

    def Think(self):
        self.nn.Update()
        self.nn.Print()

    def Save_Values(self):
        pass
        #numpy.save('data/backLegSensorValues.npy',self.sensors"")
        #numpy.save('data/frontLegSensorValues.npy',frontLegSensorValues)
        #numpy.save('data/targetAngles.npy',targetAngles)