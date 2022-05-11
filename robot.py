import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
from pyrosim.neuralNetwork import NEURAL_NETWORK
import constants as c
import numpy

from sensor import SENSOR
from motor import MOTOR

import os

class ROBOT:
    def __init__(self, solutionID):
        self.myID = solutionID

        self.sensors = {}
        self.motors = {}
        self.robot = p.loadURDF("body.urdf")
        
        self.nn = NEURAL_NETWORK("brain" + str(self.myID) + ".nndf")
        self.outF1 = open("dudeA.txt",'a')
        self.fitnessFileA = open("fitnessFileA.txt","a")
        self.outfile = open("tmp" + str(self.myID) + ".txt", 'w')
        os.system("rm brain" + str(self.myID) + ".nndf")

        self.total = 0
        self.num = 0
        self.num2 = 0
        self.yCoordinateOfLinkZero = 0

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
        for neuronName in self.nn.Get_Neuron_Names():
            if (self.nn.Is_Motor_Neuron(neuronName)):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName) * c.motorJointRange
                self.motors[jointName].Set_Value(self.robot,desiredAngle)
        '''
        for i in range(len(self.motors)):
            self.motors[self.jointNamesList[i]].Set_Value(self.robot,t)
        '''

    def Think(self):
        self.nn.Update()
        self.nn.Print()

    def Get_Fitness(self):
        stateOfLinkZero = p.getLinkState(self.robot,0)
        positionOfLinkZero = stateOfLinkZero[0]
        xCoordinateOfLinkZero = positionOfLinkZero[0]
        yCoordinateOfLinkZero = positionOfLinkZero[1]
        
        
        print("Printing(ending): " + str(yCoordinateOfLinkZero))
        print("Printing self.total: " + str(self.total))

        fitness = xCoordinateOfLinkZero

        self.outF1.write(str(fitness) + '\n')
        self.outF1.close()

        self.fitnessFileA.write(str(fitness) + '\n')
        self.fitnessFileA.close()

        self.outfile.write(str(fitness))
        os.system("mv tmp" + str(self.myID) + ".txt fitness" + str(self.myID) + ".txt")
        self.outfile.close()

    

    # Using wordlinkframeposition, better for getting z fitness
    def Get_Z_Fitness(self):
        stateOfLinkZero = p.getLinkState(self.robot,1)
        positionOfLinkZero = stateOfLinkZero[4]
        zCoordinateOfLinkZero = positionOfLinkZero[2]
        self.total += zCoordinateOfLinkZero
        self.num += 1

    def Get_Y_Fitness(self):
        stateOfLinkZero = p.getLinkState(self.robot,0)
        positionOfLinkZero = stateOfLinkZero[4]
        self.yCoordinateOfLinkZero = positionOfLinkZero[1]

    
        
        
    def Save_Values(self):
        pass
        #numpy.save('data/backLegSensorValues.npy',self.sensors"")
        #numpy.save('data/frontLegSensorValues.npy',frontLegSensorValues)
        #numpy.save('data/targetAngles.npy',targetAngles)