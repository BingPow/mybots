import numpy
import random
from pyparsing import pythonStyleComment
import pyrosim.pyrosim as pyrosim
import constants as c

import time
import os


class SOLUTION:

    def __init__(self, nextAvailableID):
        self.myID = nextAvailableID
        # If ID = 0 use old weights
        self.weights = numpy.random.rand(c.numSensorNeurons,c.numMotorNeurons)
        self.weights = self.weights * 2 - 1

    def Set_ID(self, myID):
        self.myID = myID

    def Mutate(self): #Have to change somewhere in here to constant variables
        randrow = random.randint(0,c.numSensorNeurons-1)
        randcol = random.randint(0,c.numMotorNeurons-1)
        self.weights[randrow][randcol] = random.random()*2 - 1

    def Start_Simulation(self,G):
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()
        os.system('python3 simulate.py ' + G + " " + str(self.myID) + " 2&>1 &")

    def Wait_For_Simulation_To_End(self):
        while not os.path.exists("fitness" + str(self.myID) + ".txt"):
            time.sleep(0.01)

        while os.path.getsize("fitness"+str(self.myID)+".txt")<5:
            time.sleep(0.01)

        infile = open("fitness" + str(self.myID) + ".txt", 'r')

        fitness = float(infile.readline())

        try:
            self.fitness = float(fitness)
        except ValueError as error:
            print(error)
            print("attempted to convert: "+fitness)
            print("ID: "+str(self.myID))
            exit()
        infile.close() 

        os.system("rm fitness" + str(self.myID) + ".txt")
        
        G = "DIRECT"

    def Evaluate(self,G):
        
        # Seems like when the GUI runs it doesn't save the fitness
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()
        os.system("python3 simulate.py " + G + " " + str(self.myID) + " 2&>1 &")
          
        infile = open("fitness" + str(self.myID) + ".txt", 'r')
        while not os.path.exists("fitness" + str(self.myID) + ".txt"):
            time.sleep(0.01)
        self.fitness = float(infile.readline())
        print('printing fitness: ' + str(self.fitness))
        infile.close() 

        G = "DIRECT"
        
    def Run_Sim(self):
        pass

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[-5,-5,0.5] , size=[c.length,c.width,c.height])
        pyrosim.End()

    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")

        pyrosim.Send_Cube(name="Torso", pos=[0,0,1] , size=[2,1,0.5]) 

        pyrosim.Send_Joint(name = "Torso_BackLeftShoulder", parent="Torso",child = "BackLeftShoulder", type="revolute",position = [-0.75,0.5,1.0],jointAxis = "1 0 0")
        pyrosim.Send_Cube(name = "BackLeftShoulder", pos=[0,0,0], size = [0.5,0.5,0.5])

        pyrosim.Send_Joint(name = "Torso_BackRightShoulder", parent="Torso",child = "BackRightShoulder", type="revolute",position = [-0.75,-0.5,1.0],jointAxis = "1 0 0")
        pyrosim.Send_Cube(name = "BackRightShoulder", pos=[0,0,0], size = [0.5,0.5,0.5])

        pyrosim.Send_Joint(name = "Torso_FrontLeftShoulder", parent="Torso",child = "FrontLeftShoulder", type="revolute",position = [0.75,0.5,1.0],jointAxis = "1 0 0")
        pyrosim.Send_Cube(name = "FrontLeftShoulder", pos=[0,0,0], size = [0.5,0.5,0.5])

        pyrosim.Send_Joint(name = "Torso_FrontRightShoulder", parent="Torso",child = "FrontRightShoulder", type="revolute",position = [0.75,-0.5,1.0],jointAxis = "1 0 0")
        pyrosim.Send_Cube(name = "FrontRightShoulder", pos=[0,0,0], size = [0.5,0.5,0.5])

        pyrosim.Send_Joint(name = "BackRightShoulder_BackRightUpperArm", parent="BackRightShoulder",child = "BackRightUpperArm", type="revolute",position = [0,0,0],jointAxis = "0 1 0")
        pyrosim.Send_Cube(name = "BackRightUpperArm", pos=[-0.5,-0.125,0], size = [1,0.25,0.25])

        pyrosim.Send_Joint(name = "BackLeftShoulder_BackLeftUpperArm", parent="BackLeftShoulder",child = "BackLeftUpperArm", type="revolute",position = [0,0,0],jointAxis = "0 1 0")
        pyrosim.Send_Cube(name = "BackLeftUpperArm", pos=[-0.5,0.125,0], size = [1,0.25,0.25])

        pyrosim.Send_Joint(name = "FrontRightShoulder_FrontRightUpperArm", parent="FrontRightShoulder",child = "FrontRightUpperArm", type="revolute",position = [0,0,0],jointAxis = "0 1 0")
        pyrosim.Send_Cube(name = "FrontRightUpperArm", pos=[-0.5,-0.125,0], size = [1,0.25,0.25])

        pyrosim.Send_Joint(name = "FrontLeftShoulder_FrontLeftUpperArm", parent="FrontLeftShoulder",child = "FrontLeftUpperArm", type="revolute",position = [0,0,0],jointAxis = "0 1 0")
        pyrosim.Send_Cube(name = "FrontLeftUpperArm", pos=[-0.5,0.125,0], size = [1,0.25,0.25])

        # COULD POTENTIALLY ADD ANOTHER MID JOINT THAT WOULD BE THE "KNEE"

        # I want the knee to bend the other way but you will figure that out
        pyrosim.Send_Joint(name = "BackRightUpperArm_BackRightLowerArm", parent="BackRightUpperArm",child = "BackRightLowerArm", type="revolute",position = [-1,0,0],jointAxis = "0 1 0")
        pyrosim.Send_Cube(name = "BackRightLowerArm", pos=[-0.5,-0.125,0], size = [1,0.25,0.25])

        pyrosim.Send_Joint(name = "BackLeftUpperArm_BackLeftLowerArm", parent="BackLeftUpperArm",child = "BackLeftLowerArm", type="revolute",position = [-1,0,0],jointAxis = "0 1 0")
        pyrosim.Send_Cube(name = "BackLeftLowerArm", pos=[-0.5,0.125,0], size = [1,0.25,0.25])

        pyrosim.Send_Joint(name = "FrontRightUpperArm_FrontRightLowerArm", parent="FrontRightUpperArm",child = "FrontRightLowerArm", type="revolute",position = [-1,0,0],jointAxis = "0 1 0")
        pyrosim.Send_Cube(name = "FrontRightLowerArm", pos=[-0.5,-0.125,0], size = [1,0.25,0.25])

        pyrosim.Send_Joint(name = "FrontLeftUpperArm_FrontLeftLowerArm", parent="FrontLeftUpperArm",child = "FrontLeftLowerArm", type="revolute",position = [-1,0,0],jointAxis = "0 1 0")
        pyrosim.Send_Cube(name = "FrontLeftLowerArm", pos=[-0.5,0.125,0], size = [1,0.25,0.25])

        pyrosim.End()

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeftShoulder")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "BackRightShoulder")
        pyrosim.Send_Sensor_Neuron(name = 3 , linkName = "FrontLeftShoulder")
        pyrosim.Send_Sensor_Neuron(name = 4 , linkName = "FrontRightShoulder")
        pyrosim.Send_Sensor_Neuron(name = 5 , linkName = "BackLeftUpperArm")
        pyrosim.Send_Sensor_Neuron(name = 6 , linkName = "BackRightUpperArm")
        pyrosim.Send_Sensor_Neuron(name = 7 , linkName = "FrontLeftUpperArm")
        pyrosim.Send_Sensor_Neuron(name = 8 , linkName = "FrontRightUpperArm")
        pyrosim.Send_Sensor_Neuron(name = 7 , linkName = "BackLeftLowerArm")
        pyrosim.Send_Sensor_Neuron(name = 8 , linkName = "BackRightLowerArm")
        pyrosim.Send_Sensor_Neuron(name = 9 , linkName = "FrontLeftLowerArm")
        pyrosim.Send_Sensor_Neuron(name = 10 , linkName = "FrontRightLowerArm")
        
        pyrosim.Send_Motor_Neuron( name = 11 , jointName = "BackRightShoulder_BackRightUpperArm")
        pyrosim.Send_Motor_Neuron( name = 12 , jointName = "BackLeftShoulder_BackLeftUpperArm")
        pyrosim.Send_Motor_Neuron( name = 13 , jointName = "FrontRightShoulder_FrontRightUpperArm")
        pyrosim.Send_Motor_Neuron( name = 14 , jointName = "FrontLeftShoulder_FrontLeftUpperArm")
        pyrosim.Send_Motor_Neuron( name = 15 , jointName = "BackRightUpperArm_BackRightLowerArm")
        pyrosim.Send_Motor_Neuron( name = 16 , jointName = "BackLeftUpperArm_BackLeftLowerArm")
        pyrosim.Send_Motor_Neuron( name = 17 , jointName = "FrontRightUpperArm_FrontRightLowerArm")
        pyrosim.Send_Motor_Neuron( name = 18 , jointName = "FrontLeftUpperArm_FrontLeftLowerArm")
                
        '''
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackRightLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "BackLeftLeg")
        pyrosim.Send_Sensor_Neuron(name = 3 , linkName = "FrontRightLeg")
        pyrosim.Send_Sensor_Neuron(name = 4 , linkName = "FrontLeftLeg")
        pyrosim.Send_Sensor_Neuron(name = 5 , linkName = "BackRightLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 6 , linkName = "BackLeftLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 7 , linkName = "FrontRightLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 8 , linkName = "FrontLeftLowerLeg")
        

        pyrosim.Send_Motor_Neuron( name = 9 , jointName = "Torso_BackRightLeg")
        pyrosim.Send_Motor_Neuron( name = 10 , jointName = "Torso_BackLeftLeg")
        pyrosim.Send_Motor_Neuron( name = 11 , jointName = "Torso_FrontRightLeg")
        pyrosim.Send_Motor_Neuron( name = 12 , jointName = "Torso_FrontLeftLeg")
        pyrosim.Send_Motor_Neuron( name = 13 , jointName = "BackRightLeg_BackRightLowerLeg")
        pyrosim.Send_Motor_Neuron( name = 14 , jointName = "BackLeftLeg_BackLeftLowerLeg")
        pyrosim.Send_Motor_Neuron( name = 15 , jointName = "FrontRightLeg_FrontRightLowerLeg")
        pyrosim.Send_Motor_Neuron( name = 16 , jointName = "FrontLeftLeg_FrontLeftLowerLeg")

        '''
        for currentRow in range(c.numSensorNeurons):
            for currentColumn in range(c.numMotorNeurons):
                pyrosim.Send_Synapse(sourceNeuronName = currentRow, targetNeuronName = currentColumn+c.numSensorNeurons , weight = self.weights[currentRow][currentColumn] )
        pyrosim.End()
        

    


        