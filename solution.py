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

        self.weights = numpy.random.rand(3,2)
        self.weights = self.weights * 2 - 1

    def Set_ID(self, myID):
        self.myID = myID

    def Mutate(self):
        randrow = random.randint(0,2)
        randcol = random.randint(0,1)
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
        os.system('python3 simulate.py ' + G + " " + str(self.myID) + " 2&>1 &")
          
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
        pyrosim.Send_Cube(name="Torso", pos=[1.5,0,1.5] , size=[c.length,c.width,c.height]) #[1.5,0,1.5]
        pyrosim.Send_Joint(name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [1.0,0,1.0])
        pyrosim.Send_Cube(name="BackLeg", pos=[-0.5,0,-0.5], size = [c.length,c.width,c.height])
        pyrosim.Send_Joint(name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [2.0,0,1.0])
        pyrosim.Send_Cube(name="FrontLeg", pos=[0.5,0,-0.5], size = [c.length,c.width,c.height])
        pyrosim.End()

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")
        pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")
        for currentRow in range(3):
            for currentColumn in range(2):
                pyrosim.Send_Synapse(sourceNeuronName = currentRow, targetNeuronName = currentColumn+3 , weight = self.weights[currentRow][currentColumn] )
        pyrosim.End()

    


        