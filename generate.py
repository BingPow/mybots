from pyparsing import pythonStyleComment
import pyrosim.pyrosim as pyrosim
import constants as c

import random

def Create_World():
    pyrosim.Start_SDF("world.sdf")
    pyrosim.Send_Cube(name="Box", pos=[-5,-5,0.5] , size=[c.length,c.width,c.height])
    pyrosim.End()

def Create_Robot():
    pyrosim.Start_URDF("body.urdf")
    pyrosim.Send_Cube(name="Torso", pos=[1.5,0,1.5] , size=[c.length,c.width,c.height]) #[1.5,0,1.5]
    pyrosim.Send_Joint(name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [1.0,0,1.0])
    pyrosim.Send_Cube(name="BackLeg", pos=[-0.5,0,-0.5], size = [c.length,c.width,c.height])
    pyrosim.Send_Joint(name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [2.0,0,1.0])
    pyrosim.Send_Cube(name="FrontLeg", pos=[0.5,0,-0.5], size = [c.length,c.width,c.height])
    pyrosim.End()

def Generate_Body():
    pyrosim.Start_URDF("body.urdf")
    pyrosim.Send_Cube(name="Torso", pos=[1.5,0,1.5] , size=[c.length,c.width,c.height]) #[1.5,0,1.5]
    pyrosim.Send_Cube(name="BackLeg", pos=[-0.5,0,-0.5], size = [c.length,c.width,c.height])
    pyrosim.Send_Cube(name="FrontLeg", pos=[0.5,0,-0.5], size = [c.length,c.width,c.height])
    pyrosim.Send_Joint(name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [1.0,0,1.0])
    pyrosim.Send_Joint(name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [2.0,0,1.0])
    pyrosim.End()

def Generate_Brain(weights):
    random.random()

    pyrosim.Start_NeuralNetwork("brain.nndf")
    sensorName = [0,1,2]
    motorName = [3,4]
    pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
    pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
    pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")
    pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
    pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")
    for currentRow in range(3):
        for currentColumn in range(2):
            pyrosim.Send_Synapse(sourceNeuronName = sensorName[currentRow], targetNeuronName = motorName[currentColumn+3] , weight = weights[currentRow][currentColumn] )
    
    '''
    pyrosim.Send_Synapse(sourceNeuronName = 0, targetNeuronName = 3 , weight = 1.0 )
    pyrosim.Send_Synapse(sourceNeuronName = 1, targetNeuronName = 3 , weight = 1.0 )
    pyrosim.Send_Synapse(sourceNeuronName = 2, targetNeuronName = 3 , weight = 1.0 )
    pyrosim.Send_Synapse(sourceNeuronName = 0, targetNeuronName = 4 , weight = -1.0 )
    pyrosim.Send_Synapse(sourceNeuronName = 1, targetNeuronName = 4 , weight = 1.0 )
    pyrosim.Send_Synapse(sourceNeuronName = 2, targetNeuronName = 4 , weight = 1.0 )'''

    pyrosim.End()

Create_World()

Generate_Body()

Generate_Brain()