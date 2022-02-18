import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy
from math import pi
import constants as c

class MOTOR:
    def __init__(self,jointName):
        self.jointName = jointName
        self.Prepare_To_Act()
    
    def Prepare_To_Act(self):
        self.amplitude = c.amplitude
        self.frequency = c.frequency
        self.offset = c.phaseOffSet
        self.motorValues = self.amplitude*numpy.sin(self.frequency*numpy.linspace(0,2*pi, c.steps) + self.offset)
        
    def Set_Value(self,robot,i):
        pyrosim.Set_Motor_For_Joint(
                bodyIndex = robot,
                jointName = self.jointName,
                controlMode = p.POSITION_CONTROL,
                targetPosition = self.motorValues[i],
                maxForce = c.maxForce)
        
        
