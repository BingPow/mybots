import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy
import constants as c

class SENSOR:

    def __init__(self,linkName):
        self.linkName = linkName
        self.values = numpy.zeros(c.steps)

    def Get_Value(self,t):
        self.values[t] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)
        if t == c.steps-1:
            print(self.values)
        #self.sensors[t]
        #right now trying to figure out how to get for loop to work right
        