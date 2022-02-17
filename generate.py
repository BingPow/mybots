from pyparsing import pythonStyleComment
import pyrosim.pyrosim as pyrosim
import constants as c

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

Create_World()

Create_Robot()