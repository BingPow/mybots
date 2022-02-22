import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import constants as c

class WORLD:
    def __init__(self):
        
        p.loadSDF("world.sdf")
        self.planeId = p.loadURDF("plane.urdf")