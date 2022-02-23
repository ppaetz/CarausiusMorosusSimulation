import pybullet as p
import constants as c
import pyrosim.pyrosim as pyrosim

class WORLD:

    def __init__(self):
        '''Constructor of WORLD class'''

        self.planeId = p.loadURDF("plane.urdf")
        p.changeDynamics(self.planeId, -1, lateralFriction=c.LATERAL_FRICTION_COEFFICIENT) 
