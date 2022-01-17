from world import WORLD
from robot import ROBOT

import pybullet as p
import pybullet_data
import constants as c
import time

class SIMULATION:

    def __init__(self):
        '''Constructor of the SIMULATION class'''

        
        physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(c.GRAVITY_X, c.GRAVITY_Y, c.GRAVITY_Z)

        self.world = WORLD()
        self.robot = ROBOT()

    def __del__(self):
        '''Destructor of the SIMULATION class'''

        p.disconnect()

    def RUN(self):
        '''Main loop of the simulation'''

        for t in range(0, c.SIMULATION_STEPS):    
            
            p.stepSimulation()
            time.sleep(1/240)
            t += 1