import pybullet as p

class WORLD:

    def __init__(self):
        '''Constructor of WORLD class'''

        self.planeId = p.loadURDF("plane.urdf")