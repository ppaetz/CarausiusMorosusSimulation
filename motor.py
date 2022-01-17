from cmath import pi
import constants as c
import pyrosim.pyrosim as pyrosim
import numpy as np

class MOTOR:

    def __init__(self, jointname):
        '''Constructor of MOTOR class'''

        self.jointName = jointname

    def Set_Value(self,robot):
        '''Sets the value for each motor to the given argument'''

        print("Robot: ", robot, ", Joint: ", self.jointName)
        pyrosim.Set_Motor_For_Joint(bodyIndex = robot,
                                    jointName = self.jointName,
                                    controlMode = c.CONTROL_MODE,
                                    targetPosition = 0,
                                    maxForce = c.MAX_FORCE)
