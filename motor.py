import constants as c
import pyrosim.pyrosim as pyrosim
import numpy as np

class MOTOR:

    def __init__(self, jointname):
        '''Constructor of MOTOR class'''

        self.jointName = jointname


    def Set_Value(self, desiredAngle, robot):
        '''Sets the value for each motor to the given argument'''

        pyrosim.Set_Motor_For_Joint(bodyIndex = robot,
                                    jointName = self.jointName,
                                    controlMode = c.CONTROL_MODE,
                                    targetPosition = desiredAngle,
                                    maxForce = c.MAX_FORCE)
