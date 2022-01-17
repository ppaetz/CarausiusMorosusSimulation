from pyrosim.neuralNetwork import NEURAL_NETWORK
from sensor import SENSOR
from motor import MOTOR

import pybullet as p
import pyrosim.pyrosim as pyrosim
import constants as c
import os
import numpy

class ROBOT:

    def __init__(self):
        '''Constructor of ROBOT class'''

        self.robot = p.loadURDF("urdf/Carausius_Morosus.urdf", 0, 0, 1)
        pyrosim.Prepare_To_Simulate("urdf/Carausius_Morosus.urdf")

        self.Prepare_To_Sense()

        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)
            self.motors[jointName].Set_Value(self.robot)

    def Prepare_To_Sense(self):
        '''Generates sensors for each link'''

        for jointIndex in range(0,p.getNumJoints(self.robot)):
            p.enableJointForceTorqueSensor(self.robot, jointIndex)

        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)