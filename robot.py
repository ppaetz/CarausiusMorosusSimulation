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

        self.robot = p.loadURDF("urdf/Carausius_Morosus.urdf", 0, 0, 2, -180, 0, 180, 0)
        pyrosim.Prepare_To_Simulate("urdf/Carausius_Morosus.urdf")

        self.Prepare_To_Sense()
        self.Prepare_To_Act()


    def Prepare_To_Sense(self):
        '''Generates sensors for each link'''

        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

        for jointIndex in range(0,p.getNumJoints(self.robot)):
            p.changeDynamics(self.robot, jointIndex, lateralFriction=c.LATERAL_FRICTION_COEFFICIENT)
            p.enableJointForceTorqueSensor(self.robot, jointIndex)


    def Prepare_To_Act(self):
        '''Generates motors for each joint'''

        self.changeJointLimits()

        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)


    def Sense(self, t):
        '''Gets the measured values for each sensor '''

        self.jointState = {}
        self.jointTorques = {}

        for sensor in self.sensors:
            self.sensors[sensor].Get_Value(t)

        for jointIndex in range(0,p.getNumJoints(self.robot)):
            self.jointState[jointIndex] = p.getJointState(self.robot, jointIndex)

            for torques in self.jointState.values(): 
                self.jointTorques[t] = torques[3]
                if jointIndex > 6:
                    print("Joint: ", jointIndex, ", cuurent Torque; ", self.jointTorques[t])


    def Act(self):
        '''Sets the motorvalues according to the sensorvalues'''

        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName].Set_Value(0, self.robot)


    def changeJointLimits(self):
        '''changing the limits of all joints'''

        p.changeDynamics(self.robot, 0, jointLowerLimit=c.LOWER_LIMIT_CARPUT_PROTHORAX, jointUpperLimit=c.UPPER_LIMIT_CARPUT_PROTHORAX)
        p.changeDynamics(self.robot, 1, jointLowerLimit=c.LOWER_LIMIT_PROTHORAX_MESOTHORAX, jointUpperLimit=c.UPPER_LIMIT_PROTHORAX_MESOTHORAX)
        p.changeDynamics(self.robot, 2, jointLowerLimit=c.LOWER_LIMIT_MESOTHORAX_METATHORAX, jointUpperLimit=c.UPPER_LIMIT_MESOTHORAX_METATHORAX)
        p.changeDynamics(self.robot, 3, jointLowerLimit=c.LOWER_LIMIT_METATHORAX_ABDOMEN_3, jointUpperLimit=c.UPPER_LIMIT_METATHORAX_ABDOMEN_3)
        p.changeDynamics(self.robot, 4, jointLowerLimit=c.LOWER_LIMIT_ABDOMEN_3_ABDOMEN_2, jointUpperLimit=c.UPPER_LIMIT_ABDOMEN_3_ABDOMEN_2)
        p.changeDynamics(self.robot, 5, jointLowerLimit=c.LOWER_LIMIT_ABDOMEN_2_ABDOMEN_1, jointUpperLimit=c.UPPER_LIMIT_ABDOMEN_2_ABDOMEN_1)
        p.changeDynamics(self.robot, 6, jointLowerLimit=c.LOWER_LIMIT_ABDOMEN_1_ABDOMEN_0, jointUpperLimit=c.UPPER_LIMIT_ABDOMEN_1_ABDOMEN_0)

        p.changeDynamics(self.robot, 7, jointLowerLimit=c.LOWER_LIMIT_HL_CT_L, jointUpperLimit=c.UPPER_LIMIT_HL_CT_L)
        p.changeDynamics(self.robot, 8, jointLowerLimit=c.LOWER_LIMIT_HL_FEMUR_L, jointUpperLimit=c.UPPER_LIMIT_HL_FEMUR_L)
        p.changeDynamics(self.robot, 9, jointLowerLimit=c.LOWER_LIMIT_HL_TIBIA_L, jointUpperLimit=c.UPPER_LIMIT_HL_TIBIA_L)
        p.changeDynamics(self.robot, 10, jointLowerLimit=c.LOWER_LIMIT_HL_TARSUS_L, jointUpperLimit=c.UPPER_LIMIT_HL_TARSUS_L)
        p.changeDynamics(self.robot, 11, jointLowerLimit=c.LOWER_LIMIT_HL_CT_R, jointUpperLimit=c.UPPER_LIMIT_HL_CT_R)
        p.changeDynamics(self.robot, 12, jointLowerLimit=c.LOWER_LIMIT_HL_FEMUR_R, jointUpperLimit=c.UPPER_LIMIT_HL_FEMUR_R)
        p.changeDynamics(self.robot, 13, jointLowerLimit=c.LOWER_LIMIT_HL_TIBIA_R, jointUpperLimit=c.UPPER_LIMIT_HL_TIBIA_R)
        p.changeDynamics(self.robot, 14, jointLowerLimit=c.LOWER_LIMIT_HL_TARSUS_R, jointUpperLimit=c.UPPER_LIMIT_HL_TARSUS_R)

        p.changeDynamics(self.robot, 15, jointLowerLimit=c.LOWER_LIMIT_ML_CT_L, jointUpperLimit=c.UPPER_LIMIT_ML_CT_L)
        p.changeDynamics(self.robot, 16, jointLowerLimit=c.LOWER_LIMIT_ML_FEMUR_L, jointUpperLimit=c.UPPER_LIMIT_ML_FEMUR_L)
        p.changeDynamics(self.robot, 17, jointLowerLimit=c.LOWER_LIMIT_ML_TIBIA_L, jointUpperLimit=c.UPPER_LIMIT_ML_TIBIA_L)
        p.changeDynamics(self.robot, 18, jointLowerLimit=c.LOWER_LIMIT_ML_TARSUS_L, jointUpperLimit=c.UPPER_LIMIT_ML_TARSUS_L)
        p.changeDynamics(self.robot, 19, jointLowerLimit=c.LOWER_LIMIT_ML_CT_R, jointUpperLimit=c.UPPER_LIMIT_ML_CT_R)
        p.changeDynamics(self.robot, 20, jointLowerLimit=c.LOWER_LIMIT_ML_FEMUR_R, jointUpperLimit=c.UPPER_LIMIT_ML_FEMUR_R)
        p.changeDynamics(self.robot, 21, jointLowerLimit=c.LOWER_LIMIT_ML_TIBIA_R, jointUpperLimit=c.UPPER_LIMIT_ML_TIBIA_R)
        p.changeDynamics(self.robot, 22, jointLowerLimit=c.LOWER_LIMIT_ML_TARSUS_R, jointUpperLimit=c.UPPER_LIMIT_ML_TARSUS_R)
        
        p.changeDynamics(self.robot, 23, jointLowerLimit=c.LOWER_LIMIT_FL_CT_L, jointUpperLimit=c.UPPER_LIMIT_FL_CT_L)
        p.changeDynamics(self.robot, 24, jointLowerLimit=c.LOWER_LIMIT_FL_FEMUR_L, jointUpperLimit=c.UPPER_LIMIT_FL_FEMUR_L)
        p.changeDynamics(self.robot, 25, jointLowerLimit=c.LOWER_LIMIT_FL_TIBIA_L, jointUpperLimit=c.UPPER_LIMIT_FL_TIBIA_L)
        p.changeDynamics(self.robot, 26, jointLowerLimit=c.LOWER_LIMIT_FL_TARSUS_L, jointUpperLimit=c.UPPER_LIMIT_FL_TARSUS_L)
        p.changeDynamics(self.robot, 27, jointLowerLimit=c.LOWER_LIMIT_FL_CT_R, jointUpperLimit=c.UPPER_LIMIT_FL_CT_R)
        p.changeDynamics(self.robot, 28, jointLowerLimit=c.LOWER_LIMIT_FL_FEMUR_R, jointUpperLimit=c.UPPER_LIMIT_FL_FEMUR_R)
        p.changeDynamics(self.robot, 29, jointLowerLimit=c.LOWER_LIMIT_FL_TIBIA_R, jointUpperLimit=c.UPPER_LIMIT_FL_TIBIA_R)
        p.changeDynamics(self.robot, 30, jointLowerLimit=c.LOWER_LIMIT_FL_TARSUS_R, jointUpperLimit=c.UPPER_LIMIT_FL_TARSUS_R)