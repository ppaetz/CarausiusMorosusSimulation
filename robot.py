from matplotlib.pyplot import text
from pyrosim.neuralNetwork import NEURAL_NETWORK
from sensor import SENSOR
from motor import MOTOR

import pybullet as p
import pyrosim.pyrosim as pyrosim
import constants as c
import os
import numpy

class ROBOT:

    def __init__(self, solutionID):
        '''Constructor of ROBOT class'''

        self.robot = p.loadURDF("urdf/body" + str(solutionID) + ".urdf", 0, 0, 1.5, 0, 0, 180, 0)

        pyrosim.Prepare_To_Simulate("urdf/body" + str(solutionID) + ".urdf")
        self.nn = NEURAL_NETWORK("nndf/brain" + str(solutionID) + ".nndf")

        self.maxJointTorques = 0
        self.sensorNeuronNames = {}
        self.motorNeuronNames = {}
        self.weights = numpy.random.rand(c.NUM_SENSOR_NEURONS,c.NUM_MOTOR_NEURONS)
        self.weights = self.weights * 2 - 1

        self.Prepare_To_Sense()
        self.Prepare_To_Act()

        self.solutionID = solutionID

        os.system("rm nndf/brain" + str(self.solutionID) + ".nndf")
        os.system("rm urdf/body" + str(self.solutionID) + ".urdf")


    def Prepare_To_Sense(self):
        '''Generates sensors for each link'''

        for jointIndex in range(0,p.getNumJoints(self.robot)):
            p.enableJointForceTorqueSensor(self.robot, jointIndex)

        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)


    def Prepare_To_Act(self):
        '''Generates motors for each joint'''

        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)

        # self.changeJointLimits()


    def Sense(self, t):
        '''Gets the measured values for each sensor '''

        self.jointState = {}
        self.jointTorques = {}
        self.friction = {}
        self.frictionCoefficient = {}

        for sensor in self.sensors:
            self.sensors[sensor].Get_Value(t)

        for jointIndex in range(0,p.getNumJoints(self.robot)):
            self.jointState[jointIndex] = p.getJointState(self.robot, jointIndex)

            for torques in self.jointState.values(): 
                self.jointTorques[t] = torques[3]
                self.jointTorques[t] = numpy.sqrt(torques[3] ** 2)
                self.maxJointTorques = self.maxJointTorques + self.jointTorques[t]

        
    def Think(self):
        '''updates the value for all neurons'''
        
        self.nn.Update()


    def Act(self):
        '''Sets the motorvalues according to the sensorvalues'''

        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName) * c.MOTOR_JOINT_RANGE
                desiredAngle = desiredAngle *2
                self.motors[jointName].Set_Value(desiredAngle, self.robot)


    def Get_Fitness(self):
        '''Extracts the 'fitness' and writes it into a txt file'''

        'Position of Robot'
        self.basePositionAndOrientation = p.getBasePositionAndOrientation(self.robot)
        self.basePosition = self.basePositionAndOrientation[0]
        xPosition = self.basePosition[0]
        yPosition = self.basePosition[1]
        zPosition = self.basePosition[2]

        self.fitness = yPosition
        file = open("fitness/tmp" + str(self.solutionID) + ".txt", "w")
        file.write(str(self.fitness))
        file.close()
        os.system("mv fitness/tmp" + str(self.solutionID) + ".txt fitness/fitness" + str(self.solutionID) + ".txt")

    def changeJointLimits(self):
        '''changing the limits of all joints'''

        # p.changeDynamics(self.robot, 0, jointLowerLimit=c.LOWER_LIMIT_CARPUT_PROTHORAX, jointUpperLimit=c.UPPER_LIMIT_CARPUT_PROTHORAX)
        # p.changeDynamics(self.robot, 1, jointLowerLimit=c.LOWER_LIMIT_PROTHORAX_MESOTHORAX, jointUpperLimit=c.UPPER_LIMIT_PROTHORAX_MESOTHORAX)
        # p.changeDynamics(self.robot, 2, jointLowerLimit=c.LOWER_LIMIT_MESOTHORAX_METATHORAX, jointUpperLimit=c.UPPER_LIMIT_MESOTHORAX_METATHORAX)
        # p.changeDynamics(self.robot, 3, jointLowerLimit=c.LOWER_LIMIT_METATHORAX_ABDOMEN_3, jointUpperLimit=c.UPPER_LIMIT_METATHORAX_ABDOMEN_3)
        # p.changeDynamics(self.robot, 4, jointLowerLimit=c.LOWER_LIMIT_ABDOMEN_3_ABDOMEN_2, jointUpperLimit=c.UPPER_LIMIT_ABDOMEN_3_ABDOMEN_2)
        # p.changeDynamics(self.robot, 5, jointLowerLimit=c.LOWER_LIMIT_ABDOMEN_2_ABDOMEN_1, jointUpperLimit=c.UPPER_LIMIT_ABDOMEN_2_ABDOMEN_1)
        # p.changeDynamics(self.robot, 6, jointLowerLimit=c.LOWER_LIMIT_ABDOMEN_1_ABDOMEN_0, jointUpperLimit=c.UPPER_LIMIT_ABDOMEN_1_ABDOMEN_0)
        # p.changeDynamics(self.robot, 7, jointLowerLimit=c.LOWER_LIMIT_HL_CT_L, jointUpperLimit=c.UPPER_LIMIT_HL_CT_L)
        # p.changeDynamics(self.robot, 8, jointLowerLimit=c.LOWER_LIMIT_HL_FEMUR_L, jointUpperLimit=c.UPPER_LIMIT_HL_FEMUR_L)
        # p.changeDynamics(self.robot, 9, jointLowerLimit=c.LOWER_LIMIT_HL_TIBIA_L, jointUpperLimit=c.UPPER_LIMIT_HL_TIBIA_L)
        # p.changeDynamics(self.robot, 10, jointLowerLimit=c.LOWER_LIMIT_HL_PRETARSUS_L, jointUpperLimit=c.UPPER_LIMIT_HL_PRETARSUS_L)
        # p.changeDynamics(self.robot, 11, jointLowerLimit=c.LOWER_LIMIT_HL_TARSUS_L, jointUpperLimit=c.UPPER_LIMIT_HL_TARSUS_L)
        # p.changeDynamics(self.robot, 12, jointLowerLimit=c.LOWER_LIMIT_HL_CT_R, jointUpperLimit=c.UPPER_LIMIT_HL_CT_R)
        # p.changeDynamics(self.robot, 13, jointLowerLimit=c.LOWER_LIMIT_HL_FEMUR_R, jointUpperLimit=c.UPPER_LIMIT_HL_FEMUR_R)
        # p.changeDynamics(self.robot, 14, jointLowerLimit=c.LOWER_LIMIT_HL_TIBIA_R, jointUpperLimit=c.UPPER_LIMIT_HL_TIBIA_R)
        # p.changeDynamics(self.robot, 15, jointLowerLimit=c.LOWER_LIMIT_HL_PRETARSUS_R, jointUpperLimit=c.UPPER_LIMIT_HL_PRETARSUS_R)
        # p.changeDynamics(self.robot, 16, jointLowerLimit=c.LOWER_LIMIT_HL_TARSUS_R, jointUpperLimit=c.UPPER_LIMIT_HL_TARSUS_R)
        # p.changeDynamics(self.robot, 17, jointLowerLimit=c.LOWER_LIMIT_ML_CT_L, jointUpperLimit=c.UPPER_LIMIT_ML_CT_L)
        # p.changeDynamics(self.robot, 18, jointLowerLimit=c.LOWER_LIMIT_ML_FEMUR_L, jointUpperLimit=c.UPPER_LIMIT_ML_FEMUR_L)
        # p.changeDynamics(self.robot, 19, jointLowerLimit=c.LOWER_LIMIT_ML_TIBIA_L, jointUpperLimit=c.UPPER_LIMIT_ML_TIBIA_L)
        # p.changeDynamics(self.robot, 20, jointLowerLimit=c.LOWER_LIMIT_ML_PRETARSUS_L, jointUpperLimit=c.UPPER_LIMIT_ML_PRETARSUS_L)
        # p.changeDynamics(self.robot, 21, jointLowerLimit=c.LOWER_LIMIT_ML_TARSUS_L, jointUpperLimit=c.UPPER_LIMIT_ML_TARSUS_L)
        # p.changeDynamics(self.robot, 22, jointLowerLimit=c.LOWER_LIMIT_ML_CT_R, jointUpperLimit=c.UPPER_LIMIT_ML_CT_R)
        # p.changeDynamics(self.robot, 23, jointLowerLimit=c.LOWER_LIMIT_ML_FEMUR_R, jointUpperLimit=c.UPPER_LIMIT_ML_FEMUR_R)
        # p.changeDynamics(self.robot, 24, jointLowerLimit=c.LOWER_LIMIT_ML_TIBIA_R, jointUpperLimit=c.UPPER_LIMIT_ML_TIBIA_R)
        # p.changeDynamics(self.robot, 25, jointLowerLimit=c.LOWER_LIMIT_ML_PRETARSUS_R, jointUpperLimit=c.UPPER_LIMIT_ML_PRETARSUS_R)
        # p.changeDynamics(self.robot, 26, jointLowerLimit=c.LOWER_LIMIT_ML_TARSUS_R, jointUpperLimit=c.UPPER_LIMIT_ML_TARSUS_R)
        # p.changeDynamics(self.robot, 27, jointLowerLimit=c.LOWER_LIMIT_HL_CT_L, jointUpperLimit=c.UPPER_LIMIT_HL_CT_L)
        # p.changeDynamics(self.robot, 28, jointLowerLimit=c.LOWER_LIMIT_HL_FEMUR_L, jointUpperLimit=c.UPPER_LIMIT_HL_FEMUR_L)
        # p.changeDynamics(self.robot, 29, jointLowerLimit=c.LOWER_LIMIT_HL_TIBIA_L, jointUpperLimit=c.UPPER_LIMIT_HL_TIBIA_L)
        # p.changeDynamics(self.robot, 30, jointLowerLimit=c.LOWER_LIMIT_HL_PRETARSUS_L, jointUpperLimit=c.UPPER_LIMIT_HL_PRETARSUS_L)
        # p.changeDynamics(self.robot, 31, jointLowerLimit=c.LOWER_LIMIT_HL_TARSUS_L, jointUpperLimit=c.UPPER_LIMIT_HL_TARSUS_L)
        # p.changeDynamics(self.robot, 32, jointLowerLimit=c.LOWER_LIMIT_HL_CT_R, jointUpperLimit=c.UPPER_LIMIT_HL_CT_R)
        # p.changeDynamics(self.robot, 33, jointLowerLimit=c.LOWER_LIMIT_HL_FEMUR_R, jointUpperLimit=c.UPPER_LIMIT_HL_FEMUR_R)
        # p.changeDynamics(self.robot, 34, jointLowerLimit=c.LOWER_LIMIT_HL_TIBIA_R, jointUpperLimit=c.UPPER_LIMIT_HL_TIBIA_R)
        # p.changeDynamics(self.robot, 35, jointLowerLimit=c.LOWER_LIMIT_HL_PRETARSUS_R, jointUpperLimit=c.UPPER_LIMIT_HL_PRETARSUS_R)
        # p.changeDynamics(self.robot, 36, jointLowerLimit=c.LOWER_LIMIT_HL_TARSUS_R, jointUpperLimit=c.UPPER_LIMIT_HL_TARSUS_R)
        # p.changeDynamics(self.robot, 37, jointLowerLimit=c.LOWER_LIMIT_FL_CT_L, jointUpperLimit=c.UPPER_LIMIT_FL_CT_L)
