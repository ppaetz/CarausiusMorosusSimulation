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

        self.robot = p.loadURDF("urdf/body" + str(solutionID) + ".urdf", 0, 0, 1.5)
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

        p.changeDynamics(self.robot, 0, c.LOWER_LIMIT_PROTHORAX_MESOTHORAX, c.UPPER_LIMIT_PROTHORAX_MESOTHORAX)
        p.changeDynamics(self.robot, 1, c.LOWER_LIMIT_MESOTHORAX_METATHORAX, c.UPPER_LIMIT_MESOTHORAX_METATHORAX)
        p.changeDynamics(self.robot, 2, c.LOWER_LIMIT_PROTHORAX_MESOTHORAX, c.UPPER_LIMIT_PROTHORAX_MESOTHORAX)
        p.changeDynamics(self.robot, 3, c.LOWER_LIMIT_MESOTHORAX_METATHORAX, c.UPPER_LIMIT_MESOTHORAX_METATHORAX)
        p.changeDynamics(self.robot, 4, c.LOWER_LIMIT_METATHORAX_ABDOMEN_3, c.UPPER_LIMIT_METATHORAX_ABDOMEN_3)
        p.changeDynamics(self.robot, 5, c.LOWER_LIMIT_ABDOMEN_3_ABDOMEN_2, c.UPPER_LIMIT_ABDOMEN_3_ABDOMEN_2)
        p.changeDynamics(self.robot, 5, c.LOWER_LIMIT_ABDOMEN_2_ABDOMEN_1, c.UPPER_LIMIT_ABDOMEN_2_ABDOMEN_1)
        p.changeDynamics(self.robot, 5, c.LOWER_LIMIT_ABDOMEN_1_ABDOMEN_0, c.UPPER_LIMIT_ABDOMEN_1_ABDOMEN_0)


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
            p.changeDynamics(self.robot, jointIndex, 
                             lateralFriction=c.LATERAL_FRICTION_COEFFICIENT)

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
