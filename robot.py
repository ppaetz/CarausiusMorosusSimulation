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

        self.robot = p.loadURDF("urdf/Carausius_Morosus.urdf", 0, 0, 1.5)
        pyrosim.Prepare_To_Simulate("urdf/Carausius_Morosus.urdf")
        self.nn = NEURAL_NETWORK("brain.nndf")

        self.maxJointTorques = 0
        self.sensorNeuronNames = {}
        self.motorNeuronNames = {}
        self.weights = numpy.random.rand(c.NUM_SENSOR_NEURONS,c.NUM_MOTOR_NEURONS)
        self.weights = self.weights * 2 - 1

        self.Prepare_To_Sense()
        self.Prepare_To_Act()
        self.Generate_Brain()


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

    def Generate_Brain(self):
        '''Creates the neural network of the robot'''

        pyrosim.Start_NeuralNetwork("brain.nndf")

        i = 0

        for linkName in pyrosim.linkNamesToIndices:
            pyrosim.Send_Sensor_Neuron(name = i , linkName = linkName)
            print("Sensor_Neuron: ", i, ", Link: ", linkName)
            i += 1 

        for jointName in pyrosim.jointNamesToIndices:
            pyrosim.Send_Motor_Neuron(name = i , jointName = jointName)
            print("Motor_Neuron: ", i, ", Joint: ", jointName)
            i += 1

        for currentRow in range(0, c.NUM_SENSOR_NEURONS):
            for currentColumn in range(0, c.NUM_MOTOR_NEURONS):
                pyrosim.Send_Synapse( sourceNeuronName = currentRow , targetNeuronName = currentColumn+c.NUM_SENSOR_NEURONS , weight = self.weights[currentRow][currentColumn] )


        pyrosim.End()