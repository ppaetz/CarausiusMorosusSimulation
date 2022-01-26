import numpy
import pybullet as p
import pyrosim.pyrosim as pyrosim
import os
import random
import time
import constants as c

class SOLUTION:

    def __init__(self, nextAvailableID):
        '''Constructor of SOLUTION class'''

        self.myID = nextAvailableID
        self.sensorNeuronNames = {}
        self.motorNeuronNames = {}
        self.weights = numpy.random.rand(c.NUM_SENSOR_NEURONS,c.NUM_MOTOR_NEURONS)
        self.weights = self.weights * 2 - 1

    def Start_Simulation(self, directGUI):
        '''Generating a simulation for the current instance'''
        '''directGUI:   use "GUI" for a visual presentation of the simulation (slower)'''
        '''             use in "DIRECT" for a non-visual simulation (faster)'''

        self.Create_World()
        self.Generate_Body()
        self.Generate_Brain()
        
        os.system("python3 simulate.py " + str(directGUI) + " " + str(self.myID) + " &")
        

    def Wait_For_Simulation_To_End(self):
        '''Prevents the next simulation from starting before the previous one has finished'''

        while not os.path.exists("fitness" + str(self.myID) + ".txt"):
            time.sleep(0.01)

        file = open("fitness" + str(self.myID) + ".txt", "r")
        self.fitness = file.read()
        file.close()
        os.system("rm fitness" + str(self.myID) + ".txt")

    def Mutate(self):
        '''Creates a random "mutation" of a certain number within the weight matrix'''
        
        randomRow = random.randint(0,c.NUM_SENSOR_NEURONS-1)
        randomColumn = random.randint(0,c.NUM_MOTOR_NEURONS-1)
        self.weights[randomRow,randomColumn] =  random.random() * 2 - 1

    def Set_ID(self, nextAvailableID):
        '''Gets the current unique ID'''
        
        self.myID = nextAvailableID

    def Create_World(self):
        '''Creates sdf file for the world'''
        
        length = 1
        width = 1
        height = 1
        x = 0
        y = 0
        z = height/2

        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[x, y + 2, z, 0], size=[length,width,height])
        pyrosim.End()

    def Generate_Body(self):
        '''Creates the "physical" body of the robot'''

        pyrosim.Start_URDF("body" + str(self.myID) + ".urdf")

        file = open("urdf/Carausius_Morosus.urdf")
        
        pyrosim.End()


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
