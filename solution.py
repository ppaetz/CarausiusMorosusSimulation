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

        while not os.path.exists("fitness/fitness" + str(self.myID) + ".txt"):
            time.sleep(0.01)

        file = open("fitness/fitness" + str(self.myID) + ".txt", "r")
        self.fitness = file.read()
        file.close()
        os.system("rm fitness/fitness" + str(self.myID) + ".txt")


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

        sourceFile = open("urdf/Carausius_Morosus.urdf", "r")
        content = sourceFile.read()
        sourceFile.close()

        os.system("touch urdf/body" + str(self.myID) + ".urdf")

        newFile = open("urdf/body" + str(self.myID) + ".urdf", "w")
        newFile.write(content)
        newFile.close()


    def Generate_Brain(self):
        '''Creates the neural network of the robot'''

        pyrosim.Start_NeuralNetwork("nndf/brain" + str(self.myID) + ".nndf")

        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "01_Carput")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "02_Prothorax")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "03_Mesothorax")
        pyrosim.Send_Sensor_Neuron(name = 3 , linkName = "04_Metathorax")
        pyrosim.Send_Sensor_Neuron(name = 4 , linkName = "05_Abdomen_3")
        pyrosim.Send_Sensor_Neuron(name = 5 , linkName = "06_Abdomen_2")
        pyrosim.Send_Sensor_Neuron(name = 6 , linkName = "07_Abdomen_1")
        pyrosim.Send_Sensor_Neuron(name = 7 , linkName = "08_Abdomen_0")
        pyrosim.Send_Sensor_Neuron(name = 8 , linkName = "09_HL_CT_L")
        pyrosim.Send_Sensor_Neuron(name = 9 , linkName = "10_HL_Femur_L")
        pyrosim.Send_Sensor_Neuron(name = 10 , linkName = "11_HL_Tibia_L")
        pyrosim.Send_Sensor_Neuron(name = 11 , linkName = "12_HL_Tarsus_L")
        pyrosim.Send_Sensor_Neuron(name = 12 , linkName = "13_HL_Pretarsus_L")
        pyrosim.Send_Sensor_Neuron(name = 13 , linkName = "14_HL_CT_R")
        pyrosim.Send_Sensor_Neuron(name = 14 , linkName = "15_HL_Femur_R")
        pyrosim.Send_Sensor_Neuron(name = 15 , linkName = "16_HL_Tibia_R")
        pyrosim.Send_Sensor_Neuron(name = 16 , linkName = "17_HL_Tarsus_R")
        pyrosim.Send_Sensor_Neuron(name = 17 , linkName = "18_HL_Pretarsus_R")
        pyrosim.Send_Sensor_Neuron(name = 18 , linkName = "19_ML_CT_L")
        pyrosim.Send_Sensor_Neuron(name = 19 , linkName = "20_ML_Femur_L")
        pyrosim.Send_Sensor_Neuron(name = 20 , linkName = "21_ML_Tibia_L")
        pyrosim.Send_Sensor_Neuron(name = 21 , linkName = "22_ML_Tarsus_L")
        pyrosim.Send_Sensor_Neuron(name = 22 , linkName = "23_ML_Pretarsus_L")
        pyrosim.Send_Sensor_Neuron(name = 23 , linkName = "24_ML_CT_R")
        pyrosim.Send_Sensor_Neuron(name = 24 , linkName = "25_ML_Femur_R")
        pyrosim.Send_Sensor_Neuron(name = 25 , linkName = "26_ML_Tibia_R")
        pyrosim.Send_Sensor_Neuron(name = 26 , linkName = "27_ML_Tarsus_R")
        pyrosim.Send_Sensor_Neuron(name = 27 , linkName = "28_ML_Pretarsus_R")
        pyrosim.Send_Sensor_Neuron(name = 28 , linkName = "29_FL_CT_L")
        pyrosim.Send_Sensor_Neuron(name = 29 , linkName = "30_FL_Femur_L")
        pyrosim.Send_Sensor_Neuron(name = 30 , linkName = "31_FL_Tibia_L")
        pyrosim.Send_Sensor_Neuron(name = 31 , linkName = "32_FL_Tarsus_L")
        pyrosim.Send_Sensor_Neuron(name = 32 , linkName = "33_FL_Pretarsus_L")
        pyrosim.Send_Sensor_Neuron(name = 33 , linkName = "34_FL_CT_R")
        pyrosim.Send_Sensor_Neuron(name = 34 , linkName = "35_FL_Femur_R")
        pyrosim.Send_Sensor_Neuron(name = 35 , linkName = "36_FL_Tibia_R")
        pyrosim.Send_Sensor_Neuron(name = 36 , linkName = "37_FL_Tarsus_R")
        pyrosim.Send_Sensor_Neuron(name = 37 , linkName = "38_FL_Pretarsus_R")

        pyrosim.Send_Motor_Neuron(name = 38, jointName = "02_Prothorax")
        pyrosim.Send_Motor_Neuron(name = 39, jointName = "03_Mesothorax")
        pyrosim.Send_Motor_Neuron(name = 40, jointName = "04_Metathorax")
        pyrosim.Send_Motor_Neuron(name = 41, jointName = "05_Abdomen_3")
        pyrosim.Send_Motor_Neuron(name = 42, jointName = "06_Abdomen_2")
        pyrosim.Send_Motor_Neuron(name = 43, jointName = "07_Abdomen_1")
        pyrosim.Send_Motor_Neuron(name = 44, jointName = "08_Abdomen_0")
        pyrosim.Send_Motor_Neuron(name = 45, jointName = "09_HL_CT_L")
        pyrosim.Send_Motor_Neuron(name = 46, jointName = "10_HL_Femur_L")
        pyrosim.Send_Motor_Neuron(name = 47, jointName = "11_HL_Tibia_L")
        pyrosim.Send_Motor_Neuron(name = 48, jointName = "12_HL_Tarsus_L")
        pyrosim.Send_Motor_Neuron(name = 49, jointName = "13_HL_Pretarsus_L")
        pyrosim.Send_Motor_Neuron(name = 50, jointName = "14_HL_CT_R")
        pyrosim.Send_Motor_Neuron(name = 51, jointName = "15_HL_Femur_R")
        pyrosim.Send_Motor_Neuron(name = 52, jointName = "16_HL_Tibia_R")
        pyrosim.Send_Motor_Neuron(name = 53, jointName = "17_HL_Tarsus_R")
        pyrosim.Send_Motor_Neuron(name = 54, jointName = "18_HL_Pretarsus_R")
        pyrosim.Send_Motor_Neuron(name = 55, jointName = "19_ML_CT_L")
        pyrosim.Send_Motor_Neuron(name = 56, jointName = "20_ML_Femur_L")
        pyrosim.Send_Motor_Neuron(name = 57, jointName = "21_ML_Tibia_L")
        pyrosim.Send_Motor_Neuron(name = 58, jointName = "22_ML_Tarsus_L")
        pyrosim.Send_Motor_Neuron(name = 59, jointName = "23_ML_Pretarsus_L")
        pyrosim.Send_Motor_Neuron(name = 60, jointName = "24_ML_CT_R")
        pyrosim.Send_Motor_Neuron(name = 61, jointName = "25_ML_Femur_R")
        pyrosim.Send_Motor_Neuron(name = 62, jointName = "26_ML_Tibia_R")
        pyrosim.Send_Motor_Neuron(name = 63, jointName = "27_ML_Tarsus_R")
        pyrosim.Send_Motor_Neuron(name = 64, jointName = "28_ML_Pretarsus_R")
        pyrosim.Send_Motor_Neuron(name = 65, jointName = "29_FL_CT_L")
        pyrosim.Send_Motor_Neuron(name = 66, jointName = "30_FL_Femur_L")
        pyrosim.Send_Motor_Neuron(name = 67, jointName = "31_FL_Tibia_L")
        pyrosim.Send_Motor_Neuron(name = 68, jointName = "32_FL_Tarsus_L")
        pyrosim.Send_Motor_Neuron(name = 69, jointName = "33_FL_Pretarsus_L")
        pyrosim.Send_Motor_Neuron(name = 70, jointName = "34_FL_CT_R")
        pyrosim.Send_Motor_Neuron(name = 71, jointName = "35_FL_Femur_R")
        pyrosim.Send_Motor_Neuron(name = 71, jointName = "36_FL_Tibia_R")
        pyrosim.Send_Motor_Neuron(name = 71, jointName = "37_FL_Tarsus_R")
        pyrosim.Send_Motor_Neuron(name = 72, jointName = "38_FL_Pretarsus_R")

        for currentRow in range(0, c.NUM_SENSOR_NEURONS):
            for currentColumn in range(0, c.NUM_MOTOR_NEURONS):
                pyrosim.Send_Synapse( sourceNeuronName = currentRow , targetNeuronName = currentColumn+c.NUM_SENSOR_NEURONS , weight = self.weights[currentRow][currentColumn] )

        pyrosim.End()