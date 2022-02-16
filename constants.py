import numpy
import pybullet as p

# setting up the gravity
GRAVITY_X = 0
GRAVITY_Y = 0
GRAVITY_Z = -98.1

# steps of simulation
SIMULATION_STEPS = 2500

# Steps of evolutions to be carried out
NUMBER_OF_GENERATIONS = 4

# Number of parallel generated parents
POPULATION_SIZE = 15

# Number of sensor neurons
NUM_SENSOR_NEURONS = 37

# Number of motor neurons
NUM_MOTOR_NEURONS = 36

# time between simuation steps [s]
TIME_BETWEEN_STEPS = 1/240

# parameters for all motors
CONTROL_MODE = p.POSITION_CONTROL
MAX_FORCE = 50000
MOTOR_JOINT_RANGE = 0.2

# Coefficient for friction of objects within the simulation
LATERAL_FRICTION_COEFFICIENT = 10000

# Limits of the Joints
UPPER_LIMIT_CARPUT_PROTHORAX = 0
LOWER_LIMIT_CARPUT_PROTHORAX = 0
UPPER_LIMIT_PROTHORAX_MESOTHORAX = 0
LOWER_LIMIT_PROTHORAX_MESOTHORAX = 0
UPPER_LIMIT_MESOTHORAX_METATHORAX = 0
LOWER_LIMIT_MESOTHORAX_METATHORAX = 0
UPPER_LIMIT_METATHORAX_ABDOMEN_3 = 0
LOWER_LIMIT_METATHORAX_ABDOMEN_3 = 0
UPPER_LIMIT_ABDOMEN_3_ABDOMEN_2 = 0
LOWER_LIMIT_ABDOMEN_3_ABDOMEN_2 = 0
UPPER_LIMIT_ABDOMEN_2_ABDOMEN_1= 0
LOWER_LIMIT_ABDOMEN_2_ABDOMEN_1 = 0
UPPER_LIMIT_ABDOMEN_1_ABDOMEN_0 = 0
LOWER_LIMIT_ABDOMEN_1_ABDOMEN_0 = 0
UPPER_LIMIT_FL_CT_L = 2
LOWER_LIMIT_FL_CT_L = -2
UPPER_LIMIT_FL_CT_R = 2
LOWER_LIMIT_FL_CT_R = -2
UPPER_LIMIT_ML_CT_L = 2
LOWER_LIMIT_ML_CT_L = -2
UPPER_LIMIT_ML_CT_R = 2
LOWER_LIMIT_ML_CT_R = -2
UPPER_LIMIT_HL_CT_L = 2
LOWER_LIMIT_HL_CT_L = -2
UPPER_LIMIT_HL_CT_R = 2
LOWER_LIMIT_HL_CT_R = -2
UPPER_LIMIT_FL_FEMUR_L = 2
LOWER_LIMIT_FL_FEMUR_L = -2
UPPER_LIMIT_FL_FEMUR_R = 2
LOWER_LIMIT_FL_FEMUR_R = -2
UPPER_LIMIT_ML_FEMUR_L = 2
LOWER_LIMIT_ML_FEMUR_L = -2
UPPER_LIMIT_ML_FEMUR_R = 2
LOWER_LIMIT_ML_FEMUR_R = -2
UPPER_LIMIT_HL_FEMUR_L = 2
LOWER_LIMIT_HL_FEMUR_L = -2
UPPER_LIMIT_HL_FEMUR_R = 2
LOWER_LIMIT_HL_FEMUR_R = -2
UPPER_LIMIT_FL_TIBIA_L = 2
LOWER_LIMIT_FL_TIBIA_L = -2
UPPER_LIMIT_FL_TIBIA_R = 2
LOWER_LIMIT_FL_TIBIA_R = -2
UPPER_LIMIT_ML_TIBIA_L = 2
LOWER_LIMIT_ML_TIBIA_L = -2
UPPER_LIMIT_ML_TIBIA_R = 2
LOWER_LIMIT_ML_TIBIA_R = -2
UPPER_LIMIT_HL_TIBIA_L = 2
LOWER_LIMIT_HL_TIBIA_L = -2
UPPER_LIMIT_HL_TIBIA_R = 2
LOWER_LIMIT_HL_TIBIA_R = -2
UPPER_LIMIT_FL_PRETARSUS_L = 2
LOWER_LIMIT_FL_PRETARSUS_L = -2
UPPER_LIMIT_FL_PRETARSUS_R = 2
LOWER_LIMIT_FL_PRETARSUS_R = -2
UPPER_LIMIT_ML_PRETARSUS_L = 2
LOWER_LIMIT_ML_PRETARSUS_L = -2
UPPER_LIMIT_ML_PRETARSUS_R = 2
LOWER_LIMIT_ML_PRETARSUS_R = -2
UPPER_LIMIT_HL_PRETARSUS_L = 2
LOWER_LIMIT_HL_PRETARSUS_L = -2
UPPER_LIMIT_HL_PRETARSUS_R = 2
LOWER_LIMIT_HL_PRETARSUS_R = -2
UPPER_LIMIT_FL_TARSUS_L = 2
LOWER_LIMIT_FL_TARSUS_L = -2
UPPER_LIMIT_FL_TARSUS_R = 2
LOWER_LIMIT_FL_TARSUS_R = -2
UPPER_LIMIT_ML_TARSUS_L = 2
LOWER_LIMIT_ML_TARSUS_L = -2
UPPER_LIMIT_ML_TARSUS_R = 2
LOWER_LIMIT_ML_TARSUS_R = -2
UPPER_LIMIT_HL_TARSUS_L = 2
LOWER_LIMIT_HL_TARSUS_L = -2
UPPER_LIMIT_HL_TARSUS_R = 2
LOWER_LIMIT_HL_TARSUS_R = -2
