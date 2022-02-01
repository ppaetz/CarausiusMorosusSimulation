import numpy
import pybullet as p

# setting up the gravity
GRAVITY_X = 0
GRAVITY_Y = 0
GRAVITY_Z = -981

# steps of simulation
SIMULATION_STEPS = 2500

# Steps of evolutions to be carried out
NUMBER_OF_GENERATIONS = 2

# Number of parallel generated parents
POPULATION_SIZE = 8

# Number of sensor neurons
NUM_SENSOR_NEURONS = 37

# Number of motor neurons
NUM_MOTOR_NEURONS = 36

# time between simuation steps [s]
TIME_BETWEEN_STEPS = 1/240

# parameters for all motors
CONTROL_MODE = p.POSITION_CONTROL
MAX_FORCE = 500000000

MOTOR_JOINT_RANGE = 0.2

# Coefficient for friction of objects within the simulation
LATERAL_FRICTION_COEFFICIENT = 100000

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
