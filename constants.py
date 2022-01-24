import numpy
import pybullet as p

# setting up the gravity
GRAVITY_X = 0
GRAVITY_Y = 0
GRAVITY_Z = -9.81

# steps of simulation
SIMULATION_STEPS = 2500

# Steps of evolutions to be carried out
NUMBER_OF_GENERATIONS = 2

# Number of parallel generated parents
POPULATION_SIZE = 100

# Number of sensor neurons
NUM_SENSOR_NEURONS = 37

# Number of motor neurons
NUM_MOTOR_NEURONS = 36

# time between simuation steps [s]
TIME_BETWEEN_STEPS = 1/240

# parameters for all motors
CONTROL_MODE = p.POSITION_CONTROL
MAX_FORCE = 1500
MOTOR_JOINT_RANGE = 0.2

# Mass of the robot
ROBOT_MASS = 100

# Coefficient for friction of objects within the simulation
LATERAL_FRICTION_COEFFICIENT = 10000