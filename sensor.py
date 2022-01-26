import numpy
import constants as c
import pyrosim.pyrosim as pyrosim

class SENSOR:

    def __init__(self, linkname):
        '''Constructor of the SENSOR class'''

        self.linkName = linkname
        self.values = numpy.zeros(c.SIMULATION_STEPS)


    def Get_Value(self, t):
        '''Captures the sensor values of each link'''

        self.values[t] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)