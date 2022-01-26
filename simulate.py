from simulation import SIMULATION

import sys


directGUI = str(sys.argv[1])
solutionID = str(sys.argv[2])

simulation = SIMULATION(directGUI, solutionID)

simulation.RUN()

simulation.Get_Fitness()