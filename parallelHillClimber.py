from solution import SOLUTION

import constants as c
import copy
import os
import pickle

class PARALLEL_HILL_CLIMBER:

    def __init__(self):
        '''Constructor of PARALLEL_HILL_CLIMBER class'''

        os.system("rm fitness*.txt")
        os.system("rm body*.urdf")
        os.system("rm brain*.nndf")

        self.nextAvailableID = 0
        self.parent = {}

        for i in range(0, c.POPULATION_SIZE):
            self.parent.update({i: SOLUTION(self.nextAvailableID)})
            self.nextAvailableID += 1

    def Evolve(self):
        '''Defines the number of evolving generations and starts the evolving process'''

        self.Evaluate(self.parent)

        for currentGeneration in range(0, c.NUMBER_OF_GENERATIONS):
            self.Evolve_For_One_Generation(currentGeneration)
            currentGeneration += 1
    
    def Evolve_For_One_Generation(self, currentGeneration):
        '''Creates a child for a given parent, mutates it, and creates a simulation for it'''

        self.Spawn()
        self.Mutate()
        self.Evaluate(self.child)
        self.Select()
        self.Print(currentGeneration)

    def Spawn(self):
        '''Creates a childs as a deep copies from each parent'''

        self.child = {}
        for i in self.parent:
            self.child.update({i: copy.deepcopy(self.parent[i])})
            self.child[i].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1

    def Mutate(self):
        '''Calls the Mutation method for each of the childs'''

        for i in self.child:
            self.child[i].Mutate()

    def Evaluate(self ,solutions):
        '''Calss method to start the simulation for the given argument'''

        for i in solutions:
            solutions[i].Start_Simulation("DIRECT")

        for i in solutions:
            solutions[i].Wait_For_Simulation_To_End()

    def Select(self):
        '''Weighs up wether the childs solution was better or not'''

        for i in self.parent:
            if float(self.parent[i].fitness) <= float(self.child[i].fitness):
                self.parent[i] = copy.deepcopy(self.child[i])

    def Print(self, currentGeneration):
        '''Prints the fitness of the parents and the corresponding child'''
        
        print()
        print("Current generation: ", currentGeneration)
        for i in self.parent:
            print(i, ": Parent", "fitness: ", self.parent[i].fitness, ", Child" , "fitness: ", self.child[i].fitness)
        print("This was generation: ", currentGeneration)
        print()

    def Show_Best(self):
        '''Presents the final and best fitting solution'''

        self.fitness = {}

        for i in self.parent:
            self.fitness[i] = self.parent[i].fitness

        self.maxFitness = max(self.fitness.values())
        self.corespondingParent = [k for k, v in self.fitness.items() if v == self.maxFitness]
        self.parent[self.corespondingParent[0]].Start_Simulation("GUI")

        self.save_parent(self.parent[self.corespondingParent[0]], "data/winningParent.pk1")

        print("MAX_VALUE", self.maxFitness)
        print("MAX_KEY: ", self.corespondingParent)

# TODO 
# 
#   - implementing saving of urdf and nndf file of winnerParent
#   - at the beginning of the simulation:   copying the winnerParent from the last simulation

    def save_parent(self, obj, filename):
        with open(filename, 'wb') as outp:
            pickle.dump(obj, outp, pickle.HIGHEST_PROTOCOL)

    def get_last_parent(self, filename):
        with open(filename, 'rb') as inp:
            parent = pickle.load(inp)

        return parent  