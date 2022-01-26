import pickle

parent = {}

with open("data/winningParent.pk1", 'rb') as inp:
    parent = pickle.load(inp)

parent.Start_Simulation("GUI")