import sys
import os
import pickle
import numpy as np
import matplotlib.pyplot as pyplot

genFileName = sys.argv[1]
generationFile = open(genFileName, "rb")
generation = pickle.load(generationFile)

os.system("del fitness*.txt")
os.system("del tmp*.txt")

if len(sys.argv) > 2:
    snd = sys.argv[2]
    if (snd == "print"):
        for bodyKey in generation:
            print(f"\nColumn: {bodyKey}")
            generation[bodyKey].Print()
    elif (snd == "best"):
        mostFit = generation[0]
        bestKey = 0
        for key in generation:
            if (mostFit.fitness < generation[key].fitness):
                mostFit = generation[key]
                bestKey = key
        mostFit.Start_Simulation("GUI", False)
        print(f"\nColumn: {bestKey}")
        mostFit.Print()
    elif (snd == "graph"):
        mostFit = generation[0]
        for key in generation:
            pyplot.plot(generation[key].history, marker='o')
            if (mostFit.fitness < generation[key].fitness):
                mostFit = generation[key]
        pyplot.plot(mostFit.history, label="Fittest", linewidth=4, marker='o')
        pyplot.legend()
        pyplot.xlabel("Generation")
        pyplot.ylabel("Distance from goal (fitness)")
        pyplot.show()
    else:
        generation[int(snd)].Start_Simulation("GUI", False)
        print(f"\nColumn: {int(snd)}")
        generation[int(snd)].Print()
else:
    for bodyKey in generation:
        generation[bodyKey].Start_Simulation("GUI", False)
