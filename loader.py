import sys
import os
import pickle

genFileName = sys.argv[1]
generationFile = open(genFileName, "rb")
generation = pickle.load(generationFile)

os.system("del fitness*.txt")
os.system("del tmp*.txt")

if len(sys.argv) > 2:
    snd = sys.argv[2]
    if (snd == "print"):
        for bodyKey in generation:
            generation[bodyKey].Print()
    elif (snd == "best"):
        mostFit = generation[0]
        for key in generation:
            if (mostFit.fitness < generation[key].fitness):
                mostFit = generation[key]
        mostFit.Start_Simulation("GUI", False)
    else:
        generation[snd].Start_Simulation("GUI", False)
else:
    for bodyKey in generation:
        generation[bodyKey].Start_Simulation("GUI", False)
