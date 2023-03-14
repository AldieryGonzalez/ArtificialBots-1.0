import pickle
from solution import SOLUTION
import matplotlib.pyplot as pyplot
import constants as c
import numpy as np
import copy
import os


class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        os.system("del *.nndf")
        os.system("del *.urdf")
        os.system("del fitness*.txt")
        os.system("del graph*.npy")
        self.nextAvailableID = 0
        self.parents = {}

        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1

    def Save_Generation(self, genname):
        generation = open(genname, 'wb')
        pickle.dump(self.parents, generation)
        generation.close()

    def Evolve(self):
        self.Evaluate(self.parents)
        self.Save_Generation("generation0")
        for currentGeneration in range(1, c.numberOfGenerations):
            if (currentGeneration % c.pickleEveryXGens == 0):
                self.Save_Generation(f'generation{currentGeneration}')
            self.Evolve_For_One_Generation()
        self.Save_Generation(f'generation{c.numberOfGenerations-1}')

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        self.Select()
        self.Print()
        # print(
        #     f"\n\nParent: {self.parent.fitness}, Child: {self.child.fitness}\n")

    def Spawn(self):
        self.children = {}
        for key in self.parents:
            self.children[key] = copy.deepcopy(self.parents[key])
            self.children[key].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1

    def Mutate(self):
        for child in self.children.values():
            child.Mutate()

    def Evaluate(self, solutions):
        for key in solutions:
            solutions[key].Start_Simulation("DIRECT")
        for key in solutions:
            solutions[key].Wait_For_Simulation_To_End()

    def Print(self):
        # print("\n")
        # for key in self.parents:
        #     print(
        #         f"Parent: {self.parents[key].fitness}, Child: {self.children[key].fitness}, Parent History: {self.parents[key].history}")
        # print("\n")
        pass

    def Select(self):
        for key in self.parents:
            if (self.parents[key].fitness < self.children[key].fitness):
                self.parents[key] = self.children[key]
            self.parents[key].history.append(self.parents[key].fitness)

    def Show_Initial(self):
        for key in self.parents:
            self.parents[key].Start_Simulation("GUI")

    def Show_Best(self):
        generation = open(
            f'generation{c.numberOfGenerations-1}', 'wb')
        pickle.dump(self.parents, generation)
        generation.close()
        mostFit = self.parents[0]
        for key in self.parents:
            np.save(f'graph{key}.npy', self.parents[key].history)
            if (mostFit.fitness < self.parents[key].fitness):
                mostFit = self.parents[key]
        print(
            f"\n\nMOST FIT: {mostFit}\nFITNESS: {mostFit.fitness}, {mostFit.history}\n")
        mostFit.Start_Simulation("GUI")
        self.analyze(mostFit)

    def analyze(self, fittest):
        pyplot.plot(fittest.history, label="Fittest", linewidth=4, marker='o')
        for key in self.parents:
            fittestData = np.load(f"graph{key}.npy")
            pyplot.plot(fittestData, marker='o')
        pyplot.legend()
        pyplot.xlabel("Generation")
        pyplot.ylabel("Distance from goal (fitness)")
        pyplot.show()
