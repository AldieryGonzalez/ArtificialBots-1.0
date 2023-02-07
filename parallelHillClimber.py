from solution import SOLUTION
import constants as c
import copy
import os


class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        os.system("del brain*.nndf")
        os.system("del fitness*.txt")
        self.nextAvailableID = 0
        self.parents = {}
        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1

    def Evolve(self):
        self.Evaluate(self.parents)
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation()

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
        print("\n")
        for key in self.parents:
            print(
                f"Parent: {self.parents[key].fitness}, Child: {self.children[key].fitness} ")
        print("\n")

    def Select(self):
        for key in self.parents:
            if (self.parents[key].fitness > self.children[key].fitness):
                self.parents[key] = self.children[key]

    def Show_Best(self):
        mostFit = self.parents[0]
        for key in self.parents:
            if (mostFit.fitness > self.parents[key].fitness):
                mostFit = self.parents[key]
        print(f"\n\nMOST FIT: {mostFit}\nFITNESS: {mostFit.fitness}\n")
        mostFit.Start_Simulation("GUI")
